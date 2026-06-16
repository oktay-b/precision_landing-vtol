#include "commander.hpp"

Commander::Commander(const Odometry_s &odometry, const ArucoTypes_s &aruco,
                     Odometry_s &setpoint, SystemStatus_s &system_status) : odometry_(odometry),
                                                                            aruco_(aruco),
                                                                            setpoint_(setpoint),
                                                                            system_status_(system_status)
{
}

void Commander::start()
{
    nav_started_ = true;
}

void Commander::update()
{
    static int prev_nav_mode = system_status_.nav_mode;

    if (system_status_.nav_mode != prev_nav_mode)
    {
        start();
        prev_nav_mode = system_status_.nav_mode;
    }

    if(nav_started_ == false)
    {
        return;
    }

    switch (system_status_.nav_mode)
    {
    case SystemStatus_s::NAV_RTH:
        break;
    case SystemStatus_s::NAV_TAKEOFF:
        setTakeoff();
        break;
    case SystemStatus_s::NAV_LAND:
        stop();
        break;
    case SystemStatus_s::NAV_AUTO_MISSION:
        stop();
        /* code */
        break;
    case SystemStatus_s::NAV_SEARCH:
        stop();
        /* code */
        break;
    case SystemStatus_s::NAV_PRECISION_LAND:
        /* code */
        setPrecisionLand();
        break;
    case SystemStatus_s::NAV_HOLD:
        stop();
        /* code */
        break;
    case SystemStatus_s::NAV_POSITION:
        stop();
        break;
    case SystemStatus_s::NAV_FAILSAFE:
        /* code */
        setFailSafe();
        break;

    default:
        break;
    }
}

void Commander::stop()
{
    nav_started_ = false;
}

void Commander::setArm()
{
    system_status_.arming_state = 1;
    setpoint_.x = odometry_.x;
    setpoint_.y = odometry_.y;
    setpoint_.z = odometry_.z;
    setpoint_.euler[0] = odometry_.euler[0];
    RCLCPP_INFO(rclcpp::get_logger("Commander"), "Armed");
}

void Commander::setDisarm()
{
    system_status_.arming_state = 0;
    setpoint_.x = odometry_.x;
    setpoint_.y = odometry_.y;
    setpoint_.z = odometry_.z;
    setpoint_.euler[0] = odometry_.euler[0];
    RCLCPP_INFO(rclcpp::get_logger("Commander"), "Disarmed");
}

void Commander::setTakeoff()
{
    if (system_status_.arming_state == 0)
    {
        return;
    }

    setpoint_.z = odometry_.z + takeoff_height_;
    system_status_.nav_mode = SystemStatus_s::NAV_TAKEOFF;
    RCLCPP_INFO(rclcpp::get_logger("Commander"), "Takeoff");
    stop();
}

void Commander::setPrecisionLand()
{
    static int aruco_lost_counter = 0;
    const int max_lost_count = 50;
    const double max_integral = 5.0; 
    const double min_dt = 0.01;     

    if (aruco_.is_aruco_detected && aruco_.percent < 22.0)
    {
        aruco_lost_counter = 0;

        rclcpp::Time now = rclcpp::Clock().now();
        double dt = (now - prev_time_).seconds();
        
        if (prev_time_.nanoseconds() == 0) {
            prev_time_ = now;
            return; 
        }
        
        dt = std::max(dt, min_dt); 

        double error_x = aruco_.x_error;
        double error_y = aruco_.y_error;

        integral_x = std::clamp(integral_x + error_x * dt, -max_integral, max_integral);
        integral_y = std::clamp(integral_y + error_y * dt, -max_integral, max_integral);

        double derivative_x = (error_x - prev_error_x) / dt;
        double derivative_y = (error_y - prev_error_y) / dt;

        double pid_x = p_gain_x * error_x + i_gain_x * integral_x + d_gain_x * derivative_x;
        double pid_y = p_gain_y * error_y + i_gain_y * integral_y + d_gain_y * derivative_y;

        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_time_ = now;

        pid_x = std::clamp(pid_x, -0.5, 0.5);
        pid_y = std::clamp(pid_y, -0.5, 0.5);

        updateSetpointPosition(pid_x, pid_y, 0.0);
        setpoint_.z = odometry_.z - 0.01;

        RCLCPP_INFO(rclcpp::get_logger("Commander"), 
                   "PID Output: X: %.3f Y: %.3f (ErrorX: %.2f, ErrorY: %.2f) dt: %.2f", 
                   pid_x, pid_y, error_x, error_y, dt);
    }
    else
    {
        aruco_lost_counter++;
        
        if (aruco_lost_counter > max_lost_count)
        {
            integral_x = 0;
            integral_y = 0;
            prev_error_x = 0;
            prev_error_y = 0;
            
            RCLCPP_WARN(rclcpp::get_logger("Commander"), 
                       "Aruco marker lost for too long. Switching to NAV_LAND.");
            system_status_.nav_mode = SystemStatus_s::NAV_LAND;
            RCLCPP_INFO(rclcpp::get_logger("Commander"), "Land Started");
            stop();
            aruco_lost_counter = 0;
        }
    }
}


void Commander::setFailSafe()
{
    system_status_.health_flags = 1;
    system_status_.arming_state = 0;
}

void Commander::updateLandVehicleVelocity(int data, Odometry_s &velocity)
{
    switch (data)
    {
    case 49:
        velocity.vx = -1.0;
        velocity.vz = 1.0;
        break;
    case 50:
        velocity.vx = -1.0;
        velocity.vz = 0.0;
        break;
    case 51:
        velocity.vx = -1.0;
        velocity.vz = -1.0;
        break;
    case 52:
        velocity.vx = 0.0;
        velocity.vz = 1.0;
        break;
    case 54:
        velocity.vx = 0.0;
        velocity.vz = -1.0;
        break;
    case 55:
        velocity.vx = 1.0;
        velocity.vz = 1.0;
        break;
    case 56:
        velocity.vx = 1.0;
        velocity.vz = 0.0;
        break;
    case 57:
        velocity.vx = 1.0;
        velocity.vz = -1.0;
        break;
    default:
        velocity.vx = 0.0;
        velocity.vz = 0.0;
        break;
    }
}

void Commander::setOffboardControlMode(int mode)
{
    switch (mode)
    {
    case 65:
        setArm();
        break;
    case 68:
        setDisarm();
        break;
    case 75:
        system_status_.nav_mode = SystemStatus_s::NAV_FAILSAFE;
        break;
    case 76:
        system_status_.nav_mode = SystemStatus_s::NAV_LAND;
        break;
    case 80:
        system_status_.nav_mode = SystemStatus_s::NAV_PRECISION_LAND;
        break;
    case 84:
        system_status_.nav_mode = SystemStatus_s::NAV_TAKEOFF;
        break;
    case 43:
        updateSetpointPosition(0.0, 0.0, -0.5, 0.0); 
        system_status_.nav_mode = SystemStatus_s::NAV_POSITION;
        break;
    case 45:
        updateSetpointPosition(0.0, 0.0, 0.5, 0.0); 
        system_status_.nav_mode = SystemStatus_s::NAV_POSITION;
        break;
    case 214:
        updateSetpointPosition(0.0, 0.0, 0.0, 0.05); 
        system_status_.nav_mode = SystemStatus_s::NAV_POSITION;
        break;
    case 199:
        updateSetpointPosition(0.0, 0.0, 0.0, -0.05); 
        system_status_.nav_mode = SystemStatus_s::NAV_POSITION;
        break;
    case 16777235:
        updateSetpointPosition(0.0, -0.5, 0.0); 
        system_status_.nav_mode = SystemStatus_s::NAV_POSITION;
        break;
    case 16777236:
        updateSetpointPosition(0.5, 0.0, 0.0); 
        system_status_.nav_mode = SystemStatus_s::NAV_POSITION;
        break;
    case 16777237:
        updateSetpointPosition(0.0, 0.5, 0.0); 
        system_status_.nav_mode = SystemStatus_s::NAV_POSITION;
        break;
    case 16777234:
        updateSetpointPosition(-0.5, 0.0, 0.0); 
        system_status_.nav_mode = SystemStatus_s::NAV_POSITION;
        break;
    default:
        break;
    }
}

void Commander::updateSetpointPosition(double x_offset, double y_offset, double z_offset, double yaw_offset)
{
    double cos_yaw = cos(odometry_.enu_yaw);
    double sin_yaw = sin(odometry_.enu_yaw);

    double x_error = (cos_yaw * x_offset - sin_yaw * y_offset);
    double y_error = (sin_yaw * x_offset + cos_yaw * y_offset);

    if (abs(x_error) > OFSET)
    {
        setpoint_.x = odometry_.x + x_error;
    }
    if (abs(y_error) > OFSET)
    {
        setpoint_.y = odometry_.y + y_error;
    }
    if (abs(z_offset) > OFSET)
    {
        setpoint_.z = odometry_.z + z_offset;
    }
    if (abs(yaw_offset) > OFSET_YAW)
    {
        if (abs(odometry_.euler[0] - setpoint_.euler[0]) > 0.2)
        {
            return;
        }

        setpoint_.euler[0] = normalizeAngle(setpoint_.euler[0] + yaw_offset);
        // RCLCPP_INFO(rclcpp::get_logger("Commander"), "Yaw setpoint_.euler: %lf | odometry_.euler: %lf",setpoint_.euler[0], odometry_.euler[0]);
    }
}

// sudo sync; sudo bash -c 'echo 3 > /proc/sys/vm/drop_caches
