#include "offboard_control_node.hpp"

OffboardControlNode::OffboardControlNode(): Node("offboard_control_node"),
    _commander(std::make_shared<Commander>(odometry_, aruco_, setpoint_, system_status_))
{
    init();
    timers_.vehicle_command_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OffboardControlNode::commandCallback, this));
    RCLCPP_INFO(this->get_logger(), "Offboard Control Node Created");
    _commander->setTakeoffHeight(-5.0);
}

void OffboardControlNode::arm()
{
    vehicleCommandPublish(vehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
}

void OffboardControlNode::disarm()
{
    vehicleCommandPublish(vehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0);
}

void OffboardControlNode::killSwitch()
{
    vehicleCommandPublish(vehicleCommandMsg::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0);
    RCLCPP_WARN(this->get_logger(), "Vehicle Killed");
}

void OffboardControlNode::commandCallback()
{
    if(system_status_.health_flags == 1)
    {
        killSwitch();
        return;
    }

    if (system_status_.prev_arming_state != system_status_.arming_state)
    {
        vehicleCommandPublish(vehicleCommandMsg::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        if(system_status_.arming_state == 1)
        {
            arm();
        }
        else
        {
            disarm();
        }
        system_status_.prev_arming_state = system_status_.arming_state;
    }

    if(system_status_.arming_state == 0)
    {
        return;
    }

    _commander->update();
    offboardControlModePublish();

    if(system_status_.nav_mode == SystemStatus_s::NAV_LAND)
    {
        vehicleCommandPublish(vehicleCommandMsg::VEHICLE_CMD_NAV_LAND);
    }
    else if(system_status_.nav_mode != SystemStatus_s::NAV_IDLE){
        trajectorySetpointPublish();
    }
    
}

void OffboardControlNode::vehicleStatusCallback(const vehicleStatusMsgPtr msg)
{
    vehicle_status_.arm_state = msg->arming_state;
    vehicle_status_.nav_state = msg->nav_state;
    vehicle_status_.failure_detector_status = msg->failure_detector_status;
}

void OffboardControlNode::vehicleOdometryCallback(const vehicleOdometryMsgPtr msg)
{
    odometry_.x = msg->position[0];
    odometry_.y = msg->position[1];
    odometry_.z = msg->position[2];
    odometry_.q[0] = msg->q[3];
    odometry_.q[1] = msg->q[0];
    odometry_.q[2] = msg->q[1];
    odometry_.q[3] = msg->q[2];
    quaternionToEuler(odometry_.q, odometry_.euler);
    odometry_.enu_yaw = yawNedToEnu(odometry_.euler[0]);
    odometry_.vx = msg->velocity[0];
    odometry_.vy = msg->velocity[1];
    odometry_.vz = msg->velocity[2];
}

void OffboardControlNode::failsafeFlagsCallback(const failsafeFlagsMsgPtr msg)
{
    (void)msg;
}

void OffboardControlNode::keyPressCallback(const int32MsgPtr msg)
{
    if(msg->data >= 49 && msg->data <= 57)
    {
        _commander->updateLandVehicleVelocity(msg->data, land_vehcile_setpoint_);
        landVehicleVelocityPublish();
    }
    else {
        _commander->setOffboardControlMode(msg->data);
    }
}

void OffboardControlNode::arucoPoseCallback(const twistMsgPtr msg)
{
    aruco_.x_error = -msg->linear.x;
    aruco_.y_error = -msg->linear.y;
    aruco_.percent = msg->linear.z;
    aruco_.is_aruco_detected = msg->angular.x;
    aruco_.yaw_error = msg->angular.z;
    // RCLCPP_INFO(this->get_logger(), "Aruco Pose: %f %f %f", msg->linear.x, msg->linear.y, msg->linear.z);
    // RCLCPP_INFO(this->get_logger(), "Aruco Rota: %f %f %f", msg->angular.x, msg->angular.y, msg->angular.z);
}


void OffboardControlNode::init()
{
    initPublishers();
    initSubscribers();
    declerationParameters();
    RCLCPP_INFO(this->get_logger(), "Offboard Control Node Initialized");
}

void OffboardControlNode::initPublishers()
{
    publishers_.offboard_control_mode = this->create_publisher<offboardControlModeMsg>("/fmu/in/offboard_control_mode", 10);
    publishers_.trajectory_setpoint = this->create_publisher<trajectorySetpointMsg>("/fmu/in/trajectory_setpoint", 10);
    publishers_.vehicle_command = this->create_publisher<vehicleCommandMsg>("/fmu/in/vehicle_command", 10);
    publishers_.cmd_vel = this->create_publisher<twistMsg>("/cmd_vel", 10);
}

void OffboardControlNode::initSubscribers()
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    subscribers_.vehcile_odometry = this->create_subscription<vehicleOdometryMsg>("/fmu/out/vehicle_odometry", qos, std::bind(&OffboardControlNode::vehicleOdometryCallback, this, std::placeholders::_1));
    subscribers_.failsafe_flags = this->create_subscription<failsafeFlagsMsg>("/fmu/out/failsafe_flags", qos, std::bind(&OffboardControlNode::failsafeFlagsCallback, this, std::placeholders::_1));
    subscribers_.vehicle_status = this->create_subscription<vehicleStatusMsg>("/fmu/out/vehicle_status", qos, std::bind(&OffboardControlNode::vehicleStatusCallback, this, std::placeholders::_1));
    subscribers_.keypress = this->create_subscription<int32Msg>("/keyboard/keypress", qos, std::bind(&OffboardControlNode::keyPressCallback, this, std::placeholders::_1));
    subscribers_.aruco_pose = this->create_subscription<twistMsg>("/aruco_pose", 10, std::bind(&OffboardControlNode::arucoPoseCallback, this, std::placeholders::_1));
}

void OffboardControlNode::declerationParameters()
{
    this->declare_parameter<double>("p", 1.0);
    this->declare_parameter<double>("i", 0.01);
    this->declare_parameter<double>("d", 1.0);

    pid_.p = this->get_parameter("p").as_double();
    pid_.i = this->get_parameter("i").as_double();
    pid_.d = this->get_parameter("d").as_double(); 
}

void OffboardControlNode::offboardControlModePublish()
{
    offboardControlModeMsg msg;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.direct_actuator = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    publishers_.offboard_control_mode->publish(msg);
}

void OffboardControlNode::trajectorySetpointPublish()
{
    trajectorySetpointMsg msg;
    msg.position[0] = setpoint_.x;
    msg.position[1] = setpoint_.y;
    msg.position[2] = setpoint_.z;
    msg.yaw = yawEnuToNed(setpoint_.euler[0] - M_PI_2);
    // msg.velocity[0] = setpoint_.vx;
    // msg.velocity[1] = setpoint_.vy;
    // msg.velocity[2] = setpoint_.vz;

    publishers_.trajectory_setpoint->publish(msg);
}

void OffboardControlNode::vehicleCommandPublish(uint16_t command, float param1, float param2)
{
    vehicleCommandMsg msg;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;

    publishers_.vehicle_command->publish(msg);
}

void OffboardControlNode::landVehicleVelocityPublish()
{
    twistMsg msg;
    msg.linear.x = land_vehcile_setpoint_.vx;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = land_vehcile_setpoint_.vz;

    publishers_.cmd_vel->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlNode>());
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}