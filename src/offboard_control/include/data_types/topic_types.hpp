#ifndef TOPIC_TYPES_HPP
#define TOPIC_TYPES_HPP

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Messages sent on PX4 (Mission Computers -> PX4)
using offboardControlModeMsg = px4_msgs::msg::OffboardControlMode;
using offboardControlModeMsgPtr = std::shared_ptr<offboardControlModeMsg>;
using trajectorySetpointMsg = px4_msgs::msg::TrajectorySetpoint;
using trajectorySetpointMsgPtr = std::shared_ptr<trajectorySetpointMsg>;
using vehicleCommandMsg = px4_msgs::msg::VehicleCommand;
using vehicleCommandMsgPtr = std::shared_ptr<vehicleCommandMsg>;

// Messages sent on Mission Computers (PX4 -> Mission Computers)
using vehicleStatusMsg = px4_msgs::msg::VehicleStatus;
using vehicleStatusMsgPtr = std::shared_ptr<vehicleStatusMsg>;
using vehicleOdometryMsg = px4_msgs::msg::VehicleOdometry;
using vehicleOdometryMsgPtr = std::shared_ptr<vehicleOdometryMsg>;
using failsafeFlagsMsg = px4_msgs::msg::FailsafeFlags;
using failsafeFlagsMsgPtr = std::shared_ptr<failsafeFlagsMsg>;

using int32Msg = std_msgs::msg::Int32;
using int32MsgPtr = std::shared_ptr<int32Msg>;
using twistMsg = geometry_msgs::msg::Twist;
using twistMsgPtr = std::shared_ptr<twistMsg>;

struct Publishers
{
    rclcpp::Publisher<offboardControlModeMsg>::SharedPtr offboard_control_mode;
    rclcpp::Publisher<trajectorySetpointMsg>::SharedPtr trajectory_setpoint;
    rclcpp::Publisher<vehicleCommandMsg>::SharedPtr vehicle_command;
    rclcpp::Publisher<twistMsg>::SharedPtr cmd_vel;
};

struct Subscribers
{
    rclcpp::Subscription<vehicleOdometryMsg>::SharedPtr vehcile_odometry;
    rclcpp::Subscription<failsafeFlagsMsg>::SharedPtr failsafe_flags;
    rclcpp::Subscription<vehicleStatusMsg>::SharedPtr vehicle_status;
    rclcpp::Subscription<int32Msg>::SharedPtr keypress;
    rclcpp::Subscription<twistMsg>::SharedPtr aruco_pose;
};

struct Timers
{
    rclcpp::TimerBase::SharedPtr vehicle_command_timer;
};


#endif  // TOPIC_TYPES_HPP