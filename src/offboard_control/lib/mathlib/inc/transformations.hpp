#ifndef TRANSFORMATIONS_HPP
#define TRANSFORMATIONS_HPP

#include <cmath>
#include <cstdint>
#include <array>
#include <vector>
#include <string>
#include <iostream>

inline void quaternionToEuler(double q0, double q1, double q2, double q3, double& roll, double& pitch, double& yaw)
{
    double sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp); 
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

inline void quaternionToEuler(const double quaternion[4], double euler[3])
{
    double q0 = quaternion[0];
    double q1 = quaternion[1];
    double q2 = quaternion[2];
    double q3 = quaternion[3];

    double sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);


    double sinp = 2.0 * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1.0)
        euler[1] = std::copysign(M_PI / 2.0, sinp); 
    else
        euler[1] = std::asin(sinp);

    double siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
}

inline double normalizeAngle(double angle) {
    // Use fmod to wrap angle between [-PI, PI]
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0)
        angle += 2.0 * M_PI;
    return angle - M_PI;
}

// Convert yaw from NED (North-East-Down) to ENU (East-North-Up) frame.
// In NED, yaw=0 means pointing North; in ENU, yaw=0 means pointing East.
// Formula: yaw_enu = M_PI/2 - yaw_ned.
inline double yawNedToEnu(double yaw_ned) {
    double yaw_enu = M_PI_2 - yaw_ned;
    // Normalize to [-M_PI, M_PI]
    // return normalizeAngle(yaw_enu);
    return yaw_enu;
}

// Convert yaw from ENU (East-North-Up) to NED (North-East-Down) frame.
// Formula: yaw_ned = M_PI/2 - yaw_enu.
inline double yawEnuToNed(double yaw_enu) {
    double yaw_ned = M_PI_2 - yaw_enu;
    // Normalize to [-M_PI, M_PI]
    // return normalizeAngle(yaw_ned);
    return yaw_ned;
}

#endif
