#ifndef STATUS_TYPES_HPP
#define STATUS_TYPES_HPP

#include <cstdint>

struct VehicleStatus_s {
    uint8_t arm_state;
    uint8_t nav_state;
    uint16_t failure_detector_status;
};

struct SystemStatus_s {
    uint8_t system_status;
    uint8_t health_flags;
    uint8_t arming_state;
    uint8_t prev_arming_state;

    static constexpr int NAV_IDLE = 0;
    static constexpr int NAV_RTH = 1;
    static constexpr int NAV_TAKEOFF = 2;
    static constexpr int NAV_LAND = 3;
    static constexpr int NAV_AUTO_MISSION = 4;
    static constexpr int NAV_SEARCH = 5; 
    static constexpr int NAV_PRECISION_LAND = 6;
    static constexpr int NAV_HOLD = 7;
    static constexpr int NAV_POSITION = 8;
    static constexpr int NAV_FAILSAFE = 9;

    uint8_t nav_mode = NAV_IDLE;
};

#endif  // STATUS_TYPES_HPP