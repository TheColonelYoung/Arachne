/**
 * @file motor_config.hpp
 * @author Petr Malaník (TheColonelYoung(at)gmail(dot)com)
 * @brief
 * @version 0.1
 * @date 19.05.2021
 */

#include <array>

#include "construction/motion_axis.hpp"

namespace Eyrina_config {
struct motor_config {
    float        supply_voltage    = 36;
    float        target_current    = 1.0;
    float        coil_impedance    = 4.0;
    float        coil_inductance   = 4.0;
    float        electric_constant = 0.05;

    unsigned int microsteps   = 128;
    unsigned int acceleration = 100;
    unsigned int deceleration = 100;

    unsigned int min_speed = 20;
    unsigned int max_speed = 100;

    unsigned int overcurrent   = 1600;
    unsigned int stall_current = 1200;

    unsigned int fullstep_optimization  = 0;
    bool         low_speed_optimization = false;
};

extern const std::array<motor_config, 8> motor_configurations;

struct axis_config {
    double                  fullstep_movement;
    char                   axis_name;
    Motion_axis::Direction homing_direction = Motion_axis::Direction::Forward;
    double                    min_position = 0;
    double                    max_position = 0;
    double                    homing_speed = 0;
};

extern const std::array<axis_config, 8> axis_configurations;
};
