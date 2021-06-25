#include "motor_config.hpp"

namespace Eyrina_config {
const std::array<axis_config, 8> axis_configurations{ {
    { .fullstep_movement = 0.005,
      .axis_name         = 'H',
      .homing_direction  = Motion_axis::Direction::Forward,
      .min_position      = 0, // - 0.5
      .max_position      = 0.5,
      .homing_speed      = 0.5 }
} };

const std::array<motor_config, 8> motor_configurations
{ {
    // Motor_8 - H
    {
        .supply_voltage         = 24.0,
        .target_current         = 1.0,
        .coil_impedance         = 4.2,
        .coil_inductance        = 7.2,
        .electric_constant      = 0.068,
        .microsteps             = 128,
        .acceleration           = 1000,
        .deceleration           = 2000,
        .min_speed              = 100,
        .max_speed              = 1200,
        .overcurrent            = 2400,
        .stall_current          = 800,
        .fullstep_optimization  = 400,
        .low_speed_optimization = true
    }
} };
};
