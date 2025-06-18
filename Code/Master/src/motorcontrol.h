#include "Arduino.h"

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

enum MotorDirection_t {
    MOTOR_CW, // Clockwise
    MOTOR_CCW, // Counter-clockwise
    MOTOR_SHORTEST // Shortest path
};

struct MotorControl_t {
    uint16_t position = 0; // Position in steps
    uint8_t acceleration = 50; // Acceleration in steps per second squared
    uint16_t speed = 150; // Speed in steps per second
    MotorDirection_t direction = MotorDirection_t::MOTOR_SHORTEST; // Direction of rotation
    bool keepRunning = false; // Whether the motor should keep running after reaching the target position
    uint16_t time = UINT16_MAX; // Time in milliseconds to reach the target position
};

#endif // MOTORCONTROL_H