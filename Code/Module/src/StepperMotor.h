#include "Arduino.h"
#include "../../Master/src/motorcontrol.h"

class StepperMotor
{
private:
    // Pin assignments
    int pin1A, pin1B, pin2A, pin2B;

    // Motor state
    uint16_t currentPosition; // in microsteps
    float currentSpeed = 0;   // degrees per second
    unsigned long nextStepTime;
    bool isRunning = false;

    uint16_t targetSpeed = 0; // in degrees per second

    uint32_t acceleration = 50; // Acceleration in degrees per second per second

    uint16_t targetPosition; // in microsteps

    unsigned long lastUpdateTime = micros();

    // Half-step sequence for 4-wire stepper motor
    // Using half-stepping for smoother operation
    const uint8_t stepSequence[8][4] = {
        {1, 0, 0, 0}, // Step 0
        {1, 1, 0, 0}, // Step 1
        {0, 1, 0, 0}, // Step 2
        {0, 1, 1, 0}, // Step 3
        {0, 0, 1, 0}, // Step 4
        {0, 0, 1, 1}, // Step 5
        {0, 0, 0, 1}, // Step 6
        {1, 0, 0, 1}  // Step 7
    };

    // Helper functions
    void writeStep(uint16_t microStep)
    {
        uint8_t step = microStep % (4 * MICROSTEPS);
        digitalWrite(pin1A, stepSequence[step][0]);
        digitalWrite(pin1B, stepSequence[step][1]);
        digitalWrite(pin2A, stepSequence[step][2]);
        digitalWrite(pin2B, stepSequence[step][3]);
    }

public:
    static const uint8_t MICROSTEPS = 2;
    static const uint16_t STEPS_PER_REVOLUTION = 720;                                     // Logical steps (user-facing)
    static const uint16_t MICRO_STEPS_PER_REVOLUTION = STEPS_PER_REVOLUTION * MICROSTEPS; // Physical microsteps
    static const uint16_t MICRO_STEPS_PER_DEGREE = MICRO_STEPS_PER_REVOLUTION / 360;      // Physical microsteps

    StepperMotor(int pin1A, int pin1B, int pin2A, int pin2B)
        : pin1A(pin1A), pin1B(pin1B), pin2A(pin2A), pin2B(pin2B),
          currentPosition(0), isRunning(false)
    {
        // Initialize pins
        pinMode(pin1A, OUTPUT);
        pinMode(pin1B, OUTPUT);
        pinMode(pin2A, OUTPUT);
        pinMode(pin2B, OUTPUT);

        // Set initial position
        writeStep(currentPosition);
    }

    void applyMotorControl(const MotorControl_t &control)
    {
        if (control.keepRunning == false)
        {
            targetPosition = control.position * MICRO_STEPS_PER_DEGREE;
            targetSpeed = control.speed;
            acceleration = control.acceleration;
            currentSpeed = 1;
            nextStepTime = micros();
            isRunning = true;

            if (control.time != UINT16_MAX)
            {
                float seconds = control.time / 1000.0;
                acceleration = (4 * abs((targetPosition / MICRO_STEPS_PER_DEGREE) - (currentPosition / MICRO_STEPS_PER_DEGREE))) / (seconds * seconds);
                targetSpeed = acceleration * (seconds / 2);
            }
        }
    }

    void update()
    {
        unsigned long currentTime = micros();
        unsigned long elapsedTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        if (pin1A == 4)
        {
            Serial.printf(">speed:%.3f\n>target:%d\n>position:%.3f\n", currentSpeed, targetPosition / MICRO_STEPS_PER_DEGREE, currentPosition / (float)MICRO_STEPS_PER_DEGREE);
        }

        if (!isRunning)
            return;

        // float decelerationTime = currentSpeed / acceleration; // Time to stop in seconds
        float decelerationDist = (currentSpeed * currentSpeed) / (2 * acceleration); // Distance to stop in degrees

        if (abs(targetPosition - currentPosition) <= decelerationDist * MICRO_STEPS_PER_DEGREE)
        {
            // Decelerate
            currentSpeed -= (acceleration * elapsedTime) / 1000000.0;
        }
        else
        {
            // Accelerate
            if (currentSpeed < targetSpeed) // Max speed limit
                currentSpeed += (acceleration * elapsedTime) / 1000000.0;
        }

        if (currentTime < nextStepTime)
            return; // Not time to step yet

        if (currentPosition < targetPosition)
        {
            // Move clockwise
            currentPosition = (currentPosition + 1);
        }
        else if (currentPosition > targetPosition)
        {
            // Move counter-clockwise
            currentPosition = (currentPosition - 1);
        }
        else
        {
            // Already at target position
            isRunning = false;
            currentSpeed = 0;
            return;
        }

        writeStep(currentPosition);
        nextStepTime = currentTime + (1000000 / (currentSpeed * MICRO_STEPS_PER_DEGREE)); // Calculate next step time based on speed
    }

    // Utility functions
    uint16_t getCurrentPosition() const // in DEGREES
    {
        return currentPosition / MICRO_STEPS_PER_DEGREE;
    }

    bool isMotorRunning() const
    {
        return isRunning;
    }

    void stop()
    {
        isRunning = false;
    }

    void setPosition(uint16_t position) // in degrees
    {
        // Set the current position for calibration purposes
        // This allows you to tell the motor where it actually is
        currentPosition = position * MICRO_STEPS_PER_DEGREE;
        writeStep(currentPosition);
    }

    void disable()
    {
        // Turn off all coils to save power
        digitalWrite(pin1A, LOW);
        digitalWrite(pin1B, LOW);
        digitalWrite(pin2A, LOW);
        digitalWrite(pin2B, LOW);
        isRunning = false;
    }
};