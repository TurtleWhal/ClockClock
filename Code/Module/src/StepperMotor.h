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
    bool continuous = false; // Whether to keep running after reaching target

    uint16_t targetSpeed = 0; // in degrees per second

    uint32_t acceleration = 50; // Acceleration in degrees per second per second

    float targetPosition; // in microsteps

    unsigned long lastUpdateTime = micros();

    bool clockwise = true;

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
            currentSpeed = fmax(currentSpeed, 1);

            if (!isRunning) // preserve direction if already running
            {
                if (control.direction == MOTOR_SHORTEST)
                {
                    int16_t diff = targetPosition - currentPosition;
                    if (diff != 0)
                    {
                        diff = (diff + MICRO_STEPS_PER_REVOLUTION) % MICRO_STEPS_PER_REVOLUTION; // Normalize to [0, 360)
                        clockwise = diff < (MICRO_STEPS_PER_REVOLUTION / 2);
                    }
                }
                else
                {
                    clockwise = control.direction != MOTOR_CCW;
                }
            }

            nextStepTime = micros();
            isRunning = true;
            continuous = false;

            if (control.time != UINT16_MAX)
            {
                float seconds = control.time / 1000.0;

                uint16_t diff = abs((targetPosition / MICRO_STEPS_PER_DEGREE) - (currentPosition / MICRO_STEPS_PER_DEGREE));

                if (!clockwise)
                    diff = 360 - diff;

                acceleration = (4 * diff) / (seconds * seconds);
                targetSpeed = acceleration * (seconds / 2);
            }
        }
        else
        {
            targetPosition = control.position * MICRO_STEPS_PER_DEGREE;
            targetSpeed = control.speed;
            acceleration = control.acceleration;
            currentSpeed = fmax(currentSpeed, 1);
            nextStepTime = micros();
            clockwise = control.direction != MOTOR_CCW;
            isRunning = true;
            continuous = true;
        }
    }

    void update()
    {
        unsigned long currentTime = micros();
        unsigned long elapsedTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        if (continuous)
        {
            targetPosition += ((elapsedTime * targetSpeed * MICRO_STEPS_PER_DEGREE) / 1000000.0) * (clockwise ? 1 : -1);

            if (currentPosition >= MICRO_STEPS_PER_REVOLUTION)
            {
                currentPosition %= MICRO_STEPS_PER_REVOLUTION;
                targetPosition = fmod(targetPosition, MICRO_STEPS_PER_REVOLUTION);
            }
        }

        if (pin1A == 4)
        {
            Serial.printf(">speed:%f\n>target:%f\n>position:%f\n", currentSpeed, targetPosition / MICRO_STEPS_PER_DEGREE, currentPosition / (float)MICRO_STEPS_PER_DEGREE);
        }

        if (!isRunning)
            return;

        // Stop if reached target (for non-continuous mode)
        if (!continuous && abs((int)(currentPosition - targetPosition)) < 1)
        {
            isRunning = false;
            currentSpeed = 0;
            return;
        }

        // float decelerationTime = currentSpeed / acceleration; // Time to stop in seconds
        float decelerationDist = (currentSpeed * currentSpeed) / (2 * acceleration); // Distance to stop in degrees

        // Calculate distance to target in the direction of movement (clockwise or counterclockwise), accounting for wrap-around
        uint16_t pos = currentPosition % MICRO_STEPS_PER_REVOLUTION;
        uint16_t tgt = ((int)targetPosition + MICRO_STEPS_PER_REVOLUTION) % MICRO_STEPS_PER_REVOLUTION;
        int16_t dist;
        if (clockwise)
        {
            dist = tgt - pos;
            if (dist < 0)
                dist += MICRO_STEPS_PER_REVOLUTION;
        }
        else
        {
            dist = pos - tgt;
            if (dist < 0)
                dist += MICRO_STEPS_PER_REVOLUTION;
        }
        dist /= MICRO_STEPS_PER_DEGREE;

        if (dist <= decelerationDist)
        {
            // Decelerate
            currentSpeed -= (acceleration * elapsedTime) / 1000000.0;
        }
        else
        {
            // Accelerate
            if (currentSpeed < targetSpeed || continuous) // Max speed limit
                currentSpeed += (acceleration * elapsedTime) / 1000000.0;
        }

        if (currentTime < nextStepTime)
            return; // Not time to step yet

        if (clockwise)
        {
            currentPosition = (currentPosition + 1) % MICRO_STEPS_PER_REVOLUTION;
        }
        else
        {
            currentPosition = (currentPosition == 0) ? (MICRO_STEPS_PER_REVOLUTION - 1) : (currentPosition - 1);
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