#include "Arduino.h"
#include "pwm.h"
#include "../../Master/src/motorcontrol.h"

#define MICROSTEPS 32
#define STEPS_PER_REVOLUTION 720                                       // Logical steps (user-facing)
#define MICRO_STEPS_PER_REVOLUTION (STEPS_PER_REVOLUTION * MICROSTEPS) // Physical microsteps
#define MICRO_STEPS_PER_DEGREE (MICRO_STEPS_PER_REVOLUTION / 360)      // Physical microsteps
#define STEP_AMPLITUDE 1.0f

class StepperMotor
{
private:
    // Pin assignments
    int pin1A, pin1B, pin2A, pin2B;
    bool log = false;

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

    uint16_t sinTable[MICROSTEPS * 4];

    void generateMicrostepTable()
    {
        if (log)
            Serial.print("Sine table: {");

        for (uint8_t i = 0; i < MICROSTEPS * 4; i++)
        {
            float rad = ((i % (MICROSTEPS * 4)) * (PI * 2)) / (MICROSTEPS * 4.0f);
            sinTable[i] = sinf(rad) * UINT16_MAX * STEP_AMPLITUDE; // will have the right behaviour when sin is negative when it wraps around
            
            if (log)
            {
                Serial.print(sinTable[i]);

                if (i < MICROSTEPS * 4 - 1)
                    Serial.print(", ");
            }
        }

        if (log)
            Serial.println("}");
    }

    void update(void *arg)
    {
        esp_timer_start_once(timer, 500); // delay in microseconds

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

        if (log)
        {
            Serial.printf(">speed:%f\n>target:%f\n>position:%f\n", currentSpeed, targetPosition / MICRO_STEPS_PER_DEGREE, currentPosition / (float)MICRO_STEPS_PER_DEGREE);
            Serial.flush();
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

        if (currentTime >= nextStepTime)
        {
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

        // esp_timer_start_once(timer, 500); // delay in microseconds
        // esp_timer_start_once(timer, (1000000 / (currentSpeed * MICRO_STEPS_PER_DEGREE))); // delay in microseconds
    }

    const esp_timer_create_args_t timer_args = {
        .callback = [](void *arg)
        {
            StepperMotor *motor = static_cast<StepperMotor *>(arg);
            motor->update(arg);
        },
        .arg = (void *)this, // arbitrary argument to pass to callback
        .name = "StepperTimer"};

    esp_timer_handle_t timer;

public:
    StepperMotor(int pin1A, int pin1B, int pin2A, int pin2B, bool log = false)
        : pin1A(pin1A), pin1B(pin1B), pin2A(pin2A), pin2B(pin2B),
          currentPosition(0), isRunning(false), log(log)
    {
        generateMicrostepTable();

        // Initialize pins
        pinMode(pin1A, OUTPUT);
        pinMode(pin1B, OUTPUT);
        pinMode(pin2A, OUTPUT);
        pinMode(pin2B, OUTPUT);

        // Set initial position
        writeStep(currentPosition);

        esp_timer_create(&timer_args, &timer);
    }

    void writeStep(uint16_t microStep)
    {
        uint8_t step = (microStep) % (MICROSTEPS * 4);
        uint8_t step90 = (step + MICROSTEPS) % (MICROSTEPS * 4); // 90 degrees offset

        setPWMDuty(pin1A, sinTable[step]);
        digitalWrite(pin1B, step <= (MICROSTEPS * 2) ? LOW : HIGH);

        setPWMDuty(pin2A, sinTable[step90]);
        digitalWrite(pin2B, step90 <= (MICROSTEPS * 2) ? LOW : HIGH);

        // Serial.printf(">s:%d\n", sinTable[step]);
        // Serial.printf(">r:%d\n", step <= (MICROSTEPS * 2) ? 0 : 255);
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

        esp_timer_start_once(timer, 0); // Start immediately
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