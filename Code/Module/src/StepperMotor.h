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
    bool continuous = false;    // Whether to keep running after reaching target
    bool wasContinuous = false; // Track if previous command was continuous

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
        unsigned long currentTime = micros();
        unsigned long elapsedTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        // In continuous mode, the target moves infinitely ahead
        if (continuous)
        {
            // Keep target position far ahead to prevent stopping
            if (clockwise)
            {
                targetPosition = currentPosition + MICRO_STEPS_PER_REVOLUTION;
            }
            else
            {
                targetPosition = currentPosition - MICRO_STEPS_PER_REVOLUTION;
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
        if (!continuous)
        {
            // Calculate actual distance remaining considering direction
            int32_t diff = targetPosition - currentPosition;

            if (clockwise)
            {
                // Going clockwise - if diff is negative, we haven't wrapped yet
                if (diff < 0)
                    diff += MICRO_STEPS_PER_REVOLUTION;
            }
            else
            {
                // Going counter-clockwise - if diff is positive, we need to wrap
                if (diff > 0)
                    diff -= MICRO_STEPS_PER_REVOLUTION;
                diff = -diff; // Make positive for comparison
            }

            // if (diff <= 1 && currentSpeed < 10)
            if (diff <= 1)
            {
                isRunning = false;
                currentSpeed = 0;
                currentPosition = targetPosition; // Snap to exact position
                writeStep(currentPosition);
                return;
            }
        }

        // In continuous mode, accelerate to target speed and maintain it
        if (continuous)
        {
            if (currentSpeed < targetSpeed)
            {
                float newSpeed = currentSpeed + (acceleration * elapsedTime) / 1000000.0;
                currentSpeed = (newSpeed > targetSpeed) ? targetSpeed : newSpeed;
            }
            else if (currentSpeed > targetSpeed)
            {
                float newSpeed = currentSpeed - (acceleration * elapsedTime) / 1000000.0;
                currentSpeed = (newSpeed < targetSpeed) ? targetSpeed : newSpeed;
            }
        }
        else
        {
            // Position mode: use acceleration/deceleration profile
            float decelerationDist = (currentSpeed * currentSpeed) / (2 * acceleration); // Distance to stop in degrees

            // Calculate distance to target in the direction of movement
            int32_t diff = targetPosition - currentPosition;
            float dist;

            if (clockwise)
            {
                // Going clockwise
                if (diff < 0)
                    diff += MICRO_STEPS_PER_REVOLUTION;
                dist = diff / (float)MICRO_STEPS_PER_DEGREE;
            }
            else
            {
                // Going counter-clockwise
                if (diff > 0)
                    diff -= MICRO_STEPS_PER_REVOLUTION;
                dist = (-diff) / (float)MICRO_STEPS_PER_DEGREE;
            }

            if (dist <= decelerationDist)
            {
                // Decelerate at the same rate as acceleration
                float newSpeed = currentSpeed - (acceleration * elapsedTime) / 1000000.0;
                currentSpeed = (newSpeed < 0) ? 0 : newSpeed;
            }
            else
            {
                // Accelerate toward target speed
                if (currentSpeed < targetSpeed)
                {
                    float newSpeed = currentSpeed + (acceleration * elapsedTime) / 1000000.0;
                    currentSpeed = (newSpeed > targetSpeed) ? targetSpeed : newSpeed;
                }
            }
        }

        // Prevent speed from going negative
        if (currentSpeed < 0)
            currentSpeed = 0;

        if (currentTime >= nextStepTime && currentSpeed > 0)
        {
            if (clockwise)
            {
                currentPosition++;
                if (currentPosition >= MICRO_STEPS_PER_REVOLUTION)
                    currentPosition = 0;
            }
            else
            {
                if (currentPosition == 0)
                    currentPosition = MICRO_STEPS_PER_REVOLUTION - 1;
                else
                    currentPosition--;
            }

            writeStep(currentPosition);
            nextStepTime = currentTime + (1000000 / (currentSpeed * MICRO_STEPS_PER_DEGREE)); // Calculate next step time based on speed
        }

        esp_timer_start_once(timer, 100); // delay in microseconds
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
    }

    void applyMotorControl(const MotorControl_t &control)
    {
        // Stop the timer to prevent race conditions during state update
        esp_timer_stop(timer);

        // Normalize current position to valid range first
        while (currentPosition >= MICRO_STEPS_PER_REVOLUTION)
            currentPosition -= MICRO_STEPS_PER_REVOLUTION;
        while (currentPosition < 0)
            currentPosition += MICRO_STEPS_PER_REVOLUTION;

        if (control.keepRunning == false)
        {
            // Position mode
            float newTargetPosition = control.position * MICRO_STEPS_PER_DEGREE;

            // Ensure target position is in valid range [0, MICRO_STEPS_PER_REVOLUTION)
            while (newTargetPosition >= MICRO_STEPS_PER_REVOLUTION)
                newTargetPosition -= MICRO_STEPS_PER_REVOLUTION;
            while (newTargetPosition < 0)
                newTargetPosition += MICRO_STEPS_PER_REVOLUTION;

            uint16_t newTargetSpeed = control.speed;
            uint32_t newAcceleration = control.acceleration;
            bool newClockwise = clockwise; // Default to current direction

            // For smooth starts: only preserve speed if we're continuing in same direction
            // and the new target makes sense with current motion
            bool shouldPreserveSpeed = false;

            if (isRunning)
            {
                if (wasContinuous)
                {
                    // Coming from continuous mode - always preserve speed for smooth transition
                    shouldPreserveSpeed = true;
                    // Keep existing direction when coming from continuous
                }
                else
                {
                    // Check if new target is in the same general direction
                    int32_t diff = newTargetPosition - currentPosition;

                    if (clockwise && diff > 0 && diff < MICRO_STEPS_PER_REVOLUTION / 2)
                        shouldPreserveSpeed = true;
                    else if (!clockwise && diff < 0 && diff > -(MICRO_STEPS_PER_REVOLUTION / 2))
                        shouldPreserveSpeed = true;
                }
            }

            if (shouldPreserveSpeed && currentSpeed >= 1)
            {
                // Keep current speed for smooth transition
                // Don't reset speed
            }
            else
            {
                // Start from low speed for smooth acceleration
                currentSpeed = 1;
            }

            // Determine direction
            if (wasContinuous)
            {
                // Coming from continuous mode - keep current direction for smooth transition
                newClockwise = clockwise;

                // CRITICAL: Ensure we stop at the FIRST occurrence of target position
                // We allow up to one full rotation (360°), but not multiple loops
                // The target position is already normalized to [0, 360), so we just need
                // to make sure we hit it on this revolution, not the next one
            }
            else if (!isRunning || !shouldPreserveSpeed) // Set direction if motor was stopped or changing direction
            {
                if (control.direction == MotorDirection_t::MOTOR_SHORTEST)
                {
                    // Calculate shortest path
                    int32_t diff = newTargetPosition - currentPosition;

                    // Normalize to range [-MICRO_STEPS_PER_REVOLUTION/2, MICRO_STEPS_PER_REVOLUTION/2]
                    while (diff > MICRO_STEPS_PER_REVOLUTION / 2)
                        diff -= MICRO_STEPS_PER_REVOLUTION;
                    while (diff < -(MICRO_STEPS_PER_REVOLUTION / 2))
                        diff += MICRO_STEPS_PER_REVOLUTION;

                    // Positive diff means clockwise is shorter
                    newClockwise = (diff > 0);
                }
                else
                {
                    newClockwise = control.direction != MotorDirection_t::MOTOR_CCW;
                }
            }

            // Calculate time-based motion profile if time is specified
            if (control.time != UINT16_MAX && control.time > 0)
            {
                float seconds = control.time / 1000.0;

                // Calculate distance to travel in degrees based on direction
                int32_t diff = newTargetPosition - currentPosition;

                if (newClockwise)
                {
                    // Going clockwise
                    if (diff < 0)
                        diff += MICRO_STEPS_PER_REVOLUTION;
                }
                else
                {
                    // Going counter-clockwise
                    if (diff > 0)
                        diff -= MICRO_STEPS_PER_REVOLUTION;
                    diff = -diff; // Make positive for calculation
                }

                float distDegrees = diff / (float)MICRO_STEPS_PER_DEGREE;

                // Triangular velocity profile: accelerate for half time, decelerate for half time
                // Distance = (1/2) * vmax * t (for triangular profile with equal accel/decel)
                // Since we split time in half: vmax = (2 * distance) / time
                newTargetSpeed = (2.0 * distDegrees) / seconds;

                // Acceleration: vmax = a * (t/2), so a = 2*vmax / t
                newAcceleration = (2.0 * newTargetSpeed) / seconds;

                // Ensure reasonable limits
                if (newTargetSpeed > 1440)
                    newTargetSpeed = 1440; // Max 4 rev/sec for longer distances
                if (newAcceleration > 2000)
                    newAcceleration = 2000; // Higher acceleration limit
            }
            else
            {
                // No explicit time specified - calculate required speed based on distance
                // This ensures motors taking longer paths speed up appropriately

                // Calculate distance in the current direction
                int32_t diff = (int32_t)newTargetPosition - (int32_t)currentPosition;

                if (newClockwise)
                {
                    // Going clockwise: if diff is negative, add full revolution
                    if (diff < 0)
                        diff += MICRO_STEPS_PER_REVOLUTION;
                    // Ensure diff is in valid range [0, MICRO_STEPS_PER_REVOLUTION)
                    if (diff >= MICRO_STEPS_PER_REVOLUTION)
                        diff = diff % MICRO_STEPS_PER_REVOLUTION;
                }
                else
                {
                    // Going counter-clockwise: if diff is positive, subtract full revolution
                    if (diff > 0)
                        diff -= MICRO_STEPS_PER_REVOLUTION;
                    diff = -diff; // Make positive for calculation
                    // Ensure diff is in valid range [0, MICRO_STEPS_PER_REVOLUTION)
                    if (diff >= MICRO_STEPS_PER_REVOLUTION)
                        diff = diff % MICRO_STEPS_PER_REVOLUTION;
                    if (diff < 0)
                        diff = 0;
                }

                float distDegrees = diff / (float)MICRO_STEPS_PER_DEGREE;

                // Sanity check - if distance is unreasonably large, something went wrong
                // Clamp to maximum one full revolution
                if (distDegrees > 360)
                {
                    distDegrees = fmod(distDegrees, 360.0);
                    if (distDegrees == 0)
                        distDegrees = 360; // If exactly at target after modulo, go full circle
                }

                // Calculate speed multiplier based on distance
                // Base case: 180° or less travels at specified speed
                // Longer distances scale up proportionally
                if (distDegrees > 180)
                {
                    float speedMultiplier = distDegrees / 180.0;
                    newTargetSpeed = newTargetSpeed * speedMultiplier;
                    newAcceleration = newAcceleration * speedMultiplier;

                    // Apply reasonable limits
                    if (newTargetSpeed > 1440)
                        newTargetSpeed = 1440;
                    if (newAcceleration > 2000)
                        newAcceleration = 2000;
                }
                // For distances <= 180°, use the speeds as specified
            }

            // Apply all changes atomically
            targetPosition = newTargetPosition;
            targetSpeed = newTargetSpeed;
            acceleration = newAcceleration;
            clockwise = newClockwise;
            continuous = false;
            wasContinuous = false; // Clear flag after using it
            nextStepTime = micros();
            isRunning = true;
        }
        else
        {
            // Continuous rotation mode (keepRunning = true)

            // Preserve current speed for smooth transition
            if (!isRunning || currentSpeed < 1)
                currentSpeed = 1;

            // Apply all changes atomically
            targetSpeed = control.speed;
            acceleration = control.acceleration;
            clockwise = control.direction != MotorDirection_t::MOTOR_CCW;
            continuous = true;
            wasContinuous = true; // Mark that we're in continuous mode
            nextStepTime = micros();
            isRunning = true;

            // Set target position far ahead in the direction of travel
            // This will be continuously updated in the update() function
            if (clockwise)
                targetPosition = currentPosition + MICRO_STEPS_PER_REVOLUTION;
            else
                targetPosition = currentPosition - MICRO_STEPS_PER_REVOLUTION;
        }

        // Restart timer after all state is updated
        esp_timer_start_once(timer, 0); // Start immediately
    }

    // get Current position in degrees
    uint16_t getCurrentPosition() const
    {
        return (currentPosition % MICRO_STEPS_PER_REVOLUTION) / MICRO_STEPS_PER_DEGREE;
    }

    bool isMotorRunning() const
    {
        return isRunning;
    }

    void stop()
    {
        isRunning = false;
        continuous = false;
        wasContinuous = false;
    }

    void setPosition(uint16_t position) // in degrees
    {
        // Set the current position for calibration purposes
        // This allows you to tell the motor where it actually is
        currentPosition = position * MICRO_STEPS_PER_DEGREE;
        targetPosition = currentPosition;
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
        continuous = false;
        wasContinuous = false;
    }

    void pause()
    {
        esp_timer_stop(timer);
    }
};