#include "Arduino.h"
#include "../../Master/src/motorcontrol.h"

class StepperMotor
{
private:
    // Pin assignments
    int pin1A, pin1B, pin2A, pin2B;

    // Motor state
    uint16_t currentPosition; // Logical position (0-719)
    uint16_t currentHalfStep; // Physical half-step position (0-1439)
    bool isRunning;
    bool continuousMode;

    // Motion parameters
    uint16_t targetPosition;
    uint16_t totalSteps;
    MotorDirection_t moveDirection;
    uint32_t stepInterval;
    uint32_t lastStepTime;
    uint16_t currentSpeed;
    uint16_t continuousSpeed; // Store the continuous mode speed
    uint8_t acceleration;

    // Timing variables
    uint32_t motionStartTime;
    uint32_t motionDuration;
    uint16_t accelerationPhaseSteps;
    uint16_t constantSpeedSteps;
    uint16_t decelerationPhaseSteps;
    uint16_t stepsCompleted;
    uint16_t initialMotionSteps; // Track initial motion steps separately

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
    void writeStep(uint16_t halfStep)
    {
        uint8_t step = halfStep % 8;
        digitalWrite(pin1A, stepSequence[step][0]);
        digitalWrite(pin1B, stepSequence[step][1]);
        digitalWrite(pin2A, stepSequence[step][2]);
        digitalWrite(pin2B, stepSequence[step][3]);
    }

    uint16_t calculateShortestPath(uint16_t current, uint16_t target)
    {
        uint16_t clockwiseSteps = (target >= current) ? (target - current) : (720 - current + target);
        uint16_t counterClockwiseSteps = (current >= target) ? (current - target) : (720 - target + current);
        return (clockwiseSteps <= counterClockwiseSteps) ? clockwiseSteps : counterClockwiseSteps;
    }

    MotorDirection_t getShortestDirection(uint16_t current, uint16_t target)
    {
        uint16_t clockwiseSteps = (target >= current) ? (target - current) : (720 - current + target);
        uint16_t counterClockwiseSteps = (current >= target) ? (current - target) : (720 - target + current);
        return (clockwiseSteps <= counterClockwiseSteps) ? MotorDirection_t::MOTOR_CW : MotorDirection_t::MOTOR_CCW;
    }

    void calculateAcceleratedMotionProfile(uint16_t totalHalfSteps, uint32_t timeMs, uint8_t baseAcceleration)
    {
        float totalTime = timeMs / 1000.0f; // Convert to seconds

        // Calculate what acceleration is needed to complete the motion in the specified time
        // For triangular profile: distance = 0.5 * a * t^2, so a = 2 * distance / t^2
        float requiredAcceleration = (2.0f * totalHalfSteps) / (totalTime * totalTime);

        // Use the required acceleration to meet timing, but prefer the requested acceleration if it works
        float actualAcceleration;

        // Check if the requested acceleration can complete the motion in time
        float timeNeededWithRequestedAccel = sqrt((2.0f * totalHalfSteps) / (float)baseAcceleration);

        if (timeNeededWithRequestedAccel <= totalTime)
        {
            // Requested acceleration is fast enough, but might be too fast
            // Use the slower of: requested acceleration or required acceleration
            actualAcceleration = min((float)baseAcceleration, requiredAcceleration);
        }
        else
        {
            // Requested acceleration is too slow, need to speed up
            actualAcceleration = requiredAcceleration;
        }

        // Ensure we have a reasonable minimum acceleration and maximum to prevent step skipping
        actualAcceleration = max(actualAcceleration, 10.0f);   // Minimum 10 steps/sec²
        actualAcceleration = min(actualAcceleration, 1000.0f); // Maximum 1000 steps/sec² to prevent step skipping

        // Calculate if we can achieve a trapezoidal profile or need triangular
        float timeToReachMaxSpeed = sqrt((2.0f * totalHalfSteps) / actualAcceleration);

        if (timeToReachMaxSpeed * 2 <= totalTime)
        {
            // Trapezoidal profile - accelerate, constant speed, decelerate
            float accelTime = timeToReachMaxSpeed;
            float constantTime = totalTime - 2 * accelTime;
            float maxSpeed = actualAcceleration * accelTime;

            // Limit maximum speed to prevent step skipping (adjust based on your motor)
            maxSpeed = min(maxSpeed, 500.0f); // Max 500 steps/sec

            accelerationPhaseSteps = (uint16_t)(0.5f * actualAcceleration * accelTime * accelTime);
            decelerationPhaseSteps = accelerationPhaseSteps;
            constantSpeedSteps = (uint16_t)(maxSpeed * constantTime);

            // Adjust if rounding errors cause mismatch
            uint16_t calculatedTotal = accelerationPhaseSteps + constantSpeedSteps + decelerationPhaseSteps;
            if (calculatedTotal != totalHalfSteps)
            {
                constantSpeedSteps += (totalHalfSteps - calculatedTotal);
            }

            // Store the acceleration and max speed for use in motion
            this->acceleration = (uint8_t)min(actualAcceleration, 255.0f);
            currentSpeed = (uint16_t)min(maxSpeed, 65535.0f);
        }
        else
        {
            // Triangular profile - accelerate to peak, then decelerate
            // Recalculate acceleration for perfect triangular profile
            actualAcceleration = (4.0f * totalHalfSteps) / (totalTime * totalTime);

            // Apply limits to prevent step skipping
            actualAcceleration = max(actualAcceleration, 10.0f);
            actualAcceleration = min(actualAcceleration, 1000.0f);

            float peakTime = totalTime / 2.0f;
            float peakSpeed = actualAcceleration * peakTime;

            // Limit peak speed
            peakSpeed = min(peakSpeed, 500.0f);

            accelerationPhaseSteps = totalHalfSteps / 2;
            decelerationPhaseSteps = totalHalfSteps - accelerationPhaseSteps;
            constantSpeedSteps = 0;

            // Store the acceleration and peak speed for use in motion
            this->acceleration = (uint8_t)min(actualAcceleration, 255.0f);
            currentSpeed = (uint16_t)min(peakSpeed, 65535.0f);
        }
    }

    uint16_t getCurrentTargetSpeed(uint32_t elapsedTime)
    {
        float elapsedSec = elapsedTime / 1000.0f;
        float totalTimeSec = motionDuration / 1000.0f;

        // Calculate how many steps we should have completed by now
        float timeProgress = elapsedSec / totalTimeSec;
        uint16_t totalMotionSteps = accelerationPhaseSteps + constantSpeedSteps + decelerationPhaseSteps;

        // Clamp time progress to prevent going beyond the motion profile
        timeProgress = min(timeProgress, 1.0f);
        uint16_t targetStepsCompleted = (uint16_t)(timeProgress * totalMotionSteps);

        if (targetStepsCompleted <= accelerationPhaseSteps)
        {
            // Acceleration phase: v = a * t
            float accelProgress = (float)targetStepsCompleted / accelerationPhaseSteps;
            float accelTimeProgress = accelProgress * (totalTimeSec * accelerationPhaseSteps / totalMotionSteps);
            uint16_t speed = (uint16_t)(acceleration * accelTimeProgress);
            return max(speed, (uint16_t)1); // Ensure minimum speed
        }
        else if (targetStepsCompleted <= accelerationPhaseSteps + constantSpeedSteps)
        {
            // Constant speed phase
            return currentSpeed;
        }
        else
        {
            // Deceleration phase
            uint16_t decelerationSteps = targetStepsCompleted - accelerationPhaseSteps - constantSpeedSteps;
            float decelProgress = (float)decelerationSteps / decelerationPhaseSteps;
            float remainingSpeed = currentSpeed * (1.0f - decelProgress);

            // In continuous mode, don't decelerate to zero - transition to continuous speed
            if (continuousMode)
            {
                float minSpeed = max((float)continuousSpeed, currentSpeed * 0.1f); // Don't go below 10% of max speed or continuous speed
                return (uint16_t)max(remainingSpeed, minSpeed);
            }

            // Return 0 when deceleration is complete (non-continuous mode)
            if (decelProgress >= 1.0f)
            {
                return 0;
            }

            return (uint16_t)max(remainingSpeed, 1.0f); // Ensure minimum speed until complete
        }
    }

public:
    static const uint16_t STEPS_PER_REVOLUTION = 720;       // Logical steps (user-facing)
    static const uint16_t HALF_STEPS_PER_REVOLUTION = 1440; // Physical half-steps

    StepperMotor(int pin1A, int pin1B, int pin2A, int pin2B)
        : pin1A(pin1A), pin1B(pin1B), pin2A(pin2A), pin2B(pin2B),
          currentPosition(0), currentHalfStep(0), isRunning(false), continuousMode(false),
          targetPosition(0), totalSteps(0), moveDirection(MotorDirection_t::MOTOR_CW),
          stepInterval(1000), lastStepTime(0), currentSpeed(0), continuousSpeed(0), acceleration(100),
          initialMotionSteps(0)
    {
        // Initialize pins
        pinMode(pin1A, OUTPUT);
        pinMode(pin1B, OUTPUT);
        pinMode(pin2A, OUTPUT);
        pinMode(pin2B, OUTPUT);

        // Set initial position
        writeStep(currentHalfStep);
    }

    void applyMotorControl(const MotorControl_t &control)
    {
        // Normalize target position to 0-719 range
        uint16_t normalizedTarget = control.position % STEPS_PER_REVOLUTION;

        // Determine direction and steps to move
        MotorDirection_t actualDirection = control.direction;
        uint16_t stepsToMove;

        if (control.direction == MotorDirection_t::MOTOR_SHORTEST)
        {
            actualDirection = getShortestDirection(currentPosition, normalizedTarget);
            stepsToMove = calculateShortestPath(currentPosition, normalizedTarget);
        }
        else if (control.direction == MotorDirection_t::MOTOR_CW)
        {
            stepsToMove = (normalizedTarget >= currentPosition) ? (normalizedTarget - currentPosition) : (STEPS_PER_REVOLUTION - currentPosition + normalizedTarget);
        }
        else
        { // MOTOR_CCW
            stepsToMove = (currentPosition >= normalizedTarget) ? (currentPosition - normalizedTarget) : (STEPS_PER_REVOLUTION - normalizedTarget + currentPosition);
        }

        // Handle case where we're already at target position
        if (stepsToMove == 0 && !control.keepRunning)
        {
            isRunning = false;
            return;
        }

        // Set motion parameters
        targetPosition = normalizedTarget;
        moveDirection = actualDirection;
        continuousMode = control.keepRunning;
        motionDuration = control.time;
        continuousSpeed = control.speed; // Store the continuous speed

        // For continuous mode, ensure we have some motion even if already at target
        if (control.keepRunning && stepsToMove == 0)
        {
            stepsToMove = 1; // Move at least one step to start motion
        }

        initialMotionSteps = stepsToMove * 2; // Convert logical steps to half-steps
        totalSteps = initialMotionSteps;

        // Calculate motion profile with dynamic acceleration adjustment
        if (stepsToMove > 0)
        {
            calculateAcceleratedMotionProfile(stepsToMove * 2, control.time, control.acceleration);
            motionStartTime = millis();
            stepsCompleted = 0; // Reset step counter
            isRunning = true;
        }
    }

    void update()
    {
        if (!isRunning)
            return;

        uint32_t currentTime = millis();
        uint32_t elapsedTime = currentTime - motionStartTime;

        // Determine current target speed
        uint16_t targetSpeed;

        if (continuousMode && elapsedTime >= motionDuration)
        {
            // Switch to continuous mode at specified speed
            targetSpeed = continuousSpeed;
        }
        else
        {
            targetSpeed = getCurrentTargetSpeed(elapsedTime);
        }

        // Update step interval based on current speed
        if (targetSpeed > 0)
        {
            stepInterval = 1000000 / targetSpeed; // Convert to microseconds per half-step
        }
        else
        {
            if (continuousMode)
            {
                // In continuous mode, never stop - use minimum speed
                stepInterval = 1000000; // 1 step per second minimum
            }
            else
            {
                // Non-continuous mode can stop
                isRunning = false;
                currentPosition = targetPosition;
                currentHalfStep = (targetPosition * 2) % HALF_STEPS_PER_REVOLUTION;
                writeStep(currentHalfStep);
                return;
            }
        }

        // Check if it's time for the next step
        if (currentTime - lastStepTime >= stepInterval / 1000)
        {
            // Perform the half-step
            if (moveDirection == MotorDirection_t::MOTOR_CW)
            {
                currentHalfStep = (currentHalfStep + 1) % HALF_STEPS_PER_REVOLUTION;
            }
            else
            {
                currentHalfStep = (currentHalfStep == 0) ? HALF_STEPS_PER_REVOLUTION - 1 : currentHalfStep - 1;
            }

            // Update logical position from half-step position
            currentPosition = currentHalfStep / 2;

            writeStep(currentHalfStep);
            lastStepTime = currentTime;
            stepsCompleted++;

            // Handle step counting for non-continuous mode during initial motion
            if (!continuousMode && elapsedTime < motionDuration)
            {
                if (totalSteps > 0)
                {
                    totalSteps--;
                }
                if (totalSteps == 0)
                {
                    isRunning = false;
                    currentPosition = targetPosition; // Ensure exact target position
                }
            }
            else if (!continuousMode && elapsedTime >= motionDuration)
            {
                // Non-continuous mode and time is up
                isRunning = false;
                currentPosition = targetPosition;
                currentHalfStep = (targetPosition * 2) % HALF_STEPS_PER_REVOLUTION;
                writeStep(currentHalfStep);
            }
            // For continuous mode, keep running indefinitely after initial motion
        }
    }

    // Utility functions
    uint16_t getCurrentPosition() const
    {
        return currentPosition;
    }

    bool isMotorRunning() const
    {
        return isRunning;
    }

    void stop()
    {
        isRunning = false;
        continuousMode = false;
    }

    void setPosition(uint16_t position)
    {
        // Set the current position for calibration purposes
        // This allows you to tell the motor where it actually is
        currentPosition = position % STEPS_PER_REVOLUTION;
        currentHalfStep = (currentPosition * 2) % HALF_STEPS_PER_REVOLUTION;
        writeStep(currentHalfStep);
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