#include "StepperMotor.h"

// Step sequence using 2 magnets (coils) and 8 microsteps
const StepperMotor::MagnetState stepSequence[8][2] = {
    {StepperMotor::N, StepperMotor::OFF},
    {StepperMotor::N, StepperMotor::N},
    {StepperMotor::OFF, StepperMotor::N},
    {StepperMotor::S, StepperMotor::N},
    {StepperMotor::S, StepperMotor::OFF},
    {StepperMotor::S, StepperMotor::S},
    {StepperMotor::OFF, StepperMotor::S},
    {StepperMotor::N, StepperMotor::S}};

StepperMotor::StepperMotor(int pin1A, int pin1B, int pin2A, int pin2B)
{
    motorPins[0][0] = pin1A;
    motorPins[0][1] = pin1B;
    motorPins[1][0] = pin2A;
    motorPins[1][1] = pin2B;

    for (int i = 0; i < 2; ++i)
    {
        pinMode(motorPins[i][0], OUTPUT);
        pinMode(motorPins[i][1], OUTPUT);
        digitalWrite(motorPins[i][0], LOW);
        digitalWrite(motorPins[i][1], LOW);
    }
}

void StepperMotor::setAcceleration(float acc)
{
    acceleration = acc;
}

void StepperMotor::setMaxSpeed(float speed)
{
    maxSpeed = speed;
}

void StepperMotor::setTargetPosition(long position)
{
    // Wrap position to 0–719 steps
    position = ((position % 720) + 720) % 720;
    long targetMicrostep = position * 2;

    // Wrap currentPos to range
    currentPos = ((currentPos % 1440) + 1440) % 1440;

    // Compute shortest path
    long delta = targetMicrostep - currentPos;
    if (delta > 720)
        delta -= 1440;
    else if (delta < -720)
        delta += 1440;

    targetPos = currentPos + delta;
    movingToPosition = true;
    isStopping = false;
    computeNewSpeed();
}

void StepperMotor::setTargetSpeed(float speed)
{
    targetSpeed = constrain(speed, -maxSpeed, maxSpeed);
    movingToPosition = false;
    isStopping = false;
}

void StepperMotor::setCurrentPosition(long position)
{
    currentPos = ((position % 720) + 720) % 720;
    currentPos *= 2;
}

void StepperMotor::stop()
{
    isStopping = true;
}

long StepperMotor::getCurrentPosition() const
{
    return (currentPos / 2) % 720;
}

long StepperMotor::getTargetPosition() const
{
    return (targetPos / 2) % 720;
}

void StepperMotor::computeNewSpeed()
{
    long delta = targetPos - currentPos;

    if (delta > 720)
        delta -= 1440;
    else if (delta < -720)
        delta += 1440;

    int direction = (delta > 0) ? 1 : -1;

    float requiredSpeed = sqrt(2.0f * acceleration * abs(delta));
    requiredSpeed = min(requiredSpeed, maxSpeed);
    targetSpeed = direction * requiredSpeed;
}

void StepperMotor::run()
{
    unsigned long now = micros();
    if ((now - lastStepTime) < stepInterval)
        return;

    float timeDelta = (now - lastStepTime) / 1000000.0f;

    if (movingToPosition)
    {
        long delta = targetPos - currentPos;

        // Wrap delta to shortest path
        if (delta > 720)
            delta -= 1440;
        else if (delta < -720)
            delta += 1440;

        float absDelta = abs(delta);

        // Dead zone check — if very close to target and very slow, stop
        if (absDelta <= 1 && abs(currentSpeed) < 1.0f)
        {
            currentPos = targetPos % 1440; // Snap to target
            currentSpeed = 0;
            movingToPosition = false;
            isStopping = false;
            return;
        }

        computeNewSpeed();
    }

    // Accelerate/decelerate
    if (currentSpeed < targetSpeed)
    {
        currentSpeed += acceleration * timeDelta;
        if (currentSpeed > targetSpeed)
            currentSpeed = targetSpeed;
    }
    else if (currentSpeed > targetSpeed)
    {
        currentSpeed -= acceleration * timeDelta;
        if (currentSpeed < targetSpeed)
            currentSpeed = targetSpeed;
    }

    // Stopping logic
    if (isStopping && abs(currentSpeed) < 1.0f)
    {
        currentSpeed = 0;
        return;
    }

    // Step motor if speed is non-zero
    if (currentSpeed != 0)
    {
        int direction = (currentSpeed > 0) ? 1 : -1;

        currentPos += direction;
        currentPos = (currentPos + 1440) % 1440;

        currentStepIndex = (currentStepIndex - direction + 8) % 8;
        stepMotor(currentStepIndex);

        stepInterval = abs(1000000.0f / currentSpeed);
        lastStepTime = now;
    }
}

void StepperMotor::stepMotor(int stepIndex)
{
    for (int i = 0; i < 2; i++)
    {
        setMagnet(i, stepSequence[stepIndex][i]);
    }
}

void StepperMotor::setMagnet(int index, MagnetState state)
{
    switch (state)
    {
    case N:
        digitalWrite(motorPins[index][0], HIGH);
        digitalWrite(motorPins[index][1], LOW);
        break;
    case S:
        digitalWrite(motorPins[index][0], LOW);
        digitalWrite(motorPins[index][1], HIGH);
        break;
    case OFF:
    default:
        digitalWrite(motorPins[index][0], LOW);
        digitalWrite(motorPins[index][1], LOW);
        break;
    }
}
