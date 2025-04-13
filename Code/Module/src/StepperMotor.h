#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include <Arduino.h>

class StepperMotor
{
public:
    enum MagnetState
    {
        OFF = 0,
        N = 1,
        S = 2
    };

    StepperMotor(int pin1A, int pin1B, int pin2A, int pin2B);

    void setAcceleration(float acceleration);
    void setMaxSpeed(float speed);
    void setTargetPosition(long position);
    void setTargetSpeed(float speed);
    void setCurrentPosition(long position);
    void stop();

    long getCurrentPosition() const;
    long getTargetPosition() const;

    void run(); // Call frequently in loop()

private:
    void stepMotor(int stepIndex);
    void computeNewSpeed();
    void setMagnet(int magnetIndex, MagnetState state);

    int motorPins[2][2]; // [magnet][A/B pin]
    int currentStepIndex = 0;

    long currentPos = 0;
    long targetPos = 0;

    float currentSpeed = 0.0f;
    float targetSpeed = 0.0f;
    float maxSpeed = 1000.0f;
    float acceleration = 100.0f;

    unsigned long lastStepTime = 0;
    unsigned long stepInterval = 1000; // microseconds

    bool movingToPosition = false;
    bool isStopping = false;
};

#endif
