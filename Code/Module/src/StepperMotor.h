#include "Arduino.h"

#define OFF 0
#define N 1
#define S -1

class StepperMotor
{
private:
    int _pin1;
    int _pin2;
    int _pin3;
    int _pin4;
    int _steps = 720;
    int _microsteps = 16;
    bool _microstep = false;

    int _targetPosition = 0;
    double _currentPosition = 0.0;
    int _phase = 0;

    bool _running = false;

    // double _acceleration = 120.0; // steps/second/ loop?
    // double _maxspeed = 360.0; // steps/second
    double _acceleration = 60.0; // steps/second/ loop?
    double _maxspeed = 360.0; // steps/second
    double _speed = 0.0; // steps/second

    bool _speedMode = false;
    int _targetSpeed = 0;

    unsigned long lastTime = 0; // target time for next step in microseconds
    unsigned long targetTime = 0; // target time for next step in microseconds

    const int _phases[8][2] = {
        {OFF, N},
        {N, N},
        {N, OFF},
        {N, S},
        {OFF, S},
        {S, S},
        {S, OFF},
        {S, N}
    };

    void writeMagnet(int p1, int p2, int state);

public:
    StepperMotor(int pin1, int pin2, int pin3, int pin4);

    void handle();

    void setTargetPosition(int position);
    int getTargetPosition();
    void setCurrentPosition(int position);
    int getCurrentPosition();
    void setTargetSpeed(int speed);
    void setSpeed(int speed); // steps/second
    void setAcceleration(int acceleration);
};