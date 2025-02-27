#include "Arduino.h"

#define OFF 0
#define N 1
#define S 2

class StepperMotor
{
private:
    int _pin1;
    int _pin2;
    int _pin3;
    int _pin4;
    int _steps = 720;
    int _microsteps = 2;

    int _targetPosition = 0;
    int _currentPosition = 0;
    int _phase = 0;

    bool _running = false;

    double _acceleration = 60.0; // steps/second/ loop?
    double _maxspeed = 720.0; // steps/second
    double _speed = 0.0; // steps/second

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
    int getCurrentPosition();
    int getTargetPosition();
    void setSpeed(int speed); // steps/second
    void setAcceleration(int acceleration);
};