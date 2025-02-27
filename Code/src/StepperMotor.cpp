#include "Arduino.h"
#include "StepperMotor.h"

StepperMotor::StepperMotor(int pin1, int pin2, int pin3, int pin4)
{
    _pin1 = pin1;
    _pin2 = pin2;
    _pin3 = pin3;
    _pin4 = pin4;
    _steps = 720;

    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    pinMode(_pin3, OUTPUT);
    pinMode(_pin4, OUTPUT);
}

void StepperMotor::handle()
{
    unsigned long currentTime = micros();

    if (_running)
    {
        if (targetTime < currentTime)
        {
            unsigned long elapsedTime = currentTime - lastTime;

            int _distance = (_targetPosition * _microsteps) - _currentPosition;

            // Calculate the remaining deceleration distance based on current speed
            int _decelerationDistance = (_speed * _speed) / (2 * _acceleration);

            if (_distance > 0)
            {
                if (_distance <= _decelerationDistance)
                {
                    // Start decelerating if we're within the deceleration range
                    if (_speed > 0)
                    {
                        // Reduce the speed smoothly based on elapsed time
                        _speed -= _acceleration * elapsedTime / 1000000;
                        if (_speed < 0)
                            _speed = 0; // Prevent negative speed
                    }
                }
                else
                {
                    // Continue accelerating if there's enough distance left
                    if (_speed < _maxspeed)
                    {
                        _speed += _acceleration * elapsedTime / 1000000;
                        if (_speed > _maxspeed)
                            _speed = _maxspeed; // Cap at max speed
                    }
                }

                // Serial.println(_speed);

                // Update the motor phase and position
                _phase = _currentPosition % 8;
                _currentPosition += 1;

                writeMagnet(_pin1, _pin2, _phases[_phase][0]);
                writeMagnet(_pin3, _pin4, _phases[_phase][1]);

                // Calculate the next step's target time based on the current speed
                if (_speed > 0)
                    targetTime = currentTime + (1000000 / _speed); // Use 1000000 to work in seconds
            }
            else
            {
                // Stop the motor when the target position is reached
                _running = false;
                _speed = 0;
            }

            lastTime = currentTime;
        }
    }
}

void StepperMotor::writeMagnet(int p1, int p2, int state)
{
    switch (state)
    {
    case N:
        digitalWrite(p1, HIGH);
        digitalWrite(p2, LOW);
        break;

    case S:
        digitalWrite(p1, LOW);
        digitalWrite(p2, HIGH);
        break;

    case OFF:
        digitalWrite(p1, LOW);
        digitalWrite(p2, LOW);
        break;
    }
}

void StepperMotor::setTargetPosition(int position)
{
    _targetPosition = position;
    _running = true;
}

int StepperMotor::getCurrentPosition()
{
    return _currentPosition;
}

int StepperMotor::getTargetPosition()
{
    return _targetPosition;
}

void StepperMotor::setSpeed(int speed)
{
    // do something with speed
    _maxspeed = speed;
}

void StepperMotor::setAcceleration(int acceleration)
{
    // do something with acceleration
    _acceleration = acceleration;
}

// void StepperMotor::microStep()
// {
//     static int speed = 0;
//     for (double i = 0; i < PI * 2; i += PI / 16)
//     {
//         double sineA = sin(i);
//         uint8_t raiseA = sineA <= 0 ? 1 : 0;
//         analogWrite(_pin1, (sineA + raiseA) * 255);
//         digitalWrite(_pin2, raiseA);
//         analogWrite(_pin4, (sineA + raiseA) * 255);
//         digitalWrite(_pin3, raiseA);

//         // double sineB = sin(i + (PI / 2));
//         // uint8_t raiseB = sineB <= 0 ? 1 : 0;
//         // analogWrite(14, (sineB + raiseB) * 255);
//         // digitalWrite(12, raiseB);
//         // analogWrite(16, (sineB + raiseB) * 255);
//         // digitalWrite(5, raiseB);

//         // delayMicroseconds((sin(speed) + 1) * 10000);
//         delayMicroseconds(16000);
//     }
//     // speed += PI / 8;
// }