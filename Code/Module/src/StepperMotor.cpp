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

    // analogWriteFrequency(_pin1, 20000);
    // analogWriteFrequency(_pin2, 20000);
    // analogWriteFrequency(_pin3, 20000);
    // analogWriteFrequency(_pin4, 20000);

    // analogWriteResolution(_pin1, 16);
    // analogWriteResolution(_pin2, 16);
    // analogWriteResolution(_pin3, 16);
    // analogWriteResolution(_pin4, 16);
}

void StepperMotor::handle()
{
    unsigned long currentTime = micros();

    if (_running)
    {
        unsigned long elapsedTime = currentTime - lastTime;

        if (_speedMode)
        {
            _targetPosition = fmod(_currentPosition + (double)_targetSpeed * (elapsedTime / 1000000), 720.0);
        }

        // if (targetTime < currentTime)
        if (currentTime - lastTime > targetTime)
        {
            int direction;

            //     direction = (_targetSpeed > 0) ? 1 : -1;

            //     if (_speed < abs(_targetSpeed))
            //     {
            //         _speed += _acceleration * elapsedTime / 1000000;
            //         // _speed += 0.1;
            //     }
            //     else if (_speed > abs(_targetSpeed))
            //     {
            //         _speed -= _acceleration * elapsedTime / 1000000;
            //         // _speed -= 0.1;
            //     }

            //     if (abs(_speed) > _maxspeed)
            //     {
            //         _speed = _maxspeed;
            //     }

            //     // _speed = abs(_targetSpeed);
            // }
            // else
            // {
            double _distance = _targetPosition - _currentPosition;

            // Calculate the remaining deceleration distance based on current speed
            double _decelerationDistance = (_speed * _speed) / (2 * _acceleration);

            // Determine the direction of movement (1 = forward, -1 = backward)
            direction = (_distance > 0) ? 1 : -1;

            // double _rawDelta = _targetPosition - _currentPosition;
            // double _distance = fmod(_rawDelta + 1080.0, 720.0);
            // if (_distance > 360.0)
            //     _distance -= 720.0;

            // // Calculate the remaining deceleration distance
            // double _decelerationDistance = (_speed * _speed) / (2 * _acceleration);

            // // Determine the direction of movement (1 = forward, -1 = backward)
            // int direction = (_distance > 0) ? 1 : -1;

            // Adjust logic for deceleration and speed based on distance and direction
            if (abs(_distance) <= _decelerationDistance)
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
            // }

            if (_microstep)
            {
                // Update motor position based on direction
                _currentPosition = fmod(_currentPosition + (direction * (1.0 / _microsteps)), 720.0);

                // Analog microstepping logic with direction control
                double sineA = sin(remainder(_currentPosition / 4, 1) * (2 * PI));
                uint8_t raiseA = sineA <= 0 ? 1 : 0;
                analogWrite(_pin1, (sineA + raiseA) * 255);
                digitalWrite(_pin2, raiseA);

                double sineB = sin((remainder(_currentPosition / 4, 1) * (2 * PI)) + (PI / 2));
                uint8_t raiseB = sineB <= 0 ? 1 : 0;
                analogWrite(_pin3, (sineB + raiseB) * 255);
                digitalWrite(_pin4, raiseB);

                // Calculate the next step's target time based on the current speed and direction
                if (_speed > 0)
                    targetTime = (1000000.0 / (_speed * _microsteps));
                // targetTime = currentTime + (1000000.0 / (_speed * _microsteps));
            }
            else
            {
                // Update motor position based on direction
                _currentPosition = fmod(_currentPosition + (direction * (1.0 / 2)), 720.0);

                _phase = int(_currentPosition * 2) % 8;

                writeMagnet(_pin1, _pin2, _phases[_phase][0]);
                writeMagnet(_pin3, _pin4, _phases[_phase][1]);

                // Calculate the next step's target time based on the current speed and direction
                if (_speed > 0)
                    targetTime = (1000000.0 / (_speed * 2));
                // targetTime = currentTime + (1000000.0 / (_speed * 2));
            }

            // Serial.printf("Timer: %ld, _speed: %f, _targetPosition: %d, _currentPosition: %f\n", targetTime - currentTime, _speed, _targetPosition, _currentPosition);

            // Stop the motor when the target position is reached
            if (((direction > 0 && _currentPosition >= _targetPosition) ||
                 (direction < 0 && _currentPosition <= _targetPosition)) &&
                !_speedMode)
            {
                _running = false;
                _speed = 0;
            }

            lastTime = currentTime;
        }
    }
}

void StepperMotor::writeMagnet(int p1, int p2, int state)
{
    if (state > 0)
    {
        digitalWrite(p1, state);
        digitalWrite(p2, LOW);
    }
    else if (state < 0)
    {
        digitalWrite(p1, LOW);
        digitalWrite(p2, state);
    }
    else
    {
        digitalWrite(p1, LOW);
        digitalWrite(p2, LOW);
    }
}

void StepperMotor::setTargetPosition(int position)
{
    _speedMode = false;

    if (position == _targetPosition)
        return;

    // _speed = 0;
    _targetPosition = position;
    _running = true;
}

// void StepperMotor::setTargetPosition(int position)
// {
//     _speedMode = false;

//     double delta = fmod((position - _currentPosition + 1080.0), 720.0) - 360.0;
//     if (delta == 0)
//         return;

//     _targetPosition = _currentPosition + delta;
//     _running = true;
// }

void StepperMotor::setTargetSpeed(int speed)
{
    _speedMode = true;

    // if (speed == _targetSpeed)
    //     return;

    _targetPosition = _currentPosition;

    // _speed = 0;
    _targetSpeed = speed;
    _running = true;
}

void StepperMotor::setCurrentPosition(int position)
{
    _currentPosition = position;
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
    _maxspeed = speed;
}

void StepperMotor::setAcceleration(int acceleration)
{
    _acceleration = acceleration;
}
