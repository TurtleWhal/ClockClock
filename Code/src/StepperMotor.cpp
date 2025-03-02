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

    analogWriteFrequency(20000);
}

void StepperMotor::handle()
{
    unsigned long currentTime = micros();

    if (_running)
    {
        if (targetTime < currentTime)
        {
            unsigned long elapsedTime = currentTime - lastTime;

            double _distance = _targetPosition - _currentPosition;

            // Calculate the remaining deceleration distance based on current speed
            double _decelerationDistance = (_speed * _speed) / (2 * _acceleration);

            // Determine the direction of movement (1 = forward, -1 = backward)
            int direction = (_distance > 0) ? 1 : -1;
            
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

            // Half stepping
            // Update motor position based on direction
            _currentPosition += direction * (1.0 / _microsteps);

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
                targetTime = currentTime + (1000000.0 / (_speed * _microsteps));

            // Serial.printf("Timer: %ld, _speed: %f, _targetPosition: %d, _currentPosition: %f\n", targetTime - currentTime, _speed, _targetPosition, _currentPosition);
            
            // Stop the motor when the target position is reached
            if ((direction > 0 && _currentPosition >= _targetPosition) ||
                (direction < 0 && _currentPosition <= _targetPosition))
            {
                _running = false;
                _speed = 0;
            }

            lastTime = currentTime;
        }
    }
}

void StepperMotor::writeMagnet(int p1, int p2, double state)
{
    if (state > 0)
    {
        analogWrite(p1, state * 255);
        digitalWrite(p2, LOW);
    }
    else if (state < 0)
    {
        digitalWrite(p1, LOW);
        analogWrite(p2, state * 255);
    }
    else
    {
        digitalWrite(p1, LOW);
        digitalWrite(p2, LOW);
    }
}

void StepperMotor::setTargetPosition(int position)
{
    _running = false;
    _speed = 0;
    // lastTime = micros();
    // lastTime = 0;
    // targetTime = lastTime;
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
    _maxspeed = speed;
}

void StepperMotor::setAcceleration(int acceleration)
{
    _acceleration = acceleration;
}
