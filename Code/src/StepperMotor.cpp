#include "Arduino.h"
#include "StepperMotor.h"

// Define the static member
StepperMotor* StepperMotor::_instance = nullptr;

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

    // xTaskCreate(
    //     StepperMotor::stepperTask, // Correct function pointer
    //     "StepperTask",
    //     2048,
    //     this, // Pass the current object as the parameter
    //     1,
    //     NULL);

    _instance = this;
    _stepperTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(_stepperTimer, &taskWrapper, true);
    timerAlarmWrite(_stepperTimer, 1000, true);
    timerAlarmEnable(_stepperTimer);
}

void IRAM_ATTR StepperMotor::taskWrapper() {
    _instance->stepperTask();
}

void StepperMotor::stepperTask()
{
    int _distance = (_targetPosition * _microsteps) - _currentPosition;
    if (_distance > 0)
    {
        if (_speed < _maxspeed)
        {
            _speed += _acceleration;
        }

        _phase = _currentPosition % 8;
        _currentPosition += 1;

        writeMagnet(_pin1, _pin2, _phases[_phase][0]);
        writeMagnet(_pin3, _pin4, _phases[_phase][1]);

        // Add a delay to control the speed
        // Serial.println(_speed);
        // _speed = 1;
        // delayMicroseconds(1000000 / _speed);
        // delayMicroseconds(1000);
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

// void StepperMotor::stepperTask(void *_param)
// {
//     StepperMotor *motor = static_cast<StepperMotor *>(_param); // Cast the parameter to StepperMotor*

//     while (true)
//     {

//         int _distance = (motor->_targetPosition * 2) - motor->_currentPosition;
//         if (_distance > 0)
//         {
//             if (motor->_speed < motor->_maxspeed)
//             {
//                 motor->_speed += motor->_acceleration;
//             }

//             motor->_phase = motor->_currentPosition % 8;
//             motor->_currentPosition += 1;

//             motor->writeMagnet(motor->_pin1, motor->_pin2, motor->_phases[motor->_phase][0]);
//             motor->writeMagnet(motor->_pin3, motor->_pin4, motor->_phases[motor->_phase][1]);

//             // Add a delay to control the speed
//             // Serial.println(motor->_speed);
//             motor->_speed = 1;
//             delayMicroseconds(1000000 / motor->_speed);
//             // delayMicroseconds(1000);
//         }
//         else
//         {
//             delay(100);
//         }

//         // delay(100);
//         // Serial.println("StepperTask");
//     }
// }

void StepperMotor::setTargetPosition(int position)
{
    _targetPosition = position;
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