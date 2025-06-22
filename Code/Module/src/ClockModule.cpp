#include "ClockModule.h"

ClockModule::ClockModule(int motor)
{
    auto m = motors[motor];

    hourStepper = new StepperMotor(m[0][0], m[0][1], m[0][2], m[0][3], motor == 0 ? true : false);
    minuteStepper = new StepperMotor(m[1][2], m[1][3], m[1][0], m[1][1]);

    hourStepper->setPosition(90);
    minuteStepper->setPosition(90);

    MotorControl_t hourInit;
    hourInit.position = 135;
    MotorControl_t minuteInit;
    minuteInit.position = 315;

    hourStepper->applyMotorControl(hourInit);
    minuteStepper->applyMotorControl(minuteInit);
}