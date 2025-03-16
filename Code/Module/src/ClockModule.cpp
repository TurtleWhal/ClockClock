#include "ClockModule.h"

ClockModule::ClockModule(int motor)
{
    auto m = motors[motor];
    hourStepper = new StepperMotor(m[0][0], m[0][1], m[0][2], m[0][3]);
    minuteStepper = new StepperMotor(m[1][2], m[1][3], m[1][0], m[1][1]);

    hourStepper->setCurrentPosition(90 * 2);
    minuteStepper->setCurrentPosition(90 * 2);

    hourStepper->setTargetPosition(135 * 2);
    minuteStepper->setTargetPosition(315 * 2);
}