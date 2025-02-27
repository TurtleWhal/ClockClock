#include "ClockModule.h"

ClockModule::ClockModule(int motor)
{
    auto m = motors[motor];
    minuteStepper = new StepperMotor(m[0][0], m[0][1], m[0][2], m[0][3]);
    hourStepper = new StepperMotor(m[1][2], m[1][3], m[1][1], m[1][4]);
}