#include "StepperMotor.h"
#include "pins.h"

class ClockModule
{
private:
    int motors[4][2][4] = {
        {{M1_A1, M1_A2, M1_A3, M1_A4}, {M1_B1, M1_B2, M1_B3, M1_B4}},
        {{M2_A1, M2_A2, M2_A3, M2_A4}, {M2_B1, M2_B2, M2_B3, M2_B4}},
        {{M3_A1, M3_A2, M3_A3, M3_A4}, {M3_B1, M3_B2, M3_B3, M3_B4}},
        {{M4_A1, M4_A2, M4_A3, M4_A4}, {M4_B1, M4_B2, M4_B3, M4_B4}},
    };

public:
    ClockModule(int motor);
    StepperMotor *hourStepper;
    StepperMotor *minuteStepper;
};