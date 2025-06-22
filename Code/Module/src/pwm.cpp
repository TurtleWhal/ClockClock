#include "pins.h"
#include "pwm.h"

#define PWM_RESOLUTION 102

uint8_t duties[16];

const uint8_t pinMap[16] = {M1_A1, M1_A3, M1_B1, M1_B3,
                            M2_A1, M2_A3, M2_B1, M2_B3,
                            M3_A1, M3_A3, M3_B1, M3_B3,
                            M4_A1, M4_A3, M4_B1, M4_B3};

#define gpio_out_mask (1 << M1_A1) |     \
                          (1 << M1_A3) | \
                          (1 << M1_B1) | \
                          (1 << M1_B3) | \
                          (1 << M2_A1) | \
                          (1 << M2_A3) | \
                          (1 << M2_B3) | \
                          (1 << M3_A1) | \
                          (1 << M3_A3)

#define gpio_out1_mask (1 << (M2_B1 - 32)) |     \
                           (1 << (M3_B1 - 32)) | \
                           (1 << (M3_B3 - 32)) | \
                           (1 << (M4_A1 - 32)) | \
                           (1 << (M4_A3 - 32)) | \
                           (1 << (M4_B1 - 32)) | \
                           (1 << (M4_B3 - 32))

void PWMTask(void *pvParameters)
{

    uint8_t loop = 0;

    while (true)
    {
        if (loop++ == PWM_RESOLUTION)
        {
            REG_WRITE(GPIO_OUT_W1TS_REG, gpio_out_mask);
            REG_WRITE(GPIO_OUT1_W1TS_REG, gpio_out1_mask);

            loop = 0;
        }

        if (loop == duties[0])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[0]);
        if (loop == duties[1])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[1]);
        if (loop == duties[2])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[2]);
        if (loop == duties[3])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[3]);
        if (loop == duties[4])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[4]);
        if (loop == duties[5])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[5]);
        if (loop == duties[6])
            REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (pinMap[6] - 32));
        if (loop == duties[7])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[7]);
        if (loop == duties[8])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[8]);
        if (loop == duties[9])
            REG_WRITE(GPIO_OUT_W1TC_REG, 1 << pinMap[9]);
        if (loop == duties[10])
            REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (pinMap[10] - 32));
        if (loop == duties[11])
            REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (pinMap[11] - 32));
        if (loop == duties[12])
            REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (pinMap[12] - 32));
        if (loop == duties[13])
            REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (pinMap[13] - 32));
        if (loop == duties[14])
            REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (pinMap[14] - 32));
        if (loop == duties[15])
            REG_WRITE(GPIO_OUT1_W1TC_REG, 1 << (pinMap[15] - 32));
    }
}

void initPWM()
{
    xTaskCreatePinnedToCore(
        PWMTask,
        "PWMTask",
        4096,
        NULL,
        1,
        NULL,
        1); // Run on core 0
}

void setPWMDuty(uint8_t pin, uint16_t duty)
{
    for (uint8_t i = 0; i < 16; i++)
    {
        if (pinMap[i] == pin)
        {
            duties[i] = map(duty, 0, UINT16_MAX, 0, PWM_RESOLUTION);
            return;
        }
    }
}