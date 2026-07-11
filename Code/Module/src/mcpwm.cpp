#include "Arduino.h"
#include "driver/mcpwm.h" // legacy driver, still shipped in IDF 5.x / Arduino core 3.x
#include "mcpwm.h"

// 8 "extra" coils -> MCPWM. 2 units x 3 timers x 2 gens = 12 outputs; we use 8.
struct Ch {
  mcpwm_unit_t u;
  mcpwm_timer_t t;
  mcpwm_generator_t g;
};
static const Ch ch[8] = {
    {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A},
    {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B},
    {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A},
    {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B},
    {MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A},
    {MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B},
    {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_A},
    {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_GEN_B},
};

// GPIO bound to each channel, index-aligned with ch[]. Filled by mcpwmInit so
// mcpwmWrite can be called with a real pin number (like analogWrite/setPWMDuty)
// instead of a channel index. 0xFF = unassigned.
static uint8_t mcpwmPins[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void mcpwmInit(const uint8_t *pin, uint8_t count) {
  for (int i = 0; i < count && i < 8; i++) {
    mcpwmPins[i] = pin[i];
    mcpwm_gpio_init(
        ch[i].u, (mcpwm_io_signals_t)(MCPWM0A + ch[i].t * 2 + ch[i].g), pin[i]);
  }
  mcpwm_config_t c = {.frequency = 40000,
                      .cmpr_a = 0,
                      .cmpr_b = 0,
                      .duty_mode = MCPWM_DUTY_MODE_0,
                      .counter_mode = MCPWM_UP_COUNTER};
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &c);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &c);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &c);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &c);
}

// Set a coil's duty (0..255) by its GPIO number. Looks up the channel the pin
// was bound to in mcpwmInit; silently ignores pins that aren't MCPWM channels.
void mcpwmWrite(uint8_t pin, uint8_t duty) {
  for (int i = 0; i < 8; i++) {
    if (mcpwmPins[i] == pin) {
      mcpwm_set_duty(ch[i].u, ch[i].t, ch[i].g, duty * (100.0f / 255.0f));
      mcpwm_set_duty_type(ch[i].u, ch[i].t, ch[i].g, MCPWM_DUTY_MODE_0);
      return;
    }
  }
}
