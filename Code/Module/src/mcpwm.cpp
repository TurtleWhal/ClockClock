#include "Arduino.h"
#include "driver/mcpwm.h" // legacy driver: init/config only (not the hot path)
#include "hal/mcpwm_ll.h" // direct compare-register writes for mcpwmWrite
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

// Hot-path lookup tables, all built once in mcpwmInit:
//  - chDev/chOp/chCmp: the LL target for each channel (peripheral pointer,
//    operator id, comparator id) so mcpwmWrite writes the compare register
//    directly instead of going through mcpwm_set_duty (float math + a driver
//    critical section + set_duty_type). The legacy driver maps operator==timer
//    and comparator==generator (see mcpwm_legacy.c mcpwm_set_duty).
//  - pinToCh: GPIO number -> channel index (or -1), replacing the old per-call
//    linear search with a single array index.
//  - compareTicks: duty (0..255) -> compare ticks. Exactly mcpwm_set_duty's
//    formula (peak * duty% / 100 == peak * duty / 255), precomputed so the hot
//    path has no multiply or divide.
static mcpwm_dev_t *chDev[8];
static int chOp[8];
static int chCmp[8];
static int8_t pinToCh[64];
static uint32_t compareTicks[256];

void mcpwmInit(const uint8_t *pin, uint8_t count) {
  for (int i = 0; i < 64; i++)
    pinToCh[i] = -1;

  for (int i = 0; i < count && i < 8; i++)
    mcpwm_gpio_init(ch[i].u,
                    (mcpwm_io_signals_t)(MCPWM0A + ch[i].t * 2 + ch[i].g),
                    pin[i]);

  mcpwm_config_t c = {.frequency = 40000,
                      .cmpr_a = 0,
                      .cmpr_b = 0,
                      .duty_mode = MCPWM_DUTY_MODE_0,
                      .counter_mode = MCPWM_UP_COUNTER};
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &c);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &c);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &c);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &c);

  // Cache each channel's LL target and build the pin lookup. mcpwm_init already
  // set MCPWM_DUTY_MODE_0 (the generator action) and enabled compare-update-on-
  // TEZ via its cmpr=0 set_duty, but re-enable the update explicitly so the
  // direct writes below latch exactly like mcpwm_set_duty, independent of init
  // internals.
  for (int i = 0; i < count && i < 8; i++) {
    chDev[i] = MCPWM_LL_GET_HW(ch[i].u);
    chOp[i] = (int)ch[i].t;
    chCmp[i] = (int)ch[i].g;
    mcpwm_ll_operator_enable_update_compare_on_tez(chDev[i], chOp[i], chCmp[i],
                                                   true);
    mcpwm_ll_operator_enable_update_compare_on_tep(chDev[i], chOp[i], chCmp[i],
                                                   true);
    if (pin[i] < 64)
      pinToCh[pin[i]] = (int8_t)i;
  }

  // All four timers share the 40 kHz config, so one duty->ticks table serves
  // every channel. (peak * duty) fits in uint32_t: peak is a few thousand ticks.
  uint32_t peak = mcpwm_ll_timer_get_peak(MCPWM_LL_GET_HW(MCPWM_UNIT_0),
                                          (int)MCPWM_TIMER_0, false);
  for (int d = 0; d < 256; d++)
    compareTicks[d] = (uint32_t)peak * (uint32_t)d / 255u;
}

// Set a coil's duty (0..255) by GPIO number — the stepping hot path. O(1) pin
// lookup, no float, no driver critical section: one compare-register field write
// that latches at the next period boundary (update-on-TEZ enabled in init).
// Safe without the driver lock because the only caller is the single core-1
// stepping loop; nothing else touches these registers after init.
void mcpwmWrite(uint8_t pin, uint8_t duty) {
  int8_t i = (pin < 64) ? pinToCh[pin] : (int8_t)-1;
  if (i < 0)
    return;
  mcpwm_ll_operator_set_compare_value(chDev[i], chOp[i], chCmp[i],
                                      compareTicks[duty]);
}
