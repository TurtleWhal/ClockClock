#include "Arduino.h"
#include "hal/ledc_ll.h" // direct duty-register writes for the stepping hot path
#include "ledcfast.h"

// The Module's "B" hands drive their two magnitude coils through LEDC (8-bit,
// 40 kHz). analogWrite()/ledcWrite() per microstep is heavy: a pin->channel
// lookup, argument checks, a fade-hw semaphore and two driver critical sections.
// This mirrors the MCPWM fast path — assign each pin a known channel up front,
// then per microstep write the duty register and latch it directly, no lookup or
// lock (safe: the only caller is the single core-1 stepping loop). The register
// sequence is exactly what ledc_set_duty()+ledc_update_duty() do for a static,
// non-fading duty on the S3 (no gamma path), minus the driver overhead.

static int8_t pinToCh[64];

void ledcFastInit(const uint8_t *pins, uint8_t count) {
  for (int i = 0; i < 64; i++)
    pinToCh[i] = -1;

  ledc_dev_t *hw = LEDC_LL_GET_HW();
  for (uint8_t i = 0; i < count && i < SOC_LEDC_CHANNEL_NUM; i++) {
    // Own the channel assignment (channel == i) so the hot path knows it with no
    // lookup. 8-bit @ 40 kHz matches the coils' previous analogWrite setup.
    ledcAttachChannel(pins[i], 40000, 8, i);
    if (pins[i] < 64)
      pinToCh[pins[i]] = (int8_t)i;
    // sig_out_en stays true for the channel's life; set once so the hot path
    // only touches the duty registers.
    ledc_ll_set_sig_out_en(hw, LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, true);
  }
}

// Set a coil's duty (0..255) by GPIO number — stepping hot path. Replicates
// ledc_set_duty()+ledc_update_duty() for a static (no-fade) duty via the LL,
// minus the driver's checks, semaphore, lookups and critical sections.
void ledcFastWrite(uint8_t pin, uint8_t duty) {
  int8_t ch = (pin < 64) ? pinToCh[pin] : (int8_t)-1;
  if (ch < 0)
    return;
  ledc_dev_t *hw = LEDC_LL_GET_HW();
  uint32_t d = (duty == 255) ? 256u : duty; // ledcWrite's 8-bit "full on" fix
  ledc_ll_set_duty_int_part(hw, LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch,
                            d); // register := d << 4
  ledc_ll_set_fade_param(hw, LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, 1, 1, 0,
                         1); // dir/cycle/step with scale 0 => hold (no fade)
  ledc_ll_set_duty_start(hw, LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
  ledc_ll_ls_channel_update(hw, LEDC_LOW_SPEED_MODE,
                            (ledc_channel_t)ch); // latch shadow -> active
}
