#include "Arduino.h"

// Fast LEDC duty writes for the stepping hot path — the LEDC analogue of
// mcpwm.h. Call ledcFastInit() once (from setup, after the steppers are
// constructed) with the coil pins that use MICRO_STEP_LEDC; it owns their LEDC
// channel assignment. Then ledcFastWrite() sets a coil's duty by GPIO number.
void ledcFastInit(const uint8_t *pins, uint8_t count);
void ledcFastWrite(uint8_t pin, uint8_t duty);
