#include "pins.h"
#include "pwm.h"

#define PWM_RESOLUTION 102
// #define PWM_RESOLUTION 128
// #define PWM_RESOLUTION 76
// #define PWM_RESOLUTION 96
#define NUM_PWM_PINS 16
#define PWM_TICKS (PWM_RESOLUTION + 1) // tick values 0..PWM_RESOLUTION

// Pin order here defines the duties[] index order.
static const uint8_t pinMap[NUM_PWM_PINS] = {M1_A1, M1_A3, M1_B1, M1_B3,
                                             M2_A1, M2_A3, M2_B1, M2_B3,
                                             M3_A1, M3_A3, M3_B1, M3_B3,
                                             M4_A1, M4_A3, M4_B1, M4_B3};

// Per-pin compare value, 0..PWM_RESOLUTION. Source of truth.
// Written from core 0 (esp_timer stepping, via setPWMDuty); read from core 1
// (PWM loop). A single uint8_t write is atomic on the bus, so no lock is needed.
static volatile uint8_t duties[NUM_PWM_PINS];

// Set by setPWMDuty when any duty changes; cleared by the PWM loop after it
// rebuilds its clear tables. Lets the loop skip the rebuild on idle periods.
static volatile bool dutiesDirty = true;

// Precomputed per-pin "turn off" bit masks, split by which output register the
// pin lives in: GPIO_OUT for pins < 32, GPIO_OUT1 for pins >= 32 (offset -32).
// Computed once in initPWM() from pinMap so the masks can never drift from it.
static uint32_t pinMaskLow[NUM_PWM_PINS];
static uint32_t pinMaskHigh[NUM_PWM_PINS];

// OR of every pin's mask — used to drive all coils high at the top of a period.
static uint32_t setHighLow = 0;
static uint32_t setHighHigh = 0;

// Clear tables, indexed by tick value: clearLow[t] is the OR of all pin bits
// (GPIO_OUT) that switch off at tick t, clearHigh[t] the same for GPIO_OUT1.
// Owned entirely by the PWM loop (core 1) and rebuilt from duties[] only when
// dutiesDirty is set, so the inner loop reads a fully consistent snapshot and
// never produces a torn-mask glitch from a mid-update read.
static uint32_t clearLow[PWM_TICKS];
static uint32_t clearHigh[PWM_TICKS];

// The PWM busy loop. Pinned alone on core 1 (see initPWM) and placed in IRAM so
// it never stalls on a flash-cache miss — the period stays uniform even while
// another core touches flash. Each tick does a constant two register writes
// (no per-pin branching), so every tick is the same length: maximum and jitter
// free PWM frequency for a 16-channel, two-output-register design.
void IRAM_ATTR PWMTask(void *pvParameters)
{
    while (true)
    {
        if (dutiesDirty)
        {
            // Clear the flag *before* reading duties[]: if an update lands
            // during the rebuild it re-sets the flag and we pick it up next
            // period — no update is ever lost, worst case one period stale.
            dutiesDirty = false;

            for (uint8_t t = 0; t < PWM_TICKS; t++)
            {
                clearLow[t] = 0;
                clearHigh[t] = 0;
            }
            for (uint8_t i = 0; i < NUM_PWM_PINS; i++)
            {
                uint8_t d = duties[i];
                clearLow[d] |= pinMaskLow[i];
                clearHigh[d] |= pinMaskHigh[i];
            }
        }

        // Start of period: drive every coil pin high.
        REG_WRITE(GPIO_OUT_W1TS_REG, setHighLow);
        REG_WRITE(GPIO_OUT1_W1TS_REG, setHighHigh);

        // Walk the ticks. Exactly two stores per tick (a zero mask clears
        // nothing), so the loop is branch-free and every tick is identical.
        for (uint8_t t = 0; t < PWM_TICKS; t++)
        {
            REG_WRITE(GPIO_OUT_W1TC_REG, clearLow[t]);
            REG_WRITE(GPIO_OUT1_W1TC_REG, clearHigh[t]);
        }
    }
}

void initPWM()
{
    // Build the per-pin masks and the all-pins-high masks from pinMap so the
    // bit sets always match the actual pin assignments.
    for (uint8_t i = 0; i < NUM_PWM_PINS; i++)
    {
        duties[i] = 0;
        uint8_t p = pinMap[i];
        if (p < 32)
        {
            pinMaskLow[i] = 1u << p;
            pinMaskHigh[i] = 0;
            setHighLow |= 1u << p;
        }
        else
        {
            pinMaskLow[i] = 0;
            pinMaskHigh[i] = 1u << (p - 32);
            setHighHigh |= 1u << (p - 32);
        }
    }
    dutiesDirty = true;

    // The PWM busy loop must own a core by itself: any preemption stretches the
    // PWM period and makes the motors whine/scratch. Core 0 hosts the esp_timer
    // stepping task (~80 kHz of callbacks while moving), so PWM lives on core 1,
    // and the serial loop is moved off core 1 to core 0 (see Module main.cpp).
    //
    // Priority MUST stay at 1 — the same priority as the Arduino loop task that
    // runs setup() on core 1. This loop never yields, so a *higher* priority
    // would preempt setup() the instant this task is created (inside initPWM)
    // and the rest of setup() — ClockModule construction, serialTask creation —
    // would never run, leaving the motors energized but never stepped. Equal
    // priority lets setup() finish first; core 1 is dedicated at runtime anyway
    // (the loop task parks on vTaskDelay), so PWM gets the whole core regardless.
    //
    // This loop never yields, so it starves core 1's idle task; setup()
    // reconfigures the Task WDT to stop watching idle tasks, or this triggers a
    // watchdog panic. (INT_WDT and esp_timer stepping are unaffected.)
    xTaskCreatePinnedToCore(
        PWMTask,
        "PWMTask",
        4096,
        NULL,
        1, // MUST equal the setup/loop task priority — see note above
        NULL,
        1); // core 1 — alone, away from serial and esp_timer stepping (both core 0)
}

void setPWMDuty(uint8_t pin, uint16_t duty)
{
    for (uint8_t i = 0; i < NUM_PWM_PINS; i++)
    {
        if (pinMap[i] == pin)
        {
            duties[i] = map(duty, 0, UINT16_MAX, 0, PWM_RESOLUTION);
            dutiesDirty = true;
            return;
        }
    }
}
