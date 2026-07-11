#include "Arduino.h"
#include "mcpwm.h"
#include "pwm.h"

#define MICROSTEPS 32
#define STEPS_PER_REVOLUTION 720 // Logical steps (user-facing)
#define MICRO_STEPS_PER_REVOLUTION                                             \
  (STEPS_PER_REVOLUTION * MICROSTEPS) // Physical microsteps
#define MICRO_STEPS_PER_DEGREE                                                 \
  (MICRO_STEPS_PER_REVOLUTION / 360) // Physical microsteps

enum MagnetState : uint8_t { OFF = 0, N = 1, S = 2 };

enum ControlMethod : uint8_t {
  FULL_STEP = 0,
  HALF_STEP = 1,
  MICRO_STEP_LEDC = 2,
  MICRO_STEP_MCPWM = 3,
  MICRO_STEP_SWPWM = 4
};

class NewStepper {
private:
  uint8_t pin1A, pin1B, pin2A, pin2B;
  uint8_t method;

  uint8_t currentStep;

  uint8_t sinTable[MICROSTEPS * 4];

  void writeMagnet(uint8_t magnet, uint8_t value) {

    uint8_t pinA = (magnet == 1) ? pin1A : pin2A;
    uint8_t pinB = (magnet == 1) ? pin1B : pin2B;

    switch (value) {
    case MagnetState::OFF:
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, LOW);
      break;
    case MagnetState::N:
      digitalWrite(pinA, HIGH);
      digitalWrite(pinB, LOW);
      break;
    case MagnetState::S:
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, HIGH);
      break;
    }
  }

  void writeMagnetAnalog(uint8_t magnet, float value) {

    uint8_t pinA = (magnet == 1) ? pin1A : pin2A;
    uint8_t pinB = (magnet == 1) ? pin1B : pin2B;

    // 155 is ~60% of the max power so to stop hard power difference near 100% duty cycle
    if (value > 0) {
      analogWrite(pinA, (uint8_t)(value * 155));
      digitalWrite(pinB, LOW);
    } else if (value < 0) {
      analogWrite(pinB, 255 - (uint8_t)(value * 155));
      digitalWrite(pinA, HIGH);
    } else {
      digitalWrite(pinA, LOW);
      digitalWrite(pinB, LOW);
    }
  }

  void writeMicrostep(uint8_t step) {
    uint8_t step90 =
        (step + MICROSTEPS) % (MICROSTEPS * 4); // 90 degrees offset

    if (method == MICRO_STEP_LEDC) {
      analogWrite(pin1A, sinTable[step]);
      analogWrite(pin2A, sinTable[step90]);
    } else if (method == MICRO_STEP_MCPWM) {
      mcpwmWrite(pin1A, sinTable[step]);
      mcpwmWrite(pin2A, sinTable[step90]);
    } else if (method == MICRO_STEP_SWPWM) {
      setPWMDuty(pin1A, sinTable[step]);
      setPWMDuty(pin2A, sinTable[step90]);
    }

    digitalWrite(pin1B, step <= (MICROSTEPS * 2) ? LOW : HIGH);
    digitalWrite(pin2B, step90 <= (MICROSTEPS * 2) ? LOW : HIGH);
  }

public:
  NewStepper(uint8_t pin1A, uint8_t pin1B, uint8_t pin2A, uint8_t pin2B,
             uint8_t method)
      : pin1A(pin1A), pin1B(pin1B), pin2A(pin2A), pin2B(pin2B), method(method) {

    pinMode(pin1A, OUTPUT);
    pinMode(pin1B, OUTPUT);
    pinMode(pin2A, OUTPUT);
    pinMode(pin2B, OUTPUT);

    if (method == MICRO_STEP_LEDC) {
      analogWriteFrequency(pin1A, 40000);
      analogWriteFrequency(pin2A, 40000);
    }

    for (uint8_t i = 0; i < MICROSTEPS * 4; i++) {
      float rad = ((i % (MICROSTEPS * 4)) * (PI * 2)) / (MICROSTEPS * 4.0f);
      float sinval = sinf(rad);

      sinTable[i] = sinval >= 0 ? (sinval * 255) : 255 - (-sinval * 255);
    }
  }

  // MICROSTEPS should be 1
  void fullstep(bool clockwise = true) {
    currentStep = clockwise ? (currentStep + 1) % 4 : (currentStep - 1 + 4) % 4;

    switch (currentStep) {
    case 0:
      writeMagnet(1, MagnetState::N);
      writeMagnet(2, MagnetState::OFF);
      break;
    case 1:
      writeMagnet(1, MagnetState::OFF);
      writeMagnet(2, MagnetState::N);
      break;
    case 2:
      writeMagnet(1, MagnetState::S);
      writeMagnet(2, MagnetState::OFF);
      break;
    case 3:
      writeMagnet(1, MagnetState::OFF);
      writeMagnet(2, MagnetState::S);
      break;
    }
  }

  // MICROSTEPS should be 2
  void halfstep(bool clockwise = true) {
    currentStep = clockwise ? (currentStep + 1) % 8 : (currentStep - 1 + 8) % 8;

    switch (currentStep) {
    case 0:
      writeMagnet(1, MagnetState::N);
      writeMagnet(2, MagnetState::OFF);
      break;
    case 1:
      writeMagnet(1, MagnetState::N);
      writeMagnet(2, MagnetState::N);
      break;
    case 2:
      writeMagnet(1, MagnetState::OFF);
      writeMagnet(2, MagnetState::N);
      break;
    case 3:
      writeMagnet(1, MagnetState::S);
      writeMagnet(2, MagnetState::N);
      break;
    case 4:
      writeMagnet(1, MagnetState::S);
      writeMagnet(2, MagnetState::OFF);
      break;
    case 5:
      writeMagnet(1, MagnetState::S);
      writeMagnet(2, MagnetState::S);
      break;
    case 6:
      writeMagnet(1, MagnetState::OFF);
      writeMagnet(2, MagnetState::S);
      break;
    case 7:
      writeMagnet(1, MagnetState::N);
      writeMagnet(2, MagnetState::S);
      break;
    }
  }

  void slowmicrostep(bool clockwise = true) {
    currentStep = clockwise ? (currentStep + 1) % (4 * MICROSTEPS) : (currentStep - 1 + (4 * MICROSTEPS)) % (4 * MICROSTEPS);

    writeMagnetAnalog(1, sinf((currentStep * 2 * PI) / (4 * MICROSTEPS)));
    writeMagnetAnalog(2, cosf((currentStep * 2 * PI) / (4 * MICROSTEPS)));
  }

  void microstep(bool clockwise = true) {
    currentStep = clockwise ? (currentStep + 1) % (4 * MICROSTEPS) : (currentStep - 1 + (4 * MICROSTEPS)) % (4 * MICROSTEPS);

    writeMicrostep(currentStep);
  }

  void step(bool clockwise = true) {
    if (method == FULL_STEP) {
      fullstep(clockwise);
    } else if (method == HALF_STEP) {
      halfstep(clockwise);
    } else if (method == MICRO_STEP_LEDC || method == MICRO_STEP_MCPWM ||
               method == MICRO_STEP_SWPWM) {
      microstep(clockwise);
    }
  }
};