#include "Arduino.h"

#define MICROSTEPS 2
#define STEPS_PER_REVOLUTION 720 // Logical steps (user-facing)
#define MICRO_STEPS_PER_REVOLUTION                                             \
  (STEPS_PER_REVOLUTION * MICROSTEPS) // Physical microsteps
#define MICRO_STEPS_PER_DEGREE                                                 \
  (MICRO_STEPS_PER_REVOLUTION / 360) // Physical microsteps

enum MagnetState : uint8_t { OFF = 0, N = 1, S = 2 };

class NewStepper {
private:
  uint8_t pin1A, pin1B, pin2A, pin2B;
  uint8_t currentStep;

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

public:
  NewStepper(uint8_t pin1A, uint8_t pin1B, uint8_t pin2A, uint8_t pin2B)
      : pin1A(pin1A), pin1B(pin1B), pin2A(pin2A), pin2B(pin2B) {

    pinMode(pin1A, OUTPUT);
    pinMode(pin1B, OUTPUT);
    pinMode(pin2A, OUTPUT);
    pinMode(pin2B, OUTPUT);
  }

  void fullstep() {
    currentStep = (currentStep + 1) % 4;

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

  void halfstep() {
    currentStep = (currentStep + 1) % 8;

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
};