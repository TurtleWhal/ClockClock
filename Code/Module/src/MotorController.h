#include "Arduino.h"
#include "NewStepper.h"

#include "../../Master/src/motorcontrol.h"

class MotorController {
private:
  NewStepper *motor;

  MotorControl_t &control = *(new MotorControl_t()); // default control settings

  float position = 0.0f; // current position in microsteps (CW = increasing)
  float speed = 0.0f;    // signed velocity, degrees/second (+ = CW)

  // Effective profile parameters for the active move. Normally copied straight
  // from the command, but overridden when a target time is requested (see
  // applyControl). Defaults mirror MotorControl_t.
  float accel = 50.0f;           // deg/s^2
  float cruiseSpeed = 150.0f;    // deg/s
  float decelK = 0.0f;           // MICRO_STEPS_PER_DEGREE/(2*accel); precomputed per move
  float targetMicrosteps = 0.0f; // move target, normalized to [0, REV)

  // Step-timing state. lastStepTime drives measured-dt velocity integration;
  // nextStepTime is the absolute deadline the scheduler aims each step at, so
  // callback run time and dispatch latency can't accumulate into drift.
  int64_t lastStepTime = 0; // µs (esp_timer_get_time)
  int64_t nextStepTime = 0; // µs (esp_timer_get_time)

  // Signed distance (microsteps) from the current position to `target`,
  // honoring control.direction and 360° wrap-around. Positive => clockwise
  // (step(true), position++). Both `position` and `target` must already be
  // normalized to [0, REV) — the callers keep them that way, so this stays
  // fmodf-free for the hot path.
  float signedDistance(float target) {
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    float forward = target - position; // clockwise travel, in (-REV, REV)
    if (forward < 0.0f)
      forward += REV; // -> [0, REV)
    float backward = (forward == 0.0f) ? 0.0f : REV - forward; // ccw travel

    switch (control.direction) {
    case MotorDirection_t::MOTOR_CW:
      return forward; // force clockwise, even if it's the long way around
    case MotorDirection_t::MOTOR_CCW:
      return -backward; // force counter-clockwise
    default:
      return (forward <= backward) ? forward : -backward; // shortest path
    }
  }

  // Arm the timer for the next step on an ABSOLUTE timeline. The deadline is
  // referenced to when the step *should* fire (nextStepTime += period), not to
  // "now", so this callback's own run time and the esp_timer dispatch latency
  // don't accumulate into the step period. Every motor therefore holds the
  // profile's commanded average rate: no speed drift, and equal-time moves stay
  // in lockstep. The period is capped so a near-zero speed can't stall or divide
  // by zero; speed itself is never clamped (velocity stays continuous).
  inline void scheduleNext(float absSpeed) {
    constexpr float kUsecPerStepAt1Dps =
        1000000.0f * 360.0f / MICRO_STEPS_PER_REVOLUTION; // 15625 µs
    constexpr uint32_t kMaxPeriod = 20000;                // µs (~0.78 deg/s floor)
    uint32_t period = (absSpeed > 1.0f)
                          ? (uint32_t)(kUsecPerStepAt1Dps / absSpeed)
                          : kMaxPeriod;

    nextStepTime += period;
    int64_t now = esp_timer_get_time();
    int64_t wait = nextStepTime - now;
    if (wait < 1) { // fell behind (saturated/preempted) — fire ASAP and resync
      wait = 1;
      nextStepTime = now;
    }
    esp_timer_start_once(timer, (uint64_t)wait);
  }

  // The actual task — a normal member function with full access to private
  // members (motor, etc.) via the implicit `this`. `speed` is a *signed*
  // velocity (+ = CW): the step direction follows its sign, so an interrupting
  // command never causes a discontinuous reversal — a target behind the motor
  // is reached by braking through zero.
  void controlTask() {
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;

    // Measured elapsed time since the previous step. Integrating velocity
    // against the *real* interval (not the intended one) keeps the speed profile
    // locked to wall-clock even when a tick is dispatched late.
    int64_t now = esp_timer_get_time();
    float dt = (now - lastStepTime) * 0.000001f;
    if (dt > 0.05f)
      dt = 0.05f; // clamp so a long preemption stall can't jolt the speed
    lastStepTime = now;

    // Cheap wrap: position moves <= 1 microstep per tick, so one compare-subtract
    // keeps it in [0, REV) — no fmodf in the hot path.
    if (position >= REV)
      position -= REV;
    else if (position < 0.0f)
      position += REV;

    float accelStep = accel * dt; // velocity change available this tick

    // Continuous-spin mode: ramp toward control.speed in the commanded direction
    // and keep going. A later position command inherits `speed` and decelerates.
    if (control.keepRunning) {
      float targetVel = (control.direction == MotorDirection_t::MOTOR_CCW)
                            ? -(float)control.speed
                            : (float)control.speed;
      if (speed < targetVel) {
        speed += accelStep;
        if (speed > targetVel)
          speed = targetVel;
      } else if (speed > targetVel) {
        speed -= accelStep;
        if (speed < targetVel)
          speed = targetVel;
      }

      bool cw = (speed != 0.0f) ? (speed > 0.0f) : (targetVel > 0.0f);
      if (cw) {
        motor->step(true);
        position += 1.0f;
      } else {
        motor->step(false);
        position -= 1.0f;
      }
      scheduleNext(fabsf(speed));
      return;
    }

    float diff = signedDistance(targetMicrosteps); // signed microsteps to target
    float absdiff = fabsf(diff);

    // Arrived: snap exactly and hold (leave the timer stopped). The profile
    // decelerates to ~0 here, so zeroing the speed isn't a discontinuity.
    if (absdiff < 1.0f) {
      position = targetMicrosteps;
      speed = 0.0f;
      return;
    }

    float decelDist = speed * speed * decelK; // v^2/(2a), in microsteps

    if (speed * diff < 0.0f || decelDist >= absdiff)
      speed -= copysignf(accelStep, speed); // brake toward zero / land on target
    else if (fabsf(speed) < cruiseSpeed)
      speed += copysignf(accelStep, diff); // accelerate toward the cruise cap
    else if (fabsf(speed) > cruiseSpeed)
      speed -= copysignf(accelStep, speed); // ease down to the cap

    // Step in the travel direction (toward the target when momentarily at rest).
    bool cw = (speed != 0.0f) ? (speed > 0.0f) : (diff > 0.0f);
    if (cw) {
      motor->step(true);
      position += 1.0f;
    } else {
      motor->step(false);
      position -= 1.0f;
    }
    scheduleNext(fabsf(speed));
  }

  // Timer that drives the task. The callback recovers `this` and jumps into
  // the real (non-static) member function above, matching StepperMotor.h.
  const esp_timer_create_args_t timer_args = {
      .callback =
          [](void *arg) { static_cast<MotorController *>(arg)->controlTask(); },
      .arg = (void *)this, // arbitrary argument to pass to callback
      .name = "MotorControl"};

  esp_timer_handle_t timer = nullptr;

public:
  MotorController(NewStepper *motor) : motor(motor) {
    // The esp_timer is created lazily on the first applyControl(), NOT here.
    // These controllers are constructed at global/static-init time — before the
    // esp_timer service is running — so esp_timer_create() would fail and leave
    // `timer` invalid, which then crashed esp_timer_start_once() (a
    // LoadProhibited on the garbage handle). By the first applyControl(),
    // setup() has run and creation is safe.
  }

  void applyControl(const MotorControl_t &control) {
    if (timer == nullptr)
      esp_timer_create(&timer_args, &timer);

    // Capture the outgoing mode before `control` is overwritten.
    bool wasSpinning = this->control.keepRunning;

    this->control = control;

    // Leaving constant-velocity (keepRunning) mode with a SHORTEST move: don't
    // let the motor turn around. Resolve SHORTEST to keep spinning the way it's
    // already going, so it decelerates to the target in that direction (even if
    // that's the long way around the dial).
    if (wasSpinning && !control.keepRunning && speed != 0.0f &&
        control.direction == MotorDirection_t::MOTOR_SHORTEST) {
      this->control.direction = (speed > 0.0f) ? MotorDirection_t::MOTOR_CW
                                               : MotorDirection_t::MOTOR_CCW;
    }

    // Precompute the move target once, normalized to [0, REV), so the hot loop
    // and signedDistance() stay fmodf-free.
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    targetMicrosteps =
        fmodf((float)(control.position * MICRO_STEPS_PER_DEGREE), REV);
    if (targetMicrosteps < 0.0f)
      targetMicrosteps += REV;

    // Default: drive the profile straight off the commanded accel/speed. The
    // current velocity is deliberately NOT reset, so an interrupting command
    // continues from the speed the motor already has (no discontinuity).
    accel = control.acceleration;
    cruiseSpeed = control.speed;

    // If a target time is requested, derive an accelerate-then-decelerate
    // profile that starts at the *current* velocity, ends at rest on the
    // target, and takes exactly `time` ms — so a moving motor blends into the
    // new move and still stops on time, without stopping first.
    if (!control.keepRunning && control.time != UINT16_MAX && control.time > 0) {
      float sd = signedDistance(targetMicrosteps);         // signed microsteps
      float D = fabsf(sd) / (float)MICRO_STEPS_PER_DEGREE;  // distance, degrees
      float T = control.time * 0.001f;                      // time, seconds

      if (D > 0.0f) {
        // Current velocity projected onto the move direction: + = already
        // heading toward the target, - = heading away from it.
        float v0 = (sd >= 0.0f) ? speed : -speed;

        // Peak speed vp of a profile that ramps v0 -> vp (at +a) then vp -> 0
        // (at -a), covering D in time T. From t1 + t2 = T and signed area = D:
        //   2T*vp^2 - 4D*vp + v0*(2D - T*v0) = 0
        //   vp = (2D + sqrt(4D^2 - 4D*T*v0 + 2T^2*v0^2)) / (2T)
        //   a  = (2*vp - v0) / T
        float disc = 4.0f * D * D - 4.0f * D * T * v0 + 2.0f * T * T * v0 * v0;
        if (disc < 0.0f)
          disc = 0.0f; // >= 0 analytically; guard float rounding
        float vp = (2.0f * D + sqrtf(disc)) / (2.0f * T);

        if (vp >= v0) {
          // Normal case: accelerate up to vp, then brake to land on time.
          accel = (2.0f * vp - v0) / T;
          cruiseSpeed = vp;
        } else {
          // Already faster than the profile's peak (v0 > vp): hitting the time
          // would require overshooting, so just brake smoothly onto the target.
          accel = (v0 * v0) / (2.0f * D);
          cruiseSpeed = v0; // v0 > 0 in this branch
        }
      }
    }

    // Per-move deceleration constant, so the hot loop's brake test is a
    // multiply instead of a divide: decelDist = speed^2 * decelK.
    decelK = (accel > 0.0f) ? (MICRO_STEPS_PER_DEGREE / (2.0f * accel)) : 0.0f;

    // Seed the step-timing baseline to "now" so the first step fires promptly
    // and measured-dt starts clean.
    lastStepTime = nextStepTime = esp_timer_get_time();
    esp_timer_start_once(timer, 0); // start immediately
  }

  // set speed in degrees per second
  void setSpeed(float speed) { this->speed = speed; }

  // Current motor position in degrees, normalized to [0, 360).
  float getCurrentPosition() {
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    float pos = fmodf(position, REV);
    if (pos < 0.0f)
      pos += REV;
    return pos / (float)MICRO_STEPS_PER_DEGREE;
  }

  // Tell the controller where the hand physically is, in degrees. Used by
  // calibration to seed the position reference without moving the motor — call
  // it while the motor is idle so it doesn't race the stepping timer.
  void setPosition(float degrees) {
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    position = fmodf(degrees * MICRO_STEPS_PER_DEGREE, REV);
    if (position < 0.0f)
      position += REV;
  }
};
