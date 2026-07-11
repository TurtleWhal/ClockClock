#include "Arduino.h"
#include "NewStepper.h"

#include "../../Master/src/motorcontrol.h"

class MotorController {
private:
  NewStepper *motor;

  MotorControl_t &control = *(new MotorControl_t()); // default control settings

  float position = 0.0f; // current position in microsteps (CW = increasing)
  float speed = 0.0f;    // signed velocity, degrees/second (+ = CW)

  uint64_t lastdelay = 1000; // last delay in microseconds

  // Effective profile parameters for the active move. Normally copied straight
  // from the command, but overridden when a target time is requested (see
  // applyControl). Defaults mirror MotorControl_t.
  float accel = 50.0f;        // deg/s^2
  float cruiseSpeed = 150.0f; // deg/s

  // Signed distance (microsteps) from the current position to `target`,
  // honoring control.direction and 360° wrap-around. Positive => clockwise
  // (step(true), position++). Read-only, so it's safe to call before position
  // has been normalized.
  float signedDistance(float target) {
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    float pos = fmodf(position, REV);
    if (pos < 0.0f)
      pos += REV;

    // Distance to the target going each way around the circle, both as
    // non-negative magnitudes in [0, REV).
    float forward = fmodf(target - pos, REV); // clockwise travel
    if (forward < 0.0f)
      forward += REV;
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

  // The actual task — a normal member function with full access to private
  // members (motor, etc.) via the implicit `this`. Re-arms the timer each call
  // so it fires again after one step interval. `speed` is a *signed* velocity
  // (+ = CW), so an interrupting command never causes a discontinuous reversal:
  // the step direction follows sign(speed), and a target behind the motor is
  // reached by braking through zero, not by flipping the step direction.
  void controlTask() {
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;

    // Keep position within one revolution so the wrap-around math stays exact
    // and the float doesn't drift unbounded. CW = increasing microsteps.
    position = fmodf(position, REV);
    if (position < 0.0f)
      position += REV;

    // Continuous-spin mode: accelerate toward control.speed in the commanded
    // direction and keep going — there's no target position. A later position
    // command (keepRunning=false) inherits the current speed and decelerates in.
    if (control.keepRunning) {
      float dt = lastdelay * 0.000001f;
      float targetVel = (control.direction == MotorDirection_t::MOTOR_CCW)
                            ? -(float)control.speed
                            : (float)control.speed;

      if (speed < targetVel) {
        speed += accel * dt;
        if (speed > targetVel)
          speed = targetVel;
      } else if (speed > targetVel) {
        speed -= accel * dt;
        if (speed < targetVel)
          speed = targetVel;
      }

      // Keep moving even from a standstill (also avoids a divide-by-zero delay).
      const float MIN_SPEED = 2.0f; // deg/s
      if (fabsf(speed) < MIN_SPEED)
        speed = copysignf(MIN_SPEED, targetVel);

      if (speed > 0.0f) {
        motor->step(true);
        position += 1;
      } else {
        motor->step(false);
        position -= 1;
      }

      float stepsPerSecond = fabsf(speed) * MICRO_STEPS_PER_REVOLUTION / 360.0f;
      lastdelay = (uint64_t)(1000000.0f / stepsPerSecond);
      esp_timer_start_once(timer, lastdelay);
      return;
    }

    float target = control.position * MICRO_STEPS_PER_DEGREE; // 0..REV microsteps
    float diff = signedDistance(target); // signed microsteps to target
    float absdiff = fabsf(diff);
    float dt = lastdelay * 0.000001f; // seconds

    // Arrived: snap exactly and hold (don't re-arm the timer). The creep floor
    // bounds |speed| to MIN_SPEED here, so zeroing it is never a large jump.
    if (absdiff < 1.0f) {
      position = target;
      speed = 0.0f;
      return;
    }

    // Distance needed to brake the current speed to zero, in microsteps.
    float decelDist = (speed * speed) / (2.0f * accel) * MICRO_STEPS_PER_DEGREE;

    if (speed * diff < 0.0f)
      // Moving away from the target -> brake toward zero, then reverse.
      speed -= copysignf(accel * dt, speed);
    else if (decelDist >= absdiff)
      // Within braking distance -> decelerate so we land on the target.
      speed -= copysignf(accel * dt, speed);
    else if (fabsf(speed) < cruiseSpeed)
      // Room to spare -> accelerate toward the cruise cap.
      speed += copysignf(accel * dt, diff);
    else if (fabsf(speed) > cruiseSpeed)
      // Above the cap (e.g. inherited a higher speed) -> ease down to it.
      speed -= copysignf(accel * dt, speed);

    // Creep floor: never stall before arriving; always nudge toward the target.
    const float MIN_SPEED = 2.0f; // deg/s
    if (fabsf(speed) < MIN_SPEED)
      speed = copysignf(MIN_SPEED, diff);

    // Step in the direction we're actually traveling.
    if (speed > 0.0f) {
      motor->step(true);
      position += 1;
    } else {
      motor->step(false);
      position -= 1;
    }

    float stepsPerSecond = fabsf(speed) * MICRO_STEPS_PER_REVOLUTION / 360.0f;
    lastdelay = (uint64_t)(1000000.0f / stepsPerSecond);
    esp_timer_start_once(timer, lastdelay);
  }

  // Timer that drives the task. The callback recovers `this` and jumps into
  // the real (non-static) member function above, matching StepperMotor.h.
  const esp_timer_create_args_t timer_args = {
      .callback =
          [](void *arg) { static_cast<MotorController *>(arg)->controlTask(); },
      .arg = (void *)this, // arbitrary argument to pass to callback
      .name = "MotorControl"};

  esp_timer_handle_t timer;

public:
  MotorController(NewStepper *motor) : motor(motor) {
    esp_timer_create(&timer_args, &timer);
    // esp_timer_start_once(timer, 0); // start immediately
  }

  void applyControl(const MotorControl_t &control) {
    this->control = control;

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
      float target = control.position * MICRO_STEPS_PER_DEGREE;
      float sd = signedDistance(target);                   // signed microsteps
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

    esp_timer_start_once(timer, 0); // start immediately
  }

  // set speed in degrees per second
  void setSpeed(float speed) { this->speed = speed; }
};
