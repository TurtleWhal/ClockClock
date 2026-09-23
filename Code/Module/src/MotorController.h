#include "Arduino.h"
#include "NewStepper.h"

#include "../../Master/src/motorcontrol.h"

// Two-loop motion architecture:
//
//   * FAST loop  (core 1) — a dedicated busy task, MotorController::fastLoopTask.
//     Every iteration it reads the clock once and walks all motors, calling
//     stepTick(now): an absolute-deadline scheduler that emits ONE microstep
//     each time wall-clock passes the motor's next-step deadline. Uniform step
//     spacing (low jitter), and — at most one step per pass — no back-to-back
//     bursts for the motor to skip on when a pass runs late. This is the only
//     place `position` is written and the only place motor->step() is called. It
//     spins as fast as it can, so the achievable step rate is bounded by step()'s
//     own cost, not by a per-step timer re-arm.
//
//   * SLOW loop  (core 0) — a single periodic esp_timer at ~1 kHz,
//     MotorController::velocityTimerCb. It runs the accel/decel/cruise profile
//     for every motor and publishes each one's signed step velocity (stepVel,
//     microsteps/s) for the fast loop to consume. Velocity changes slowly
//     (accel is deg/s²), so 1 kHz is plenty and it replaces the old per-motor
//     one-shot step timers with one shared tick.
//
// Cross-loop handoff is two single-word floats: the slow loop writes `stepVel`
// and reads `position`; the fast loop writes `position` and reads `stepVel`. A
// 32-bit aligned load/store is atomic on the S3, so those need no lock — a
// read that is one tick stale is harmless. The command/profile fields that
// applyControl() (serial task) and the velocity tick (esp_timer task) share are
// guarded by a per-motor spinlock so a multi-field update can't be read torn.

// Maximum microstep rate the motors can physically follow — about a 50 µs step
// interval (20 kHz). Beyond this the rotor can't keep up and skips. The fast loop
// now delivers uniform, burst-free steps (see stepTick), so this is a genuine
// pull-out limit rather than the bursting artifact that made lower rates skip
// before. The clamp guards against commands (or time-profiles) that would demand
// a shorter interval. It is a STEP rate (microsteps/s), so finer microstepping
// automatically yields a proportionally lower top speed (deg/s). TUNE if your
// motors differ: raise until stepping starts to skip, then back off.
#define MAX_STEP_RATE_HZ 40000

// Per-motor phase stagger. When several motors resume from rest together (a whole
// display update), identical periods would make their step deadlines land in the
// same fast-loop pass, so their step() calls serialize and jitter each other.
// Offsetting each motor's deadline by (its index * this) spreads them across the
// pass so at most one steps at a time. A few µs — on the order of one step() — is
// enough; the 8 motors then span index*this across the period.
#define STEP_STAGGER_US 5

// Once a move gets within this many microsteps of its target, stop enforcing the
// commanded direction and home on the shortest path. A forced CW/CCW distance is
// always measured the long way round [0, REV), so without this a small overshoot
// past the setpoint reads as "go almost a full revolution again" and the motor
// spins around. 10° is far larger than any realistic overshoot yet well short of
// half a turn, so it never overrides the forced direction mid-travel.
#define HOMING_MARGIN (10 * MICRO_STEPS_PER_DEGREE)

class MotorController {
private:
  NewStepper *motor;

  MotorControl_t control; // active command (defaults from MotorControl_t)

  // ---- shared between the fast (core 1) and slow (core 0) loops ----
  // position: current position in microsteps, [0, REV). CW = increasing.
  //   Written ONLY by the fast loop; read by the slow loop and getCurrentPosition.
  // stepVel:  signed step velocity in microsteps/second (+ = CW).
  //   Written ONLY by the slow loop; read by the fast loop.
  volatile float position = 0.0f;
  volatile float stepVel = 0.0f;

  // ---- fast-loop private state (core 1 only) ----
  int64_t nextStepAt = 0;   // absolute µs deadline for this motor's next step
  float cachedVel = 0.0f;   // the stepVel the cached period was derived from
  int64_t cachedPeriod = 0; // µs between steps at cachedVel
  uint8_t motorIndex = 0;   // registration order; drives the per-motor stagger

  // ---- slow-loop private state (core 0 only) ----
  float speed = 0.0f; // signed velocity, degrees/second (+ = CW)

  // Effective profile parameters for the active move. Normally copied straight
  // from the command, but overridden when a target time is requested (see
  // applyControl). Defaults mirror MotorControl_t.
  float accel = 50.0f;           // deg/s^2
  float cruiseSpeed = 150.0f;    // deg/s
  float decelK = 0.0f;           // MICRO_STEPS_PER_DEGREE/(2*accel); precomputed per move
  float targetMicrosteps = 0.0f; // move target, normalized to [0, REV)
  bool active = false;           // false until the first applyControl()
  bool homing = false;           // latched near target: home shortest, not forced

  // Guards the command/profile fields (control, targetMicrosteps, accel,
  // cruiseSpeed, decelK) shared between applyControl() on the serial task and
  // updateVelocity() on the velocity-timer task — both on core 0.
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  // Signed distance (microsteps) from the current position to `target`,
  // honoring `direction` and 360° wrap-around. Positive => clockwise
  // (step(true), position++). `target` must already be normalized to [0, REV);
  // `position` is read once (it is normalized by the fast loop).
  float signedDistance(float target, MotorDirection_t direction) {
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    float pos = position;              // single atomic read of fast-loop state
    float forward = target - pos;      // clockwise travel, in (-REV, REV)
    if (forward < 0.0f)
      forward += REV; // -> [0, REV)
    float backward = (forward == 0.0f) ? 0.0f : REV - forward; // ccw travel

    switch (direction) {
    case MotorDirection_t::MOTOR_CW:
      return forward; // force clockwise, even if it's the long way around
    case MotorDirection_t::MOTOR_CCW:
      return -backward; // force counter-clockwise
    default:
      return (forward <= backward) ? forward : -backward; // shortest path
    }
  }

  // Clamp the profile speed to the motor's followable step rate, then publish it
  // for the fast loop. Clamping `speed` itself (not just the output) keeps the
  // brake-distance math consistent and prevents velocity winding up above the
  // cap (which would desync `speed` from the motor's actual motion).
  inline void publishStepVel() {
    constexpr float maxSpeed =
        (float)MAX_STEP_RATE_HZ / (float)MICRO_STEPS_PER_DEGREE; // deg/s
    if (speed > maxSpeed)
      speed = maxSpeed;
    else if (speed < -maxSpeed)
      speed = -maxSpeed;
    stepVel = speed * (float)MICRO_STEPS_PER_DEGREE;
  }

  // ---- registry + loop plumbing (a module drives 4 clocks = 8 motors) ----
  inline static MotorController *registry[8] = {};
  inline static uint8_t registryCount = 0;

  // Fast stepping loop. Pinned alone on core 1 (free now that the SW-PWM busy
  // loop is gone). Reads the clock once per pass and lets each motor step if its
  // deadline has arrived, so every motor is timed against the same `now`.
  static void fastLoopTask(void *) {
    for (;;) {
      int64_t now = esp_timer_get_time();
      for (uint8_t i = 0; i < registryCount; i++)
        registry[i]->stepTick(now);
    }
  }

  // Slow control loop: recompute every motor's velocity profile and publish its
  // step velocity. Runs on the esp_timer service task (core 0). Measured dt so
  // periodic-timer jitter can't bias the accel integration.
  static void velocityTimerCb(void *) {
    static int64_t last = 0;
    int64_t now = esp_timer_get_time();
    float dt = (last == 0) ? 0.001f : (now - last) * 0.000001f;
    last = now;
    for (uint8_t i = 0; i < registryCount; i++)
      registry[i]->updateVelocity(dt);
  }

public:
  MotorController(NewStepper *motor) : motor(motor) {
    // Constructed at global/static-init time (before FreeRTOS/esp_timer are
    // up), so just record ourselves; the loops are launched later by
    // startLoops() from setup(). Bounded so the MOTOR_TEST build's extra
    // instances can't overflow the array.
    if (registryCount < 8) {
      motorIndex = registryCount;
      registry[registryCount++] = this;
    }
  }

  // Launch both motion loops. Call once from setup(), after the motors are
  // configured. Safe only after FreeRTOS and esp_timer are running.
  static void startLoops() {
    // Fast loop on core 1. Priority 1 == the setup/loop task so this can't
    // preempt setup() before it finishes; core 1 is dedicated at runtime once
    // loop() parks on vTaskDelay. This loop never yields (like the old PWM
    // loop), so the Task WDT must not watch core 1's idle task — setup()
    // already reconfigures it with idle_core_mask = 0.
    xTaskCreatePinnedToCore(fastLoopTask, "MotorFast", 4096, NULL, 1, NULL, 1);

    // Slow loop: one periodic esp_timer (dispatched on core 0) driving all
    // motors' velocity profiles at 1 kHz.
    static esp_timer_handle_t velTimer = nullptr;
    const esp_timer_create_args_t args = {
        .callback = velocityTimerCb, .arg = nullptr, .name = "MotorVel"};
    esp_timer_create(&args, &velTimer);
    esp_timer_start_periodic(velTimer, 1000); // µs -> 1 ms tick
  }

  // FAST loop step (core 1). Absolute-deadline scheduler: emit one microstep when
  // wall-clock passes this motor's next-step deadline, then push the deadline out
  // by one period. `stepVel`'s sign is the direction, so a reversing command just
  // flips it — no discontinuity. At most one step per call: a pass that runs late
  // can't fire a back-to-back burst; if it fell a whole period behind it resyncs
  // (drops the backlog) and the slow loop's position control makes it up. Owns
  // `position`.
  void stepTick(int64_t now) {
    float v = stepVel; // microsteps/s, signed (atomic read of slow-loop state)
    if (v == 0.0f) {
      nextStepAt = 0; // idle: re-seed the deadline when motion resumes
      return;
    }

    // Recompute the period only when the commanded velocity actually changes
    // (the slow loop updates it at most every 1 ms), so cruising stays
    // divide-free in the hot path.
    if (v != cachedVel) {
      cachedVel = v;
      cachedPeriod = (int64_t)(1000000.0f / fabsf(v));
      if (cachedPeriod < 1)
        cachedPeriod = 1;
    }

    // Stagger this motor's deadline vs the others so their steps don't pile into
    // the same fast-loop pass (applied on resume and on a fell-behind resync).
    int64_t stagger = (int64_t)motorIndex * STEP_STAGGER_US;

    if (nextStepAt == 0)
      nextStepAt = now + cachedPeriod + stagger; // first step, phase-offset

    if (now < nextStepAt)
      return; // not due yet

    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    float pos = position;
    if (v > 0.0f) {
      motor->step(true);
      pos += 1.0f;
      if (pos >= REV)
        pos -= REV;
    } else {
      motor->step(false);
      pos -= 1.0f;
      if (pos < 0.0f)
        pos += REV;
    }
    position = pos;

    nextStepAt += cachedPeriod;
    if (nextStepAt <= now) // fell a full period behind: resync, don't burst
      nextStepAt = now + cachedPeriod + stagger;
  }

  // SLOW loop update (core 0). Integrate the accel/decel/cruise profile against
  // measured `dt` and publish the resulting signed step velocity for the fast
  // loop. This is the old controlTask logic minus the stepping itself.
  void updateVelocity(float dt) {
    if (!active)
      return;

    // Snapshot the command/profile under the lock so an applyControl() on the
    // serial task can't be read half-updated.
    MotorControl_t c;
    float target, a, cruise, dk;
    taskENTER_CRITICAL(&mux);
    c = control;
    target = targetMicrosteps;
    a = accel;
    cruise = cruiseSpeed;
    dk = decelK;
    taskEXIT_CRITICAL(&mux);

    float accelStep = a * dt; // velocity change available this tick

    // Continuous-spin mode: ramp toward control.speed in the commanded
    // direction and keep going. A later position command inherits `speed` and
    // decelerates.
    if (c.keepRunning) {
      float targetVel = (c.direction == MotorDirection_t::MOTOR_CCW)
                            ? -(float)c.speed
                            : (float)c.speed;
      if (speed < targetVel) {
        speed += accelStep;
        if (speed > targetVel)
          speed = targetVel;
      } else if (speed > targetVel) {
        speed -= accelStep;
        if (speed < targetVel)
          speed = targetVel;
      }
      publishStepVel();
      return;
    }

    // The forced direction governs the bulk of the travel, but arrival must be
    // by the shortest path: a forced distance is always measured [0, REV), so a
    // small overshoot past the setpoint would otherwise read as ~a full turn and
    // send the motor around again. Once we come within HOMING_MARGIN of the
    // target, latch into shortest-path homing so an overshoot corrects by a hair.
    float sd = signedDistance(target, c.direction);
    if (fabsf(sd) <= (float)HOMING_MARGIN)
      homing = true;
    float diff = homing
                     ? signedDistance(target, MotorDirection_t::MOTOR_SHORTEST)
                     : sd; // signed microsteps to target
    float absdiff = fabsf(diff);

    // Arrived: hold. The fast loop leaves position within <1 microstep (<0.008°)
    // of target, which is invisible, so no exact snap is needed.
    if (absdiff < 1.0f) {
      speed = 0.0f;
      stepVel = 0.0f;
      return;
    }

    float decelDist = speed * speed * dk; // v^2/(2a), in microsteps

    if (speed * diff < 0.0f || decelDist >= absdiff)
      speed -= copysignf(accelStep, speed); // brake toward zero / land on target
    else if (fabsf(speed) < cruise)
      speed += copysignf(accelStep, diff); // accelerate toward the cruise cap
    else if (fabsf(speed) > cruise)
      speed -= copysignf(accelStep, speed); // ease down to the cap

    publishStepVel();
  }

  void applyControl(const MotorControl_t &cmd) {
    // Resolve everything into locals first (especially the sqrtf-heavy time
    // profile), then publish the whole set under the lock so the velocity tick
    // never sees a torn update.
    MotorControl_t c = cmd;

    bool wasSpinning = control.keepRunning; // last published mode (we own writes)
    float v0speed = speed;                  // current velocity (atomic read)

    // Leaving constant-velocity (keepRunning) mode with a SHORTEST move: don't
    // let the motor turn around. Resolve SHORTEST to keep spinning the way it's
    // already going, so it decelerates to the target in that direction (even if
    // that's the long way around the dial).
    if (wasSpinning && !c.keepRunning && v0speed != 0.0f &&
        c.direction == MotorDirection_t::MOTOR_SHORTEST) {
      c.direction = (v0speed > 0.0f) ? MotorDirection_t::MOTOR_CW
                                     : MotorDirection_t::MOTOR_CCW;
    }

    // Precompute the move target once, normalized to [0, REV), so the loops
    // and signedDistance() stay fmodf-free.
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    float target = fmodf((float)(c.position * MICRO_STEPS_PER_DEGREE), REV);
    if (target < 0.0f)
      target += REV;

    // Default: drive the profile straight off the commanded accel/speed. The
    // current velocity is deliberately NOT reset, so an interrupting command
    // continues from the speed the motor already has (no discontinuity).
    float a = c.acceleration;
    float cruise = c.speed;

    // If a target time is requested, derive an accelerate-then-decelerate
    // profile that starts at the *current* velocity, ends at rest on the
    // target, and takes exactly `time` ms — so a moving motor blends into the
    // new move and still stops on time, without stopping first.
    if (!c.keepRunning && c.time != UINT16_MAX && c.time > 0) {
      float sd = signedDistance(target, c.direction);      // signed microsteps
      float D = fabsf(sd) / (float)MICRO_STEPS_PER_DEGREE;  // distance, degrees
      float T = c.time * 0.001f;                            // time, seconds

      if (D > 0.0f) {
        // Current velocity projected onto the move direction: + = already
        // heading toward the target, - = heading away from it.
        float v0 = (sd >= 0.0f) ? v0speed : -v0speed;

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
          a = (2.0f * vp - v0) / T;
          cruise = vp;
        } else {
          // Already faster than the profile's peak (v0 > vp): hitting the time
          // would require overshooting, so just brake smoothly onto the target.
          a = (v0 * v0) / (2.0f * D);
          cruise = v0; // v0 > 0 in this branch
        }
      }
    }

    // Per-move deceleration constant, so the velocity tick's brake test is a
    // multiply instead of a divide: decelDist = speed^2 * decelK.
    float dk = (a > 0.0f) ? (MICRO_STEPS_PER_DEGREE / (2.0f * a)) : 0.0f;

    // Publish atomically for the velocity tick. stepVel is intentionally left
    // for the next tick to recompute — the fast loop keeps stepping at the old
    // (continuous) velocity for <=1 ms, no glitch.
    taskENTER_CRITICAL(&mux);
    control = c;
    targetMicrosteps = target;
    accel = a;
    cruiseSpeed = cruise;
    decelK = dk;
    active = true;
    homing = false; // re-enforce the commanded direction for the new move
    taskEXIT_CRITICAL(&mux);
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
  // it while the motor is idle (stepVel == 0) so it doesn't race the fast loop.
  void setPosition(float degrees) {
    const float REV = (float)MICRO_STEPS_PER_REVOLUTION;
    float pos = fmodf(degrees * MICRO_STEPS_PER_DEGREE, REV);
    if (pos < 0.0f)
      pos += REV;
    position = pos;
  }
};
