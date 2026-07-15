#pragma once
//
// Robot manager, handles EVERYTHING.
//
// State machine (proto::State). The rise/sit animations are uninterruptable:
// once started they play to completion. The only thing that can break in is an
// over-current fault, which halts and de-energizes from any energized state.
//
//   OFF       de-energized, legs curled, awaiting enable(). Boot starts here, and
//             a completed sit-down returns here.
//   SETUP     rising: enable() runs the stand-up animation (curl the legs from
//             wherever they are -> slew to neutral -> raise the body) then -> IDLE.
//   IDLE      standing, feet at neutral; interpolates body pose. Any velocity -> WALK.
//   WALK      gait-driven; returns to IDLE when velocity settles to zero.
//   SHUTDOWN  lowering: shutdown() runs the sit-down animation (legs to neutral ->
//             lower the body -> curl the legs, femur to max) then cuts power -> OFF.
//   FAULT     emergency stop: halted and de-energized immediately (no animation);
//             any energized state can trip here (software over-current). enable() recovers.
//
// Body pose is absolute internally (mm, rad); standing == (0, 0, STANDING_HEIGHT).
// set_body_pose() takes offsets from that reference. Angles go to rad only at IK

#include <cmath>
#include <initializer_list>

#include "core/config.hpp"
#include "utils/math.hpp"
#include "communication/protocol.hpp"
#include "core/gait.hpp"
#include "core/kinematics.hpp"
#include "communication/hardware.hpp"

namespace robot {

using math::Vec3;

// Servo-frame translation.
//
// The gait and any body-pose command produce foot targets in the ground frame.
// This layer is the geometric glue to the actuators: for each leg it runs the IK
// (composing in the commanded body pose) and maps the resulting kinematic angles
// into servo space using the per-joint calibration data in config.hpp.
//
//   foot targets (+ body pose) -> IK -> kinematic angles (rad) -> calib -> servo deg
//

// 18 servo angles in degrees, indexed by cfg::servo_channel(leg, joint)
struct ServoAngles {
  float deg[cfg::N_SERVOS];
};

// Kinematic angle (rad) -> servo angle (deg), per the config calibration:
//   servo_deg = clamp(kin_deg * direction + trim, min, max)
// direction/trim are per-servo (leg-major); the clamp is per-joint-type.
inline float map_servo(int leg, int joint, float kin_rad) {
  const int ch = cfg::servo_channel(leg, joint);
  const float deg = math::rad2deg(kin_rad) * cfg::DIRECTION[ch] + cfg::TRIM_DEG[ch];
  return math::clampf(deg, cfg::RANGE[joint].min_deg, cfg::RANGE[joint].max_deg);
}

// Inverse of map_servo, ignoring the clamp: servo angle (deg) -> kinematic (rad).
// Handy for seeding FK from measured servo positions
inline float unmap_servo(int leg, int joint, float servo_deg) {
  const int ch = cfg::servo_channel(leg, joint);
  return math::deg2rad((servo_deg - cfg::TRIM_DEG[ch]) / cfg::DIRECTION[ch]);
}

// Resolve one tick's foot targets into servo angles. body_pos/body_rpy is the
// commanded body pose in the ground frame (body_pos.z carries the standing
// height). Returns false if any leg's target is geometrically unreachable; in
// that case that leg's angles are not meaningful and the caller should hold the
// previous command instead of applying this one
inline bool resolve(const Vec3 (&feet)[cfg::N_LEGS], const Vec3 &body_pos,
                    const Vec3 &body_rpy, ServoAngles &out) {
  bool ok = true;
  for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
    const kin::IkResult r = kin::inverse(leg, feet[leg], body_pos, body_rpy);
    if (!r.ok)
      ok = false;
    out.deg[cfg::servo_channel(leg, cfg::COXA)] = map_servo(leg, cfg::COXA, r.coxa);
    out.deg[cfg::servo_channel(leg, cfg::FEMUR)] = map_servo(leg, cfg::FEMUR, r.femur);
    out.deg[cfg::servo_channel(leg, cfg::TIBIA)] = map_servo(leg, cfg::TIBIA, r.tibia);
  }
  return ok;
}

class Robot {
public:
  explicit Robot(hw::Interface &hw) : hw_(hw) { hw_.set_power(false); }

  // Intent API (called by the dispatcher). Each marks the link alive (petting watchdog)

  // Stand up. Allowed only from a de-energized rest state: OFF (boot or fully sat
  // down) or FAULT. Ignored mid-animation, so a rise/sit always runs to completion.
  void enable() {
    pet();
    if (state_ == proto::State::OFF || state_ == proto::State::FAULT)
      start_stand_up();
  }

  // Orderly shutdown: sit-down animation, then cut power. From IDLE/WALK only, so
  // it cannot interrupt an in-progress stand-up.
  void shutdown() {
    pet();
    if (state_ == proto::State::IDLE || state_ == proto::State::WALK)
      start_sit_down();
  }

  // Soft stop: zero the velocity, keep standing
  void stop() {
    pet();
    if (state_ == proto::State::IDLE || state_ == proto::State::WALK)
      tvx_ = tvy_ = twz_ = 0.0f;
  }

  void set_velocity(float vx, float vy, float wz) {
    pet();
    if (state_ != proto::State::IDLE && state_ != proto::State::WALK) return;
    tvx_ = math::clampf(vx, -cfg::LIN_VEL_MAX, cfg::LIN_VEL_MAX);
    tvy_ = math::clampf(vy, -cfg::LIN_VEL_MAX, cfg::LIN_VEL_MAX);
    twz_ = math::clampf(wz, -cfg::ANG_VEL_MAX, cfg::ANG_VEL_MAX);
  }

  void set_body_pose(float x, float y, float z, float roll, float pitch,
                     float yaw) {
    pet();
    if (state_ != proto::State::IDLE && state_ != proto::State::WALK) return;
    const cfg::BodyPoseLimits &L = cfg::BODY_POSE;
    body_pos_t_ = {math::clampf(x, L.x_min, L.x_max),
                   math::clampf(y, L.y_min, L.y_max),
                   cfg::STANDING_HEIGHT + math::clampf(z, L.z_min, L.z_max)};
    body_rpy_t_ = {math::deg2rad(math::clampf(roll, L.roll_min, L.roll_max)),
                   math::deg2rad(math::clampf(pitch, L.pitch_min, L.pitch_max)),
                   math::deg2rad(math::clampf(yaw, L.yaw_min, L.yaw_max))};
  }

  void set_gait(gait::GaitId g) { pet(); gait_.set_gait(g); }

  void set_led(uint8_t mode, uint8_t r, uint8_t g, uint8_t b, float freq_hz) {
    pet();
    hw_.set_led(mode, r, g, b, freq_hz);
  }

  void heartbeat() { pet(); }

  // Provisioning support (called by the dispatcher). Config and the calibration
  // jog are only allowed while de-energized, so nothing reconfigures or drives a
  // raw servo mid-motion; the Pi provisions at connect, before enable().
  bool provisionable() const {
    return state_ == proto::State::OFF || state_ == proto::State::FAULT;
  }
  // Re-derive config-cached state after geometry (mounts/stance) is provisioned.
  void reconfigure() { gait_.reconfigure(); }
  // Physical provisioning / calibration jog live in the HAL.
  void set_servo_calibration(uint8_t ch, uint16_t min_us, uint16_t mid_us,
                             uint16_t max_us) {
    hw_.set_servo_calibration(ch, min_us, mid_us, max_us);
  }
  void set_servo_pin(uint8_t ch, uint8_t pin) { hw_.set_servo_pin(ch, pin); }
  void jog_servo(uint8_t ch, uint16_t pulse_us) { hw_.jog_servo(ch, pulse_us); }

  // Periodic tick (called by the control loop at ~CONTROL_RATE_HZ)
  void update(float dt) {
    hw_.update_leds(); // the board owns its LED animation; we just tick it here

    voltage_ = hw_.read_voltage();
    current_ = hw_.read_current();

    // Software over-current / low-voltage can trip an emergency stop from ANY
    // energized state (both only checked while energized, where the rail is live)
    if (energized_ && (current_ > cfg::CURRENT_MAX || voltage_ < cfg::VOLTAGE_MIN)) {
      trip_fault();
      return;
    }

    switch (state_) {
      case proto::State::SETUP:    // rising  : plays to completion (fault aside)
      case proto::State::SHUTDOWN: // lowering: plays to completion (fault aside)
        run_sequencer(dt);
        break;
      case proto::State::IDLE:
      case proto::State::WALK:
        run_active(dt);
        break;
      case proto::State::OFF:   // de-energized standby; nothing to do until enable()
      case proto::State::FAULT: // halted and de-energized; sticky until enable()
        break;
    }
  }

  // Queries (for replies/telemetry)
  proto::State state() const { return state_; }
  float voltage() const { return voltage_; }
  float current() const { return current_; }
  void odometry(float &x, float &y, float &yaw_rad) const {
    x = odom_x_;
    y = odom_y_;
    yaw_rad = odom_yaw_;
  }
  const ServoAngles &joints() const { return servos_; }

private:
  static constexpr float VEL_DEADBAND = 0.5f; // mm/s  : snap smoothed v to 0 below
  static constexpr float ANG_DEADBAND = 0.3f; // deg/s
  enum class Step : uint8_t { Curl, LegsToNeutral, RaiseBody, LowerBody };

  void pet() { since_cmd_ = 0.0f; }

  // IDLE / WALK
  void run_active(float dt) {
    since_cmd_ += dt;
    if (since_cmd_ > cfg::WATCHDOG_TIMEOUT_MS / 1000.0f) // lost comms -> stand still
      tvx_ = tvy_ = twz_ = 0.0f;

    smooth_commands(dt);
    slew_body(dt); // toward the commanded body pose

    const gait::Generator::Feet feet = gait_.update(dt, cvx_, cvy_, cwz_);
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) leg_[leg] = feet.pos[leg];
    resolve_write();
    integrate_odometry(dt);

    const bool moving = tvx_ != 0.0f || tvy_ != 0.0f || twz_ != 0.0f ||
                        cvx_ != 0.0f || cvy_ != 0.0f || cwz_ != 0.0f;
    state_ = moving ? proto::State::WALK : proto::State::IDLE;
  }

  // SETUP / SHUTDOWN sequencer
  void run_sequencer(float dt) {
    if (run_step(seq_[seq_idx_], dt)) {
      ++seq_idx_;
      if (seq_idx_ >= seq_len_) finalize_sequence();
    }
  }

  bool run_step(Step s, float dt) {
    switch (s) {
    case Step::Curl:
      return curl_step(dt);
    case Step::LegsToNeutral: {
      for (int leg = 0; leg < cfg::N_LEGS; ++leg) leg_t_[leg] = gait_.neutral(leg);
      const bool done = slew_legs(dt);
      resolve_write();
      return done;
    }
    case Step::RaiseBody:
      body_pos_t_ = {0.0f, 0.0f, cfg::STANDING_HEIGHT};
      body_rpy_t_ = {};
      { const bool done = slew_body(dt); resolve_write(); return done; }
    case Step::LowerBody:
      body_pos_t_ = {};
      body_rpy_t_ = {};
      { const bool done = slew_body(dt); resolve_write(); return done; }
    }
    return true;
  }

  void finalize_sequence() {
    if (post_seq_ == proto::State::IDLE) { // stand-up finished -> standing
      gait_.reset();
      reset_motion();
      body_pos_t_ = {0.0f, 0.0f, cfg::STANDING_HEIGHT};
      body_rpy_t_ = {};
      state_ = proto::State::IDLE;
    } else { // sit-down finished: legs curled -> cut power and rest in OFF
      hw_.set_power(false);
      energized_ = false;
      state_ = proto::State::OFF;
    }
  }

  void start_stand_up() {
    hw_.set_power(true);
    energized_ = true;
    reset_motion();
    gait_.reset();
    body_pos_ = {};   body_pos_t_ = {};   // start with the body on the ground
    body_rpy_ = {};   body_rpy_t_ = {};
    // Seed from where the legs ACTUALLY are: read the servo angles back from the
    // board and derive the Cartesian foot state by FK. The Curl step then folds
    // the legs from their real starting pose (no "already curled" assumption).
    hw_.read_servos(servos_.deg);
    seed_legs_from_servos();
    begin_sequence({Step::Curl, Step::LegsToNeutral, Step::RaiseBody},
                   proto::State::IDLE);
    state_ = proto::State::SETUP;
  }

  void start_sit_down() {
    tvx_ = tvy_ = twz_ = 0.0f;
    cvx_ = cvy_ = cwz_ = 0.0f;
    begin_sequence({Step::LegsToNeutral, Step::LowerBody, Step::Curl},
                   proto::State::OFF);
    state_ = proto::State::SHUTDOWN;
  }

  // Emergency stop: kill motion and power right now; recoverable via enable().
  void trip_fault() {
    tvx_ = tvy_ = twz_ = 0.0f;
    cvx_ = cvy_ = cwz_ = 0.0f;
    hw_.set_power(false);
    energized_ = false;
    seq_len_ = seq_idx_ = 0;
    state_ = proto::State::FAULT;
  }

  // animation primitives

  // Fold the legs to the safe power-off pose (femur/tibia at their servo maxima),
  // interpolating in servo space at JOINT_VEL_MAX. Seeds the Cartesian foot state
  // from FK once folded, so the following LegsToNeutral step starts from truth.
  bool curl_step(float dt) {
    ServoAngles target;
    set_curl_pose(target);
    const float step = cfg::JOINT_VEL_MAX * dt;
    bool done = true;
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
      float *cur = &servos_.deg[cfg::servo_channel(leg, 0)];
      const float *tgt = &target.deg[cfg::servo_channel(leg, 0)];
      const float ex = tgt[0] - cur[0], ey = tgt[1] - cur[1], ez = tgt[2] - cur[2];
      const float d = std::sqrt(ex * ex + ey * ey + ez * ez);
      if (d > step) {
        const float k = step / d;
        cur[0] += ex * k; cur[1] += ey * k; cur[2] += ez * k;
        done = false;
      } else {
        cur[0] = tgt[0]; cur[1] = tgt[1]; cur[2] = tgt[2];
      }
    }
    hw_.write_servos(servos_.deg);
    if (done) seed_legs_from_servos();
    return done;
  }

  // Rate-limited Cartesian move of every foot toward its target (mm/s).
  bool slew_legs(float dt) {
    const float step = cfg::BODY_LIN_VEL_MAX * dt;
    bool done = true;
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
      const Vec3 err = leg_t_[leg] - leg_[leg];
      const float d = err.norm();
      if (d > step) { leg_[leg] += err * (step / d); done = false; }
      else          { leg_[leg] = leg_t_[leg]; }
    }
    return done;
  }

  // Rate-limited move of the body pose toward its target (pos mm/s, ori deg/s).
  bool slew_body(float dt) {
    bool done = true;
    const Vec3 perr = body_pos_t_ - body_pos_;
    const float pm = perr.norm();
    const float lin = cfg::BODY_LIN_VEL_MAX * dt;
    if (pm > lin) { body_pos_ += perr * (lin / pm); done = false; }
    else          { body_pos_ = body_pos_t_; }

    Vec3 oerr = body_rpy_t_ - body_rpy_;
    oerr.z = math::wrap_angle(oerr.z);
    const float om = oerr.norm();
    const float ang = math::deg2rad(cfg::BODY_ANG_VEL_MAX) * dt;
    if (om > ang) { body_rpy_ += oerr * (ang / om); done = false; }
    else          { body_rpy_ = body_rpy_t_; }
    return done;
  }

  // helpers

  void resolve_write() {
    ServoAngles next;
    if (resolve(leg_, body_pos_, body_rpy_, next))
      servos_ = next; // else hold the last good pose rather than lunge
    hw_.write_servos(servos_.deg);
  }

  // Safe folded pose in servo degrees: coxa neutral, femur & tibia at their maxima.
  static void set_curl_pose(ServoAngles &s) {
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
      s.deg[cfg::servo_channel(leg, cfg::COXA)] = map_servo(leg, cfg::COXA, 0.0f);
      s.deg[cfg::servo_channel(leg, cfg::FEMUR)] = cfg::RANGE[cfg::FEMUR].max_deg;
      s.deg[cfg::servo_channel(leg, cfg::TIBIA)] = cfg::RANGE[cfg::TIBIA].max_deg;
    }
  }

  // Body-frame foot positions from the current servo angles (FK of the unmapped
  // kinematic angles).
  void seed_legs_from_servos() {
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
      const float c = unmap_servo(leg, cfg::COXA,  servos_.deg[cfg::servo_channel(leg, cfg::COXA)]);
      const float f = unmap_servo(leg, cfg::FEMUR, servos_.deg[cfg::servo_channel(leg, cfg::FEMUR)]);
      const float t = unmap_servo(leg, cfg::TIBIA, servos_.deg[cfg::servo_channel(leg, cfg::TIBIA)]);
      leg_[leg] = kin::forward(leg, c, f, t, body_pos_, body_rpy_, false);
    }
  }

  void begin_sequence(std::initializer_list<Step> steps, proto::State post) {
    uint8_t i = 0;
    for (Step s : steps) seq_[i++] = s;
    seq_len_ = i;
    seq_idx_ = 0;
    post_seq_ = post;
  }

  void reset_motion() {
    tvx_ = tvy_ = twz_ = 0.0f;
    cvx_ = cvy_ = cwz_ = 0.0f;
    odom_x_ = odom_y_ = odom_yaw_ = 0.0f;
    since_cmd_ = 0.0f;
  }

  // First-order low-pass (alpha = dt/(tau+dt), as in the legacy controller), with
  // a deadband so a zero command settles to EXACTLY zero and the gait can freeze.
  void smooth_commands(float dt) {
    const float a = dt / (cfg::VEL_SMOOTH_TAU + dt);
    cvx_ += a * (tvx_ - cvx_);
    cvy_ += a * (tvy_ - cvy_);
    cwz_ += a * (twz_ - cwz_);
    if (tvx_ == 0.0f && std::fabs(cvx_) < VEL_DEADBAND) cvx_ = 0.0f;
    if (tvy_ == 0.0f && std::fabs(cvy_) < VEL_DEADBAND) cvy_ = 0.0f;
    if (twz_ == 0.0f && std::fabs(cwz_) < ANG_DEADBAND) cwz_ = 0.0f;
  }

  // Integrate the gait's ACHIEVED (post-clamp) twist into a world pose. Convention
  // matches the protocol: +x forward, +y left, +yaw CCW (right-handed).
  void integrate_odometry(float dt) {
    odom_yaw_ += math::deg2rad(gait_.achieved_yaw_rate()) * dt;
    const float c = std::cos(odom_yaw_), s = std::sin(odom_yaw_);
    const float vx = gait_.achieved_vx(), vy = gait_.achieved_vy();
    odom_x_ += (c * vx - s * vy) * dt;
    odom_y_ += (s * vx + c * vy) * dt;
  }

  hw::Interface &hw_;
  proto::State state_ = proto::State::OFF; // boot: de-energized standby
  bool energized_ = false;
  gait::Generator gait_;

  // Velocity command targets / smoothed values.
  float tvx_ = 0.0f, tvy_ = 0.0f, twz_ = 0.0f; // mm/s, mm/s, deg/s
  float cvx_ = 0.0f, cvy_ = 0.0f, cwz_ = 0.0f;

  // Body pose (absolute) + targets: pos mm, rpy rad.
  Vec3 body_pos_{}, body_rpy_{};
  Vec3 body_pos_t_{}, body_rpy_t_{};

  // Foot targets (body frame): current + Cartesian slew target (animations).
  Vec3 leg_[cfg::N_LEGS];
  Vec3 leg_t_[cfg::N_LEGS];

  ServoAngles servos_{}; // last commanded (also curl interpolation state)

  // Sequencer.
  Step seq_[4] = {};
  uint8_t seq_len_ = 0, seq_idx_ = 0;
  proto::State post_seq_ = proto::State::IDLE;

  // Telemetry / safety.
  float voltage_ = 0.0f, current_ = 0.0f;
  float odom_x_ = 0.0f, odom_y_ = 0.0f, odom_yaw_ = 0.0f; // mm, mm, rad
  float since_cmd_ = 0.0f;                                 // s (watchdog)
};

} // namespace robot
