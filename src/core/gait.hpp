#pragma once
//
// Foot-trajectory generator.
//
// Speed model: constant-cadence, velocity-scaled stride.
//   One full cycle every cfg::CYCLE_TIME seconds at every speed. Speed comes
//   from growing the step, not quickening the rhythm: stride = foot_velocity *
//   stance duration. When the biggest required step exceeds the reachable
//   max_stride, the whole twist (linear + angular together) is scaled down so
//   the turn radius is preserved.
//
// Output: foot targets, one Vec3 per leg (mm). They live in the body's neutral
//   ground frame: xy in the body plane, z = 0 for a grounded foot and z > 0
//   mid-swing. Standing height, body lean and body shift are not taken into
//   account here

#include <cmath>
#include <cstdint>

#include "core/config.hpp"
#include "utils/math.hpp"

namespace gait {

using math::Vec3;

// Gait identifiers. Order must match cfg::GAIT[] and proto::GaitId so a wire id
// indexes straight through. Kept local so this core file stays protocol-free
enum GaitId : uint8_t { TRIPOD = 0, WAVE = 1, RIPPLE = 2, N_GAITS = 3 };

// How a gait spreads its phase over the legs: each leg belongs to one phase
// group, and the groups lift off one after another, evenly spaced over the
// cycle.
//
// group_of[leg] == -1 marks a leg that does NOT take part in this gait. The
// classic three use all six legs, so none are -1. A crab gait walks on four
// legs and idles the other two e.g. the front-four crab (legs FR, MR, ML, FL)
// would be
//     { 2, { 0, 1, -1, -1, 1, 0 } }
// with the two rear legs idle. Only the table changes; the update() logic
// already skips idle legs
// TODO remember we actually need to implement the crab walk
struct Pattern {
  uint8_t num_groups;
  int8_t
      group_of[cfg::N_LEGS]; // phase group per leg, -1 = not part of this gait
};

// Leg index is the cfg::Leg enum:
//   FRONT_RIGHT=0  MIDDLE_RIGHT=1  REAR_RIGHT=2  REAR_LEFT=3  MIDDLE_LEFT=4
//   FRONT_LEFT=5
constexpr Pattern PATTERN[N_GAITS] = {
    // TRIPOD: two alternating tripods (FR,RR,ML) vs (MR,RL,FL)
    {2, {0, 1, 0, 1, 0, 1}},
    // WAVE: one leg at a time, down the right side then up the left
    {6, {0, 1, 2, 3, 4, 5}},
    // RIPPLE: three diagonal pairs
    {3, {0, 1, 2, 0, 2, 1}},
};

static_assert(
    sizeof(cfg::GAIT) / sizeof(cfg::GAIT[0]) == N_GAITS,
    "PATTERN[] and cfg::GAIT[] must describe the same gaits in the same order");

class Generator {
public:
  struct Feet {
    Vec3 pos[cfg::N_LEGS];
  };

  Generator() {
    compute_neutral();
    reset();
  }

  void set_gait(GaitId g) { gait_ = g; }
  GaitId gait() const { return gait_; }

  // Re-derive config-cached state (the neutral stance depends on cfg::MOUNT and
  // cfg::STANCE_RADIUS). Call after those are provisioned; the per-gait params
  // (duty/stride/...) are read live from cfg::GAIT each update, so they need no
  // refresh.
  void reconfigure() { compute_neutral(); }

  // Clear phase and per-update state. Call before starting a fresh walk
  void reset() {
    phase_ = 0.0f;
    achieved_vx_ = achieved_vy_ = achieved_wz_deg_ = 0.0f;
  }

  // Advance the gait by dt and return this tick's foot targets.
  //   dt         elapsed time since the previous call (s)
  //   vx, vy     commanded body linear velocity (mm/s)
  //   wz_deg     commanded yaw rate (deg/s, +ccw)
  //
  // dt is a parameter on purpose: the gait then owns no clock and no assumption
  // about the loop rate, so cadence stays correct under timing jitter and the
  // whole thing steps deterministically in a test. The caller is
  // expected to have already clamped/smoothed the command; the only limit
  // applied here is the stride-reachability clamp
  Feet update(float dt, float vx, float vy, float wz_deg);

  // Post-clamp twist actually executed last update, for odometry. Equal to
  // the command unless the stride saturated, in which case both parts were
  // scaled by the same factor (radius preserved)
  float achieved_vx() const { return achieved_vx_; }
  float achieved_vy() const { return achieved_vy_; }
  float achieved_yaw_rate() const { return achieved_wz_deg_; }

  // Queries (handy for telemetry/body compensation)
  float phase() const { return phase_; }
  const Vec3 &neutral(int leg) const { return neutral_[leg]; }
  bool in_stance(int leg) const {
    return PATTERN[gait_].group_of[leg] < 0 || leg_phase(leg, phase_) < duty();
  }

private:
  // Per-gait parameters, pulled from the baked config
  float duty() const { return cfg::GAIT[gait_].duty_factor; }
  float step_height() const { return cfg::GAIT[gait_].step_height; }
  float max_stride() const { return cfg::GAIT[gait_].max_stride; }
  float overlap() const { return cfg::GAIT[gait_].overlap; }

  static float wrap01(float p) { return p - std::floor(p); }

  // Neutral (mid-stride, grounded) foot position: mount point pushed out by
  // STANCE_RADIUS along the leg's mounting yaw. Computed once
  void compute_neutral() {
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
      const float yaw = math::deg2rad(cfg::MOUNT[leg].yaw_deg);
      neutral_[leg] = {cfg::MOUNT[leg].x + cfg::STANCE_RADIUS * std::cos(yaw),
                       cfg::MOUNT[leg].y + cfg::STANCE_RADIUS * std::sin(yaw),
                       0.0f};
    }
  }

  // Phase of one leg [0,1): [0,duty) stance, [duty,1) swing
  float leg_phase(int leg, float global) const {
    const Pattern &pat = PATTERN[gait_];
    const float offset =
        float(pat.group_of[leg]) * (1.0f - overlap()) / float(pat.num_groups);
    return wrap01(global + offset);
  }

  // Ground velocity this foot must track during stance: body linear velocity
  // plus the tangential velocity from body yaw (omega x r) at the neutral foot
  // point
  Vec3 foot_velocity(int leg, float vx, float vy, float wz_rad) const {
    const Vec3 &n = neutral_[leg];
    return {vx - n.y * wz_rad, vy + n.x * wz_rad, 0.0f};
  }

  // Foot position for a leg at the given phase, given its full-stance stride
  Vec3 foot_position(int leg, float global, const Vec3 &stride) const {
    const Vec3 &n = neutral_[leg];
    const float lp = leg_phase(leg, global);
    const float df = duty();
    if (lp < df) {
      // Stance: on the ground, sliding front (+half stride) to back (-half)
      const float progress = 0.5f - lp / df;
      Vec3 p = n + stride * progress;
      p.z = 0.0f;
      return p;
    }
    // Swing: airborne, arcing back (-half) to front (+half) on a parabola
    const float s = (lp - df) / (1.0f - df); // 0..1 across swing
    Vec3 p = n + stride * (s - 0.5f);
    p.z = step_height() * 4.0f * s * (1.0f - s);
    return p;
  }

  GaitId gait_ = TRIPOD;
  float phase_ = 0.0f;
  float achieved_vx_ = 0.0f, achieved_vy_ = 0.0f, achieved_wz_deg_ = 0.0f;
  Vec3 neutral_[cfg::N_LEGS];
};

inline Generator::Feet Generator::update(float dt, float vx, float vy,
                                         float wz_deg) {
  const Pattern &pat = PATTERN[gait_];
  const float wz = math::deg2rad(wz_deg);
  const float stance_dur = duty() * cfg::CYCLE_TIME;

  // Per-leg ground velocity to track, and the fastest of them
  Vec3 foot_vel[cfg::N_LEGS];
  float max_speed = 0.0f;
  for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
    if (pat.group_of[leg] < 0)
      continue; // idle leg (crab): no stride
    foot_vel[leg] = foot_velocity(leg, vx, vy, wz);
    max_speed = std::fmax(max_speed, foot_vel[leg].norm_xy());
  }

  Vec3 stride[cfg::N_LEGS] = {};
  float twist_scale = 1.0f;
  bool advancing = false;

  if (max_speed > 1e-3f) {
    // Normal walking. Stride = foot velocity * stance duration, clamped so the
    // biggest step stays reachable (scaling the whole twist keeps the radius)
    advancing = true;
    float max_needed = 0.0f;
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
      if (pat.group_of[leg] < 0)
        continue;
      stride[leg] = foot_vel[leg] * stance_dur;
      max_needed = std::fmax(max_needed, stride[leg].norm_xy());
    }
    if (max_needed > max_stride()) {
      twist_scale = max_stride() / max_needed;
      for (int leg = 0; leg < cfg::N_LEGS; ++leg)
        stride[leg] = stride[leg] * twist_scale;
    }
  }
  // else: zero command -> freeze. We fall through with advancing == false, which
  // returns every foot to neutral WITHOUT advancing the phase, so no foot is ever
  // raised while standing. A continuous-swing gait (tripod, ripple) has no
  // all-grounded phase, so stillness has to come from leaving the cycle, not from
  // freezing at some particular phase. (A leg that was mid-swing when the command
  // hit zero therefore drops straight to neutral; if that ever needs to be graceful
  // it belongs in the walk<->stand transition of the body/FSM layer, not here.)

  achieved_vx_ = advancing ? vx * twist_scale : 0.0f;
  achieved_vy_ = advancing ? vy * twist_scale : 0.0f;
  achieved_wz_deg_ = advancing ? wz_deg * twist_scale : 0.0f;

  Feet out;
  if (!advancing) {
    for (int leg = 0; leg < cfg::N_LEGS; ++leg)
      out.pos[leg] = neutral_[leg];
    return out;
  }

  phase_ = wrap01(phase_ + dt / cfg::CYCLE_TIME);
  for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
    out.pos[leg] = (pat.group_of[leg] < 0)
                       ? neutral_[leg] // idle leg parked at neutral
                       : foot_position(leg, phase_, stride[leg]);
  }
  return out;
}

} // namespace gait
