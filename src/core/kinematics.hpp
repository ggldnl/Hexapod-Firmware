#pragma once
//
// Per-leg forward/inverse kinematics.
//
// Frames:
//   - Foot targets handed to inverse() are in the WORLD (ground) frame.
//   - body_pos/body_rpy is the body's pose in that world frame; it is
//     composed in/out here, so a foot target stays put on the ground while 
//     the body leans.
//   - Each leg's mount frame is a yaw rotation + translation in the body frame.
//
// Angles are radians in the kinematic frame. Mapping to servo space
// (direction/trim/clamp, and the conversion to degrees) happens in
// the robot layer, not here.

#include <cmath>

#include "core/config.hpp"
#include "utils/math.hpp"

namespace kin {

using math::Vec3;

struct IkResult {
  bool ok = false;
  float coxa = 0.0f;  // rad
  float femur = 0.0f; // rad
  float tibia = 0.0f; // rad
};

// Leg mount pose in the body frame, from baked config
inline Vec3 mount_pos(int leg) {
  return {cfg::MOUNT[leg].x, cfg::MOUNT[leg].y, cfg::MOUNT[leg].z};
}
inline float mount_yaw(int leg) {
  return math::deg2rad(cfg::MOUNT[leg].yaw_deg);
}

// 2-link planar IK in the leg frame: the coxa is a yaw about +z; femur and
// tibia live in a vertical plane parallel to the radial one but pushed
// COXA_OFFSET to the side, so the coxa has to turn past the target's bearing
// by however much the offset subtends. Returns ok=false when the target is out
// of reach, including targets closer to the axis than the offset itself
inline IkResult leg_inverse(float x, float y, float z) {
  IkResult r;
  const float rho2 = x * x + y * y;
  if (rho2 < cfg::COXA_OFFSET * cfg::COXA_OFFSET) return r; // inside the offset
  // Distance covered inside the leg plane, and the bearing the offset eats
  const float radial = std::sqrt(rho2 - cfg::COXA_OFFSET * cfg::COXA_OFFSET);
  const float coxa = std::atan2(y, x) - std::atan2(cfg::COXA_OFFSET, radial);
  const float reach = radial - cfg::COXA_LEN;
  const float d = std::sqrt(reach * reach + z * z);
  if (d > (cfg::FEMUR_LEN + cfg::TIBIA_LEN) ||
      d < std::fabs(cfg::FEMUR_LEN - cfg::TIBIA_LEN))
    return r; // unreachable

  const float cos_tibia = (cfg::FEMUR_LEN * cfg::FEMUR_LEN +
                           cfg::TIBIA_LEN * cfg::TIBIA_LEN - d * d) /
                          (2.0f * cfg::FEMUR_LEN * cfg::TIBIA_LEN);

  const float tibia =
      std::acos(math::clampf(cos_tibia, -1.0f, 1.0f)) - math::PI;

  const float cos_femur = (cfg::FEMUR_LEN * cfg::FEMUR_LEN + d * d -
                           cfg::TIBIA_LEN * cfg::TIBIA_LEN) /
                          (2.0f * cfg::FEMUR_LEN * d);

  const float femur =
      std::atan2(z, reach) + std::acos(math::clampf(cos_femur, -1.0f, 1.0f));

  r.ok = true;
  r.coxa = coxa;
  r.femur = femur;
  r.tibia = tibia;
  return r;
}

// FK in the leg frame: joint angles (rad) -> foot position (mm)
inline Vec3 leg_forward(float coxa, float femur, float tibia) {
  const float radial = cfg::COXA_LEN + cfg::FEMUR_LEN * std::cos(femur) +
                       cfg::TIBIA_LEN * std::cos(femur + tibia);
  const float z = cfg::FEMUR_LEN * std::sin(femur) +
                  cfg::TIBIA_LEN * std::sin(femur + tibia);
  // The leg reaches out inside its own plane, which the coxa swings around
  const float c = std::cos(coxa), s = std::sin(coxa);
  return {radial * c - cfg::COXA_OFFSET * s, radial * s + cfg::COXA_OFFSET * c,
          z};
}

// Full IK for one leg: world-frame foot target + body pose -> joint angles
inline IkResult inverse(int leg, const Vec3 &foot_world, const Vec3 &body_pos,
                        const Vec3 &body_rpy) {
  const Vec3 in_body = math::world_to_body(foot_world, body_pos, body_rpy);
  const Vec3 in_leg = math::rot_z(-mount_yaw(leg), in_body - mount_pos(leg));
  return leg_inverse(in_leg.x, in_leg.y, in_leg.z);
}

// Full FK for one leg. With apply_body=false it returns the BODY-frame foot
// position (used to seed leg state from measured joint angles, exactly like the
// Python forward() called with no body args)
inline Vec3 forward(int leg, float coxa, float femur, float tibia,
                    const Vec3 &body_pos, const Vec3 &body_rpy,
                    bool apply_body) {
  const Vec3 in_body =
      math::rot_z(mount_yaw(leg), leg_forward(coxa, femur, tibia)) +
      mount_pos(leg);
  return apply_body ? math::body_to_world(in_body, body_pos, body_rpy)
                    : in_body;
}

} // namespace kin
