#pragma once
//
// Geometry primitives shared by kinematics, gait and locomotion
//

#include <cmath>

namespace math {

constexpr float PI = 3.14159265358979323846f;

inline float deg2rad(float d) { return d * (PI / 180.0f); }
inline float rad2deg(float r) { return r * (180.0f / PI); }
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
// Wrap an angle (rad) into (-PI, PI]
inline float wrap_angle(float a) {
  return std::atan2(std::sin(a), std::cos(a));
}

struct Vec3 {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  Vec3() = default;
  Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
  Vec3 &operator+=(const Vec3 &o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }

  float norm() const { return std::sqrt(x * x + y * y + z * z); }
  float norm_xy() const {
    return std::sqrt(x * x + y * y);
  } // planar (xy) length
};

// Rotate v about +z by yaw (planar rotation in the xy-plane)
inline Vec3 rot_z(float yaw, const Vec3 &v) {
  const float c = std::cos(yaw), s = std::sin(yaw);
  return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}

// Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll), row-major into r[9]
inline void rpy_matrix(const Vec3 &rpy, float r[9]) {
  const float cr = std::cos(rpy.x), sr = std::sin(rpy.x);
  const float cp = std::cos(rpy.y), sp = std::sin(rpy.y);
  const float cy = std::cos(rpy.z), sy = std::sin(rpy.z);
  r[0] = cy * cp;
  r[1] = cy * sp * sr - sy * cr;
  r[2] = cy * sp * cr + sy * sr;
  r[3] = sy * cp;
  r[4] = sy * sp * sr + cy * cr;
  r[5] = sy * sp * cr - cy * sr;
  r[6] = -sp;
  r[7] = cp * sr;
  r[8] = cp * cr;
}

// Body -> world:  R(rpy) * p + t
inline Vec3 body_to_world(const Vec3 &p, const Vec3 &t, const Vec3 &rpy) {
  float r[9];
  rpy_matrix(rpy, r);
  return {r[0] * p.x + r[1] * p.y + r[2] * p.z + t.x,
          r[3] * p.x + r[4] * p.y + r[5] * p.z + t.y,
          r[6] * p.x + r[7] * p.y + r[8] * p.z + t.z};
}

// World -> body:  R(rpy)^T * (p - t)   (exact inverse of body_to_world)
inline Vec3 world_to_body(const Vec3 &p, const Vec3 &t, const Vec3 &rpy) {
  float r[9];
  rpy_matrix(rpy, r);
  const Vec3 d = p - t;
  return {r[0] * d.x + r[3] * d.y + r[6] * d.z,
          r[1] * d.x + r[4] * d.y + r[7] * d.z,
          r[2] * d.x + r[5] * d.y + r[8] * d.z};
}

} // namespace math
