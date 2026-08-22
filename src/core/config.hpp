#pragma once
//
// Everything about the robot that the firmware needs to know:
// link lengths, where each leg mounts, servo-kinematic mapping,
// gait parameters, board pins, and safety limits.
//
// SDK-free: only ints/floats/enums so that pure core and tests can
// include it without the Pico SDK. UART-related stuff is platform
// dependent and lives with the platform code; only the pin numbers
// are here.
//
// Units: lengths mm, angles deg. Canonical leg order is fixed by
// the Leg enum and must match the order the Pi assumes.
//
// Baked defaults vs. provisioned overrides
// ----------------------------------------
// The values below are the baked DEFAULTS: the board is fully functional
// standalone with exactly these. Anything the Pi is allowed to override at
// connect time (geometry, kinematic map, gaits, safety limits) is a mutable
// `inline` global here rather than `constexpr`, so provisioning simply assigns
// into it and every existing read site keeps working unchanged. Structural
// facts that can never be provisioned (leg/joint counts, the servo_channel
// formula, board GPIOs, control rate, transport constants) stay `constexpr`.
//
// Per-servo physical calibration (pulse widths) and the logical->physical pin
// map are NOT here: they are a HAL concern (hw::Interface), provisioned straight
// to the driver, because they describe wiring and individual servos, not the
// kinematic model the pure core reasons about.

#include <cstdint>

namespace cfg {

// Useful constants (structural, never provisioned)
constexpr int N_LEGS = 6;
constexpr int N_JOINTS = 3;
constexpr int N_SERVOS = N_LEGS * N_JOINTS;

enum Leg : uint8_t {
  FRONT_RIGHT = 0,
  MIDDLE_RIGHT,
  REAR_RIGHT,
  REAR_LEFT,
  MIDDLE_LEFT,
  FRONT_LEFT,
};

enum Joint : uint8_t { COXA = 0, FEMUR, TIBIA };

// Leg link geometry (mm), shared by all six legs. COXA_LEN is the RADIAL
// distance from the coxa axis to the coxa-femur joint; COXA_OFFSET is how far
// that joint sits to the side of the coxa axis (+ left, looking out along the
// leg), which puts the femur/tibia plane off the axis rather than through it.
inline float COXA_LEN = 68.0f;
inline float COXA_OFFSET = 0.0f;
inline float FEMUR_LEN = 76.7f;
inline float TIBIA_LEN = 99.6f;

// Where each leg mounts on the body (body frame)
struct LegMount {
  float x;       // mm
  float y;       // mm
  float z;       // mm, height of the coxa-femur joint over the body origin
  float yaw_deg; // mounting yaw of the coxa axis
};
inline LegMount MOUNT[N_LEGS] = {
    // x, y, z, yaw
    {82.0f, -57.0f, 0.0f, -45.0f},   // FRONT_RIGHT
    {0.0f, -71.0f, 0.0f, -90.0f},    // MIDDLE_RIGHT
    {-82.0f, -57.0f, 0.0f, -135.0f}, // REAR_RIGHT
    {-82.0f, 57.0f, 0.0f, 135.0f},   // REAR_LEFT
    {0.0f, 71.0f, 0.0f, 90.0f},      // MIDDLE_LEFT
    {82.0f, 57.0f, 0.0f, 45.0f},     // FRONT_LEFT
};

// Servo-kinematic mapping
// The IK works in a clean kinematic frame; servos do not necessarily match it:
//
//      direct : servo_deg = clamp(kin_deg * direction + trim_deg, min_deg, max_deg)
//      inverse: kin_deg    = (servo_deg - trim_deg) / direction
//
// DIRECTION and TRIM_DEG are PER SERVO (leg-major, cfg::servo_channel order), so
// they mirror the config YAML's hardware.direction / hardware.trim one-to-one and
// a leg mounted differently can override just its own entries. On this robot the
// values are uniform across legs (a mounting convention), but keeping them
// per-servo matches the legacy layout and leaves per-leg tweaks possible.
inline float DIRECTION[N_SERVOS] = {
    // coxa, femur, tibia
    -1.0f, 1.0f, -1.0f, // FRONT_RIGHT
    -1.0f, 1.0f, -1.0f, // MIDDLE_RIGHT
    -1.0f, 1.0f, -1.0f, // REAR_RIGHT
    -1.0f, 1.0f, -1.0f, // REAR_LEFT
    -1.0f, 1.0f, -1.0f, // MIDDLE_LEFT
    -1.0f, 1.0f, -1.0f, // FRONT_LEFT
};
inline float TRIM_DEG[N_SERVOS] = {
    // coxa, femur, tibia
    0.0f, 0.0f, -163.0f, // FRONT_RIGHT
    0.0f, 0.0f, -163.0f, // MIDDLE_RIGHT
    0.0f, 0.0f, -163.0f, // REAR_RIGHT
    0.0f, 0.0f, -163.0f, // REAR_LEFT
    0.0f, 0.0f, -163.0f, // MIDDLE_LEFT
    0.0f, 0.0f, -163.0f, // FRONT_LEFT
};

// Servo-space clamp, per JOINT TYPE (all coxae share a range, etc.). Mirrors the
// config YAML's safety.{coxa,femur,tibia}_range.
struct JointRange {
  float min_deg;
  float max_deg;
};
inline JointRange RANGE[N_JOINTS] = {
    {-90.0f, 90.0f},  // COXA
    {-90.0f, 90.0f},  // FEMUR
    {-120.0f, 0.0f},  // TIBIA
};

// Servo channel driving (leg, joint): leg-major, 0..17. Canonical LOGICAL
// ordering shared with the Pi and the sim/URDF; the physical pin behind each
// channel is a HAL concern (provisioned separately).
constexpr uint8_t servo_channel(int leg, int joint) {
  return static_cast<uint8_t>(leg * N_JOINTS + joint);
}

// Gait parameters
struct GaitParams {
  float duty_factor; // fraction of the cycle a leg spends in stance
  float step_height; // mm, peak swing lift
  float max_stride;  // mm, stride is grown with speed up to this, then twist
                     // clamps
  float overlap;     // phase overlap between groups (0 = none)
};
inline GaitParams GAIT[3] = {
    // duty, step_h, max_stride, overlap
    {0.5000f, 60.0f, 120.0f, 0.0f}, // TRIPOD
    {0.8333f, 60.0f, 90.0f, 0.0f},  // WAVE
    {0.6700f, 60.0f, 110.0f, 0.0f}, // RIPPLE
};

// Constant-cadence locomotion model: one full gait cycle every CYCLE_TIME
// seconds at every speed; stride length scales with commanded velocity
inline float CYCLE_TIME = 0.8f;       // s  (1.25 Hz cadence)
inline float STANCE_RADIUS = 155.0f;  // mm (neutral foot distance from mount)
inline float STANDING_HEIGHT = 80.0f; // mm (body height when standing)

// Command clamps & smoothing
inline float LIN_VEL_MAX = 300.0f;   // mm/s
inline float ANG_VEL_MAX = 60.0f;    // deg/s
inline float VEL_SMOOTH_TAU = 0.05f; // s, first-order velocity smoothing

// Body-pose interpolation rates
inline float BODY_LIN_VEL_MAX = 50.0f; // mm/s
inline float BODY_ANG_VEL_MAX = 30.0f; // deg/s
inline float JOINT_VEL_MAX = 200.0f;   // deg/s

// Body-pose command limits, each an independent [min, max] so they need not be
// symmetric. x/y/z are mm (z relative to STANDING_HEIGHT); roll/pitch/yaw are deg.
struct BodyPoseLimits {
  float x_min, x_max;
  float y_min, y_max;
  float z_min, z_max;
  float roll_min, roll_max;
  float pitch_min, pitch_max;
  float yaw_min, yaw_max;
};
inline BodyPoseLimits BODY_POSE = {
    -50.0f, 50.0f,   // x
    -50.0f, 50.0f,   // y
    -20.0f, 20.0f,   // z
    -10.0f, 10.0f,   // roll
    -10.0f, 10.0f,   // pitch
    -10.0f, 10.0f,   // yaw
};

// Servo2040 pins (structural)
// The 18 servo channels, the 6 WS2812 pixels, the analog mux and the shared ADC
// all come from the Pimoroni servo2040 namespace. These are the GPIOs
constexpr unsigned UART_TX_PIN = 20;
constexpr unsigned UART_RX_PIN = 21;
constexpr unsigned POWER_CUTOFF_PIN = 19; // power cutoff trace
// NOTE: the hardware over-current auto-cutoff doesn't work on the current
// version of the board, so the control loop enforces CURRENT_MAX (and the
// VOLTAGE_MIN low-voltage cutoff) in software

// Control loop (structural)
constexpr float CONTROL_RATE_HZ = 50.0f;
constexpr unsigned WATCHDOG_TIMEOUT_MS = 500; // no setpoint/heartbeat within this window -> auto sit-down and power off

// Safety (provisionable: the Pi may tighten/relax the software cutoffs). Both are
// enforced in the control loop, but only while energized.
inline float CURRENT_MAX = 12.0f;  // A, software over-current cutoff -> FAULT
inline float VOLTAGE_MIN = 5.0f;   // V, software low-voltage cutoff  -> FAULT

// Transport constants (structural)
constexpr uint32_t BAUD = 921600;
constexpr uint8_t SOF = 0xAA;
constexpr uint8_t MAX_PAYLOAD = 128; // largest is GetJoints reply (72 B)

} // namespace cfg
