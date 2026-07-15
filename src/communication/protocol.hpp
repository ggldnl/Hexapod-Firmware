#pragma once
//
// Protocol that details message passing between Raspberry Pi and Servo2040.
// The python interface mirrors this file by hand.
//
// Frame layout
//
//     ┌──────┬──────┬────────┬───────────────┬──────────┬──────────┐
//     │ SOF  │ LEN  │ opcode │ payload[LEN]  │ CRC lo   │ CRC hi   │
//     │ 0xAA │ 0..N │ 1 byte │ LEN bytes     │          │          │
//     └──────┴──────┴────────┴───────────────┴──────────┴──────────┘
//
//   - SOF: start-of-frame marker (0xAA)
//   - LEN: payload length in bytes (0..MAX_PAYLOAD); the opcode is NOT counted
//   - CRC: CRC16-CCITT (poly 0x1021, init 0xFFFF), little-endian, computed over
//          LEN + opcode + payload (so the length field is protected too)
//   - Total: frame size = LEN + 5. There is no end-of-frame byte: LEN bounds
//          the frame and the CRC validates it. On a bad CRC (or LEN > MAX_PAYLOAD) 
//          the receiver discards and rescans for the next SOF
//
// All multi-byte fields are little-endian; floats are IEEE-754.
// Take the float vx = 1.0. In IEEE-754 its 32 bits are 0x3F800000.
// There are two ways to lay those 4 bytes out:
//
//   - little-endian (least-significant byte first):   00 00 80 3F
//   - big-endian    (most-significant byte first):    3F 80 00 00
//
// UART sends a flat stream of bytes. If the two ends disagree on the order,
// the receiver reassembles garbage.
// Both boards are little-endian ARM, so payload structs are memcpy'd directly.
//
// Messages could be:
//
//   - Fire-and-forget: no reply. Pets the command watchdog. Setpoints/commands
//   - Request/reply:   exactly one reply frame, carrying the same opcode
//
// kind_of() encodes this so both the Servo2040 and the Pi can establish
// if they have to wait for a reply

#include <cstddef>
#include <cstdint>

#include "core/config.hpp"

namespace proto {

// Shared enums (these values travel on the wire)
//
// Lifecycle: OFF --Enable--> SETUP --> IDLE <--> WALK --Shutdown--> SHUTDOWN --> OFF
//            any energized state --over-current/low-voltage--> FAULT --Enable--> SETUP
// SETUP (rising) and SHUTDOWN (lowering) are uninterruptible animations; only a
// fault can break in. OFF is appended (value 5) so wire values 0..4 stay stable.
enum class State : uint8_t {
  SETUP = 0,    // RISING: stand-up animation running (de-energized rest is OFF)
  IDLE = 1,     // standing, zero velocity
  WALK = 2,     // executing a gait
  SHUTDOWN = 3, // LOWERING: sit-down animation running
  FAULT = 4,    // emergency-stopped: halted and de-energized, needs Enable to recover
  OFF = 5,      // de-energized standby: booted or fully sat down, awaiting Enable
};

enum class GaitId : uint8_t {
  TRIPOD = 0,
  WAVE = 1,
  RIPPLE = 2,
};

enum class LedMode : uint8_t {
  SOLID = 0,
  BLINK = 1,
};

// Status codes returned in reply payloads / error frames
enum class Status : uint8_t {
  REJECTED = 0x00, // understood but refused (wrong state, etc.)
  OK = 0x01,
  UNREACHABLE = 0x02, // IK target out of range
  BAD_OPCODE = 0x03,  // unknown opcode
  BAD_LENGTH = 0x04,  // payload size != expected for this opcode
};

// Opcodes
// High nibble groups by kind for readability:
//   0x0x  low-level servo/LED debug (request/reply, ack)
//   0x1x  provisioning / runtime config (request/reply, ack)
//   0x3x  fire-and-forget setpoints / commands
//   0x4x  request/reply queries
//   0xEE  board-initiated error reply
//
// Provisioning and jog are request/reply: the board applies and answers with the
// SAME opcode carrying an AckReply(Status::OK), or an Error frame if it refused
// (wrong state) or the length was bad. They are only honoured while de-energized
// (OFF/FAULT) so nothing reconfigures mid-motion; the Pi provisions at connect,
// before enable(). (0x0x/0x1x/0x2x stay reserved for further debug/config.)
enum class Opcode : uint8_t {
  // low-level debug (0x0x)
  JogServo = 0x01, // JogServoMsg -> AckReply  (raw pulse to one servo)

  // provisioning / runtime config (0x1x) -> AckReply
  ProvisionBody = 0x10,      // ProvisionBodyMsg       link lengths, heights, cadence
  ProvisionMounts = 0x11,    // ProvisionMountsMsg     per-leg mount pose (x, y, yaw)
  ProvisionDirection = 0x12, // ProvisionDirectionMsg  per-servo direction (+/-1)
  ProvisionTrim = 0x13,      // ProvisionTrimMsg       per-servo trim (deg)
  ProvisionRanges = 0x14,    // ProvisionRangesMsg     per-joint-type servo clamp
  ProvisionGaits = 0x15,     // ProvisionGaitsMsg      per-gait parameters
  ProvisionLimits = 0x16,    // ProvisionLimitsMsg     command clamps + over-current
  ProvisionBodyPose = 0x17,  // ProvisionBodyPoseMsg   body-pose limits
  ProvisionServoCal = 0x18,  // ProvisionServoCalMsg   per-servo [min,mid,max] us (HAL)
  ProvisionPins = 0x19,      // ProvisionPinsMsg       logical->physical pin map (HAL)

  // fire-and-forget (no reply; pets the watchdog)
  SetVelocity = 0x30, // SetVelocityMsg
  SetBodyPose = 0x31, // SetBodyPoseMsg
  SetGait = 0x32,     // SetGaitMsg
  Enable = 0x33,      // (no payload) run stand-up sequence -> IDLE
  Shutdown = 0x34,    // (no payload) run sit-down sequence -> SETUP
  Stop = 0x35,        // (no payload) zero velocity (soft stop)
  SetLed = 0x36,      // SetLedMsg
  Heartbeat = 0x37,   // (no payload) keepalive, pets the watchdog

  // request/reply (board answers with the same opcode)
  GetTelemetry = 0x40, // ()           -> TelemetryReply
  GetVoltage = 0x41,   // ()           -> VoltageReply
  GetCurrent = 0x42,   // ()           -> CurrentReply
  GetJoints = 0x43,    // ()           -> JointsReply
  GetBodyPose = 0x44,  // ()           -> BodyPoseReply

  // board-initiated
  Error = 0xEE, // ErrorReply
};

enum class Kind : uint8_t { FireAndForget, RequestReply };

// Which kind is this opcode. Unknown opcodes default to RequestReply so the
// router can answer with an Error frame rather than silently dropping
constexpr Kind kind_of(Opcode op) {
  switch (op) {
  case Opcode::SetVelocity:
  case Opcode::SetBodyPose:
  case Opcode::SetGait:
  case Opcode::Enable:
  case Opcode::Shutdown:
  case Opcode::Stop:
  case Opcode::SetLed:
  case Opcode::Heartbeat:
    return Kind::FireAndForget;
  default:
    return Kind::RequestReply;
  }
}

// Payloads
// Packed so sizeof() == on-wire size with no padding. The trailing
// static_asserts pin those sizes
#pragma pack(push, 1)

// Commands ("py: <..." is the Python struct format the Pi must use, "<" is
// little endian)

struct SetVelocityMsg { // py: "<fff"
  float vx;             // mm/s, body +x (forward)
  float vy;             // mm/s, body +y (left)
  float wz;             // deg/s, yaw (+ = CCW)
};

struct SetBodyPoseMsg {   // py: "<ffffff"
  float x, y, z;          // mm, body shift (z relative to standing height)
  float roll, pitch, yaw; // deg
};

struct SetGaitMsg { // py: "<B"
  uint8_t gait_id;  // proto::GaitId
};

struct SetLedMsg { // py: "<BBBBf"
  uint8_t mode;    // proto::LedMode
  uint8_t r, g, b;
  float freq_hz; // blink frequency (mode == BLINK)
};

// Low-level debug: drive one servo to a raw pulse (calibration jog). Bypasses
// the FSM/gait; energizes the rail on demand. pulse_us == 0 releases (disables)
// that channel.
struct JogServoMsg { // py: "<BH"
  uint8_t channel;   // logical channel, cfg::servo_channel(leg, joint)
  uint16_t pulse_us; // 0 = release
};

// Provisioning: the Pi pushes the full runtime config at connect, one section
// per message. Applied only while de-energized. Arrays are leg-major /
// joint-major to mirror the config layout.
struct ProvisionBodyMsg { // py: "<ffffff"
  float coxa_len, femur_len, tibia_len; // mm
  float standing_height;                // mm
  float stance_radius;                  // mm
  float cycle_time;                     // s
};

struct ProvisionMountsMsg {          // py: "<18f"  per leg: x, y, yaw_deg
  float mount[cfg::N_LEGS][3];       // [leg] = {x mm, y mm, yaw deg}
};

// Servo-kinematic map, split to mirror the config YAML: direction and trim are
// per-servo (leg-major, cfg::servo_channel order); the clamp is per-joint-type.
struct ProvisionDirectionMsg {       // py: "<18f"  per servo: direction (+/-1)
  float direction[cfg::N_SERVOS];
};

struct ProvisionTrimMsg {            // py: "<18f"  per servo: trim_deg
  float trim_deg[cfg::N_SERVOS];
};

struct ProvisionRangesMsg {          // py: "<6f"  per joint type: min_deg, max_deg
  float range[cfg::N_JOINTS][2];     // [joint] = {min_deg, max_deg}
};

struct ProvisionGaitsMsg {           // py: "<12f"  per gait: duty, step_h, stride, overlap
  float gait[3][4];                  // [gait] = {duty_factor, step_height, max_stride, overlap}
};

struct ProvisionLimitsMsg { // py: "<ffffffff"
  float lin_vel_max, ang_vel_max, vel_smooth_tau;
  float body_lin_vel_max, body_ang_vel_max, joint_vel_max;
  float current_max;          // A, over-current cutoff
  float voltage_min;          // V, low-voltage cutoff
};

struct ProvisionBodyPoseMsg { // py: "<12f"  per axis: min, max
  float x_min, x_max;         // mm
  float y_min, y_max;         // mm
  float z_min, z_max;         // mm, relative to standing height
  float roll_min, roll_max;   // deg
  float pitch_min, pitch_max; // deg
  float yaw_min, yaw_max;     // deg
};

struct ProvisionServoCalMsg {          // py: "<54H"  per servo: min, mid, max (us)
  uint16_t pulse[cfg::N_SERVOS][3];    // [channel] = {min_us, mid_us, max_us}
};

struct ProvisionPinsMsg {           // py: "<18B"  physical pin per logical channel
  uint8_t pin[cfg::N_SERVOS];
};

// Replies

struct TelemetryReply { // py: "<Bfffff"
  uint8_t state;        // proto::State
  float odom_x;         // mm
  float odom_y;         // mm
  float odom_yaw;       // deg
  float voltage;        // V
  float current;        // A
};

struct VoltageReply { // py: "<f"
  float voltage;      // V
};

struct CurrentReply { // py: "<f"
  float current;      // A
};

struct JointsReply {          // py: "<18f"
  float angle[cfg::N_SERVOS]; // servo-space deg, leg-major (leg*3 + joint)
};

// Live body pose: the interpolated value the board is slewing toward. 
// Same layout/units as SetBodyPoseMsg so a set/get round-trips 
// (z relative to standing height).
struct BodyPoseReply {    // py: "<ffffff"
  float x, y, z;          // mm, body shift (z relative to standing height)
  float roll, pitch, yaw; // deg
};

struct ErrorReply { // py: "<B"
  uint8_t status;   // proto::Status
};

// Ack for jog / provisioning: same opcode as the request, carrying the result.
struct AckReply { // py: "<B"
  uint8_t status; // proto::Status (OK on success)
};

#pragma pack(pop)

// Sizes must hold for the Python mirror to line up
static_assert(sizeof(SetVelocityMsg) == 12, "SetVelocityMsg size");
static_assert(sizeof(SetBodyPoseMsg) == 24, "SetBodyPoseMsg size");
static_assert(sizeof(SetGaitMsg) == 1, "SetGaitMsg size");
static_assert(sizeof(SetLedMsg) == 8, "SetLedMsg size");
static_assert(sizeof(JogServoMsg) == 3, "JogServoMsg size");
static_assert(sizeof(ProvisionBodyMsg) == 24, "ProvisionBodyMsg size");
static_assert(sizeof(ProvisionMountsMsg) == 72, "ProvisionMountsMsg size");
static_assert(sizeof(ProvisionDirectionMsg) == 72, "ProvisionDirectionMsg size");
static_assert(sizeof(ProvisionTrimMsg) == 72, "ProvisionTrimMsg size");
static_assert(sizeof(ProvisionRangesMsg) == 24, "ProvisionRangesMsg size");
static_assert(sizeof(ProvisionGaitsMsg) == 48, "ProvisionGaitsMsg size");
static_assert(sizeof(ProvisionLimitsMsg) == 32, "ProvisionLimitsMsg size");
static_assert(sizeof(ProvisionBodyPoseMsg) == 48, "ProvisionBodyPoseMsg size");
static_assert(sizeof(ProvisionServoCalMsg) == 108, "ProvisionServoCalMsg size");
static_assert(sizeof(ProvisionPinsMsg) == 18, "ProvisionPinsMsg size");
static_assert(sizeof(TelemetryReply) == 21, "TelemetryReply size");
static_assert(sizeof(VoltageReply) == 4, "VoltageReply size");
static_assert(sizeof(CurrentReply) == 4, "CurrentReply size");
static_assert(sizeof(JointsReply) == 72, "JointsReply size");
static_assert(sizeof(BodyPoseReply) == 24, "BodyPoseReply size");
static_assert(sizeof(ErrorReply) == 1, "ErrorReply size");
static_assert(sizeof(AckReply) == 1, "AckReply size");

// Framing (HDLC-style codec)
//
// Implements the frame layout documented at the top of this file.
// encode_frame() writes a complete frame into a caller-owned
// buffer, and FrameParser consumes bytes one at a time and reports when a
// CRC-valid frame has arrived. The CRC is an integrity guarantee: on a
// bad CRC or an implausible length byte, the parser silently resyncs
// on the next SOF

// CRC16-CCITT (poly 0x1021, init 0xFFFF), folding in one byte
constexpr uint16_t crc16_update(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (int i = 0; i < 8; ++i)
    crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                         : static_cast<uint16_t>(crc << 1);
  return crc;
}

// Write SOF | LEN | opcode | payload | CRClo | CRChi into "out" (which must
// hold at least len + 5 bytes). Returns the frame size, or 0 if the payload is
// too big
inline size_t encode_frame(uint8_t opcode, const uint8_t *payload, uint8_t len,
                           uint8_t *out) {
  if (len > cfg::MAX_PAYLOAD)
    return 0;
  uint16_t crc = 0xFFFF;
  size_t n = 0;
  out[n++] = cfg::SOF;
  out[n++] = len;
  crc = crc16_update(crc, len);
  out[n++] = opcode;
  crc = crc16_update(crc, opcode);
  for (uint8_t i = 0; i < len; ++i) {
    out[n++] = payload[i];
    crc = crc16_update(crc, payload[i]);
  }
  out[n++] = static_cast<uint8_t>(crc & 0xFF);        // CRC lo
  out[n++] = static_cast<uint8_t>((crc >> 8) & 0xFF); // CRC hi
  return n;
}

// Byte-at-a-time receiver. feed() returns true exactly once (on the byte that
// completes a valid frame) after which opcode()/payload()/length() are valid
// until the next feed()
class FrameParser {
public:
  bool feed(uint8_t b) {
    switch (phase_) {
    case Phase::Sof:
      if (b == cfg::SOF)
        phase_ = Phase::Len;
      return false;
    case Phase::Len:
      if (b > cfg::MAX_PAYLOAD) { // implausible length -> resync
        phase_ = (b == cfg::SOF) ? Phase::Len : Phase::Sof;
        return false;
      }
      length_ = b;
      crc_ = crc16_update(0xFFFF, b);
      phase_ = Phase::Opcode;
      return false;
    case Phase::Opcode:
      opcode_ = b;
      crc_ = crc16_update(crc_, b);
      idx_ = 0;
      phase_ = (length_ == 0) ? Phase::CrcLo : Phase::Payload;
      return false;
    case Phase::Payload:
      payload_[idx_++] = b;
      crc_ = crc16_update(crc_, b);
      if (idx_ >= length_)
        phase_ = Phase::CrcLo;
      return false;
    case Phase::CrcLo:
      rx_crc_ = b;
      phase_ = Phase::CrcHi;
      return false;
    case Phase::CrcHi:
      rx_crc_ |= static_cast<uint16_t>(b) << 8;
      phase_ = Phase::Sof;
      return rx_crc_ == crc_; // complete iff CRC matches
    }
    return false;
  }

  uint8_t opcode() const { return opcode_; }
  const uint8_t *payload() const { return payload_; }
  uint8_t length() const { return length_; }
  void reset() { phase_ = Phase::Sof; }

private:
  enum class Phase : uint8_t { Sof, Len, Opcode, Payload, CrcLo, CrcHi };
  Phase phase_ = Phase::Sof;
  uint8_t length_ = 0;
  uint8_t opcode_ = 0;
  uint8_t idx_ = 0;
  uint16_t crc_ = 0;    // running CRC over LEN..payload
  uint16_t rx_crc_ = 0; // CRC carried in the frame
  uint8_t payload_[cfg::MAX_PAYLOAD];
};

} // namespace proto
