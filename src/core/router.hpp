#pragma once
//
// Given one decoded frame (opcode + payload) it drives the robot's intent API 
// and builds a reply payload if needed.
// 


#include <cstddef>
#include <cstdint>
#include <cstring>

#include "communication/protocol.hpp"
#include "utils/math.hpp"
#include "core/gait.hpp"
#include "core/robot.hpp"

namespace router {

// Act on one decoded frame. For a request/reply opcode, writes the reply payload
// into out[], sets reply_op to the opcode to frame it with, and returns the reply
// length. Fire-and-forget opcodes return 0. A malformed request replies Error; a
// malformed fire-and-forget frame is ignored; an unknown opcode replies Error.
inline uint8_t dispatch(robot::Robot &r, uint8_t opcode, const uint8_t *payload,
                        uint8_t len, uint8_t *out, uint8_t &reply_op) {
  using proto::Opcode;
  using proto::Status;

  auto error = [&](Status s) -> uint8_t {
    proto::ErrorReply e{static_cast<uint8_t>(s)};
    reply_op = static_cast<uint8_t>(Opcode::Error);
    std::memcpy(out, &e, sizeof e);
    return sizeof e;
  };
  auto reply = [&](const void *msg, uint8_t n) -> uint8_t {
    reply_op = opcode;
    std::memcpy(out, msg, n);
    return n;
  };
  // Positive ack for jog / provisioning: same opcode, one status byte
  auto ack = [&](Status s) -> uint8_t {
    proto::AckReply a{static_cast<uint8_t>(s)};
    reply_op = opcode;
    std::memcpy(out, &a, sizeof a);
    return sizeof a;
  };

  switch (static_cast<Opcode>(opcode)) {

  // fire-and-forget: apply and stay silent (ignore malformed length)
  case Opcode::SetVelocity: {
    if (len != sizeof(proto::SetVelocityMsg)) return 0;
    proto::SetVelocityMsg m;
    std::memcpy(&m, payload, sizeof m);
    r.set_velocity(m.vx, m.vy, m.wz);
    return 0;
  }
  case Opcode::SetBodyPose: {
    if (len != sizeof(proto::SetBodyPoseMsg)) return 0;
    proto::SetBodyPoseMsg m;
    std::memcpy(&m, payload, sizeof m);
    r.set_body_pose(m.x, m.y, m.z, m.roll, m.pitch, m.yaw);
    return 0;
  }
  case Opcode::SetGait: {
    if (len != sizeof(proto::SetGaitMsg)) return 0;
    proto::SetGaitMsg m;
    std::memcpy(&m, payload, sizeof m);
    if (m.gait_id < gait::N_GAITS)
      r.set_gait(static_cast<gait::GaitId>(m.gait_id));
    return 0;
  }
  case Opcode::SetLed: {
    if (len != sizeof(proto::SetLedMsg)) return 0;
    proto::SetLedMsg m;
    std::memcpy(&m, payload, sizeof m);
    r.set_led(m.mode, m.r, m.g, m.b, m.freq_hz);
    return 0;
  }
  case Opcode::Enable: {
    r.enable();
    return 0;
  }
  case Opcode::Shutdown: {
    r.shutdown();
    return 0;
  }
  case Opcode::Stop: {
    r.stop();
    return 0;
  }
  case Opcode::Heartbeat: {
    r.heartbeat(); 
    return 0;
  }

  // low-level debug / provisioning: apply and ack. Only while de-energized; a
  // wrong length replies BAD_LENGTH, an energized robot replies rejected
  case Opcode::JogServo: {
    if (len != sizeof(proto::JogServoMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::JogServoMsg m;
    std::memcpy(&m, payload, sizeof m);
    if (m.channel >= cfg::N_SERVOS) return error(Status::REJECTED);
    r.jog_servo(m.channel, m.pulse_us);
    return ack(Status::OK);
  }
  case Opcode::ProvisionBody: {
    if (len != sizeof(proto::ProvisionBodyMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionBodyMsg m;
    std::memcpy(&m, payload, sizeof m);
    cfg::COXA_LEN = m.coxa_len;
    cfg::COXA_OFFSET = m.coxa_offset;
    cfg::FEMUR_LEN = m.femur_len;
    cfg::TIBIA_LEN = m.tibia_len;
    cfg::STANDING_HEIGHT = m.standing_height;
    cfg::STANCE_RADIUS = m.stance_radius;
    cfg::CYCLE_TIME = m.cycle_time;
    r.reconfigure(); // stance radius feeds the neutral stance
    return ack(Status::OK);
  }
  case Opcode::ProvisionMounts: {
    if (len != sizeof(proto::ProvisionMountsMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionMountsMsg m;
    std::memcpy(&m, payload, sizeof m);
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
      cfg::MOUNT[leg].x = m.mount[leg][0];
      cfg::MOUNT[leg].y = m.mount[leg][1];
      cfg::MOUNT[leg].z = m.mount[leg][2];
      cfg::MOUNT[leg].yaw_deg = m.mount[leg][3];
    }
    r.reconfigure();
    return ack(Status::OK);
  }
  case Opcode::ProvisionDirection: {
    if (len != sizeof(proto::ProvisionDirectionMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionDirectionMsg m;
    std::memcpy(&m, payload, sizeof m);
    for (int ch = 0; ch < cfg::N_SERVOS; ++ch) cfg::DIRECTION[ch] = m.direction[ch];
    r.reconfigure(); // the servo map moves the de-energized rest pose
    return ack(Status::OK);
  }
  case Opcode::ProvisionTrim: {
    if (len != sizeof(proto::ProvisionTrimMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionTrimMsg m;
    std::memcpy(&m, payload, sizeof m);
    for (int ch = 0; ch < cfg::N_SERVOS; ++ch) cfg::TRIM_DEG[ch] = m.trim_deg[ch];
    r.reconfigure();
    return ack(Status::OK);
  }
  case Opcode::ProvisionRanges: {
    if (len != sizeof(proto::ProvisionRangesMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionRangesMsg m;
    std::memcpy(&m, payload, sizeof m);
    for (int j = 0; j < cfg::N_JOINTS; ++j) {
      cfg::RANGE[j].min_deg = m.range[j][0];
      cfg::RANGE[j].max_deg = m.range[j][1];
    }
    r.reconfigure(); // the joint maxima ARE the folded pose
    return ack(Status::OK);
  }
  case Opcode::ProvisionGaits: {
    if (len != sizeof(proto::ProvisionGaitsMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionGaitsMsg m;
    std::memcpy(&m, payload, sizeof m);
    for (int g = 0; g < 3; ++g) {
      cfg::GAIT[g].duty_factor = m.gait[g][0];
      cfg::GAIT[g].step_height = m.gait[g][1];
      cfg::GAIT[g].max_stride = m.gait[g][2];
      cfg::GAIT[g].overlap = m.gait[g][3];
    }
    return ack(Status::OK);
  }
  case Opcode::ProvisionLimits: {
    if (len != sizeof(proto::ProvisionLimitsMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionLimitsMsg m;
    std::memcpy(&m, payload, sizeof m);
    cfg::LIN_VEL_MAX = m.lin_vel_max;
    cfg::ANG_VEL_MAX = m.ang_vel_max;
    cfg::VEL_SMOOTH_TAU = m.vel_smooth_tau;
    cfg::BODY_LIN_VEL_MAX = m.body_lin_vel_max;
    cfg::BODY_ANG_VEL_MAX = m.body_ang_vel_max;
    cfg::JOINT_VEL_MAX = m.joint_vel_max;
    cfg::CURRENT_MAX = m.current_max;
    cfg::VOLTAGE_MIN = m.voltage_min;
    return ack(Status::OK);
  }
  case Opcode::ProvisionBodyPose: {
    if (len != sizeof(proto::ProvisionBodyPoseMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionBodyPoseMsg m;
    std::memcpy(&m, payload, sizeof m);
    cfg::BODY_POSE.x_min = m.x_min;         cfg::BODY_POSE.x_max = m.x_max;
    cfg::BODY_POSE.y_min = m.y_min;         cfg::BODY_POSE.y_max = m.y_max;
    cfg::BODY_POSE.z_min = m.z_min;         cfg::BODY_POSE.z_max = m.z_max;
    cfg::BODY_POSE.roll_min = m.roll_min;   cfg::BODY_POSE.roll_max = m.roll_max;
    cfg::BODY_POSE.pitch_min = m.pitch_min; cfg::BODY_POSE.pitch_max = m.pitch_max;
    cfg::BODY_POSE.yaw_min = m.yaw_min;     cfg::BODY_POSE.yaw_max = m.yaw_max;
    return ack(Status::OK);
  }
  case Opcode::ProvisionServoCal: {
    if (len != sizeof(proto::ProvisionServoCalMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionServoCalMsg m;
    std::memcpy(&m, payload, sizeof m);
    for (int ch = 0; ch < cfg::N_SERVOS; ++ch)
      r.set_servo_calibration(ch, m.pulse[ch][0], m.pulse[ch][1], m.pulse[ch][2]);
    return ack(Status::OK);
  }
  case Opcode::ProvisionPins: {
    if (len != sizeof(proto::ProvisionPinsMsg)) return error(Status::BAD_LENGTH);
    if (!r.provisionable()) return error(Status::REJECTED);
    proto::ProvisionPinsMsg m;
    std::memcpy(&m, payload, sizeof m);
    for (int ch = 0; ch < cfg::N_SERVOS; ++ch)
      r.set_servo_pin(ch, m.pin[ch]);
    return ack(Status::OK);
  }

  // request/reply: answer with the same opcode
  case Opcode::GetTelemetry: {
    proto::TelemetryReply t{};
    float ox, oy, oyaw;
    r.odometry(ox, oy, oyaw);
    t.state = static_cast<uint8_t>(r.state());
    t.odom_x = ox;
    t.odom_y = oy;
    t.odom_yaw = math::rad2deg(oyaw); // wire carries degrees
    t.voltage = r.voltage();
    t.current = r.current();
    return reply(&t, sizeof t);
  }
  case Opcode::GetVoltage: {
    proto::VoltageReply v{r.voltage()};
    return reply(&v, sizeof v);
  }
  case Opcode::GetCurrent: {
    proto::CurrentReply c{r.current()};
    return reply(&c, sizeof c);
  }
  case Opcode::GetJoints: {
    proto::JointsReply j;
    std::memcpy(j.angle, r.joints().deg, sizeof j.angle);
    return reply(&j, sizeof j);
  }
  case Opcode::GetBodyPose: {
    proto::BodyPoseReply p{};
    r.body_pose(p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
    return reply(&p, sizeof p);
  }

  default:
    return error(Status::BAD_OPCODE);
  }
}

} // namespace router
