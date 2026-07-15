// Host test: the robot manager + the opcode router.
//
// A fake hw::Interface stands in for the board. We check the stand-up / sit-down
// animations, the state machine, the comms watchdog, the over-current FAULT trip
// and recovery, and that frames decoded through the real parser + router drive
// the robot and produce well-formed replies.

#include <cstdio>
#include <cstring>
#include <cmath>

#include "check.hpp"
#include "core/config.hpp"
#include "communication/protocol.hpp"
#include "communication/hardware.hpp"
#include "core/robot.hpp"
#include "core/router.hpp"

using proto::State;
using proto::Opcode;

static const float DT = 1.0f / cfg::CONTROL_RATE_HZ;

struct FakeHW : hw::Interface {
    float servos[cfg::N_SERVOS] = {};
    bool  wrote = false;
    bool  powered = false;
    float volts = 7.4f;
    float amps = 0.5f;
    int   led_calls = 0;
    int   led_updates = 0;

    void write_servos(const float (&deg)[cfg::N_SERVOS]) override {
        for (int i = 0; i < cfg::N_SERVOS; ++i) servos[i] = deg[i];
        wrote = true;
    }
    void read_servos(float (&deg)[cfg::N_SERVOS]) override {
        for (int i = 0; i < cfg::N_SERVOS; ++i) deg[i] = servos[i];
    }
    void set_power(bool on) override { powered = on; }
    float read_voltage() override { return volts; }
    float read_current() override { return amps; }
    void set_led(uint8_t, uint8_t, uint8_t, uint8_t, float) override { ++led_calls; }
    void update_leds() override { ++led_updates; }
};

static void pump(robot::Robot& r, int n) { for (int i = 0; i < n; ++i) r.update(DT); }

// Encode a frame, run it through the real parser, dispatch it. Returns reply len.
static uint8_t route(robot::Robot& r, uint8_t op, const void* pl, uint8_t len,
                     uint8_t* reply, uint8_t& reply_op) {
    uint8_t frame[cfg::MAX_PAYLOAD + 5];
    const size_t n = proto::encode_frame(op, static_cast<const uint8_t*>(pl), len, frame);
    proto::FrameParser p;
    for (size_t i = 0; i < n; ++i)
        if (p.feed(frame[i]))
            return router::dispatch(r, p.opcode(), p.payload(), p.length(), reply, reply_op);
    return 0;
}

static bool servos_in_range(const robot::Robot& r) {
    for (int leg = 0; leg < cfg::N_LEGS; ++leg)
        for (int j = 0; j < cfg::N_JOINTS; ++j) {
            const float d = r.joints().deg[cfg::servo_channel(leg, j)];
            if (d < cfg::RANGE[j].min_deg - 1e-3f || d > cfg::RANGE[j].max_deg + 1e-3f)
                return false;
        }
    return true;
}

static bool femur_at_max(const robot::Robot& r) {
    for (int leg = 0; leg < cfg::N_LEGS; ++leg)
        if (!approx(r.joints().deg[cfg::servo_channel(leg, cfg::FEMUR)],
                    cfg::RANGE[cfg::FEMUR].max_deg, 1e-2f))
            return false;
    return true;
}

static void stand_up() {
    std::printf("boot OFF (de-energized) -> enable -> stand-up animation -> IDLE\n");
    FakeHW hw;
    robot::Robot r(hw);
    CHECK(r.state() == State::OFF, "boots in OFF");
    CHECK(!hw.powered, "de-energized at boot");
    r.update(DT);
    CHECK(!hw.wrote, "no servo writes before enable");

    r.enable();
    CHECK(hw.powered, "powered after enable");
    CHECK(r.state() == State::SETUP, "stand-up animation runs in SETUP");
    r.update(DT);
    CHECK(r.state() == State::SETUP, "still animating a few ticks in");

    r.shutdown();  // uninterruptible: must not abort an in-progress stand-up
    CHECK(r.state() == State::SETUP, "shutdown ignored mid stand-up");

    pump(r, 500);
    CHECK(r.state() == State::IDLE, "stand-up completes -> IDLE");
    CHECK(servos_in_range(r), "standing servo angles within limits");
}

static void walk_and_stop() {
    std::printf("walk then stop, with odometry\n");
    FakeHW hw;
    robot::Robot r(hw);
    r.enable();
    pump(r, 500);
    CHECK(r.state() == State::IDLE, "standing");

    for (int i = 0; i < 60; ++i) { r.set_velocity(120.0f, 0.0f, 0.0f); r.update(DT); }
    CHECK(r.state() == State::WALK, "nonzero command -> WALK");
    float x, y, yaw; r.odometry(x, y, yaw);
    CHECK(x > 50.0f, "odometry advances forward");

    for (int i = 0; i < 60; ++i) { r.set_velocity(0.0f, 0.0f, 0.0f); r.update(DT); }
    CHECK(r.state() == State::IDLE, "zero command -> IDLE");
    float x2, y2, yaw2; r.odometry(x2, y2, yaw2);
    r.set_velocity(0.0f, 0.0f, 0.0f); r.update(DT);
    float x3, y3, yaw3; r.odometry(x3, y3, yaw3);
    CHECK(approx(x2, x3, 1e-3f) && approx(y2, y3, 1e-3f), "odometry frozen when stopped");
}

static void watchdog() {
    std::printf("watchdog: lost comms stops the robot but keeps it standing\n");
    FakeHW hw;
    robot::Robot r(hw);
    r.enable();
    pump(r, 500);
    for (int i = 0; i < 30; ++i) { r.set_velocity(120.0f, 0.0f, 0.0f); r.update(DT); }
    CHECK(r.state() == State::WALK, "walking with a live link");
    for (int i = 0; i < 40; ++i) r.update(DT);   // 0.8 s of silence > 0.5 s timeout
    CHECK(r.state() == State::IDLE, "watchdog stopped the robot");
    CHECK(hw.powered, "still enabled (standing, not shut down)");
}

static void over_current_fault() {
    std::printf("over-current -> FAULT (emergency stop), then recover via enable\n");
    FakeHW hw;
    robot::Robot r(hw);
    r.enable();
    pump(r, 500);
    r.set_velocity(120.0f, 0.0f, 0.0f);
    r.update(DT);
    CHECK(r.state() == State::WALK, "walking");

    hw.amps = 15.0f;                              // over CURRENT_MAX (12 A)
    r.update(DT);
    CHECK(r.state() == State::FAULT, "over-current -> FAULT");
    CHECK(!hw.powered, "power cut on FAULT");
    hw.amps = 0.5f;
    r.update(DT); r.update(DT);
    CHECK(r.state() == State::FAULT, "FAULT is sticky");

    r.enable();                                   // recover
    pump(r, 500);
    CHECK(r.state() == State::IDLE, "enable recovers from FAULT -> stands up -> IDLE");
    CHECK(hw.powered, "re-energized after recovery");
}

static void low_voltage_fault() {
    std::printf("low-voltage -> FAULT (emergency stop)\n");
    FakeHW hw;
    robot::Robot r(hw);
    r.enable();
    pump(r, 500);
    CHECK(r.state() == State::IDLE, "idle after stand-up");

    hw.volts = 4.5f;                              // under VOLTAGE_MIN (5 V)
    r.update(DT);
    CHECK(r.state() == State::FAULT, "low-voltage -> FAULT");
    CHECK(!hw.powered, "power cut on FAULT");
}

static void sit_down() {
    std::printf("walk -> shutdown: sit-down animation -> curled, de-energized OFF\n");
    FakeHW hw;
    robot::Robot r(hw);
    r.enable();
    pump(r, 500);
    for (int i = 0; i < 20; ++i) { r.set_velocity(100.0f, 0.0f, 30.0f); r.update(DT); }
    CHECK(r.state() == State::WALK, "walking before shutdown");

    r.shutdown();
    CHECK(r.state() == State::SHUTDOWN, "shutdown -> SHUTDOWN (sit-down running)");
    CHECK(hw.powered, "still powered while sitting down");
    r.enable();  // uninterruptible: must not abort an in-progress sit-down
    CHECK(r.state() == State::SHUTDOWN, "enable ignored mid sit-down");
    pump(r, 500);
    CHECK(r.state() == State::OFF, "sit-down completes -> OFF");
    CHECK(!hw.powered, "power cut once curled");
    CHECK(femur_at_max(r), "legs curled at the end (femur to max)");
}

static void routing() {
    std::printf("router: frames drive the robot and produce replies\n");
    FakeHW hw;
    hw.volts = 8.1f;
    hw.amps = 1.25f;
    robot::Robot r(hw);
    uint8_t reply[cfg::MAX_PAYLOAD];
    uint8_t rop = 0;

    route(r, uint8_t(Opcode::Enable), nullptr, 0, reply, rop);
    CHECK(r.state() == State::SETUP, "Enable frame starts stand-up (SETUP)");
    pump(r, 500);
    CHECK(r.state() == State::IDLE, "stands up to IDLE");

    proto::SetVelocityMsg v{120.0f, 0.0f, 0.0f};
    for (int i = 0; i < 40; ++i) {
        route(r, uint8_t(Opcode::SetVelocity), &v, sizeof v, reply, rop);
        r.update(DT);
    }
    CHECK(r.state() == State::WALK, "SetVelocity frame -> walking");

    uint8_t n = route(r, uint8_t(Opcode::GetVoltage), nullptr, 0, reply, rop);
    CHECK(n == sizeof(proto::VoltageReply), "voltage reply length");
    CHECK(rop == uint8_t(Opcode::GetVoltage), "voltage reply carries its opcode");
    proto::VoltageReply vr; std::memcpy(&vr, reply, sizeof vr);
    CHECK(approx(vr.voltage, 8.1f, 1e-4f), "voltage reply value");

    n = route(r, uint8_t(Opcode::GetTelemetry), nullptr, 0, reply, rop);
    CHECK(n == sizeof(proto::TelemetryReply), "telemetry reply length");
    proto::TelemetryReply t; std::memcpy(&t, reply, sizeof t);
    CHECK(t.state == uint8_t(State::WALK), "telemetry reports WALK");
    CHECK(approx(t.current, 1.25f, 1e-4f), "telemetry current");

    n = route(r, uint8_t(Opcode::GetJoints), nullptr, 0, reply, rop);
    CHECK(n == sizeof(proto::JointsReply), "joints reply length");

    n = route(r, 0x7F, nullptr, 0, reply, rop);   // unknown opcode
    CHECK(rop == uint8_t(Opcode::Error), "unknown opcode -> Error frame");
    proto::ErrorReply er; std::memcpy(&er, reply, sizeof er);
    CHECK(er.status == uint8_t(proto::Status::BAD_OPCODE), "error status = BAD_OPCODE");
}

int main() {
    stand_up();
    walk_and_stop();
    watchdog();
    over_current_fault();
    low_voltage_fault();
    sit_down();
    routing();
    return test_summary("robot");
}
