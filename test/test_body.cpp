// Host test: servo-frame translation (robot::resolve etc., formerly the body
// layer) - foot targets + body pose -> servo angles.
//
// Checks the calibration map is invertible, that it clamps to the joint limits,
// and that a full resolve of the neutral standing pose is reachable, in range,
// and round-trips (unmap -> FK reproduces the foot targets).

#include <cstdio>
#include <cmath>

#include "check.hpp"
#include "core/config.hpp"
#include "utils/math.hpp"
#include "core/kinematics.hpp"
#include "core/gait.hpp"   // neutral stance geometry
#include "core/robot.hpp"  // servo-frame translation lives here now

using math::Vec3;

static bool same(const Vec3& a, const Vec3& b, float eps = 0.05f) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

static void servo_map_roundtrip() {
    std::printf("servo map <-> unmap are inverse (inside the clamp range)\n");
    // Mid-range kinematic angles, chosen to land inside each servo's limits.
    const float rads[cfg::N_JOINTS] = {
        math::deg2rad(10), math::deg2rad(-20), math::deg2rad(-70) };
    for (int j = 0; j < cfg::N_JOINTS; ++j) {
        const float sd   = robot::map_servo(0, j, rads[j]);
        const float back = robot::unmap_servo(0, j, sd);
        CHECK(approx(back, rads[j], 1e-4f), "unmap(map(x)) == x");
    }
}

static void servo_clamp() {
    std::printf("servo mapping clamps to the joint limits\n");
    // FEMUR: direction +1, trim 0, range [-90, 90].
    CHECK(approx(robot::map_servo(0, cfg::FEMUR, math::deg2rad(200.0f)),   90.0f, 1e-3f), "over-max clamps to max");
    CHECK(approx(robot::map_servo(0, cfg::FEMUR, math::deg2rad(-200.0f)), -90.0f, 1e-3f), "under-min clamps to min");
}

static void resolve_neutral() {
    std::printf("resolve neutral standing pose: reachable, in range, round-trips\n");
    gait::Generator g;
    Vec3 feet[cfg::N_LEGS];
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) feet[leg] = g.neutral(leg);

    const Vec3 body_pos(0.0f, 0.0f, cfg::STANDING_HEIGHT);
    const Vec3 body_rpy(0.0f, 0.0f, 0.0f);
    robot::ServoAngles s;
    CHECK(robot::resolve(feet, body_pos, body_rpy, s), "all legs reachable");

    bool in_range = true;
    for (int leg = 0; leg < cfg::N_LEGS; ++leg)
        for (int j = 0; j < cfg::N_JOINTS; ++j) {
            const float d = s.deg[cfg::servo_channel(leg, j)];
            if (d < cfg::RANGE[j].min_deg - 1e-3f || d > cfg::RANGE[j].max_deg + 1e-3f)
                in_range = false;
        }
    CHECK(in_range, "all servo angles within limits");

    bool rt = true;
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
        const float c = robot::unmap_servo(leg, cfg::COXA,  s.deg[cfg::servo_channel(leg, cfg::COXA)]);
        const float f = robot::unmap_servo(leg, cfg::FEMUR, s.deg[cfg::servo_channel(leg, cfg::FEMUR)]);
        const float t = robot::unmap_servo(leg, cfg::TIBIA, s.deg[cfg::servo_channel(leg, cfg::TIBIA)]);
        const Vec3 world = kin::forward(leg, c, f, t, body_pos, body_rpy, true);
        if (!same(world, feet[leg])) rt = false;
    }
    CHECK(rt, "unmap -> FK reproduces the foot targets");
}

static void resolve_unreachable() {
    std::printf("resolve reports an unreachable foot\n");
    gait::Generator g;
    Vec3 feet[cfg::N_LEGS];
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) feet[leg] = g.neutral(leg);
    feet[cfg::MIDDLE_LEFT] = Vec3(1000.0f, 1000.0f, 0.0f);   // far out of reach

    const Vec3 body_pos(0.0f, 0.0f, cfg::STANDING_HEIGHT);
    const Vec3 body_rpy(0.0f, 0.0f, 0.0f);
    robot::ServoAngles s;
    CHECK(!robot::resolve(feet, body_pos, body_rpy, s), "one unreachable leg => resolve false");
}

int main() {
    servo_map_roundtrip();
    servo_clamp();
    resolve_neutral();
    resolve_unreachable();
    return test_summary("body");
}
