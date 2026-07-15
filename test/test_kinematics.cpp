// Host test: forward/inverse kinematics.
//
// The strong invariant for a 2-link leg is the FOOT POSITION round-trip
// (FK -> IK -> FK), not the joint angles; IK may legitimately return the other
// elbow solution, but it must always reproduce the foot it was given.

#include <cstdio>

#include "check.hpp"
#include "core/config.hpp"
#include "utils/math.hpp"
#include "core/kinematics.hpp"

using math::Vec3;
using kin::IkResult;

static bool same(const Vec3& a, const Vec3& b, float eps = 0.05f) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

// Hand-computed sanity check: zero joint angles fully extend the leg radially
// (62+77+100 = 239 mm along +x in the leg frame), then the -45 deg mount rotation
// and the (82,-57) offset put leg 0's foot at ~(251, -226, 0) in the body frame.
static void fk_absolute() {
    std::printf("FK absolute value (leg 0, zero angles)\n");
    Vec3 foot = kin::forward(cfg::FRONT_RIGHT, 0, 0, 0, {}, {}, false);
    CHECK(approx(foot.x,  251.0f, 0.1f), "leg0 zero-pose foot x ~= 251");
    CHECK(approx(foot.y, -226.0f, 0.1f), "leg0 zero-pose foot y ~= -226");
    CHECK(approx(foot.z,    0.0f, 0.1f), "leg0 zero-pose foot z ~= 0");
}

static void roundtrip_no_body() {
    std::printf("FK -> IK -> FK round-trip (no body pose)\n");
    const float angles[][3] = {
        { math::deg2rad(0),   math::deg2rad(-30), math::deg2rad(-60) },
        { math::deg2rad(10),  math::deg2rad(-20), math::deg2rad(-70) },
        { math::deg2rad(-15), math::deg2rad(-40), math::deg2rad(-50) },
    };
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
        for (auto& a : angles) {
            Vec3 foot = kin::forward(leg, a[0], a[1], a[2], {}, {}, false);
            IkResult r = kin::inverse(leg, foot, {}, {});
            char m[64];
            std::snprintf(m, sizeof m, "leg %d: IK reachable", leg);
            CHECK(r.ok, m);
            Vec3 foot2 = kin::forward(leg, r.coxa, r.femur, r.tibia, {}, {}, false);
            std::snprintf(m, sizeof m, "leg %d: foot reproduced", leg);
            CHECK(same(foot, foot2), m);
        }
    }
}

static void roundtrip_with_body() {
    std::printf("FK -> IK -> FK round-trip (with body pose)\n");
    Vec3 body_pos(5.0f, -3.0f, 2.0f);
    Vec3 body_rpy(math::deg2rad(5), math::deg2rad(4), math::deg2rad(10));
    float a[3] = { math::deg2rad(5), math::deg2rad(-25), math::deg2rad(-65) };
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
        Vec3 world = kin::forward(leg, a[0], a[1], a[2], body_pos, body_rpy, true);
        IkResult r = kin::inverse(leg, world, body_pos, body_rpy);
        char m[64];
        std::snprintf(m, sizeof m, "leg %d: IK reachable with body pose", leg);
        CHECK(r.ok, m);
        Vec3 world2 = kin::forward(leg, r.coxa, r.femur, r.tibia, body_pos, body_rpy, true);
        std::snprintf(m, sizeof m, "leg %d: world foot reproduced", leg);
        CHECK(same(world, world2), m);
    }
}

static void unreachable() {
    std::printf("unreachable target rejected\n");
    IkResult r = kin::inverse(0, Vec3(1000, 1000, 500), {}, {});
    CHECK(!r.ok, "far target reports unreachable");
}

int main() {
    fk_absolute();
    roundtrip_no_body();
    roundtrip_with_body();
    unreachable();
    return test_summary("kinematics");
}
