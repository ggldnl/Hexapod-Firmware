// Host test: gait generator behaviour.
//
// Pins down the properties we actually care about:
//   - zero command  => perfectly still, no foot ever leaves the ground;
//   - stride grows linearly with commanded speed (constant cadence);
//   - the reachability clamp caps speed while preserving the turn radius;
//   - stopping, restarting and switching gait never jerk a foot up or down.

#include <cstdio>
#include <cmath>

#include "check.hpp"
#include "core/config.hpp"
#include "core/gait.hpp"

using math::Vec3;
using gait::Generator;
using gait::GaitId;

static const float DT = 1.0f / cfg::CONTROL_RATE_HZ;   // 50 Hz control tick

static bool same(const Vec3& a, const Vec3& b, float eps = 0.05f) {
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

// Neutral foot points sit STANCE_RADIUS out from each mount, flat on the ground.
static void neutral_geometry() {
    std::printf("neutral stance: STANCE_RADIUS from each mount, on the ground\n");
    Generator g;
    for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
        const Vec3 n = g.neutral(leg);
        const float dx = n.x - cfg::MOUNT[leg].x, dy = n.y - cfg::MOUNT[leg].y;
        CHECK(approx(std::sqrt(dx * dx + dy * dy), cfg::STANCE_RADIUS, 0.01f), "radius = STANCE_RADIUS");
        CHECK(approx(n.z, 0.0f, 1e-6f), "on the ground");
    }
}

// The headline requirement: a fresh generator given zero velocity must hold every
// foot at neutral, on the ground, forever, for every gait.
static void still_from_rest() {
    std::printf("zero command from rest: perfectly still, no foot lift (all gaits)\n");
    const GaitId gaits[] = { gait::TRIPOD, gait::WAVE, gait::RIPPLE };
    const char* names[]  = { "tripod", "wave", "ripple" };
    for (int gi = 0; gi < 3; ++gi) {
        Generator g;
        g.set_gait(gaits[gi]);
        const float phase0 = g.phase();
        bool at_neutral = true, flat = true, frozen = true, quiet = true;
        for (int i = 0; i < 200; ++i) {
            const Generator::Feet f = g.update(DT, 0.0f, 0.0f, 0.0f);
            for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
                if (!same(f.pos[leg], g.neutral(leg))) at_neutral = false;
                if (std::fabs(f.pos[leg].z) > 1e-6f)   flat = false;
            }
            if (g.phase() != phase0) frozen = false;
            if (g.achieved_vx() != 0.0f || g.achieved_vy() != 0.0f ||
                g.achieved_yaw_rate() != 0.0f) quiet = false;
        }
        char m[80];
        std::snprintf(m, sizeof m, "%s: every foot stays at neutral", names[gi]);   CHECK(at_neutral, m);
        std::snprintf(m, sizeof m, "%s: no foot ever leaves the ground", names[gi]); CHECK(flat, m);
        std::snprintf(m, sizeof m, "%s: phase never advances", names[gi]);           CHECK(frozen, m);
        std::snprintf(m, sizeof m, "%s: achieved twist stays zero", names[gi]);      CHECK(quiet, m);
    }
}

// After walking (and turning), a zero command must bring the robot to rest at
// neutral and keep it there.
static void still_after_walk() {
    std::printf("walk + turn, then zero command: settles at neutral and holds\n");
    Generator g;
    g.set_gait(gait::TRIPOD);
    for (int i = 0; i < 80; ++i) g.update(DT, 150.0f, 0.0f, 20.0f);   // ~2 cycles walking+turning

    bool settled = true, flat = true, frozen = true;
    float p_prev = 0.0f; bool have_prev = false;
    for (int i = 0; i < 60; ++i) {
        const Generator::Feet f = g.update(DT, 0.0f, 0.0f, 0.0f);
        for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
            if (!same(f.pos[leg], g.neutral(leg))) settled = false;
            if (std::fabs(f.pos[leg].z) > 1e-6f)   flat = false;
        }
        if (have_prev && g.phase() != p_prev) frozen = false;
        p_prev = g.phase(); have_prev = true;
    }
    CHECK(settled, "all feet at neutral after stopping");
    CHECK(flat,    "no foot lifts after stopping");
    CHECK(frozen,  "phase frozen after stopping");
}

// Horizontal travel of one foot through its stance == its stride length. Stance
// only: the swing deliberately overshoots the stride ends a little.
static float x_span_one_cycle(Generator& g, float vx) {
    g.reset();
    const int N = 3600;
    const float dt = cfg::CYCLE_TIME / N;
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < N; ++i) {
        const Generator::Feet f = g.update(dt, vx, 0.0f, 0.0f);
        if (!g.in_stance(cfg::FRONT_RIGHT)) continue;
        const float x = f.pos[cfg::FRONT_RIGHT].x;
        lo = std::fmin(lo, x); hi = std::fmax(hi, x);
    }
    return hi - lo;
}

// Constant cadence => stride scales linearly with commanded speed.
static void stride_scales() {
    std::printf("stride grows linearly with speed (below the clamp)\n");
    Generator g;
    g.set_gait(gait::TRIPOD);
    const float duty = cfg::GAIT[gait::TRIPOD].duty_factor;
    const float s1 = x_span_one_cycle(g, 100.0f);
    const float s2 = x_span_one_cycle(g, 200.0f);
    CHECK(approx(s1, 100.0f * duty * cfg::CYCLE_TIME, 0.5f), "span at v=100 = v*duty*cycle_time");
    CHECK(approx(s2, 200.0f * duty * cfg::CYCLE_TIME, 0.5f), "span at v=200 = v*duty*cycle_time");
    CHECK(approx(s2, 2.0f * s1, 0.5f), "doubling speed doubles stride");
}

// The stride clamp is the only speed cap, and it scales the whole twist so the
// commanded turn radius survives.
static void clamp_preserves_radius() {
    std::printf("stride clamp caps speed and preserves turn radius\n");
    const float duty = cfg::GAIT[gait::TRIPOD].duty_factor;

    // Pure straight line: top speed = max_stride / stance_duration.
    {
        Generator g; g.set_gait(gait::TRIPOD);
        g.update(DT, 400.0f, 0.0f, 0.0f);
        const float cap = cfg::GAIT[gait::TRIPOD].max_stride / (duty * cfg::CYCLE_TIME);
        CHECK(approx(g.achieved_vx(), cap, 0.5f), "straight top speed = max_stride/stance_duration");
        CHECK(g.achieved_vx() < 400.0f, "clamp engaged");
    }
    // Combined twist: linear and angular scaled by the same factor.
    {
        Generator g; g.set_gait(gait::TRIPOD);
        const float vx = 400.0f, wz = 40.0f;
        g.update(DT, vx, 0.0f, wz);
        CHECK(g.achieved_vx() < vx, "twist scaled down (saturated)");
        CHECK(approx(g.achieved_vx() / vx, g.achieved_yaw_rate() / wz, 1e-4f),
              "v and w scaled equally => turn radius preserved");
    }
}

static const GaitId GAITS[] = { gait::TRIPOD, gait::WAVE, gait::RIPPLE };
static const char* NAMES[]  = { "tripod", "wave", "ripple" };
static const int CYCLE_TICKS = int(cfg::CYCLE_TIME / DT + 0.5f);

// Largest per-tick change of any foot's height over a cycle of steady walking:
// the yardstick for "no jump" in the transition tests below.
static float steady_dz(GaitId gid, float vx) {
    Generator g; g.set_gait(gid);
    float prev[cfg::N_LEGS] = {}, worst = 0.0f;
    for (int i = 0; i < 2 * CYCLE_TICKS; ++i) {
        const Generator::Feet f = g.update(DT, vx, 0.0f, 0.0f);
        for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
            if (i >= CYCLE_TICKS) worst = std::fmax(worst, std::fabs(f.pos[leg].z - prev[leg]));
            prev[leg] = f.pos[leg].z;
        }
    }
    return worst;
}

// Tracks every foot's height across ticks and flags any change bigger than limit.
struct HeightWatch {
    float prev[cfg::N_LEGS] = {};
    float limit;
    explicit HeightWatch(float lim) : limit(lim) {}
    bool see(const Generator::Feet& f) {
        bool ok = true;
        for (int leg = 0; leg < cfg::N_LEGS; ++leg) {
            if (std::fabs(f.pos[leg].z - prev[leg]) > limit) ok = false;
            prev[leg] = f.pos[leg].z;
        }
        return ok;
    }
};

// Stop at every point of the cycle, then start again: the swings in progress land
// before the cycle freezes (within one swing), the robot rests on all feet, and
// neither the stop nor the restart moves a foot vertically faster than walking does.
static void stop_and_restart_on_the_ground() {
    std::printf("stop/restart from any phase: feet land first, no height jumps (all gaits)\n");
    const float v = 150.0f;
    for (int gi = 0; gi < 3; ++gi) {
        const float limit = steady_dz(GAITS[gi], v) * 1.05f + 1e-3f;
        const float swing = (1.0f - cfg::GAIT[GAITS[gi]].duty_factor) * cfg::CYCLE_TIME;
        const int max_stop_ticks = int(swing / DT) + 1;
        bool smooth_stop = true, prompt = true, at_rest = true, smooth_start = true;
        for (int k = 0; k < CYCLE_TICKS; ++k) {
            Generator g; g.set_gait(GAITS[gi]);
            HeightWatch w(limit);
            for (int i = 0; i < 2 * CYCLE_TICKS + k; ++i) w.see(g.update(DT, v, 0.0f, 0.0f));

            int ticks = 0;
            do {
                if (!w.see(g.update(DT, 0.0f, 0.0f, 0.0f))) smooth_stop = false;
            } while (g.walking() && ++ticks <= max_stop_ticks);
            if (g.walking()) prompt = false;

            const float frozen = g.phase();
            for (int i = 0; i < 10; ++i) {
                const Generator::Feet f = g.update(DT, 0.0f, 0.0f, 0.0f);
                if (!w.see(f)) smooth_stop = false;
                for (int leg = 0; leg < cfg::N_LEGS; ++leg)
                    if (!same(f.pos[leg], g.neutral(leg)) || f.pos[leg].z != 0.0f) at_rest = false;
                if (g.phase() != frozen || g.walking()) at_rest = false;
            }

            for (int i = 0; i < CYCLE_TICKS; ++i)
                if (!w.see(g.update(DT, v, 0.0f, 0.0f))) smooth_start = false;
        }
        char m[96];
        std::snprintf(m, sizeof m, "%s: no foot drops when stopping", NAMES[gi]);          CHECK(smooth_stop, m);
        std::snprintf(m, sizeof m, "%s: stop completes within one swing", NAMES[gi]);      CHECK(prompt, m);
        std::snprintf(m, sizeof m, "%s: rests frozen with every foot at neutral", NAMES[gi]); CHECK(at_rest, m);
        std::snprintf(m, sizeof m, "%s: no foot jumps up when restarting", NAMES[gi]);     CHECK(smooth_start, m);
    }
}

// A gait change mid-walk would re-map every leg's phase at once, so it is held
// until the walk is at rest; at rest it applies immediately.
static void gait_switch_waits_for_rest() {
    std::printf("set_gait mid-walk is held until the walk is at rest\n");
    const float v = 150.0f;
    Generator g; g.set_gait(gait::TRIPOD);
    HeightWatch w(std::fmax(steady_dz(gait::TRIPOD, v), steady_dz(gait::WAVE, v)) * 1.05f + 1e-3f);
    bool smooth = true;
    for (int i = 0; i < CYCLE_TICKS + 7; ++i) smooth &= w.see(g.update(DT, v, 0.0f, 0.0f));
    g.set_gait(gait::WAVE);
    CHECK(g.gait() == gait::TRIPOD, "still tripod right after the request");
    for (int i = 0; i < CYCLE_TICKS; ++i) smooth &= w.see(g.update(DT, v, 0.0f, 0.0f));
    CHECK(g.gait() == gait::TRIPOD, "still tripod while walking");
    for (int i = 0; i < CYCLE_TICKS && g.walking(); ++i) smooth &= w.see(g.update(DT, 0.0f, 0.0f, 0.0f));
    CHECK(!g.walking(), "came to rest");
    CHECK(g.gait() == gait::WAVE, "switched to wave once at rest");
    for (int i = 0; i < CYCLE_TICKS; ++i) smooth &= w.see(g.update(DT, v, 0.0f, 0.0f));
    CHECK(smooth, "no height jump across the whole switch");
    for (int i = 0; i < CYCLE_TICKS && g.walking(); ++i) g.update(DT, 0.0f, 0.0f, 0.0f);
    g.set_gait(gait::RIPPLE);
    CHECK(g.gait() == gait::RIPPLE, "at rest the switch is immediate");
}

int main() {
    neutral_geometry();
    still_from_rest();
    still_after_walk();
    stride_scales();
    clamp_preserves_radius();
    stop_and_restart_on_the_ground();
    gait_switch_waits_for_rest();
    return test_summary("gait");
}
