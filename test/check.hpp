#pragma once
//
// Tiny host-test helper: a CHECK macro, a float comparison, and a summary.
// Each test_*.cpp is its own executable, so g_failures is per-binary.

#include <cstdio>
#include <cmath>

inline int g_failures = 0;

#define CHECK(cond, msg) do {                                          \
    if (cond) std::printf("  ok    %s\n", (msg));                      \
    else { std::printf("  FAIL  %s   [%s:%d]\n", (msg), __FILE__, __LINE__); ++g_failures; } \
} while (0)

inline bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

inline int test_summary(const char* name) {
    std::printf("\n[%s] %s - %d failure(s)\n",
                name, g_failures ? "FAILED" : "PASSED", g_failures);
    return g_failures ? 1 : 0;
}
