// SPDX-License-Identifier: MIT
#ifndef FIRMWARE_TEST_UTIL_H
#define FIRMWARE_TEST_UTIL_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int passed;
    int failed;
    const char *current_test;
} test_state_t;

static test_state_t g_test = {0, 0, NULL};

#define TEST_BEGIN(name) do { \
    g_test.current_test = (name); \
    printf("  [ RUN  ] %s\n", (name)); \
} while(0)

#define TEST_PASS() do { \
    g_test.passed++; \
    printf("  [ PASS ] %s\n", g_test.current_test); \
} while(0)

#define TEST_FAIL(fmt, ...) do { \
    g_test.failed++; \
    printf("  [ FAIL ] %s: " fmt "\n", g_test.current_test, ##__VA_ARGS__); \
} while(0)

#define TEST_SUMMARY() do { \
    printf("\n  %d passed, %d failed\n", g_test.passed, g_test.failed); \
} while(0)

#define TEST_EXIT() (g_test.failed > 0 ? 1 : 0)

/* -------------------------------------------------------------------------- */

static inline bool assert_u32(const char *label, uint32_t expected, uint32_t got) {
    if (expected != got) {
        TEST_FAIL("%s: expected %u, got %u", label, expected, got);
        return false;
    }
    return true;
}

static inline bool assert_u64(const char *label, uint64_t expected, uint64_t got) {
    if (expected != got) {
        TEST_FAIL("%s: expected %llu, got %llu", label,
                  (unsigned long long)expected, (unsigned long long)got);
        return false;
    }
    return true;
}

static inline bool assert_i32(const char *label, int expected, int got) {
    if (expected != got) {
        TEST_FAIL("%s: expected %d, got %d", label, expected, got);
        return false;
    }
    return true;
}

static inline bool assert_true(const char *label, bool val) {
    if (!val) {
        TEST_FAIL("%s: expected true, got false", label);
        return false;
    }
    return true;
}

static inline bool assert_false(const char *label, bool val) {
    if (val) {
        TEST_FAIL("%s: expected false, got true", label);
        return false;
    }
    return true;
}

static inline bool assert_ptr_nonnull(const char *label, const void *p) {
    if (p == NULL) {
        TEST_FAIL("%s: expected non-NULL pointer", label);
        return false;
    }
    return true;
}

static inline bool assert_mem_eq(const char *label,
                                  const uint8_t *expected, const uint8_t *got,
                                  size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (expected[i] != got[i]) {
            TEST_FAIL("%s: byte %zu: expected 0x%02X, got 0x%02X",
                      label, i, expected[i], got[i]);
            return false;
        }
    }
    return true;
}

static inline bool assert_float_close(const char *label,
                                       float expected, float got, float tol) {
    float diff = got - expected;
    if (diff < 0) diff = -diff;
    if (diff > tol) {
        TEST_FAIL("%s: expected %.6f, got %.6f (diff %.6f > tol %.6f)",
                  label, (double)expected, (double)got,
                  (double)diff, (double)tol);
        return false;
    }
    return true;
}

static inline bool assert_double_close(const char *label,
                                        double expected, double got, double tol) {
    double diff = got - expected;
    if (diff < 0) diff = -diff;
    if (diff > tol) {
        TEST_FAIL("%s: expected %.12f, got %.12f (diff %.12f > tol %.12f)",
                  label, expected, got, diff, tol);
        return false;
    }
    return true;
}

#endif /* FIRMWARE_TEST_UTIL_H */
