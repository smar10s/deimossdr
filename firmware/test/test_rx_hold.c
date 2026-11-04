// SPDX-License-Identifier: MIT
/*
 * Host tests for the serialized radio transaction: hold/release, note_*,
 * and poll concurrency.
 */
#include "test_util.h"
#include "deimos_rx.h"
#include "hal.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stddef.h>
#include <unistd.h>

void stub_reset_writes(void);
size_t stub_total_writes(void);
size_t stub_writes_to(uint32_t addr, uint32_t *out, size_t max);
void stub_set_tag_status(uint32_t status);

static const deimos_radio_config_t k_radio = {
    .channel = 6,
    .rx_gain_db = 30.0,
    .agc_mode = DEIMOS_AGC_FAST_ATTACK,
    .configure_tx = true,
    .tx_atten_db = 3.0,
    .bandwidth_mhz = 28,
    .skip_calibration = true,
};

static bool check_stf_order(const char *label)
{
    uint32_t stf[8];
    size_t n = stub_writes_to(REG_DEIMOS_STF_ENABLE, stf, 8);
    if (!assert_u32("stf write count", 2, (uint32_t)n))
        return false;
    if (!assert_u32("stf first", 0, stf[0]))
        return false;
    if (!assert_u32("stf second", 1, stf[1]))
        return false;
    (void)label;
    return true;
}

static void test_before_init(void)
{
    TEST_BEGIN("hold/release before init");
    bool ok = assert_i32("hold", -1, deimos_rx_hold());
    ok = assert_i32("release", -1, deimos_rx_release()) && ok;
    if (ok)
        TEST_PASS();
}

static void test_freq_to_channel(void)
{
    TEST_BEGIN("freq to channel rounding");
    bool ok = true;
    ok = assert_i32("2412", 1,
                    deimos_rx_freq_to_channel(2412000000ULL)) && ok;
    ok = assert_i32("2437", 6,
                    deimos_rx_freq_to_channel(2437000000ULL)) && ok;
    ok = assert_i32("2436.999998", 6,
                    deimos_rx_freq_to_channel(2436999998ULL)) && ok;
    ok = assert_i32("2462", 11,
                    deimos_rx_freq_to_channel(2462000000ULL)) && ok;
    ok = assert_i32("2472", 13,
                    deimos_rx_freq_to_channel(2472000000ULL)) && ok;
    ok = assert_i32("2484", 14,
                    deimos_rx_freq_to_channel(2484000000ULL)) && ok;
    ok = assert_i32("5180", 36,
                    deimos_rx_freq_to_channel(5180000000ULL)) && ok;
    if (ok)
        TEST_PASS();
}

static void test_start(void)
{
    TEST_BEGIN("start for transaction tests");
    deimos_pipeline_config_t pipe = DEIMOS_PIPELINE_LIVE_DEFAULT;
    bool ok = assert_i32("start", 0, deimos_rx_start(&k_radio, &pipe));
    if (ok)
        TEST_PASS();
}

static void test_hold_release(void)
{
    TEST_BEGIN("hold/release toggles STF");
    stub_reset_writes();
    bool ok = assert_i32("hold", 0, deimos_rx_hold());
    ok = assert_i32("second hold", -1, deimos_rx_hold()) && ok;
    ok = assert_i32("release", 0, deimos_rx_release()) && ok;
    ok = assert_i32("second release", -1, deimos_rx_release()) && ok;
    ok = check_stf_order("hold/release") && ok;
    if (ok)
        TEST_PASS();
}

static void test_note_reentrant(void)
{
    TEST_BEGIN("note_* under hold does not relock or touch hardware");
    stub_reset_writes();
    bool ok = assert_i32("hold", 0, deimos_rx_hold());
    ok = assert_i32("note channel", 0, deimos_rx_note_channel(6)) && ok;
    ok = assert_i32("note gain", 0, deimos_rx_note_gain(30.0)) && ok;
    ok = assert_i32("note agc", 0,
                    deimos_rx_note_agc(DEIMOS_AGC_FAST_ATTACK)) && ok;
    ok = assert_i32("note bandwidth", 0,
                    deimos_rx_note_bandwidth(28)) && ok;
    ok = assert_i32("release", 0, deimos_rx_release()) && ok;
    ok = check_stf_order("note under hold") && ok;
    ok = assert_u32("only STF and FIFO writes", 6,
                    (uint32_t)stub_total_writes()) && ok;

    stub_reset_writes();
    ok = assert_i32("note channel (idle)", 0,
                    deimos_rx_note_channel(11)) && ok;
    ok = assert_i32("note gain (idle)", 0,
                    deimos_rx_note_gain(20.0)) && ok;
    ok = assert_i32("note agc (idle)", 0,
                    deimos_rx_note_agc(DEIMOS_AGC_MANUAL)) && ok;
    ok = assert_i32("note bandwidth (idle)", 0,
                    deimos_rx_note_bandwidth(20)) && ok;
    ok = assert_u32("notes write nothing", 0,
                    (uint32_t)stub_total_writes()) && ok;
    if (ok)
        TEST_PASS();
}

static void test_poll_after_release(void)
{
    TEST_BEGIN("poll returns empty after release");
    stub_set_tag_status(TAG_STATUS_EMPTY);
    deimos_rx_frame_t frame;
    bool ok = assert_i32("poll", 0, deimos_rx_poll(&frame));
    if (ok)
        TEST_PASS();
}

static void test_setters(void)
{
    TEST_BEGIN("hot setters use hold/release");

    stub_reset_writes();
    bool ok = assert_i32("set freq", 0, deimos_rx_set_freq(2437000000ULL));
    ok = check_stf_order("set freq") && ok;

    stub_reset_writes();
    ok = assert_i32("set channel", 0, deimos_rx_set_channel(11)) && ok;
    ok = check_stf_order("set channel") && ok;

    stub_reset_writes();
    ok = assert_i32("set bandwidth", 0, deimos_rx_set_bandwidth(20)) && ok;
    ok = check_stf_order("set bandwidth") && ok;

    stub_reset_writes();
    ok = assert_i32("set gain", 0, deimos_rx_set_gain(20.0)) && ok;
    ok = check_stf_order("set gain") && ok;

    stub_reset_writes();
    ok = assert_i32("set agc", 0,
                    deimos_rx_set_agc_mode(DEIMOS_AGC_MANUAL)) && ok;
    ok = check_stf_order("set agc") && ok;

    stub_reset_writes();
    ok = assert_i32("set threshold", 0, deimos_rx_set_threshold(2)) && ok;
    ok = assert_u32("threshold write", 1,
                    (uint32_t)stub_total_writes()) && ok;

    if (ok)
        TEST_PASS();
}

static atomic_bool g_poll_stop;
static atomic_int g_poll_errors;

static void *poll_thread(void *arg)
{
    (void)arg;
    deimos_rx_frame_t frame;
    while (!atomic_load(&g_poll_stop)) {
        int rc = deimos_rx_poll(&frame);
        if (rc < 0)
            atomic_fetch_add(&g_poll_errors, 1);
    }
    return NULL;
}

static void test_stress(void)
{
    TEST_BEGIN("poll thread serializes with hold/release");

    atomic_store(&g_poll_stop, false);
    atomic_store(&g_poll_errors, 0);

    pthread_t tid;
    bool ok = assert_i32("thread create", 0,
                         pthread_create(&tid, NULL, poll_thread, NULL));
    if (!ok) {
        atomic_store(&g_poll_stop, true);
        return;
    }

    for (int i = 0; i < 100; i++) {
        if (deimos_rx_hold() != 0) {
            TEST_FAIL("hold iteration %d failed", i);
            ok = false;
            break;
        }
        if (deimos_rx_release() != 0) {
            TEST_FAIL("release iteration %d failed", i);
            ok = false;
            break;
        }
    }

    atomic_store(&g_poll_stop, true);
    pthread_join(tid, NULL);
    ok = assert_i32("poll errors", 0, atomic_load(&g_poll_errors)) && ok;
    if (ok)
        TEST_PASS();
}

int main(void)
{
    test_before_init();
    test_freq_to_channel();
    test_start();
    test_hold_release();
    test_note_reentrant();
    test_poll_after_release();
    test_setters();
    test_stress();

    deimos_rx_cleanup();
    TEST_SUMMARY();
    return TEST_EXIT();
}
