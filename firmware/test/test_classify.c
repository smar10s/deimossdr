// SPDX-License-Identifier: MIT
#include "test_util.h"
#include "deimos_rx.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * Helper: classify a single input and check result
 * -------------------------------------------------------------------------- */

typedef struct {
    int rate_mbps;
    uint32_t length;
    bool fcs_ok;
    deimos_frame_class_t expected;
    const char *label;
} classify_case_t;

static bool run_cases(const classify_case_t *cases, size_t n_cases)
{
    bool all_ok = true;
    for (size_t i = 0; i < n_cases; i++) {
        const classify_case_t *c = &cases[i];
        deimos_frame_class_t got =
            deimos_rx_classify_frame(c->rate_mbps, c->length, c->fcs_ok);
        if (got != c->expected) {
            TEST_FAIL("%s: rate=%d len=%u fcs=%d -> class %d, expected %d",
                      c->label, c->rate_mbps, c->length, (int)c->fcs_ok,
                      (int)got, (int)c->expected);
            all_ok = false;
        }
    }
    return all_ok;
}

/* --------------------------------------------------------------------------
 * ACK frames (Priority 1): len<=14 OR (len==20 AND rate in {6,12,24})
 * -------------------------------------------------------------------------- */

static void test_ack_frames(void)
{
    TEST_BEGIN("classify ack frames");

    classify_case_t cases[] = {
        /* Length <= 14: always ACK, any rate, any fcs */
        { 0,   0, true,  DEIMOS_FCLASS_ACK, "len=0" },
        { 54,  0, false, DEIMOS_FCLASS_ACK, "len=0 rate=54 fcs_fail" },
        { 6,   1, true,  DEIMOS_FCLASS_ACK, "len=1" },
        { 6,  13, false, DEIMOS_FCLASS_ACK, "len=13 fcs_fail" },
        { 54, 14, true,  DEIMOS_FCLASS_ACK, "len=14 boundary" },
        { 9,  14, false, DEIMOS_FCLASS_ACK, "len=14 rate=9 fcs_fail" },
        { 12, 14, true,  DEIMOS_FCLASS_ACK, "len=14 rate=12" },
        { 24, 14, true,  DEIMOS_FCLASS_ACK, "len=14 rate=24" },

        /* Len=15: NOT ACK (boundary +1 from <=14) */
        { 6,  15, true,  DEIMOS_FCLASS_DATA, "len=15 too long for ack" },
        { 54, 15, false, DEIMOS_FCLASS_HT_VHT, "len=15 rate=54 fcs_fail" },

        /* Len=20 AND rate in {6,12,24}: ACK */
        { 6,  20, true,  DEIMOS_FCLASS_ACK, "len=20 rate=6" },
        { 6,  20, false, DEIMOS_FCLASS_ACK, "len=20 rate=6 fcs_fail" },
        { 12, 20, true,  DEIMOS_FCLASS_ACK, "len=20 rate=12" },
        { 12, 20, false, DEIMOS_FCLASS_ACK, "len=20 rate=12 fcs_fail" },
        { 24, 20, true,  DEIMOS_FCLASS_ACK, "len=20 rate=24" },
        { 24, 20, false, DEIMOS_FCLASS_ACK, "len=20 rate=24 fcs_fail" },

        /* Len=20 with rates NOT in {6,12,24}: NOT ACK */
        { 9,  20, false, DEIMOS_FCLASS_HT_VHT, "len=20 rate=9 fcs_fail" },
        { 9,  20, true,  DEIMOS_FCLASS_DATA,   "len=20 rate=9 fcs_ok" },
        { 18, 20, false, DEIMOS_FCLASS_HT_VHT, "len=20 rate=18 fcs_fail" },
        { 18, 20, true,  DEIMOS_FCLASS_DATA,   "len=20 rate=18 fcs_ok" },
        { 36, 20, false, DEIMOS_FCLASS_HT_VHT, "len=20 rate=36 fcs_fail" },
        { 36, 20, true,  DEIMOS_FCLASS_DATA,   "len=20 rate=36 fcs_ok" },
        { 48, 20, false, DEIMOS_FCLASS_HT_VHT, "len=20 rate=48 fcs_fail" },
        { 48, 20, true,  DEIMOS_FCLASS_DATA,   "len=20 rate=48 fcs_ok" },
        { 54, 20, false, DEIMOS_FCLASS_HT_VHT, "len=20 rate=54 fcs_fail" },
        { 54, 20, true,  DEIMOS_FCLASS_DATA,   "len=20 rate=54 fcs_ok" },
    };

    if (run_cases(cases, sizeof(cases) / sizeof(cases[0])))
        TEST_PASS();
}

/* --------------------------------------------------------------------------
 * BLOCK_ACK (Priority 2): rate in {12,24} AND 28<=len<=32
 * -------------------------------------------------------------------------- */

static void test_block_ack_frames(void)
{
    TEST_BEGIN("classify block-ack frames");

    classify_case_t cases[] = {
        /* Core range */
        { 12, 28, true,  DEIMOS_FCLASS_BLOCK_ACK, "rate=12 len=28" },
        { 12, 28, false, DEIMOS_FCLASS_BLOCK_ACK, "rate=12 len=28 fcs_fail" },
        { 12, 30, true,  DEIMOS_FCLASS_BLOCK_ACK, "rate=12 len=30" },
        { 12, 32, true,  DEIMOS_FCLASS_BLOCK_ACK, "rate=12 len=32" },
        { 24, 28, false, DEIMOS_FCLASS_BLOCK_ACK, "rate=24 len=28 fcs_fail" },
        { 24, 30, true,  DEIMOS_FCLASS_BLOCK_ACK, "rate=24 len=30" },
        { 24, 32, true,  DEIMOS_FCLASS_BLOCK_ACK, "rate=24 len=32" },

        /* Boundaries: just outside range */
        { 12, 27, true,  DEIMOS_FCLASS_DATA, "rate=12 len=27 below blk_ack" },
        { 12, 33, true,  DEIMOS_FCLASS_DATA, "rate=12 len=33 above blk_ack" },
        { 24, 27, true,  DEIMOS_FCLASS_DATA, "rate=24 len=27 below blk_ack" },
        { 24, 33, true,  DEIMOS_FCLASS_DATA, "rate=24 len=33 above blk_ack" },

        /* Wrong rate, same length range */
        { 6,  30, true,  DEIMOS_FCLASS_DATA,   "rate=6 len=30 not_blk_ack" },
        { 9,  30, false, DEIMOS_FCLASS_HT_VHT, "rate=9 len=30 fcs_fail" },
        { 9,  30, true,  DEIMOS_FCLASS_DATA,   "rate=9 len=30 fcs_ok" },
        { 18, 30, false, DEIMOS_FCLASS_HT_VHT, "rate=18 len=30 fcs_fail" },
        { 36, 30, false, DEIMOS_FCLASS_HT_VHT, "rate=36 len=30 fcs_fail" },
        { 54, 30, true,  DEIMOS_FCLASS_DATA,   "rate=54 len=30 fcs_ok" },

        /* ACK takes priority over BLOCK_ACK: len=20 rate=12 → ACK */
        { 12, 20, true,  DEIMOS_FCLASS_ACK, "len=20 rate=12 -> ack beats blk_ack" },
    };

    if (run_cases(cases, sizeof(cases) / sizeof(cases[0])))
        TEST_PASS();
}

/* --------------------------------------------------------------------------
 * HT_VHT when fcs_ok == false (Priority 3)
 * -------------------------------------------------------------------------- */

static void test_ht_vht_fcs_fail(void)
{
    TEST_BEGIN("classify ht/vht fcs-fail");

    classify_case_t cases[] = {
        /* --- rate=6, fcs_ok=false --- */
        /* length==23: explicit HT_VHT */
        { 6,  23, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=23" },

        /* length in [18,66] && multiple of 3: HT_VHT */
        { 6,  18, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=18" },
        { 6,  21, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=21" },
        { 6,  24, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=24" },
        { 6,  27, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=27" },
        { 6,  30, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=30" },
        { 6,  33, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=33" },
        { 6,  36, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=36" },
        { 6,  39, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=39" },
        { 6,  42, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=42" },
        { 6,  45, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=45" },
        { 6,  48, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=48" },
        { 6,  51, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=51" },
        { 6,  54, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=54" },
        { 6,  57, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=57" },
        { 6,  60, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=60" },
        { 6,  63, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=63" },
        { 6,  66, false, DEIMOS_FCLASS_HT_VHT, "rate=6 len=66" },

        /* Boundaries for [18,66]: just outside */
        { 6,  17, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=17 below 18" },
        { 6,  67, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=67 above 66" },

        /* Non-multiple-of-3, not 23: UNKNOWN */
        { 6,  19, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=19" },
        { 6,  22, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=22" },
        { 6,  25, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=25" },
        { 6,  26, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=26" },
        { 6,  28, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=28" },
        { 6,  29, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=29" },
        { 6,  31, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=31" },
        { 6,  32, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=32" },
        { 6,  34, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=34" },
        { 6,  35, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=35" },
        { 6,  37, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=37" },
        { 6,  38, false, DEIMOS_FCLASS_UNKNOWN, "rate=6 len=38" },

        /* --- rate=12, fcs_ok=false ---
         * len<=14 -> ACK, len=20 -> ACK, len[28,32] -> BLOCK_ACK.
         * All other -> UNKNOWN (HT_VHT at len=20 is unreachable due to ACK priority) */
        { 12, 15, false, DEIMOS_FCLASS_UNKNOWN, "rate=12 len=15 fcs_fail" },
        { 12, 21, false, DEIMOS_FCLASS_UNKNOWN, "rate=12 len=21 fcs_fail" },
        { 12, 25, false, DEIMOS_FCLASS_UNKNOWN, "rate=12 len=25 fcs_fail" },
        { 12, 35, false, DEIMOS_FCLASS_UNKNOWN, "rate=12 len=35 fcs_fail" },
        { 12, 100,false, DEIMOS_FCLASS_UNKNOWN, "rate=12 len=100 fcs_fail" },

        /* --- rate=24, fcs_ok=false ---
         * len<=14 -> ACK, len=20 -> ACK, len[28,32] -> BLOCK_ACK.
         * All other -> UNKNOWN (HT_VHT at len=28,32 unreachable due to BLOCK_ACK) */
        { 24, 15, false, DEIMOS_FCLASS_UNKNOWN, "rate=24 len=15 fcs_fail" },
        { 24, 25, false, DEIMOS_FCLASS_UNKNOWN, "rate=24 len=25 fcs_fail" },
        { 24, 35, false, DEIMOS_FCLASS_UNKNOWN, "rate=24 len=35 fcs_fail" },
        { 24, 200,false, DEIMOS_FCLASS_UNKNOWN, "rate=24 len=200 fcs_fail" },

        /* --- rates {9,18,36,48,54}, fcs_ok=false: ANY length -> HT_VHT ---
         * (except len<=14 which is ACK) */
        { 9,  15, false, DEIMOS_FCLASS_HT_VHT, "rate=9 len=15" },
        { 9,  50, false, DEIMOS_FCLASS_HT_VHT, "rate=9 len=50" },
        { 9,  500,false, DEIMOS_FCLASS_HT_VHT, "rate=9 len=500" },
        { 18, 15, false, DEIMOS_FCLASS_HT_VHT, "rate=18 len=15" },
        { 18, 200,false, DEIMOS_FCLASS_HT_VHT, "rate=18 len=200" },
        { 36, 15, false, DEIMOS_FCLASS_HT_VHT, "rate=36 len=15" },
        { 36, 300,false, DEIMOS_FCLASS_HT_VHT, "rate=36 len=300" },
        { 48, 15, false, DEIMOS_FCLASS_HT_VHT, "rate=48 len=15" },
        { 48, 400,false, DEIMOS_FCLASS_HT_VHT, "rate=48 len=400" },
        { 54, 15, false, DEIMOS_FCLASS_HT_VHT, "rate=54 len=15" },
        { 54, 1000,false,DEIMOS_FCLASS_HT_VHT, "rate=54 len=1000" },

        /* ACK priority check: len<=14 with these rates -> ACK, not HT_VHT */
        { 9,  14, false, DEIMOS_FCLASS_ACK, "rate=9 len=14 ack beats ht_vht" },
        { 18, 14, false, DEIMOS_FCLASS_ACK, "rate=18 len=14 ack" },
        { 36, 14, false, DEIMOS_FCLASS_ACK, "rate=36 len=14 ack" },
        { 48, 14, false, DEIMOS_FCLASS_ACK, "rate=48 len=14 ack" },
        { 54, 14, false, DEIMOS_FCLASS_ACK, "rate=54 len=14 ack" },

        /* --- rate=0 (invalid), fcs_ok=false: UNKNOWN (no category matches) --- */
        { 0,  15, false, DEIMOS_FCLASS_UNKNOWN, "rate=0 len=15 fcs_fail" },
        { 0,  200,false, DEIMOS_FCLASS_UNKNOWN, "rate=0 len=200 fcs_fail" },
    };

    if (run_cases(cases, sizeof(cases) / sizeof(cases[0])))
        TEST_PASS();
}

/* --------------------------------------------------------------------------
 * FCS-OK classification (Priority 4): rate=6 classified by length range
 * -------------------------------------------------------------------------- */

static void test_fcs_ok_classification(void)
{
    TEST_BEGIN("classify fcs-ok rate=6");

    classify_case_t cases[] = {
        /* rate=6, fcs_ok: length 23 → HT_VHT */
        { 6, 23, true, DEIMOS_FCLASS_HT_VHT, "rate=6 len=23 fcs_ok" },

        /* rate=6, fcs_ok: [50, 299] → MGMT */
        { 6, 49,  true, DEIMOS_FCLASS_DATA,  "rate=6 len=49 below mgmt" },
        { 6, 50,  true, DEIMOS_FCLASS_MGMT,  "rate=6 len=50 mgmt entry" },
        { 6, 150, true, DEIMOS_FCLASS_MGMT,  "rate=6 len=150 mgmt interior" },
        { 6, 299, true, DEIMOS_FCLASS_MGMT,  "rate=6 len=299 mgmt ceiling" },
        { 6, 300, true, DEIMOS_FCLASS_BEACON,"rate=6 len=300 mgmt exit" },

        /* rate=6, fcs_ok: [300, 500] → BEACON */
        { 6, 299, true, DEIMOS_FCLASS_MGMT,  "rate=6 len=299 below beacon" },
        { 6, 300, true, DEIMOS_FCLASS_BEACON,"rate=6 len=300 beacon entry" },
        { 6, 400, true, DEIMOS_FCLASS_BEACON,"rate=6 len=400 beacon interior" },
        { 6, 500, true, DEIMOS_FCLASS_BEACON,"rate=6 len=500 beacon ceiling" },
        { 6, 501, true, DEIMOS_FCLASS_PROBE, "rate=6 len=501 beacon exit" },

        /* rate=6, fcs_ok: > 500 → PROBE */
        { 6, 501, true, DEIMOS_FCLASS_PROBE, "rate=6 len=501 probe entry" },
        { 6, 800, true, DEIMOS_FCLASS_PROBE, "rate=6 len=800 probe interior" },
        { 6, 1500,true, DEIMOS_FCLASS_PROBE, "rate=6 len=1500 probe large" },

        /* rate=6, fcs_ok: [15,22] → DATA (gap between ACK and HT_VHT23) */
        { 6, 15,  true, DEIMOS_FCLASS_DATA, "rate=6 len=15 fcs_ok data" },
        { 6, 20,  true, DEIMOS_FCLASS_ACK,  "rate=6 len=20 still ack" },
        { 6, 22,  true, DEIMOS_FCLASS_DATA, "rate=6 len=22 fcs_ok data" },

        /* rate=6, fcs_ok: [24,49] → DATA (gap between HT_VHT23 and MGMT) */
        { 6, 24,  true, DEIMOS_FCLASS_DATA, "rate=6 len=24 fcs_ok data" },
        { 6, 30,  true, DEIMOS_FCLASS_DATA, "rate=6 len=30 fcs_ok data" },
        { 6, 40,  true, DEIMOS_FCLASS_DATA, "rate=6 len=40 fcs_ok data" },
        { 6, 49,  true, DEIMOS_FCLASS_DATA, "rate=6 len=49 fcs_ok data" },

        /* Other rates, fcs_ok: fall through to DATA
         * (except ACK and BLOCK_ACK which take priority) */
        { 9,  15, true,  DEIMOS_FCLASS_DATA, "rate=9 len=15 fcs_ok" },
        { 12, 15, true,  DEIMOS_FCLASS_DATA, "rate=12 len=15 fcs_ok" },
        { 12, 50, true,  DEIMOS_FCLASS_DATA, "rate=12 len=50 fcs_ok" },
        { 24, 50, true,  DEIMOS_FCLASS_DATA, "rate=24 len=50 fcs_ok" },
        { 36, 100,true,  DEIMOS_FCLASS_DATA, "rate=36 len=100 fcs_ok" },
        { 48, 200,true,  DEIMOS_FCLASS_DATA, "rate=48 len=200 fcs_ok" },
        { 54, 500,true,  DEIMOS_FCLASS_DATA, "rate=54 len=500 fcs_ok" },
        { 0,  50, true,  DEIMOS_FCLASS_DATA, "rate=0 len=50 fcs_ok" },
    };

    if (run_cases(cases, sizeof(cases) / sizeof(cases[0])))
        TEST_PASS();
}

/* --------------------------------------------------------------------------
 * rate_mbps lookup
 * -------------------------------------------------------------------------- */

static void test_rate_mbps(void)
{
    TEST_BEGIN("rate_mbps lookup");

    bool ok = true;
    ok &= assert_i32("0x0B->6",  6,  deimos_rx_rate_mbps(0x0B));
    ok &= assert_i32("0x0F->9",  9,  deimos_rx_rate_mbps(0x0F));
    ok &= assert_i32("0x0A->12", 12, deimos_rx_rate_mbps(0x0A));
    ok &= assert_i32("0x0E->18", 18, deimos_rx_rate_mbps(0x0E));
    ok &= assert_i32("0x09->24", 24, deimos_rx_rate_mbps(0x09));
    ok &= assert_i32("0x0D->36", 36, deimos_rx_rate_mbps(0x0D));
    ok &= assert_i32("0x08->48", 48, deimos_rx_rate_mbps(0x08));
    ok &= assert_i32("0x0C->54", 54, deimos_rx_rate_mbps(0x0C));

    ok &= assert_i32("unknown->0", 0, deimos_rx_rate_mbps(0x00));
    ok &= assert_i32("0xFF->0",    0, deimos_rx_rate_mbps(0xFF));
    ok &= assert_i32("0x07->0",    0, deimos_rx_rate_mbps(0x07));

    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * should_parse
 * -------------------------------------------------------------------------- */

static void test_should_parse(void)
{
    TEST_BEGIN("should_parse");

    bool ok = true;

    ok &= assert_true("all mode 6M/100",
        deimos_rx_should_parse(DEIMOS_DECODE_ALL, 0x0B, 100));
    ok &= assert_true("all mode 24M/300",
        deimos_rx_should_parse(DEIMOS_DECODE_ALL, 0x09, 300));
    ok &= assert_true("all mode 54M/50",
        deimos_rx_should_parse(DEIMOS_DECODE_ALL, 0x0C, 50));

    ok &= assert_false("none mode 6M/100",
        deimos_rx_should_parse(DEIMOS_DECODE_NONE, 0x0B, 100));
    ok &= assert_false("none mode any",
        deimos_rx_should_parse(DEIMOS_DECODE_NONE, 0x0C, 500));

    ok &= assert_true("eapol mode 6M/100",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x0B, 100));
    ok &= assert_true("eapol mode 24M/200",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x09, 200));

    ok &= assert_false("eapol mode 9M/100",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x0F, 100));
    ok &= assert_false("eapol mode 6M/49",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x0B, 49));
    ok &= assert_false("eapol mode 6M/501",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x0B, 501));

    ok &= assert_true("eapol mode 6M/50 boundary",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x0B, 50));
    ok &= assert_true("eapol mode 6M/500 boundary",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x0B, 500));
    ok &= assert_true("eapol mode 24M/50",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x09, 50));
    ok &= assert_true("eapol mode 24M/500",
        deimos_rx_should_parse(DEIMOS_DECODE_EAPOL, 0x09, 500));

    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * channel_to_freq
 * -------------------------------------------------------------------------- */

static void test_channel_to_freq(void)
{
    TEST_BEGIN("channel_to_freq");

    bool ok = true;

    ok &= assert_u64("ch1",  2412000000ULL,
                     deimos_rx_channel_to_freq(1));
    ok &= assert_u64("ch6",  2437000000ULL,
                     deimos_rx_channel_to_freq(6));
    ok &= assert_u64("ch11", 2462000000ULL,
                     deimos_rx_channel_to_freq(11));
    ok &= assert_u64("ch14", 2484000000ULL,
                     deimos_rx_channel_to_freq(14));
    ok &= assert_u64("ch36", 5180000000ULL,
                     deimos_rx_channel_to_freq(36));
    ok &= assert_u64("ch100", 5500000000ULL,
                     deimos_rx_channel_to_freq(100));
    ok &= assert_u64("ch165", 5825000000ULL,
                     deimos_rx_channel_to_freq(165));

    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------- */

int main(void)
{
    test_ack_frames();
    test_block_ack_frames();
    test_ht_vht_fcs_fail();
    test_fcs_ok_classification();
    test_rate_mbps();
    test_should_parse();
    test_channel_to_freq();

    TEST_SUMMARY();
    return TEST_EXIT();
}
