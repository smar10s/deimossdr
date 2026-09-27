// SPDX-License-Identifier: MIT
/*
 * test_waveform_parse.c — host tests for deimos_tool_load_waveform()
 *
 * Covers the JSON waveform parser in tools/common/deimos_tool.c. The bug
 * this guards: a well-formed-length but malformed-content array (an
 * unparseable token in both "real" and "imag" at the same position) was
 * silently accepted and returned a short waveform with no error.
 */

#include "test_util.h"
#include "deimos_tool.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* deimos_tool.c references lib80211_append_fcs from the frame builders,
 * which this test never calls. Provide a local definition so the test
 * links without pulling in the full lib80211 archive. */
void lib80211_append_fcs(uint8_t *buf, size_t len)
{
    (void)buf;
    (void)len;
}

static const char *write_tmp(const char *name, const char *content)
{
    static char path[256];
    snprintf(path, sizeof(path), "%s", name);
    FILE *f = fopen(path, "w");
    if (!f) return NULL;
    fputs(content, f);
    fclose(f);
    return path;
}

static void test_valid_waveform(void)
{
    TEST_BEGIN("load_waveform accepts a valid waveform");

    const char *path = write_tmp("wf_valid.json",
        "{\"real\":[1.0,2.0,3.0],\"imag\":[4.0,5.0,6.0],\"stf_offset\":42}");
    if (!path) { TEST_FAIL("could not write temp file"); return; }

    float *re = NULL, *im = NULL;
    int n = -1, stf = -1;
    int rc = deimos_tool_load_waveform(path, &re, &im, &n, &stf, NULL);
    remove(path);

    bool ok = assert_i32("rc", 0, rc);
    ok &= assert_i32("n_samples", 3, n);
    ok &= assert_i32("stf_offset", 42, stf);
    if (ok) {
        ok &= assert_float_close("re[0]", 1.0f, re[0], 1e-6f);
        ok &= assert_float_close("re[2]", 3.0f, re[2], 1e-6f);
        ok &= assert_float_close("im[0]", 4.0f, im[0], 1e-6f);
        ok &= assert_float_close("im[2]", 6.0f, im[2], 1e-6f);
    }
    free(re);
    free(im);
    if (ok) TEST_PASS();
}

/* The regression: a malformed token in both arrays yields equal (short)
 * lengths, so the length-mismatch check does not catch it. Must error. */
static void test_malformed_token(void)
{
    TEST_BEGIN("load_waveform rejects an unparseable token");

    const char *path = write_tmp("wf_bad_token.json",
        "{\"real\":[1.0,2.0,nope,4.0],\"imag\":[5.0,6.0,nope,8.0]}");
    if (!path) { TEST_FAIL("could not write temp file"); return; }

    float *re = NULL, *im = NULL;
    int n = -1;
    int rc = deimos_tool_load_waveform(path, &re, &im, &n, NULL, NULL);
    remove(path);

    bool ok = assert_i32("rc", -1, rc);
    if (ok) TEST_PASS();
}

static void test_truncated_array(void)
{
    TEST_BEGIN("load_waveform rejects a truncated (unclosed) array");

    const char *path = write_tmp("wf_truncated.json",
        "{\"real\":[1.0,2.0],\"imag\":[1.0,2.0");
    if (!path) { TEST_FAIL("could not write temp file"); return; }

    float *re = NULL, *im = NULL;
    int n = -1;
    int rc = deimos_tool_load_waveform(path, &re, &im, &n, NULL, NULL);
    remove(path);

    bool ok = assert_i32("rc", -1, rc);
    if (ok) TEST_PASS();
}

int main(void)
{
    test_valid_waveform();
    test_malformed_token();
    test_truncated_array();

    TEST_SUMMARY();
    return TEST_EXIT();
}
