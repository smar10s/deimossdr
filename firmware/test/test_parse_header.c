// SPDX-License-Identifier: MIT
#include "test_util.h"
#include "deimos_rx.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

static bool check_meta(const char *label, const deimos_rx_meta_t *meta,
                       const uint8_t *exp_bssid, bool has_bssid,
                       const uint8_t *exp_sa, bool has_sa,
                       const uint8_t *exp_da, bool has_da,
                       const char *exp_ssid, bool has_ssid,
                       bool is_eapol)
{
    bool ok = true;

    if (meta->has_bssid != has_bssid) {
        TEST_FAIL("%s: has_bssid=%d expected %d", label, meta->has_bssid, has_bssid);
        ok = false;
    }
    if (has_bssid && memcmp(meta->bssid, exp_bssid, 6) != 0) {
        TEST_FAIL("%s: bssid %02x:%02x:%02x:%02x:%02x:%02x, expected %02x:%02x:%02x:%02x:%02x:%02x",
                  label,
                  meta->bssid[0], meta->bssid[1], meta->bssid[2],
                  meta->bssid[3], meta->bssid[4], meta->bssid[5],
                  exp_bssid[0], exp_bssid[1], exp_bssid[2],
                  exp_bssid[3], exp_bssid[4], exp_bssid[5]);
        ok = false;
    }

    if (meta->has_sa != has_sa) {
        TEST_FAIL("%s: has_sa=%d expected %d", label, meta->has_sa, has_sa);
        ok = false;
    }
    if (has_sa && memcmp(meta->sa, exp_sa, 6) != 0) {
        TEST_FAIL("%s: sa %02x:%02x:%02x:%02x:%02x:%02x, expected %02x:%02x:%02x:%02x:%02x:%02x",
                  label,
                  meta->sa[0], meta->sa[1], meta->sa[2],
                  meta->sa[3], meta->sa[4], meta->sa[5],
                  exp_sa[0], exp_sa[1], exp_sa[2],
                  exp_sa[3], exp_sa[4], exp_sa[5]);
        ok = false;
    }

    if (meta->has_da != has_da) {
        TEST_FAIL("%s: has_da=%d expected %d", label, meta->has_da, has_da);
        ok = false;
    }
    if (has_da && memcmp(meta->da, exp_da, 6) != 0) {
        TEST_FAIL("%s: da %02x:%02x:%02x:%02x:%02x:%02x, expected %02x:%02x:%02x:%02x:%02x:%02x",
                  label,
                  meta->da[0], meta->da[1], meta->da[2],
                  meta->da[3], meta->da[4], meta->da[5],
                  exp_da[0], exp_da[1], exp_da[2],
                  exp_da[3], exp_da[4], exp_da[5]);
        ok = false;
    }

    if (meta->has_ssid != has_ssid) {
        TEST_FAIL("%s: has_ssid=%d expected %d", label, meta->has_ssid, has_ssid);
        ok = false;
    }
    if (has_ssid && exp_ssid && strcmp(meta->ssid, exp_ssid) != 0) {
        TEST_FAIL("%s: ssid='%s' expected '%s'", label, meta->ssid, exp_ssid);
        ok = false;
    }

    if (meta->is_eapol != is_eapol) {
        TEST_FAIL("%s: is_eapol=%d expected %d", label, meta->is_eapol, is_eapol);
        ok = false;
    }

    return ok;
}

/* --------------------------------------------------------------------------
 * Management: Beacon with SSID
 * -------------------------------------------------------------------------- */

static void test_beacon_with_ssid(void)
{
    TEST_BEGIN("parse beacon with SSID");

    uint8_t DA[6]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t SA[6]    = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    uint8_t BSSID[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

    uint8_t psdu[64];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x80;  /* FC: type=0 (mgmt), subtype=8 (beacon) */
    psdu[1] = 0x00;
    /* duration[2:3] = 0 */
    memcpy(psdu + 4,  DA, 6);
    memcpy(psdu + 10, SA, 6);
    memcpy(psdu + 16, BSSID, 6);
    /* sequence[22:23] = 0 */
    /* timestamp[24:31] = 0 */
    /* beacon interval[32:33] = 0 */
    /* capability[34:35] = 0 */

    /* SSID tag */
    psdu[36] = 0x00;  /* tag ID = SSID */
    psdu[37] = 0x07;  /* length = 7 */
    memcpy(psdu + 38, "testnet", 7);

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 45, &meta);

    bool ok = check_meta("beacon", &meta,
                         BSSID, true, SA, true, DA, true,
                         "testnet", true, false);
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Management: Beacon with SSID truncated to 32 chars
 * -------------------------------------------------------------------------- */

static void test_beacon_ssid_truncate(void)
{
    TEST_BEGIN("parse beacon SSID truncation");

    uint8_t DA[6]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t SA[6]    = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    uint8_t BSSID[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

    uint8_t psdu[100];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x80;
    psdu[1] = 0x00;
    memcpy(psdu + 4,  DA, 6);
    memcpy(psdu + 10, SA, 6);
    memcpy(psdu + 16, BSSID, 6);

    /* SSID tag with 40 chars (should be truncated to 32) */
    psdu[36] = 0x00;
    psdu[37] = 40;
    memset(psdu + 38, 'X', 40);

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 80, &meta);

    bool ok = true;
    if (!meta.has_ssid) {
        TEST_FAIL("trunc_ssid: has_ssid expected true");
        ok = false;
    } else {
        if (strlen(meta.ssid) > 32) {
            TEST_FAIL("trunc_ssid: ssid length %zu exceeds 32", strlen(meta.ssid));
            ok = false;
        }
    }
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Management: Probe response with SSID
 * -------------------------------------------------------------------------- */

static void test_probe_response(void)
{
    TEST_BEGIN("parse probe response");

    uint8_t DA[6]    = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    uint8_t SA[6]    = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
    uint8_t BSSID[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};

    uint8_t psdu[64];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x50;  /* FC: type=0, subtype=5 (probe resp) */
    psdu[1] = 0x00;
    memcpy(psdu + 4,  DA, 6);
    memcpy(psdu + 10, SA, 6);
    memcpy(psdu + 16, BSSID, 6);

    psdu[36] = 0x00;
    psdu[37] = 0x04;
    memcpy(psdu + 38, "WiFi", 4);

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 42, &meta);

    bool ok = check_meta("probe_resp", &meta,
                         BSSID, true, SA, true, DA, true,
                         "WiFi", true, false);
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Management: non-beacon/probe-resp (no ssid extraction)
 * -------------------------------------------------------------------------- */

static void test_assoc_req_no_ssid(void)
{
    TEST_BEGIN("parse assoc req (no SSID extraction)");

    uint8_t DA[6]    = {0x02, 0x00, 0x00, 0x00, 0x00, 0x10};
    uint8_t SA[6]    = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
    uint8_t BSSID[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x10};

    uint8_t psdu[64];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x00;  /* FC: type=0, subtype=0 (assoc req) */
    psdu[1] = 0x00;
    memcpy(psdu + 4,  DA, 6);
    memcpy(psdu + 10, SA, 6);
    memcpy(psdu + 16, BSSID, 6);

    /* SSID tag present but subtype != 8,5 so it's ignored */
    psdu[36] = 0x00;
    psdu[37] = 0x04;
    memcpy(psdu + 38, "IGNR", 4);

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 42, &meta);

    bool ok = check_meta("assoc_req", &meta,
                         BSSID, true, SA, true, DA, true,
                         NULL, false, false);
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Data: Ad-hoc (ToDS=0, FromDS=0)
 * -------------------------------------------------------------------------- */

static void test_data_adhoc(void)
{
    TEST_BEGIN("parse data adhoc (to_ds=0 from_ds=0)");

    uint8_t DA_exp[6]    = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    uint8_t SA_exp[6]    = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    uint8_t BSSID_exp[6] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22};

    uint8_t psdu[32];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x08;  /* FC: type=2 (data), subtype=0 */
    psdu[1] = 0x00;  /* to_ds=0, from_ds=0 */
    memcpy(psdu + 4,  DA_exp, 6);
    memcpy(psdu + 10, SA_exp, 6);
    memcpy(psdu + 16, BSSID_exp, 6);

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 24, &meta);

    bool ok = check_meta("adhoc", &meta,
                         BSSID_exp, true, SA_exp, true, DA_exp, true,
                         NULL, false, false);
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Data: ToDS (ToDS=1, FromDS=0) — addr1=BSSID, addr2=SA, addr3=DA
 * -------------------------------------------------------------------------- */

static void test_data_to_ds(void)
{
    TEST_BEGIN("parse data to_ds (to_ds=1 from_ds=0)");

    uint8_t BSSID_exp[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};
    uint8_t SA_exp[6]    = {0x11, 0x11, 0x11, 0x11, 0x11, 0x02};
    uint8_t DA_exp[6]    = {0x22, 0x22, 0x22, 0x22, 0x22, 0x03};

    uint8_t psdu[32];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x08;
    psdu[1] = 0x01;  /* to_ds=1, from_ds=0 */
    memcpy(psdu + 4,  BSSID_exp, 6);
    memcpy(psdu + 10, SA_exp, 6);
    memcpy(psdu + 16, DA_exp, 6);

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 24, &meta);

    bool ok = check_meta("to_ds", &meta,
                         BSSID_exp, true, SA_exp, true, DA_exp, true,
                         NULL, false, false);
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Data: FromDS (ToDS=0, FromDS=1) — addr1=DA, addr2=BSSID, addr3=SA
 * -------------------------------------------------------------------------- */

static void test_data_from_ds(void)
{
    TEST_BEGIN("parse data from_ds (to_ds=0 from_ds=1)");

    uint8_t DA_exp[6]    = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x04};
    uint8_t BSSID_exp[6] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x05};
    uint8_t SA_exp[6]    = {0x22, 0x22, 0x22, 0x22, 0x22, 0x06};

    uint8_t psdu[32];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x08;
    psdu[1] = 0x02;  /* to_ds=0, from_ds=1 */
    memcpy(psdu + 4,  DA_exp, 6);
    memcpy(psdu + 10, BSSID_exp, 6);
    memcpy(psdu + 16, SA_exp, 6);

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 24, &meta);

    bool ok = check_meta("from_ds", &meta,
                         BSSID_exp, true, SA_exp, true, DA_exp, true,
                         NULL, false, false);
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Data: WDS (ToDS=1, FromDS=1) — needs psdu_len >= 30 for SA
 * -------------------------------------------------------------------------- */

static void test_data_wds(void)
{
    TEST_BEGIN("parse data wds (to_ds=1 from_ds=1)");

    uint8_t DA_exp[6]    = {0x22, 0x22, 0x22, 0x22, 0x22, 0x11};
    uint8_t BSSID_exp[6] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x10};
    uint8_t SA_exp[6]    = {0x33, 0x33, 0x33, 0x33, 0x33, 0x12};

    uint8_t psdu[32];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x08;
    psdu[1] = 0x03;  /* to_ds=1, from_ds=1 */
    /* addr2 = BSSID (at bytes 10-15) */
    memcpy(psdu + 10, BSSID_exp, 6);
    /* addr3 = DA (at bytes 16-21) */
    memcpy(psdu + 16, DA_exp, 6);
    /* addr4 = SA (at bytes 22-27) */
    memcpy(psdu + 22, SA_exp, 6);

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 30, &meta);

    bool ok = check_meta("wds", &meta,
                         BSSID_exp, true, SA_exp, true, DA_exp, true,
                         NULL, false, false);
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Data: WDS with len < 30 — SA not set
 * -------------------------------------------------------------------------- */

static void test_data_wds_short(void)
{
    TEST_BEGIN("parse data wds short (len < 30, SA missing)");

    uint8_t DA_exp[6]    = {0x22, 0x22, 0x22, 0x22, 0x22, 0x11};
    uint8_t BSSID_exp[6] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x10};

    uint8_t psdu[28];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x08;
    psdu[1] = 0x03;
    memcpy(psdu + 10, BSSID_exp, 6);
    memcpy(psdu + 16, DA_exp, 6);
    /* no addr4 at [22,27] — len < 30 */

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 28, &meta);

    /* DA and BSSID should be set; SA should NOT be set because len < 30 */
    bool ok = true;
    ok &= check_meta("wds_short_da_bssid", &meta,
                     BSSID_exp, true, NULL, false, DA_exp, true,
                     NULL, false, false);
    if (meta.has_sa) {
        TEST_FAIL("wds_short: has_sa true when len < 30");
        ok = false;
    }
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Data: EAPOL (LLC/SNAP + ethertype 0x888E)
 * -------------------------------------------------------------------------- */

static void test_data_eapol(void)
{
    TEST_BEGIN("parse data eapol");

    uint8_t DA_exp[6]    = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    uint8_t SA_exp[6]    = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    uint8_t BSSID_exp[6] = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22};

    uint8_t psdu[64];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x08;
    psdu[1] = 0x00;  /* adhoc */
    memcpy(psdu + 4,  DA_exp, 6);
    memcpy(psdu + 10, SA_exp, 6);
    memcpy(psdu + 16, BSSID_exp, 6);

    /* LLC/SNAP header at byte 24 */
    psdu[24] = 0xAA;
    psdu[25] = 0xAA;
    psdu[26] = 0x03;
    psdu[27] = 0x00;
    psdu[28] = 0x00;
    psdu[29] = 0x00;
    psdu[30] = 0x88;  /* ethertype 0x888E (big-endian) */
    psdu[31] = 0x8E;

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 32, &meta);

    bool ok = check_meta("eapol", &meta,
                         BSSID_exp, true, SA_exp, true, DA_exp, true,
                         NULL, false, true);
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Data: Not EAPOL (LLC/SNAP with wrong ethertype)
 * -------------------------------------------------------------------------- */

static void test_data_not_eapol(void)
{
    TEST_BEGIN("parse data not eapol (ethertype 0x0800)");

    uint8_t psdu[64];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x08;
    psdu[1] = 0x00;

    psdu[24] = 0xAA;
    psdu[25] = 0xAA;
    psdu[26] = 0x03;
    psdu[27] = 0x00;
    psdu[28] = 0x00;
    psdu[29] = 0x00;
    psdu[30] = 0x08;  /* ethertype 0x0800 = IPv4 */
    psdu[31] = 0x00;

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 32, &meta);

    if (meta.is_eapol) {
        TEST_FAIL("not_eapol: is_eapol true for ethertype 0x0800");
        return;
    }
    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Data: QoS (subtype & 0x08) — LLC at offset header+2
 * -------------------------------------------------------------------------- */

static void test_data_qos(void)
{
    TEST_BEGIN("parse data QoS (hdr_len += 2)");

    uint8_t psdu[64];
    memset(psdu, 0, sizeof(psdu));

    psdu[0] = 0x88;  /* FC: type=2, subtype=8 (QoS data) */
    psdu[1] = 0x00;

    /* LLC/SNAP at byte 26 (24 header + 2 QoS control) */
    psdu[26] = 0xAA;
    psdu[27] = 0xAA;
    psdu[28] = 0x03;
    psdu[29] = 0x00;
    psdu[30] = 0x00;
    psdu[31] = 0x00;
    psdu[32] = 0x88;
    psdu[33] = 0x8E;

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 34, &meta);

    if (!meta.is_eapol) {
        TEST_FAIL("qos_eapol: is_eapol false, expected true (LLC at offset 26)");
        return;
    }
    TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Truncated: < 24 bytes
 * -------------------------------------------------------------------------- */

static void test_truncated(void)
{
    TEST_BEGIN("parse truncated frame (< 24 bytes)");

    uint8_t psdu[16];
    memset(psdu, 0xFF, sizeof(psdu));
    psdu[0] = 0x08;

    deimos_rx_meta_t meta;
    memset(&meta, 0xAA, sizeof(meta));  /* poison */
    deimos_rx_parse_header(psdu, 16, &meta);

    bool ok = true;
    if (meta.has_bssid) { TEST_FAIL("truncated: has_bssid true"); ok = false; }
    if (meta.has_sa)    { TEST_FAIL("truncated: has_sa true");    ok = false; }
    if (meta.has_da)    { TEST_FAIL("truncated: has_da true");    ok = false; }
    if (meta.has_ssid)  { TEST_FAIL("truncated: has_ssid true");  ok = false; }
    if (meta.is_eapol)  { TEST_FAIL("truncated: is_eapol true");  ok = false; }
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * Control frame (type=1) — no matching case, metadata all zero
 * -------------------------------------------------------------------------- */

static void test_control_frame(void)
{
    TEST_BEGIN("parse control frame (type=1)");

    uint8_t psdu[24];
    memset(psdu, 0, sizeof(psdu));
    psdu[0] = 0x04;  /* FC: type=1 (control), subtype=0 */

    deimos_rx_meta_t meta;
    deimos_rx_parse_header(psdu, 24, &meta);

    bool ok = true;
    if (meta.has_bssid) { TEST_FAIL("control: has_bssid true"); ok = false; }
    if (meta.has_sa)    { TEST_FAIL("control: has_sa true");    ok = false; }
    if (meta.has_da)    { TEST_FAIL("control: has_da true");    ok = false; }
    if (meta.has_ssid)  { TEST_FAIL("control: has_ssid true");  ok = false; }
    if (meta.is_eapol)  { TEST_FAIL("control: is_eapol true");  ok = false; }
    if (ok) TEST_PASS();
}

/* --------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------- */

int main(void)
{
    test_beacon_with_ssid();
    test_beacon_ssid_truncate();
    test_probe_response();
    test_assoc_req_no_ssid();
    test_data_adhoc();
    test_data_to_ds();
    test_data_from_ds();
    test_data_wds();
    test_data_wds_short();
    test_data_eapol();
    test_data_not_eapol();
    test_data_qos();
    test_truncated();
    test_control_frame();

    TEST_SUMMARY();
    return TEST_EXIT();
}
