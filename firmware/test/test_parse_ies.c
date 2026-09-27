// SPDX-License-Identifier: MIT
#include "test_util.h"
#include "deimos_rx.h"
#include <string.h>

/* --------------------------------------------------------------------------
 * Helper: build a minimal beacon frame with specific IEs
 * -------------------------------------------------------------------------- */

static size_t build_beacon(uint8_t *buf, size_t buf_size,
                           const uint8_t *ies, size_t ies_len)
{
    if (36 + ies_len > buf_size) return 0;
    memset(buf, 0, 36);
    buf[0] = 0x80;  /* FC: type=0 (mgmt), subtype=8 (beacon) */
    /* DA = broadcast */
    memset(buf + 4, 0xFF, 6);
    /* Append IEs */
    memcpy(buf + 36, ies, ies_len);
    return 36 + ies_len;
}

/* -------------------------------------------------------------------------- */

static void test_ds_parameter(void)
{
    TEST_BEGIN("IE: DS Parameter (channel)");
    uint8_t ies[] = { 3, 1, 36 };
    uint8_t psdu[128];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, sizeof(ies));

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_true("has_ie_summary", ie.has_ie_summary);
    ok &= assert_u32("channel", 36, ie.channel);
    if (ok) TEST_PASS();
}

static void test_rsn_wpa2_psk_ccmp(void)
{
    TEST_BEGIN("IE: RSN WPA2-PSK/CCMP");
    uint8_t rsn_body[] = {
        0x01, 0x00,                         /* version 1 */
        0x00, 0x0F, 0xAC, 0x04,           /* group: CCMP */
        0x01, 0x00,                         /* pairwise count: 1 */
        0x00, 0x0F, 0xAC, 0x04,           /* pairwise: CCMP */
        0x01, 0x00,                         /* AKM count: 1 */
        0x00, 0x0F, 0xAC, 0x02,           /* AKM: PSK */
        0x00, 0x00,                         /* RSN capabilities */
    };
    uint8_t ies[64];
    ies[0] = 48;
    ies[1] = sizeof(rsn_body);
    memcpy(ies + 2, rsn_body, sizeof(rsn_body));

    uint8_t psdu[256];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, 2 + sizeof(rsn_body));

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_true("has_ie_summary", ie.has_ie_summary);
    ok &= assert_u32("security", DEIMOS_SEC_WPA2_PSK, ie.security);
    ok &= assert_u32("cipher", DEIMOS_CIPHER_CCMP, ie.cipher);
    if (ok) TEST_PASS();
}

static void test_rsn_wpa3_sae(void)
{
    TEST_BEGIN("IE: RSN WPA3-SAE/CCMP");
    uint8_t rsn_body[] = {
        0x01, 0x00,
        0x00, 0x0F, 0xAC, 0x04,           /* group: CCMP */
        0x01, 0x00,
        0x00, 0x0F, 0xAC, 0x04,           /* pairwise: CCMP */
        0x01, 0x00,
        0x00, 0x0F, 0xAC, 0x08,           /* AKM: SAE */
        0xC0, 0x00,                         /* MFP capable+required */
    };
    uint8_t ies[64];
    ies[0] = 48;
    ies[1] = sizeof(rsn_body);
    memcpy(ies + 2, rsn_body, sizeof(rsn_body));

    uint8_t psdu[256];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, 2 + sizeof(rsn_body));

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_true("has_ie_summary", ie.has_ie_summary);
    ok &= assert_u32("security", DEIMOS_SEC_WPA3_SAE, ie.security);
    ok &= assert_u32("cipher", DEIMOS_CIPHER_CCMP, ie.cipher);
    if (ok) TEST_PASS();
}

static void test_ht_vht_caps(void)
{
    TEST_BEGIN("IE: HT + VHT capabilities");
    uint8_t ies[64];
    int pos = 0;

    /* HT Capabilities: tag=45, len=26, cap_info=0x086F (40MHz set) */
    ies[pos++] = 45; ies[pos++] = 26;
    ies[pos++] = 0x6F; ies[pos++] = 0x08;  /* cap info LE: bit1 = 40MHz */
    memset(ies + pos, 0, 24); pos += 24;

    /* VHT Capabilities: tag=191, len=12, cap_info=0x3FC35832 */
    ies[pos++] = 191; ies[pos++] = 12;
    ies[pos++] = 0x32; ies[pos++] = 0x58; ies[pos++] = 0xC3; ies[pos++] = 0x3F;
    memset(ies + pos, 0, 8); pos += 8;

    uint8_t psdu[256];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, (size_t)pos);

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_true("has_ie_summary", ie.has_ie_summary);
    ok &= assert_true("HT present", (ie.phy_flags & DEIMOS_PHY_HT) != 0);
    ok &= assert_true("HT 40MHz", (ie.phy_flags & DEIMOS_PHY_HT_40MHZ) != 0);
    ok &= assert_true("VHT present", (ie.phy_flags & DEIMOS_PHY_VHT) != 0);
    ok &= assert_true("VHT 80MHz", (ie.phy_flags & DEIMOS_PHY_VHT_80MHZ) != 0);
    if (ok) TEST_PASS();
}

static void test_supported_rates(void)
{
    TEST_BEGIN("IE: Supported Rates (max rate)");
    uint8_t ies[] = {
        1, 8,  /* Supported Rates, 8 entries */
        0x8C, 0x12, 0x98, 0x24, 0x30, 0x48, 0x60, 0x6C,
    };
    uint8_t psdu[128];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, sizeof(ies));

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    /* 0x6C & 0x7F = 108 (54 Mbps in 0.5 Mbps units) */
    bool ok = assert_u32("max_rate", 108, ie.max_rate);
    if (ok) TEST_PASS();
}

static void test_country_ie(void)
{
    TEST_BEGIN("IE: Country (CA)");
    uint8_t ies[] = {
        7, 6,
        'C', 'A', ' ', 0x24, 0x01, 0x1E,
    };
    uint8_t psdu[128];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, sizeof(ies));

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_true("has_ie_summary", ie.has_ie_summary);
    ok &= assert_u32("country[0]", 'C', ie.country[0]);
    ok &= assert_u32("country[1]", 'A', ie.country[1]);
    if (ok) TEST_PASS();
}

static void test_vendor_oui(void)
{
    TEST_BEGIN("IE: Vendor OUI (Microsoft 0050F2)");
    uint8_t ies[] = {
        221, 6,
        0x00, 0x50, 0xF2, 0x01, 0x00, 0x00,
    };
    uint8_t psdu[128];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, sizeof(ies));

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_true("has_ie_summary", ie.has_ie_summary);
    ok &= assert_u32("vendor[0]", 0x00, ie.vendor_oui[0]);
    ok &= assert_u32("vendor[1]", 0x50, ie.vendor_oui[1]);
    ok &= assert_u32("vendor[2]", 0xF2, ie.vendor_oui[2]);
    if (ok) TEST_PASS();
}

static void test_open_network(void)
{
    TEST_BEGIN("IE: Open network (no RSN, no Privacy)");
    uint8_t ies[] = { 3, 1, 6 };
    uint8_t psdu[128];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, sizeof(ies));

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_u32("security", DEIMOS_SEC_OPEN, ie.security);
    ok &= assert_u32("cipher", DEIMOS_CIPHER_NONE, ie.cipher);
    if (ok) TEST_PASS();
}

static void test_wep_network(void)
{
    TEST_BEGIN("IE: WEP network (no RSN, Privacy bit set)");
    uint8_t ies[] = { 3, 1, 11 };
    uint8_t psdu[128];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, sizeof(ies));
    /* Set Privacy bit in Capability Info (offset 34, bit 4) */
    psdu[34] = 0x10;

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_u32("security", DEIMOS_SEC_WEP, ie.security);
    ok &= assert_u32("cipher", DEIMOS_CIPHER_WEP, ie.cipher);
    if (ok) TEST_PASS();
}

static void test_non_beacon_ignored(void)
{
    TEST_BEGIN("IE: non-beacon frame returns empty");
    uint8_t psdu[64];
    memset(psdu, 0, sizeof(psdu));
    psdu[0] = 0x00;  /* assoc request (subtype=0) */

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, 64, &ie);

    bool ok = assert_false("has_ie_summary", ie.has_ie_summary);
    if (ok) TEST_PASS();
}

static void test_combined_real_beacon(void)
{
    TEST_BEGIN("IE: combined real beacon");
    uint8_t ies[256];
    int pos = 0;

    /* SSID */
    ies[pos++] = 0; ies[pos++] = 9;
    memcpy(ies + pos, "TestSSID9", 9); pos += 9;
    /* Supported Rates */
    ies[pos++] = 1; ies[pos++] = 8;
    uint8_t rates[] = {0x8C, 0x12, 0x98, 0x24, 0x30, 0x48, 0x60, 0x6C};
    memcpy(ies + pos, rates, 8); pos += 8;
    /* DS Parameter */
    ies[pos++] = 3; ies[pos++] = 1; ies[pos++] = 36;
    /* Country */
    ies[pos++] = 7; ies[pos++] = 6;
    memcpy(ies + pos, "CA \x24\x01\x1E", 6); pos += 6;
    /* HT Capabilities */
    ies[pos++] = 45; ies[pos++] = 26;
    ies[pos++] = 0x6F; ies[pos++] = 0x08;
    memset(ies + pos, 0, 24); pos += 24;
    /* RSN */
    uint8_t rsn[] = {0x01,0x00, 0x00,0x0F,0xAC,0x04, 0x01,0x00,
                     0x00,0x0F,0xAC,0x04, 0x01,0x00, 0x00,0x0F,0xAC,0x02,
                     0x00,0x00};
    ies[pos++] = 48; ies[pos++] = sizeof(rsn);
    memcpy(ies + pos, rsn, sizeof(rsn)); pos += (int)sizeof(rsn);
    /* VHT Capabilities */
    ies[pos++] = 191; ies[pos++] = 12;
    uint8_t vht[] = {0x32,0x58,0xC3,0x3F, 0xAA,0xFF,0x00,0x00, 0xAA,0xFF,0x00,0x00};
    memcpy(ies + pos, vht, 12); pos += 12;
    /* Vendor */
    ies[pos++] = 221; ies[pos++] = 4;
    ies[pos++] = 0x00; ies[pos++] = 0x50; ies[pos++] = 0xF2; ies[pos++] = 0x02;

    uint8_t psdu[512];
    size_t len = build_beacon(psdu, sizeof(psdu), ies, (size_t)pos);

    deimos_ie_summary_t ie;
    deimos_rx_parse_ies(psdu, len, &ie);

    bool ok = assert_true("has_ie_summary", ie.has_ie_summary);
    ok &= assert_u32("channel", 36, ie.channel);
    ok &= assert_u32("security", DEIMOS_SEC_WPA2_PSK, ie.security);
    ok &= assert_u32("cipher", DEIMOS_CIPHER_CCMP, ie.cipher);
    ok &= assert_true("HT", (ie.phy_flags & DEIMOS_PHY_HT) != 0);
    ok &= assert_true("HT_40", (ie.phy_flags & DEIMOS_PHY_HT_40MHZ) != 0);
    ok &= assert_true("VHT", (ie.phy_flags & DEIMOS_PHY_VHT) != 0);
    ok &= assert_true("VHT_80", (ie.phy_flags & DEIMOS_PHY_VHT_80MHZ) != 0);
    ok &= assert_u32("max_rate", 108, ie.max_rate);
    ok &= assert_u32("country[0]", 'C', ie.country[0]);
    ok &= assert_u32("country[1]", 'A', ie.country[1]);
    ok &= assert_u32("vendor[0]", 0x00, ie.vendor_oui[0]);
    ok &= assert_u32("vendor[1]", 0x50, ie.vendor_oui[1]);
    ok &= assert_u32("vendor[2]", 0xF2, ie.vendor_oui[2]);
    if (ok) TEST_PASS();
}

/* -------------------------------------------------------------------------- */

int main(void)
{
    test_ds_parameter();
    test_rsn_wpa2_psk_ccmp();
    test_rsn_wpa3_sae();
    test_ht_vht_caps();
    test_supported_rates();
    test_country_ie();
    test_vendor_oui();
    test_open_network();
    test_wep_network();
    test_non_beacon_ignored();
    test_combined_real_beacon();

    TEST_SUMMARY();
    return TEST_EXIT();
}
