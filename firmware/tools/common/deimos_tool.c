// SPDX-License-Identifier: MIT
/*
 * deimos_tool.c — Shared scaffolding implementation
 *
 * Extracted from deimos_hil_inject, deimos_fabric_loopback,
 * deimos_adc_capture, and deimos_burst_loopback.
 */

#include "deimos_tool.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include <lib80211/mac.h>

/* ============================================================================
 * Snap buffer tag search
 * ============================================================================ */

uint32_t deimos_tool_find_snap_tag(uint32_t trig_pos)
{
    for (int off = 0; off < 16; off++) {
        int addr = ((int)trig_pos - off) & 0x3FF;
        hal_reg_write(REG_SNAP_RD_ADDR, (uint32_t)addr);
        usleep(1);
        uint32_t word = hal_reg_read(REG_SNAP_RD_DATA);
        uint32_t state = SNAP_STATE(word);
        if (state == S_TAG_OUT || state == S_DONE)
            return word;
    }
    /* No tag state in the last 16 entries — use the trigger entry itself. */
    hal_reg_write(REG_SNAP_RD_ADDR, trig_pos);
    usleep(1);
    return hal_reg_read(REG_SNAP_RD_DATA);
}

/* ============================================================================
 * Rate code table
 * ============================================================================ */

const uint8_t DEIMOS_RATE_CODES[55] = {
    [6]  = 0x0B, [9]  = 0x0F, [12] = 0x0A, [18] = 0x0E,
    [24] = 0x09, [36] = 0x0D, [48] = 0x08, [54] = 0x0C,
};

/* ============================================================================
 * JSON waveform parsing
 * ============================================================================ */

static int deimos_tool_parse_float_array(const char *json, const char *key,
                                         float **out, int *out_len)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);

    const char *p = strstr(json, search);
    if (!p) return -1;

    p = strchr(p, '[');
    if (!p) return -1;
    p++;  /* skip '[' */

    /* Empty array is valid and yields a zero-length result. */
    while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
    if (*p == ']') {
        *out = NULL;
        *out_len = 0;
        return 0;
    }

    /* Count elements (commas + 1). Requires a closing ']' at depth 0. */
    int count = 0;
    const char *scan = p;
    int depth = 1;
    while (*scan && depth > 0) {
        if (*scan == '[') depth++;
        else if (*scan == ']') depth--;
        else if (*scan == ',' && depth == 1) count++;
        scan++;
    }
    if (depth != 0) return -1;
    count++;  /* one more element than commas */

    float *arr = malloc((size_t)count * sizeof(float));
    if (!arr) return -1;

    int idx = 0;
    const char *cur = p;
    while (idx < count) {
        while (*cur == ' ' || *cur == '\n' || *cur == '\r' || *cur == '\t')
            cur++;
        char *end;
        float v = strtof(cur, &end);
        if (end == cur) {          /* unparseable token */
            free(arr);
            return -1;
        }
        arr[idx++] = v;
        cur = end;
        while (*cur == ',' || *cur == ' ' || *cur == '\n' || *cur == '\r' || *cur == '\t')
            cur++;
    }

    /* The element count must match exactly and be terminated by ']'.
     * This is what rejected the old silent-truncation behavior. */
    while (*cur == ' ' || *cur == '\n' || *cur == '\r' || *cur == '\t')
        cur++;
    if (idx != count || *cur != ']') {
        free(arr);
        return -1;
    }

    *out = arr;
    *out_len = idx;
    return 0;
}

static int deimos_tool_parse_int_field(const char *json, const char *key)
{
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);

    const char *p = strstr(json, search);
    if (!p) return -1;

    /* Skip past key and colon */
    p += strlen(search);
    while (*p == ' ' || *p == ':' || *p == '\t') p++;

    char *end;
    long val = strtol(p, &end, 10);
    if (end == p) return -1;
    return (int)val;
}

int deimos_tool_load_waveform(const char *path,
                              float **re, float **im, int *n_samples,
                              int *stf_offset, char **json_out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "ERROR: Cannot open %s\n", path);
        return -1;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "ERROR: %s is not seekable\n", path);
        fclose(f);
        return -1;
    }
    long fsize = ftell(f);
    if (fsize < 0) {
        /* ftell fails on directories and non-seekable streams. Without this
         * check, fsize == -1 gives malloc(0) then fread(.., (size_t)-1, ..). */
        fprintf(stderr, "ERROR: Cannot determine size of %s\n", path);
        fclose(f);
        return -1;
    }
    rewind(f);

    char *json = malloc((size_t)fsize + 1);
    if (!json) { fclose(f); return -1; }

    size_t rd = fread(json, 1, (size_t)fsize, f);
    fclose(f);
    json[rd] = '\0';

    int re_len = 0, im_len = 0;
    if (deimos_tool_parse_float_array(json, "real", re, &re_len) != 0) {
        fprintf(stderr, "ERROR: Cannot parse 'real' array from %s\n", path);
        free(json);
        return -1;
    }
    if (deimos_tool_parse_float_array(json, "imag", im, &im_len) != 0) {
        fprintf(stderr, "ERROR: Cannot parse 'imag' array from %s\n", path);
        free(json); free(*re);
        return -1;
    }

    /* Parse optional stf_offset metadata */
    if (stf_offset)
        *stf_offset = deimos_tool_parse_int_field(json, "stf_offset");

    if (re_len != im_len) {
        fprintf(stderr, "ERROR: real/imag length mismatch (%d vs %d)\n", re_len, im_len);
        free(json); free(*re); free(*im);
        return -1;
    }

    *n_samples = re_len;

    if (json_out)
        *json_out = json;
    else
        free(json);

    return 0;
}

/* ============================================================================
 * 12-bit DDR quantization
 * ============================================================================ */

float deimos_tool_quantize_to_ddr(const float *re, const float *im,
                                  int n_samples, volatile uint32_t *ddr_buf)
{
    /* Find peak for scaling to 12-bit range [-2047, +2047] */
    float peak = 0.0f;
    for (int i = 0; i < n_samples; i++) {
        float ar = fabsf(re[i]);
        float ai = fabsf(im[i]);
        if (ar > peak) peak = ar;
        if (ai > peak) peak = ai;
    }

    float scale = (peak > 0.0f) ? 2047.0f / peak : 1.0f;

    for (int i = 0; i < n_samples; i++) {
        int16_t ri = (int16_t)roundf(re[i] * scale);
        int16_t qi = (int16_t)roundf(im[i] * scale);
        /* Clamp to 12-bit */
        if (ri > 2047) ri = 2047;
        if (ri < -2048) ri = -2048;
        if (qi > 2047) qi = 2047;
        if (qi < -2048) qi = -2048;
        ddr_buf[i] = IQ_PACK(ri, qi);
    }

    return scale;
}

/* ============================================================================
 * Frame builders
 * ============================================================================ */

size_t deimos_tool_build_test_psdu(uint8_t *buf, int payload_len)
{
    buf[0] = 0x08; buf[1] = 0x00;  /* Data frame */
    buf[2] = 0x00; buf[3] = 0x00;  /* Duration */
    memset(&buf[4], 0xFF, 6);       /* DA = broadcast */
    buf[10] = 0x02; buf[11] = 0x00; buf[12] = 0x00;
    buf[13] = 0xDE; buf[14] = 0xAD; buf[15] = 0x01;  /* SA */
    buf[16] = 0x02; buf[17] = 0x00; buf[18] = 0x00;
    buf[19] = 0xDE; buf[20] = 0xAD; buf[21] = 0x01;  /* BSSID */
    buf[22] = 0x00; buf[23] = 0x00;  /* Seq ctrl */
    for (int i = 0; i < payload_len; i++)
        buf[24 + i] = (uint8_t)(i & 0xFF);
    lib80211_append_fcs(buf, 24 + payload_len);
    return 24 + payload_len + 4;
}

#define EAPOL_ETHERTYPE_HI  0x88
#define EAPOL_ETHERTYPE_LO  0x8E

size_t deimos_tool_build_frame(uint8_t *buf, int payload_bytes,
                               uint8_t seq_num, bool eapol)
{
    int offset = 0;

    /* MAC header (24 bytes) */
    buf[offset++] = 0x08;  /* FC: Data */
    buf[offset++] = 0x02;  /* FC: From DS */
    buf[offset++] = 0x00;  /* Duration */
    buf[offset++] = 0x00;
    /* DA = broadcast */
    memset(&buf[offset], 0xFF, 6); offset += 6;
    /* BSSID */
    buf[offset++] = 0x02; buf[offset++] = 0x00; buf[offset++] = 0x00;
    buf[offset++] = 0xDE; buf[offset++] = 0xAD; buf[offset++] = 0x01;
    /* SA */
    buf[offset++] = 0x02; buf[offset++] = 0x00; buf[offset++] = 0x00;
    buf[offset++] = 0xDE; buf[offset++] = 0xAD; buf[offset++] = 0x02;
    /* Seq ctrl */
    buf[offset++] = (uint8_t)(seq_num & 0xFF);
    buf[offset++] = 0x00;

    /* LLC/SNAP header (8 bytes) */
    buf[offset++] = 0xAA;  /* DSAP */
    buf[offset++] = 0xAA;  /* SSAP */
    buf[offset++] = 0x03;  /* Control */
    buf[offset++] = 0x00;  /* OUI */
    buf[offset++] = 0x00;
    buf[offset++] = 0x00;
    if (eapol) {
        buf[offset++] = EAPOL_ETHERTYPE_HI;
        buf[offset++] = EAPOL_ETHERTYPE_LO;
    } else {
        buf[offset++] = 0x08;  /* IP: 0x0800 */
        buf[offset++] = 0x00;
    }

    /* Payload: magic + seq_num + incrementing pattern */
    buf[offset++] = 0xDE;  /* magic[0] */
    buf[offset++] = 0x10;  /* magic[1] */
    buf[offset++] = 0x05;  /* magic[2] */
    buf[offset++] = seq_num;
    for (int i = 4; i < payload_bytes; i++)
        buf[offset++] = (uint8_t)((seq_num + i) & 0xFF);

    /* FCS */
    lib80211_append_fcs(buf, offset);
    return offset + 4;
}
