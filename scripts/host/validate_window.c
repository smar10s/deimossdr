/**
 * validate_window — Independent (non-RTL) decode of a deimos_ota_capture window
 *
 * ONE-OFF diagnostic helper (M2-loss investigation). Delete with the capture tool.
 *
 * Reads the raw packed-DDR .bin from deimos_ota_capture and asks one question
 * with lib80211 (which does not share code with the fabric):
 *
 *     Is the STA response M2 (EAPOL, 159-byte PSDU) decodable in this window?
 *
 * This is the discriminator the sim replay alone cannot provide:
 *   sim decodes M2, lib80211 doesn't  -> signal/SNR problem, not a fabric bug
 *   sim doesn't decode M2, lib80211 does -> fabric defect (clean, decodable IQ)
 *   neither decodes M2                -> IQ/SNR problem
 *
 * Frames are enumerated with lib80211_sync_detect (frame_start) and decoded
 * with lib80211_rx_decode, advancing past each frame by its estimated duration.
 *
 * Build (from project root, macOS vDSP backend):
 *   cmake -S extern/lib80211 -B build/lib80211-native \
 *         -DLIB80211_BUILD_TESTS=OFF -DLIB80211_BUILD_OTA=OFF
 *   cmake --build build/lib80211-native --target lib80211
 *   cc -O2 -o build/validate_window scripts/host/validate_window.c \
 *      -Iextern/lib80211/include -Lbuild/lib80211-native/src -llib80211 \
 *      -framework Accelerate -lm
 *
 * Usage:
 *   ./build/validate_window /tmp/ddr_win.bin
 *
 * stdout: one JSON summary object. stderr: per-frame human lines.
 */

#include "lib80211/rx.h"
#include "lib80211/sync.h"
#include "lib80211/fft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define MIN_BUF_SAMPLES 400
#define MAX_FRAMES      4096

static const uint8_t EAPOL_SNAP[8] =
    { 0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E };

static inline int16_t iq_real(uint32_t w)
{
    int16_t v = (int16_t)(w & 0xFFF);
    if (v & 0x800) v |= (int16_t)0xF000;
    return v;
}

static inline int16_t iq_imag(uint32_t w)
{
    int16_t v = (int16_t)((w >> 12) & 0xFFF);
    if (v & 0x800) v |= (int16_t)0xF000;
    return v;
}

static int rate_to_ndbps(int mbps)
{
    switch (mbps) {
    case 6:  return 24;
    case 9:  return 36;
    case 12: return 48;
    case 18: return 72;
    case 24: return 96;
    case 36: return 144;
    case 48: return 192;
    case 54: return 216;
    default: return 24;
    }
}

static size_t est_frame_samples(int mbps, size_t length)
{
    int ndbps = rate_to_ndbps(mbps);
    int n_symbols = (int)((16 + 8 * length + 6 + (size_t)ndbps - 1) / (size_t)ndbps);
    return (size_t)(160 + 160 + 80 + n_symbols * 80);
}

static bool is_eapol(const uint8_t *psdu, size_t len)
{
    if (len < sizeof(EAPOL_SNAP)) return false;
    for (size_t i = 0; i + sizeof(EAPOL_SNAP) <= len; i++) {
        if (memcmp(psdu + i, EAPOL_SNAP, sizeof(EAPOL_SNAP)) == 0)
            return true;
    }
    return false;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <ddr_capture.bin>\n", argv[0]);
        return 1;
    }
    const char *path = argv[1];

    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    long bytes = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (bytes <= 0 || bytes % 4 != 0) {
        fprintf(stderr, "ERROR: %s size %ld is not a multiple of 4\n", path, bytes);
        fclose(f);
        return 1;
    }
    size_t n = (size_t)bytes / 4;
    uint32_t *words = malloc(n * sizeof(uint32_t));
    float *re = malloc(n * sizeof(float));
    float *im = malloc(n * sizeof(float));
    if (!words || !re || !im) {
        fprintf(stderr, "ERROR: alloc failed\n");
        free(words); free(re); free(im); fclose(f);
        return 1;
    }
    if (fread(words, sizeof(uint32_t), n, f) != n) {
        fprintf(stderr, "ERROR: short read\n");
        free(words); free(re); free(im); fclose(f);
        return 1;
    }
    fclose(f);

    for (size_t i = 0; i < n; i++) {
        re[i] = (float)iq_real(words[i]) / 2047.0f;
        im[i] = (float)iq_imag(words[i]) / 2047.0f;
    }
    free(words);

    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "ERROR: FFT plan creation failed\n");
        free(re); free(im);
        return 1;
    }

    fprintf(stderr, "validate_window: %s (%zu samples, %.1f ms)\n",
            path, n, (double)n / 20000.0);

    size_t cursor = 0;
    int n_frames = 0, n_fcs_ok = 0;
    bool m2_decodable = false;
    size_t m2_start = 0;

    printf("{\"path\":\"%s\",\"samples\":%zu,\"frames\":[", path, n);

    while (cursor + MIN_BUF_SAMPLES < n && n_frames < MAX_FRAMES) {
        lib80211_sync_result sync;
        if (lib80211_sync_detect(plan, re + cursor, im + cursor,
                                 n - cursor, &sync) != 0)
            break;

        size_t fs = cursor + sync.frame_start;
        lib80211_rx_result res;
        int rc = lib80211_rx_decode(plan, re + fs, im + fs, n - fs, &res);

        bool eapol = (rc == 0) && is_eapol(res.psdu, res.psdu_len);
        bool this_m2 = (rc == 0) && res.fcs_valid && res.psdu_len == 159 &&
                       eapol;

        if (n_frames) printf(",");
        printf("{\"start\":%zu,\"ltf\":%zu,\"rate\":%d,\"len\":%zu,"
               "\"fcs\":%s,\"eapol\":%s}",
               fs, cursor + sync.ltf_start,
               rc == 0 ? res.rate_mbps : 0,
               rc == 0 ? res.psdu_len : 0,
               (rc == 0 && res.fcs_valid) ? "true" : "false",
               eapol ? "true" : "false");

        fprintf(stderr,
                "  [%3d] start=%zu ltf=%zu rate=%d len=%zu fcs=%s%s%s\n",
                n_frames, fs, cursor + sync.ltf_start,
                rc == 0 ? res.rate_mbps : 0,
                rc == 0 ? res.psdu_len : 0,
                (rc == 0 && res.fcs_valid) ? "OK" : "FAIL",
                eapol ? " EAPOL" : "",
                this_m2 ? "  <-- M2 DECODABLE" : "");

        if (rc == 0 && res.fcs_valid) n_fcs_ok++;
        if (this_m2) { m2_decodable = true; m2_start = fs; }

        /* Advance past this frame; on decode failure skip past the preamble
         * so the next sync_detect does not re-find the same frame. */
        size_t adv;
        if (rc == 0) {
            int mbps = res.rate_mbps ? res.rate_mbps : 6;
            adv = sync.frame_start + est_frame_samples(mbps, res.psdu_len) + 64;
        } else {
            adv = sync.frame_start + 320;
        }
        if (adv == 0) adv = 1;
        cursor += adv;
        n_frames++;
    }

    printf("],\"n_frames\":%d,\"n_fcs_ok\":%d,\"m2_decodable\":%s,"
           "\"m2_start\":%zu}\n",
           n_frames, n_fcs_ok, m2_decodable ? "true" : "false", m2_start);

    fprintf(stderr, "\n--- validate_window summary ---\n");
    fprintf(stderr, "frames=%d fcs_ok=%d  M2 (EAPOL len159) decodable: %s\n",
            n_frames, n_fcs_ok, m2_decodable ? "YES" : "NO");

    lib80211_fft_plan_destroy(plan);
    free(re);
    free(im);
    return 0;
}
