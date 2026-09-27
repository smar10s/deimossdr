// SPDX-License-Identifier: MIT
/*
 * deimos_burst_loopback — Burst cable loopback with tag-only FIFO readback
 *
 * Exercises the complete fabric → tag FIFO → ARM path under realistic
 * traffic conditions. Transmits a burst of N frames at once, then reads
 * decoded tags from the tag FIFO (TAG_LO + TAG_HI registers).
 *
 * Tag-only: verifies rate, length, FCS, and DDR offset for each decoded
 * frame. No byte FIFO — payload verification is not done here (ARM decode
 * from DDR is a separate tool concern).
 *
 * Verification:
 *   - Frames are matched to manifest by (rate_code, length) pairs
 *   - FCS pass rate is the primary metric
 *   - DDR offset must be non-zero and monotonically increasing
 *   - Tag drop count from hardware shows FIFO overflow events
 *
 * Usage:
 *   deimos_burst_loopback -n 20              # 20 frames, mixed rates
 *   deimos_burst_loopback -n 50 -r 6         # 50 frames at rate 6 only
 *   deimos_burst_loopback --eapol            # inject EAPOL-sized frames
 *   deimos_burst_loopback -g 200             # 200-sample inter-frame gap
 *   deimos_burst_loopback --stress           # max frames, min gap
 *   deimos_burst_loopback --file burst.json  # TX a JSON capture through cable
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <getopt.h>
#include <time.h>

#include "deimos_tool.h"
#include "dma_tx.h"
#include "deimos_rx.h"

#include <lib80211/fft.h>
#include <lib80211/tx.h>
#include <lib80211/rx.h>
#include <lib80211/mac.h>


/* --------------------------------------------------------------------------
 * JSON waveform loader (for --file mode) — uses deimos_tool shared library
 * -------------------------------------------------------------------------- */

/* Wrapper matching the old load_capture_file signature for minimal diff */
static int load_capture_file(const char *path, float **re, float **im, int *n_samples,
                             char **json_out)
{
    return deimos_tool_load_waveform(path, re, im, n_samples, NULL, json_out);
}

/* --------------------------------------------------------------------------
 * Expected frames parsing (for --file --verify-psdu)
 * -------------------------------------------------------------------------- */

#define MAX_PSDU_BYTES 1500
#define MAX_EXPECTED_FRAMES 32

typedef struct {
    uint8_t rate_code;
    uint16_t length;
    uint8_t psdu[MAX_PSDU_BYTES];
    uint16_t psdu_len;  /* actual byte count from hex (length - 4 excludes FCS) */
} expected_frame_t;

/* Parse hex string into byte array. Returns number of bytes parsed. */
static int hex_to_bytes(const char *hex, int hex_len, uint8_t *out, int max_out)
{
    int nbytes = hex_len / 2;
    if (nbytes > max_out) nbytes = max_out;
    for (int i = 0; i < nbytes; i++) {
        unsigned int byte;
        sscanf(hex + i * 2, "%2x", &byte);
        out[i] = (uint8_t)byte;
    }
    return nbytes;
}

/* Parse expected_frames array from JSON buffer. Returns count, or 0 if not found. */
static int parse_expected_frames(const char *json, expected_frame_t *out, int max_frames)
{
    const char *p = strstr(json, "\"expected_frames\"");
    if (!p) return 0;

    p = strchr(p, '[');
    if (!p) return 0;
    p++;  /* skip '[' */

    int count = 0;
    while (count < max_frames) {
        /* Find next object */
        const char *obj = strchr(p, '{');
        if (!obj) break;

        /* Find end of this object */
        const char *obj_end = strchr(obj, '}');
        if (!obj_end) break;

        /* Parse rate_code */
        const char *rc = strstr(obj, "\"rate_code\"");
        if (rc && rc < obj_end) {
            rc = strchr(rc, ':');
            if (rc) out[count].rate_code = (uint8_t)atoi(rc + 1);
        }

        /* Parse length */
        const char *ln = strstr(obj, "\"length\"");
        if (ln && ln < obj_end) {
            ln = strchr(ln, ':');
            if (ln) out[count].length = (uint16_t)atoi(ln + 1);
        }

        /* Parse psdu_hex */
        const char *ph = strstr(obj, "\"psdu_hex\"");
        if (ph && ph < obj_end) {
            ph = strchr(ph, ':');
            if (ph) {
                /* Find the quoted string */
                const char *qs = strchr(ph, '"');
                if (qs) {
                    qs++;
                    const char *qe = strchr(qs, '"');
                    if (qe) {
                        int hex_len = (int)(qe - qs);
                        out[count].psdu_len = hex_to_bytes(qs, hex_len,
                            out[count].psdu, MAX_PSDU_BYTES);
                    }
                }
            }
        }

        count++;
        p = obj_end + 1;
    }

    return count;
}

/* --------------------------------------------------------------------------
 * Tag FIFO registers (via hal.h: REG_TAG_FIFO_*)
 * Additional aliases for local use (burst_loopback diagnostic registers)
 * -------------------------------------------------------------------------- */

#define REG_TAG_STATUS       REG_TAG_FIFO_STATUS
#define REG_TAG_FRAME_CNT    REG_TAG_FIFO_FRAME_CNT
#define REG_TAG_ACCEPTED_CNT (REG_TAG_FIFO_BASE + 0x20)
#define REG_TAG_ABORT_CNT    (REG_TAG_FIFO_BASE + 0x24)

/* HIL registers (via hal.h: REG_HIL_CTRL_*)
 * Local aliases for readability in the extensive diagnostic code */
#define REG_HIL_BASE         REG_HIL_CTRL_BASE
#define REG_HIL_CONTROL      REG_HIL_CTRL_CONTROL
/* SNAP_MODE lives in deimos_regs_axi (ACCEL_B + 0x0C), NOT in hil_ctrl.
 * The old REG_HIL_CTRL_SNAP_MODE alias resolved to (REG_HIL_CTRL_BASE+0x1C),
 * which hil_ctrl decodes as index 7 and silently ignores — every snap-mode
 * write in this file was a no-op. Use the canonical REG_DEIMOS_SNAP_MODE. */

/* SNAP_CTRL_ARM, SNAP_CTRL_SWTRIG, SNAP_CTRL_CIRCULAR from deimos_tool.h;
 * REG_SNAP_* from hal.h (styx submodule) */
#define SNAP_CTRL_SW_TRIG    SNAP_CTRL_SWTRIG  /* local alias */

/* HIL inject mode */
#define REG_HIL_DDR_BASE_REG REG_HIL_CTRL_DDR_BASE
#define REG_HIL_PLAY_COUNT   REG_HIL_CTRL_PLAY_COUNT
/* HIL playback buffer: single definition in hal_deimos.h (was 0x18000000
 * here, which aliased the DMA TX buffer that dma_tx_load fills). */
#define HIL_DDR_BASE         DEIMOS_HIL_DDR_BASE
#define HIL_MAX_SAMPLES      DEIMOS_HIL_MAX_SAMPLES

/* --------------------------------------------------------------------------
 * Frame definitions
 * -------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ       20000000ULL
#define BANDWIDTH_HZ         28000000ULL
#define DEFAULT_CHANNEL      149
#define DEFAULT_GAP_SAMPLES  400    /* 20 us inter-frame gap (realistic DIFS) */
#define MAX_BURST_FRAMES     200
#define MAX_PSDU_LEN         1500
/* Rate codes from shared library */
#define RATE_CODES DEIMOS_RATE_CODES

static const int SUPPORTED_RATES[] = {6, 12, 24};
#define N_SUPPORTED_RATES 3

static const int MIXED_RATES[] = {6, 9, 12, 18, 24, 36};
#define N_MIXED_RATES 6

static const char *rate_to_str(uint8_t code) {
    static char buf[4];
    int mbps = deimos_rx_rate_mbps(code);
    if (mbps == 0)
        return "??";
    snprintf(buf, sizeof(buf), "%2d", mbps);
    return buf;
}

/* --------------------------------------------------------------------------
 * Frame manifest — what we transmit and expect to receive
 * -------------------------------------------------------------------------- */

typedef struct {
    int rate_mbps;
    size_t psdu_len;      /* including FCS */
    int payload_bytes;    /* payload only (for TX regeneration) */
    bool is_eapol;
    uint8_t seq_num;      /* unique frame ID */
} frame_manifest_t;

typedef struct {
    int n_frames;
    frame_manifest_t frames[MAX_BURST_FRAMES];
} burst_manifest_t;

/* --------------------------------------------------------------------------
 * Frame builders
 * -------------------------------------------------------------------------- */

/* Use deimos_rx_channel_to_freq() from deimos_rx.h */
#define channel_to_freq deimos_rx_channel_to_freq

/* Use shared frame builder */
#define build_frame deimos_tool_build_frame

/* --------------------------------------------------------------------------
 * Burst generation
 * -------------------------------------------------------------------------- */

static void build_manifest(burst_manifest_t *m, int n_frames,
                           int fixed_rate, int payload_size,
                           bool inject_eapol, bool mix_rates,
                           unsigned int seed)
{
    srand(seed);
    m->n_frames = n_frames;

    int min_payload = (payload_size > 0) ? payload_size : 40;
    int max_payload = (payload_size > 0) ? payload_size : 400;
    int eapol_interval = inject_eapol ? 5 : 0;

    for (int i = 0; i < n_frames; i++) {
        frame_manifest_t *f = &m->frames[i];
        f->seq_num = (uint8_t)(i & 0xFF);

        /* Rate selection */
        if (fixed_rate > 0) {
            f->rate_mbps = fixed_rate;
        } else if (mix_rates) {
            f->rate_mbps = MIXED_RATES[rand() % N_MIXED_RATES];
        } else {
            f->rate_mbps = SUPPORTED_RATES[rand() % N_SUPPORTED_RATES];
        }

        /* EAPOL injection */
        f->is_eapol = (eapol_interval > 0 && ((i % eapol_interval) == 2));

        /* Payload size */
        int plen;
        if (f->is_eapol) {
            plen = 95;  /* realistic EAPOL key frame size */
        } else if (payload_size > 0) {
            plen = payload_size;
        } else {
            plen = min_payload + (rand() % (max_payload - min_payload + 1));
        }

        /* Build frame to get the actual PSDU length */
        uint8_t tmp[MAX_PSDU_LEN + 4];
        f->payload_bytes = plen;
        f->psdu_len = build_frame(tmp, plen, f->seq_num, f->is_eapol);
    }
}

/*
 * Generate IQ waveform for the entire burst.
 * If hil_mode: write to HIL DDR buffer (digital inject, no RF).
 * Otherwise: load into TX DMA (cable loopback via DAC).
 * Returns total number of samples, or 0 on error.
 */
static size_t generate_burst_iq(lib80211_fft_plan *plan,
                                const burst_manifest_t *m,
                                int gap_samples, bool hil_mode,
                                const char *dump_hil_path)
{
    /* First pass: compute total sample count */
    size_t pre_pad = hil_mode ? 0 : DEIMOS_SILENCE_PAD;
    size_t total_samples = pre_pad;
    for (int i = 0; i < m->n_frames; i++) {
        lib80211_tx_legacy_params params = {
            .rate_mbps = m->frames[i].rate_mbps,
            .psdu_len = m->frames[i].psdu_len,
        };
        total_samples += lib80211_tx_legacy_samples(&params);
        if (i < m->n_frames - 1)
            total_samples += gap_samples;
    }
    total_samples += DEIMOS_SILENCE_PAD;

    if (total_samples > DMA_TX_MAX_SAMPLES) {
        fprintf(stderr, "ERROR: burst too large (%zu samples, max %d)\n",
                total_samples, DMA_TX_MAX_SAMPLES);
        return 0;
    }

    /* Allocate and fill */
    float *tx_real = calloc(total_samples, sizeof(float));
    float *tx_imag = calloc(total_samples, sizeof(float));
    if (!tx_real || !tx_imag) {
        free(tx_real); free(tx_imag);
        fprintf(stderr, "ERROR: malloc failed for %zu samples\n", total_samples);
        return 0;
    }

    /* Generate each frame from manifest metadata */
    size_t offset = pre_pad;
    for (int i = 0; i < m->n_frames; i++) {
        const frame_manifest_t *f = &m->frames[i];

        uint8_t psdu[MAX_PSDU_LEN + 4];
        size_t psdu_len = build_frame(psdu, f->payload_bytes,
                                      f->seq_num, f->is_eapol);

        lib80211_tx_legacy_params params = {
            .rate_mbps = f->rate_mbps,
            .psdu = psdu,
            .psdu_len = psdu_len,
            .scrambler_seed = (uint8_t)(1 + (i % 127)),  /* valid 7-bit seeds: 1-127 */
        };

        size_t gen = lib80211_tx_legacy(plan, &params,
                                        tx_real + offset, tx_imag + offset);
        if (gen == 0) {
            fprintf(stderr, "ERROR: lib80211_tx_legacy failed for frame %d\n", i);
            free(tx_real); free(tx_imag);
            return 0;
        }
        offset += gen;

        if (i < m->n_frames - 1)
            offset += gap_samples;
    }

    /* Load waveform */
    if (hil_mode) {
        /* HIL: add noise floor to silence regions.
         * STF uses normalized autocorrelation — needs non-zero energy in
         * the denominator between frames to reset properly. Without noise,
         * the metric stays latched after the first frame and subsequent
         * STF detections never fire. Noise level: ~1% of signal peak. */
        float sig_peak = 0.0f;
        for (size_t i = 0; i < total_samples; i++) {
            float ar = fabsf(tx_real[i]);
            float ai = fabsf(tx_imag[i]);
            if (ar > sig_peak) sig_peak = ar;
            if (ai > sig_peak) sig_peak = ai;
        }
        float noise_level = sig_peak * 0.01f;  /* -40 dB below signal (1%) */
        for (size_t i = 0; i < total_samples; i++) {
            if (tx_real[i] == 0.0f && tx_imag[i] == 0.0f) {
                tx_real[i] = noise_level * ((float)rand() / RAND_MAX * 2.0f - 1.0f);
                tx_imag[i] = noise_level * ((float)rand() / RAND_MAX * 2.0f - 1.0f);
            }
        }

        /* Quantize to 12-bit and write to HIL DDR region */
        if (total_samples > HIL_MAX_SAMPLES) {
            fprintf(stderr, "ERROR: burst too large for HIL (%zu > %d samples)\n",
                    total_samples, HIL_MAX_SAMPLES);
            free(tx_real); free(tx_imag);
            return 0;
        }
        volatile uint32_t *tx_buf = hal_ddr_tx_buf();
        volatile uint32_t *hil_buf = tx_buf + (HIL_DDR_BASE - DDR_TX_BASE) / 4;
        deimos_tool_quantize_to_ddr(tx_real, tx_imag, (int)total_samples, hil_buf);

        if (dump_hil_path) {
            FILE *df = fopen(dump_hil_path, "wb");
            if (df) {
                fwrite((const void *)hil_buf, sizeof(uint32_t), total_samples, df);
                fclose(df);
                fprintf(stderr, "  HIL dump: %zu words -> %s\n",
                        total_samples, dump_hil_path);
            } else {
                fprintf(stderr, "  WARNING: cannot open HIL dump path %s\n",
                        dump_hil_path);
            }
        }
    } else {
        /* Cable: load into TX DMA (one-shot) */
        if (dma_tx_load(tx_real, tx_imag, total_samples, false) != 0) {
            fprintf(stderr, "ERROR: dma_tx_load failed\n");
            free(tx_real); free(tx_imag);
            return 0;
        }
    }

    free(tx_real);
    free(tx_imag);
    return total_samples;
}

/* --------------------------------------------------------------------------
 * RX: read tags from FIFO (tag-only, no byte reads)
 * -------------------------------------------------------------------------- */

typedef struct {
    uint32_t tag_lo;
    uint32_t tag_hi;
    uint8_t rate_code;
    uint16_t length;
    bool fcs_ok;
    uint16_t psdu_addr;     /* BRAM base address from TAG_HI */
} rx_tag_t;

typedef struct {
    int rx_count;
    rx_tag_t tags[MAX_BURST_FRAMES * 2];
    uint8_t psdu_buf[MAX_BURST_FRAMES * 2][MAX_PSDU_BYTES];
    uint16_t psdu_len[MAX_BURST_FRAMES * 2];
    uint32_t hw_frame_cnt;
    uint32_t hw_drop_cnt;
} rx_result_t;

/*
 * Drain the tag FIFO via deimos_rx_poll(), collecting all available tags.
 * PSDU bytes are always read (deimos_rx_poll reads them for FCS-OK frames).
 */
static void drain_tags(rx_result_t *rx, int timeout_ms, int drain_quiet_ms,
                       bool verify_psdu)
{
    (void)verify_psdu;  /* PSDU always read by deimos_rx_poll */
    rx->rx_count = 0;

    int wait_ms = 0;
    int quiet_ms = 0;
    int max_tags = MAX_BURST_FRAMES * 2;

    while (wait_ms < timeout_ms && rx->rx_count < max_tags) {
        deimos_rx_frame_t frame;
        int rc = deimos_rx_poll(&frame);

        if (rc == 0) {
            usleep(1000);
            wait_ms++;
            quiet_ms++;
            if (rx->rx_count > 0 && quiet_ms >= drain_quiet_ms)
                break;
            continue;
        }
        if (rc < 0) break;

        quiet_ms = 0;

        rx_tag_t *t = &rx->tags[rx->rx_count];
        t->tag_lo = 0;  /* raw tag words no longer exposed */
        t->tag_hi = 0;
        t->fcs_ok = frame.fcs_ok;
        t->rate_code = frame.rate_code;
        t->length = frame.length;
        t->psdu_addr = frame.psdu_addr;

        /* Copy PSDU bytes */
        if (frame.psdu_len > 0) {
            memcpy(rx->psdu_buf[rx->rx_count], frame.psdu, frame.psdu_len);
            rx->psdu_len[rx->rx_count] = frame.psdu_len;
        } else {
            rx->psdu_len[rx->rx_count] = 0;
        }

        rx->rx_count++;
    }

    /* Read hardware counters */
    rx->hw_frame_cnt = deimos_rx_frame_count();
    rx->hw_drop_cnt = deimos_rx_drop_count();
}

/*
 * PSDU verification for file mode: compare BRAM-read PSDU bytes
 * against expected_frames parsed from the JSON capture file.
 * Returns 0 on success (all matched), 1 if any mismatch occurred.
 */
static int verify_psdu_file_mode(const char *json_buf, const rx_result_t *rx,
                                 bool verbose, int *out_ok, int *out_fail)
{
    if (out_ok) *out_ok = 0;
    if (out_fail) *out_fail = 0;

    if (!json_buf) {
        fprintf(stderr, "  PSDU verify: no JSON buffer available (skipping)\n");
        return 0;
    }

    expected_frame_t expected_frames[MAX_EXPECTED_FRAMES];
    int n_expected = parse_expected_frames(json_buf, expected_frames, MAX_EXPECTED_FRAMES);

    if (n_expected == 0) {
        fprintf(stderr, "  PSDU verify: no expected_frames in JSON (skipping)\n");
        return 0;
    }

    int psdu_ok = 0, psdu_fail = 0;
    bool ef_matched[MAX_EXPECTED_FRAMES];
    memset(ef_matched, 0, sizeof(ef_matched));

    for (int i = 0; i < rx->rx_count; i++) {
        const rx_tag_t *t = &rx->tags[i];
        if (!t->fcs_ok || rx->psdu_len[i] == 0) continue;

        /* Find matching expected_frame by (rate_code, length) */
        int ef_idx = -1;
        for (int j = 0; j < n_expected; j++) {
            if (ef_matched[j]) continue;
            if (t->rate_code == expected_frames[j].rate_code &&
                t->length == expected_frames[j].length) {
                ef_idx = j;
                ef_matched[j] = true;
                break;
            }
        }
        if (ef_idx < 0) continue;  /* No match in expected — skip */

        /* Compare bytes */
        uint16_t cmp_len = expected_frames[ef_idx].psdu_len;
        if (cmp_len != rx->psdu_len[i]) {
            psdu_fail++;
            if (verbose)
                fprintf(stderr, "  TAG[%2d]: PSDU len mismatch: got %u, expect %u\n",
                        i, rx->psdu_len[i], cmp_len);
            continue;
        }

        bool match = true;
        for (uint16_t b = 0; b < cmp_len; b++) {
            if (rx->psdu_buf[i][b] != expected_frames[ef_idx].psdu[b]) {
                match = false;
                if (verbose)
                    fprintf(stderr, "  TAG[%2d]: PSDU byte %u mismatch: got 0x%02x, expect 0x%02x\n",
                            i, b, rx->psdu_buf[i][b], expected_frames[ef_idx].psdu[b]);
                break;
            }
        }
        if (match) psdu_ok++;
        else psdu_fail++;
    }

    fprintf(stderr, "  PSDU verify (file): %d/%d match\n",
            psdu_ok, psdu_ok + psdu_fail);

    if (out_ok) *out_ok = psdu_ok;
    if (out_fail) *out_fail = psdu_fail;

    return (psdu_fail > 0) ? 1 : 0;
}

/* --------------------------------------------------------------------------
 * Verification: compare RX tags against manifest
 * -------------------------------------------------------------------------- */

typedef struct {
    int matched;          /* tags matched to manifest by (rate, length) */
    int fcs_pass;         /* total FCS-ok tags */
    int fcs_fail;         /* total FCS-fail tags */
    int unmatched;        /* FCS-ok but no manifest match (spurious) */
    int eapol_sent;
    int eapol_matched;    /* manifest EAPOL entries that got a matching tag */
    uint32_t hw_tag_drops;
} verify_result_t;

/*
 * Match received tags against manifest.
 * Since we can't read payload, we match by (rate_code, length) pairs.
 * For mixed-rate bursts with unique lengths, this is unambiguous.
 * For same-rate same-size bursts, we match in order.
 */
static verify_result_t verify_burst(const rx_result_t *rx,
                                    const burst_manifest_t *m)
{
    verify_result_t v = {0};

    bool manifest_matched[MAX_BURST_FRAMES] = {false};

    /* Count EAPOL in manifest */
    for (int i = 0; i < m->n_frames; i++) {
        if (m->frames[i].is_eapol)
            v.eapol_sent++;
    }

    for (int i = 0; i < rx->rx_count; i++) {
        const rx_tag_t *t = &rx->tags[i];

        if (t->fcs_ok)
            v.fcs_pass++;
        else
            v.fcs_fail++;
        if (!t->fcs_ok)
            continue;

        /* Match against manifest: find first unmatched entry with same rate+length */
        uint8_t expected_rate = 0;
        bool found = false;
        for (int j = 0; j < m->n_frames; j++) {
            if (manifest_matched[j])
                continue;
            expected_rate = RATE_CODES[m->frames[j].rate_mbps];
            if (t->rate_code == expected_rate &&
                t->length == (uint16_t)m->frames[j].psdu_len) {
                manifest_matched[j] = true;
                v.matched++;
                if (m->frames[j].is_eapol)
                    v.eapol_matched++;
                found = true;
                break;
            }
        }
        if (!found)
            v.unmatched++;
    }

    v.hw_tag_drops = rx->hw_drop_cnt;
    return v;
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n\n"
        "  Burst cable loopback: TX mixed frames, verify fabric decode via tag FIFO.\n"
        "  Tag-only: validates rate, length, FCS, and DDR offset.\n\n"
        "Options:\n"
        "  -n count       Number of frames in burst (default: 20, max: %d)\n"
        "  -r rate        Fixed rate (6/9/12/18/24/36); default: mixed 6/12/24\n"
        "  -p payload     Fixed payload size in bytes (default: random 40-400)\n"
        "  -g gap         Inter-frame gap in samples (default: %d = %.0f us)\n"
        "  -c channel     RF channel (default: %d)\n"
        "  --eapol        Inject EAPOL frames (every 5th frame)\n"
        "  --mix-rates    Include rates 6-36 (default: 6/12/24 only)\n"
        "  --stress       Maximum stress: 100 frames, 100-sample gap\n"
        "  --verify-psdu  Read PSDU bytes from fabric BRAM, compare to expected\n"
        "  --hil          HIL mode: inject burst via DDR (no RF, digital only)\n"
        "  --file <path>  TX a JSON capture file through cable loopback\n"
        "  --dump-hil <path>  Save quantized HIL waveform (raw 32-bit words)\n"
        "                 (format: {\"real\":[...],\"imag\":[...]})\n"
        "  -v             Verbose (per-tag details)\n"
        "  -h             Help\n\n"
        "Returns: 0 if all FCS-ok tags match manifest, 1 otherwise.\n",
        prog, MAX_BURST_FRAMES,
        DEFAULT_GAP_SAMPLES, DEFAULT_GAP_SAMPLES * 1e6 / SAMPLE_RATE_HZ,
        DEFAULT_CHANNEL);
}

int main(int argc, char *argv[])
{
    int n_frames = 20;
    int fixed_rate = 0;
    int payload_size = 0;
    int gap_samples = DEFAULT_GAP_SAMPLES;
    int channel = DEFAULT_CHANNEL;
    bool inject_eapol = false;
    bool mix_rates = false;
    bool verbose = false;
    bool verify_psdu = false;
    bool hil_mode = false;
    const char *file_path = NULL;
    const char *dump_hil_path = NULL;

    static struct option long_opts[] = {
        {"eapol",      no_argument,       NULL, 'E'},
        {"mix-rates",  no_argument,       NULL, 'M'},
        {"stress",     no_argument,       NULL, 'S'},
        {"verify-psdu", no_argument,      NULL, 'P'},
        {"hil",        no_argument,       NULL, 'H'},
        {"file",       required_argument, NULL, 'F'},
        {"dump-hil",   required_argument, NULL, 'D'},
        {"help",       no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "n:r:p:g:c:vh", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'n': n_frames = atoi(optarg); break;
        case 'r': fixed_rate = atoi(optarg); break;
        case 'p': payload_size = atoi(optarg); break;
        case 'g': gap_samples = atoi(optarg); break;
        case 'c': channel = atoi(optarg); break;
        case 'E': inject_eapol = true; break;
        case 'M': mix_rates = true; break;
        case 'S': n_frames = 100; gap_samples = 100; inject_eapol = true; break;
        case 'P': verify_psdu = true; break;
        case 'H': hil_mode = true; break;
        case 'F': file_path = optarg; break;
        case 'D': dump_hil_path = optarg; break;
        case 'v': verbose = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (n_frames < 1 || n_frames > MAX_BURST_FRAMES) {
        fprintf(stderr, "ERROR: frame count must be 1-%d\n", MAX_BURST_FRAMES);
        return 1;
    }

    /* Initialize radio via unified API */
    deimos_radio_config_t radio_cfg = {
        .channel = channel,
        .rx_gain_db = 24.0,
        .agc_mode = DEIMOS_AGC_MANUAL,
        .configure_tx = !hil_mode,
        .tx_atten_db = 3.0,
        .bandwidth_mhz = 0,  /* default 28 MHz */
        .skip_calibration = false,
    };
    if (deimos_radio_init(&radio_cfg) != 0) {
        fprintf(stderr, "ERROR: deimos_radio_init failed\n");
        return 1;
    }

    uint64_t freq = deimos_rx_channel_to_freq(channel);

    /* Create FFT plan */
    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "ERROR: FFT plan creation failed\n");
        deimos_rx_cleanup();
        return 1;
    }

    /* ======================================================================
     * --file mode: load a JSON capture and inject it.
     *   --file alone: TX through cable (DAC → cable → ADC → fabric)
     *   --file --hil: inject via HIL DDR (digital, no RF)
     * Reports how many tags the fabric produces (no manifest matching).
     * ====================================================================== */
    char *json_buf = NULL;  /* retained for --verify-psdu in file mode */

    if (file_path) {
        float *file_re = NULL, *file_im = NULL;
        int file_n = 0;

        fprintf(stderr, "Loading capture: %s\n", file_path);
        if (load_capture_file(file_path, &file_re, &file_im, &file_n,
                              verify_psdu ? &json_buf : NULL) != 0) {
            lib80211_fft_plan_destroy(plan);
            deimos_rx_cleanup();
            return 1;
        }
        fprintf(stderr, "  %d samples (%.1f ms)\n",
                file_n, file_n * 1000.0 / SAMPLE_RATE_HZ);

        if (hil_mode) {
            /* --file --hil: inject capture via HIL DDR (digital path) */
            if (file_n > HIL_MAX_SAMPLES) {
                fprintf(stderr, "ERROR: capture too large for HIL (%d samples, max %d)\n",
                        file_n, HIL_MAX_SAMPLES);
                free(file_re); free(file_im);
                lib80211_fft_plan_destroy(plan);
                deimos_rx_cleanup();
                return 1;
            }

            /* Quantize to 12-bit and write to HIL DDR region.
             * Captures store raw 12-bit ADC values (±2047). Scale to use
             * 90% of the 12-bit range to preserve signal dynamics without
             * clipping. Add noise floor to near-silence regions so the
             * correlator's energy denominator can reset between frames. */
            float peak = 0.0f;
            for (int i = 0; i < file_n; i++) {
                float ar = fabsf(file_re[i]);
                float ai = fabsf(file_im[i]);
                if (ar > peak) peak = ar;
                if (ai > peak) peak = ai;
            }
            float scale = (peak > 0.0f) ? (2047.0f * 0.9f) / peak : 1.0f;

            /* Add noise floor to near-silence regions (same as synthetic HIL).
             * Without this, tight single-frame extractions with quiet pre-STF
             * regions starve the correlator energy denominator. */
            float noise_level = peak * 0.02f;  /* 2% of signal peak */
            for (int i = 0; i < file_n; i++) {
                if (fabsf(file_re[i]) < noise_level && fabsf(file_im[i]) < noise_level) {
                    file_re[i] += noise_level * ((float)rand() / RAND_MAX * 2.0f - 1.0f);
                    file_im[i] += noise_level * ((float)rand() / RAND_MAX * 2.0f - 1.0f);
                }
            }

            fprintf(stderr, "  HIL inject: peak=%.1f, scale=%.4f (→90%% FS), noise=%.1f\n",
                    peak, scale, noise_level);

            volatile uint32_t *tx_buf = hal_ddr_tx_buf();
            volatile uint32_t *hil_buf = tx_buf + (HIL_DDR_BASE - DDR_TX_BASE) / 4;
            for (int i = 0; i < file_n; i++) {
                int16_t ri = (int16_t)roundf(file_re[i] * scale);
                int16_t qi = (int16_t)roundf(file_im[i] * scale);
                if (ri > 2047) ri = 2047;
                if (ri < -2048) ri = -2048;
                if (qi > 2047) qi = 2047;
                if (qi < -2048) qi = -2048;
                hil_buf[i] = IQ_PACK(ri, qi);
            }
            free(file_re);
            free(file_im);

            /* HIL trigger sequence: warm-up → arm → real pass */
            hal_reg_write(REG_HIL_DDR_BASE_REG, HIL_DDR_BASE);
            hal_reg_write(REG_HIL_PLAY_COUNT, (uint32_t)file_n);

            /* Warm-up pass */
            hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE | HIL_CTRL_TRIGGER);
            int warmup_us = (int)((double)file_n / 20.0 * 5.0) + 20000;
            usleep(warmup_us);
            hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE);
            usleep(15000);

            /* ARM PIPELINE between warm-up and real pass.
             * Replaces manual STF toggle + tag flush. */
            deimos_pipeline_config_t file_hil_pipe = DEIMOS_PIPELINE_HIL_DEFAULT;
            deimos_pipeline_arm(&file_hil_pipe);

            /* Real pass: enable STF + trigger HIL */
            deimos_stf_gate(DEIMOS_TRIGGER_HIL);

            /* Drain tags */
            double burst_ms = file_n * 1000.0 / SAMPLE_RATE_HZ;
            int timeout_ms = (int)(burst_ms * 5.0) + 500;  /* 5× real-time (1-in-5 playback) */
            static rx_result_t rx;
            memset(&rx, 0, sizeof(rx));
            drain_tags(&rx, timeout_ms, 200, verify_psdu);

            /* Report */
            fprintf(stderr, "\n");
            fprintf(stderr, "============================================================\n");
            fprintf(stderr, "FILE HIL INJECT RESULTS: %s\n", file_path);
            fprintf(stderr, "============================================================\n");
            fprintf(stderr, "  Samples:         %d (%.1f ms)\n", file_n, burst_ms);
            fprintf(stderr, "  RX tags:         %d\n", rx.rx_count);

            int fcs_pass = 0, fcs_fail = 0;
            for (int i = 0; i < rx.rx_count; i++) {
                if (rx.tags[i].fcs_ok) fcs_pass++;
                else fcs_fail++;
            }
            fprintf(stderr, "  FCS pass:        %d\n", fcs_pass);
            fprintf(stderr, "  FCS fail:        %d\n", fcs_fail);
            fprintf(stderr, "  HW frame_cnt:    %u\n", rx.hw_frame_cnt);
            fprintf(stderr, "  HW drop_cnt:     %u\n", rx.hw_drop_cnt);

            /* Per-tag dump */
            fprintf(stderr, "\nPer-tag:\n");
            for (int i = 0; i < rx.rx_count; i++) {
                const rx_tag_t *t = &rx.tags[i];
                fprintf(stderr, "  [%2d] rate=%s fcs=%s len=%3u bram=0x%04x\n",
                        i, rate_to_str(t->rate_code),
                        t->fcs_ok ? "OK" : "!!",
                        t->length, t->psdu_addr);
            }

            /* PSDU verification (file mode) — run before the JSON line so the
             * machine-readable counts the ladders parse include PSDU results. */
            int psdu_result = 0;
            int psdu_ok = 0, psdu_fail = 0;
            if (verify_psdu) {
                psdu_result = verify_psdu_file_mode(json_buf, &rx, verbose,
                                                    &psdu_ok, &psdu_fail);
                free(json_buf);
                json_buf = NULL;
            }

            /* JSON output */
            printf("{\"mode\":\"file_hil\",\"file\":\"%s\",\"samples\":%d,"
                   "\"rx_tags\":%d,\"fcs_pass\":%d,\"fcs_fail\":%d,"
                   "\"hw_frame_cnt\":%u,\"hw_drop_cnt\":%u",
                   file_path, file_n, rx.rx_count, fcs_pass, fcs_fail,
                   rx.hw_frame_cnt, rx.hw_drop_cnt);
            if (verify_psdu)
                printf(",\"psdu_ok\":%d,\"psdu_fail\":%d", psdu_ok, psdu_fail);
            printf("}\n");

            lib80211_fft_plan_destroy(plan);
            deimos_rx_cleanup();
            return (fcs_pass > 0 && psdu_result == 0) ? 0 : 1;
        }

        /* --file without --hil: cable loopback path (original) */
        if (file_n > DMA_TX_MAX_SAMPLES) {
            fprintf(stderr, "ERROR: capture too large (%d samples, max %d)\n",
                    file_n, DMA_TX_MAX_SAMPLES);
            free(file_re); free(file_im);
            lib80211_fft_plan_destroy(plan);
            deimos_rx_cleanup();
            return 1;
        }

        /* Scale: captures store raw 12-bit ADC values (±2047).
         * dma_tx_load expects float normalized to [-1, 1].
         * Use 99.9th percentile as "effective peak" to avoid being crushed by
         * single-sample clipping artifacts. Target: effective peak → 0.25 DAC FS
         * (same level as single-frame cable loopback tools). */
        float peak = 0;
        /* Find 99.9th percentile: sort absolute values, pick index 0.999*N.
         * For 1M samples this is fast enough on ARM (<100ms). */
        int n_abs = file_n * 2;  /* both I and Q channels */
        float *abs_vals = malloc(n_abs * sizeof(float));
        if (!abs_vals) {
            fprintf(stderr, "ERROR: malloc failed for percentile calc\n");
            free(file_re); free(file_im);
            lib80211_fft_plan_destroy(plan);
            deimos_rx_cleanup();
            return 1;
        }
        for (int i = 0; i < file_n; i++) {
            abs_vals[2*i]     = fabsf(file_re[i]);
            abs_vals[2*i + 1] = fabsf(file_im[i]);
        }
        /* Partial sort: find the value at index 99.9% */
        /* Simple approach: histogram with 2048 bins (matches 12-bit range) */
        int hist[2048] = {0};
        for (int i = 0; i < n_abs; i++) {
            int bin = (int)abs_vals[i];
            if (bin >= 2048) bin = 2047;
            hist[bin]++;
        }
        free(abs_vals);
        /* Find 99.9th percentile from histogram */
        int target_count = (int)(n_abs * 0.999f);
        int cumulative = 0;
        for (int b = 0; b < 2048; b++) {
            cumulative += hist[b];
            if (cumulative >= target_count) {
                peak = (float)b;
                break;
            }
        }
        if (peak < 10.0f) peak = 10.0f;  /* safety floor */
        /* Target: 99.9th percentile peak maps to 0.25 (25% DAC FS) */
        float scale = 0.25f / peak;
        for (int i = 0; i < file_n; i++) {
            file_re[i] *= scale;
            file_im[i] *= scale;
        }
        fprintf(stderr, "  p99.9 peak=%.1f, TX scale=%.6f (p99.9→25%% DAC FS)\n",
                peak, scale);

        /* File replay already applies software amplitude scaling to
         * prevent DAC clipping.  Use 0 dB hardware attenuation so
         * marginal captures (low p99.9 peak) aren't pushed below
         * the STF detection floor by extra analog attenuation. */
        hal_ad9361_set_tx_attenuation(0.0);

        /* ARM PIPELINE before DMA load — STF disabled during slow work */
        deimos_pipeline_config_t file_pipe_cfg = DEIMOS_PIPELINE_LIVE_DEFAULT;
        deimos_pipeline_arm(&file_pipe_cfg);

        /* Load into TX DMA */
        if (dma_tx_load(file_re, file_im, (size_t)file_n, false) != 0) {
            fprintf(stderr, "ERROR: dma_tx_load failed\n");
            free(file_re); free(file_im);
            lib80211_fft_plan_destroy(plan);
            deimos_rx_cleanup();
            return 1;
        }
        free(file_re);
        free(file_im);

        /* Enable STF + trigger TX atomically */
        fprintf(stderr, "Transmitting capture through cable...\n");
        if (deimos_stf_gate(DEIMOS_TRIGGER_DMA_TX) != 0) {
            fprintf(stderr, "ERROR: stf_gate + dma_tx_trigger failed\n");
            lib80211_fft_plan_destroy(plan);
            deimos_rx_cleanup();
            return 1;
        }

        /* Drain tags */
        double burst_ms = file_n * 1000.0 / SAMPLE_RATE_HZ;
        int timeout_ms = (int)(burst_ms + 500);
        static rx_result_t rx;
        memset(&rx, 0, sizeof(rx));
        drain_tags(&rx, timeout_ms, 200, verify_psdu);
        dma_tx_stop();

        /* STF lockup detection is done below via a single-frame probe */

        /* Report */
        fprintf(stderr, "\n");
        fprintf(stderr, "============================================================\n");
        fprintf(stderr, "FILE LOOPBACK RESULTS: %s\n", file_path);
        fprintf(stderr, "============================================================\n");
        fprintf(stderr, "  TX samples:      %d (%.1f ms)\n", file_n, burst_ms);
        fprintf(stderr, "  RX tags:         %d\n", rx.rx_count);

        int fcs_pass = 0, fcs_fail = 0;
        for (int i = 0; i < rx.rx_count; i++) {
            if (rx.tags[i].fcs_ok) fcs_pass++;
            else fcs_fail++;
        }
        fprintf(stderr, "  FCS pass:        %d\n", fcs_pass);
        fprintf(stderr, "  FCS fail:        %d\n", fcs_fail);
        fprintf(stderr, "  HW frame_cnt:    %u\n", rx.hw_frame_cnt);
        fprintf(stderr, "  HW drop_cnt:     %u\n", rx.hw_drop_cnt);

        /* Per-tag dump */
        fprintf(stderr, "\nPer-tag:\n");
        for (int i = 0; i < rx.rx_count; i++) {
            const rx_tag_t *t = &rx.tags[i];
            fprintf(stderr, "  [%2d] rate=%s fcs=%s len=%3u bram=0x%04x\n",
                    i, rate_to_str(t->rate_code),
                    t->fcs_ok ? "OK" : "!!",
                    t->length, t->psdu_addr);
        }

        /* STF liveness: TX one more single frame and check detection.
         * This tests whether the STF detector can detect a new frame without
         * an explicit stf_enable toggle. If it fails, the detector is stuck
         * (lockup bug or pipeline stuck state). */
        fprintf(stderr, "\nSTF liveness check (single frame after burst)...\n");
        usleep(100000);
        uint32_t fc_pre = hal_reg_read(REG_TAG_FRAME_CNT);
        {
            /* Generate and TX a single rate-6 frame */
            uint8_t psdu[104];
            memset(psdu, 0xAA, 100);
            lib80211_append_fcs(psdu, 100);
            lib80211_tx_legacy_params p = { .rate_mbps = 6, .psdu = psdu,
                                            .psdu_len = 104, .scrambler_seed = 42 };
            size_t ns = lib80211_tx_legacy_samples(&p);
            float *tre = calloc(ns + 2000, sizeof(float));
            float *tim = calloc(ns + 2000, sizeof(float));
            lib80211_tx_legacy(plan, &p, tre + 1000, tim + 1000);
            dma_tx_load(tre, tim, ns + 2000, false);
            free(tre); free(tim);
            usleep(5000);
            dma_tx_trigger();
            usleep((unsigned int)((ns + 2000) * 1000000ULL / SAMPLE_RATE_HZ) + 5000);
            dma_tx_stop();
        }
        usleep(50000);
        uint32_t fc_post = hal_reg_read(REG_TAG_FRAME_CNT);
        bool liveness_ok = (fc_post > fc_pre);
        fprintf(stderr, "  frame_cnt: %u → %u  %s\n",
                fc_pre, fc_post, liveness_ok ? "OK (STF alive)" : "STUCK! (STF locked up)");

        /* PSDU verification (file mode) — before the JSON line so counts are
         * included in the machine-readable output. */
        int psdu_result = 0;
        int psdu_ok = 0, psdu_fail = 0;
        if (verify_psdu) {
            psdu_result = verify_psdu_file_mode(json_buf, &rx, verbose,
                                                &psdu_ok, &psdu_fail);
            free(json_buf);
            json_buf = NULL;
        }

        /* JSON output */
        printf("{\"file\":\"%s\",\"tx_samples\":%d,\"rx_tags\":%d,"
               "\"fcs_pass\":%d,\"fcs_fail\":%d,"
               "\"hw_frame_cnt\":%u,\"hw_drop_cnt\":%u,"
               "\"stf_alive\":%s",
               file_path, file_n, rx.rx_count,
               fcs_pass, fcs_fail,
               rx.hw_frame_cnt, rx.hw_drop_cnt,
               liveness_ok ? "true" : "false");
        if (verify_psdu)
            printf(",\"psdu_ok\":%d,\"psdu_fail\":%d", psdu_ok, psdu_fail);
        printf("}\n");

        fprintf(stderr, "============================================================\n");
        fprintf(stderr, "  VERDICT: %s\n",
                liveness_ok ? (fcs_pass > 0 ? "PASS" : "NO DECODE (check signal level)")
                            : "FAIL — STF LOCKED UP");
        if (verify_psdu && psdu_result != 0)
            fprintf(stderr, "  PSDU:    FAIL\n");
        fprintf(stderr, "============================================================\n");

        lib80211_fft_plan_destroy(plan);
        deimos_rx_cleanup();
        return (liveness_ok && psdu_result == 0) ? 0 : 1;
    }

    /* Build manifest */
    unsigned int seed = (unsigned int)time(NULL);
    burst_manifest_t manifest;
    build_manifest(&manifest, n_frames, fixed_rate, payload_size,
                   inject_eapol, mix_rates, seed);

    fprintf(stderr, "deimos_burst_loopback: %d frames, ch=%d (%llu MHz)\n",
            n_frames, channel, (unsigned long long)(freq / 1000000));
    fprintf(stderr, "  rates: %s, gap: %d samples (%.0f us)\n",
            fixed_rate > 0 ? "fixed" : (mix_rates ? "mixed 6-36" : "mixed 6/12/24"),
            gap_samples, gap_samples * 1e6 / SAMPLE_RATE_HZ);
    if (inject_eapol) {
        int eapol_count = 0;
        for (int i = 0; i < n_frames; i++)
            if (manifest.frames[i].is_eapol) eapol_count++;
        fprintf(stderr, "  EAPOL frames injected: %d/%d\n", eapol_count, n_frames);
    }
    fprintf(stderr, "  seed: %u\n\n", seed);

    /* Generate burst IQ and load into DMA */
    fprintf(stderr, "Generating burst waveform...\n");

    /* ARM PIPELINE before slow waveform generation (cable mode).
     * STF stays disabled during generation + DMA load — eliminates the
     * noise accumulation race that caused detect_cnt=0 on cable path. */
    deimos_pipeline_config_t pipe_cfg = hil_mode
        ? (deimos_pipeline_config_t)DEIMOS_PIPELINE_HIL_DEFAULT
        : (deimos_pipeline_config_t)DEIMOS_PIPELINE_LIVE_DEFAULT;
    if (!hil_mode) {
        deimos_pipeline_arm(&pipe_cfg);
    }

    size_t total_samples = generate_burst_iq(plan, &manifest, gap_samples, hil_mode,
                                             dump_hil_path);
    if (total_samples == 0) {
        lib80211_fft_plan_destroy(plan);
        deimos_rx_cleanup();
        return 1;
    }
    fprintf(stderr, "  %zu samples (%.1f ms at %llu MSPS)\n",
            total_samples, total_samples * 1000.0 / SAMPLE_RATE_HZ,
            (unsigned long long)(SAMPLE_RATE_HZ / 1000000));

    /* Small delay for FIFO flush to settle */
    usleep(10000);

    /* Trigger */
    fprintf(stderr, "Transmitting...\n");
    uint32_t stf_pre = 0;   /* STF counter snapshot before real pass */
    if (hil_mode) {
        /* HIL: configure playback address + count, then trigger */
        hal_reg_write(REG_HIL_DDR_BASE_REG, HIL_DDR_BASE);
        hal_reg_write(REG_HIL_PLAY_COUNT, (uint32_t)total_samples);
        /* Warm-up pass (initializes Viterbi path memory, etc) */
        hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE | HIL_CTRL_TRIGGER);
        /* Wait for playback to complete + pipeline drain */
        int warmup_us = (int)(total_samples * 50) + 20000;  /* samples/20MHz + 20ms margin */
        usleep(warmup_us);
        hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE);
        usleep(15000);  /* watchdog timeout to fully reset pipeline */

        /* ARM PIPELINE between warm-up and real pass.
         * pipeline_arm does: STF disable (12ms), set all registers, flush tags.
         * This replaces the manual STF toggle + tag flush that was here before. */
        deimos_pipeline_arm(&pipe_cfg);

        /* Debug: verify flush worked */
        {
            uint32_t fc_after_flush = hal_reg_read(REG_TAG_FRAME_CNT);
            uint32_t status_after_flush = hal_reg_read(REG_TAG_STATUS);
            fprintf(stderr, "  [diag] after flush: frame_cnt=%u status=0x%08x (empty=%u count=%u)\n",
                    fc_after_flush, status_after_flush,
                    status_after_flush & 1, (status_after_flush >> 8) & 0xFF);
        }
        /* Snapshot STF detect_cnt BEFORE real pass (for differential) */
        hal_reg_write(REG_DEIMOS_SNAP_MODE, 6);
        hal_reg_write(REG_SNAP_CONTROL, 0);
        usleep(1000);
        hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM);
        usleep(1000);
        hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM | SNAP_CTRL_SW_TRIG);
        usleep(1000);
        hal_reg_write(REG_SNAP_RD_ADDR, 500);
        usleep(1);
        stf_pre = hal_reg_read(REG_SNAP_RD_DATA);
        hal_reg_write(REG_DEIMOS_SNAP_MODE, 0);
        /* Real pass: enable STF + trigger HIL */
        deimos_stf_gate(DEIMOS_TRIGGER_HIL);
    } else {
        /* Cable: enable STF + trigger DMA TX atomically.
         * pipeline_arm() was called before waveform generation, so STF has
         * been disabled the entire time. stf_gate enables detection and
         * fires TX in back-to-back writes — no noise accumulation window. */
        if (deimos_stf_gate(DEIMOS_TRIGGER_DMA_TX) != 0) {
            fprintf(stderr, "ERROR: stf_gate + dma_tx_trigger failed\n");
            lib80211_fft_plan_destroy(plan);
            deimos_rx_cleanup();
            return 1;
        }
    }

    /* Drain tags concurrently with TX.
     * Tag FIFO is only 4-deep, so we must read as frames decode. */
    fprintf(stderr, "Reading decoded tags from FIFO...\n");
    {
        uint32_t status_before_drain = hal_reg_read(REG_TAG_STATUS);
        fprintf(stderr, "  [diag] before drain: status=0x%08x (empty=%u count=%u)\n",
                status_before_drain, status_before_drain & 1, (status_before_drain >> 8) & 0xFF);
    }
    double burst_duration_ms = total_samples * 1000.0 / SAMPLE_RATE_HZ;
    int total_timeout_ms = (int)(burst_duration_ms + 500);
    static rx_result_t rx;
    memset(&rx, 0, sizeof(rx));
    drain_tags(&rx, total_timeout_ms, 200, verify_psdu);

    if (hil_mode) {
        /* Clear trigger, stay in test mode */
        hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE);
    } else {
        dma_tx_stop();
    }

    /* --------------------------------------------------------------------------
     * Pipeline diagnostic counters (tag_fifo_axi registers — session 9)
     * Cleared on FIFO flush, so these are real-pass only values.
     * -------------------------------------------------------------------------- */
    uint32_t d_accepted = hal_reg_read(REG_TAG_ACCEPTED_CNT);
    uint32_t d_abort    = hal_reg_read(REG_TAG_ABORT_CNT);
    fprintf(stderr, "\n--- Pipeline diagnostics (tag_fifo_axi) ---\n");
    fprintf(stderr, "  trigger_accepted=%u  abort=%u  frame_cnt=%u  drop=%u\n",
            d_accepted, d_abort, rx.hw_frame_cnt, rx.hw_drop_cnt);
    fprintf(stderr, "  (accepted should = frame_cnt + abort)\n");
    fprintf(stderr, "---\n");

    /* STF detect_cnt via snap mode 6 (absolute, not cleared on flush) */
    uint32_t diag_stf_persist = 0;
    {
        hal_reg_write(REG_DEIMOS_SNAP_MODE, 6);
        hal_reg_write(REG_SNAP_CONTROL, 0);
        usleep(1000);
        hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM);
        usleep(1000);
        hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM | SNAP_CTRL_SW_TRIG);
        usleep(1000);
        hal_reg_write(REG_SNAP_RD_ADDR, 500);
        usleep(1);
        diag_stf_persist = hal_reg_read(REG_SNAP_RD_DATA);
        hal_reg_write(REG_DEIMOS_SNAP_MODE, 0);
    }
    uint8_t stf_detect_cnt   = (diag_stf_persist & 0xFF) - (stf_pre & 0xFF);
    uint16_t stf_thresh_cnt  = ((diag_stf_persist >> 8) & 0xFFFF) - ((stf_pre >> 8) & 0xFFFF);
    uint8_t stf_max_persist  = (diag_stf_persist >> 24) & 0xFF;
    fprintf(stderr, "--- STF diagnostics (snap mode 6, delta) ---\n");
    fprintf(stderr, "  detect_cnt=%u  max_persist=%u  thresh_met_cnt=%u\n",
            stf_detect_cnt, stf_max_persist, stf_thresh_cnt);
    fprintf(stderr, "---\n");

    /* --------------------------------------------------------------------------
     * Snap diagnostic: capture STF rearm state during burst.
     * Mode 5 = stf_detect diag_out: {threshold_met, detected_latch, window_full,
     *   persist_cnt[4:0], acc_e1[31:20], acc_e2[31:20]}
     *
     * Strategy: arm circular + ext_trig. Re-play burst. ext_trig fires on
     * frame_detect during non-IDLE (pending trigger scenario). If ext_trig
     * doesn't fire (pending path never entered), fall back to SW-trigger
     * timed to capture during frame 2's STF region.
     * -------------------------------------------------------------------------- */
    {
        /* Set snap mode 5 and arm with circular capture */
        hal_reg_write(REG_DEIMOS_SNAP_MODE, 5);
        hal_reg_write(REG_SNAP_CONTROL, 0);  /* reset */
        usleep(100);
        hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM | SNAP_CTRL_CIRCULAR);
        usleep(100);

        /* Re-trigger the burst for snap capture */
        if (hil_mode) {
            hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE | HIL_CTRL_TRIGGER);
            /* SW-trigger timed to frame 2's STF region:
             * Frame 1 = ~3200 samples (rate 6 golden vector)
             * Gap = 320 samples
             * Frame 2 STF starts at sample ~3520 from burst start
             * At 20 MSPS = 176 us from trigger.
             * Add 50 us for STF buildup = 226 us.
             * Add leading zeros (varies) — use 300 us for safety. */
            {
                /* Compute frame 2 STF timing for mode 5 snap */
                lib80211_tx_legacy_params f1p5 = {
                    .rate_mbps = manifest.frames[0].rate_mbps,
                    .psdu_len = manifest.frames[0].psdu_len,
                };
                size_t f1s = lib80211_tx_legacy_samples(&f1p5);
                usleep((useconds_t)((f1s + gap_samples) / 20 + 5));
            }
            /* SW trigger to freeze buffer */
            hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM | SNAP_CTRL_SW_TRIG);
            /* Wait for burst to finish */
            int cap_us = (int)(total_samples * 50) + 5000;
            usleep(cap_us);
            hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE);
        } else {
            usleep(10000);
            hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM | SNAP_CTRL_CIRCULAR | SNAP_CTRL_SW_TRIG);
        }
        usleep(100);

        /* Read snap status and decode stf_detect state */
        uint32_t snap_st = hal_reg_read(REG_SNAP_STATUS);
        uint32_t trig_pos = (snap_st >> 16) & 0x3FF;
        fprintf(stderr, "\n--- STF rearm snap (mode 5): status=0x%08x trig_pos=%u ---\n",
                snap_st, trig_pos);
        if (snap_st & 1) {
            /* Print samples around trigger: look for persist_cnt progression */
            fprintf(stderr, "  Format: thr=threshold_met, lat=detected_latch, wf=window_full, per=persist_cnt, e1/e2=energy\n");
            int transitions = 0;
            uint32_t prev_per = 99;
            for (int i = -512; i < 512; i++) {
                int addr = ((int)trig_pos + i) & 0x3FF;
                hal_reg_write(REG_SNAP_RD_ADDR, (uint32_t)addr);
                usleep(1);
                uint32_t w = hal_reg_read(REG_SNAP_RD_DATA);
                uint32_t thr = (w >> 31) & 1;
                uint32_t lat = (w >> 30) & 1;
                uint32_t wf  = (w >> 29) & 1;
                uint32_t per = (w >> 24) & 0x1F;
                uint32_t e1  = (w >> 12) & 0xFFF;
                uint32_t e2  = w & 0xFFF;
                /* Only print transitions and interesting state */
                if (per != prev_per || thr || lat || (i >= -5 && i <= 5)) {
                    fprintf(stderr, "  [%+4d] thr=%u lat=%u wf=%u per=%2u e1=0x%03x e2=0x%03x%s\n",
                            i, thr, lat, wf, per, e1, e2,
                            (per > 0 && per < prev_per) ? " <-- RESET" : "");
                    transitions++;
                }
                prev_per = per;
            }
            fprintf(stderr, "  (%d transitions shown)\n---\n\n", transitions);
        } else {
            fprintf(stderr, "  (no capture)\n---\n\n");
        }
        /* Restore snap mode 0 */
        hal_reg_write(REG_DEIMOS_SNAP_MODE, 0);
    }

    /* --------------------------------------------------------------------------
     * Snap diagnostic 2: raw IQ capture during frame 2's STF region.
     * Mode 2 = {iq_valid_in, 7'b0, iq_re_in[11:0], iq_im_in[11:0]}
     *
     * Purpose: Verify IQ data actually reaches stf_detect for frames 2+.
     * The SIFS burst drop shows persist_cnt=0 for frames 2-10, but we don't
     * know if the correlator receives valid IQ or if the DMA/FIFO stops
     * delivering data. This capture proves which.
     *
     * Timing: Frame 2 STF starts at (frame_duration + gap) samples from burst
     * start. At 20 MSPS with 1-in-5 fabric clock, each sample = 50 ns.
     * We use multiple usleep delays to sweep for the STF region.
     * -------------------------------------------------------------------------- */
    if (hil_mode && n_frames >= 2) {
        /* Compute frame 2 STF timing */
        lib80211_tx_legacy_params f1p = {
            .rate_mbps = manifest.frames[0].rate_mbps,
            .psdu_len = manifest.frames[0].psdu_len,
        };
        size_t frame1_samples = lib80211_tx_legacy_samples(&f1p);
        size_t frame2_stf_start_us = (frame1_samples + gap_samples) / 20;  /* μs */
        size_t trigger_us = frame2_stf_start_us + 5;  /* 5 μs into STF */

        fprintf(stderr, "\n--- Raw IQ snap (mode 2): targeting frame 2 STF @ ~%zu μs ---\n",
                frame2_stf_start_us);

        /* Set snap mode 2 and arm circular */
        hal_reg_write(REG_DEIMOS_SNAP_MODE, 2);
        hal_reg_write(REG_SNAP_CONTROL, 0);
        usleep(100);
        hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM | SNAP_CTRL_CIRCULAR);
        usleep(100);

        /* Re-trigger burst */
        hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE | HIL_CTRL_TRIGGER);

        /* Wait for frame 2 STF region.
         * usleep is imprecise (~10-100 μs jitter on Zynq), so we target
         * the middle of the STF (8 μs long) for best chance. */
        usleep((useconds_t)trigger_us);

        /* SW-trigger to freeze circular buffer */
        hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM | SNAP_CTRL_SW_TRIG);

        /* Wait for burst to finish */
        int cap2_us = (int)(total_samples * 50) + 5000;
        usleep(cap2_us);
        hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE);
        usleep(100);

        /* Read and analyze IQ snap data */
        uint32_t snap_st2 = hal_reg_read(REG_SNAP_STATUS);
        uint32_t trig_pos2 = (snap_st2 >> 16) & 0x3FF;
        fprintf(stderr, "  status=0x%08x trig_pos=%u\n", snap_st2, trig_pos2);

        if (snap_st2 & 1) {
            int valid_count = 0;
            int64_t energy_sum = 0;
            int max_re = 0, max_im = 0;
            int stf_like_count = 0;  /* samples with energy > 500,000 (STF level) */
            int noise_like_count = 0;  /* samples with energy < 5,000 (noise level) */

            for (int i = 0; i < 1024; i++) {
                int addr = ((int)trig_pos2 + i - 1023) & 0x3FF;
                hal_reg_write(REG_SNAP_RD_ADDR, (uint32_t)addr);
                usleep(1);
                uint32_t w = hal_reg_read(REG_SNAP_RD_DATA);

                int iq_valid = (w >> 31) & 1;
                if (!iq_valid) continue;

                valid_count++;
                /* Extract signed 12-bit IQ */
                int re_raw = (w >> 12) & 0xFFF;
                int im_raw = w & 0xFFF;
                int re = (re_raw & 0x800) ? (re_raw | ~0xFFF) : re_raw;
                int im = (im_raw & 0x800) ? (im_raw | ~0xFFF) : im_raw;

                int64_t e = (int64_t)re * re + (int64_t)im * im;
                energy_sum += e;

                if (abs(re) > max_re) max_re = abs(re);
                if (abs(im) > max_im) max_im = abs(im);

                if (e > 500000) stf_like_count++;
                else if (e < 5000) noise_like_count++;
            }

            int64_t avg_energy = valid_count > 0 ? energy_sum / valid_count : 0;
            fprintf(stderr, "  valid_samples=%d avg_energy=%lld max_re=%d max_im=%d\n",
                    valid_count, (long long)avg_energy, max_re, max_im);
            fprintf(stderr, "  stf_level(>500K)=%d noise_level(<5K)=%d\n",
                    stf_like_count, noise_like_count);

            if (valid_count == 0) {
                fprintf(stderr, "  !! NO IQ VALID PULSES — DMA/FIFO not delivering data !!\n");
            } else if (stf_like_count > valid_count / 2) {
                fprintf(stderr, "  IQ FLOWING: STF-level energy present. Correlator should detect.\n");
            } else if (noise_like_count > valid_count / 2) {
                fprintf(stderr, "  !! IQ NOISE ONLY — data not reaching stf_detect !!\n");
                fprintf(stderr, "  (expected STF amplitude ~1000-1600 LSBs, seeing ~20 LSBs)\n");
            } else {
                fprintf(stderr, "  MIXED: possibly in transition region (DATA tail or gap→STF).\n");
                fprintf(stderr, "  Retry with adjusted timing (+/- 50 μs).\n");
            }

            /* Print first 20 valid samples for manual inspection */
            fprintf(stderr, "  First IQ samples: ");
            int printed = 0;
            for (int i = 0; i < 1024 && printed < 20; i++) {
                int addr = ((int)trig_pos2 + i - 1023) & 0x3FF;
                hal_reg_write(REG_SNAP_RD_ADDR, (uint32_t)addr);
                usleep(1);
                uint32_t w = hal_reg_read(REG_SNAP_RD_DATA);
                if (!((w >> 31) & 1)) continue;
                int re_raw = (w >> 12) & 0xFFF;
                int im_raw = w & 0xFFF;
                int re = (re_raw & 0x800) ? (re_raw | ~0xFFF) : re_raw;
                int im = (im_raw & 0x800) ? (im_raw | ~0xFFF) : im_raw;
                fprintf(stderr, "(%d,%d) ", re, im);
                printed++;
            }
            fprintf(stderr, "\n");
        } else {
            fprintf(stderr, "  (no capture — snap not triggered)\n");
        }

        /* Also check play_ptr: did all samples get output? */
        uint32_t play_count = hal_reg_read(REG_HIL_BASE + 0x10);  /* play_ptr register */
        fprintf(stderr, "  play_ptr=%u (expected %zu) %s\n",
                play_count, total_samples,
                play_count == (uint32_t)total_samples ? "OK" : "!! MISMATCH !!");

        hal_reg_write(REG_DEIMOS_SNAP_MODE, 0);
        fprintf(stderr, "---\n\n");
    }

    /* Verify */
    verify_result_t v = verify_burst(&rx, &manifest);

    /* --------------------------------------------------------------------------
     * PSDU verification (--verify-psdu)
     * Compare BRAM-read PSDU bytes against regenerated expected frames.
     * -------------------------------------------------------------------------- */
    int psdu_ok = 0, psdu_fail = 0;

    if (verify_psdu && !file_path) {
        /* Manifest mode: regenerate expected frame and compare */
        bool manifest_matched_psdu[MAX_BURST_FRAMES];
        memset(manifest_matched_psdu, 0, sizeof(manifest_matched_psdu));

        for (int i = 0; i < rx.rx_count; i++) {
            const rx_tag_t *t = &rx.tags[i];
            if (!t->fcs_ok || rx.psdu_len[i] == 0) continue;

            /* Find matching manifest entry by (rate, length) */
            int mf_idx = -1;
            uint8_t expected_rate;
            for (int j = 0; j < manifest.n_frames; j++) {
                if (manifest_matched_psdu[j]) continue;
                expected_rate = RATE_CODES[manifest.frames[j].rate_mbps];
                if (t->rate_code == expected_rate &&
                    t->length == (uint16_t)manifest.frames[j].psdu_len) {
                    mf_idx = j;
                    manifest_matched_psdu[j] = true;
                    break;
                }
            }
            if (mf_idx < 0) continue;

            /* Regenerate expected frame */
            uint8_t expected[MAX_PSDU_BYTES + 4];
            size_t expected_len = build_frame(expected,
                manifest.frames[mf_idx].payload_bytes,
                manifest.frames[mf_idx].seq_num,
                manifest.frames[mf_idx].is_eapol);

            /* Compare (FCS not in BRAM) */
            uint16_t cmp_len = (expected_len > 4) ? (uint16_t)(expected_len - 4) : 0;
            if (cmp_len != rx.psdu_len[i]) {
                psdu_fail++;
                if (verbose)
                    fprintf(stderr, "  TAG[%2d]: PSDU len mismatch: got %u, expect %u\n",
                            i, rx.psdu_len[i], cmp_len);
                continue;
            }

            bool match = true;
            for (uint16_t b = 0; b < cmp_len; b++) {
                if (rx.psdu_buf[i][b] != expected[b]) {
                    match = false;
                    if (verbose)
                        fprintf(stderr, "  TAG[%2d]: PSDU byte %u mismatch: got 0x%02x, expect 0x%02x\n",
                                i, b, rx.psdu_buf[i][b], expected[b]);
                    break;
                }
            }
            if (match) psdu_ok++;
            else psdu_fail++;
        }

        fprintf(stderr, "  PSDU verify: %d/%d bytes match\n",
                psdu_ok, psdu_ok + psdu_fail);
    }

    /* Report */
    fprintf(stderr, "\n");
    fprintf(stderr, "============================================================\n");
    fprintf(stderr, "BURST LOOPBACK RESULTS%s\n",
            verify_psdu ? " (tag +PSDU verify)" : " (tag-only)");
    fprintf(stderr, "============================================================\n");
    fprintf(stderr, "  TX frames:       %d\n", n_frames);
    fprintf(stderr, "  RX tags:         %d\n", rx.rx_count);
    fprintf(stderr, "  FCS pass:        %d\n", v.fcs_pass);
    fprintf(stderr, "  FCS fail:        %d\n", v.fcs_fail);
    fprintf(stderr, "  Matched:         %d / %d\n", v.matched, n_frames);
    fprintf(stderr, "  Unmatched:       %d (FCS-ok but no manifest match)\n", v.unmatched);
    if (inject_eapol) {
        fprintf(stderr, "  EAPOL sent:      %d\n", v.eapol_sent);
        fprintf(stderr, "  EAPOL matched:   %d\n", v.eapol_matched);
    }
    fprintf(stderr, "  HW tag drops:    %u\n", v.hw_tag_drops);
    if (verify_psdu) {
        fprintf(stderr, "  PSDU verify:     %d/%d\n",
                psdu_ok, psdu_ok + psdu_fail);
    }
    fprintf(stderr, "------------------------------------------------------------\n");

    int match_pct = n_frames > 0 ? (v.matched * 100 / n_frames) : 0;
    fprintf(stderr, "  MATCH RATE:      %d%% (%d/%d)\n", match_pct, v.matched, n_frames);

    /* Pass condition: no spurious matches, at least some frames decoded.
     * When --verify-psdu: also require all PSDU comparisons to succeed. */
    bool pass = (v.unmatched == 0 && v.matched > 0);
    if (verify_psdu && psdu_fail > 0)
        pass = false;

    if (!pass) {
        fprintf(stderr, "  VERDICT:         FAIL");
        if (verify_psdu && psdu_fail > 0)
            fprintf(stderr, " (PSDU: %d failures)", psdu_fail);
        fprintf(stderr, "\n");
    } else if (v.matched == n_frames) {
        fprintf(stderr, "  VERDICT:         PASS (100%%)\n");
    } else {
        fprintf(stderr, "  VERDICT:         PARTIAL (%d dropped, likely rearm gap)\n",
                n_frames - v.matched);
    }
    fprintf(stderr, "============================================================\n");

    /* Verbose per-tag dump */
    if (verbose) {
        fprintf(stderr, "\nPer-tag details:\n");
        for (int i = 0; i < rx.rx_count; i++) {
            const rx_tag_t *t = &rx.tags[i];
            fprintf(stderr, "  TAG[%2d]: rate=%s fcs=%s len=%u bram=0x%04x\n",
                    i, rate_to_str(t->rate_code),
                    t->fcs_ok ? "OK" : "!!",
                    t->length, t->psdu_addr);
        }
    }

    /* JSON output to stdout for scripting */
    printf("{\"tx_frames\":%d,\"rx_tags\":%d,\"fcs_pass\":%d,\"fcs_fail\":%d,"
           "\"matched\":%d,\"unmatched\":%d,"
           "\"hw_tag_drops\":%u",
           n_frames, rx.rx_count, v.fcs_pass, v.fcs_fail,
           v.matched, v.unmatched,
           v.hw_tag_drops);
    if (verify_psdu)
        printf(",\"psdu_ok\":%d,\"psdu_fail\":%d", psdu_ok, psdu_fail);
    if (inject_eapol)
        printf(",\"eapol_sent\":%d,\"eapol_matched\":%d",
               v.eapol_sent, v.eapol_matched);
    printf(",\"gap_samples\":%d,\"seed\":%u,"
           "\"diag_trig_accepted\":%u,\"diag_abort\":%u,"
           "\"diag_stf_detect_cnt\":%u,"
           "\"pass\":%s}\n",
           gap_samples, seed,
           d_accepted, d_abort,
           stf_detect_cnt,
           pass ? "true" : "false");

    lib80211_fft_plan_destroy(plan);
    deimos_rx_cleanup();
    return pass ? 0 : 1;
}
