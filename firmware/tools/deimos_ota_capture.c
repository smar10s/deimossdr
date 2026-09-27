// SPDX-License-Identifier: MIT
/*
 * deimos_ota_capture — ONE-OFF DIAGNOSTIC: capture one long raw-DDR IQ window
 * around an OTA EAPOL handshake, triggered by the M1 tag.
 *
 * DELETE WHEN THE M2-LOSS INVESTIGATION CLOSES. Not part of `make deploy`.
 *
 * Why this exists / why it replaces deimos_ddr_capture (611ca31):
 *   The old tool located each tagged frame by a 25-bit DDR sample offset in
 *   TAG_HI and dumped a small region around it. The current tag carries
 *   psdu_addr[13:0] (a BRAM address), so per-tag DDR ground truth is no longer
 *   possible. Instead we enable the iq_dma_rx ring, poll the fabric tag FIFO
 *   for an M1, snapshot WR_PTR, keep the DMA running for one long window, then
 *   dump that window plus every tag the fabric produced inside it.
 *
 * Goal: get one window in which the fabric tagged M1 but did NOT tag the STA
 * response (M2). Replay it offline (sim + lib80211) to decide whether the
 * defect is in the fabric or upstream of it (AGC/analog).
 *
 * Output (binary is ground truth; sidecar carries geometry + tag log):
 *   <prefix>.bin    raw packed DDR words, window_samples x uint32
 *                   ({8'b0, imag[11:0], real[11:0]} — see hal.h IQ_PACK)
 *   <prefix>.json   sidecar: geometry + every fabric tag seen in the window
 *
 * Usage:
 *   deimos_ota_capture -c 36 -w 30 -n 20 -o /tmp/ddr_win
 *   deimos_ota_capture --trigger-len 137 --absent-len 159
 *
 * Exit: 0 = captured a window with trigger present and absent-frame missing;
 *       1 = error; 2 = ran out of attempts without finding one (last window
 *       is still written for inspection).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <getopt.h>

#include "deimos_rx.h"
#include "deimos_tool.h"

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ      20000000ULL
#define SAMPLES_PER_MS      (SAMPLE_RATE_HZ / 1000ULL)          /* 20000 */
#define DDR_RX_BUF_SAMPLES  ((uint32_t)(DDR_RX_SIZE / 4))       /* 128MB/4 */
#define MAX_WINDOW_SAMPLES  (4u * 1024u * 1024u)                /* 16 MB cap */
#define MAX_TAGS            256

/* 6 Mbps SIGNAL rate code — M1/M2 are legacy 6 Mbps in our captures. */
#define RATE_CODE_6M        0x0B

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

typedef struct {
    int      rate_mbps;
    uint32_t length;
    bool     fcs_ok;
    uint32_t offset;   /* sample offset within the window */
} tag_rec_t;

static double elapsed_sec(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double s = (double)(now.tv_sec - start->tv_sec);
    s += (double)(now.tv_nsec - start->tv_nsec) / 1e9;
    return s;
}

static uint32_t now_wr(void)
{
    return hal_reg_read(REG_IQ_DMA_RX_WR_PTR) & (DDR_RX_BUF_SAMPLES - 1u);
}

static void ring_read(volatile uint32_t *ddr, uint32_t start,
                      uint32_t count, uint32_t *out)
{
    uint32_t pos = start;
    for (uint32_t i = 0; i < count; i++) {
        out[i] = ddr[pos];
        pos++;
        if (pos >= DDR_RX_BUF_SAMPLES)
            pos = 0;
    }
}

/* Rate code → NDBPS (data bits per OFDM symbol) */
static int rate_ndbps(uint32_t rate_code)
{
    switch (rate_code) {
    case 0x0B: return 24;   /* 6 Mbps */
    case 0x0F: return 36;   /* 9 Mbps */
    case 0x0A: return 48;   /* 12 Mbps */
    case 0x0E: return 72;   /* 18 Mbps */
    case 0x09: return 96;   /* 24 Mbps */
    case 0x0D: return 144;  /* 36 Mbps */
    case 0x08: return 192;  /* 48 Mbps */
    case 0x0C: return 216;  /* 54 Mbps */
    default:   return 24;
    }
}

/* Estimate total frame duration in samples (STF+LTF+SIG+DATA). */
static uint32_t estimate_frame_samples(uint32_t rate_code, uint32_t length)
{
    int ndbps = rate_ndbps(rate_code);
    int n_symbols = (16 + 8 * (int)length + 6 + ndbps - 1) / ndbps;
    return (uint32_t)(160 + 160 + 80 + n_symbols * 80);
}

/* Direction of a decoded FCS-OK frame from its 802.11 MAC frame-control word.
 * Returns 1 for AP->STA (FromDS), 0 for STA->AP (ToDS), -1 if indeterminate.
 *
 * Why this matters: the trigger is matched on (rate, length) alone, but M1 is
 * 137 bytes AP->STA and M4 is 137 bytes STA->AP. Without a direction check the
 * tool selects an M4-triggered window, in which "159 absent" is trivially true
 * (M2 precedes M4), producing a false failing window. Require the AP->STA
 * direction for an M1 trigger. */
static int frame_direction(const deimos_rx_frame_t *f)
{
    if (!f->fcs_ok || f->psdu_len < 2)
        return -1;
    uint16_t fc = (uint16_t)(f->psdu[0] | (f->psdu[1] << 8));
    int tods   = (fc >> 8) & 1;
    int fromds = (fc >> 9) & 1;
    if (fromds && !tods) return 1;   /* AP -> STA */
    if (tods && !fromds) return 0;   /* STA -> AP */
    return -1;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "  Capture one long raw-DDR IQ window around an M1-triggered OTA\n"
        "  handshake, plus the fabric tag log for that window.\n"
        "\n"
        "Options:\n"
        "  -c channel       WiFi channel (default: 36)\n"
        "  -w window_ms     Window length in ms (default: 30)\n"
        "  -n max_windows   Max M1-triggered windows to try (default: 20)\n"
        "  -p pre_samples   Pre-roll before M1 start (default: 4000)\n"
        "  -d duration_s    Overall timeout, 0=infinite (default: 120)\n"
        "  -o prefix        Output prefix (default: /tmp/ddr_win)\n"
        "      --trigger-len L  Trigger frame length (default: 137 = M1)\n"
        "      --trigger-dir D  Trigger direction: ap|sta|any (default: ap = M1;\n"
        "                       required to avoid selecting an M4-triggered window)\n"
        "      --absent-len  L  Frame length that must be absent (default: 159 = M2)\n"
        "  -a               Write every window, not just the selected/last\n"
        "  -v               Verbose to stderr\n"
        "  -h               Show this help\n",
        prog);
}

/* Write <base>.bin and <base>.json. Returns 0 on success. */
static int write_capture(const char *base, uint32_t window_start,
                         uint32_t window_samples, int channel,
                         uint32_t pre_samples, int trigger_len, int absent_len,
                         const uint32_t *buf, const tag_rec_t *tags, int ntags,
                         bool trigger_present, bool absent_present,
                         bool selected, int attempt, bool verbose)
{
    char path[1024];
    int n;
    FILE *f;

    n = snprintf(path, sizeof(path), "%s.bin", base);
    if (n < 0 || n >= (int)sizeof(path)) {
        fprintf(stderr, "ERROR: output path too long\n");
        return -1;
    }
    f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "ERROR: cannot open %s for writing\n", path);
        return -1;
    }
    if (fwrite(buf, sizeof(uint32_t), window_samples, f) != window_samples) {
        fprintf(stderr, "ERROR: short write to %s\n", path);
        fclose(f);
        return -1;
    }
    fclose(f);

    n = snprintf(path, sizeof(path), "%s.json", base);
    if (n < 0 || n >= (int)sizeof(path)) {
        fprintf(stderr, "ERROR: output path too long\n");
        return -1;
    }
    f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "ERROR: cannot open %s for writing\n", path);
        return -1;
    }
    fprintf(f,
        "{\"tool\":\"deimos_ota_capture\",\"version\":1,"
        "\"channel\":%d,\"sample_rate_hz\":%llu,"
        "\"window_samples\":%u,\"window_ms\":%.3f,\"window_start\":%u,"
        "\"pre_samples\":%u,\"trigger_len\":%d,\"absent_len\":%d,"
        "\"trigger_present\":%s,\"absent_present\":%s,"
        "\"selected\":%s,\"attempt\":%d,\"tag_count\":%d,\"tags\":[",
        channel, (unsigned long long)SAMPLE_RATE_HZ,
        window_samples, (double)window_samples / (double)SAMPLES_PER_MS,
        window_start, pre_samples, trigger_len, absent_len,
        trigger_present ? "true" : "false",
        absent_present ? "true" : "false",
        selected ? "true" : "false", attempt, ntags);
    for (int i = 0; i < ntags; i++) {
        fprintf(f, "%s{\"rate\":%d,\"len\":%u,\"fcs\":%s,\"offset\":%u}",
                i ? "," : "", tags[i].rate_mbps, tags[i].length,
                tags[i].fcs_ok ? "true" : "false", tags[i].offset);
    }
    fprintf(f, "]}\n");
    fclose(f);

    if (verbose) {
        fprintf(stderr, "  wrote %s.bin (%u samples, %.2f MB) + .json\n",
                base, window_samples,
                (double)window_samples * 4.0 / (1024.0 * 1024.0));
    }
    return 0;
}

int main(int argc, char *argv[])
{
    int channel       = 36;
    int window_ms     = 30;
    int max_windows   = 20;
    uint32_t pre      = 4000;
    int duration_s    = 120;
    const char *prefix = "/tmp/ddr_win";
    int trigger_len   = 137;
    int absent_len    = 159;
    int trigger_dir   = 1;      /* 1 = ap->sta (M1), 0 = sta->ap, -1 = any */
    bool write_all    = false;
    bool verbose      = false;

    enum { OPT_TRIGGER_LEN = 256, OPT_ABSENT_LEN, OPT_TRIGGER_DIR };
    static const struct option long_opts[] = {
        { "trigger-len", required_argument, NULL, OPT_TRIGGER_LEN },
        { "trigger-dir", required_argument, NULL, OPT_TRIGGER_DIR },
        { "absent-len",  required_argument, NULL, OPT_ABSENT_LEN  },
        { "help",        no_argument,       NULL, 'h'             },
        { NULL, 0, NULL, 0 },
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "c:w:n:p:d:o:avh",
                              long_opts, NULL)) != -1) {
        switch (opt) {
        case 'c': channel     = atoi(optarg); break;
        case 'w': window_ms   = atoi(optarg); break;
        case 'n': max_windows = atoi(optarg); break;
        case 'p': pre         = (uint32_t)strtoul(optarg, NULL, 0); break;
        case 'd': duration_s  = atoi(optarg); break;
        case 'o': prefix      = optarg; break;
        case 'a': write_all   = true; break;
        case 'v': verbose     = true; break;
        case OPT_TRIGGER_LEN: trigger_len = atoi(optarg); break;
        case OPT_TRIGGER_DIR:
            if (!strcmp(optarg, "ap"))       trigger_dir = 1;
            else if (!strcmp(optarg, "sta")) trigger_dir = 0;
            else if (!strcmp(optarg, "any")) trigger_dir = -1;
            else { fprintf(stderr, "ERROR: --trigger-dir must be ap|sta|any\n");
                   return 1; }
            break;
        case OPT_ABSENT_LEN:  absent_len  = atoi(optarg); break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (window_ms <= 0 || max_windows <= 0 || trigger_len <= 0 ||
        absent_len <= 0) {
        fprintf(stderr, "ERROR: window_ms, max_windows and lengths must be > 0\n");
        return 1;
    }

    uint32_t window_samples = (uint32_t)((uint64_t)window_ms *
                                         (uint64_t)SAMPLES_PER_MS);
    if (window_samples == 0 || window_samples > MAX_WINDOW_SAMPLES) {
        fprintf(stderr, "ERROR: window %u samples exceeds cap %u\n",
                window_samples, MAX_WINDOW_SAMPLES);
        return 1;
    }

    styx_install_shutdown_handler();

    deimos_radio_config_t radio = {
        .channel          = channel,
        .agc_mode         = DEIMOS_AGC_FAST_ATTACK,
        .configure_tx     = false,
        .bandwidth_mhz    = 0,
        .skip_calibration = false,
    };
    if (deimos_radio_init(&radio) != 0) {
        fprintf(stderr, "deimos_ota_capture: radio init failed\n");
        return 1;
    }

    deimos_pipeline_config_t pipeline = {
        .mode          = DEIMOS_RX_MODE_LIVE,
        .stf_threshold = 0,
        .stf_skip      = 4,
        .ltf_skip      = 1,
        .cfo_thresh    = 64,
    };
    deimos_pipeline_arm(&pipeline);

    /* Enable the raw ADC → DDR ring (separate from the fabric decode path). */
    hal_reg_write(REG_IQ_DMA_RX_DDR_BASE, DDR_RX_BASE);
    hal_reg_write(REG_IQ_DMA_RX_CONTROL, RX_CTRL_ENABLE);
    usleep(10000);

    if (deimos_stf_gate(DEIMOS_TRIGGER_NONE) != 0) {
        fprintf(stderr, "deimos_ota_capture: STF gate failed\n");
        styx_disarm_cleanup();
        deimos_rx_cleanup();
        return 1;
    }

    volatile uint32_t *ddr = hal_ddr_rx_buf();
    uint32_t *winbuf = malloc((size_t)window_samples * sizeof(uint32_t));
    tag_rec_t *tags  = malloc(MAX_TAGS * sizeof(tag_rec_t));
    if (!winbuf || !tags) {
        fprintf(stderr, "ERROR: malloc failed\n");
        free(winbuf); free(tags);
        hal_reg_write(REG_IQ_DMA_RX_CONTROL, 0);
        styx_disarm_cleanup();
        deimos_rx_cleanup();
        return 1;
    }

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    fprintf(stderr, "deimos_ota_capture: ch%d, fast_attack, window %ums, "
            "trigger len=%d dir=%s, absent len=%d\n",
            channel, window_ms, trigger_len,
            trigger_dir == 1 ? "ap->sta" : trigger_dir == 0 ? "sta->ap" : "any",
            absent_len);
    fprintf(stderr, "Waiting for M1 (rate 6, len %d, %s, FCS OK)... "
            "toggle Mac wifi to trigger a handshake.\n", trigger_len,
            trigger_dir == 1 ? "AP->STA" : trigger_dir == 0 ? "STA->AP" : "any dir");

    bool selected = false;
    int attempt = 0;

    for (attempt = 1; attempt <= max_windows && !selected; attempt++) {
        if (styx_shutdown_requested())
            break;
        if (duration_s > 0 && elapsed_sec(&t0) >= (double)duration_s) {
            fprintf(stderr, "deimos_ota_capture: overall timeout reached\n");
            break;
        }

        /* --- Wait for the trigger tag --- */
        bool got_trigger = false;
        uint32_t trig_wr = 0;
        while (!styx_shutdown_requested()) {
            if (duration_s > 0 && elapsed_sec(&t0) >= (double)duration_s)
                break;

            deimos_rx_frame_t frame;
            int ret = deimos_rx_poll(&frame);
            if (ret == 1) {
                int dir = frame_direction(&frame);
                if (frame.fcs_ok && frame.rate_mbps == 6 &&
                    frame.length == (uint16_t)trigger_len &&
                    (trigger_dir < 0 || dir == trigger_dir)) {
                    trig_wr = now_wr();
                    got_trigger = true;
                    break;
                }
            } else if (ret == -1) {
                break;
            } else {
                usleep(200);
            }
        }
        if (!got_trigger) {
            if (styx_shutdown_requested() ||
                (duration_s > 0 && elapsed_sec(&t0) >= (double)duration_s))
                break;
            continue;
        }

        uint32_t frame_dur = estimate_frame_samples(RATE_CODE_6M,
                                                    (uint32_t)trigger_len);
        uint32_t window_start =
            (trig_wr + DDR_RX_BUF_SAMPLES - frame_dur - pre) %
            DDR_RX_BUF_SAMPLES;

        int ntags = 0;
        tags[ntags].rate_mbps = 6;
        tags[ntags].length    = (uint32_t)trigger_len;
        tags[ntags].fcs_ok    = true;
        tags[ntags].offset =
            (trig_wr + DDR_RX_BUF_SAMPLES - window_start) % DDR_RX_BUF_SAMPLES;
        ntags++;

        if (verbose)
            fprintf(stderr, "  [attempt %d] M1 @ wr=%u, window_start=%u\n",
                    attempt, trig_wr, window_start);

        /* --- Fill the window, logging every tag the fabric emits --- */
        while (!styx_shutdown_requested()) {
            uint32_t wr = now_wr();
            uint32_t written =
                (wr + DDR_RX_BUF_SAMPLES - window_start) % DDR_RX_BUF_SAMPLES;
            if (written >= window_samples)
                break;

            deimos_rx_frame_t frame;
            int ret = deimos_rx_poll(&frame);
            if (ret == 1) {
                if (ntags < MAX_TAGS) {
                    tags[ntags].rate_mbps = frame.rate_mbps;
                    tags[ntags].length    = frame.length;
                    tags[ntags].fcs_ok    = frame.fcs_ok;
                    tags[ntags].offset =
                        (now_wr() + DDR_RX_BUF_SAMPLES - window_start) %
                        DDR_RX_BUF_SAMPLES;
                    ntags++;
                }
                if (verbose)
                    fprintf(stderr, "    tag: rate=%d len=%u fcs=%d\n",
                            frame.rate_mbps, (unsigned)frame.length,
                            frame.fcs_ok ? 1 : 0);
            } else if (ret == -1) {
                break;
            } else {
                usleep(200);
            }
        }

        /* Stop DMA so the captured window is stable while we read it. */
        hal_reg_write(REG_IQ_DMA_RX_CONTROL, 0);

        /* --- Evaluate the window's tag set --- */
        bool trigger_present = false, absent_present = false;
        for (int i = 0; i < ntags; i++) {
            if (!tags[i].fcs_ok || tags[i].offset >= window_samples)
                continue;
            if (tags[i].rate_mbps == 6 &&
                tags[i].length == (uint32_t)trigger_len)
                trigger_present = true;
            if (tags[i].length == (uint32_t)absent_len)
                absent_present = true;
        }

        bool this_selected = trigger_present && !absent_present;
        bool last = (attempt == max_windows);

        fprintf(stderr,
                "  [attempt %d] tags=%d trigger(len %d)=%s absent(len %d)=%s%s\n",
                attempt, ntags, trigger_len,
                trigger_present ? "yes" : "NO",
                absent_len, absent_present ? "yes" : "NO",
                this_selected ? "  <-- FAILING WINDOW" : "");

        if (this_selected || write_all || last) {
            char base[600];
            int bn;
            if (write_all || !this_selected) {
                bn = snprintf(base, sizeof(base), "%s_w%02d", prefix, attempt);
            } else {
                bn = snprintf(base, sizeof(base), "%s", prefix);
            }
            if (bn < 0 || bn >= (int)sizeof(base)) {
                fprintf(stderr, "ERROR: output prefix too long\n");
                break;
            }
            ring_read(ddr, window_start, window_samples, winbuf);
            write_capture(base, window_start, window_samples, channel, pre,
                          trigger_len, absent_len, winbuf, tags, ntags,
                          trigger_present, absent_present, this_selected,
                          attempt, verbose);
        }

        if (this_selected) {
            selected = true;
            break;
        }

        /* Re-arm the ring for the next attempt. */
        hal_reg_write(REG_IQ_DMA_RX_CONTROL, RX_CTRL_ENABLE);
        usleep(10000);
    }

    hal_reg_write(REG_IQ_DMA_RX_CONTROL, 0);
    free(winbuf);
    free(tags);

    uint32_t total = deimos_rx_frame_count();
    uint32_t drops = deimos_rx_drop_count();
    fprintf(stderr, "deimos_ota_capture: %u fabric frames, %u drops\n",
            total, drops);

    styx_disarm_cleanup();
    deimos_rx_cleanup();

    if (selected) {
        fprintf(stderr, "SELECTED failing window (trigger present, absent "
                "missing) after %d attempt(s).\n", attempt);
        return 0;
    }
    fprintf(stderr, "No failing window in %d attempt(s) — wrote the last one "
            "for inspection.\n", attempt);
    return 2;
}
