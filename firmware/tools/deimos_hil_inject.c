// SPDX-License-Identifier: MIT
/*
 * deimos_hil_inject — HIL golden vector injection + PSDU verify + snap readback
 *
 * Writes a waveform to DDR, triggers hil_ctrl playback in test mode,
 * arms the snap probe, reads back captured data, and optionally verifies
 * PSDU bytes from the tag FIFO's PSDU_DATA register.
 *
 * Input: waveform JSON file (lib80211 format: {"real":[...],"imag":[...]})
 *        or a rate shorthand like "6mbps" → /usr/share/deimos/vectors/legacy_6mbps_waveform.json
 *
 * Options:
 *   -v, --verify-psdu   Read PSDU bytes from fabric and compare to golden vector
 *
 * Output: if -v: "PSDU OK" or "PSDU MISMATCH" to stdout, details to stderr.
 *         Otherwise: 1024 lines of hex (snap buffer contents) to stdout.
 *
 * Register map:
 *   hil_ctrl @ 0x7C500000:
 *     0x00 CONTROL   [0]=test_mode, [1]=trigger(W1S)
 *     0x04 STATUS    [0]=playback_active, [1]=playback_done
 *     0x08 DDR_BASE  Physical base of test waveform
 *     0x0C PLAY_COUNT  Number of samples
 *     0x10 PLAY_PTR   Current position (RO)
 *
 *   snap_axi @ 0x7C4E0000:
 *     0x00 CONTROL   [0]=arm, [1]=sw_trigger, [2]=circular_en
 *     0x04 STATUS    [0]=captured, [1]=armed
 *     0x0C RD_ADDR   [9:0]
 *     0x10 RD_DATA   [31:0]
 *
 *   tag_fifo_axi @ 0x7C510000:
 *     0x00 STATUS    [0]=tag_empty
 *     0x04 TAG_LO    tag metadata (peek, no pop)
 *     0x08 TAG_HI    psdu_addr[13:0] (read-pop, sets PSDU cursor)
 *     0x28 PSDU_DATA one PSDU byte (read-pop)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <getopt.h>

#include "deimos_tool.h"
#include "deimos_rx.h"

/* --------------------------------------------------------------------------
 * Register aliases (from hal.h via deimos_rx.h — local short names)
 * -------------------------------------------------------------------------- */

#define REG_HIL_CONTROL     REG_HIL_CTRL_CONTROL
#define REG_HIL_STATUS      REG_HIL_CTRL_STATUS
#define REG_HIL_DDR_BASE    REG_HIL_CTRL_DDR_BASE
#define REG_HIL_PLAY_COUNT  REG_HIL_CTRL_PLAY_COUNT
#define REG_HIL_PLAY_PTR    REG_HIL_CTRL_PLAY_PTR

/* HIL LTF skip. NOTE: legacy — ltf_skip/cfo_thresh are no longer written to
 * hardware (read-only register addresses), so -s and -t on the command line
 * are currently informational only. Kept for the day the registers return.
 * Golden vectors start at sample 0 = STF start; STF(160) + GI2(32) = 192. */
#define HIL_LTF_SKIP_DEFAULT   192

/* Tag FIFO / PSDU registers (local aliases for readability) */
#define REG_TAG_STATUS       REG_TAG_FIFO_STATUS
#define REG_TAG_LO           REG_TAG_FIFO_TAG_LO
#define REG_TAG_HI           REG_TAG_FIFO_TAG_HI

#define STATUS_TAG_EMPTY     TAG_STATUS_EMPTY

#define TAG_LO_FCS_OK(t)    (((t) >> 23) & 0x01)
#define TAG_LO_RATE(t)      (((t) >> 19) & 0x0F)
#define TAG_LO_LENGTH(t)    (((t) >> 4) & 0xFFF)

/* Use an offset into the TX buffer region for HIL injection (avoids
 * conflicting with the RX ring buffer or normal TX waveforms).
 * Single definition in hal_deimos.h — shared with deimos_burst_loopback. */
#define HIL_DDR_BASE        DEIMOS_HIL_DDR_BASE
#define HIL_MAX_SAMPLES     DEIMOS_HIL_MAX_SAMPLES

/* --------------------------------------------------------------------------
 * HIL inject + snap capture
 * -------------------------------------------------------------------------- */

static int hil_inject_and_capture(int n_samples, uint32_t snap_buf[SNAP_DEPTH],
                                   int cfo_thresh, int ltf_skip)
{
    /* 1. ARM PIPELINE for HIL mode.
     *    Sets test_mode, disables STF (12ms drain), writes stf_threshold,
     *    flushes tags. (stf_skip/ltf_skip/cfo_thresh are no longer wired to
     *    hardware — their register addresses are read-only reserves.) */
    deimos_pipeline_config_t pipe_cfg = {
        .mode = DEIMOS_RX_MODE_HIL,
        .stf_threshold = 0,
        .stf_skip = 4,
        .ltf_skip = ltf_skip,
        .cfo_thresh = cfo_thresh,
    };
    deimos_pipeline_arm(&pipe_cfg);

    /* 2. Configure playback: base address + sample count */
    hal_reg_write(REG_HIL_DDR_BASE, HIL_DDR_BASE);
    hal_reg_write(REG_HIL_PLAY_COUNT, (uint32_t)n_samples);

    /* 2b. Warm-up pass: run one full decode cycle to initialize pipeline
     *     hardware state (Viterbi path memory, distributed RAM, etc).
     *     Need STF enabled for warm-up to actually detect + decode. */
    deimos_stf_gate(DEIMOS_TRIGGER_HIL);
    /* Wait for pipeline to complete: watchdog is 10ms, allow 15ms. */
    usleep(15000);
    /* Clear trigger, stay in test mode */
    hal_reg_write(REG_HIL_CONTROL, HIL_CTRL_TEST_MODE);
    usleep(1000);

    /* 2c. Re-arm pipeline after warm-up (full STF reset + flush).
     *     This clears any accumulated state from the warm-up pass. */
    deimos_pipeline_arm(&pipe_cfg);

    /* 3. Arm snap probe.
     *    Write 0 first to force out of any stale captured state,
     *    then write ARM to transition to armed. */
    hal_reg_write(REG_SNAP_CONTROL, 0);
    usleep(10);
    hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM);
    usleep(10);

    /* Verify armed */
    uint32_t snap_status = hal_reg_read(REG_SNAP_STATUS);
    if (!(snap_status & 0x02)) {
        fprintf(stderr, "WARNING: Snap not armed (status=0x%08x)\n", snap_status);
    }

    /* 4. Start real pass: enable STF + trigger HIL atomically. */
    deimos_stf_gate(DEIMOS_TRIGGER_HIL);

    /* 5. Wait for snap capture complete (ext_trig fires, then 512 post-trigger samples) */
    int timeout_ms = 500;
    while (timeout_ms > 0) {
        snap_status = hal_reg_read(REG_SNAP_STATUS);
        if (snap_status & 0x01)  /* captured */
            break;
        usleep(1000);
        timeout_ms--;
    }
    if (!(snap_status & 0x01)) {
        fprintf(stderr, "ERROR: Snap capture timeout (status=0x%08x)\n", snap_status);
        /* Also report hil status for debugging */
        uint32_t hil_status = hal_reg_read(REG_HIL_STATUS);
        uint32_t hil_ptr = hal_reg_read(REG_HIL_PLAY_PTR);
        fprintf(stderr, "  hil_status=0x%08x, play_ptr=%u\n", hil_status, hil_ptr);
        return -1;
    }

    /* 6. Ensure playback is complete before reading (avoids bus contention) */
    timeout_ms = 100;
    while (timeout_ms > 0) {
        uint32_t hil_status = hal_reg_read(REG_HIL_STATUS);
        if (hil_status & 0x02)  /* playback_done */
            break;
        usleep(1000);
        timeout_ms--;
    }

    /* 7. Read snap buffer (1024 entries) */
    for (int i = 0; i < SNAP_DEPTH; i++) {
        hal_reg_write(REG_SNAP_RD_ADDR, (uint32_t)i);
        usleep(1);  /* allow BRAM latency */
        snap_buf[i] = hal_reg_read(REG_SNAP_RD_DATA);
    }

    /* 8. Disable test mode */
    hal_reg_write(REG_HIL_CONTROL, 0);

    return 0;
}

/* --------------------------------------------------------------------------
 * PSDU verification — read tag + PSDU bytes, compare to golden vector
 * -------------------------------------------------------------------------- */

/* Expected PSDU (Annex I.1, 100 bytes). psdu_packer emits 96 payload bytes. */
static const uint8_t expected_psdu_100[] = {
    0x04, 0x02, 0x00, 0x2E, 0x00, 0x60, 0x08, 0xCD, 0x37, 0xA6,
    0x00, 0x20, 0xD6, 0x01, 0x3C, 0xF1, 0x00, 0x60, 0x08, 0xAD,
    0x3B, 0xAF, 0x00, 0x00, 0x4A, 0x6F, 0x79, 0x2C, 0x20, 0x62,
    0x72, 0x69, 0x67, 0x68, 0x74, 0x20, 0x73, 0x70, 0x61, 0x72,
    0x6B, 0x20, 0x6F, 0x66, 0x20, 0x64, 0x69, 0x76, 0x69, 0x6E,
    0x69, 0x74, 0x79, 0x2C, 0x0A, 0x44, 0x61, 0x75, 0x67, 0x68,
    0x74, 0x65, 0x72, 0x20, 0x6F, 0x66, 0x20, 0x45, 0x6C, 0x79,
    0x73, 0x69, 0x75, 0x6D, 0x2C, 0x0A, 0x46, 0x69, 0x72, 0x65,
    0x2D, 0x69, 0x6E, 0x73, 0x69, 0x72, 0x65, 0x64, 0x20, 0x77,
    0x65, 0x20, 0x74, 0x72, 0x65, 0x61, 0x67, 0x33, 0x21, 0xB6,
};

static int verify_psdu_output(bool quiet)
{
    /* Wait briefly for tag to arrive (decode + tag write pipeline delay) */
    usleep(20000);  /* 20ms — generous for 1 frame */

    uint32_t status = hal_reg_read(REG_TAG_STATUS);

    if (status & STATUS_TAG_EMPTY) {
        fprintf(stderr, "PSDU FAIL: no tag in FIFO after injection\n");
        printf("PSDU FAIL\n");
        return 1;
    }

    /* Read TAG_LO (peek — metadata) */
    uint32_t tag_lo = hal_reg_read(REG_TAG_LO);
    uint32_t fcs_ok   = TAG_LO_FCS_OK(tag_lo);
    uint32_t rate     = TAG_LO_RATE(tag_lo);
    uint32_t length   = TAG_LO_LENGTH(tag_lo);

    /* Read TAG_HI (pop — psdu_addr + advance FIFO + set PSDU cursor).
     *
     * TAG_HI carries {18'b0, psdu_addr[13:0]} — the BRAM base address, NOT a
     * byte count (tag_fifo_axi.v:15, 346). This code previously decoded it as
     * TAG_HI_BYTE_COUNT and aborted whenever it read 0 — which is exactly what
     * a first frame at BRAM address 0 returns, so -v never verified anything.
     * The byte count comes from TAG_LO's length field: psdu_packer emits
     * length-4 bytes (FCS excluded). deimos_burst_loopback reads TAG_HI
     * correctly as psdu_addr. */
    uint32_t tag_hi    = hal_reg_read(REG_TAG_HI);
    uint32_t psdu_addr = TAG_HI_PSDU_ADDR(tag_hi);
    uint32_t byte_count = (length > 4) ? (length - 4) : 0;

    if (!quiet) {
        fprintf(stderr, "TAG: fcs_ok=%u rate=%u length=%u psdu_addr=%u bytes=%u\n",
                fcs_ok, rate, length, psdu_addr, byte_count);
    }

    if (!fcs_ok) {
        fprintf(stderr, "PSDU FAIL: FCS check failed (fcs_ok=0)\n");
        printf("PSDU FAIL\n");
        return 1;
    }

    if (byte_count == 0) {
        fprintf(stderr, "PSDU FAIL: length=%u yields no PSDU bytes\n", length);
        printf("PSDU FAIL\n");
        return 1;
    }

    /* Read PSDU bytes */
    uint8_t hw_psdu[4096];
    uint32_t bytes_read = 0;
    for (uint32_t i = 0; i < byte_count && i < sizeof(hw_psdu); i++) {
        uint32_t data = hal_reg_read(REG_PSDU_DATA);
        hw_psdu[i] = (uint8_t)(data & 0xFF);
        bytes_read++;
    }

    if (!quiet)
        fprintf(stderr, "Read %u PSDU bytes\n", bytes_read);

    /* Compare against expected PSDU payload (first byte_count bytes of expected_psdu_100) */
    uint32_t expected_len = byte_count;
    if (expected_len > sizeof(expected_psdu_100))
        expected_len = sizeof(expected_psdu_100);

    uint32_t errors = 0;
    for (uint32_t i = 0; i < expected_len && i < bytes_read; i++) {
        if (hw_psdu[i] != expected_psdu_100[i]) {
            if (errors < 10 && !quiet) {
                fprintf(stderr, "  Byte %u: hw=0x%02X expected=0x%02X\n",
                        i, hw_psdu[i], expected_psdu_100[i]);
            }
            errors++;
        }
    }

    if (errors > 0) {
        fprintf(stderr, "PSDU MISMATCH: %u byte errors / %u bytes\n",
                errors, expected_len);
        printf("PSDU MISMATCH\n");
        return 1;
    }

    if (!quiet)
        fprintf(stderr, "PSDU OK: %u bytes match\n", expected_len);

    printf("PSDU OK\n");
    return 0;
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options] <waveform>\n"
        "\n"
        "  <waveform>   Path to JSON file, or rate shorthand (e.g. 6mbps)\n"
        "\n"
        "Options:\n"
        "  -c <hz>      Apply CFO (frequency offset in Hz) before injection\n"
        "  -t <val>     CFO threshold (default: 64 for captures with stf_offset,\n"
        "               1024 for golden vectors; use 64 to enable CFO correction)\n"
        "  -s <val>     LTF skip (samples after trigger before LTF capture;\n"
        "               default: stf_offset+192 if metadata present, else 192)\n"
        "  -o <file>    Write snap output to file instead of stdout\n"
        "  -v           Verify PSDU: read fabric-decoded PSDU bytes and compare\n"
        "               against the Annex I.1 golden vector expected PSDU\n"
        "  -q           Quiet: suppress stderr diagnostics\n"
        "  -h           Show this help\n"
        "\n"
        "When the JSON contains \"stf_offset\", ltf_skip and cfo_threshold are\n"
        "auto-computed unless overridden by -s/-t.\n"
        "\n"
        "Output: 1024 lines of 32-bit hex (snap buffer contents)\n",
        prog);
}

int main(int argc, char *argv[])
{
    const char *outfile = NULL;
    bool quiet = false;
    bool verify_psdu = false;
    double cfo_hz = 0.0;
    int cfo_threshold = -1;  /* -1 = use default (1024) */
    int ltf_skip_val = -1;   /* -1 = use default (192) */
    int opt;

    while ((opt = getopt(argc, argv, "c:t:s:o:qvh")) != -1) {
        switch (opt) {
        case 'c': cfo_hz = atof(optarg); break;
        case 't': cfo_threshold = atoi(optarg); break;
        case 's': ltf_skip_val = atoi(optarg); break;
        case 'o': outfile = optarg; break;
        case 'q': quiet = true; break;
        case 'v': verify_psdu = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "ERROR: No waveform specified\n");
        usage(argv[0]);
        return 1;
    }

    const char *waveform_arg = argv[optind];

    /* Resolve path: rate shorthand or explicit file */
    char path_buf[256];
    const char *waveform_path;

    if (strchr(waveform_arg, '/') || strchr(waveform_arg, '.')) {
        /* Explicit path */
        waveform_path = waveform_arg;
    } else {
        /* Rate shorthand: "6mbps" -> vector file */
        snprintf(path_buf, sizeof(path_buf),
                 "/usr/share/deimos/vectors/legacy_%s_waveform.json",
                 waveform_arg);
        waveform_path = path_buf;
    }

    if (!quiet)
        fprintf(stderr, "Loading waveform: %s\n", waveform_path);

    /* Load and parse waveform */
    float *re = NULL, *im = NULL;
    int n_samples = 0;
    int stf_offset = -1;

    if (deimos_tool_load_waveform(waveform_path, &re, &im, &n_samples, &stf_offset, NULL) != 0)
        return 1;

    if (!quiet) {
        fprintf(stderr, "  Samples: %d (%.2f ms at 20 MSPS)\n",
                n_samples, n_samples / 20000.0);
        if (stf_offset >= 0)
            fprintf(stderr, "  stf_offset: %d (from JSON metadata)\n", stf_offset);
    }

    if (n_samples > HIL_MAX_SAMPLES) {
        fprintf(stderr, "ERROR: Waveform too large (%d > %d samples)\n",
                n_samples, HIL_MAX_SAMPLES);
        free(re); free(im);
        return 1;
    }

    /* Apply CFO if requested (rotate IQ by cfo_hz at 20 MSPS) */
    if (cfo_hz != 0.0) {
        double sample_rate = 20e6;
        double phase_inc = 2.0 * M_PI * cfo_hz / sample_rate;
        if (!quiet)
            fprintf(stderr, "  Applying CFO: %.1f Hz (%.6f rad/sample)\n",
                    cfo_hz, phase_inc);
        for (int i = 0; i < n_samples; i++) {
            double phase = phase_inc * (double)i;
            double c = cos(phase);
            double s = sin(phase);
            float r_new = (float)(re[i] * c - im[i] * s);
            float i_new = (float)(re[i] * s + im[i] * c);
            re[i] = r_new;
            im[i] = i_new;
        }
    }

    /* Initialize HAL */
    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        free(re); free(im);
        return 1;
    }

    /* Get DDR TX buffer pointer and offset to HIL region */
    volatile uint32_t *tx_buf = hal_ddr_tx_buf();
    /* HIL_DDR_BASE is 0x18200000, DDR_TX_BASE is 0x18000000
     * Offset = (0x18200000 - 0x18000000) / 4 = 0x80000 words */
    volatile uint32_t *hil_buf = tx_buf + (HIL_DDR_BASE - DDR_TX_BASE) / 4;

    /* Quantize and write to DDR */
    if (!quiet)
        fprintf(stderr, "Quantizing to 12-bit and writing to DDR @ 0x%08X...\n",
                HIL_DDR_BASE);
    float scale = deimos_tool_quantize_to_ddr(re, im, n_samples, hil_buf);
    if (!quiet)
        fprintf(stderr, "  Scale factor: %.1f\n", scale);
    free(re); free(im);

    /* Resolve threshold and ltf_skip defaults.
     * If the capture has stf_offset metadata and -s was not given,
     * auto-compute: ltf_skip = stf_offset + 192 (STF + GI2).
     * Also auto-enable CFO correction for real captures (threshold 64). */
    int thresh;
    if (cfo_threshold >= 0) {
        thresh = cfo_threshold;
    } else if (stf_offset >= 0) {
        thresh = 64;  /* real capture: enable CFO correction */
    } else {
        thresh = 1024;  /* golden vector: disable CFO correction */
    }

    int ltf_sk;
    if (ltf_skip_val >= 0) {
        ltf_sk = ltf_skip_val;
    } else if (stf_offset >= 0) {
        ltf_sk = stf_offset + HIL_LTF_SKIP_DEFAULT;
        if (!quiet)
            fprintf(stderr, "  Auto ltf_skip: %d + %d = %d (from stf_offset)\n",
                    stf_offset, HIL_LTF_SKIP_DEFAULT, ltf_sk);
    } else {
        ltf_sk = HIL_LTF_SKIP_DEFAULT;
    }

    /* Inject and capture */
    if (!quiet)
        fprintf(stderr, "Triggering HIL playback (%d samples, cfo_thresh=%d, ltf_skip=%d)...\n",
                n_samples, thresh, ltf_sk);

    uint32_t snap_buf[SNAP_DEPTH];
    if (hil_inject_and_capture(n_samples, snap_buf, thresh, ltf_sk) != 0) {
        hal_cleanup();
        return 1;
    }

    if (!quiet)
        fprintf(stderr, "Snap capture complete.\n");

    /* ---- PSDU verification mode ---- */
    if (verify_psdu) {
        int result = verify_psdu_output(quiet);
        hal_cleanup();
        return result;
    }

    /* Output snap data */
    FILE *out = stdout;
    if (outfile) {
        out = fopen(outfile, "w");
        if (!out) {
            fprintf(stderr, "ERROR: Cannot open output file %s\n", outfile);
            hal_cleanup();
            return 1;
        }
    }

    for (int i = 0; i < SNAP_DEPTH; i++) {
        fprintf(out, "%08x\n", snap_buf[i]);
    }

    if (outfile) fclose(out);

    hal_cleanup();

    if (!quiet)
        fprintf(stderr, "Done.\n");

    return 0;
}
