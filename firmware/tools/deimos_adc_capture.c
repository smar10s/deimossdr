// SPDX-License-Identifier: MIT
/*
 * deimos_adc_capture — Capture raw ADC IQ during cable loopback TX
 *
 * Diagnostic tool: TX a known frame via DAC/cable, capture the raw ADC
 * IQ that the fabric receives, and dump it as JSON. The captured IQ can
 * then be replayed through deimos_hil_inject to isolate whether decode
 * failures are caused by IQ quality or by pipeline detection logic.
 *
 * Method:
 *   1. Configure AD9361 (same as fabric_loopback)
 *   2. Enable iq_dma_rx (continuous ADC → DDR ring buffer)
 *   3. Record WR_PTR (start position)
 *   4. TX the frame (one-shot DMA)
 *   5. Wait for TX complete + propagation margin
 *   6. Record WR_PTR (end position)
 *   7. Read the DDR region [start, end) — this IS the received signal
 *   8. Output as JSON: {"real":[...],"imag":[...]}
 *
 * The output JSON is directly feedable to deimos_hil_inject for replay.
 *
 * Usage:
 *   deimos_adc_capture -r 6 > captured_6mbps.json
 *   deimos_hil_inject captured_6mbps.json   # replay through fabric
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <getopt.h>

#include "deimos_tool.h"
#include "dma_tx.h"
#include "deimos_rx.h"

#include <lib80211/fft.h>
#include <lib80211/tx.h>
#include <lib80211/mac.h>

/* --------------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------------- */

#define SAMPLE_RATE_HZ   20000000ULL
#define BANDWIDTH_HZ     28000000ULL
#define DEFAULT_PAYLOAD  80       /* bytes */
#define MAX_PAYLOAD      1400

/* DDR ring buffer parameters (must match iq_dma_rx RTL) */
#define DDR_RX_BUF_SAMPLES  (DDR_RX_SIZE / 4)  /* 128MB / 4 bytes = 33554432 */

/* Extra samples to capture before and after the TX frame window */
#define PRE_CAPTURE_SAMPLES  4000   /* ~200us before frame — captures noise floor + STF start */
#define POST_CAPTURE_SAMPLES 2000   /* ~100us after frame ends */

/* Max capture size (safety limit: 2M samples = 8 MB) */
#define MAX_CAPTURE_SAMPLES  (2 * 1024 * 1024)

static const int RATES[] = {6, 9, 12, 18, 24, 36, 48, 54};
#define N_RATES 8

/* --------------------------------------------------------------------------
 * Ring buffer read helper — handles wrap-around
 * -------------------------------------------------------------------------- */

static void ring_read(volatile uint32_t *ddr_rx, uint32_t start, uint32_t end,
                      int16_t *out_re, int16_t *out_im, uint32_t *out_count)
{
    uint32_t count;
    if (end >= start) {
        count = end - start;
    } else {
        /* Wrapped around */
        count = (DDR_RX_BUF_SAMPLES - start) + end;
    }

    if (count > MAX_CAPTURE_SAMPLES) {
        fprintf(stderr, "WARNING: Capture too large (%u samples), truncating to %d\n",
                count, MAX_CAPTURE_SAMPLES);
        /* Take the last MAX_CAPTURE_SAMPLES before end */
        if (end >= MAX_CAPTURE_SAMPLES) {
            start = end - MAX_CAPTURE_SAMPLES;
        } else {
            start = DDR_RX_BUF_SAMPLES - (MAX_CAPTURE_SAMPLES - end);
        }
        count = MAX_CAPTURE_SAMPLES;
    }

    *out_count = count;

    uint32_t pos = start;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t word = ddr_rx[pos];
        out_re[i] = IQ_REAL(word);
        out_im[i] = IQ_IMAG(word);
        pos++;
        if (pos >= DDR_RX_BUF_SAMPLES)
            pos = 0;
    }
}

/* -------------------------------------------------------------------------- */

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "  Capture raw ADC IQ during a cable loopback TX frame.\n"
        "  Output is JSON (deimos_hil_inject compatible) on stdout.\n"
        "\n"
        "Options:\n"
        "  -c channel       Channel number (default: 149)\n"
        "  -r rate          TX rate in Mbps (default: 6)\n"
        "  -p payload_bytes Payload size (default: 80, max: 1400)\n"
        "  -a tx_atten_dB   TX attenuation (default: 3.0)\n"
        "  -g rx_gain_dB    RX gain (default: 24.0)\n"
        "  -o file          Write JSON to file instead of stdout\n"
        "  -R               Output raw hex (one 32-bit word per line) instead of JSON\n"
        "  -L               Level telemetry only: print level JSON to stdout, exit 0\n"
        "  -v               Verbose: print diagnostics to stderr\n"
        "  -h               Show this help\n"
        "\n"
        "Examples:\n"
        "  %s -r 6 > captured_6mbps.json\n"
        "  deimos_hil_inject captured_6mbps.json    # replay\n",
        prog, prog);
}

int main(int argc, char *argv[])
{
    int channel = 149;
    int rate = 6;
    double tx_atten = 3.0;
    double rx_gain = 24.0;
    int payload_len = DEFAULT_PAYLOAD;
    const char *outfile = NULL;
    bool verbose = false;
    bool raw_hex = false;
    bool level_only = false;
    int opt;

    while ((opt = getopt(argc, argv, "c:r:p:a:g:o:RLvh")) != -1) {
        switch (opt) {
        case 'c': channel = atoi(optarg); break;
        case 'r': rate = atoi(optarg); break;
        case 'p': payload_len = atoi(optarg); break;
        case 'a': tx_atten = atof(optarg); break;
        case 'g': rx_gain = atof(optarg); break;
        case 'o': outfile = optarg; break;
        case 'R': raw_hex = true; break;
        case 'L': level_only = true; break;
        case 'v': verbose = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (payload_len < 1) payload_len = 1;
    if (payload_len > MAX_PAYLOAD) payload_len = MAX_PAYLOAD;

    /* Validate rate */
    bool valid_rate = false;
    for (int i = 0; i < N_RATES; i++) {
        if (RATES[i] == rate) { valid_rate = true; break; }
    }
    if (!valid_rate) {
        fprintf(stderr, "ERROR: Invalid rate %d (must be 6/9/12/18/24/36/48/54)\n", rate);
        return 1;
    }

    /* Build test PSDU */
    uint8_t psdu[24 + MAX_PAYLOAD + 4];
    size_t psdu_len = deimos_tool_build_test_psdu(psdu, payload_len);

    /* Initialize HAL */
    if (hal_init() != 0) {
        fprintf(stderr, "ERROR: hal_init failed\n");
        return 1;
    }

    /* Configure radio (same as fabric_loopback) */
    uint64_t freq = deimos_rx_channel_to_freq(channel);
    styx_rf_config_t rf = {
        .sample_rate_hz = SAMPLE_RATE_HZ,
        .tx_lo_hz = freq,
        .rx_lo_hz = freq,
        .tx_bw_hz = BANDWIDTH_HZ,
        .rx_bw_hz = BANDWIDTH_HZ,
        .tx_atten_db = tx_atten,
        .rx_gain_mode = "manual",
        .rx_gain_db = rx_gain,
        .settle_us = 100000,
    };
    if (styx_rf_configure(&rf) != 0) {
        fprintf(stderr, "ERROR: RF configure failed\n");
        hal_cleanup();
        return 1;
    }

    if (verbose)
        fprintf(stderr, "ADC capture: ch=%d, freq=%llu Hz, rate=%d Mbps, "
                "atten=%.1f dB, gain=%.1f dB, payload=%d bytes\n",
                channel, (unsigned long long)freq, rate, tx_atten, rx_gain,
                payload_len);

    /* Arm the pipeline exactly like fabric_loopback: the STF-disabled
     * 12ms drain resets stf_detect and lets any in-flight decode abort
     * (frame_fifo drains), live registers are configured, and the tag
     * FIFO is flushed. Without this, leftover state from HIL tools
     * leaves the fabric unable to tag — snap never triggers and the
     * capture reports 100% 'timeout' despite valid IQ. */
    deimos_pipeline_config_t pipe_cfg = DEIMOS_PIPELINE_LIVE_DEFAULT;
    deimos_pipeline_arm(&pipe_cfg);

    /* Enable iq_dma_rx: set DDR base, then enable */
    hal_reg_write(REG_IQ_DMA_RX_DDR_BASE, DDR_RX_BASE);
    hal_reg_write(REG_IQ_DMA_RX_CONTROL, RX_CTRL_ENABLE);
    usleep(10000);  /* let it start capturing */

    /* Verify DMA is running: check that WR_PTR is advancing */
    uint32_t ptr_a = hal_reg_read(REG_IQ_DMA_RX_WR_PTR);
    usleep(1000);  /* 1ms = ~20000 samples at 20 MSPS */
    uint32_t ptr_b = hal_reg_read(REG_IQ_DMA_RX_WR_PTR);
    if (ptr_a == ptr_b) {
        fprintf(stderr, "ERROR: iq_dma_rx WR_PTR not advancing (stuck at %u)\n", ptr_a);
        hal_cleanup();
        return 1;
    }
    if (verbose)
        fprintf(stderr, "  iq_dma_rx active: WR_PTR %u -> %u (+%u samples in 1ms)\n",
                ptr_a, ptr_b, (ptr_b > ptr_a) ? ptr_b - ptr_a : ptr_b + DDR_RX_BUF_SAMPLES - ptr_a);

    /* Generate TX waveform */
    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "ERROR: fft_plan_create failed\n");
        hal_cleanup();
        return 1;
    }

    lib80211_tx_legacy_params tx_params = {
        .rate_mbps = rate,
        .psdu = psdu,
        .psdu_len = psdu_len,
        .scrambler_seed = 0x5D,
    };

    size_t frame_samples = lib80211_tx_legacy_samples(&tx_params);
    size_t total_tx = DEIMOS_SILENCE_PAD + frame_samples + DEIMOS_SILENCE_PAD;

    if (total_tx > DMA_TX_MAX_SAMPLES) {
        fprintf(stderr, "ERROR: TX waveform too large (%zu samples)\n", total_tx);
        lib80211_fft_plan_destroy(plan);
        hal_cleanup();
        return 1;
    }

    float *tx_real = calloc(total_tx, sizeof(float));
    float *tx_imag = calloc(total_tx, sizeof(float));
    if (!tx_real || !tx_imag) {
        fprintf(stderr, "ERROR: alloc failed\n");
        free(tx_real); free(tx_imag);
        lib80211_fft_plan_destroy(plan);
        hal_cleanup();
        return 1;
    }

    size_t gen = lib80211_tx_legacy(plan, &tx_params,
                                    tx_real + DEIMOS_SILENCE_PAD,
                                    tx_imag + DEIMOS_SILENCE_PAD);
    lib80211_fft_plan_destroy(plan);

    if (gen == 0) {
        fprintf(stderr, "ERROR: lib80211_tx_legacy failed\n");
        free(tx_real); free(tx_imag);
        hal_cleanup();
        return 1;
    }

    if (verbose)
        fprintf(stderr, "  TX frame: %zu samples (%.2f ms) + %d silence = %zu total\n",
                frame_samples, frame_samples / 20000.0, DEIMOS_SILENCE_PAD * 2, total_tx);

    /* Load TX waveform (don't trigger yet — we need to record WR_PTR first) */
    if (dma_tx_load(tx_real, tx_imag, total_tx, false) != 0) {
        fprintf(stderr, "ERROR: dma_tx_load failed\n");
        free(tx_real); free(tx_imag);
        hal_cleanup();
        return 1;
    }
    free(tx_real); free(tx_imag);

    /* Record WR_PTR before TX (gives us the capture start, minus pre-margin) */
    uint32_t wr_ptr_before = hal_reg_read(REG_IQ_DMA_RX_WR_PTR);
    if (verbose)
        fprintf(stderr, "  WR_PTR before TX: %u\n", wr_ptr_before);

    /* Arm snap probe — triggers on the decode engine's snap_trig (S_TAG_OUT) */
    hal_reg_write(REG_SNAP_CONTROL, 0);     /* clear stale state */
    usleep(10);
    hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM);
    usleep(10);

    /* Trigger TX: enable STF then fire DMA TX (atomic via stf_gate —
     * same as fabric_loopback's phase 4). The snap was armed while STF
     * was disabled, so it can only capture the real frame's decode. */
    if (deimos_stf_gate(DEIMOS_TRIGGER_DMA_TX) != 0) {
        fprintf(stderr, "ERROR: deimos_stf_gate failed\n");
        hal_cleanup();
        return 1;
    }

    /* Wait for TX to complete.
     * TX time = total_tx / 20e6 seconds.
     * Add generous margin for pipeline latency and DAC/ADC round-trip. */
    uint32_t tx_time_us = (uint32_t)((total_tx * 1000000ULL) / SAMPLE_RATE_HZ);
    uint32_t wait_us = tx_time_us + 5000;  /* +5ms margin */
    if (verbose)
        fprintf(stderr, "  Waiting %u us for TX + propagation...\n", wait_us);
    usleep(wait_us);

    /* Wait for DMA TX done */
    int timeout_ms = 100;
    while (timeout_ms > 0 && !dma_tx_done()) {
        usleep(1000);
        timeout_ms--;
    }
    dma_tx_stop();

    /* Check fabric decode result via snap probe.
     * The snap triggers on S_TAG_OUT — by now (after TX wait) it should
     * already have fired if the fabric decoded the frame.
     * Note: DMA RX is still running during this check, but that's fine —
     * the ring buffer is 128MB (1.7s) so the extra few ms won't wrap. */
    int decode_result = 0;  /* 0=timeout, 1=pass, -1=fcs_fail */
    {
        int snap_wait_ms = 50;  /* extra wait if not yet captured */
        bool snap_captured = false;
        while (snap_wait_ms > 0) {
            uint32_t ss = hal_reg_read(REG_SNAP_STATUS);
            if (ss & 0x01) {
                snap_captured = true;
                break;
            }
            usleep(1000);
            snap_wait_ms--;
        }

        if (snap_captured) {
            uint32_t trig_pos = (hal_reg_read(REG_SNAP_STATUS) >> 16) & 0x3FF;
            uint32_t tag_word = deimos_tool_find_snap_tag(trig_pos);

            uint32_t fcs_ok = SNAP_FCS_OK(tag_word);
            uint32_t decoded_rate = SNAP_RATE(tag_word);
            uint32_t length = SNAP_LENGTH(tag_word);
            uint8_t expected_rate = DEIMOS_RATE_CODES[rate];

            if (fcs_ok && decoded_rate == expected_rate) {
                decode_result = 1;
                fprintf(stderr, "Fabric decode: PASS (rate=0x%x, len=%u)\n",
                        decoded_rate, length);
            } else {
                decode_result = -1;
                fprintf(stderr, "Fabric decode: FAIL (fcs_ok=%u, rate=0x%x want 0x%x, len=%u)\n",
                        fcs_ok, decoded_rate, expected_rate, length);
            }
        } else {
            decode_result = 0;
            fprintf(stderr, "Fabric decode: TIMEOUT (no detect)\n");
        }
    }

    /* Record WR_PTR after TX + wait */
    uint32_t wr_ptr_after = hal_reg_read(REG_IQ_DMA_RX_WR_PTR);

    /* CRITICAL: Stop DMA RX immediately to prevent overwriting our captured data.
     * The 128 MB ring buffer wraps in ~1.7 seconds at 20 MSPS. If we don't stop,
     * DDR reads below will return stale/overwritten data. */
    hal_reg_write(REG_IQ_DMA_RX_CONTROL, 0);

    if (verbose)
        fprintf(stderr, "  WR_PTR after TX:  %u (DMA stopped)\n", wr_ptr_after);

    /* Calculate capture window:
     * Start = wr_ptr_before - PRE_CAPTURE_SAMPLES  (for noise baseline)
     * End   = wr_ptr_after + POST_CAPTURE_SAMPLES  (for tail) */
    uint32_t cap_start;
    if (wr_ptr_before >= PRE_CAPTURE_SAMPLES) {
        cap_start = wr_ptr_before - PRE_CAPTURE_SAMPLES;
    } else {
        cap_start = DDR_RX_BUF_SAMPLES - (PRE_CAPTURE_SAMPLES - wr_ptr_before);
    }

    uint32_t cap_end = (wr_ptr_after + POST_CAPTURE_SAMPLES) % DDR_RX_BUF_SAMPLES;

    /* Calculate total capture size */
    uint32_t cap_count;
    if (cap_end >= cap_start) {
        cap_count = cap_end - cap_start;
    } else {
        cap_count = (DDR_RX_BUF_SAMPLES - cap_start) + cap_end;
    }

    if (verbose)
        fprintf(stderr, "  Capture window: [%u, %u) = %u samples (%.2f ms)\n",
                cap_start, cap_end, cap_count, cap_count / 20000.0);

    if (cap_count == 0 || cap_count > MAX_CAPTURE_SAMPLES) {
        fprintf(stderr, "ERROR: Unexpected capture size %u\n", cap_count);
        hal_cleanup();
        return 1;
    }

    /* Read captured IQ from DDR */
    volatile uint32_t *ddr_rx = hal_ddr_rx_buf();
    int16_t *cap_re = malloc(cap_count * sizeof(int16_t));
    int16_t *cap_im = malloc(cap_count * sizeof(int16_t));
    if (!cap_re || !cap_im) {
        fprintf(stderr, "ERROR: alloc for capture buffer failed\n");
        free(cap_re); free(cap_im);
        hal_cleanup();
        return 1;
    }

    uint32_t actual_count = 0;
    ring_read(ddr_rx, cap_start, cap_end, cap_re, cap_im, &actual_count);

    if (verbose)
        fprintf(stderr, "  Read %u samples from DDR\n", actual_count);

    hal_cleanup();

    /* Signal-level diagnostics — always printed (key diagnostic output) */
    int16_t peak_re = 0, peak_im = 0;
    double rms_re = 0.0, rms_im = 0.0;
    int clip_count = 0;
    for (uint32_t i = 0; i < actual_count; i++) {
        int16_t r = cap_re[i], q = cap_im[i];
        if (abs(r) > abs(peak_re)) peak_re = r;
        if (abs(q) > abs(peak_im)) peak_im = q;
        rms_re += (double)r * r;
        rms_im += (double)q * q;
        /* Clipping: within 1 LSB of full-scale */
        if (abs(r) >= 2047 || abs(q) >= 2047) clip_count++;
    }
    rms_re = sqrt(rms_re / actual_count);
    rms_im = sqrt(rms_im / actual_count);

    fprintf(stderr, "Signal analysis (%u samples, %.2f ms):\n",
            actual_count, (double)actual_count / 20000.0);
    fprintf(stderr, "  Peak I: %d / 2047 (%.1f%%)\n",
            peak_re, fabs((double)peak_re) / 2047.0 * 100.0);
    fprintf(stderr, "  Peak Q: %d / 2047 (%.1f%%)\n",
            peak_im, fabs((double)peak_im) / 2047.0 * 100.0);
    fprintf(stderr, "  RMS I: %.1f  RMS Q: %.1f\n", rms_re, rms_im);
    fprintf(stderr, "  Clipped samples: %d / %u (%.2f%%)\n",
            clip_count, actual_count,
            (double)clip_count / actual_count * 100.0);
    if (clip_count > 0)
        fprintf(stderr, "  WARNING: Clipping detected — consider increasing TX attenuation\n");
    if (fabs((double)peak_re) < 100 && fabs((double)peak_im) < 100)
        fprintf(stderr, "  WARNING: Very low signal — consider decreasing TX attenuation or increasing RX gain\n");

    if (level_only) {
        /* Telemetry mode: compact JSON on stdout, exit 0 regardless of
         * decode result (this is a measurement, not a test). */
        printf("{\"peak_i\":%d,\"peak_q\":%d,\"rms_i\":%.1f,\"rms_q\":%.1f,"
               "\"clipped\":%d,\"samples\":%u}\n",
               peak_re, peak_im, rms_re, rms_im, clip_count, actual_count);
        free(cap_re); free(cap_im);
        return 0;
    }

    /* Output */
    FILE *out = stdout;
    if (outfile) {
        out = fopen(outfile, "w");
        if (!out) {
            fprintf(stderr, "ERROR: Cannot open %s for writing\n", outfile);
            free(cap_re); free(cap_im);
            return 1;
        }
    }

    if (raw_hex) {
        /* Raw hex output (one word per line, same format as DDR) */
        uint32_t pos = cap_start;
        for (uint32_t i = 0; i < actual_count; i++) {
            fprintf(out, "%08x\n", IQ_PACK(cap_re[i], cap_im[i]));
            pos++;
            if (pos >= DDR_RX_BUF_SAMPLES) pos = 0;
        }
    } else {
        /* JSON output compatible with deimos_hil_inject.
         * Added: fabric_decode field for scripted capture-on-fail workflows. */
        const char *decode_str = (decode_result == 1) ? "pass" :
                                 (decode_result == -1) ? "fail" : "timeout";
        fprintf(out, "{\"rate\":%d,\"fabric_decode\":\"%s\",\"real\":[", rate, decode_str);
        for (uint32_t i = 0; i < actual_count; i++) {
            if (i > 0) fprintf(out, ",");
            /* Output as float (12-bit integer values, same scale as lib80211 vectors
             * after quantization — divide by 2047 to normalize to [-1, 1]) */
            fprintf(out, "%.6f", (double)cap_re[i] / 2047.0);
        }
        fprintf(out, "],\"imag\":[");
        for (uint32_t i = 0; i < actual_count; i++) {
            if (i > 0) fprintf(out, ",");
            fprintf(out, "%.6f", (double)cap_im[i] / 2047.0);
        }
        fprintf(out, "]}\n");
    }

    if (outfile) fclose(out);

    free(cap_re); free(cap_im);

    fprintf(stderr, "Done. Replay with: deimos_hil_inject %s\n",
            outfile ? outfile : "<stdout_file>");

    /* Exit code: 0=pass, 1=fail/timeout (useful for scripted capture-on-fail) */
    return (decode_result == 1) ? 0 : 2;
}
