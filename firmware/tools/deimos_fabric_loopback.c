// SPDX-License-Identifier: MIT
/*
 * deimos_fabric_loopback — Cable loopback with fabric decode validation
 *
 * TX a known frame via DMA/DAC, receive via ADC into the fabric decode
 * pipeline (live mode, test_mode=0), read snap buffer for decode result.
 *
 * This is the "Layer 3" validation tool: proves the fabric decodes real
 * RF signals (with CFO, channel distortion, noise) — not just golden vectors.
 *
 * Method:
 *   1. Configure AD9361 (freq, gain, sample rate)
 *   2. Ensure test_mode=0 (live ADC feeds pipeline)
 *   3. Set hil_ctrl STF_END_SKIP=1 (live mode timing)
 *   4. Generate TX waveform via lib80211 at requested rate
 *   5. Arm snap probe (waits for ext_trig from decode_engine S_TAG_OUT)
 *   6. TX the frame (one-shot DMA)
 *   7. Wait for snap capture (fabric detected + decoded the frame)
 *   8. Read snap: {state[4:0], fcs_ok, parsed_rate[3:0], tag_fcs_ok, 9'b0, parsed_length[11:0]}
 *   9. Report result
 *
 * Unlike pluto_loopback (which uses lib80211 ARM decode), this tool
 * validates the FPGA fabric decode path on real RF signals.
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

/* -------------------------------------------------------------------------- */

#define DEFAULT_PAYLOAD  80    /* bytes */
#define MAX_PAYLOAD      1400

static const int RATES[] = {6, 9, 12, 18, 24, 36, 48, 54};
#define N_RATES 8

/*
 * Run one fabric loopback trial.
 *
 * Sequencing: disable → arm → load → enable → trigger
 *
 * This eliminates the race condition where noise triggers stf_detect between
 * re-enable and snap arm. By arming the snap while stf_detect is disabled
 * (decode pipeline guaranteed IDLE), the snap can only fire on the real TX frame.
 *
 * Returns:
 *   1  = PASS (snap captured, fcs_ok=1, rate matches)
 *   0  = FAIL (snap captured, fcs_ok=0 or wrong rate)
 *  -1  = TIMEOUT (snap never captured — no STF detect or pipeline stall)
 *  -2  = ERROR (TX or setup failure)
 */
static int run_one_trial(lib80211_fft_plan *plan,
                         const uint8_t *psdu, size_t psdu_len,
                         int rate, bool verbose)
{
    lib80211_tx_legacy_params tx_params = {
        .rate_mbps = rate,
        .psdu = psdu,
        .psdu_len = psdu_len,
        .scrambler_seed = 0x5D,
    };

    size_t frame_samples = lib80211_tx_legacy_samples(&tx_params);
    size_t total_tx = DEIMOS_SILENCE_PAD + frame_samples + DEIMOS_SILENCE_PAD;
    if (total_tx > DMA_TX_MAX_SAMPLES)
        return -2;

    float *tx_real = calloc(total_tx, sizeof(float));
    float *tx_imag = calloc(total_tx, sizeof(float));
    if (!tx_real || !tx_imag) {
        free(tx_real); free(tx_imag);
        return -2;
    }

    size_t gen = lib80211_tx_legacy(plan, &tx_params,
                                    tx_real + DEIMOS_SILENCE_PAD,
                                    tx_imag + DEIMOS_SILENCE_PAD);
    if (gen == 0) {
        free(tx_real); free(tx_imag);
        return -2;
    }

    /* ---- Phase 1: ARM PIPELINE ----
     * Disables STF (12ms drain), configures all registers, flushes tags.
     * On return: pipeline configured, STF disabled — safe to do slow work. */
    deimos_pipeline_config_t pipe_cfg = DEIMOS_PIPELINE_LIVE_DEFAULT;
    deimos_pipeline_arm(&pipe_cfg);

    /* ---- Phase 2: ARM SNAP ----
     * The decode pipeline is guaranteed in S_IDLE (watchdog expired or was
     * already idle). stf_detect is disabled so no triggers can fire.
     * Arming while pipeline is quiescent means the snap can only capture
     * the real TX frame's decode. */
    hal_reg_write(REG_SNAP_CONTROL, 0);     /* clear any stale state */
    usleep(10);
    hal_reg_write(REG_SNAP_CONTROL, SNAP_CTRL_ARM);
    usleep(10);

    /* Verify armed */
    uint32_t snap_status = hal_reg_read(REG_SNAP_STATUS);
    if (!(snap_status & 0x02)) {
        if (verbose)
            fprintf(stderr, "  WARNING: snap not armed (status=0x%08x)\n", snap_status);
    }

    /* ---- Phase 3: LOAD TX DMA ----
     * Load waveform into DDR + configure DMA registers.
     * Does not transmit yet (two-phase: load then trigger). */
    if (dma_tx_load(tx_real, tx_imag, total_tx, false) != 0) {
        free(tx_real); free(tx_imag);
        return -2;
    }

    /* ---- Phase 4: ENABLE + TRIGGER ----
     * Enable STF then immediately trigger TX (atomic via stf_gate).
     * The DEIMOS_SILENCE_PAD (2000 samples = 100μs) at the start of the TX waveform
     * gives stf_detect time to fill its window (82 samples at 20 MSPS = 4.1μs)
     * before the real frame's STF arrives. */
    if (deimos_stf_gate(DEIMOS_TRIGGER_DMA_TX) != 0) {
        free(tx_real); free(tx_imag);
        return -2;
    }

    free(tx_real); free(tx_imag);

    /* Wait for snap capture (fabric decode completes and fires snap_trig).
     * Budget: TX time + pipeline latency + decode time.
     * Worst case: 54Mbps, 1400B payload = ~5ms total. Use 200ms timeout. */
    int timeout_ms = 200;
    bool captured = false;
    while (timeout_ms > 0) {
        snap_status = hal_reg_read(REG_SNAP_STATUS);
        if (snap_status & 0x01) {
            captured = true;
            break;
        }
        usleep(1000);
        timeout_ms--;
    }

    /* Stop TX DMA (should already be done for one-shot) */
    dma_tx_stop();

    if (!captured) {
        if (verbose)
            fprintf(stderr, "  TIMEOUT: snap never captured (no decode)\n");
        return -1;
    }

    /* Read snap buffer — we only need a few entries around the trigger point
     * to find the tag output. The snap fires at S_TAG_OUT, so the last few
     * entries before freeze contain the final state. Read the trigger position
     * and then look for state=S_TAG_OUT or S_DONE entries. */
    uint32_t trig_pos = (hal_reg_read(REG_SNAP_STATUS) >> 16) & 0x3FF;

    /* Entry at trig_pos should be S_TAG_OUT or S_DONE with tag data; search
     * backwards a little in case the trigger landed on a neighbouring state. */
    uint32_t tag_word = deimos_tool_find_snap_tag(trig_pos);

    uint32_t state = SNAP_STATE(tag_word);
    uint32_t fcs_ok = SNAP_FCS_OK(tag_word);
    uint32_t decoded_rate = SNAP_RATE(tag_word);
    uint32_t tag_fcs = SNAP_TAG_FCS(tag_word);
    uint32_t length = SNAP_LENGTH(tag_word);

    if (verbose) {
        fprintf(stderr, "  snap[%u]: state=%u, fcs_ok=%u, rate=0x%x, "
                "tag_fcs=%u, length=%u\n",
                trig_pos, state, fcs_ok, decoded_rate, tag_fcs, length);

        /* In non-metadata snap modes, dump raw buffer around trigger */
        uint32_t current_mode = hal_reg_read(REG_DEIMOS_SNAP_MODE);
        if (current_mode != 0) {
            fprintf(stderr, "  snap_mode=%u raw dump (trigger @ %u):\n", current_mode, trig_pos);
            /* Mode 4 (STF threshold diag): dump full buffer for analysis */
            int dump_start = (current_mode == 4) ? -512 : -32;
            int dump_end   = (current_mode == 4) ?  511 :   8;
            for (int off = dump_start; off <= dump_end; off++) {
                int addr = ((int)trig_pos + off) & 0x3FF;
                hal_reg_write(REG_SNAP_RD_ADDR, (uint32_t)addr);
                usleep(1);
                uint32_t w = hal_reg_read(REG_SNAP_RD_DATA);
                /* Mode 4: only print non-trivial samples (threshold_met or nonzero operands) */
                if (current_mode == 4) {
                    uint32_t thr = (w >> 31) & 1;
                    uint32_t lat = (w >> 30) & 1;
                    uint32_t psq = (w >> 16) & 0x1FFF;
                    uint32_t epr = (w >> 3) & 0x1FFF;
                    if (thr || lat || psq || epr)
                        fprintf(stderr, "    [%+4d] 0x%08x  thr=%u lat=%u psq=0x%04x epr=0x%04x per=%u\n",
                                off, w, thr, lat, psq, epr, w & 7);
                } else {
                    fprintf(stderr, "    [%+3d] 0x%08x\n", off, w);
                }
            }
        }
    }

    /* Validate: rate matches expected and FCS OK */
    uint8_t expected_rate = (rate <= 54) ? DEIMOS_RATE_CODES[rate] : 0;
    size_t expected_len = psdu_len - 4;  /* PSDU len without FCS */

    if (fcs_ok && decoded_rate == expected_rate) {
        return 1;  /* PASS */
    } else {
        if (verbose) {
            fprintf(stderr, "  FAIL: fcs_ok=%u (want 1), rate=0x%x (want 0x%x), "
                    "len=%u (want %zu)\n",
                    fcs_ok, decoded_rate, expected_rate, length, expected_len);
        }
        return 0;  /* FAIL */
    }
}

/* -------------------------------------------------------------------------- */

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "  Cable loopback test: TX a frame, fabric decodes via real RF path.\n"
        "  Reports whether fabric decode produces correct rate + FCS OK.\n"
        "\n"
        "Options:\n"
        "  -c channel       Channel number (default: 149)\n"
        "  -n trials        Trials per rate (default: 3)\n"
        "  -r rate          Test only this rate (6/9/12/18/24/36/48/54)\n"
        "  -p payload_bytes Payload size (default: 80, max: 1400)\n"
        "  -a tx_atten_dB   TX attenuation (default: 3.0)\n"
        "  -g rx_gain_dB    RX gain (default: 24.0)\n"
        "  -A agc_mode      AGC mode (0=manual 1=fast_attack 2=slow_attack, default: 0)\n"
        "  -b bw_MHz        RX/TX analog bandwidth in MHz (default: 28)\n"
        "  -m snap_mode     Snap observation (0=meta 1=ltf_peak 2=IQ 3=debug, default: 0)\n"
        "  -v               Verbose: print per-trial details\n"
        "  -h               Show this help\n"
        "\n"
        "Output: JSON summary to stdout, diagnostics to stderr.\n",
        prog);
}

int main(int argc, char *argv[])
{
    int channel = 149;
    double tx_atten = 3.0;
    double rx_gain = 24.0;
    int agc_mode = DEIMOS_AGC_MANUAL;
    int n_trials = 3;
    int payload_len = DEFAULT_PAYLOAD;
    int single_rate = 0;  /* 0 = test all rates */
    int snap_mode = 0;    /* 0=metadata, 1=ltf_peak metric, 2=IQ, 3=debug */
    int bw_mhz = 28;     /* analog filter bandwidth in MHz */
    bool verbose = false;
    int opt;

    while ((opt = getopt(argc, argv, "c:n:r:p:a:g:A:b:m:vh")) != -1) {
        switch (opt) {
        case 'c': channel = atoi(optarg); break;
        case 'n': n_trials = atoi(optarg); break;
        case 'r': single_rate = atoi(optarg); break;
        case 'p': payload_len = atoi(optarg); break;
        case 'a': tx_atten = atof(optarg); break;
        case 'g': rx_gain = atof(optarg); break;
        case 'A': agc_mode = atoi(optarg); break;
        case 'b': bw_mhz = atoi(optarg); break;
        case 'm': snap_mode = atoi(optarg); break;
        case 'v': verbose = true; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }

    if (n_trials < 1) n_trials = 1;
    if (n_trials > 100) n_trials = 100;
    if (payload_len < 1) payload_len = 1;
    if (payload_len > MAX_PAYLOAD) payload_len = MAX_PAYLOAD;

    /* Build test PSDU */
    uint8_t psdu[24 + MAX_PAYLOAD + 4];
    size_t psdu_len = deimos_tool_build_test_psdu(psdu, payload_len);

    /* Configure radio via unified API */
    deimos_radio_config_t radio_cfg = {
        .channel = channel,
        .rx_gain_db = rx_gain,
        .agc_mode = agc_mode,
        .configure_tx = true,
        .tx_atten_db = tx_atten,
        .bandwidth_mhz = bw_mhz,
        .skip_calibration = false,
    };
    if (deimos_radio_init(&radio_cfg) != 0) {
        fprintf(stderr, "ERROR: deimos_radio_init failed\n");
        return 1;
    }

    uint64_t freq = deimos_rx_channel_to_freq(channel);
    fprintf(stderr, "Fabric loopback: ch=%d, freq=%llu Hz, bw=%d MHz, atten=%.1f dB, "
            "gain=%.1f dB, trials=%d, payload=%d bytes\n",
            channel, (unsigned long long)freq, bw_mhz, tx_atten, rx_gain,
            n_trials, payload_len);

    /* Create FFT plan for TX generation */
    lib80211_fft_plan *plan = lib80211_fft_plan_create();
    if (!plan) {
        fprintf(stderr, "ERROR: fft_plan_create failed\n");
        deimos_rx_cleanup();
        return 1;
    }

    /* Warm-up: TX silence to flush DAC and settle DC offset */
    {
        float warmup[4096] = {0};
        dma_tx_start(warmup, warmup, 4096, false);
        usleep(5000);
        dma_tx_stop();
    }

    /* Set snap observation mode */
    hal_reg_write(REG_DEIMOS_SNAP_MODE, (uint32_t)snap_mode);
    if (snap_mode != 0) {
        fprintf(stderr, "Snap mode: %d (%s)\n", snap_mode,
                snap_mode == 1 ? "ltf_peak metric" :
                snap_mode == 2 ? "IQ" :
                snap_mode == 3 ? "debug" : "unknown");
    }

    /* Determine which rates to test */
    int test_rates[N_RATES];
    int n_test_rates = 0;
    if (single_rate > 0) {
        test_rates[0] = single_rate;
        n_test_rates = 1;
    } else {
        for (int i = 0; i < N_RATES; i++)
            test_rates[i] = RATES[i];
        n_test_rates = N_RATES;
    }

    /* Results */
    typedef struct {
        int rate;
        int pass, fail, timeout, error;
    } result_t;
    result_t results[N_RATES] = {0};
    int rates_passed = 0;

    for (int r = 0; r < n_test_rates; r++) {
        int rate = test_rates[r];
        results[r].rate = rate;

        fprintf(stderr, "  Rate %2d Mbps: ", rate);

        for (int t = 0; t < n_trials; t++) {
            int rc = run_one_trial(plan, psdu, psdu_len, rate, verbose);
            switch (rc) {
            case 1:  results[r].pass++; break;
            case 0:  results[r].fail++; break;
            case -1: results[r].timeout++; break;
            case -2: results[r].error++; break;
            }
            /* Brief delay between trials. The next trial's disable phase
             * (12ms) handles full pipeline quiesce, so this is just for
             * DMA cleanup and DAC settling. */
            usleep(5000);
        }

        int pct = (results[r].pass * 100) / n_trials;
        bool rate_pass = (results[r].pass == n_trials);
        if (rate_pass) {
            fprintf(stderr, "%d/%d (100%%) PASS\n", results[r].pass, n_trials);
            rates_passed++;
        } else {
            fprintf(stderr, "%d/%d (%d%%)", results[r].pass, n_trials, pct);
            if (results[r].fail > 0)
                fprintf(stderr, " fail=%d", results[r].fail);
            if (results[r].timeout > 0)
                fprintf(stderr, " timeout=%d", results[r].timeout);
            if (results[r].error > 0)
                fprintf(stderr, " error=%d", results[r].error);
            fprintf(stderr, " FAIL\n");
        }
    }

    lib80211_fft_plan_destroy(plan);

    /* Clean up */
    deimos_rx_cleanup();

    /* JSON output */
    printf("{\"rates\":[");
    for (int i = 0; i < n_test_rates; i++) {
        if (i > 0) printf(",");
        printf("{\"rate\":%d,\"trials\":%d,\"pass\":%d,\"fail\":%d,"
               "\"timeout\":%d,\"error\":%d,\"pct\":%d}",
               results[i].rate, n_trials, results[i].pass,
               results[i].fail, results[i].timeout, results[i].error,
               (results[i].pass * 100) / n_trials);
    }
    printf("],\"summary\":{\"rates_passed\":%d,\"rates_total\":%d,\"trials\":%d}}\n",
           rates_passed, n_test_rates, n_trials);

    fprintf(stderr, "\nResult: %d/%d rates passed (%d trials each)\n",
            rates_passed, n_test_rates, n_trials);

    return (rates_passed == n_test_rates) ? 0 : 1;
}
