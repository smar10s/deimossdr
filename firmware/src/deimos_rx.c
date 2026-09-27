// SPDX-License-Identifier: MIT
/*
 * deimos_rx.c — Unified radio + pipeline lifecycle implementation
 *
 * Three primitives: radio_init, pipeline_arm, stf_gate
 * Plus tag/PSDU polling and hot reconfigure.
 */

#include "deimos_rx.h"
#include "hal.h"
#include "dma_tx.h"
#include <unistd.h>
#include <string.h>
#include <stdio.h>

/* --------------------------------------------------------------------------
 * Timing constants (microseconds)
 * -------------------------------------------------------------------------- */

#define DEIMOS_PLL_SETTLE_US     100000  /* PLL lock + settling after init */
#define DEIMOS_WATCHDOG_HOLD_US   12000  /* covers 2^19-clock watchdog (5.24 ms @ 100 MHz) + margin */
#define DEIMOS_RETUNE_SETTLE_US   50000  /* LO switch settling */
#define DEIMOS_REG_SETTLE_US       1000  /* short settle / FIFO flush */

/* --------------------------------------------------------------------------
 * Frame classification thresholds (802.11)
 * -------------------------------------------------------------------------- */

#define DEIMOS_ACK_MAX_LEN         14   /* ACK frame length */
#define DEIMOS_ACK_CTRL_LEN        20   /* block-ACK-request length at legacy rates */
#define DEIMOS_BA_MIN_LEN          28   /* block ACK total length (with MAC header) */
#define DEIMOS_BA_MAX_LEN          32
#define DEIMOS_HT_VHT_LSIG_LEN     23   /* HT/VHT length fingerprint at rate 6 */
#define DEIMOS_HT_VHT_SYM_MIN      18   /* 3-symbol OFDM boundary range */
#define DEIMOS_HT_VHT_SYM_MAX      66
#define DEIMOS_HT_VHT_SYM_MOD       3
#define DEIMOS_BEACON_MIN_LEN     300
#define DEIMOS_BEACON_MAX_LEN     500
#define DEIMOS_MGMT_MIN_LEN        50
#define DEIMOS_MGMT_MAX_LEN       299

/* --------------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------------- */

static deimos_radio_config_t    g_radio_cfg;
static deimos_pipeline_config_t g_pipeline_cfg;
static bool g_initialized = false;

/* --------------------------------------------------------------------------
 * Utilities
 * -------------------------------------------------------------------------- */

int deimos_rx_rate_mbps(uint8_t code)
{
    switch (code) {
    case 0x0B: return 6;
    case 0x0F: return 9;
    case 0x0A: return 12;
    case 0x0E: return 18;
    case 0x09: return 24;
    case 0x0D: return 36;
    case 0x08: return 48;
    case 0x0C: return 54;
    default:   return 0;
    }
}

uint64_t deimos_rx_channel_to_freq(int ch)
{
    if (ch >= 1 && ch <= 14) {
        if (ch == 14) return 2484000000ULL;
        return (2412 + (ch - 1) * 5) * 1000000ULL;
    }
    return (5000 + ch * 5) * 1000000ULL;
}

/* --------------------------------------------------------------------------
 * Core Primitive 1: Radio Init
 *
 * AD9361 write order: sample_rate → TX_LO → RX_LO → TX_BW → RX_BW →
 *   gain_mode → gain → tx_atten → PLL settle → calibration
 *
 * TX before RX keeps both PLLs tracking (empirically validated in
 * deimos_fabric_loopback which has 100% cable decode rate).
 * -------------------------------------------------------------------------- */

int deimos_radio_init(const deimos_radio_config_t *cfg)
{
    if (hal_init() != 0)
        return -1;

    memcpy(&g_radio_cfg, cfg, sizeof(g_radio_cfg));

    uint64_t freq = deimos_rx_channel_to_freq(cfg->channel);
    uint64_t bw_hz = cfg->bandwidth_mhz > 0
        ? (uint64_t)cfg->bandwidth_mhz * 1000000ULL
        : 28000000ULL;

    /* Sample rate first — changes clock tree */
    hal_ad9361_set_sample_rate(20000000ULL);

    /* TX LO before RX LO — keeps both PLLs tracking */
    if (cfg->configure_tx) {
        hal_ad9361_set_tx_lo(freq);
        hal_ad9361_set_tx_bandwidth(bw_hz);
    }

    hal_ad9361_set_rx_lo(freq);
    hal_ad9361_set_rx_bandwidth(bw_hz);

    /* AGC / gain mode */
    switch (cfg->agc_mode) {
    case DEIMOS_AGC_FAST_ATTACK:
        hal_ad9361_set_rx_gain_mode("fast_attack");
        break;
    case DEIMOS_AGC_SLOW_ATTACK:
        hal_ad9361_set_rx_gain_mode("slow_attack");
        break;
    case DEIMOS_AGC_MANUAL:
    default:
        hal_ad9361_set_rx_gain_mode("manual");
        hal_ad9361_set_rx_gain(cfg->rx_gain_db);
        break;
    }

    if (cfg->configure_tx) {
        hal_ad9361_set_tx_attenuation(cfg->tx_atten_db);
    }

    /* PLL lock + settling */
    usleep(DEIMOS_PLL_SETTLE_US);

    /* AD9361 calibration — corrects DC offset, IQ imbalance, LO leakage.
     * Cold boot can leave RX cal in a poor state. This toggle fixes it.
     * Takes ~200ms but prevents 50%→12% FCS degradation from bad cal. */
    if (!cfg->skip_calibration) {
        hal_ad9361_run_calibration();
    }

    g_initialized = true;
    return 0;
}

/* --------------------------------------------------------------------------
 * Core Primitive 2: Pipeline Arm
 *
 * Disables STF, configures all pipeline registers, flushes tag FIFO.
 * On exit: pipeline is fully configured, STF is DISABLED.
 *
 * The 12ms hold covers the watchdog period (2^19 clocks = 5.24ms @ 100MHz)
 * plus safety margin, ensuring all stf_detect accumulators are zeroed and
 * any in-progress decode has completed via watchdog timeout.
 * -------------------------------------------------------------------------- */

void deimos_pipeline_arm(const deimos_pipeline_config_t *cfg)
{
    memcpy(&g_pipeline_cfg, cfg, sizeof(g_pipeline_cfg));

    /* Set mode: test_mode for HIL, 0 for live ADC */
    hal_reg_write(REG_HIL_CTRL_CONTROL,
                  cfg->mode == DEIMOS_RX_MODE_HIL ? HIL_CTRL_TEST_MODE : 0);

    /* Disable STF — zeros accumulators, forces the decode pipeline to IDLE */
    hal_reg_write(REG_DEIMOS_STF_ENABLE, 0);
    usleep(DEIMOS_WATCHDOG_HOLD_US);

    /* Pipeline timing register (safe to write while STF disabled) */
    hal_reg_write(REG_DEIMOS_STF_THRESH, cfg->stf_threshold);

    /* NOTE: cfg->stf_skip / ltf_skip / cfo_thresh are no longer written.
     * Their old register addresses (0x10/0x14/0x08) are read-only reserved
     * diagnostics, so those writes were silently ignored. Hardware uses
     * fixed acquisition timing (ltf_skip is "(unused in new architecture)"
     * in rx_pipeline.v). The struct fields remain for source compatibility. */

    /* Flush tag FIFO — discard any stale tags from prior decode */
    hal_reg_write(REG_TAG_FIFO_CONTROL, TAG_CTRL_FLUSH);
    usleep(DEIMOS_REG_SETTLE_US);
    hal_reg_write(REG_TAG_FIFO_CONTROL, 0);
}

/* --------------------------------------------------------------------------
 * Core Primitive 3: STF Gate
 *
 * Enables STF detection and fires the trigger in back-to-back writes.
 * No delay between STF enable and trigger — the SILENCE_PAD in waveforms
 * provides the correlator fill time.
 * -------------------------------------------------------------------------- */

int deimos_stf_gate(deimos_trigger_t trigger)
{
    /* Enable STF — starts correlator accumulation */
    hal_reg_write(REG_DEIMOS_STF_ENABLE, 1);

    /* Fire stimulus immediately */
    switch (trigger) {
    case DEIMOS_TRIGGER_NONE:
        /* OTA / passive — just go live */
        break;

    case DEIMOS_TRIGGER_DMA_TX:
        /* Cable loopback — fire DMA TX */
        if (dma_tx_trigger() != 0) {
            fprintf(stderr, "deimos_rx: dma_tx_trigger failed\n");
            return -1;
        }
        break;

    case DEIMOS_TRIGGER_HIL:
        /* HIL inject — set trigger bit (test_mode already set by arm) */
        hal_reg_write(REG_HIL_CTRL_CONTROL,
                      HIL_CTRL_TEST_MODE | HIL_CTRL_TRIGGER);
        break;
    }

    return 0;
}

/* --------------------------------------------------------------------------
 * Convenience: Full init (radio + arm + gate)
 * -------------------------------------------------------------------------- */

int deimos_rx_start(const deimos_radio_config_t *radio,
                    const deimos_pipeline_config_t *pipeline)
{
    if (deimos_radio_init(radio) != 0)
        return -1;
    deimos_pipeline_arm(pipeline);
    return deimos_stf_gate(DEIMOS_TRIGGER_NONE);
}

/* --------------------------------------------------------------------------
 * Lifecycle
 * -------------------------------------------------------------------------- */

void deimos_rx_cleanup(void)
{
    if (g_initialized) {
        hal_reg_write(REG_DEIMOS_STF_ENABLE, 0);
        hal_cleanup();
        g_initialized = false;
    }
}

/* --------------------------------------------------------------------------
 * Hot reconfigure
 * -------------------------------------------------------------------------- */

int deimos_rx_set_freq(uint64_t freq_hz)
{
    if (!g_initialized) return -1;

    uint64_t freq_mhz = freq_hz / 1000000;
    int channel;
    if (freq_mhz >= 2412 && freq_mhz <= 2484)
        channel = (int)(freq_mhz - 2412) / 5 + 1;
    else
        channel = (int)(freq_mhz - 5000) / 5;
    g_radio_cfg.channel = channel;

    hal_reg_write(REG_DEIMOS_STF_ENABLE, 0);
    if (g_radio_cfg.configure_tx)
        hal_ad9361_set_tx_lo(freq_hz);
    hal_ad9361_set_rx_lo(freq_hz);
    usleep(DEIMOS_RETUNE_SETTLE_US);

    hal_reg_write(REG_DEIMOS_STF_ENABLE, 1);
    usleep(DEIMOS_REG_SETTLE_US);
    deimos_rx_flush();
    return 0;
}

int deimos_rx_set_channel(int channel)
{
    if (!g_initialized) return -1;

    g_radio_cfg.channel = channel;
    uint64_t freq = deimos_rx_channel_to_freq(channel);

    /* Disable STF during retune — LO change causes ADC garbage */
    hal_reg_write(REG_DEIMOS_STF_ENABLE, 0);
    if (g_radio_cfg.configure_tx)
        hal_ad9361_set_tx_lo(freq);
    hal_ad9361_set_rx_lo(freq);
    usleep(DEIMOS_RETUNE_SETTLE_US);

    hal_reg_write(REG_DEIMOS_STF_ENABLE, 1);
    usleep(DEIMOS_REG_SETTLE_US);
    deimos_rx_flush();
    return 0;
}

int deimos_rx_set_bandwidth(int bandwidth_mhz)
{
    if (!g_initialized) return -1;

    uint64_t bw_hz = bandwidth_mhz > 0
        ? (uint64_t)bandwidth_mhz * 1000000ULL
        : 28000000ULL;
    g_radio_cfg.bandwidth_mhz = bandwidth_mhz;

    hal_reg_write(REG_DEIMOS_STF_ENABLE, 0);
    if (g_radio_cfg.configure_tx)
        hal_ad9361_set_tx_bandwidth(bw_hz);
    hal_ad9361_set_rx_bandwidth(bw_hz);
    usleep(DEIMOS_RETUNE_SETTLE_US);
    hal_reg_write(REG_DEIMOS_STF_ENABLE, 1);
    usleep(DEIMOS_REG_SETTLE_US);
    deimos_rx_flush();
    return 0;
}

int deimos_rx_set_gain(double rx_gain_db)
{
    if (!g_initialized) return -1;

    g_radio_cfg.rx_gain_db = rx_gain_db;
    hal_ad9361_set_rx_gain(rx_gain_db);

    /* Large gain steps saturate ADC → corrupt accumulators.
     * Full reset with 12ms hold covers watchdog drain. */
    hal_reg_write(REG_DEIMOS_STF_ENABLE, 0);
    usleep(DEIMOS_WATCHDOG_HOLD_US);
    hal_reg_write(REG_DEIMOS_STF_ENABLE, 1);
    usleep(DEIMOS_REG_SETTLE_US);
    deimos_rx_flush();
    return 0;
}

int deimos_rx_set_agc_mode(deimos_agc_mode_t mode)
{
    if (!g_initialized) return -1;

    switch (mode) {
    case DEIMOS_AGC_FAST_ATTACK:
        hal_ad9361_set_rx_gain_mode("fast_attack");
        break;
    case DEIMOS_AGC_SLOW_ATTACK:
        hal_ad9361_set_rx_gain_mode("slow_attack");
        break;
    case DEIMOS_AGC_MANUAL:
    default:
        hal_ad9361_set_rx_gain_mode("manual");
        hal_ad9361_set_rx_gain(g_radio_cfg.rx_gain_db);
        break;
    }
    g_radio_cfg.agc_mode = mode;

    /* Gain mode changes can cause ADC saturation transients.
     * Reset STF with 12ms hold covers watchdog drain. */
    hal_reg_write(REG_DEIMOS_STF_ENABLE, 0);
    usleep(DEIMOS_WATCHDOG_HOLD_US);
    hal_reg_write(REG_DEIMOS_STF_ENABLE, 1);
    usleep(DEIMOS_REG_SETTLE_US);
    deimos_rx_flush();
    return 0;
}

int deimos_rx_set_threshold(int stf_threshold)
{
    if (!g_initialized) return -1;

    g_pipeline_cfg.stf_threshold = stf_threshold;
    hal_reg_write(REG_DEIMOS_STF_THRESH, stf_threshold);
    return 0;
}

void deimos_rx_flush(void)
{
    hal_reg_write(REG_TAG_FIFO_CONTROL, TAG_CTRL_FLUSH);
    usleep(DEIMOS_REG_SETTLE_US);
    hal_reg_write(REG_TAG_FIFO_CONTROL, 0);
}

/* --------------------------------------------------------------------------
 * Tag/PSDU polling
 * -------------------------------------------------------------------------- */

int deimos_rx_poll(deimos_rx_frame_t *frame)
{
    if (!g_initialized) return -1;

    uint32_t status = hal_reg_read(REG_TAG_FIFO_STATUS);
    if (status & TAG_STATUS_EMPTY)
        return 0;  /* no frame available */

    /* Read TAG_LO (peek — no pop) */
    uint32_t lo = hal_reg_read(REG_TAG_FIFO_TAG_LO);
    /* Read the shared good-frame snapshot NOW, before TAG_HI pops and before
     * the PSDU readback. The snapshot register tracks the newest tag; reading
     * it here (1-2 AXI reads after STATUS) keeps it equal to this frame's
     * value, whereas reading it after the PSDU readback lets later-arriving
     * frames (SIFS bursts) overwrite it — the v1 A/B misattribution bug. */
    frame->phase_inc = DIAG_TAG_PHASE(hal_reg_read(REG_DEIMOS_DIAG_TAG_CTX));
    /* Read TAG_HI (pops tag + sets BRAM cursor) */
    uint32_t hi = hal_reg_read(REG_TAG_FIFO_TAG_HI);

    frame->fcs_ok    = TAG_LO_FCS_OK(lo) != 0;
    frame->rate_code = (uint8_t)TAG_LO_RATE(lo);
    frame->rate_mbps = deimos_rx_rate_mbps(frame->rate_code);
    frame->length    = TAG_LO_LENGTH(lo);
    frame->psdu_addr = TAG_HI_PSDU_ADDR(hi);

    /* Read PSDU bytes from BRAM cursor.
     * Only for FCS-OK frames — FCS-fail bytes are unreliable.
     * Must complete before next TAG_HI read (next pop resets cursor). */
    if (frame->fcs_ok && frame->length > 4) {
        uint16_t nbytes = frame->length - 4;
        if (nbytes > DEIMOS_RX_MAX_PSDU)
            nbytes = DEIMOS_RX_MAX_PSDU;
        for (uint16_t i = 0; i < nbytes; i++) {
            uint32_t d = hal_reg_read(REG_PSDU_DATA);
            frame->psdu[i] = (uint8_t)(d & 0xFF);
        }
        frame->psdu_len = nbytes;
    } else {
        frame->psdu_len = 0;
    }

    return 1;  /* frame available */
}

uint32_t deimos_rx_frame_count(void)
{
    return hal_reg_read(REG_TAG_FIFO_FRAME_CNT);
}

uint32_t deimos_rx_drop_count(void)
{
    return hal_reg_read(REG_TAG_FIFO_DROP_CNT);
}

/* --------------------------------------------------------------------------
 * Frame classification
 * -------------------------------------------------------------------------- */

deimos_frame_class_t deimos_rx_classify_frame(int rate_mbps, uint32_t length,
                                              bool fcs_ok)
{
    /*
     * Priority 1: ACK — very short frames or 20-byte control at legacy rates.
     * ACK frames are 14 bytes. Block ACK request can be 20 bytes.
     */
    if (length <= DEIMOS_ACK_MAX_LEN)
        return DEIMOS_FCLASS_ACK;
    if (length == DEIMOS_ACK_CTRL_LEN &&
        (rate_mbps == 6 || rate_mbps == 12 || rate_mbps == 24))
        return DEIMOS_FCLASS_ACK;

    /*
     * Priority 2: Block ACK — mid-size control at OFDM rates 12/24 Mbps.
     * Block ACK frame body is 16-20 bytes (BA control + BA info),
     * giving 28-32 total with MAC header.
     */
    if ((rate_mbps == 12 || rate_mbps == 24) &&
        length >= DEIMOS_BA_MIN_LEN && length <= DEIMOS_BA_MAX_LEN)
        return DEIMOS_FCLASS_BLOCK_ACK;

    /*
     * Priority 3: FCS-failed frames. These are typically HT/VHT frames
     * misinterpreted as legacy — the L-SIG rate field maps to a different
     * constellation in HT/VHT, producing FCS failures.
     */
    if (!fcs_ok) {
        if (rate_mbps == 6) {
            /* Rate 6 Mbps with FCS fail: the L-SIG length of HT/VHT
             * frames decodes to specific byte-length patterns.
             *   - 23: a specific HT/VHT frame-length fingerprint
             *   - Multiples of 3 in [18,66]: 3-symbol OFDM boundaries
             *     that correspond to common HT/VHT PSDU lengths. */
            if (length == DEIMOS_HT_VHT_LSIG_LEN)
                return DEIMOS_FCLASS_HT_VHT;
            if (length >= DEIMOS_HT_VHT_SYM_MIN &&
                length <= DEIMOS_HT_VHT_SYM_MAX &&
                length % DEIMOS_HT_VHT_SYM_MOD == 0)
                return DEIMOS_FCLASS_HT_VHT;
        }

        /* Rates 9, 18, 36, 48, 54 Mbps never appear in legacy control
         * frames — any FCS failure at these rates is overwhelmingly
         * likely an HT/VHT frame decoded with the wrong constellation. */
        if (rate_mbps == 9 || rate_mbps == 18 || rate_mbps == 36 ||
            rate_mbps == 48 || rate_mbps == 54)
            return DEIMOS_FCLASS_HT_VHT;

        /*
         * Other rate×length combinations (rate 12 with non-20-byte
         * frames, rate 24 outside [28,32], rate 0, etc.) have no
         * reliable heuristic — mark unknown.
         */
        return DEIMOS_FCLASS_UNKNOWN;
    }

    /* Priority 4: FCS-OK frames — classify by legacy rate=6 length ranges. */
    if (rate_mbps == 6) {
        if (length == DEIMOS_HT_VHT_LSIG_LEN)
            return DEIMOS_FCLASS_HT_VHT;
        if (length >= DEIMOS_BEACON_MIN_LEN && length <= DEIMOS_BEACON_MAX_LEN)
            return DEIMOS_FCLASS_BEACON;
        if (length >= DEIMOS_MGMT_MIN_LEN && length <= DEIMOS_MGMT_MAX_LEN)
            return DEIMOS_FCLASS_MGMT;
        if (length > DEIMOS_BEACON_MAX_LEN)
            return DEIMOS_FCLASS_PROBE;
    }

    /* All other FCS-OK frames (rates != 6, short rate-6 data, etc.). */
    return DEIMOS_FCLASS_DATA;
}

/* --------------------------------------------------------------------------
 * 802.11 header parser
 * -------------------------------------------------------------------------- */

void deimos_rx_parse_header(const uint8_t *psdu, size_t psdu_len,
                            deimos_rx_meta_t *meta)
{
    memset(meta, 0, sizeof(*meta));

    if (psdu_len < 24)
        return;

    uint8_t fc0 = psdu[0];
    uint8_t type = (fc0 >> 2) & 0x03;
    uint8_t subtype = (fc0 >> 4) & 0x0F;

    if (type == 0) {
        memcpy(meta->da, psdu + 4, 6);
        memcpy(meta->sa, psdu + 10, 6);
        memcpy(meta->bssid, psdu + 16, 6);
        meta->has_da = true;
        meta->has_sa = true;
        meta->has_bssid = true;

        if ((subtype == 8 || subtype == 5) && psdu_len > 36) {
            size_t offset = 36;
            while (offset + 2 <= psdu_len) {
                uint8_t tag_id = psdu[offset];
                uint8_t tag_len = psdu[offset + 1];
                if (offset + 2 + tag_len > psdu_len)
                    break;
                if (tag_id == 0) {
                    size_t copy_len = tag_len > 32 ? 32 : tag_len;
                    memcpy(meta->ssid, psdu + offset + 2, copy_len);
                    meta->ssid[copy_len] = '\0';
                    meta->has_ssid = true;
                    break;
                }
                offset += 2 + tag_len;
            }
        }
    } else if (type == 2) {
        uint8_t fc1 = psdu[1];
        uint8_t to_ds = (fc1 >> 0) & 0x01;
        uint8_t from_ds = (fc1 >> 1) & 0x01;

        if (!to_ds && !from_ds) {
            memcpy(meta->da, psdu + 4, 6);
            memcpy(meta->sa, psdu + 10, 6);
            memcpy(meta->bssid, psdu + 16, 6);
        } else if (to_ds && !from_ds) {
            memcpy(meta->bssid, psdu + 4, 6);
            memcpy(meta->sa, psdu + 10, 6);
            memcpy(meta->da, psdu + 16, 6);
        } else if (!to_ds && from_ds) {
            memcpy(meta->da, psdu + 4, 6);
            memcpy(meta->bssid, psdu + 10, 6);
            memcpy(meta->sa, psdu + 16, 6);
        } else {
            memcpy(meta->da, psdu + 16, 6);
            if (psdu_len >= 30) {
                memcpy(meta->sa, psdu + 22, 6);
                meta->has_sa = true;
            }
            memcpy(meta->bssid, psdu + 10, 6);
        }
        meta->has_da = true;
        if (!(to_ds && from_ds)) meta->has_sa = true;
        meta->has_bssid = true;

        size_t hdr_len = (!to_ds && !from_ds) ? 24 :
                         (to_ds && from_ds) ? 30 : 24;
        if (subtype & 0x08)
            hdr_len += 2;
        if (psdu_len >= hdr_len + 8) {
            const uint8_t *llc = psdu + hdr_len;
            if (llc[0] == 0xAA && llc[1] == 0xAA && llc[2] == 0x03 &&
                llc[3] == 0x00 && llc[4] == 0x00 && llc[5] == 0x00) {
                uint16_t ethertype = ((uint16_t)llc[6] << 8) | llc[7];
                if (ethertype == 0x888E)
                    meta->is_eapol = true;
            }
        }
    }
}

bool deimos_rx_should_parse(deimos_decode_mode_t mode,
                             uint8_t rate_code, uint32_t length)
{
    if (mode == DEIMOS_DECODE_NONE)
        return false;
    if (mode == DEIMOS_DECODE_EAPOL) {
        int r = deimos_rx_rate_mbps(rate_code);
        if (r != 6 && r != 24)
            return false;
        if (length < 50 || length > 500)
            return false;
    }
    return true;
}

/* --------------------------------------------------------------------------
 * IE summary extraction (beacons and probe responses only)
 *
 * Walks the IE chain starting at PSDU offset 36 (after beacon fixed fields:
 * FC(2) + Duration(2) + DA(6) + SA(6) + BSSID(6) + SeqCtrl(2) +
 * Timestamp(8) + Beacon Interval(2) + Capability(2) = 36).
 *
 * Extracts: DS Parameter (ch), RSN (security + cipher), HT Caps,
 * VHT Caps, Supported Rates (max), Country, first Vendor OUI.
 * -------------------------------------------------------------------------- */

void deimos_rx_parse_ies(const uint8_t *psdu, size_t psdu_len,
                         deimos_ie_summary_t *ies)
{
    memset(ies, 0, sizeof(*ies));

    if (psdu_len < 38)
        return;

    /* Verify this is a beacon (subtype 8) or probe response (subtype 5) */
    uint8_t fc0 = psdu[0];
    uint8_t type = (fc0 >> 2) & 0x03;
    uint8_t subtype = (fc0 >> 4) & 0x0F;
    if (type != 0 || (subtype != 8 && subtype != 5))
        return;

    size_t pos = 36;
    bool found_rsn = false;
    bool found_vendor = false;

    while (pos + 2 <= psdu_len) {
        uint8_t tag_id  = psdu[pos];
        uint8_t tag_len = psdu[pos + 1];
        size_t  tag_end = pos + 2 + tag_len;
        if (tag_end > psdu_len)
            break;

        const uint8_t *data = psdu + pos + 2;

        switch (tag_id) {
        case 1:  /* Supported Rates */
            for (int i = 0; i < tag_len; i++) {
                uint8_t rate = data[i] & 0x7F;  /* strip "basic" bit */
                if (rate > ies->max_rate)
                    ies->max_rate = rate;
            }
            break;

        case 3:  /* DS Parameter Set */
            if (tag_len >= 1)
                ies->channel = data[0];
            break;

        case 7:  /* Country */
            if (tag_len >= 2) {
                ies->country[0] = data[0];
                ies->country[1] = data[1];
            }
            break;

        case 45: /* HT Capabilities (26 bytes) */
            if (tag_len >= 2) {
                uint16_t ht_cap = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
                ies->phy_flags |= DEIMOS_PHY_HT;
                if (ht_cap & 0x0002)  /* Supported Channel Width Set */
                    ies->phy_flags |= DEIMOS_PHY_HT_40MHZ;
            }
            break;

        case 48: /* RSN (variable) */
            if (!found_rsn && tag_len >= 8) {
                found_rsn = true;
                /* version(2) + group_oui(3) + group_type(1) = offset 5 for type */
                uint8_t group_type = data[5];
                if (group_type == 4)      ies->cipher = DEIMOS_CIPHER_CCMP;
                else if (group_type == 2) ies->cipher = DEIMOS_CIPHER_TKIP;
                else if (group_type == 8) ies->cipher = DEIMOS_CIPHER_GCMP256;
                else if (group_type == 1) ies->cipher = DEIMOS_CIPHER_WEP;

                /* Parse pairwise cipher (override group with pairwise if present) */
                if (tag_len >= 10) {
                    uint16_t pw_count = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
                    if (pw_count >= 1 && tag_len >= 12) {
                        uint8_t pw_type = data[11]; /* first pairwise cipher type */
                        if (pw_type == 4)      ies->cipher = DEIMOS_CIPHER_CCMP;
                        else if (pw_type == 2) ies->cipher = DEIMOS_CIPHER_TKIP;
                        else if (pw_type == 8) ies->cipher = DEIMOS_CIPHER_GCMP256;
                    }
                    /* Parse AKM */
                    size_t akm_offset = 8 + (size_t)pw_count * 4;
                    if (akm_offset + 2 <= (size_t)tag_len) {
                        uint16_t akm_count = (uint16_t)data[akm_offset]
                            | ((uint16_t)data[akm_offset + 1] << 8);
                        if (akm_count >= 1 && akm_offset + 6 <= (size_t)tag_len) {
                            uint8_t akm_type = data[akm_offset + 5];
                            if (akm_type == 2)      ies->security = DEIMOS_SEC_WPA2_PSK;
                            else if (akm_type == 1) ies->security = DEIMOS_SEC_WPA2_EAP;
                            else if (akm_type == 8) ies->security = DEIMOS_SEC_WPA3_SAE;
                            else if (akm_type == 6) ies->security = DEIMOS_SEC_WPA2_PSK;
                            if (akm_count > 1)
                                ies->security = DEIMOS_SEC_MIXED;
                        }
                    }
                }
            }
            break;

        case 50: /* Extended Supported Rates */
            for (int i = 0; i < tag_len; i++) {
                uint8_t rate = data[i] & 0x7F;
                if (rate > ies->max_rate)
                    ies->max_rate = rate;
            }
            break;

        case 191: /* VHT Capabilities (12 bytes) */
            if (tag_len >= 4) {
                uint32_t vht_cap = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                    | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
                ies->phy_flags |= DEIMOS_PHY_VHT;
                /* All VHT supports 80 MHz (mandatory) */
                ies->phy_flags |= DEIMOS_PHY_VHT_80MHZ;
                uint8_t chan_width = (vht_cap >> 2) & 0x03;
                if (chan_width >= 1)
                    ies->phy_flags |= DEIMOS_PHY_VHT_160;
            }
            break;

        case 221: /* Vendor Specific */
            if (!found_vendor && tag_len >= 3) {
                found_vendor = true;
                ies->vendor_oui[0] = data[0];
                ies->vendor_oui[1] = data[1];
                ies->vendor_oui[2] = data[2];
            }
            break;
        }

        pos = tag_end;
    }

    /* If no RSN IE but Capability has Privacy bit, it's WEP */
    if (!found_rsn && psdu_len >= 36) {
        uint16_t capability = (uint16_t)psdu[34] | ((uint16_t)psdu[35] << 8);
        if (capability & 0x0010) {  /* Privacy bit */
            ies->security = DEIMOS_SEC_WEP;
            ies->cipher = DEIMOS_CIPHER_WEP;
        }
    }

    ies->has_ie_summary = true;
}
