// SPDX-License-Identifier: MIT
/*
 * deimos_rx.h — Unified radio + pipeline lifecycle API
 *
 * Three composable primitives:
 *   1. deimos_radio_init()       — AD9361 config (frequency, gain, bandwidth, calibration)
 *   2. deimos_pipeline_arm()     — configure pipeline registers, STF disabled at exit
 *   3. deimos_stf_gate()         — enable STF + optional trigger (atomic)
 *
 * The discipline: between arm() and gate(), the tool does all slow work
 * (waveform generation, DMA load, DDR writes). STF is disabled during that
 * window so no noise/silence can accumulate. gate() enables STF and fires
 * the stimulus in back-to-back register writes.
 *
 * Additionally provides:
 *   - Non-blocking tag poll with automatic PSDU BRAM readback
 *   - Hot reconfigure (channel/gain/threshold) with proper STF cycling
 *
 * Used by all firmware tools. Links against hal (register access + AD9361 sysfs).
 */

#ifndef DEIMOS_RX_H
#define DEIMOS_RX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DEIMOS_RX_MAX_PSDU  1500

/* ============================================================================
 * Pipeline mode
 * ============================================================================ */

typedef enum {
    DEIMOS_RX_MODE_LIVE = 0,   /* ADC -> fabric (OTA or cable loopback) */
    DEIMOS_RX_MODE_HIL,        /* HIL inject -> fabric (digital, no RF) */
} deimos_rx_mode_t;

/* ============================================================================
 * Trigger type for deimos_stf_gate()
 * ============================================================================ */

typedef enum {
    DEIMOS_TRIGGER_NONE = 0,   /* Just enable STF (OTA / passive receive) */
    DEIMOS_TRIGGER_DMA_TX,     /* Enable STF then dma_tx_trigger() */
    DEIMOS_TRIGGER_HIL,        /* Enable STF then HIL_CTRL |= TRIGGER */
} deimos_trigger_t;

/* ============================================================================
 * AGC mode
 * ============================================================================ */

typedef enum {
    DEIMOS_AGC_MANUAL = 0,     /* Fixed gain (cable loopback, controlled tests) */
    DEIMOS_AGC_FAST_ATTACK,    /* Fast-attack AGC (OTA, multi-transmitter) */
    DEIMOS_AGC_SLOW_ATTACK,    /* Slow-attack AGC (sustained single-transmitter) */
} deimos_agc_mode_t;

/* ============================================================================
 * Configuration structures
 * ============================================================================ */

/* Radio configuration */
typedef struct {
    int              channel;        /* WiFi channel number (1-14, 36-165) */
    double           rx_gain_db;     /* RX gain in dB (used when agc=MANUAL) */
    deimos_agc_mode_t agc_mode;     /* AGC mode selection */
    /* TX fields (only used when configure_tx is true) */
    bool             configure_tx;   /* also set TX LO/BW/attenuation */
    double           tx_atten_db;    /* TX attenuation in dB (e.g. 3.0 for cable) */
    /* Bandwidth override (0 = default 28 MHz) */
    int              bandwidth_mhz;  /* analog filter bandwidth, 0 = 28 */
    /* Calibration control */
    bool             skip_calibration; /* skip AD9361 recal (for rapid retune) */
} deimos_radio_config_t;

/* Pipeline configuration */
typedef struct {
    deimos_rx_mode_t mode;           /* live vs HIL */
    int              stf_threshold;  /* STF threshold shift 0-7. 0 = standard
                                      * 802.11 threshold 0.36 (most strict);
                                      * each step doubles sensitivity
                                      * (1 = 2x, 2 = 4x, ...). See stf_detect.v */
    int              stf_skip;       /* DEPRECATED — no longer written to hardware
                                      * (address was a read-only reserve); kept
                                      * for source compatibility only */
    int              ltf_skip;       /* DEPRECATED — see stf_skip. RTL marks it
                                      * "(unused in new architecture)" */
    int              cfo_thresh;     /* DEPRECATED — see stf_skip */
} deimos_pipeline_config_t;

/* One decoded frame from the fabric */
typedef struct {
    uint8_t     rate_code;      /* 0x0B=6M, 0x0F=9M, ... 0x0C=54M */
    int         rate_mbps;      /* convenience: 6, 9, 12, 18, 24, 36, 48, 54 */
    uint16_t    length;         /* SIGNAL length field (includes 4-byte FCS) */
    bool        fcs_ok;         /* fabric FCS check result */
    uint16_t    psdu_addr;      /* BRAM address (diagnostics) */
    uint8_t     psdu[DEIMOS_RX_MAX_PSDU];  /* decoded bytes (length - 4) */
    uint16_t    psdu_len;       /* actual bytes read (0 if fcs_ok=false) */
} deimos_rx_frame_t;

/* ============================================================================
 * Convenience defaults
 * ============================================================================ */

/* Pipeline config for live cable/OTA (most common case) */
#define DEIMOS_PIPELINE_LIVE_DEFAULT { \
    .mode = DEIMOS_RX_MODE_LIVE, \
    .stf_threshold = 0, \
    .stf_skip = 4, \
    .ltf_skip = 1, \
    .cfo_thresh = 64, \
}

/* Pipeline config for HIL golden vectors */
#define DEIMOS_PIPELINE_HIL_DEFAULT { \
    .mode = DEIMOS_RX_MODE_HIL, \
    .stf_threshold = 0, \
    .stf_skip = 4, \
    .ltf_skip = 192, \
    .cfo_thresh = 1024, \
}

/* ============================================================================
 * Core API: Three Primitives
 * ============================================================================ */

/*
 * Initialize HAL and configure AD9361 radio.
 * Blocks for ~300ms (PLL lock + calibration).
 * In HIL mode, skips all AD9361 writes (no RF needed).
 *
 * AD9361 write order: sample_rate → TX_LO → RX_LO → TX_BW → RX_BW →
 *   gain_mode → gain → tx_atten → PLL settle → calibration
 *
 * Returns 0 on success, -1 on failure.
 */
int deimos_radio_init(const deimos_radio_config_t *cfg);

/*
 * Arm the pipeline: configure all registers with STF DISABLED.
 *
 * Sequence:
 *   1. Set mode (test_mode for HIL, 0 for live)
 *   2. STF_ENABLE = 0 + 12ms settle (accumulator drain)
 *   3. Write all pipeline registers (skip, thresh, cfo)
 *   4. Flush tag FIFO
 *
 * On return, pipeline is fully configured but STF is DISABLED.
 * The caller does slow work (DMA load, waveform gen) then calls
 * deimos_stf_gate() to go live.
 */
void deimos_pipeline_arm(const deimos_pipeline_config_t *cfg);

/*
 * Enable STF detection and optionally trigger a stimulus.
 *
 * This is two back-to-back register writes with no delay:
 *   STF_ENABLE = 1
 *   [trigger DMA TX / HIL inject / nothing]
 *
 * For cable loopback, the SILENCE_PAD at waveform start gives the
 * correlator time to fill before the real STF arrives.
 *
 * Returns 0 on success, -1 on trigger failure.
 */
int deimos_stf_gate(deimos_trigger_t trigger);

/* ============================================================================
 * Convenience: init + arm + gate in one call (OTA / simple cases)
 * ============================================================================ */

/*
 * Full init: radio + arm + gate(NONE). For tools that just want to go live.
 * Equivalent to radio_init() + pipeline_arm() + stf_gate(TRIGGER_NONE).
 * Backward-compatible entry point.
 */
int deimos_rx_start(const deimos_radio_config_t *radio,
                    const deimos_pipeline_config_t *pipeline);

/* ============================================================================
 * Lifecycle
 * ============================================================================ */

/*
 * Shut down: disable STF, release HAL resources.
 */
void deimos_rx_cleanup(void);

/* ============================================================================
 * Hot reconfigure (no full teardown)
 * Each includes STF disable → change → re-enable → flush.
 * ============================================================================ */

int  deimos_rx_set_freq(uint64_t freq_hz);
int  deimos_rx_set_channel(int channel);
int  deimos_rx_set_bandwidth(int bandwidth_mhz);
int  deimos_rx_set_gain(double rx_gain_db);
int  deimos_rx_set_agc_mode(deimos_agc_mode_t mode);
int  deimos_rx_set_threshold(int stf_threshold);
void deimos_rx_flush(void);

/* ============================================================================
 * Tag/PSDU polling
 * ============================================================================ */

/*
 * Poll for next decoded frame. Non-blocking.
 *
 * Returns:
 *    1 = frame available (written to *frame)
 *    0 = no frame (tag FIFO empty)
 *   -1 = error
 */
int deimos_rx_poll(deimos_rx_frame_t *frame);

/* Hardware counters */
uint32_t deimos_rx_frame_count(void);
uint32_t deimos_rx_drop_count(void);

/* ============================================================================
 * Utilities
 * ============================================================================ */

int      deimos_rx_rate_mbps(uint8_t rate_code);
uint64_t deimos_rx_channel_to_freq(int channel);

/* ============================================================================
 * Frame decode mode
 * ============================================================================ */

typedef enum {
    DEIMOS_DECODE_ALL = 0,
    DEIMOS_DECODE_EAPOL,
    DEIMOS_DECODE_NONE,
} deimos_decode_mode_t;

/* ============================================================================
 * Frame classification (CPU-side, based on tag fields alone)
 * ============================================================================ */

typedef enum {
    DEIMOS_FCLASS_ACK = 0,
    DEIMOS_FCLASS_BLOCK_ACK,
    DEIMOS_FCLASS_HT_VHT,
    DEIMOS_FCLASS_BEACON,
    DEIMOS_FCLASS_MGMT,
    DEIMOS_FCLASS_PROBE,
    DEIMOS_FCLASS_DATA,
    DEIMOS_FCLASS_UNKNOWN,
} deimos_frame_class_t;

deimos_frame_class_t deimos_rx_classify_frame(int rate_mbps, uint32_t length,
                                              bool fcs_ok);

/* ============================================================================
 * 802.11 header parser (from PSDU bytes)
 * ============================================================================ */

typedef struct {
    uint8_t  bssid[6];
    uint8_t  sa[6];
    uint8_t  da[6];
    char     ssid[33];
    bool     has_bssid;
    bool     has_sa;
    bool     has_da;
    bool     has_ssid;
    bool     is_eapol;
} deimos_rx_meta_t;

void deimos_rx_parse_header(const uint8_t *psdu, size_t psdu_len,
                             deimos_rx_meta_t *meta);

bool deimos_rx_should_parse(deimos_decode_mode_t mode,
                             uint8_t rate_code, uint32_t length);

/* ============================================================================
 * IE summary extraction (beacons and probe responses)
 * ============================================================================ */

typedef enum {
    DEIMOS_SEC_OPEN = 0,
    DEIMOS_SEC_WEP,
    DEIMOS_SEC_WPA2_PSK,
    DEIMOS_SEC_WPA2_EAP,
    DEIMOS_SEC_WPA3_SAE,
    DEIMOS_SEC_MIXED,       /* multiple AKMs advertised */
} deimos_security_t;

typedef enum {
    DEIMOS_CIPHER_NONE = 0,
    DEIMOS_CIPHER_WEP,
    DEIMOS_CIPHER_TKIP,
    DEIMOS_CIPHER_CCMP,
    DEIMOS_CIPHER_GCMP256,
} deimos_cipher_t;

/* PHY flags (bitfield) */
#define DEIMOS_PHY_HT        0x01  /* HT Capabilities IE present */
#define DEIMOS_PHY_HT_40MHZ  0x02  /* HT supports 40 MHz */
#define DEIMOS_PHY_VHT       0x04  /* VHT Capabilities IE present */
#define DEIMOS_PHY_VHT_80MHZ 0x08  /* VHT supports 80 MHz */
#define DEIMOS_PHY_VHT_160   0x10  /* VHT supports 160 MHz */

typedef struct {
    uint8_t  channel;       /* DS Parameter IE value (0 = absent) */
    uint8_t  security;      /* deimos_security_t */
    uint8_t  cipher;        /* deimos_cipher_t (pairwise) */
    uint8_t  phy_flags;     /* DEIMOS_PHY_* bitfield */
    uint8_t  max_rate;      /* max from Supported Rates IE (0.5 Mbps units) */
    uint8_t  vendor_oui[3]; /* first Vendor Specific IE OUI */
    uint8_t  country[2];    /* Country IE first 2 chars (e.g. 'C','A') */
    bool     has_ie_summary;
} deimos_ie_summary_t;

void deimos_rx_parse_ies(const uint8_t *psdu, size_t psdu_len,
                         deimos_ie_summary_t *ies);

#endif /* DEIMOS_RX_H */
