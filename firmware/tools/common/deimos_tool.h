// SPDX-License-Identifier: MIT
/*
 * deimos_tool.h — Shared scaffolding for deimos firmware tools
 *
 * Extends styx_tool.h with deimos-specific utilities:
 *   - JSON waveform parsing (load_waveform)
 *   - 12-bit DDR quantization (float → packed IQ)
 *   - Frame builders (test PSDU, LLC/SNAP/EAPOL)
 *   - Snap buffer decode macros
 *   - Rate code table
 *
 * All deimos tools link against deimos_tool (which links styx_tool).
 */

#ifndef DEIMOS_TOOL_H
#define DEIMOS_TOOL_H

#include "styx_tool.h"
#include "hal.h"
#include "hal_deimos.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ============================================================================
 * Snap buffer decode macros (fabric tag format)
 *
 * Snap data from decode_engine (mode 0):
 *   {state[4:0], fcs_result_ok, parsed_rate[3:0], tag_fcs_ok, 9'b0, parsed_length[11:0]}
 *    bits 31:27   bit 26        bits 25:22        bit 21      20:12  bits 11:0
 * ============================================================================ */

#define SNAP_STATE(w)       (((w) >> 27) & 0x1F)
#define SNAP_FCS_OK(w)      (((w) >> 26) & 0x01)
#define SNAP_RATE(w)        (((w) >> 22) & 0x0F)
#define SNAP_TAG_FCS(w)     (((w) >> 21) & 0x01)
#define SNAP_LENGTH(w)      ((w) & 0xFFF)

/* decode_engine states of interest */
#define S_IDLE      0
#define S_TAG_OUT   15
#define S_DONE      16

/* Snap probe control bits */
#define SNAP_CTRL_ARM       (1 << 0)
#define SNAP_CTRL_SWTRIG    (1 << 1)
#define SNAP_CTRL_CIRCULAR  (1 << 2)
#define SNAP_DEPTH          1024

/* Samples of silence (at 20 MSPS = 100 us) padded before/after a TX frame.
 * Gives the STF correlator fill time before the first real sample. */
#define DEIMOS_SILENCE_PAD  2000

/* Search the snap buffer backwards from trig_pos for the S_TAG_OUT/S_DONE
 * entry and return its raw word. Falls back to the entry at trig_pos when no
 * tag state is found. Shared by fabric_loopback and adc_capture. */
uint32_t deimos_tool_find_snap_tag(uint32_t trig_pos);

/* ============================================================================
 * Rate code table (SIGNAL field encoding → Mbps)
 * ============================================================================ */

/* Indexed by rate Mbps: DEIMOS_RATE_CODES[6] = 0x0B, etc.
 * Only valid for indices 6,9,12,18,24,36,48,54. */
extern const uint8_t DEIMOS_RATE_CODES[55];

/* ============================================================================
 * JSON waveform parsing
 *
 * Parses {"real":[f,f,...],"imag":[f,f,...]} with optional metadata.
 * No external JSON library — minimal, self-contained.
 * ============================================================================ */

/*
 * Load a waveform JSON file. Parses real/imag arrays and optional stf_offset.
 * Caller must free *re and *im.
 * If json_out is non-NULL, the raw JSON buffer is returned (caller must free).
 * Returns 0 on success, -1 on error.
 */
int deimos_tool_load_waveform(const char *path,
                              float **re, float **im, int *n_samples,
                              int *stf_offset, char **json_out);

/* ============================================================================
 * 12-bit DDR quantization
 *
 * Scales float IQ to ±2047 and packs into DDR format via IQ_PACK().
 * ============================================================================ */

/*
 * Quantize float IQ to 12-bit and write to a DDR buffer.
 * Finds peak, scales to ±2047, clamps, packs via IQ_PACK().
 * Returns the scale factor used.
 */
float deimos_tool_quantize_to_ddr(const float *re, const float *im,
                                  int n_samples, volatile uint32_t *ddr_buf);

/* ============================================================================
 * Frame builders
 * ============================================================================ */

/*
 * Build a simple test data frame (broadcast, incrementing payload).
 * Used by fabric_loopback, adc_capture, pluto_loopback.
 * Returns total PSDU length including FCS.
 */
size_t deimos_tool_build_test_psdu(uint8_t *buf, int payload_len);

/*
 * Build a data frame with LLC/SNAP header and optional EAPOL ethertype.
 * Includes unique sequence number in payload for matching.
 * Used by burst_loopback tools.
 * Returns total PSDU length including FCS.
 */
size_t deimos_tool_build_frame(uint8_t *buf, int payload_bytes,
                               uint8_t seq_num, bool eapol);

#endif /* DEIMOS_TOOL_H */
