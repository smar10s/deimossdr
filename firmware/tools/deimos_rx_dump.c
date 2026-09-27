// SPDX-License-Identifier: MIT
/*
 * deimos_rx_dump — Streaming fabric decode to JSONL
 *
 * Initializes the radio + fabric pipeline, then polls the tag FIFO
 * for decoded frames and writes one JSON line per frame to stdout.
 * Replaces deimos_daemon for EAPOL validation and OTA monitoring.
 *
 * Gain is AGC fast-attack only (D27) — there is no gain option.
 *
 * Usage:
 *   deimos_rx_dump -c 36 -d 30              # 30s on ch36
 *   deimos_rx_dump -c 36 -d 30 -o out.jsonl # write to file
 *   deimos_rx_dump -c 36                    # until SIGTERM
 *   deimos_rx_dump -c 36 -t 2 -d 60         # STF threshold 2
 *
 * JSONL format per frame:
 *   {"ts":N.NNN,"rate":R,"rate_code":0xRR,"fcs":B,"len":L,
 *    "class":"...","bssid":"...","sa":"...","da":"...",
 *    "is_eapol":B,"psdu":"HEX"}
 *
 * Fields omitted when not available (e.g. no bssid/sa/da for FCS-fail).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include <time.h>
#include <getopt.h>
#include <errno.h>

#include "deimos_rx.h"
#include "deimos_tool.h"

static const char *class_name(deimos_frame_class_t c)
{
    switch (c) {
    case DEIMOS_FCLASS_ACK:       return "ack";
    case DEIMOS_FCLASS_BLOCK_ACK: return "block_ack";
    case DEIMOS_FCLASS_HT_VHT:    return "ht_vht";
    case DEIMOS_FCLASS_BEACON:    return "beacon";
    case DEIMOS_FCLASS_MGMT:      return "mgmt";
    case DEIMOS_FCLASS_PROBE:     return "probe";
    case DEIMOS_FCLASS_DATA:      return "data";
    case DEIMOS_FCLASS_UNKNOWN:   return "unknown";
    default:                      return "???";
    }
}

static double elapsed_sec(struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double s = (double)(now.tv_sec - start->tv_sec);
    s += (double)(now.tv_nsec - start->tv_nsec) / 1e9;
    return s;
}

static void mac_to_str(const uint8_t *mac, char *out)
{
    if (!mac) {
        out[0] = '\0';
        return;
    }
    sprintf(out, "%02x:%02x:%02x:%02x:%02x:%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void psdu_to_hex(const uint8_t *psdu, uint16_t len, char *out, size_t out_size)
{
    if (len == 0) {
        out[0] = '\0';
        return;
    }
    size_t max = (out_size - 1) / 2;
    if (len > max) len = (uint16_t)max;
    for (uint16_t i = 0; i < len; i++)
        sprintf(out + i * 2, "%02x", psdu[i]);
    out[len * 2] = '\0';
}

/* Append ,"key":"value" only when present. Absent fields are omitted; the
 * previous inline form relied on the value buffers being empty in that case. */
static void json_str_field(FILE *out, const char *key, const char *val, bool present)
{
    if (present)
        fprintf(out, ",\"%s\":\"%s\"", key, val);
}

typedef struct {
    int    channel;
    int    duration_sec;
    int    stf_threshold;
    const char *output_file;
    bool   verbose;
} dump_config_t;

static int parse_args(int argc, char *argv[], dump_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->channel       = 36;
    cfg->duration_sec  = 0;
    cfg->stf_threshold = 0;
    cfg->output_file   = NULL;
    cfg->verbose       = false;

    int opt;
    while ((opt = getopt(argc, argv, "c:d:t:o:vh")) != -1) {
        switch (opt) {
        case 'c': cfg->channel       = atoi(optarg); break;
        case 'd': cfg->duration_sec  = atoi(optarg); break;
        case 't': cfg->stf_threshold = atoi(optarg); break;
        case 'o': cfg->output_file   = optarg; break;
        case 'v': cfg->verbose       = true; break;
        case 'h':
            printf("Usage: deimos_rx_dump [options]\n"
                   "  -c channel     WiFi channel (default: 36)\n"
                   "  -d duration    Capture duration in seconds (0 = until signal)\n"
                   "  -t threshold   STF threshold shift 0-7 (default: 0 = strictest;\n"
                   "                 each step doubles sensitivity)\n"
                   "  -o outfile     Write to file instead of stdout\n"
                   "  -v             Verbose diagnostics to stderr\n"
                   "  -h             Show this help\n");
            return 1;
        default: return 1;
        }
    }
    return 0;
}

int main(int argc, char *argv[])
{
    dump_config_t cfg;
    if (parse_args(argc, argv, &cfg) != 0)
        return 1;

    styx_install_shutdown_handler();

    FILE *out = stdout;
    if (cfg.output_file) {
        out = fopen(cfg.output_file, "w");
        if (!out) {
            fprintf(stderr, "deimos_rx_dump: cannot open %s: %s\n",
                    cfg.output_file, strerror(errno));
            return 1;
        }
        setlinebuf(out);
    } else {
        setlinebuf(stdout);
    }

    /* Radio init */
    deimos_radio_config_t radio = {
        .channel     = cfg.channel,
        .agc_mode    = DEIMOS_AGC_FAST_ATTACK,
        .configure_tx = false,
        .bandwidth_mhz = 0,
        .skip_calibration = false,
    };

    if (cfg.verbose)
        fprintf(stderr, "deimos_rx_dump: initializing radio ch%d (AGC fast-attack)...\n",
                cfg.channel);

    if (deimos_radio_init(&radio) != 0) {
        fprintf(stderr, "deimos_rx_dump: radio init failed\n");
        if (out != stdout) fclose(out);
        return 1;
    }

    /* Pipeline arm */
    deimos_pipeline_config_t pipeline = {
        .mode          = DEIMOS_RX_MODE_LIVE,
        .stf_threshold = cfg.stf_threshold,
        .stf_skip      = 4,
        .ltf_skip      = 1,
        .cfo_thresh    = 64,
    };
    deimos_pipeline_arm(&pipeline);

    /* Go live */
    if (deimos_stf_gate(DEIMOS_TRIGGER_NONE) != 0) {
        fprintf(stderr, "deimos_rx_dump: STF gate failed\n");
        styx_disarm_cleanup();
        deimos_rx_cleanup();
        if (out != stdout) fclose(out);
        return 1;
    }

    if (cfg.verbose) {
        fprintf(stderr, "deimos_rx_dump: live on ch%d",
                cfg.channel);
        if (cfg.duration_sec > 0)
            fprintf(stderr, " for %ds", cfg.duration_sec);
        else
            fprintf(stderr, " (press Ctrl-C to stop)");
        fprintf(stderr, "\n");
    }

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (!styx_shutdown_requested()) {
        if (cfg.duration_sec > 0 && elapsed_sec(&t0) >= (double)cfg.duration_sec)
            break;

        deimos_rx_frame_t frame;
        int ret = deimos_rx_poll(&frame);
        if (ret == 1) {
            double ts = elapsed_sec(&t0);
            deimos_frame_class_t cls = deimos_rx_classify_frame(
                frame.rate_mbps, frame.length, frame.fcs_ok);

            deimos_rx_meta_t meta;
            if (frame.fcs_ok && frame.psdu_len > 0)
                deimos_rx_parse_header(frame.psdu, frame.psdu_len, &meta);
            else
                memset(&meta, 0, sizeof(meta));

            char bssid_str[18] = {0};
            char sa_str[18]    = {0};
            char da_str[18]    = {0};
            if (meta.has_bssid) mac_to_str(meta.bssid, bssid_str);
            if (meta.has_sa)    mac_to_str(meta.sa, sa_str);
            if (meta.has_da)    mac_to_str(meta.da, da_str);

            char psdu_hex[2048];
            psdu_to_hex(frame.psdu, frame.psdu_len, psdu_hex, sizeof(psdu_hex));

            fprintf(out,
                "{"
                "\"ts\":%.3f,"
                "\"rate\":%d,"
                "\"rate_code\":%d,"
                "\"fcs\":%s,"
                "\"len\":%u,"
                "\"class\":\"%s\"",
                ts,
                frame.rate_mbps,
                frame.rate_code,
                frame.fcs_ok ? "true" : "false",
                (unsigned)frame.length,
                class_name(cls));
            json_str_field(out, "psdu", psdu_hex, frame.psdu_len > 0);
            json_str_field(out, "bssid", bssid_str, meta.has_bssid);
            json_str_field(out, "sa", sa_str, meta.has_sa);
            json_str_field(out, "da", da_str, meta.has_da);
            if (meta.is_eapol)
                fputs(",\"is_eapol\":true", out);
            fputs("}\n", out);
        } else if (ret == -1) {
            if (cfg.verbose)
                fprintf(stderr, "deimos_rx_dump: poll error\n");
            break;
        } else {
            /* Idle: brief backoff only. Sleeping after every frame starves
             * the drain during bursts and overflows the 16-deep tag FIFO,
             * dropping frames (observed as OTA EAPOL loss). */
            usleep(200);
        }
    }

    /* Final stats to stderr */
    uint32_t total_frames = deimos_rx_frame_count();
    uint32_t drops        = deimos_rx_drop_count();
    if (cfg.verbose || total_frames > 0) {
        fprintf(stderr, "deimos_rx_dump: %u frames, %u drops\n",
                total_frames, drops);
    }

    styx_disarm_cleanup();
    deimos_rx_cleanup();
    if (out != stdout) fclose(out);
    return 0;
}
