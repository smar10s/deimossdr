// SPDX-License-Identifier: MIT
#include "hal.h"
#include "dma_tx.h"
#include <stdint.h>
#include <stddef.h>

#define STUB_MAX_WRITES 4096

typedef struct {
    uint32_t addr;
    uint32_t val;
} stub_write_t;

static stub_write_t g_writes[STUB_MAX_WRITES];
static size_t g_write_count;
static uint32_t g_tag_status = TAG_STATUS_EMPTY;

void stub_reset_writes(void)
{
    g_write_count = 0;
}

size_t stub_total_writes(void)
{
    return g_write_count;
}

size_t stub_writes_to(uint32_t addr, uint32_t *out, size_t max)
{
    size_t n = 0;
    for (size_t i = 0; i < g_write_count && n < max; i++) {
        if (g_writes[i].addr == addr)
            out[n++] = g_writes[i].val;
    }
    return n;
}

void stub_set_tag_status(uint32_t status)
{
    g_tag_status = status;
}

int hal_init(void) { return 0; }
void hal_cleanup(void) {}

uint32_t hal_reg_read(uint32_t addr)
{
    if (addr == REG_TAG_FIFO_STATUS)
        return g_tag_status;
    return 0;
}

void hal_reg_write(uint32_t addr, uint32_t val)
{
    if (g_write_count < STUB_MAX_WRITES) {
        g_writes[g_write_count].addr = addr;
        g_writes[g_write_count].val = val;
        g_write_count++;
    }
}

int hal_ad9361_set_rx_lo(uint64_t freq_hz)         { (void)freq_hz; return 0; }
int hal_ad9361_set_tx_lo(uint64_t freq_hz)         { (void)freq_hz; return 0; }
int hal_ad9361_set_sample_rate(uint64_t rate_hz)   { (void)rate_hz; return 0; }
int hal_ad9361_set_rx_bandwidth(uint64_t bw_hz)    { (void)bw_hz; return 0; }
int hal_ad9361_set_tx_bandwidth(uint64_t bw_hz)    { (void)bw_hz; return 0; }
int hal_ad9361_set_rx_gain(double gain_db)         { (void)gain_db; return 0; }
double hal_ad9361_get_rx_gain(void)                { return 50.0; }
int hal_ad9361_set_rx_gain_mode(const char *mode)  { (void)mode; return 0; }
int hal_ad9361_set_tx_attenuation(double atten_db) { (void)atten_db; return 0; }
int hal_ad9361_run_calibration(void)               { return 0; }

int dma_tx_trigger(void) { return 0; }
