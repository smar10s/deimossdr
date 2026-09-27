// SPDX-License-Identifier: MIT
#include "hal.h"
#include "dma_tx.h"
#include <stdint.h>
#include <stddef.h>

int hal_init(void) { return 0; }
void hal_cleanup(void) {}

uint32_t hal_reg_read(uint32_t addr)     { (void)addr; return 0; }
void hal_reg_write(uint32_t addr, uint32_t val) { (void)addr; (void)val; }

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
