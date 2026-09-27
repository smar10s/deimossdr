# cocotb simulation Makefile for deimos FPGA modules
#
# Usage:
#   make test_debug_snap
#   make test_snap_axi
#   make test_iq_dma_rx
#
# Simulator selection:
#   SIM=verilator (default, compiled, ~6x faster than Icarus)
#   SIM=icarus    (fallback simulator; slower, not used by the gate suite)
#
# Verilator builds are cached in sim_build_* dirs and only recompile on RTL changes.

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

COCOTB_CONFIG ?= $(shell pwd)/../../.venv/bin/cocotb-config

RTL := $(shell pwd)/../rtl
STYX_RTL := $(shell pwd)/../../platform/styx/fpga/rtl

# Verilator-specific flags: suppress benign width warnings, disable timing
ifeq ($(SIM),verilator)
EXTRA_ARGS += --no-timing +define+SIM -Wno-ASCRANGE \
              -Wno-CASEINCOMPLETE -Wno-UNOPTFLAT
# Verilator coverage adds 4-5x slowdown. Enable with: make VERILATOR_COVERAGE=1 test_...
ifeq ($(VERILATOR_COVERAGE),1)
EXTRA_ARGS += --coverage
endif
endif

# ---- Build signature (cache invalidation) ----
# Cocotb rebuilds only when a source file is newer than the cached build.
# Compiler flags are not a prerequisite, so changing a warning flag leaves
# a stale binary in place and the suite keeps passing against old RTL.
#
# Record the inputs that are invisible to the file-mtime check and make the
# compile depend on that record. The file is rewritten only when the content
# changes, so an unchanged signature does not force a rebuild.
BUILD_SIG := $(SIM_BUILD)/.build_signature
CUSTOM_COMPILE_DEPS += $(BUILD_SIG)

.PHONY: FORCE
FORCE:

$(BUILD_SIG): FORCE | $(SIM_BUILD)
	@{ \
	  echo "sim=$(SIM)"; \
	  echo "tool=$$($(if $(filter verilator,$(SIM)),verilator --version,echo n/a) 2>/dev/null)"; \
	  echo "extra_args=$(EXTRA_ARGS)"; \
	  echo "compile_args=$(COMPILE_ARGS)"; \
	  echo "toplevel=$(TOPLEVEL)"; \
	  echo "sources=$(VERILOG_SOURCES)"; \
	  echo "styx=$$(git -C $(STYX_RTL) rev-parse HEAD 2>/dev/null)"; \
	} > $@.tmp
	@cmp -s $@.tmp $@ 2>/dev/null && rm -f $@.tmp || { \
	  if [ -f $@ ]; then echo "  build inputs changed -> recompiling $(SIM_BUILD)"; fi; \
	  mv $@.tmp $@; \
	}

.PHONY: clean_all
clean_all: ## Remove every cached simulation build and result
	rm -rf sim_build_* results.xml dump.vcd lint_report.txt
	@echo "Cleaned simulation artifacts."

.PHONY: all
all: sim

# ---- Lint target ----
# Standalone Verilator lint (no sim, no cocotb). Runs in ~2 seconds.
# Intentionally stricter than the sim compile: catches width issues,
# incomplete cases, and CDC signals that the sim suppresses.
#
# Usage:
#   make lint           -- all warnings (first pass: review + fix or waive)
#   make lint-strict    -- errors on CASEINCOMPLETE and UNOPTFLAT (gate check)

LINT_SRCS = $(wildcard $(RTL)/*.v)

# Warnings we always suppress (noise, not bugs):
#   DECLFILENAME  - module name != filename (rx_frontend vs rx_pipeline internals)
#   UNUSED        - unused signals (intentional in cocotb wrappers)
#   MULTITOP      - multiple top modules (expected when linting all files)
#   PINCONNECTEMPTY - unconnected diagnostic ports in sim wrappers
#   ASCRANGE      - [0:N] ranges (style choice, not bug)
LINT_SUPPRESS = -Wno-DECLFILENAME -Wno-UNUSED -Wno-MULTITOP \
                -Wno-PINCONNECTEMPTY -Wno-ASCRANGE

.PHONY: lint
lint:
	@echo "=== Verilator lint (advisory) ==="
	verilator --lint-only --Wall $(LINT_SUPPRESS) \
		--top-module rx_frontend $(LINT_SRCS) 2>&1 | tee lint_report.txt
	@echo ""
	@echo "--- Summary ---"
	@grep -c "^%Warning" lint_report.txt || echo "0 warnings"

.PHONY: lint-strict
lint-strict:
	@echo "=== Verilator lint (strict gate) ==="
	verilator --lint-only --Wall $(LINT_SUPPRESS) \
		-Werror-CASEINCOMPLETE -Werror-UNOPTFLAT \
		-Werror-WIDTHTRUNC -Werror-WIDTHEXPAND -Werror-CMPCONST \
		--top-module rx_frontend $(LINT_SRCS)
	@echo "LINT PASSED (strict)"

# Generated sim-view freshness: rx_pipeline.v/rx_frontend.v must match
# what system_bd.tcl generates (single source of truth, D19).
.PHONY: sim_view_check
sim_view_check:
	python3 ../scripts/gen_sim_pipeline.py --check \
		--bd ../fpga/project/system_bd.tcl --out-dir ../fpga/rtl

# ---- Convenience targets ----

.PHONY: test_debug_snap
test_debug_snap:
	$(MAKE) sim SIM_BUILD=sim_build_debug_snap \
		TOPLEVEL=debug_snap \
		VERILOG_SOURCES="$(STYX_RTL)/debug_snap.v" \
		COCOTB_TEST_MODULES=test_debug_snap

.PHONY: test_snap_axi
test_snap_axi:
	$(MAKE) sim SIM_BUILD=sim_build_snap_axi \
		TOPLEVEL=snap_axi \
		VERILOG_SOURCES="$(STYX_RTL)/snap_axi.v $(STYX_RTL)/debug_snap.v" \
		COCOTB_TEST_MODULES=test_snap_axi

.PHONY: test_iq_dma_rx
test_iq_dma_rx:
	$(MAKE) sim SIM_BUILD=sim_build_iq_dma_rx \
		TOPLEVEL=iq_dma_rx \
		VERILOG_SOURCES="$(STYX_RTL)/iq_dma_rx.v" \
		COCOTB_TEST_MODULES=test_iq_dma_rx

.PHONY: test_iq_dma_tx
test_iq_dma_tx:
	$(MAKE) sim SIM_BUILD=sim_build_iq_dma_tx \
		TOPLEVEL=iq_dma_tx \
		VERILOG_SOURCES="$(STYX_RTL)/iq_dma_tx.v" \
		COCOTB_TEST_MODULES=test_iq_dma_tx

# Deimos overrides PROJECT_ID to "WIFI" (0x57494649) at build time.
# Expose the override in sim so the test validates the Deimos value.
ifeq ($(SIM),verilator)
AXI_BUILD_ID_OVERRIDE := EXTRA_ARGS="$(EXTRA_ARGS) -GPROJECT_ID=0x57494649"
else
AXI_BUILD_ID_OVERRIDE := COMPILE_ARGS="-Paxi_build_id.PROJECT_ID=0x57494649"
endif

.PHONY: test_axi_build_id
test_axi_build_id:
	$(MAKE) sim SIM_BUILD=sim_build_axi_build_id \
		TOPLEVEL=axi_build_id \
		VERILOG_SOURCES="$(STYX_RTL)/axi_build_id.v" \
		COCOTB_TEST_MODULES=test_axi_build_id \
		$(AXI_BUILD_ID_OVERRIDE)

# Parameter override: DDR_BUF_SAMPLES=64 for wrap-boundary test.
# Verilator uses -G (top-level param) via EXTRA_ARGS (to not clobber cocotb's COMPILE_ARGS).
# Icarus uses -P (hierarchical) via COMPILE_ARGS (iverilog-only; EXTRA_ARGS goes to vvp too).
ifeq ($(SIM),verilator)
IQ_DMA_RX_WRAP_OVERRIDE := EXTRA_ARGS="$(EXTRA_ARGS) -GDDR_BUF_SAMPLES=64"
else
IQ_DMA_RX_WRAP_OVERRIDE := COMPILE_ARGS="-Piq_dma_rx.DDR_BUF_SAMPLES=64"
endif

.PHONY: test_iq_dma_rx_wrap
test_iq_dma_rx_wrap:
	$(MAKE) sim SIM_BUILD=sim_build_iq_dma_rx_wrap \
		TOPLEVEL=iq_dma_rx \
		VERILOG_SOURCES="$(STYX_RTL)/iq_dma_rx.v" \
		COCOTB_TEST_MODULES=test_iq_dma_rx \
		$(IQ_DMA_RX_WRAP_OVERRIDE)

.PHONY: test_async_fifo
test_async_fifo:
	$(MAKE) sim SIM_BUILD=sim_build_async_fifo \
		TOPLEVEL=async_fifo \
		VERILOG_SOURCES="$(STYX_RTL)/async_fifo.v" \
		COCOTB_TEST_MODULES=test_async_fifo

.PHONY: test_adc_sync
test_adc_sync:
	$(MAKE) sim SIM_BUILD=sim_build_adc_sync \
		TOPLEVEL=adc_sync \
		VERILOG_SOURCES="$(STYX_RTL)/adc_sync.v $(STYX_RTL)/async_fifo.v" \
		COCOTB_TEST_MODULES=test_adc_sync

.PHONY: test_deimos_regs_axi
test_deimos_regs_axi:
	$(MAKE) sim SIM_BUILD=sim_build_deimos_regs_axi \
		TOPLEVEL=deimos_regs_axi \
		VERILOG_SOURCES="$(RTL)/deimos_regs_axi.v" \
		COCOTB_TEST_MODULES=test_deimos_regs_axi

.PHONY: test_hil_ctrl
test_hil_ctrl:
	$(MAKE) sim SIM_BUILD=sim_build_hil_ctrl \
		TOPLEVEL=hil_ctrl \
		VERILOG_SOURCES="$(STYX_RTL)/hil_ctrl.v" \
		COCOTB_TEST_MODULES=test_hil_ctrl

.PHONY: test_stf_detect
test_stf_detect:
	$(MAKE) sim SIM_BUILD=sim_build_stf_detect \
		TOPLEVEL=stf_detect \
		VERILOG_SOURCES="$(RTL)/stf_detect.v $(RTL)/bram_delay_tap.v" \
		COCOTB_TEST_MODULES=test_stf_detect

.PHONY: test_bram_delay_tap
test_bram_delay_tap:
	$(MAKE) sim SIM_BUILD=sim_build_bram_delay_tap \
		TOPLEVEL=bram_delay_tap \
		VERILOG_SOURCES="$(RTL)/bram_delay_tap.v" \
		COCOTB_TEST_MODULES=test_bram_delay_tap

.PHONY: test_cfo_est
test_cfo_est:
	$(MAKE) sim SIM_BUILD=sim_build_cfo_est \
		TOPLEVEL=cfo_est \
		VERILOG_SOURCES="$(RTL)/cfo_est.v $(RTL)/cordic_atan2.v" \
		COCOTB_TEST_MODULES=test_cfo_est

.PHONY: test_cfo_mixer
test_cfo_mixer:
	$(MAKE) sim SIM_BUILD=sim_build_cfo_mixer \
		TOPLEVEL=cfo_mixer \
		VERILOG_SOURCES="$(RTL)/cfo_mixer.v" \
		COCOTB_TEST_MODULES=test_cfo_mixer

.PHONY: test_fft4_sdf
test_fft4_sdf:
	$(MAKE) sim SIM_BUILD=sim_build_fft4_sdf \
		TOPLEVEL=fft4_sdf \
		VERILOG_SOURCES="$(RTL)/fft64_sdf.v" \
		COCOTB_TEST_MODULES=test_fft4_sdf

.PHONY: test_fft16_sdf
test_fft16_sdf:
	$(MAKE) sim SIM_BUILD=sim_build_fft16_sdf \
		TOPLEVEL=fft16_sdf \
		VERILOG_SOURCES="$(RTL)/fft64_sdf.v" \
		COCOTB_TEST_MODULES=test_fft16_sdf

.PHONY: test_fft64_sdf
test_fft64_sdf:
	$(MAKE) sim SIM_BUILD=sim_build_fft64_sdf \
		TOPLEVEL=fft64_sdf \
		VERILOG_SOURCES="$(RTL)/fft64_sdf.v" \
		COCOTB_TEST_MODULES=test_fft64_sdf

.PHONY: diag_fft_precision
diag_fft_precision:
	$(MAKE) sim SIM_BUILD=sim_build_fft64_sdf \
		TOPLEVEL=fft64_sdf \
		VERILOG_SOURCES="$(RTL)/fft64_sdf.v" \
		COCOTB_TEST_MODULES=diag_fft_precision

.PHONY: diag_fft_cp_gap
diag_fft_cp_gap:
	$(MAKE) sim SIM_BUILD=sim_build_fft64_sdf \
		TOPLEVEL=fft64_sdf \
		VERILOG_SOURCES="$(RTL)/fft64_sdf.v" \
 		COCOTB_TEST_MODULES=diag_fft_cp_gap

.PHONY: test_chan_est
test_chan_est:
	$(MAKE) sim SIM_BUILD=sim_build_chan_est \
		TOPLEVEL=chan_est \
		VERILOG_SOURCES="$(RTL)/chan_est.v" \
		COCOTB_TEST_MODULES=test_chan_est

.PHONY: test_equalizer
test_equalizer:
	$(MAKE) sim SIM_BUILD=sim_build_equalizer \
		TOPLEVEL=equalizer \
		VERILOG_SOURCES="$(RTL)/equalizer.v" \
		COCOTB_TEST_MODULES=test_equalizer

.PHONY: test_demap_mux
test_demap_mux:
	$(MAKE) sim SIM_BUILD=sim_build_demap_mux \
		TOPLEVEL=demap_mux \
		VERILOG_SOURCES="$(RTL)/demap_mux.v" \
		COCOTB_TEST_MODULES=test_demap_mux

.PHONY: test_demapper
test_demapper:
	$(MAKE) sim SIM_BUILD=sim_build_demapper \
		TOPLEVEL=demapper \
		VERILOG_SOURCES="$(RTL)/demapper.v" \
		COCOTB_TEST_MODULES=test_demapper

.PHONY: test_deinterleaver
test_deinterleaver:
	$(MAKE) sim SIM_BUILD=sim_build_deinterleaver \
		TOPLEVEL=deinterleaver \
		VERILOG_SOURCES="$(RTL)/deinterleaver.v" \
		COCOTB_TEST_MODULES=test_deinterleaver

.PHONY: test_depuncturer
test_depuncturer:
	$(MAKE) sim SIM_BUILD=sim_build_depuncturer \
		TOPLEVEL=depuncturer \
		VERILOG_SOURCES="$(RTL)/depuncturer.v" \
		COCOTB_TEST_MODULES=test_depuncturer

.PHONY: test_viterbi_k7
test_viterbi_k7:
	$(MAKE) sim SIM_BUILD=sim_build_viterbi_k7 \
		TOPLEVEL=viterbi_k7 \
		VERILOG_SOURCES="$(RTL)/viterbi_k7.v" \
		COCOTB_TEST_MODULES=test_viterbi_k7

.PHONY: test_descrambler
test_descrambler:
	$(MAKE) sim SIM_BUILD=sim_build_descrambler \
		TOPLEVEL=descrambler \
		VERILOG_SOURCES="$(RTL)/descrambler.v" \
		COCOTB_TEST_MODULES=test_descrambler

.PHONY: test_fcs_check
test_fcs_check:
	$(MAKE) sim SIM_BUILD=sim_build_fcs_check \
		TOPLEVEL=fcs_check \
		VERILOG_SOURCES="$(RTL)/fcs_check.v" \
		COCOTB_TEST_MODULES=test_fcs_check

.PHONY: test_psdu_packer
test_psdu_packer:
	$(MAKE) sim SIM_BUILD=sim_build_psdu_packer \
		TOPLEVEL=psdu_packer \
		VERILOG_SOURCES="$(RTL)/psdu_packer.v" \
		COCOTB_TEST_MODULES=test_psdu_packer

.PHONY: test_tag_fifo_axi
test_tag_fifo_axi:
	$(MAKE) sim SIM_BUILD=sim_build_tag_fifo_axi \
		TOPLEVEL=tag_fifo_axi \
		VERILOG_SOURCES="$(RTL)/tag_fifo_axi.v $(RTL)/psdu_bram.v" \
		COCOTB_TEST_MODULES=test_tag_fifo_axi

.PHONY: test_vit_fifo
test_vit_fifo:
	$(MAKE) sim SIM_BUILD=sim_build_vit_fifo \
		TOPLEVEL=vit_fifo \
		VERILOG_SOURCES="$(RTL)/vit_fifo.v" \
		COCOTB_TEST_MODULES=test_vit_fifo

.PHONY: test_soft_pairer
test_soft_pairer:
	$(MAKE) sim SIM_BUILD=sim_build_soft_pairer \
		TOPLEVEL=soft_pairer \
		VERILOG_SOURCES="$(RTL)/soft_pairer.v" \
		COCOTB_TEST_MODULES=test_soft_pairer

.PHONY: test_decode_chain
test_decode_chain:
	$(MAKE) sim SIM_BUILD=sim_build_decode_chain \
		TOPLEVEL=decode_chain \
		VERILOG_SOURCES="$(RTL)/decode_chain.v $(RTL)/depuncturer.v $(RTL)/soft_pairer.v $(RTL)/vit_fifo.v $(RTL)/viterbi_k7.v" \
		COCOTB_TEST_MODULES=test_decode_chain

# test_rx_top removed (2026-06-22): fft_wrap deleted, FFT now internal to decode_engine.
# Unique coverage (bad_parity abort) moved to test_rx_pipeline.test_signal_abort_bad_parity.
# Remaining coverage already superseded by test_rx_pipeline (14 tests) + test_continuous_decode (5 tests).

.PHONY: test_rx_pipeline
test_rx_pipeline:
	$(MAKE) sim SIM_BUILD=sim_build_rx_pipeline \
		TOPLEVEL=rx_pipeline \
		VERILOG_SOURCES="$(RTL)/rx_pipeline.v $(RTL)/acquisition_ctrl.v $(RTL)/frame_fifo.v $(RTL)/decode_engine.v $(RTL)/cfo_mixer.v $(RTL)/ltf_correlator.v $(RTL)/ltf_rom32.v $(RTL)/ltf_peak_detect.v $(RTL)/fft64_sdf.v $(RTL)/chan_est.v $(RTL)/equalizer.v $(RTL)/pilot_track.v $(RTL)/cordic_atan2_sm.v $(RTL)/cordic_rotate.v $(RTL)/demap_mux.v $(RTL)/demapper.v $(RTL)/deinterleaver.v $(RTL)/depuncturer.v $(RTL)/soft_pairer.v $(RTL)/vit_fifo.v $(RTL)/viterbi_k7.v $(RTL)/descrambler.v $(RTL)/fcs_check.v $(RTL)/psdu_packer.v" \
		COCOTB_TEST_MODULES=test_rx_pipeline

.PHONY: test_cordic_rotate
test_cordic_rotate:
	$(MAKE) sim SIM_BUILD=sim_build_cordic_rotate \
		TOPLEVEL=cordic_rotate \
		VERILOG_SOURCES="$(RTL)/cordic_rotate.v" \
		COCOTB_TEST_MODULES=test_cordic_rotate

.PHONY: test_pilot_track
test_pilot_track:
	$(MAKE) sim SIM_BUILD=sim_build_pilot_track \
		TOPLEVEL=pilot_track \
		VERILOG_SOURCES="$(RTL)/pilot_track.v $(RTL)/cordic_atan2_sm.v $(RTL)/cordic_rotate.v" \
		COCOTB_TEST_MODULES=test_pilot_track

.PHONY: test_ltf_correlator
test_ltf_correlator:
	$(MAKE) sim SIM_BUILD=sim_build_ltf_correlator \
		TOPLEVEL=ltf_correlator \
		VERILOG_SOURCES="$(RTL)/ltf_correlator.v $(RTL)/ltf_rom32.v" \
		COCOTB_TEST_MODULES=test_ltf_correlator

.PHONY: test_ltf_peak_detect
test_ltf_peak_detect:
	$(MAKE) sim SIM_BUILD=sim_build_ltf_peak_detect \
		TOPLEVEL=ltf_peak_detect \
		VERILOG_SOURCES="$(RTL)/ltf_peak_detect.v" \
		COCOTB_TEST_MODULES=test_ltf_peak_detect

# --- rx_frontend sources (shared between test and diag) ---
RX_FRONTEND_SRCS = $(RTL)/rx_frontend.v $(RTL)/stf_detect.v $(RTL)/bram_delay_tap.v \
	$(RTL)/cfo_est.v \
	$(RTL)/cordic_atan2.v $(RTL)/cfo_mixer.v $(RTL)/rx_pipeline.v \
	$(RTL)/acquisition_ctrl.v $(RTL)/frame_fifo.v $(RTL)/decode_engine.v \
	$(RTL)/ltf_correlator.v $(RTL)/ltf_rom32.v $(RTL)/ltf_peak_detect.v \
	$(RTL)/fft64_sdf.v $(RTL)/chan_est.v \
	$(RTL)/equalizer.v $(RTL)/pilot_track.v $(RTL)/cordic_atan2_sm.v $(RTL)/cordic_rotate.v \
	$(RTL)/demap_mux.v $(RTL)/demapper.v \
	$(RTL)/deinterleaver.v $(RTL)/depuncturer.v $(RTL)/soft_pairer.v \
	$(RTL)/vit_fifo.v $(RTL)/viterbi_k7.v $(RTL)/descrambler.v $(RTL)/fcs_check.v \
	$(RTL)/psdu_packer.v

print_rx_frontend_srcs:
	@echo $(RX_FRONTEND_SRCS)

.PHONY: test_rx_frontend
test_rx_frontend:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_rx_frontend

.PHONY: test_long_frame
test_long_frame:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_long_frame

.PHONY: test_adc_replay
test_adc_replay:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_adc_replay

.PHONY: test_hil_burst_replay
test_hil_burst_replay:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_hil_burst_replay

.PHONY: test_cfo_resilience
test_cfo_resilience:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_cfo_resilience

.PHONY: test_continuous_decode
test_continuous_decode:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_continuous_decode

.PHONY: test_system_traffic
test_system_traffic:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_system_traffic

.PHONY: test_latency_ratchet
test_latency_ratchet:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_latency_ratchet

.PHONY: test_pending_trigger
test_pending_trigger:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_pending_trigger

.PHONY: test_pending_stress
test_pending_stress:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_pending_stress

.PHONY: test_constrained_random
test_constrained_random:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_constrained_random

.PHONY: test_channel_model
test_channel_model:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=test_channel_model

.PHONY: test_acquisition_ctrl
test_acquisition_ctrl:
	$(MAKE) sim SIM_BUILD=sim_build_acquisition_ctrl \
		TOPLEVEL=acquisition_ctrl \
		VERILOG_SOURCES="$(RTL)/acquisition_ctrl.v" \
		COCOTB_TEST_MODULES=test_acquisition_ctrl

.PHONY: test_frame_fifo
test_frame_fifo:
	$(MAKE) sim SIM_BUILD=sim_build_frame_fifo \
		TOPLEVEL=frame_fifo \
		VERILOG_SOURCES="$(RTL)/frame_fifo.v" \
		COCOTB_TEST_MODULES=test_frame_fifo

.PHONY: diag_rx_frontend
diag_rx_frontend:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_rx_frontend

.PHONY: diag_bit_path_dump
diag_bit_path_dump:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_bit_path_dump

.PHONY: diag_hil_burst_replay
diag_hil_burst_replay:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_hil_burst_replay

# Single-rate 6-frame burst sweep. Separates code rate from modulation
# order when diagnosing a burst regression (BURST_RATE, BURST_IMPAIR).
.PHONY: diag_rate_matrix_burst
diag_rate_matrix_burst:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_rate_matrix_burst

# Depuncturer residual state at each vit_frame_start. Locates undrained
# elastic-FIFO bits that survive a frame boundary (BURST_RATE, BURST_GAP).
.PHONY: diag_depunct_residual
diag_depunct_residual:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_depunct_residual

# Per-frame valid-pulse counts at each decode stage boundary. Shows which
# stage loses bits across a frame boundary (BURST_RATE, BURST_GAP).
.PHONY: diag_stage_counts
diag_stage_counts:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_stage_counts

# Does code_rate_out change while the depuncturer still holds a backlog?
# Catches a puncture-pattern reset landing mid-stream (BURST_RATE, BURST_GAP).
.PHONY: diag_coderate_overlap
diag_coderate_overlap:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_coderate_overlap

# Task 9 diagnostics: replay raw deimos_burst_loopback --dump-hil .hil files
# and trace frame-boundary / per-symbol datapath state (lever 2a-prime
# layer-6 investigation). Degrade gracefully on pre-stall-chain RTL.
.PHONY: diag_burst_hil_replay
diag_burst_hil_replay:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_burst_hil_replay

.PHONY: diag_burst_boundary_trace
diag_burst_boundary_trace:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_burst_boundary_trace

.PHONY: diag_burst_symbol_counts
diag_burst_symbol_counts:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_burst_symbol_counts

.PHONY: diag_burst_stream_dump
diag_burst_stream_dump:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_burst_stream_dump

.PHONY: diag_throughput
diag_throughput:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_throughput

.PHONY: diag_decode_profile
diag_decode_profile:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_decode_profile

.PHONY: diag_pending_vcd
diag_pending_vcd:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_pending_vcd

.PHONY: diag_stf_clear_race
diag_stf_clear_race:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_stf_clear_race

.PHONY: diag_evm
diag_evm:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_rx_frontend \
		TESTCASE=diag_evm_analysis

.PHONY: diag_long_frame
diag_long_frame:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_long_frame

# --- Throughput diagnostics (latency budget) ---

.PHONY: diag_eq_throughput
diag_eq_throughput:
	$(MAKE) sim SIM_BUILD=sim_build_equalizer \
		TOPLEVEL=equalizer \
		VERILOG_SOURCES="$(RTL)/equalizer.v" \
		COCOTB_TEST_MODULES=diag_eq_throughput

.PHONY: diag_pilot_throughput
diag_pilot_throughput:
	$(MAKE) sim SIM_BUILD=sim_build_pilot_track \
		TOPLEVEL=pilot_track \
		VERILOG_SOURCES="$(RTL)/pilot_track.v $(RTL)/cordic_atan2_sm.v $(RTL)/cordic_rotate.v" \
		COCOTB_TEST_MODULES=diag_pilot_throughput

.PHONY: diag_viterbi_throughput
diag_viterbi_throughput:
	$(MAKE) sim SIM_BUILD=sim_build_viterbi_k7 \
		TOPLEVEL=viterbi_k7 \
		VERILOG_SOURCES="$(RTL)/viterbi_k7.v" \
		COCOTB_TEST_MODULES=diag_viterbi_throughput

.PHONY: diag_symbol_cycle
diag_symbol_cycle:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_symbol_cycle

.PHONY: diag_byte_alignment
diag_byte_alignment:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_byte_alignment

.PHONY: diag_power_ratio
diag_power_ratio:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_power_ratio

# Per-frame CFO step across a SIFS pair (M1 AP -> M2 STA). Reads the RTL
# abort snapshot to check whether frame 2's L-SIG parity fails, reproducing
# the OTA EAPOL loss (0x13E1). See docs/acquisition-window-fix.md.
.PHONY: diag_sifs_cfo_step
diag_sifs_cfo_step:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_sifs_cfo_step

# Co-channel interferer summed over the target M2 preamble, slid in time.
# Tests the surviving hypothesis left after diag_sifs_cfo_step was falsified:
# overlapping (not merely stepped) energy biases the coarse estimate.
.PHONY: diag_cochannel_overlap
diag_cochannel_overlap:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_cochannel_overlap

# Class-A acquisition measurement: is a second, correctly-timed M2 acquisition
# reachable? Reports acquisition_ctrl descriptor counts (port-level) + which
# frame's LTF was acquired. Sim-only, no RTL change. SIM_BUILD name matches the
# `waves` target's convention so `make waves TARGET=diag_class_a_acquisition`
# rebuilds with tracing.
.PHONY: diag_class_a_acquisition
diag_class_a_acquisition:
	$(MAKE) sim SIM_BUILD=sim_build_diag_class_a_acquisition \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_class_a_acquisition

# Replay a captured OTA raw-DDR window (deimos_ota_capture + the host converter)
# through the fabric and compare the OTA tag log with the sim tag set. The
# fabric-vs-upstream decision fork. Set OTA_WINDOW_JSON to the stimulus path.
.PHONY: diag_ota_window_replay
diag_ota_window_replay:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_ota_window_replay

.PHONY: diag_lsig_constellation
diag_lsig_constellation:
	$(MAKE) sim SIM_BUILD=sim_build_rx_frontend \
		TOPLEVEL=rx_frontend \
		VERILOG_SOURCES="$(RX_FRONTEND_SRCS)" \
		COCOTB_TEST_MODULES=diag_lsig_constellation

# Wrong per-frame CFO applied to a clean frame: closes the causal link from a
# stale/wrong descriptor phase_inc to an S_PARSE_SIGNAL abort (OTA 0x13E1).
.PHONY: diag_stale_cfo_abort
diag_stale_cfo_abort:
	$(MAKE) sim SIM_BUILD=sim_build_diag_stale_cfo \
		TOPLEVEL=rx_pipeline \
		VERILOG_SOURCES="$(RTL)/rx_pipeline.v $(RTL)/acquisition_ctrl.v $(RTL)/frame_fifo.v $(RTL)/decode_engine.v $(RTL)/cfo_mixer.v $(RTL)/ltf_correlator.v $(RTL)/ltf_rom32.v $(RTL)/ltf_peak_detect.v $(RTL)/fft64_sdf.v $(RTL)/chan_est.v $(RTL)/equalizer.v $(RTL)/pilot_track.v $(RTL)/cordic_atan2_sm.v $(RTL)/cordic_rotate.v $(RTL)/demap_mux.v $(RTL)/demapper.v $(RTL)/deinterleaver.v $(RTL)/depuncturer.v $(RTL)/soft_pairer.v $(RTL)/vit_fifo.v $(RTL)/viterbi_k7.v $(RTL)/descrambler.v $(RTL)/fcs_check.v $(RTL)/psdu_packer.v" \
		COCOTB_TEST_MODULES=diag_stale_cfo_abort

# ---- Waveform generation ----

WAVES_DIR := $(CURDIR)/waves
STALE_SECONDS := 600

.PHONY: waves waves-check

waves: ## Generate VCD for TARGET (e.g. make waves TARGET=test_rx_pipeline)
ifndef TARGET
	@echo "ERROR: specify TARGET=test_xxx (e.g. make waves TARGET=test_rx_pipeline)"
	@echo "Available:"; grep -E '^\.PHONY:.*test_' Makefile | grep -hoE 'test_[a-z_]+' | sort -u | sed 's/^/  /'
	@exit 1
endif
	@# Trace requires different Verilator build flags — remove stale sim_build
	@SIM_DIR="sim_build_$$(echo $(TARGET) | sed 's/^test_//')"; \
	if [ -d "$$SIM_DIR" ]; then rm -rf "$$SIM_DIR"; fi
	@echo "=== Generating VCD for $(TARGET) ==="
	@# Verilator needs VERILATOR_TRACE=1 (cocotb's WAVES=1 only applies to
	@# Riviera/Questa). Pass the right one so `make waves` actually dumps a VCD.
	@$(MAKE) $(TARGET) $(if $(filter verilator,$(SIM)),VERILATOR_TRACE=1,WAVES=1) \
		$(if $(TESTCASE),TESTCASE=$(TESTCASE))
	@mkdir -p $(WAVES_DIR)
	@VCD=$$(find . -maxdepth 2 -name '*.vcd' -newer Makefile 2>/dev/null | head -1); \
	if [ -z "$$VCD" ]; then \
		echo "ERROR: No VCD produced. Check that WAVES=1 is supported."; \
		exit 1; \
	fi; \
	STAMP=$$(date +%Y%m%d_%H%M%S); \
	TC_SUFFIX=""; \
	if [ -n "$(TESTCASE)" ]; then TC_SUFFIX="_$(TESTCASE)"; fi; \
	DEST="$(WAVES_DIR)/$(TARGET)$${TC_SUFFIX}_$${STAMP}.vcd"; \
	cp "$$VCD" "$$DEST"; \
	ln -sf "$$(basename $$DEST)" "$(WAVES_DIR)/$(TARGET)$${TC_SUFFIX}_latest.vcd"; \
	SIZE=$$(du -h "$$DEST" | cut -f1); \
	echo "=== VCD: $$DEST ($$SIZE) ==="; \
	echo ""; \
	echo "Query:"; \
	echo "  python scripts/vcd_query.py list $$DEST"; \
	echo "  python scripts/vcd_query.py symbol-timing $$DEST"

waves-check: ## Warn if latest VCD is stale (>10 min old)
ifndef TARGET
	@echo "ERROR: specify TARGET=test_xxx"
	@exit 1
endif
	@LATEST="$(WAVES_DIR)/$(TARGET)_latest.vcd"; \
	if [ ! -f "$$LATEST" ]; then \
		echo "ERROR: No VCD found for $(TARGET)"; \
		echo "  Generate with: make waves TARGET=$(TARGET)"; \
		exit 1; \
	fi; \
	if [ "$$(uname)" = "Darwin" ]; then \
		MTIME=$$(stat -f %m "$$LATEST"); \
	else \
		MTIME=$$(stat -c %Y "$$LATEST"); \
	fi; \
	NOW=$$(date +%s); \
	AGE=$$((NOW - MTIME)); \
	if [ $$AGE -gt $(STALE_SECONDS) ]; then \
		echo "WARNING: VCD is $$((AGE / 60)) minutes old (stale)"; \
		echo "  File: $$LATEST"; \
		echo "  Regenerate: make waves TARGET=$(TARGET)"; \
		exit 1; \
	else \
		echo "VCD is fresh ($${AGE}s old): $$LATEST"; \
	fi

include $(shell $(COCOTB_CONFIG) --makefiles)/Makefile.sim
