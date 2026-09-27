# SPDX-License-Identifier: MIT
# fpga/Makefile — Deimos FPGA build system (delegates to Styx build infra)
#
# Styx provides: styx_procs.tcl (reusable build procs)
# Deimos provides: project dir, RTL dir, OOC module list, own build.tcl
#
# Targets:
#   make bitstream   — Full build: synth + impl + bitstream
#   make sim         — Run cocotb simulation suite
#   make waves       — Generate VCD waveform (TARGET=test_xxx)
#   make clean       — Remove build artifacts
#
# Configuration:
#   Copy config.mk.example to config.mk (project root) and edit, or pass
#   on command line:
#     make bitstream PLACE_STRATEGY=EarlyBlockPlacement
#     make bitstream REMOTE=buildhost.example.com

.PHONY: bitstream sim waves clean help

# ---- Configuration ----
-include ../config.mk

# Vivado
VIVADO          ?= vivado
VIVADO_FLAGS    ?= -mode batch -nojournal -nolog

# Remote builds (empty = local). Defaults match the top-level Makefile.
REMOTE          ?=
REMOTE_USER     ?= $(USER)
REMOTE_PORT     ?= 22
REMOTE_DIR      ?= ~/deimos

# Vivado environment (for remote builds where vivado isn't on PATH)
VIVADO_SETTINGS ?= /opt/Xilinx/2025.2/Vivado/settings64.sh

# Extra library path prepended to LD_LIBRARY_PATH on the remote build host.
# Empty by default. Some Vivado installs need a local shim directory here
# (e.g. patched libstdc++); set REMOTE_LD_PATH in config.mk if yours does.
REMOTE_LD_PATH  ?=

# Build parameters
# WLDrivenBlockPlacement worked at 89% utilization (old incremental baseline).
# After OOC synth state loss, fresh synthesis produces ~96% pre-opt LUTs.
# AltSpreadLogic_high + ExploreArea (opt_design) fit the Aug 27 build
# (WNS +0.180) but misses today's netlist by 19 ps (WNS -0.019, deterministic
# across runs). RTL logic is unchanged since Aug 27 — the netlist delta is
# just the build-id constant after the Sep 2 fingerprint/comments changes.
# ExtraPostPlacementOpt + ExploreArea closes it: 15694 LUTs, WNS +0.208.
# See D21 for history.
PLACE_STRATEGY  ?= ExtraPostPlacementOpt

# opt_design directive applied during implementation (styx::implement
# -opt_directive). ExploreArea is required to fit at ~96% pre-opt LUTs
# (~200 LUT headroom); the styx default (ExploreWithRemap) does not fit.
# Fit-tuning knob: overridable from the command line, see D21.
OPT_DIRECTIVE   ?= ExploreArea

# OOC modules: area-optimized synthesis on compute-heavy receiver modules
OOC_MODULES     ?= stf_detect cfo_est viterbi_k7 demapper pilot_track \
                   cfo_mixer fft64_sdf ltf_correlator ltf_peak_detect \
                   decode_engine acquisition_ctrl

INCREMENTAL     ?=

# Directories
ROOT_DIR        := $(abspath ..)
PROJECT_DIR     := $(CURDIR)/project
RTL_DIR         := $(CURDIR)/rtl
TCL_DIR         := $(CURDIR)/tcl
ADI_HDL_DIR     := $(abspath ../platform/styx/fpga/extern/adi-hdl)
OUTPUT_DIR      := $(ROOT_DIR)/build/fpga
SIM_DIR         := $(CURDIR)/test

# ---- Derived ----
SSH             = ssh -p $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE)
RSYNC           = rsync -az --delete --exclude-from=$(ROOT_DIR)/.rsync-exclude -e "ssh -p $(REMOTE_PORT)"

# Optional LD_LIBRARY_PATH prefix for the remote build (see REMOTE_LD_PATH).
ifneq ($(REMOTE_LD_PATH),)
  REMOTE_LD_EXPORT = export LD_LIBRARY_PATH=$(REMOTE_LD_PATH):\$$LD_LIBRARY_PATH && 
else
  REMOTE_LD_EXPORT =
endif

BUILD_TCL_ARGS  = -project_dir $(PROJECT_DIR) \
                  -rtl_dir $(RTL_DIR) \
                  -adi_hdl_dir $(ADI_HDL_DIR) \
                  -output_dir $(OUTPUT_DIR) \
                  -place_strategy $(PLACE_STRATEGY) \
                  -opt_directive $(OPT_DIRECTIVE)

ifneq ($(OOC_MODULES),)
  BUILD_TCL_ARGS += -ooc_modules "$(OOC_MODULES)"
endif

ifneq ($(INCREMENTAL),)
  BUILD_TCL_ARGS += -incremental $(INCREMENTAL)
endif

# ---- Targets ----

help: ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

ifdef REMOTE
# ===== REMOTE BUILD MODE =====

bitstream: _check_adi ## Build bitstream (remote)
	@echo "=== Remote build on $(REMOTE) (strategy=$(PLACE_STRATEGY) opt=$(OPT_DIRECTIVE)) ==="
	$(RSYNC) $(ROOT_DIR)/ $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/
	$(SSH) "cd $(REMOTE_DIR)/fpga && \
		source $(VIVADO_SETTINGS) && \
		$(REMOTE_LD_EXPORT)export XILINX_LOCAL_USER_DATA=no && \
		make bitstream REMOTE= \
		VIVADO='vivado' \
		PLACE_STRATEGY=$(PLACE_STRATEGY) \
		OPT_DIRECTIVE=$(OPT_DIRECTIVE) \
		OOC_MODULES='$(OOC_MODULES)' \
		INCREMENTAL='$(INCREMENTAL)'"
	@mkdir -p $(OUTPUT_DIR)
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/system_top.xsa $(OUTPUT_DIR)/
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/system_top.bit $(OUTPUT_DIR)/
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/fingerprint $(OUTPUT_DIR)/
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/timing_status $(OUTPUT_DIR)/
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/deimos_impl.dcp $(OUTPUT_DIR)/ 2>/dev/null || true
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/utilization.rpt $(OUTPUT_DIR)/ 2>/dev/null || true
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/bitstream.sha256 $(OUTPUT_DIR)/ 2>/dev/null || true
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/build_info.json $(OUTPUT_DIR)/ 2>/dev/null || true
	@# build evidence may be shared outside the repo; Vivado stamps the build
	@# host and an absolute remote path into the utilization header. Redact both.
	@if [ -f $(OUTPUT_DIR)/utilization.rpt ]; then \
		sed -i.bak -e 's/^| Host  *:.*/| Host         : (redacted)/' \
		           -e 's/^| Command  *:.*/| Command      : report_utilization -hierarchical -hierarchical_depth 2/' \
		    $(OUTPUT_DIR)/utilization.rpt && rm -f $(OUTPUT_DIR)/utilization.rpt.bak; \
	fi
	@echo "=== Build complete ==="
	@echo "Fingerprint: $$(cat $(OUTPUT_DIR)/fingerprint)"
	@if [ -f $(OUTPUT_DIR)/bitstream.sha256 ]; then echo "Bitstream sha256: $$(cat $(OUTPUT_DIR)/bitstream.sha256)"; fi

else
# ===== LOCAL BUILD MODE =====

bitstream: _check_adi ## Build bitstream (local)
	@echo "=== Building bitstream (strategy=$(PLACE_STRATEGY) opt=$(OPT_DIRECTIVE)) ==="
	@mkdir -p $(OUTPUT_DIR)
	$(VIVADO) $(VIVADO_FLAGS) -source $(TCL_DIR)/build.tcl \
		-tclargs $(BUILD_TCL_ARGS); \
	RC=$$?; \
	if [ $$RC -eq 0 ]; then \
		echo "=== Build SUCCESS (timing MET) ==="; \
		echo "met" > $(OUTPUT_DIR)/timing_status; \
	elif [ $$RC -eq 2 ]; then \
		echo "=== Build FAILED (timing VIOLATED) ==="; \
		echo "violated" > $(OUTPUT_DIR)/timing_status; \
		exit 2; \
	else \
		echo "=== Build FAILED ==="; \
		echo "error" > $(OUTPUT_DIR)/timing_status; \
		exit 1; \
	fi
	@echo "Fingerprint: $$(cat $(OUTPUT_DIR)/fingerprint)"

endif

# ===== COMMON TARGETS =====

sim: ## Run cocotb simulation suite
	@../scripts/sim.sh

waves: ## Generate VCD waveform (TARGET=test_xxx)
	@$(MAKE) -C $(SIM_DIR) waves$(if $(TARGET), TARGET=$(TARGET))$(if $(TESTCASE), TESTCASE=$(TESTCASE))

clean: ## Remove FPGA build artifacts
	rm -rf $(OUTPUT_DIR)/*
	rm -rf $(PROJECT_DIR)/pluto.xpr $(PROJECT_DIR)/pluto.gen \
		$(PROJECT_DIR)/pluto.runs $(PROJECT_DIR)/pluto.srcs \
		$(PROJECT_DIR)/pluto.cache $(PROJECT_DIR)/pluto.hw \
		$(PROJECT_DIR)/pluto.ip_user_files $(PROJECT_DIR)/pluto.sim
	@echo "Cleaned."

# ---- Dependency checks ----

_check_adi:
ifndef REMOTE
	@if [ ! -d "$(ADI_HDL_DIR)/projects/scripts" ]; then \
		echo "ERROR: ADI HDL submodule not initialized."; \
		echo "Run: git submodule update --init platform/styx"; \
		echo "Then: cd platform/styx && git submodule update --init fpga/extern/adi-hdl"; \
		exit 1; \
	fi
endif
