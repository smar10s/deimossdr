# SPDX-License-Identifier: MIT
# Makefile — Deimos top-level build orchestration
#
# Layers on top of Styx foundation. Delegates FPGA build to fpga/Makefile
# (which delegates to styx build.tcl), cross-compiles firmware via Docker,
# and packages everything into a flashable pluto.frm.
#
# Quick reference:
#   make setup        — One-time: init all submodules, build plutosdr-fw (~2hrs)
#   make bitstream    — Build FPGA bitstream (local or remote via config.mk)
#   make firmware     — Cross-compile ARM firmware (Docker)
#   make firmware-clean — Remove firmware build artifacts
#   make package      — Create flashable pluto.frm (bitstream + kernel + rootfs)
#   make flash        — Flash pluto.frm to PlutoSDR
#   make validate     — Post-flash validation (AD9361, fingerprint)
#   make deploy       — Deploy firmware binaries to PlutoSDR via SCP
#   make sim          — Run FPGA simulation suite
#   make test         — Run firmware host unit tests (no hardware)
#   make waves        — Generate VCD waveforms (use TARGET=test_xxx)
#   make all          — bitstream + firmware + package
#   make clean        — Remove all build artifacts

.PHONY: all setup remote-setup bitstream firmware firmware-clean package \
        flash validate deploy sim test waves clean help hooks

PLUTO_IP       ?= 192.168.2.1
PLUTO_USER     ?= root
PLUTO_PASS     ?= analog
PLUTO_SSH_OPTS  = -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null

# Configuration (shared with fpga/Makefile)
-include config.mk
REMOTE_PORT    ?= 22
REMOTE_USER    ?= $(USER)
REMOTE_DIR     ?= ~/deimos
ifdef REMOTE
  SSH = ssh -p $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE)
  RSYNC = rsync -az --delete --exclude-from=.rsync-exclude -e "ssh -p $(REMOTE_PORT)"
endif

STYX_DIR       := platform/styx
PLUTOSDR_FW    := $(STYX_DIR)/extern/plutosdr-fw
FW_DOCKER_IMG  := deimos-firmware
FW_BUILD_DIR   := build/firmware
HOST_BUILD_DIR := build/host

all: bitstream firmware package ## Build everything (bitstream + firmware + package)

# ============================================================================
# Setup (one-time)
# ============================================================================

setup: ## One-time: init all submodules + build plutosdr-fw (~2hrs first run)
	@echo "=== Initializing styx submodules ==="
	git -C $(STYX_DIR) submodule update --init fpga/extern/adi-hdl
	git -C $(STYX_DIR) submodule update --init extern/plutosdr-fw
	@echo "=== Building ADI HDL IP library ==="
	$(MAKE) -C $(STYX_DIR)/fpga/extern/adi-hdl/library -j$$(nproc 2>/dev/null || sysctl -n hw.ncpu || echo 4)
	@echo "=== Initializing plutosdr-fw dependencies ==="
	cd $(PLUTOSDR_FW) && git submodule update --init --recursive || true
	@echo "=== Building PlutoSDR firmware base (kernel + rootfs) ==="
	@echo "    First run takes 1-2 hours. Subsequent runs are cached."
	@$(MAKE) _build_plutosdr_fw
	@echo "=== Setup complete ==="

remote-setup: ## One-time: sync + build plutosdr-fw on remote build host
ifdef REMOTE
	@echo "=== Remote setup on $(REMOTE_USER)@$(REMOTE):$(REMOTE_PORT) ==="
	@echo "    Syncing source tree..."
	$(RSYNC) ./ $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/
	@echo "    Running make setup on remote (~2 hours first time)..."
	$(SSH) "cd $(REMOTE_DIR) && make setup REMOTE="
else
	@echo "ERROR: REMOTE not set. Configure config.mk or pass REMOTE=host"
	@exit 1
endif

_build_plutosdr_fw:
	@if [ ! -f "$(PLUTOSDR_FW)/buildroot/output/images/rootfs.cpio.gz" ]; then \
		echo "Building plutosdr-fw (kernel + rootfs)..."; \
		if command -v docker >/dev/null 2>&1; then \
			echo "  Using Docker..."; \
			docker build -t plutosdr-fw-builder -f firmware/docker/Dockerfile.plutosdr-fw firmware/docker/; \
			docker run --rm -v "$(abspath $(PLUTOSDR_FW)):/src" plutosdr-fw-builder \
				make -C /src TARGET=pluto -j$$(nproc 2>/dev/null || sysctl -n hw.ncpu || echo 4); \
		else \
			echo "  Docker not found, building natively..."; \
			$(MAKE) -C $(PLUTOSDR_FW) TARGET=pluto -j$$(nproc 2>/dev/null || echo 4); \
		fi; \
	else \
		echo "plutosdr-fw already built (cached)."; \
	fi

# ============================================================================
# FPGA
# ============================================================================

bitstream: ## Build FPGA bitstream (delegates to fpga/Makefile)
	@$(MAKE) -C fpga bitstream

sim: ## Run FPGA simulation suite
	@./scripts/sim.sh

test: ## Run firmware host unit tests (no hardware)
	@echo "=== Firmware host unit tests ==="
	@cmake -B $(HOST_BUILD_DIR) -S firmware \
		-DDEIMOS_BUILD_TOOLS=OFF -DCMAKE_BUILD_TYPE=Debug
	@cmake --build $(HOST_BUILD_DIR) --parallel
	@ctest --test-dir $(HOST_BUILD_DIR) --output-on-failure
	@echo "=== Host tests passed ==="

waves: ## Generate VCD waveform (use TARGET=test_xxx)
	@$(MAKE) -C fpga waves$(if $(TARGET), TARGET=$(TARGET))$(if $(TESTCASE), TESTCASE=$(TESTCASE))

# ============================================================================
# Firmware (ARM cross-compile via Docker)
# ============================================================================

firmware: _check_lib80211 ## Cross-compile ARM firmware
	@echo "=== Building firmware (ARM cross-compile) ==="
	@if ! docker image inspect $(FW_DOCKER_IMG) >/dev/null 2>&1; then \
		docker build -t $(FW_DOCKER_IMG) -f firmware/docker/Dockerfile firmware/docker/; \
	fi
	@if [ ! -f "$(FW_BUILD_DIR)/CMakeCache.txt" ]; then \
		docker run --rm -v "$(CURDIR):/src" $(FW_DOCKER_IMG) \
			cmake -B /src/$(FW_BUILD_DIR) -S /src/firmware \
			-DCMAKE_TOOLCHAIN_FILE=/src/firmware/cmake/toolchain-pluto.cmake \
			-DCMAKE_BUILD_TYPE=Release; \
	fi
	docker run --rm -v "$(CURDIR):/src" $(FW_DOCKER_IMG) \
		cmake --build /src/$(FW_BUILD_DIR) --parallel
	@echo "=== Firmware build complete ==="
	@ls $(FW_BUILD_DIR)/tools/ 2>/dev/null | grep -v CMake || true

firmware-clean: ## Remove firmware build artifacts
	rm -rf $(FW_BUILD_DIR)

# ============================================================================
# Packaging + Deployment
# ============================================================================

package: build/fpga/system_top.bit ## Create pluto.frm
	@if [ ! -f build/fpga/timing_status ]; then \
		echo "ERROR: Cannot package — missing build/fpga/timing_status"; \
		echo "Run 'make bitstream' to completion before packaging."; \
		exit 1; \
	fi
	@if [ "$$(cat build/fpga/timing_status)" != "met" ]; then \
		echo "ERROR: Cannot package — timing status is '$$(cat build/fpga/timing_status)'"; \
		echo "Fix timing violations before packaging."; \
		exit 1; \
	fi
ifdef REMOTE
	@echo "=== Packaging on remote ($(REMOTE)) ==="
	$(SSH) "cd $(REMOTE_DIR) && packaging/package.sh build/fpga"
	@mkdir -p build/fpga
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/pluto.frm build/fpga/
	@echo "=== Downloaded: build/fpga/pluto.frm ==="
else
	@echo "=== Packaging pluto.frm ==="
	@$(MAKE) _package_frm
	@echo "=== Package complete: build/fpga/pluto.frm ==="
endif

_package_frm:
	@if [ ! -f "$(PLUTOSDR_FW)/buildroot/output/images/rootfs.cpio.gz" ]; then \
		echo "ERROR: plutosdr-fw not built. Run 'make setup' first."; exit 1; \
	fi
	@if [ ! -f "build/fpga/system_top.bit" ]; then \
		echo "ERROR: No bitstream. Run 'make bitstream' first."; exit 1; \
	fi
	packaging/package.sh build/fpga

flash: package ## Flash a freshly packaged pluto.frm to PlutoSDR
	@./bin/flash.sh build/fpga/pluto.frm

validate: ## Validate PlutoSDR after flash (AD9361, fingerprint)
	@./bin/validate.sh

hooks: ## Install git hooks from scripts/hooks/ (blocks RTL commits to main)
	@git rev-parse --git-dir >/dev/null 2>&1 || { \
		echo "ERROR: not a git repository."; exit 1; }
	@mkdir -p "$$(git rev-parse --git-dir)/hooks"
	@for h in scripts/hooks/*; do \
		[ -f "$$h" ] || continue; \
		install -m 755 "$$h" "$$(git rev-parse --git-dir)/hooks/$$(basename $$h)"; \
		echo "  installed $$(basename $$h)"; \
	done
	@echo "=== Git hooks installed ==="

deploy: ## Deploy firmware binaries + golden vectors to PlutoSDR via SCP
	@echo "=== Deploying firmware to $(PLUTO_IP) ==="
	@if [ ! -d "$(FW_BUILD_DIR)/tools" ]; then \
		echo "ERROR: No firmware build. Run 'make firmware' first."; exit 1; \
	fi
	@echo "  Deploying deimos tools..."
	@for bin in deimos_hil_inject deimos_fabric_loopback deimos_adc_capture \
	            deimos_burst_loopback deimos_rx_dump; do \
		if [ -f "$(FW_BUILD_DIR)/tools/$$bin" ]; then \
			echo "    $$bin"; \
			sshpass -p $(PLUTO_PASS) scp -O $(PLUTO_SSH_OPTS) \
				"$(FW_BUILD_DIR)/tools/$$bin" $(PLUTO_USER)@$(PLUTO_IP):/usr/bin/; \
		fi; \
	done
	@if [ -d "$(FW_BUILD_DIR)/tools/platform" ]; then \
		echo "  Deploying platform tools..."; \
		for bin in pluto_loopback pluto_sigladder pluto_dma_test pluto_stream_test; do \
			if [ -f "$(FW_BUILD_DIR)/tools/platform/$$bin" ]; then \
				echo "    $$bin"; \
				sshpass -p $(PLUTO_PASS) scp -O $(PLUTO_SSH_OPTS) \
					"$(FW_BUILD_DIR)/tools/platform/$$bin" $(PLUTO_USER)@$(PLUTO_IP):/usr/bin/; \
			fi; \
		done; \
	fi
	@if [ -d "extern/lib80211/vectors" ]; then \
		echo "  Deploying golden vectors..."; \
		sshpass -p $(PLUTO_PASS) ssh $(PLUTO_SSH_OPTS) \
			$(PLUTO_USER)@$(PLUTO_IP) "mkdir -p /usr/share/deimos/vectors"; \
		for vec in extern/lib80211/vectors/legacy_*_waveform.json; do \
			echo "    $$(basename $$vec)"; \
			sshpass -p $(PLUTO_PASS) scp -O $(PLUTO_SSH_OPTS) \
				"$$vec" $(PLUTO_USER)@$(PLUTO_IP):/usr/share/deimos/vectors/; \
		done; \
	fi
	@echo "=== Deploy complete ==="

# ============================================================================
# Cleanup
# ============================================================================

clean: ## Remove all build artifacts
	@$(MAKE) -C fpga clean
	rm -rf $(FW_BUILD_DIR)
	rm -rf $(HOST_BUILD_DIR)
	rm -f build/fpga/pluto.frm
	@echo "Cleaned."

# ============================================================================
# Dependency checks
# ============================================================================

_check_lib80211:
	@if [ ! -f "extern/lib80211/CMakeLists.txt" ]; then \
		echo "Initializing lib80211 submodule..."; \
		git submodule update --init extern/lib80211; \
	fi

# ============================================================================
# Help
# ============================================================================

help: ## Show available targets
	@grep -hE '^[a-zA-Z_-]+:.*?## .*$$' Makefile | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'
