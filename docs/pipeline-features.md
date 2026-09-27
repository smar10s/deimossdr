# Pipeline Features

Compact reference of non-obvious implementation details in the RX pipeline.
Each entry describes what's there and why — a changelog of architectural
decisions that affect correctness and performance.

Register map: `docs/registers.md`

---

- **STF DC immunity** — HPF in stf_detect prevents false triggers from
  AD9361 DC offset during power-on/frequency changes.
- **BRAM collision fix** — DEPTH=82 in bram_delay_tap eliminates
  read-during-write collision at TAP3. DONT_TOUCH prevents BRAM merging.
- **Watchdog** — clears pipeline state on timeout, preventing lockup
  from non-DC false triggers or HT/VHT frames that stall the FSM.
- **STF rearm timeout** — 256-sample timeout clears `detected_latch` when
  pipeline has acked but metric stays above threshold (continuous signal).
  Prevents permanent lockup on busy channels. Sim-gated by
  `test_detected_latch_lockup_continuous_signal`. Hardware-gated by
  `loopback_test.sh -l 7` (EAPOL burst at SIFS timing).
- **STF internal self-ack** — `pipeline_ack_latch` set at detection time
  inside stf_detect (no external BD routing dependency). Eliminates the
  hardware-only failure where BD self-loop timing prevented rearm.
- **HIL 1-in-5 gating** — hil_ctrl gates `test_valid` to 1-per-5 clocks,
  matching live ADC rate. All downstream modules (correlator, windowed-max,
  stf_end timing) operate identically in HIL and live mode. Eliminates
  the class of bugs where 1-per-clock injection broke multi-stage pipelines.
- **Runtime STF threshold** — configurable sensitivity via
  `deimos_regs_axi` STF_THRESH (shift=0-7, 0=standard, 1=2x, 2=4x sensitive).
- **STF enable gate** — firmware-controlled datapath enable via
  `deimos_regs_axi` STF_ENABLE.
  Default 0 (held in reset on FPGA load). Firmware sets to 1 after
  AD9361 is configured at 20 MSPS. Toggle 0->1 for full accumulator
  reset. Prevents boot-time corruption from wrong sample rate.
- **AD9363 bandwidth** — 28 MHz (1.4x signal BW) for all firmware tools.
  Prevents band-edge EVM degradation on higher-rate subcarriers.
- **Sliding LTF correlator** — 4-engine, 16-tap cross-correlator.
  All-DSP (16 DSP48E1), registered tap pipeline for timing closure.
  Free-running, zero search latency. Acquisition runs independently of
  decode, so SIFS-spaced frames are never missed (D22/D24).
- **FFT direct feed** — fft64_sdf as external OOC module, connected via
  ports to decode_engine. No intermediate buffer.
- **OOC module isolation** — fft64_sdf, ltf_correlator, and ltf_peak_detect
  promoted to BD-level entities with independent OOC synthesis. Reduces
  decode_engine control set count for better slice packing.
- **Frame descriptor handoff** — acquisition pushes `{ltf_pos, phase_inc}`
  into `frame_fifo`; only accepted detections push, so a rejected trigger
  cannot desync the tag stream. `tag_abort` pops on L-SIG fail or watchdog
  timeout. See D22.
- **Narrow DDR decode window** — +/-40 sample LTF search window in firmware
  DDR decode. Covers observed ltf_delta range [-5, -36] without reaching
  into SIFS-adjacent frames (min inter-frame distance >400 samples).
- **5-bit Viterbi soft quantization** — internal quantizer uses /8 (range
  +/-15) instead of /4 (range +/-7). Doubles soft decision resolution for all
  modulations. Max branch metric 30 fits PM_WIDTH=11 with normalization.
- **16-QAM 2x LLR scaling** — demapper applies scale2_sat16/17 to all 16-QAM
  LLR outputs, matching the scale4 pattern used for 64-QAM. Eliminates the
  quantization floor where b1 (+/-20 unscaled) was crushed to +/-1 by the
  Viterbi quantizer. Rate 36 cable loopback: 75% -> 98%.
- **tag_abort on invalid rate** — S_CONFIG_DATA fires tag_abort when L-SIG
  decodes with valid parity but invalid rate code (e.g. 0000). Prevents
  offset queue desync from frames that pass parity check but have no valid
  rate mapping.
- **Frame-boundary state reset** — every cross-frame shared sequential
  register is reset explicitly at the frame/setup boundary that begins the
  new frame (the depuncturer's puncture-pattern position via
  `depunct_restart`, for example), never by relying on downstream drain
  timing, and never sharing a reset with per-symbol state. Violations are
  latent until chain timing changes. See D25.
- **Budget-derived module throughput** — a module's target is
  `budget / work_per_symbol`, set by the system's binding stage, not by the
  module's best case. The Viterbi ships at ~1.27 clk/pair (not 1.0) because
  rate 54 allows ~1.85; buying area for headroom the pipeline cannot use
  fails placement. See D23.
