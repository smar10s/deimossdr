#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# gen_sim_pipeline_test.py — unit tests for the BD->sim inversion tooling.
# Run: python3 scripts/gen_sim_pipeline_test.py

import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

BD = os.path.join(HERE, "..", "fpga", "project", "system_bd.tcl")
FPGA_TEST = os.path.join(HERE, "..", "fpga", "test")


def test_parse_counts():
    from bd_graph import parse_bd

    cells, edges = parse_bd(BD)
    assert cells["decode_engine_0"] == "decode_engine"
    assert cells["demapper_0"] == "demapper"
    assert len(cells) == 24  # 22 pipeline + deimos_regs_axi + tag_fifo_axi
    assert len(edges) == 230  # recorded baseline (2026-09-16); update only with a BD change


def test_const_values():
    from bd_graph import const_values

    cv = const_values(open(BD).read())
    assert cv["mixer_enable_const"] == (1, 1)
    assert cv["mixer_phase_reset_const"] == (1, 0)
    assert cv["fifo_flush_const"] == (1, 0)
    assert cv["fft_idx_zero"] == (6, 0)
    assert cv["diag_clip_zero"] == (16, 0)


def _srclist():
    return subprocess.run(
        ["make", "-s", "-C", FPGA_TEST, "print_rx_frontend_srcs"],
        capture_output=True, text=True, check=True).stdout.split()


def _xml(tmp):
    subprocess.run(
        ["verilator", "--xml-only", "--top-module", "rx_frontend",
         "--Mdir", tmp, "-Wno-DECLFILENAME", "-Wno-UNUSED", "-Wno-MULTITOP",
         "-Wno-PINCONNECTEMPTY", "-Wno-ASCRANGE", "-Wno-SYNCASYNCNET",
         "-Wno-fatal", *_srclist()],
        check=True, capture_output=True)
    return os.path.join(tmp, "Vrx_frontend.xml")


def test_ports():
    from verilog_ports import module_ports, instances

    with tempfile.TemporaryDirectory() as tmp:
        xml = _xml(tmp)
        p = module_ports(xml)
        assert p["deinterleaver"]["clk"].dir == "input"
        assert p["deinterleaver"]["valid_in"].width == 1
        assert p["decode_engine"]["ddr_wr_ptr"].width == 25
        assert p["demapper"]["soft_wide0"].width == 8
        rp = instances(xml, "rx_pipeline")
        assert rp["u_demapper"] == "demapper"
        assert rp["u_viterbi"] == "viterbi_k7"


def test_generated_view_is_fresh():
    r = subprocess.run(
        [sys.executable, os.path.join(HERE, "gen_sim_pipeline.py"),
         "--bd", BD, "--out-dir", os.path.join(HERE, "..", "fpga", "rtl"),
         "--check"],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"PASS {t.__name__}")
    print("ok")
