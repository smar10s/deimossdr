# Stimulus — Generated Test Input

`stimulus/` holds deterministic, seeded IQ streams that gate the decode
pipeline. `captures/` holds real ADC recordings (cable loopback etc.);
`stimulus/` holds *generated* equivalents. Naming follows hardware
verification convention: stimulus = generated input, capture = recorded
field data. This split replaces an earlier policy where sensitive OTA
captures (real AP beacons, a real 4-way EAPOL handshake containing
ANonce/SNonce/MIC key material) lived in `captures/passing/`.

## Files

| File | Models | Gate |
|------|--------|------|
| `passing/eapol_4way_burst.json` | 14 decodable legacy frames (6 Mbps) + 1 HT/VHT stand-in (L-SIG abort, no tag). STF-to-STF deltas measured from the removed OTA capture: EAPOLs 7.2/42.8/1.2 ms apart, but M2/M4 ride tight ACK chains and every ACK follows its data at the original channel's spacing. | `test_eapol_burst_continuous` (>=14 FCS-OK, 4 EAPOL matched, stand-in must NOT decode); HIL L7 replay |
| `passing/eapol_m1.json` | EAPOL M1-shaped (6 Mbps, L-SIG 137) | `test_stimulus_replay_passing` |
| `passing/eapol_m2.json` | EAPOL M2-shaped (6 Mbps, L-SIG 159) | `test_stimulus_replay_passing` |
| `passing/eapol_m3.json` | EAPOL M3-shaped (6 Mbps, L-SIG 193) | `test_stimulus_replay_passing` |
| `passing/eapol_m4.json` | EAPOL M4-shaped (6 Mbps, L-SIG 137) | `test_stimulus_replay_passing` |

## Regeneration

All files are produced by one generator with a fixed seed:

    python3 scripts/gen_stimulus.py            # write files
    python3 scripts/gen_stimulus.py --verify   # byte-compare vs committed

Files are committed (not gitignored) because hardware scripts deploy them
to the Pluto without Python. Treat them as build artifacts: if a file
changes, `--verify` must pass.

## Schema

Same shape as the removed captures: `real`/`imag` (12-bit ints),
`stf_offsets`, `expected_frames` (`rate_code`, `length`, `psdu_hex` —
payload WITHOUT FCS; `length` = payload + 4), plus `n_frames`,
`rate_mbps`, `scale_factor`, `peak_amplitude`. The burst adds
`abort_frames: [{stf_offset, type}]` for frames that must NOT decode.

`rate_code` uses lib80211's LSB-first convention: 6 Mbps = `0b1011`.

## Policy

- Real OTA captures (live traffic, real MACs, key material) must NEVER be
  committed. `captures/*eapol*.json` is gitignored.
- If a gate needs a new waveform shape, add a scenario to
  `scripts/gen_stimulus.py` — do not hand-craft JSON.
- Do not "normalize" the burst gap schedule to uniform SIFS/DIFS. The
  measured structure (tight ACK chains + ms-scale EAPOL gaps) is what
  the test covers.
