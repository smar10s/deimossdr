# OTA FCS Rate Analysis

Date: 2026-06-16
Build: 0x177deb14
Channel: 36 (5180 MHz), antenna, no gain control

> Measurement snapshot. Retained for the raw-vs-true FCS distinction, which
> is still valid and still the reason raw FCS% is not a fabric health metric.
> Absolute percentages are channel- and environment-dependent and will not
> reproduce exactly. Reproduce with `deimos_rx_dump` (see
> `firmware/tools/deimos_rx_dump.c`) and aggregate by `fcs`/`rate`/`len`.

## Summary

The raw FCS OK rate is 33-42%. This is misleading — it's diluted by HT/VHT
traffic that the fabric correctly rejects. The true legacy decode FCS rate
is **79-95%** depending on channel conditions.

## Traffic Breakdown (60s sample, 4692 frames)

| Category | Count | % |
|----------|------:|---:|
| HT/VHT correctly rejected | 2153 | 45% |
| Real legacy FCS OK | 2012 | 43% |
| Real legacy FCS FAIL | 527 | 11% |

**True legacy FCS rate: 79%**

## HT/VHT Recognition Signatures

HT/VHT frames pass the L-SIG parity/tail/rate checks (they use valid
rate=6 L-SIG), so the fabric processes them through DATA. FCS always
fails because the bits after L-SIG are HT-SIG/HT-DATA, not legacy DATA.

Known always-fail patterns (never pass FCS):

| Rate | Length | Inter-arrival | Notes |
|------|--------|---------------|-------|
| 6 | 27 | ~82ms | Most common HT L-SIG duration |
| 6 | 30 | ~146ms | Second most common |
| 6 | 21, 18, 24, 33, 36, 39, 48, 57 | varies | Other HT durations |
| 12 | 20 | — | Possible SIGNAL misread from HT-SIG |
| 24 | 32 | — | Possible SIGNAL misread from HT-SIG |
| 24 | 28 | — | Same pattern |
| 9, 18, 36, 48, 54 | random large | — | Garbled HT-SIG decoded as SIGNAL |

These account for 45-48% of all STF-triggered frames that produce a tag.

## Real Legacy Performance

### By frame type (rate 6)

| Length | Frame type | FCS rate | Notes |
|--------|-----------|----------|-------|
| 371-380 | Beacons (several APs) | 94-100% | Drops to ~70% during fading |
| 23 | ACK/control | 100% | Perfect |
| 28 | ACK/control | 100% | Perfect |
| 14 | ACK/control | 100% | Perfect |
| 20 | Null/control | 100% | Perfect |

### By rate

| Rate | Typical FCS | Notes |
|------|------------|-------|
| 6 (short) | 100% | Frames ≤50 bytes |
| 6 (beacons) | 94-100% | ~370-380 byte frames |
| 12 (short) | 87-97% | len=14, 30 |
| 24 (ACKs) | 99% | len=20 |
| 24 (len=14) | 71% | Marginal signal |

### Remaining real failures

~21% of real legacy traffic fails FCS. Sources:

1. **Weak/distant APs** — rate 12 len=28 (5% pass), rate 24 len=14 (71% pass)
2. **Fading dips** — beacon failures cluster temporally (5s windows)
3. **Co-channel interference** — rare large frames (len=201, 914)

## Why Raw Rate Varies Between Sessions

The raw FCS% swings based on **traffic mix**, not fabric performance:

- Quiet channel (beacons dominate) → raw 40-50%
- Active HT/VHT data bursts → raw 18-25%
- The fabric decode rate on legacy traffic is stable

## Conclusion

The ~21% real legacy failure rate is acceptable for OTA with no AGC and
passive antenna reception. Primary causes are channel fading and weak
signals, not pipeline bugs. The fabric decodes all cable-loopback frames
perfectly (20/20 at rates 6-36), confirming the decode pipeline is correct.

Improving OTA legacy FCS further would require:
- AGC (hardware change)
- Better antenna/placement
- MRC diversity (hardware change)

None of these are in scope. The current rate is sufficient for EAPOL capture.
