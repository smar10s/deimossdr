# AD9361 Gain Mode Gotcha

Date: 2026-06-17
Severity: Critical (causes complete OTA decode failure)

## Symptom

OTA FCS rate collapses from 94-100% (beacons) to 0-10%. Short frames
may intermittently pass. Cable loopback still works perfectly. Gain
sweep shows no improvement. Appears to be "environmental" but isn't.

## Root Cause

The AD9361 driver defaults to `slow_attack` (AGC) gain control mode
after a cold boot. In AGC mode, manual gain writes via sysfs are
**silently rejected** (`EOPNOTSUPP`), but only if you check the return
value — most shell pipelines don't.

The sequence that breaks things:

```
# After cold boot, driver defaults to slow_attack mode
echo 50 > /sys/bus/iio/devices/iio:device1/in_voltage0_hardwaregain
# ^^^ SILENTLY FAILS — returns "write error: Operation not supported"
# Gain stays at whatever AGC decided (typically 62 dB = max)
```

The AGC in `slow_attack` mode sees mostly noise (WiFi frames have low
duty cycle), settles at max gain, and may cause transient/settling
artifacts during actual frame reception that destroy decode quality.

## Why It's Insidious

1. `cat in_voltage0_gain_control_mode` shows `slow_attack` but you
   don't think to check because the capture tool supposedly sets it.
2. If the capture tool was killed (by trap, test script, or manual),
   the sysfs state persists as whatever was last set by whoever last
   wrote it.
3. The gain readback shows 62 dB which looks "reasonable" for high-gain
   OTA reception — you don't immediately realize it's wrong.
4. RSSI reads 112-118 dB (very weak) which looks like an antenna or
   signal problem, not a gain mode problem.

## The Fix

**Always set gain mode BEFORE setting gain value.** The correct
two-step sequence:

```bash
DEV=/sys/bus/iio/devices/iio:device1
echo manual > $DEV/in_voltage0_gain_control_mode
echo 50 > $DEV/in_voltage0_hardwaregain
```

The firmware does this correctly in `deimos_rx_configure_radio()`
(`firmware/src/deimos_rx.c:86-98`): `hal_ad9361_set_rx_gain_mode("manual")`
then `hal_ad9361_set_rx_gain()`. Any tool built on it (`deimos_rx_dump`,
`deimos_fabric_loopback`) inherits the correct order. The problem only
occurs when something configures the radio via sysfs directly without
setting the mode first.

OTA is the exception: `deimos_rx_dump` intentionally uses `fast_attack` AGC
(not manual) because multiple transmitters at varying distances need per-frame
gain tracking (D27). It has no gain option.

## Diagnostic Checklist

If OTA performance suddenly collapses:

```bash
# 1. Check gain mode (OTA: "fast_attack", D27; cable tools: "manual")
cat /sys/bus/iio/devices/iio:device1/in_voltage0_gain_control_mode

# 2. Check actual gain (should match what you set)
cat /sys/bus/iio/devices/iio:device1/in_voltage0_hardwaregain

# 3. Try writing gain — if this fails, mode is wrong
echo 50 > /sys/bus/iio/devices/iio:device1/in_voltage0_hardwaregain 2>&1

# 4. Fix: set mode first, then gain
echo manual > /sys/bus/iio/devices/iio:device1/in_voltage0_gain_control_mode
echo 50 > /sys/bus/iio/devices/iio:device1/in_voltage0_hardwaregain
```

## When This Bites

- After cold boot (driver defaults to slow_attack)
- After killing a capture tool and trying manual radio config
- After `deimos_fabric_loopback` or other tools that might set
  different gain modes
- After `session_start.sh` completes (leaves radio configured for
  cable loopback, not OTA)

## Prevention

Always let the firmware configure the radio — `deimos_rx_dump` for OTA
runs the full initialization sequence in the correct order. If you must
configure manually without a capture running, use the two-step sequence
above: mode first, then gain.
