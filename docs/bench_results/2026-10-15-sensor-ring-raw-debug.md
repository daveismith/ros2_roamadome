# Sensor ring raw-mask debug, 2026-10-15 (dome installed)

Purpose: find the physical origin of the position jumps seen in the with-dome
sensor sweeps ([2026-10-14-with-dome.md](2026-10-14-with-dome.md),
[2026-10-15-with-dome-reseated.md](2026-10-15-with-dome-reseated.md)). The
jumps decode to single sensor bits (bit 0, then bit 1) reading "set" where
the code table expects "clear". This session read the ring's own debug
stream to see the raw sensor states instead of the decoded, median-filtered
angle that reaches the controller and ROS.

## Setup

- Sensor ring console on `/dev/ttyUSB0` at 115200, `#DPDEBUG1` on, `#DPDEBUG0`
  off. In debug mode the ring prints one line per sensor-mask change (plus
  about 10 lines/s when nothing changes):
  `POS: <raw angle>  s8 s7 ... s0 : <filtered angle>`, with `B` meaning the
  sensor reads 1 (the bit is set in the decode mask) and `-` meaning 0. The
  printed order is `SENSORS[8]` first, `SENSORS[0]` last; this was verified
  by decoding every printed mask with the `sDomeAngle[]` table from
  `Reeltwo/src/drive/DomeSensorRing.h` (5000 of 5000 samples matched only
  with that order).
- Dome controller on `/dev/ttyACM0`, driven directly with `:DPR` and `:DPA`
  (no ROS), `#DPREPORT28` on, every `DOME POSITION` line logged with a host
  timestamp. Firmware settings unchanged throughout (MaxSpeed 80, Scaling off,
  HomePos 0, so the controller's angle equals the ring's filtered angle).
- Scripts and raw logs: `/home/pi/dome_bench/ring_debug/`
  (`record_ring.py`, `drive_controller.py`, `drive_slow.py`, `static_scan.py`,
  `analyze_ring.py`; `ring_run1.csv`/`ctl_run1.csv` at 35 % and 50 % both
  directions, `ring_run2.csv`/`ctl_run2.csv` at 33 %, `ring_static*.csv`/
  `ctl_static*.csv` for the parked holds).

## Rotating: raw glitches recur every pass at fixed angles

A glitch is a raw decoded angle more than 6 degrees from the previous sample
that returns within a few samples. 39 glitches in run 1 (35 % and 50 %), 23 in
run 2 (33 %). "+%" turns the reported angle down, "-%" turns it up.

| True angle | Direction | Spurious bit | Reads as | Passes with glitch | Notes |
|---|---|---|---|---|---|
| 198-200 | angle increasing (-%) | bit 0 set | 11-13 | every pass at 33, 35, 50 % | 2-3 raw samples wide; once also at +35 % |
| 235-237 | angle increasing (-%) | bit 0 set | 274-276 | most passes | 1-2 samples |
| 253 | angle decreasing (+%) | bit 0 set | 260 | every pass | 1 sample; 7 degree jump |
| 241-242 | angle decreasing (+%) | bit 1 set | 52-57 | most passes | often together with a legitimate change of bit 8 or 5 |
| 277 | angle decreasing (+%) | bit 1 set | 316 | every pass | 1 sample |
| 290-294 | angle decreasing (+%) | bit 1 set | | every pass at 33 % | |

Everything else in the raw stream (about 470 legitimate transitions per bit
per run) decodes to adjacent angles. The firmware's median-of-5 filter removes
most of these glitches before they reach the controller: in run 1 only one
reached the `DOME POSITION` stream (238 -> 274 -> 238 at +35 %). They get
through when the dome is slow enough that a glitch spans three ring samples,
which is why the bench sensor sweeps saw them mostly at 35 % and 50 %.

## Parked: sensor 0 is wrong at rest at two of the angles

The dome was parked (with short drive pulses, then held) inside each glitch
zone, one degree at a time, and the raw mask watched for 5-6 s at rest.

| Parked at (controller) | Raw majority mask, bit 0..8 | Decodes to | Behaviour at rest |
|---|---|---|---|
| 195, 196, 197 | matches the code for 196, 197, 198 | | clean |
| **198** | `B---BBB--` | **13** | **bit 0 toggling: 1103 of 1615 samples read 13, 512 read 198; the ring printed about 290 lines/s because the bit flips every few ms** |
| 235, 236, 237, 238, 240, 241 | matches | | clean |
| **252** | `BBBB--B--` | **260** | **bit 0 set in all 165 samples, no flicker** |
| 254 | matches 255 | | clean |
| 276, 279, 289, 292 | valid codes (276, 279, 287, 300) | | clean, no flicker; sensor 1 shows no static fault |
| 207, 208 (earlier) | matches | | clean |

So sensor 0 (`SENSORS[0]`, mask bit 0) reads 1 at rest at 198 degrees (marginal,
flickering) and at 252-253 degrees (solidly), where its track must read 0. The
same sensor's dynamic glitch at 235-237 did not reproduce parked, nor did any
sensor 1 glitch; those appear only while moving.

Two firmware details matter for interpreting this:

- `DomeSensorRing::getAngle()` appends a sample to its median buffer only
  when the mask changes and drops masks that decode to -1. A wrong but valid
  code held steadily at rest therefore never propagates (the parked 252 read
  260 for 6 s while the controller kept reporting 252), and a wrong code
  during motion is one sample among the changing ones.
- A glitch that is held for three or more ring samples passes the
  median-of-5 in `DomeSensorRingSerialListener`.

## Interpretation

- The fault is at fixed positions on the ring, not a random electrical
  glitch: the same angles every pass, in both rotating runs, and two of them
  reproduce with the dome stationary.
- It is one sensor, `SENSORS[0]`, with `SENSORS[1]` (the adjacent channel)
  starting to show the same thing dynamically. Without the dome, 280
  revolutions showed none of it; with the dome, re-seating did not change the
  angles.
- Reading 1 where the track should read 0 is what a reflective sensor
  returns when it sees too little reflection: a dark mark, a gap that has
  opened, or the sensor tilted so its spot leaves the track. The dome's weight
  changing the ring-to-sensor geometry at specific angles (runout, a low spot
  on the ring, or the sensor board tilting) fits all of the evidence,
  including the direction dependence of the dynamic glitches and the two
  adjacent channels being affected.

## What to check on the hardware

1. With the dome on, park at 198 and at 252 (`:DPA198`, `:DPA252`, or the
   `static_scan.py` script) and watch `#DPDEBUG1`: bit 0 (the last symbol on
   the line) should read `-`. Then lift or unload the dome without moving the
   ring: if the symbol goes clean, it is load geometry (gap or tilt), not a
   mark on the track.
2. Measure the sensor-to-ring gap at 198, 236 and 252 against a clean angle
   such as 207, with the dome on. Sensors 0 and 1 are the pair to look at.
3. Inspect sensor 0's track at 198 and 252-253 for dirt, a scratch or a
   lifted mark; clean it and re-park.
4. Check the sensor board mounting and the cable to channels 0 and 1 for
   anything that flexes with the dome on.

After the fix, re-run only the sensor sweep (`docs/bench_results` has the
procedure) and confirm `suggested_pass` true.

## Software mitigation (independent of the fix)

The glitches are single-bit, non-adjacent decoded steps. A rule in the ring
firmware before the median (`DomeSensorRing::getAngle`): if the mask differs
from the last mask by one bit but the decoded angle is more than a few
degrees from the last angle, hold the last angle, count a sensor error, and
only accept the new value if it persists for several samples. The same gate
in `DomeSensorRingSerialListener` protects the controller if the ring cannot
be reflashed. Both live in the Reeltwo library (`reeltwo/Reeltwo`,
`src/drive/`), pulled unpinned by `DomeControlFirmware/platformio.ini`. The
ROS plugin's `PositionTracker` can apply the same rule to the reported
angle to protect `unwrapped_rad` and position control.
