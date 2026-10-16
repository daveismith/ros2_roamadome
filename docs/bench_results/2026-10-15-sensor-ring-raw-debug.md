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

## Addendum: the faults map to three spots on the shared track

The ring is one track read by nine sensors spaced 40 degrees apart. The decode
table proves it: bit `i` of the code at angle `A` equals the track value at
`A - 40 i` for all 360 angles, and the track has 40 edges (20 marks), the 20
paths of `Patterns/DomeSensorCompact.svg` in `reeltwo/DomeSensorFirmware32`.

Converting every spurious raw reading to its track position (`A - 40 i`):

| Track position | Sensor 0 spurious "B" | Sensor 1 spurious "B" | Sensors 2-8 |
|---|---|---|---|
| 198-201 | 18 | 5 | none |
| 234-237 | 6 | 9 | none |
| 252 | 6 | 5 | none |

Sensor 1 sees each spot 40 degrees of dome rotation after sensor 0 (dome
angles 241, 277, 292 against 198-201, 236, 253). The defect is on the ring at
those three positions; sensors 0 and 1 are the two with the least margin,
since the other seven read the same spots cleanly. The spots are 4-10 degrees
from the nearest track edge (edges at 194, 227, 259), so a pattern edge parked
over a sensor is not the cause; the sensors sitting on edges at those angles
(6 and 7 at 198, 1 and 3 at 252) did not flicker.

Pins: the ring firmware uses `SENSORS[]` from Reeltwo's `DomeSensorRing.h`;
for the ESP32 build without `PIN_NEOPIXEL` that is `{18, 19, 21, 22, 32, 33,
25, 26, 27}`, so bit 0 is GPIO 18 and bit 1 is GPIO 19 (the firmware's
self-test prints them 1-based as sensors 1 and 2). The inputs are pulled up; a
reflective sensor pulls its line low on reflection, so a raw 1 ("B") means too
little reflection.

To inspect: park at 198 and at 252, the spots are under sensor 0 (GPIO 18);
at 238 and 292 they are under sensor 1 (GPIO 19). Compare gap and track
surface there with a clean angle such as 207, and the gap at sensors 0 and 1
with the other seven.

## Addendum 2: the same spots glitch without the dome

Repeated the rotation run (35 % and 50 %, both directions) with the dome shell
removed, same scripts (`ring_nodome_run.csv`, `ctl_nodome_run.csv`), and the
parked holds at 253 and near 198 (`ring_nodome_static.csv`).

| Spurious "B" by track position | With dome (runs 1+2, 12399 samples) | No dome (8684 samples) |
|---|---|---|
| sensor 0 at 198-201 | 18 | 11 |
| sensor 0 at 234-237 | 6 | 0 |
| sensor 0 at 252 | 6 | 4 |
| sensor 1 at 198-201 | 5 | 7 |
| sensor 1 at 237 | 9 | 4 |
| sensor 1 at 252 | 5 | 4 |
| raw glitches per 1000 samples | 5.0 | 4.3 |

Parked at 253 without the dome the raw mask is correct and steady
(`-BBB--B--`); with the dome it read solidly wrong (`BBBB--B--`, 260). The
park at 198 did not converge without the dome (the bare platform overshoots
the drive pulses), so that spot was only observed in motion.

Conclusion: the defect at track positions 198-201, 234-237 and 252 exists
with or without the dome, at about the same rate of raw glitches per
revolution. The dome makes each glitch last longer (long enough to pass the
median-of-5 and, at 252, to hold at rest), which is why the no-dome bench
sweeps showed zero jumps and the with-dome sweeps showed many. The physical
spots to inspect are the same three, under sensor 0 (GPIO 18) when the dome
reads 198 and 252, and under sensor 1 (GPIO 19) at 238 and 292.

## Addendum 3 (2026-10-16): new sensor plate, no dome

The user replaced the plate that carries the pattern with a stiffer one. Same
rotation run as before (35 % and 50 %, both directions, no dome;
`ring_newplate_run.csv`, about 25 revolutions, 8010 ring samples).

| Spurious "B" by track position | Old plate, no dome | New plate, no dome |
|---|---|---|
| sensor 0 at 198-201 / 234-237 / 252 | 11 / 0 / 4 | 0 / 0 / 0 |
| sensor 1 at 198-201 / 237 / 252 | 7 / 4 / 4 | 0 / 0 / 0 |
| raw glitches total | 37 | 13 |

Sensors 0 and 1 are clean: no spurious reading anywhere on the track, and the
per-sensor mismatch statistics for sensors 0 and 1 now match the other seven.
The three bad spots were on the old plate.

The 13 remaining glitches are all the same event: going in the
angle-decreasing direction through dome angle 264, sensor 3 sets one sample
before sensor 1 clears, so the raw code reads 154 (`BBBBBBB--`) for one
sample between 265 and 263 instead of the adjacent code for 264. This is a
race between two pattern edges that lie within a degree of each other (track
edges 144 under sensor 3 and 225 under sensor 1 at that dome angle); the
code is not a Gray code, so the order in which they switch decides whether
the intermediate code is adjacent or far. It is one sample wide, only in one
direction, and the ring's median filter already removes it (the filtered
angle stayed at 266). The old plate had the same class of event at other
angles (sensors 3, 4 and 6). It is a property of the pattern and the
alignment tolerance, not a defect, but it is the reason a one-bit plausibility
gate in the ring firmware would still be worthwhile.

Note: the controller's `#DPREPORT28` was swallowed by its boot banner at the
start of this run, so there are no controller-side reports for it; the raw
ring data and the phase timestamps are sufficient for this analysis.
