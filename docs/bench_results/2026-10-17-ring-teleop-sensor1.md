# 2026-10-17 late: sensor 1 misreads between 40 and 120 deg (raw ring, Foxglove teleop)

After the realignment and the MinSpeed study, the robot launch's log showed the plugin's
plausibility gate rejecting and "confirming" position jumps while the dome was driven slowly
by teleop (`Rejected implausible position report 43 deg (the next report, 130 deg ...)`,
`Position jump to 133 deg confirmed ...`). In ten minutes of teleop the gate counted 25 held,
10 rejected and 15 confirmed jumps; the firmware's own sensor error counter stood at 6. The
43 <-> 130 deg pair is a single bit in the decode table: bit 1, sensor 1 (GPIO 19), one of the
two sensors that misread on the old plate.

## The recording

The user drove the dome slowly from Foxglove (no sticky feel reported) while the sensor ring's
raw debug stream on `/dev/ttyUSB0` (`#DPDEBUG1`, `POS: raw s8..s0 : filtered`, 10 Hz) and the
controller's reported angle (`/dome_state_broadcaster/joint_states`, 10 Hz) were recorded:
3327 ring samples over 136 s, every 10 deg bin covered. Raw CSV next to this file
(`2026-10-17-ring-teleop-sensor1-raw.csv`, columns `t,raw,bits,filtered`; the bit string is
printed most significant first, s8 to s0, so string index 7 is sensor 1 and index 8 is sensor 0).

## Result: sensor 1 flickers at several angles

- **94 implausible raw steps** (more than 25 deg between consecutive samples). 53 of them flip
  sensor 1 alone, and sensor 1 is part of 24 more; sensor 0 alone 3, sensor 7 (string index 1,
  always together with sensor 1 or 0) 13, others 5.
- They recur at the same true angles on every pass: about **43 to 44 deg** (reads 130 or 138),
  **48** (reads 92), **78** (118 reads 77), **87** (reads 36), **95 to 98** (reads 355). All
  within 40 to 120 deg of ring angle.
- **25 invalid codes** (the ring prints `---`), mostly around 93 to 99 deg.
- **The ring's own median filter did not remove them**: the filtered angle jumped by more than
  25 deg 140 times, and the controller's reported angle, as the plugin saw it, jumped 12 times
  at the 10 Hz joint-state rate (83 -> 48 -> 98, 130 -> 44 -> 130, 116 -> 78 -> 119, ...). The
  misreads last several samples at teleop speed, which is why the plugin's one-report gate
  confirms some of them as real jumps.
- The 21:08 sensor sweep passed (0 jumps in 35 revolutions at 20 to 100 %) because at those
  speeds a sensor is over any one spot for a few milliseconds; the fault needs slow travel or
  a pause on the spot, which teleop and position moves provide.

## Reading

The old plate's faults were sensor 0 at 198 to 258 deg (at rest and moving) and sensor 1 at
241, 277 and 290 (moving only). Those were gone on the new plate on 2026-10-16. After the
adjustment of 2026-10-17 sensor 1 misreads over 40 to 120 deg instead: the same sensor, a new
sector. Something about sensor 1's reading of the track changed with the adjustment: its gap to
the plate, its alignment, or the plate's seating on that side. Sensor 1 is the second entry in
the firmware's `SENSORS[]` (GPIO 19 on this board).

The plugin's gate cannot cover this (it holds a single report, these last several), and the
velocity map is unaffected (the speed sweeps are clean). Position moves and teleop that pass
through 40 to 120 deg can read 40 to 100 deg wrong for a few reports, which a position
controller would chase.

## Next

Inspect sensor 1 and the plate on that side (gap, squareness, cleanliness of the track);
then repeat this recording: a slow teleop pass through 30 to 130 deg with the raw ring listener
(`~/dome_bench/2026-10-17-ring-teleop/record_ring_raw.py <csv>` on `/dev/ttyUSB0`) and the
analysis in this file's history, or the static holds of
[2026-10-15-sensor-ring-raw-debug.md](2026-10-15-sensor-ring-raw-debug.md) with the launch
stopped.
