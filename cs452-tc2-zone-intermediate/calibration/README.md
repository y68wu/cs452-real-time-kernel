# TC2 calibration data

This directory is the version-controlled source of the motion-model values
used by TC2.  Values marked `seed` are conservative software defaults carried
forward from the TC1 implementation; they are not represented as measurements
from a particular physical train.

`tc2_seed_model.csv` records the velocity and stopping-distance lookup table
used when no train-specific Track D measurement is available.  A clock tick is
10 ms.  The historical velocity observations were recorded in centimetres per
second; the same numbers therefore represent tenths of a millimetre per
10 ms tick.  Keeping that fixed-point unit avoids the unsafe 10x/100x timing
error that would result from reading `101` as `101 mm/10 ms`.

The slow bound is half the seed velocity and is used only for late-arrival
watchdogs.  The fast bound is the seed velocity plus 20% and is used for
earliest-arrival plausibility and conservative braking-command timing.  These
bounds are explicit software uncertainty, not train-specific measurements.
User speeds are mapped to the nearest model row by:

```text
model_index = clamp((speed * 14 + 60) / 120, 1, 14)
```

`track_d_measurement_template.csv` is the required raw-data format for lab
calibration.  Add rows rather than replacing earlier observations.  The
software build does not parse the CSV at run time, so any accepted calibration
change must update both the model constants and this directory in the same
commit.

`track_d_measured_geometry.csv` records the July 29 operator measurements
after correcting the tape-reading unit to inches.  It is the current source
for the d1-d8 directed offsets, train-14 body/pickup geometry, measured
command-to-rest curve, and the E7-to-D7 speed-20 timing.  Millimetre values in
the runtime catalog are rounded to the nearest whole millimetre; stopping
values retain micrometre precision in the motion model.

`track_d_physical_runs.csv` also records rejected or incomplete physical runs
that revealed a model or controller mismatch.  Its `accepted_calibration`
column must remain `false` unless the run contains the steady-state, stopping,
and settling evidence required below.  These rows are diagnostic evidence,
not motion-model constants.

`track_d_dispatch_test_matrix.csv` is the 48-run A-F to d1-d8 physical
calibration checklist at speed 80.  `PASS` means the software route,
prediction, reservation suffix, sensor projection, and UI publication were
simulated successfully; it does not claim a physical measurement.  Record a
signed residual only after a real run (`negative` = stopped before the
operator mark, `positive` = stopped past it).

Run `make dispatch-matrix-report` to rebuild the detailed speed-80 report
directly from the same catalog and planner used by TC2.  The report includes
the selected destination side, anchor/endpoint distances, braking command
point, and expected sensor sequence.  Before emitting those 48 rows, the
report validates every integer speed 1 through 120 for all 48 start/destination
pairs (5,760 plans).

## Superseded historical prediction notes

The sections below preserve the sequence of earlier d7 experiments for audit
history.  Their old +280 mm endpoint and 396 um/tick/10.243 mm motion values
are not current runtime constants.  The authoritative July 29 geometry and
motion inputs are in `track_d_measured_geometry.csv` and the model described
above.

## Provisional prediction and virtual endpoint

One later supervised T14/A-direction speed-20 run supplies a first-round point
prediction so all integer command speeds can be tested without first
hard-coding 120 unrelated values.  The raw CAN/sensor timing established:

- C13 to E7: the capture was evaluated against the legacy 780 mm graph
  value, yielding 39.632 mm/s over 19.681009 s; the instructor-endorsed
  physical-parts reconstruction is now 875 mm, so the timing remains raw
  evidence but the legacy derived velocity is not reclassified as accepted
  calibration;
- normalized model value: 396 micrometres per 10 ms tick;
- E7 report to the speed-zero request frame: 1.129316 s, approximately
  44.757 mm at that velocity;
- final E7 to pickup leading edge: 55 mm;
- inferred command-to-stop distance: 10.243 mm.

The physical reference is the pickup leading edge in the direction of travel,
not the locomotive nose or wheel axle.  The pickup length used to convert the
nearest-edge measurement was 45 mm.

`Tc2MotionProvisionalVelocityUmPerTick()` continuously interpolates the
historical curve shape and normalizes speed 20 to 396 um/tick.  It is the
pre-sensor fallback for all speeds 1..120.  After every accepted ordered
sensor interval, Stage 16 computes
`round(segment_mm * 1000 / elapsed_10ms_ticks)` and uses that observed point
velocity for both the remaining travel deadline and the velocity-squared
10.243 mm stopping prior.  The whole-millimetre helper is a rounded-up display
value only; collision protection continues to use the independent
conservative envelope.

The operator-labelled destination d7 is not the same physical point as sensor
D7.  For the measured A-to-d7 direction, sensor D7 is the localization anchor
and the fixed physical base to the operator endpoint is 280 mm.  Stopping
behavior is speed-dependent.  The current signed correction anchors are
+66/+50/+32/+15/-99/-555 mm at speeds 20/40/60/80/100/120.  Stage 32
increased C13-to-E7 by exactly 95 mm and rebased every controller correction
by -95 mm.  The fixed +280 mm base and the physical A-to-effective-d7
distances therefore remain 2570/2554/2536/2519/2405/1949 mm.  Every
intervening integer speed interpolates continuously in provisional velocity
space; speeds below 20 clamp to +66 mm.  Valid observed intervals adjust
point timing at runtime.  When the effective endpoint lies before D7,
the controller represents the same point as a positive offset after E7
(speed 120: E7 +101 mm, equivalently 275 mm before physical D7) while retaining
logical destination d7.  The speed-120
Stage-15 residual was repeated, but the Stage-16 correction remains
unaccepted until physically tested.  The D7-side base is confirmed only in this
direction; interpolated speeds and the other directed endpoint offsets remain
provisional until measured.

## Stage 32 Track D geometry audit

The instructor-endorsed physical-parts audit changes both directed
representations of four physical edges: C13-E7/E8-C14 to 875 mm,
D5-E6/E5-D6 to 376 mm, D9-E12/E11-D10 to 369 mm, and
E10-E13/E14-E9 to 376 mm.  BR153-EN1 remains 253 mm because no endorsed
replacement was provided.  The A-to-D7 controller rebase described above is
algebraically exact: the graph route gains 95 mm and its speed correction
loses 95 mm, so the operator endpoint and physical target do not move.

No historic physical-run row was rewritten.  One new audit row records the
geometry provenance separately from motion calibration.  The Stage-32 host
matrix checks 48 A-F-to-d1-d8 routes, 96 directed sides, speeds 1..120
(11,520 predictions), adjacency/distance sums, turnouts, sensor replay,
prediction bounds, and reservation guards.  This is offline graph/software
evidence, not CAN, live-sensor, stopping-accuracy, or collision-avoidance
evidence.

This is exposed through the single-job
`caldispatch 14 A <speed> d7` workflow and the normal multi-train route
planner.  The virtual offset is also included in route cost and in the shared
reservation/braking envelope; the point timer cannot shrink collision
protection.

The source stopping-model row remains `accepted_calibration=false`: there is
only one run, the
physical settle timestamp was not measured, and every non-20 speed is an
extrapolation.  Report each new run as early/late by a signed millimetre
offset measured from the user endpoint—not the physical anchor sensor—to the
pickup leading edge after the train is fully stationary; keep the packet
capture, requested speed, `trips` output
(predicted/actual request and CAN-confirmation ticks), and result together.
The displayed `actual_request_tick` is sampled immediately before the
application sends its speed-zero CAN IPC, while `confirmed_tick` is sampled
after the matching CS3 response returns; use the packet capture for exact
on-wire times.  A manual cancel before the automatic trigger is a safety event,
not a valid prediction-calibration observation.

For each train and speed, collect at least three runs in each travel direction:

1. place the train at a named sensor occurrence and record the start event;
2. run at the requested speed long enough to reach steady motion;
3. issue speed zero at a recorded sensor event;
4. measure the final stopping point and record the distance after the stop
   command;
5. measure `stop_settle_ticks` from the matching CS3 speed-zero confirmation
   until the locomotive is physically stationary;
6. record missed/spurious sensors, wheel slip, turnout changes, and any manual
intervention in `notes`.

Capture `6.pcapng` records the mapping failure that established the virtual
offset: Stage 9 observed A1, C13, E7, and physical D7 in order, then requested
speed zero about 2.906 ms after the D7 report and stopped near that sensor.
The measured user endpoint was 180 mm farther downstream, so this run is
diagnostic only and must not be used as accepted stopping calibration.
The following Stage-10 +180 mm virtual-endpoint run still stopped 100 mm
before the labelled user endpoint.  That direct residual refines the
A-to-D7 directed offset to 280 mm for Stage 11; it does not by itself accept
the stopping curve.

The next Stage-11 speed-20 run stopped 68 mm before the user endpoint even
though its predicted and actual speed-zero request ticks differed by only one
tick.  A separate Stage-11 speed-100 run stopped exactly at the endpoint with
the same +280 mm base.  This disproves a single +348 mm physical offset for
all speeds.  Stage 13 kept the geometry at +280 mm and applied the first
two-point correction.  Tests of that version then measured 93 mm short at
speeds 20/40/60/80, 4 mm beyond at speed 100, and 240 mm beyond at speed 120.
Stage 14 added those signed residuals to the previous correction curve.
The operator subsequently reported speeds 20 through 100 acceptable but
speed 120 still 180 mm beyond the endpoint.  Stage 15 therefore keeps the
20..100 anchors and changes speed 120 from -240 to -420 mm, interpolating the
101..120 high-speed transition.  Two repetitions of Stage 15 both remained
40 mm beyond the endpoint.  Stage 16 therefore keeps every anchor through
speed 100 and changes only speed 120 from -420 to -460 mm.  The online
estimator remains active for every command speed.

Until those rows exist for a train/speed/direction combination, TC2 must retain
its braking guard, use the conservative speed-derived settling deadline, use
the seed model, and treat a timer-only stop before the target as
`STOP_UNCONFIRMED`.  That state preserves the last confirmed progress, next
sensor, and complete remaining-route reservation until the operator completes
the `cancel` then `remove` workflow; it is never reported as an arrival.
