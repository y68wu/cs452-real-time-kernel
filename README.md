# CS452 Train Control Part 2

This repository contains the TC2 multi-train controller for the CS452 Track D
layout.  The TC1 calibration commands remain available in an explicit TC1
build, while the default build is TC2.

## Author

Name: Yutong Wu / Chris Wu  

## Source

The repository was originally exported from the TC1 commit of the project and
then extended with the TC2 scheduler, route monitor, reservation service,
sensor attribution, cooperative terminal I/O, and reliable CAN transport.

Original repository: https://git.uwaterloo.ca/y68wu/cs452_kernel.git  
TC1 commit: d97b69d Complete TC1 calibrated routes and stopping

Earlier A0 and kernel milestone source snapshots are preserved on the
[`historical-coursework` branch](https://github.com/y68wu/cs452-real-time-kernel/tree/historical-coursework).
A later, unverified three-train physical-demo variant is preserved on the
[`remote-physical-demo` branch](https://github.com/y68wu/cs452-real-time-kernel/tree/remote-physical-demo).

## Build TC2

Build on the linux.student.cs environment:

    make clean
    make

or explicitly:

    make MODE=TC2

The build must finish with:

    Built and verified MODE=TC2 image: iotest.img

TC1, TC2, and PERF objects are kept in separate
`build/<MODE>/<config-id>/` directories.
An ambient shell `MODE` cannot silently change the default TC2 build, while an
explicit command-line `MODE=...` remains supported.  The final link is rejected
if its task symbols and application banner do not match the requested mode.

The generated image is:

    iotest.img

## Upload to Track D

    /u/cs452/public/tools/upload.sh ./iotest.img d8:3a:dd:1b:36:9e

## Run Terminal

    gtkterm -p /dev/ttyS0 -s 115200

## Operate TC2

The primary workflow stages any number of trips (up to the 16-job scheduler
capacity)
and submits the staged set as one launch epoch:

    dispatch <train> <A-F|CURRENT> <speed> <d1-d8>
    go
    trips
    trackview

After `go`, the live operator view is intentionally minimal and follows the
physical drawing orientation: A/B are below, C/D/E/F are above, d7 is left,
and d1 is at the far right. T14 is always red. The header shows each active
train number, color, and current commanded speed. Between physical detector
events the marker advances along its immutable route projection at the
predicted speed; each attributed CAN detector event snaps it back to the
physical detector and flashes that detector pair. Arrival settles at the
lowercase operator endpoint. The command prompt remains on row 34, and every
command redraws back to the live map unless the operator explicitly enters
`uioff`, so `trips`, `cancel`, and `remove` remain available during motion.

For supervised first-round T14 timing tests, Stage 18 retains an isolated
provisional workflow:

    caldispatch 14 A <speed 1..120> d7
    go
    trips

It predicts every integer user speed from the first T14/A-direction physical
anchor, then updates point velocity and its velocity-squared stopping prior
from each accepted ordered sensor interval.  These estimates are not labelled
as accepted calibration and do not reduce the normal collision-reservation
envelope.  `trips` reports
the predicted and actual speed-zero request ticks, the matching CAN
confirmation tick, and their signed timing difference for each supervised
run. Stage 8 also fixes the observed launch-time sensor-heartbeat race:
one bounded `N`/`N+1` scheduler skew is accepted, while stale or otherwise
invalid courier health still stops the train and is identified in `trips`.
Stage 9 keeps the provisional model out of sensor plausibility validation:
ordered observations use the independent conservative fast envelope, while
the narrower provisional family remains confined to the supervised
speed-zero prediction and its fail-closed watchdog.

The current model separates measured endpoint geometry from braking control.
All sixteen directed d1-d8 offsets come from the operator's corrected inch
measurements; d5 and d6 are exactly sensor-centred.  Train 14 has an 8-inch
body with its pickup at the geometric centre.  Command-to-rest distances are
interpolated from the corrected 20/40/60/70/80/90/100/120 measurements of
50/110/240/390/530/740/875/1435 mm.  The first physical validation pass is
limited to departures from A: the controller keeps the requested speed and
commands zero one measured command-to-rest distance before each stopping
point. Collision reservation continues to use the larger independent safety
envelope. Other departures retain the prior precision-anchor controller
until the A-to-d1..d8 pass is physically accepted.

Stage 18 moves all A–F launch aliases and all 8x2 directed destination-side
definitions into `tc2_track_model.c`.  Startup and host tests validate the
catalog against the generated 144-node graph, including 80
direction-specific sensors/40 reverse physical pairs and 22 turnout
branch/merge pairs.  The directed endpoint offsets in millimetres are
d1=381/381, d2=226/226, d3=89/254, d4=76/279, d5=0/0, d6=0/0,
d7=305/305, and d8=203/203.  They remain `TRIAL` until repeated physical
runs confirm them.  This catalog is the shared basis for routing, simulation,
and the live coordinate UI.

Stage 19 adds `tc2_prediction_model.c`, a pure route-relative planner for all
6 starts, 8 destinations, 2 directed endpoint sides, and speeds 1 through
120.  It keeps physical endpoint geometry, speed-corrected control endpoint,
speed-zero command point, and conservative collision reservation as distinct
values. Only the measured T14/A/d7/forward/no-reversal case is labelled
exact; every other case is explicitly labelled as a transferred T14/A/d7
prior.  An exhaustive 11,520-case host test checks graph reachability,
re-anchoring, signed/range-safe arithmetic, evidence labels, and the invariant
that point prediction cannot reduce the conservative reservation ceiling.
This stage validates the reusable planner; it does not claim that every
transferred case has been physically calibrated.

Stage 20 attaches the prediction result to the ordinary fixed-start dispatch
route that was actually selected.  The attachment is metadata-only and cannot
alter motion, arrival, or collision reservations; noncanonical routes are
downgraded from exact evidence, and `CURRENT` plus `caldispatch` remain
isolated.

Stage 21 adds a bounded complete Track D operations map to `trackview`.  It
shows A–F, operator destinations d1–d8, all 40 detector pairs as 80 distinct
directional sensor labels, and switches 1–18/153–156.  This release is
deliberately static: colored moving-train markers and sensor flashes are not
yet represented, and the map is not evidence of physical CAN movement or new
calibration.

Stage 22 adds only the allocation-free dynamic overlay state: 16 stable train
colors, separately addressable directed sensors, refresh-phase marker motion,
300 ms sensor flashes, wrap-safe stale-update rejection, and an explicit
`OFFLINE SIMULATION / NO PHYSICAL MOTION` default.  Stage 23 adds a
nonblocking terminal byte/empty/error poll that never occupies a blocking
`Getc` waiter slot.  Stage 24 adds a bounded ANSI renderer: its
first frame draws the complete Track D map and subsequent frames emit only
changed runs; all 16 trains have stable, distinct colors, directed sensor
labels flash without being overwritten, and multiple
trains in one display cell become a visible red `!`.  Offline frames state
that CAN, physical motion, and live sensor evidence are absent.  Detector
flashes retain source, quality, and trip-generation provenance: LIVE accepts
only sensor-confirmed CAN events, while OFFLINE accepts only explicitly
predicted or estimated events.  Removed trips retain a generation tombstone,
preventing delayed equal/older updates from recreating ghost markers.
Periodic command-loop integration and the offline route-time animation engine
are still intentionally unclaimed at this point.  Stage 25 adds the
deterministic bridge from the exact dispatcher-selected route to discrete UI
waypoints: start, each direction-specific sensor, selected turnout direction,
reversal, and the logical d1-d8 operator endpoint.  It reuses the selected
route and its prediction rather than finding another route.  The complete
route copy retains its safety suffix for audit, while semantic waypoints stop
at the exact operator scalar; an endpoint inside an edge uses the preceding
real node only as provenance and never fabricates a graph node.  Physical
anchor sensors remain distinct from virtual operator destinations.  The
ordinary fixed A-F contract is covered by an ASan/UBSan matrix of all 11,520
A-F/destination-side/speed requests and 96 speed-independent geometries;
`CURRENT`, localization, and provisional calibration geometry remain outside
this projection boundary.  Runtime time-stepping remains the next stage.

Stage 26 publishes that already-validated projection from the dispatcher as
an immutable, fixed-width read-only snapshot.  The full projection is built
in private scratch before reservation CAS, and only a successful CAS can
publish it.  A header plus eight-waypoint pages are bound to train,
publication serial, reservation generation, and launch epoch; a page token
from an older task is rejected as stale instead of being spliced into a new
route.  Re-stage, cancel, control failure, and physical removal invalidate
the UI publication immediately, while the independent reservation remains
held until the normal protected removal workflow completes.  `CURRENT`,
localization, and provisional calibration trips explicitly report no
projection.  This is a software publication and audit boundary only: it does
not claim CAN motion, live detector evidence, or physical endpoint accuracy.

Stage 27 adds a deterministic, allocation-free offline runtime for those
immutable publications.  Up to 16 predicted trains advance with the shared
fixed-point calibration, interpolate across the Track D display, and emit
explicitly non-live sensor/turnout/arrival events.  Resource windows include
graph vertices, normalized physical edges, co-located detector pairs,
turnouts, and destination footprints; conflicting trains wait under a stable
oldest-first policy while disjoint trains advance together.  All 5,760
A-F/d1-d8/speed requests, 1/2/3/4/6/16-train sets, 80 directed sensors,
tick wrap, and malformed/stale publication pages are exercised under
ASan/UBSan.  This remains offline evidence only; the periodic terminal
connection is a separate stage.

Stage 28 adds the pure route planner in front of that runtime.  Every A-F,
d1-d8, and integer-speed 1..120 request is planned twice and compared
byte-for-byte under ASan/UBSan, for 5,760 deterministic matrix entries.
The planner shares the validated Track D catalog, generalized-cost
shortest-path and reversal rules, calibrated prediction, and projection
contracts, but performs no CAN, IPC, reservation, live-sensor, or physical
train operation.  Synthetic tests force a legal reversal, make an origin
unreachable, mutate the catalog, and verify fail-closed empty results without
changing the input graph.  That Stage 28 result deliberately excludes the
periodic terminal/controller integration added by Stages 29-31 below.

Stages 29-31 complete the bounded offline presentation pipeline.  A
three-slot atomic transmit pump drains immutable renderer frames through the
all-or-none, nonblocking `TerminalTryWrite` transport in chunks of at most
192 bytes; backpressure preserves the exact byte offset and a hard error
forces a full redraw.  One static/BSS controller then owns planner, 16-train
runtime, provenance overlay, delta renderer, and transmit pump.  It provides
stable distinct train colors, pulsing predicted markers, direction-specific
300 ms detector flashes, deterministic conflict waits, generation-safe
remove/reset, and fail-closed provenance checks.

The integrated software matrix covers all
`6 * 8 * 120 = 5,760` A-F/d1-d8/speed requests plus
1/2/3/4/6/16-train scenarios.  Every frame carries the exact label
`OFFLINE SIMULATION / PREDICTED / NO CAN / NO PHYSICAL MOTION`.
This validates predicted software behavior only.  Physical CAN delivery,
switch commands, train motion, detector timing, endpoint accuracy, and
collision acceptance remain pending Track D tests after CAN is restored.

Stage 32 reconciles the generated Track D graph with the
instructor-endorsed physical-parts audit.  Both directed representations of
each physical edge now agree: C13-E7/E8-C14 is 875 mm, D5-E6/E5-D6 is
376 mm, D9-E12/E11-D10 is 369 mm, and E10-E13/E14-E9 is 376 mm.  The
unendorsed BR153-EN1 discrepancy remains at 253 mm.  Because the first
correction adds exactly 95 mm to the A-to-d7 graph route, the T14/A/d7
controller anchors are rebased by -95 mm and retain the same physical
operator endpoints.

The Stage-32 host matrix validates all 48 A-F-to-d1-d8 operator routes,
96 directed destination sides, and every speed 1..120: 11,520 prediction
cases plus route adjacency/distance, turnout sequences, direction-specific
sensor replay, prediction bounds, and reservation guards.  This is
deterministic graph/software evidence only; it does not prove CAN delivery,
physical turnout actuation, live sensor ordering, endpoint accuracy, or
collision avoidance on hardware.

Stage 33 makes the sensor-flash renderer independently validate the catalog
column and width against the 132-column frame before styling any cell.  This
closes the optimized AArch64 compiler's `stringop-overflow` finding and keeps
malformed catalog geometry fail-closed.  The renderer regression is compiled
at `-O3` so the release-build optimization path remains covered.

Stage 34 completes the course-toolchain closure for that renderer check.
Catalog columns and widths are unsigned bytes, so the write-site guard now
promotes them to `size_t` and validates the complete interval without an
invalid negative comparison.  Runtime, post-link, test, and documentation
markers are synchronized at Stage 34.

See [README-TC2](README-TC2) for the complete command reference, launch and
destination map, safety behavior, sensor-error policy, tests, and the physical
Track D validation checklist.  Version-controlled motion-model data and the
raw measurement format are in [calibration/](calibration/).  The
requirement-to-code/test mapping is in
[TC2_REQUIREMENTS.md](TC2_REQUIREMENTS.md).

## Build legacy TC1

    make clean
    make MODE=TC1

The generated image is also `iotest.img`; do not use the TC1 image for a TC2
demonstration.

## Version 2 kernel safety

The controller performs only integer calculations.  Every target translation
unit is therefore compiled with `-mgeneral-regs-only`, and the final linked
image is rejected if its disassembly names any SIMD/FP register.  Task switches
use the shared-layout frame containing `x0`-`x30`, `ELR`, and `SPSR`; this
avoids depending on optional SIMD/FP task state during boot or operation.
