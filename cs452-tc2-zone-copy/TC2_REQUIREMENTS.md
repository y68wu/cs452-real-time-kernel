# TC2 requirement traceability

This file maps the TC2 acceptance areas to implementation and regression
evidence.  It separates software-complete behavior from physical Track D
validation; passing a host test is never recorded as a locomotive calibration.

| Acceptance area | Implementation evidence | Automated evidence |
| --- | --- | --- |
| Multiple trains move under one controller | `tc2_dispatch.c` stores 16 jobs, stages one launch epoch, admits a maximal disjoint subset during an oldest-first pass while conflicts wait, and prepares later disjoint jobs while other trains remain active; one queued launch wave is serialized and confirmed by the CAN protocol | real-route admission/fairness scenarios plus synthetic 2/3/4/6 confirmed launch waves; reservation-window test |
| Command includes train, origin, speed, destination | `dispatch <train> <A-F\|CURRENT> <speed> <d1-d8>`; strict protocol bounds in `train_control.c`/`tc2_dispatch.c`; `CURRENT` carries the complete old safety hold, while an estimated train's position ambiguity is narrowed to the travelled last-trusted-sensor-to-endpoint corridor so adjacent paired-turnout guards remain protected without becoming false positions; it safely relocalizes exact or estimated arrivals and automatically restages the original lowercase destination request after a bounded internal clearance/localization stop; supervised first-round timing uses the narrower `caldispatch 14 A <speed 1..120> d7` protocol and is explicitly provisional | alias/parser boundaries, exact caldispatch scope/isolation, all 128 exact directed-side-to-destination and all 64 estimated destination-to-destination planning/convergence cases, leading-reversal settling, and stage/preparation journal and turnout-race tests; physical non-stall calibration remains required |
| Six launch points and eight visible destinations | `tc2_track_model.c` is the single source for A–F and all 8x2 directed destination mappings; startup validation requires six entry nodes, 80 directed sensors/40 reverse physical pairs, 22 branch/merge turnout pairs, and all 16 anchors to agree with the graph. Measured directed offsets are d1=381/381, d2=226/226, d3=89/254, d4=76/279, d5=0/0, d6=0/0, d7=305/305, and d8=203/203 mm; all remain trial evidence pending repetition | catalog validation and mutation rejection; all 48 A-F/d regions, 96 directed sides, 11,520 speed predictions, 16 trial sides, and operator-orientation/render bounds |
| Current destination and position visible | dispatch snapshot separates the chosen `anchor_sensor` from the lowercase user endpoint and publishes immutable route waypoints. `dispatch` followed by `go` opens the persistent 36-row map with status on row 37 and command input on row 38. T13/T14/T15/T17 are fixed green/red/blue/yellow independent of staging order. Each marker advances monotonically on rendered track cells; an attributed physical CAN event rebases route distance and flashes the complete physical sensor-pair label without pulling the marker backward. ARRIVED/STOPPED settles at the logical endpoint. The header shows active train number, color, and actual commanded speed. Ordinary commands remain available; multiline diagnostic output pauses the map until `uion` so it remains readable | live UI regression covers all 48 routes and 84 adjacent visual pairs with 256 samples per non-zero segment, explicit ordered geometry for the 25 formerly-snapping bends, delayed-event non-regression, four fixed colors, braking coast, endpoint settlement, render/drain, prompt refresh without scroll, and active-dashboard `trips`/`cancel`/`remove` execution |
| Shortest safe route selection | unavailable-aware Dijkstra evaluates forward routes to both directed destination sides, excludes unavailable physical pairs, and selects the least physical track distance; turnout optimization cost breaks equal-distance ties. Sensor-reversal routing is a fallback only when no safe forward route exists | every one of the 5,760 normal A-F/d1-d8/speed plans is forward and physically shortest; an independent 48-route physical turnout oracle at speeds 20/80/120 locks every logical sequence, including C routes starting `5C`, F-to-d1 starting `18S`, and F-to-d2..d8 starting `18C`; paired-approach, synthetic reversal, and dynamically selected blocked-route tests |
| Track D graph geometry | Stage 32 applies the instructor-endorsed bidirectional physical-parts corrections C13-E7/E8-C14=875 mm, D5-E6/E5-D6=376 mm, D9-E12/E11-D10=369 mm, and E10-E13/E14-E9=376 mm; BR153-EN1 remains 253 mm because no endorsed replacement exists; the A-to-D7 graph gain and controller rebase are exactly +95/-95 mm, preserving the physical endpoint | symmetric-edge correction regression plus 48 A-F-to-d1-d8 routes, 96 directed sides, 11,520 speed predictions, adjacency/distance sums, turnout sequences, directed sensor replay, prediction bounds, and reservation guards; offline evidence only |
| Direction changes | automatic planning permits reversals only at sensor/reverse-sensor pairs, rejects consecutive zero-distance reversals, considers a forward-only-after-reverse leading alternative for a stopped `CURRENT` train, and creates per-leg turnout plans; a zero-motion result is not published as `ARRIVED` until its final braking-extension turnout batch is confirmed and any leading reverse has settled; speed zero is sent at the fast-bound calibrated stop-distance trigger while the larger braking-plus-uncertainty guard remains reserved, and missing evidence fails closed after the speed-derived post-confirmation settle interval | complete stop/reverse/turnout/resume/outbound-sensor path plus sensor-only routing, leg-boundary, planned and constructed zero-motion leading barriers, same-target speed-change turnout confirmation, leading-reversal ordering/settling, stop-envelope, missing-evidence fail-stop, watchdog, and reversal-clearance tests |
| Consistent sensor attribution | reservation lookup supplies owner and plan generation; per-train attributed-event journal preserves order | latch, timestamp recovery, journal paging/overflow tests |
| One sensor error | occurrence monitor tolerates one missing event and one attributed or globally unattributed spurious event; target-shaped reports require later ordered evidence before track is released; a virtual endpoint requires its physical anchor before timer arrival, remains estimated after settling, and retains its hold; a timer-only stop before the anchor becomes `STOP_UNCONFIRMED`, while observing a downstream guard at or beyond the endpoint ceiling before stopping fails closed; an estimated `CURRENT` continuation retains the old owner set through the first new expected report and releases only after a second plausible occurrence | missing/duplicate/spurious/repeated-occurrence, one-confirmation lag, exact A→D7/A1-only timer-stop, physical-D7-not-arrival, virtual-D7-requires-anchor, endpoint-overshoot, uncorroborated-anchor hold, two-evidence recurrence, and second-error fail-stop tests |
| Collision avoidance | atomic physical-pair reservations include tail, remaining path, virtual endpoint offset, and braking/reversal guards; footprint slides only with matching generation; a virtual arrival retains the complete endpoint guard until cancel/remove; `CURRENT` atomically replaces `old hold ∪ new safety` only when both expected old destination and generation match; the provisional point timer is isolated from and cannot shrink the shared braking-plus-uncertainty reservation envelope | atomic conflict/stale-generation/sliding-window, virtual-endpoint footprint/hold, provisional point-vs-guard separation, and destination+generation CAS replacement tests |
| No turnout switched under a train | reservation is acquired before asynchronous current-leg plus final braking-extension turnout commands; reversal and every `CURRENT` route carry the preceding final-leg confirmed turnout plan and reject a conflicting next setting inside the body/uncertainty hold, while an identical setting is reused without retransmission; before estimated-position relocalization, same-direction routing also forbids leaving and later re-entering the carried physical footprint; exact or estimated unsafe departures use a bounded confirmed-direction clearance chain; stage-to-prepare sensor, journal, CAN, or unsolicited-turnout changes are rejected before the reservation CAS; paired turnouts are one interlocked resource; only the dispatcher has CAN turnout authority | asynchronous preparation barrier, active-train preparation, repeated-BUSY barrier, exact/estimated all-pairs recurrence and convergence, strict carried-footprint re-entry rejection, prelaunch sensor/journal/turnout races, real ambiguity-corridor turnout rejection, ARRIVED-turnout-change rejection, paired-interlock, authorization, and command-UI bypass tests |
| Unavailable track changes routing | `block`, `unblock`, and `blocks`; active/reserved landmarks cannot be blocked | unavailable physical-pair and alternate-route tests |
| Robust communications | cooperative terminal waiters; `TryGetc` adds an immediate byte/empty/error observation without allocating a blocking waiter or changing `Getc` FIFO order; `TerminalTryWrite` is a nonblocking, all-or-none 1..192-byte bulk write; the three-slot UI pump retains the exact immutable-frame offset on `WOULD_BLOCK`, prevents generation interleaving, restores the caller's exact prompt bytes, and invalidates deltas after a hard output failure; critical CAN commands require exact CS3 responses and sequential pacing; emergency stops cancel queued setup and motion controls and run after at most the in-flight command; absolute retry quarantine is safe; queue exhaustion is retryable backpressure rather than a health fault; terminal batch states are retained; RXB0/RXB1 and overflow health | terminal regression covers more than the 16-waiter capacity in consecutive empty polls, blocking-waiter preservation, FIFO, bytes `0x00`/`0xff`, short/error replies, bulk queue and wrap boundaries, and malformed bulk requests; UI pump tests cover partial drain, repeated backpressure, generation wrap, pending replacement, exact prompt restoration, output failure, and full-redraw recovery; priority/quarantine/backpressure CAN state, batch-retention, repeated-BUSY dispatch barrier, SPI-timeout, and MCP2515 transport tests |
| Complete task context | every target translation unit uses `-mgeneral-regs-only`, final-link disassembly rejects SIMD/FP operands, and every exception/SVC frame saves and restores `x0`-`x30`, `ELR`, and `SPSR` through one shared C/assembly layout | complete 272-byte GPR save/restore and context-layout regressions, build-policy regression, and final linked-image disassembly gate |
| Runtime liveness / fail-stop | the raw-image boot path zeroes the complete linker-defined BSS before C and kernel startup defensively resets event-list roots; a priority-0 TC2 bootstrap creates the complete topology before service load; the terminal notifiers, sensor courier, UI, dispatcher ticker, and servers share the priority-4 control-plane round-robin class while bounded CAN RX and the independent emergency supervisor remain higher priority; periodic UI input can use immediate `TryGetc` polling without blocking for a future byte or filling the blocking-waiter queue; notifier error retries timer-block instead of high-priority Yield-spinning; the dispatcher uses a bounded raw-timer wait for the courier READY handshake before declaring the scheduler healthy; CAN TX/RX and sensor courier heartbeats gate launch/motion, and the courier blocks after every bounded four-frame raw-CAN burst; a cached dispatcher tick `N` accepts only the legitimate equal-priority courier heartbeat skew `N+1`, while larger future skew, stale heartbeat, or new courier receive/time failure still fails closed with a distinct diagnostic; raw-timer supervisor watches dispatch heartbeat, logical time, and real elapsed microseconds; peer exit wakes blocked IPC clients | BSS linker/boot zero-loop and event-root reset gates; TC2/non-TC2 bootstrap-priority, complete-topology, shared terminal/courier/UI/ticker priority, terminal nonblocking-poll/waiter-isolation and notifier error-backoff, delayed/absent READY startup-barrier, equal-priority progress, `N`/`N+1` first-RUNNING acceptance, and excessive-future-skew fail-close regressions; notifier/courier stale-heartbeat, continuous valid/invalid CAN fairness, wall-stall, emergency-stop, and task-exit IPC tests |
| Fast trips, non-stall speed, and braking | the 10 ms model interpolates corrected train-14 command-to-rest measurements 50/110/240/390/530/740/875/1435 mm at speeds 20/40/60/70/80/90/100/120. A-to-d1..d8 validation trips keep the requested speed and command zero exactly one measured coast distance before each final or reversal stop; every ordered sensor rebases the remaining-distance deadline. Other starts retain the prior precision-anchor controller until A is physically accepted. Reservation, watchdog, and settling remain based on the larger conservative envelope | exhaustive 1..120 monotonic interpolation, all A destination direct-stop profiles at representative measured speeds, observed-velocity, 5,760 live and offline plans, zero-offset endpoint projection, missing-anchor fail-close, and request/confirmation timing tests; repeated per-route physical calibration remains required |
| Build identity and documentation | `README-TC2`, calibration files, target `-Werror`, mode/config-checksummed object trees, stale-root invalidation, the exact Stage 35 runtime build marker, and mode-specific post-link gates for `Tc2OfflineControllerStep`, `Tc2UiTxPumpDrain`, and generic `TerminalTryWrite` | build-plan warning/mode/config isolation and ambient-MODE regression, exact Stage 35 marker and symbol gates, optimized renderer compile, `make test`, both-mode AArch64 syntax, and `git diff --check` |

## Stage 19 prediction evidence boundary

`tc2_prediction_model.c` converts a validated route to one catalog endpoint
side into a signed, route-relative plan.  It keeps the physical operator
endpoint, the transferred speed-corrected control endpoint, the speed-zero
command point, and the conservative reservation ceiling as four independent
quantities.  Endpoint and command scalars are re-anchored against the actual
route, so an earlier high-speed control point may move to a preceding
direction-specific sensor without relabelling the user destination.  A
pre-start command point fails safe as an explicit immediate stop.

Only T14/A/D7/side 0 with no reversal receives
`EXACT_T14_A_D7`; all other train, start, destination-side, or reversal
combinations receive `TRANSFERRED_T14_A_D7`.  Geometry evidence remains a
separate catalog property.  The minimum reservation ceiling is based on the
uncorrected physical endpoint plus the conservative braking envelope, so a
negative point-control correction cannot reduce collision protection.

The host regression enumerates every
`6 * 8 * 2 * 120 = 11,520` Track D prediction request, checks exact catalog
anchors and shortest-route consistency, validates scalar/re-anchor identities,
and proves the evidence and reservation invariants.  This is automated
prediction coverage, not 11,520 physical calibration claims.

## Stage 20/21 integration and UI evidence boundary

For an ordinary fixed-start dispatch, Stage 20 computes prediction metadata
from the route actually selected by the safety planner.  It does not change
the selected route, train command, stopping behavior, arrival state, or
physical reservation, and it excludes `CURRENT` plus `caldispatch`.

Stage 21 renders one bounded static Track D operations map.  The map covers
all six start labels, all eight operator endpoints, all 80 direction-specific
sensor nodes in 40 co-located detector pairs, and all 22 switches.  The
renderer validates catalog uniqueness and every output line is at most 132
characters.  Operator `D1`–`D8` labels are not treated as sensor identities.

This evidence proves static UI coverage only.  It does not prove live train
position, colored multi-train identity, sensor flashing, CAN health, or
physical endpoint accuracy.  Dynamic overlays and the explicit offline
simulation source are later stages; no Stage 21 field may be interpreted as
physical confirmation.

## Stage 25 route-projection evidence boundary

`tc2_route_projection.c` consumes the route already selected by the safety
planner and that route's prediction.  It does not call shortest-path planning
again, alter graph geometry, or change reservations.  It emits only discrete
map waypoints supported by catalog/layout data: the declared A-F start, every
direction-specific sensor occurrence, the direction selected for each
turnout, explicit reversals, and the logical d1-d8 operator destination.
Coordinates identify the semantic label glyphs rather than the surrounding
ASCII decoration.  A logical operator endpoint is attached to the final real
route node at or before its exact scalar only as provenance.  If the endpoint
lies inside an edge, the following graph node and every later safety-suffix
event are excluded from UI motion, so a destination such as D7 is never
turned into a fabricated sensor or graph node.  The full selected route still
remains in the projection for reservation audit, and prediction is derived
only from the prefix through its physical anchor.  This contract is for
ordinary fixed A-F dispatch base/operator endpoints; `CURRENT`, localization,
and supervised `caldispatch` correction points are explicitly outside it.

The exhaustive host regression validates 11,520 request projections over 96
speed-independent geometries under ASan/UBSan and strict warnings.  Canonical
shortest routes cover 65/80 directed sensors and 21/22 turnouts; elements not
selected by those canonical routes are not claimed as runtime or physical
coverage.  This stage proves deterministic route geometry for the future
offline simulator.  It does not yet prove periodic UI timing, multi-train
conflict scheduling in that simulator, CAN observations, or physical endpoint
accuracy.

## Stage 26 atomic-publication evidence boundary

`tc2_dispatch.c` now constructs and validates the Stage-25 projection in
private scratch before reservation CAS.  A reservation conflict or failed CAS
cannot publish any route.  After CAS succeeds, the dispatcher copies the
fixed-width header and waypoint payload and commits validity last.  The
read-only API returns a header and pages of at most eight waypoints; each page
token includes train, publication serial, reservation generation, and launch
epoch.  A token from an older stage or task returns `STALE`, preventing pages
from different jobs from being combined.

Re-stage, cancel, control failure, and operator removal invalidate the UI
publication immediately.  This display invalidation is intentionally
separate from safety ownership: reservations remain protected through
cancel/STOP_HOLD until physical removal and `remove`.  `ARRIVED` retains its
projection until cancel or remove.  Fixed A-F jobs with an authoritative
prediction can publish; `CURRENT`, localization-only, and provisional
`caldispatch` jobs return `NOT_READY` rather than fabricating geometry.

Host tests reconstruct published routes page by page, verify canonical zero
tails and bounds, reject serial/generation/epoch staleness, prove that CAS
conflicts expose no projection, and exercise re-stage/cancel/failure/remove
invalidation.  Stage 26 is still software evidence only and does not claim
new CAN traffic, live sensor observations, or physical stopping accuracy.

## Stage 27 offline-runtime evidence boundary

`tc2_offline_runtime.c` consumes a complete immutable Stage-26 projection
either atomically or through contiguous token-matched pages.  It rejects
missing, reordered, stale, aliased, non-canonical, and out-of-range input
before a trip can become runnable.  At most 16 fixed slots are advanced by
the shared calibrated integer velocity model; tick arithmetic supports one
`uint32_t` wrap and rejects ambiguous or oversized advances without partial
state change.

The runtime reserves prediction-only resource windows over graph vertices,
normalized physical edges, co-located direction-sensor detector pairs,
turnouts, and destination footprints.  Overlap produces a stopped
`WAIT_CONFLICT` state.  Older waiters have deterministic priority and claims
shrink as trains move, so disjoint work can proceed concurrently.  All
sensor-looking output is tagged `PREDICTED_OFFLINE` with `is_live=0`.
Consequently this layer cannot be mistaken for CAN evidence or movement
authority and does not call the dispatcher reservation service.

ASan/UBSan coverage includes all 5,760 A-F/d1-d8/speed combinations,
1/2/3/4/6/16 simultaneous records, all 80 directed sensor identities,
collision/FIFO release, tick wrap, fixed snapshot interpolation, and
fail-closed page reconstruction.  The periodic renderer integration is
deliberately outside this stage.

## Stage 28 pure offline-planner evidence boundary

`tc2_offline_planner.c` turns only caller-supplied Track D data and the
four-part request `(train, A-F, speed 1..120, d1-d8)` into a deterministic
route, calibrated prediction, and immutable projection.  It shares the
catalog, route guards, reversal policy, virtual-destination geometry, and
projection validation already covered by earlier stages.  It owns no task
ID, performs no IPC, and cannot call CAN, the reservation service, the
sensor service, or physical train control.

The sanitizer test builds and validates every one of the 5,760 matrix
requests twice and compares every published byte.  It also removes selected
graph edges to force the reversal fallback, cuts the unique origin edge to
prove `UNREACHABLE`, mutates catalog data to prove `CATALOG_INVALID`, checks
invalid input bounds, and confirms that the complete Track D graph remains
unchanged.  Every failure returns a canonical empty, non-runnable
publication.  This is deterministic planning evidence, not live sensor,
collision-safety, or physical stopping evidence; the periodic controller and
terminal integration are intentionally covered by Stages 29-31 below.

## Stages 29-31 offline-controller and terminal evidence boundary

Stage 29 adds the nonblocking, all-or-none terminal bulk primitive and the
three-slot atomic frame pump.  Stage 30 adds one allocation-free controller
for planner, runtime, overlay, renderer, and pump.  Stage 31 validates the
combined surface over the complete
`6 * 8 * 120 = 5,760` request matrix and
1/2/3/4/6/16-train cardinalities.  The map continues to distinguish all
80 directed sensor names, including co-located opposite directions; train
colors are stable and distinct; predicted detector pulses retain train,
generation, quality, and source provenance.

Every offline controller snapshot and rendered frame is bounded by the exact
label:

```text
OFFLINE SIMULATION / PREDICTED / NO CAN / NO PHYSICAL MOTION
```

The controller advances on explicit absolute ticks, and render/drain budgets
cannot advance model time.  Terminal `WOULD_BLOCK` retains the exact frame
offset; hard output failure invalidates deltas and requires a full redraw.
Impossible runtime or provenance transitions fail closed until reset.
Therefore the matrix is evidence for deterministic software planning,
conflict arbitration, position interpolation, coloring, flashes, and terminal
recovery only.  It is not evidence of CAN delivery, physical turnout
actuation, locomotive movement, live sensor ordering, stopping accuracy, or
real collision avoidance.

Physical proof still required before running unattended:

- verify A–F orientation and d1–d8 placement on the actual layout;
- append real train/speed/direction observations under `calibration/`, including
  the lowest repeatably non-stalling speed for each demo train/direction;
- for each `caldispatch` run, record requested speed and signed
  user-endpoint-to-pickup-leading-edge error after full stop (not the physical
  anchor-sensor offset), then replace the provisional prior only after
  repeated evidence;
- demonstrate conflicting and disjoint 2/3/4/6-job epochs;
- inject one intermediate missing/non-target spurious report and then a
  second-error fail-stop;
- verify stopping envelopes and turnout behavior at the selected demo speeds.

## Stage 33 renderer build-safety closure

The optimized renderer validates the directed-sensor flash interval at the
write site, so neither corrupt overlay state nor future catalog changes can
index outside the fixed 132-column frame.  `make test` compiles the renderer
regression at `-O3`; the course AArch64 `MODE=TC2` build remains the final
target-toolchain acceptance check.
