# Active simulator

The simulator uses the same internal layout and `evaluatePose` function as the
optimizer. Lengths are millimetres, pose rotations and `betaAngles` are radians,
and `servoRangeRad` is radians. Downloaded layouts convert the servo range to
degrees through the shared layout export contract.

## Controller boundary

`createSimulatorController()` in `src/simulator/controller.js` owns a copy of the
active layout. `loadLayout(layout, { source, options })` resets pose state and
checks home. `source` identifies a candidate, import, or editable copy; candidate
sources carry `candidateId`. The controller never edits the retained optimizer
result. `getReferenceLayout()` returns another independent copy for optimizer
reference input or JSON transfer.

`requestPose(pose, { source })` always records six finite requested coordinates
and calls `evaluatePose` with `recordLegData: true`. `getState()` returns the
requested pose, evaluator `assessment`, last valid `accepted` pose, and
`acceptedAssessment`. A failed request keeps its leg-specific diagnostics but
does not change `accepted`; if home fails, `accepted` remains `null`. The
rendered mechanism is always the accepted pose; a rejected request is shown
only as the dimmed ghost overlay described below. `setOptions(patch)` reevaluates both the accepted
pose and current request. `subscribe(listener)` provides a snapshot immediately
and after every change; its returned function unsubscribes. Geometry controls
and diagnostics should use these calls rather than duplicating the evaluator.

`setAnimation(pattern, playing, settings)` and `tick(deltaSeconds, settings)`
support wobble, ping-pong, rotation, tilt, and helical motion. An invalid
animation frame pauses playback and leaves its request and failure visible.
Markers and bounded accepted-position traces (the most recent 300 accepted
platform-origin positions) are controlled by `setMarkers`, `setTraces`, and
`clearTrace`. `setOverlays(patch)` switches named scene overlays on or off;
names left out of the patch keep their state, and an unknown name or a
non-boolean value throws without changing anything. Animation patterns advance at most 0.1 s of simulated time per
frame, scaled by the speed multiplier (0.1 to 5 in the UI). Each pattern is a
sum of sine and cosine terms in the frame time, so `animationState(pattern,
seconds, settings, rate)` returns its pose together with exact velocity and
acceleration in the cycle model's trajectory-state form (m/s, m/s², and the
angular velocity and acceleration from the Euler-angle rates, rad/s and
rad/s²). `rate` is the speed multiplier: derivatives are taken against real
time, so velocity scales with it and acceleration with its square.

The payload and servo ratings behind the **Loads** overlay are controller state
too, `loadModel` in `getState()`: an object of requirement JSON keys in their
units, the mass fields `mass_kg`, `center_of_mass_mm`, `inertia_kg_m2`,
`external_force_n` and `external_moment_nm` and the servo rating keys
(`servo_torque_rating_nm`, `per_servo_ratings`, `servo_actuator` and the rest
listed in [REQUIREMENTS.md](./REQUIREMENTS.md)), or `null` for none.
`parseLoadModel` in `src/simulator/loads.js` keeps only those keys and checks
them with the optimizer's `normalizeMassProperties` and `normalizeServoRatings`.
`loadLayout(layout, { loadModel })` replaces it in the same validated load
(`null` removes it, an invalid model throws before anything changes); a load
that leaves it out keeps it, so a geometry edit keeps the loads, and `clear()`
removes it. After every request the controller solves `loads` in `getState()`
for the accepted pose with `dynamicsAtPose` from `src/model/cycle.js`, the
function the cycle demand uses: on an animation frame with that frame's
`animationState`, and for every other request (manual, keyboard, pointer,
gamepad, settings, load, replay) statically, with zero velocity and
acceleration. A rejected request keeps the accepted pose and its loads.
`loads` is `null` without a load model or an accepted pose; otherwise it is
`{ valid, motion, reason, rodForceN, servoTorqueNm, servoSpeedRadPerSec,
ratedTorqueNm, utilization, staticForceN }`, with `motion` `'static'` or
`'animation'`. `rodForceN` is positive in compression (the rod pushes the
platform from horn tip toward platform point) and negative in tension.
`servoTorqueNm` is the signed output-shaft torque in the +horn-angle direction,
the load torque plus any reduced actuator model (`servo_actuator`), as the
cycle's capacity check uses it. `ratedTorqueNm` is each servo's peak torque
rating (`servo_torque_rating_nm` or its `per_servo_ratings` `torque_nm`), and
`utilization` is |`servoTorqueNm`| over it, `null` for an unrated servo; speed,
torque-speed curve, continuous and duration ratings are not checked per frame.
`staticForceN` is the magnitude of the payload weight plus the external force.
A singular pose gives `{ valid: false, reason }` with the cycle model's message.

The requirement workspace ranges drawn by the **Workspace box** overlay are
controller state, `workspaceRanges` in `getState()`: one `{ min, max }` per
pose axis about home, millimetres for X/Y/Z and radians for Rx/Ry/Rz, or
`null` for none. `loadLayout(layout, { workspaceRanges })` replaces them in the
same validated load (`null` removes them); a load that leaves the option out
keeps the current ranges, so a geometry edit keeps the box.
`setWorkspaceRanges(ranges)` replaces them on their own, and `clear()` removes
them. Axes may be left out; each given axis needs finite `min <= max`, `step`
and other keys are dropped, and invalid ranges throw with the axis named
before anything changes.

`setReachabilityCloud({ enabled, sampleCount, mode, sliceZ })` patches the
**Reachability cloud** (see Scene builders and overlays). `enabled` is the
`reachabilityCloud` overlay toggle, so `setOverlays({ reachabilityCloud })`
does the same; the other three keys are `reachability` in `getState()`, with
defaults `REACHABILITY_DEFAULTS` in `src/simulator/reachability.js`:
`sampleCount` 1,024 (one of the workspace `SAMPLE_PRESETS`, 256, 1,024 or
4,096), `mode` `'cloud'` or `'slice'`, and `sliceZ` 0 (mm about home, a finite
number). Every key is checked before anything changes. While the overlay is on
and a layout is loaded, the controller sweeps samples through `evaluatePose`
with the current simulator options on the main thread, `REACHABILITY_CHUNK`
(200) poses at a time, yielding to the next animation frame before each chunk
(a timer outside a browser; `createSimulatorController({ schedule })` replaces
it). The result is `reachabilityCloud` in `getState()`: `{ points, progress,
total, orientation, error }`, where each point is `{ at, reachable }` with
`at` the evaluator's platform-origin `translation` in world millimetres,
`progress` runs from 0 to 1, `orientation` is the `{ rx, ry, rz }` swept and
`error` is the evaluator's message if a sample threw (the sweep then stops
there), else `null`. The cloud is published after every chunk, frozen and
shared by all snapshots rather than copied; it is `null` while the overlay is
off or no layout is loaded. A change to the requested rotation, the settings,
the layout, the options or the workspace ranges aborts the running sweep
through its AbortSignal and starts again from no points, so points from two
sweeps are never mixed. A request that changes only X/Y/Z keeps the cloud.
`clear()` and `dispose()` abort the sweep. While an animation rotates the
platform the sweep restarts every frame, so the cloud fills in once the
rotation stops.

## Mechanical geometry controls

`createGeometryControls({ document, container, controller })` mounts the active
geometry editor in `#simGeometryControls`. Its editor copies the controller's
layout before every edit and reloads that copy through `loadLayout`, which
reevaluates the home pose and publishes fresh diagnostics. A selected optimizer
candidate becomes an `editable` simulator source carrying its `candidateId`;
the retained optimizer candidate stays unchanged. Every edit rederives the
layout's `derived` mounting directions against the new home geometry and keeps
`supplied` ones, so the exported `mounting` block matches the evaluator's.
Reset restores the geometry and source last loaded from outside the editor
while keeping current evaluator options.

Circular, C3 paired, and rectangular paired layouts start in parametric mode.
Radius, C3 anchor pair gap, rectangular aspect, base/platform turn, and horn
direction offset use the shared `topologyGeometry` generator. C3 has only a base
turn, since its platform pairs are locked 60° from the base pairs, and its horn
offset is limited to ±90°. A **Horn direction** select (Outward, the default, or
Inward) sits above the C3 horn offset; it applies the editor's
`{ type: 'hornDirection', value }` edit, which stores
`topology_parameters.horn_direction` and regenerates the layout. Outward horns
start tangent pointing away from their partner, Inward horns toward it, so the
two directions with the ±90° offset cover every mirrored horn angle. Inward
horns near tangent can cross their partner; the link-collision check reports
that rather than the editor refusing it. Turn and horn
direction controls display degrees but store radians. The generator keeps the
selected topology's anchor and beta-angle invariants. Horn/rod length, home
height, and servo range are separate scalar controls; the servo range displays
degrees and stores radians.

Circular and Rectangular also expose an alternating horn offset. Newly generated
layouts start at 30°; legacy imports without this parameter display 0° and keep
their original directions until explicitly edited.

**Edit anchors explicitly** preserves the current anchors and beta angles
exactly while switching topology metadata to `free`. Explicit mode offers each
base/platform XYZ coordinate and each horn direction. **Generate selected
topology** is the deliberate reverse mode switch: it replaces explicit anchors
with coordinates from the chosen shared generator. Changing unrelated lengths,
height, or servo bounds never regenerates explicit anchors. Sliders and numeric
fields show the same current value; invalid edits, including an empty or
non-numeric field (never read as 0), leave the last valid layout in place,
restore the field and report a field-specific error. A focused field whose
text the user has changed is not rewritten by animation frames, so geometry
can be edited while a pattern plays; a rejected edit still restores the stored
value. A focused field that has not been edited follows every layout change,
so Reset geometry, a candidate or reference load and a browser-save restore
update it even in browsers where clicking a button leaves focus in the field.
A committed entry counts as untouched even when the stored value prints
differently from the typed text (`77.0` for 77), and values print with at most
12 significant digits so a degree that round-trips through radians shows 30
rather than 29.999999999999996.
## Browser workflow

The Optimize and Simulate tabs share the retained candidate selection. The
simulator loads the selected candidate automatically after a run and when the
chart or either candidate list changes. Switching tabs leaves the current pose,
camera, input mode and animation state in memory. A new run clears the previous
candidate selection. After **Load optimizer reference** the candidate select
gains an **Imported reference** entry, is enabled even without a run, and
choosing that entry again reloads the imported document after a candidate was
shown.

The native WebGL2 renderer draws two layers, both from evaluator output and
never from its own constraint solving. The solid layer is the accepted pose:
base anchors, solved horn tips, actual rods, platform points, servo orientation
angles and platform frame, with legs that fail in the requested pose coloured
as described under Live diagnostics. When the latest request was rejected, the
**Rejected pose ghost** overlay adds a second, dimmed layer showing that
request (see Scene builders and overlays). The ghost is the only geometry drawn
from a pose the evaluator did not accept. Orbit camera is the initial mouse mode: dragging with the primary button or
first touch changes yaw and pitch (a right click or second finger does not
reset the drag). The wheel zooms toward the point under the cursor, keeping
that point (taken on the plane through the camera target that faces the
camera) under the cursor, with the camera distance between 10 and 2,500 mm, so
a close-up can separate lines a few millimetres apart. Shift+drag pans in
either mouse mode: the point under the cursor follows it. The camera target is
centred on the layout (half the home height above the origin) when a layout
loads and again whenever a newly loaded layout has a different home height;
pose requests, animation and display changes leave a zoomed or panned view
alone. **Reset camera** restores the default orientation and distance and
recentres the target on the layout. Lines that cross the renderer's 1 mm near
plane are clipped there, so a close-up keeps the visible part of a line passing
beside the camera instead of dropping it. The projected depth range follows the camera distance so a
zoomed-out view keeps near/far line ordering instead of saturating the depth
buffer. Moving the platform by pointer requires selecting **Move
platform**, which converts drag distance to X/Y requests. Numeric controls and
sliders request six-axis poses; sliders span ±50 mm and ±30°, while the numeric
fields accept any finite value. As with the geometry and diagnostics fields, a
pose field the user is typing into keeps its text while an animation plays, an
untouched focused field follows every request, a committed entry counts as
untouched, and an invalid entry restores the requested value. Keyboard arrows move X/Y, Page Up/Down moves Z,
W/S tilts about X, A/D tilts about Y, and Q/E rotates about Z, each by 1 mm or
1° per press (Shift doubles the step); keys are ignored while a field has focus
or the Simulate tab is hidden. An optional gamepad maps the left stick to X/Y,
the right stick to tilt, the triggers to Z and the shoulder buttons to yaw with
a 0.15 dead zone. Pose requests do not change anchor coordinates.

The pose labels share the colours of the axes drawn in the view: X and Rx red,
Y and Ry green, Z and Rz blue. The colours come from `SCENE_COLORS` in
`src/simulator/scene.js` (published as the `--axis-x/y/z` CSS variables) and
do not change with the frame. **Move along** chooses the frame every
incremental pose input acts in (`src/simulator/input-frame.js`). **Base axes**
(default) is the fixed world frame at the base plate. **Platform axes** is the
frame of the requested pose, so X moves along, and W/S tilts about, the
platform's own red axis. In that frame the X/Y/Z fields and sliders read the
requested translation `t` as `Rᵀ t`, an edited field requests `t = R t_local`,
and arrow and Page keys, **Move platform** drags and the gamepad sticks and
triggers step along the platform axes. The rotation keys and the gamepad's
right stick and shoulder buttons turn about the chosen axes through the
platform origin, which stays put: `R' = Rδ R` about the base axes and
`R' = R Rδ` about the platform axes, with `Rδ` the step's rotation vector
(Rodrigues) and the result converted back to Euler angles (`R = Rz Ry Rx`),
each unwrapped toward its previous value. In the base frame a Q/E yaw changes
only Rz; a W/S or A/D tilt of an already turned platform is a true tilt about
the fixed X or Y axis rather than a step of one Euler angle. The Rx/Ry/Rz fields
and sliders always hold the stored Euler angles in both frames, since an
absolute angle needs a fixed frame, and a rotation field edit keeps the
platform origin where it is. Poses are always stored, evaluated, animated and
saved in the base frame; switching frames only changes how the fields read the
same pose and never moves the platform.

**Use geometry as optimizer reference** copies the active layout and a replay
snapshot to the optimizer's reference JSON field. **Load optimizer reference**
can restore that layout and saved requested/accepted poses. **Download simulator
JSON** saves the same geometry plus run settings, simulator options, pose,
camera, animation, markers, traces, overlay toggles, reachability cloud
settings, workspace ranges, load model and input mode. The shared importer validates
the layout and ignores old scores; every pose is checked again by the current
evaluator. The `simulator` block is validated as a whole before anything is
applied: `options` (only the known keys are kept; limits, servo bounds,
tolerance, condition limit and the clamp flag must have the right type),
`requested` and `accepted` poses, `camera` (finite yaw, pitch within ±1.4,
distance 10 to 2,500, three-coordinate target), `animation.speed` (positive),
`markers`, `tracesEnabled`, `overlays` (each known overlay name true or false;
unknown names are dropped, and a file without the block keeps the current
toggles), `reachability` (`sampleCount` 256, 1024 or 4096, `mode` `cloud` or
`slice`, finite `sliceZ` in mm; other keys are dropped, and a file without the
block keeps the current settings; whether the cloud is on is
`overlays.reachabilityCloud`, and the samples themselves are never saved, so a
load sweeps afresh), `workspaceRanges` and `loadModel` (see below), `pointerMode` (`orbit` or `platform`) and
`inputFrame` (`base` or `platform`; a file without it keeps the current frame). A rejected
file or browser save reports the offending `simulator.` field and leaves the
current layout, pose, camera, animation and, for a browser save, the optimizer
inputs untouched. If WebGL2 is unavailable, the simulator names the missing capability
and suggests enabling hardware acceleration or a modern desktop browser;
optimization remains usable. If the browser loses the WebGL2 context (GPU reset,
driver crash, backgrounded mobile tab, too many contexts on one page), the pose
status reports it, the renderer rebuilds its program and buffers when the
context is restored and redraws the accepted pose; pose requests are still
evaluated while the context is lost.

Loaded animations remain paused. A saved idle (`none`) or unrecognized pattern
selects Wobble as the playable default, so Play works after a save/load before
the first animation has been started.

`simulator.workspaceRanges` holds the ranges the workspace box draws, in the
units of the requirements and of a run's `effective_settings.bounds`: X/Y/Z in
millimetres and Rx/Ry/Rz in degrees, one `{ min, max }` per axis (`step` is
dropped), or `null` for none. Degrees are written with at most 12 significant
digits, so a 12° range saves as 12 after the radian round trip. Selecting an
optimizer candidate takes the ranges from the current run's
`effective_settings.bounds`. Loading simulator JSON takes them from the block,
else from the file's `run.effective_settings.bounds`, else there are none and no
box is drawn; a malformed saved block rejects the file naming the axis, while
malformed run bounds only mean no box.

`simulator.loadModel` holds the payload and servo ratings the **Loads** overlay
solves against, in requirement JSON keys and units (see Controller boundary),
or `null` for none. Selecting an optimizer candidate takes them from the
current run's `effective_settings`: the mass fields of
`effective_settings.requirements` and the rating input
`effective_settings.servoRatings`, which carries any rating overrides typed in
the Optimize tab. Loading simulator JSON takes them from the block, else from
the file's `run.effective_settings` the same way, else there are none and the
overlay is unavailable; a malformed saved block rejects the file naming
`simulator.loadModel`, while a run whose load does not parse only means no loads.

## Scene builders and overlays

`buildSceneGeometry(state)` in `src/simulator/scene.js` (re-exported by the
renderer) concatenates the output of an ordered list of builders,
`SCENE_BUILDERS`. Each builder is `(state, layout, solved) => { lines, points }`,
where `solved` is the accepted assessment or `null`, so builders only draw data
the evaluator already produced. The list order is the draw order: `groundGrid`,
`base` (base polygon, servo direction stubs and base markers), `platform`
(platform polygon), `legs` (horns, rods and their markers), `servoArcs`,
`jointCones`, `workspaceBox`, `reachabilityCloud`, `conditioningEllipsoid`,
`loads`, `requestedGhost`, `platformAxes`, `worldAxes` and `trace`. Markers and traces keep their own
controls. Each point carries a pixel `size` (markers 6 or 7, reachability
samples 3); the renderer draws each size in its own call, and a point without
one at 7.

A builder with an `overlay` key is drawn only when that key is on in
`state.overlays`. `OVERLAY_DEFAULTS` lists every toggleable overlay and its
default; today these are **Ground grid** (`groundGrid`), **Servo arcs**
(`servoArcs`), **Joint cones** (`jointCones`), **Workspace box**
(`workspaceBox`), **Reachability cloud** (`reachabilityCloud`),
**Conditioning ellipsoid** (`conditioningEllipsoid`), **Loads** (`loads`),
**Rejected pose ghost** (`requestedGhost`), **Platform axes** (`platformAxes`)
and **World axes** (`worldAxes`), all on except the reachability cloud, the
conditioning ellipsoid and the loads. With only the two
axis overlays on, the scene matches the original single-function renderer line
for line (a frozen fixture in `tests/fixtures/scene-geometry.json` checks this). The Simulate tab shows one checkbox per overlay in the
**Overlays** group, with the id `simOverlay` plus the capitalised name (for
example `simOverlayWorldAxes`). The info button beside the heading
(`simOverlaysInfoButton`) shows and hides `#simOverlaysInfo`, a panel with a
short description of each toggle. A new overlay adds one builder, one default,
one checkbox, one entry in that panel and a paragraph here; new overlays default off unless their
issue says otherwise.

**Ground grid** (on by default) gives the scene a scale: a square grid of dim
lines on the base plane (z = 0), one every `GROUND_GRID_PITCH_MM` (25 mm, not
user-editable) through the origin, reaching 1.5 times the base radius (the
farthest base anchor from the Z axis) rounded up to whole cells. The grid
shares its plane with the base polygon, servo stubs and world axes, so its
lines carry a depth bias of `GROUND_DEPTH_BIAS_MM` (2 mm) away from the camera
and lose every depth tie with them.

**Workspace box** (off by default) draws the twelve edges of the requirement
X/Y/Z ranges as a box about home: X and Y from their `min` to `max`, Z from
`homeHeight + min` to `homeHeight + max`, from the controller's
`workspaceRanges`. It is the region the platform origin must reach, not the
platform's extent, and it stays put as the pose moves. Rotation ranges are not
drawn. Without all three translation ranges (for example a layout imported
without a run or saved ranges) there is no box.

**Reachability cloud** (off by default, because every sample costs an
evaluator call) shows where the platform origin can go at the current
requested rotation. The controller's sweep (see Controller boundary) evaluates
Halton translations, the optimizer's `halton-v1` sequence through
`workspacePoses` in `src/workspace/sampling.js`, inside the requirement X, Y
and Z ranges from `workspaceRanges`, or ±100 mm
(`DEFAULT_REACHABILITY_HALF_RANGE_MM`) for any axis without a range, each at
the requested Rx/Ry/Rz. Every sample is a 3 px point at its evaluated origin,
green when `evaluatePose` accepts it and dim red when it does not; lines are
never drawn between samples. **Z slice only** (`mode: 'slice'`) samples a
single plane instead: Z is fixed at the **Slice Z** offset from home, so all
samples share that plane and the full sample count covers X/Y. Below the
toggles, **Cloud samples** picks 256, 1,024 or 4,096 samples, the slice
checkbox and **Slice Z (mm)** field (enabled in slice mode; an empty or
non-numeric entry is reported and restored, never read as 0) set the mode, and
a status line reads, for example, `Swept 1024 of 1024 samples at Rx 0, Ry 0,
Rz 0°: 812 reachable. Evaluated samples only, not a continuous envelope.`
(`Sweeping` while in progress). The cloud shows evaluated samples only: a gap
between green points is not shown to be reachable, an isolated red point does
not bound the region, and nothing here claims a continuous envelope or
boundary. It uses the same checks as every other pose (servo range, ball
joints, rod length, link clearance, conditioning).

**Conditioning ellipsoid** (off by default) draws the translation
manipulability ellipsoid of the accepted pose at the platform origin, so the
reciprocal condition number shown as text in the diagnostics panel has a
shape. Its principal axes are the right singular vectors of the translation
block of the accepted assessment's `conditioning.jacobianRows`, from
`translationSingularSystem` in `src/model/conditioning.js` (see
[CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md#translation-conditioning-ellipsoid)).
Each half-axis is proportional to its singular value, and the largest equals
the platform axis length (`platformAxisLength`, the larger of 18 mm and 0.35
horn lengths), so the drawing shows proportions, not a physical size. The
builder draws the three axes as full diameters, carrying the ground grid's
2 mm depth bias away from the camera so the platform axes they overlap stay on
top, and the three great circles through each pair of axes (32 segments each).
As the pose nears a translational singularity one axis shrinks and the
ellipsoid flattens toward a disc or a line. A symmetric layout at home has
equal horizontal axes; with near-vertical rods the vertical axis is the
longest, so it is not a sphere. The colour comes from the full six-axis
reciprocal condition number of the accepted pose, which also covers rotation
the shape does not show: `conditioningColor` blends from light blue
(`wellConditioned`) at reciprocal 1 to the near-limit yellow at
`CONDITIONING_GRADE_FLOOR` (0.01, condition 100) on a log scale, or at
1 / `conditionLimit` when a condition limit above 1 is set. When the requested
pose fails a conditioning check (`conditionLimit` or `numericalSingularity`)
the ellipsoid, still drawn at the held accepted pose, turns the whole-platform
failure magenta. Without an accepted pose or usable Jacobian rows nothing is
drawn.

**Servo arcs** (off by default) draw each servo's allowed travel as an arc of
horn-length radius about its base anchor, from the effective minimum to the
effective maximum servo angle (the simulator's `servoRangeRad` option, else the
layout's range, else ±90°, as `effectiveServoRange` in `src/model/pose.js`
resolves it for the evaluator). The arc uses the evaluator's horn frame, so its
ends are exactly the horn tips at the two stops. Short ticks mark both stops and
a marker crosses the arc at the accepted horn angle; with no accepted pose the
marker is omitted. The arc and its ticks are grey, the marker is horn orange,
and all three turn yellow when the accepted angle is within
`NEAR_LIMIT_MARGIN_RAD` (5°) of a stop, or red when the requested pose fails
that servo's range (`servoLimit`). Other failures leave the arc colours alone;
the leg itself is still coloured as described under Live diagnostics. The margin
constant lives in `src/simulator/scene.js` for all limit overlays.

**Joint cones** (off by default) draw each ball-joint socket's allowed cone at
the accepted pose: 12 cones, a lower one with its apex at the horn tip and an
upper one with its apex at the platform point. The axis is the socket normal
in world coordinates from `socketNormalsInWorld` in `src/model/pose.js`, the
same construction `evaluatePose` measures joint angles from (see
[JOINT_MODEL.md](./JOINT_MODEL.md)), so the rod lies inside a cone exactly when
its joint angle is within the limit. The half-angle is that socket's effective
limit from the accepted assessment's `jointLimits`; an edit in the diagnostics
panel reevaluates the accepted pose and the cones redraw in the same
notification. Each cone is a ring at a fixed slant length of
`max(12, 0.35 × hornLength)` mm from the apex, the same length as the axis
markers, plus four generatrix lines; a fixed slant rather than a fixed height
keeps wide limits, up to 180°, bounded. Cones are grey, yellow when the
accepted joint angle is within `NEAR_LIMIT_MARGIN_RAD` (5°) of the limit, and
red when the requested pose violates that socket (`ballJoint` for that leg and
joint). The rod drawn by `legs` shows the current direction against the cone.
The cones visualise the joint limit only; they are not a collision check.
Link collisions are reported by the evaluator and coloured on both legs.

**Loads** (off by default) draws the rod forces and servo torques the
controller solved for the accepted pose (`state.loads`, see Controller
boundary); the builder solves nothing. Its checkbox is disabled, with the hint
`#simLoadsHint` below the reachability status, until the load model loads the
platform at all: a positive `mass_kg` or a nonzero external force or moment
(`hasLoad` in `src/simulator/loads.js`). Each rod is redrawn from horn tip to
platform point in a colour graded by its force by `rodForceColor`: the plain
rod colour at zero, toward compression blue (`compression`) when the rod
pushes and toward tension orange-red (`tension`) when it pulls, reaching the
full colour when one rod carries `staticForceN`, the whole static payload
weight plus external force (or, with neither, the largest rod force of the
frame). These rods carry a `LOAD_DEPTH_BIAS_MM` (2 mm) depth bias toward the
camera so they win the depth tie with the plain rods beneath them. A leg the
held pose already failure-colours (see Live diagnostics) is not redrawn, so the
failure stays visible. Each servo with a peak torque rating gets a torque
gauge in its horn plane: WebGL lines are one pixel wide, so the gauge is a band
of six concentric arcs at `TORQUE_BAND_RADII` (1.3 to 1.45 horn lengths, just
outside the servo arc and its angle marker). It starts at mid-travel and sweeps
toward the maximum stop for positive torque or the minimum stop for negative,
reaching the stop at 100 % utilisation and capped there above it.
`torqueUtilizationColor` grades it from green (`torqueLow`) at zero to the
near-limit yellow at 100 %, and above 100 % it turns the failure red. Unrated
servos get no gauge; their torque is still listed in the diagnostics. Loads
are static (zero velocity and acceleration) for a manual pose and dynamic,
from the pattern's analytic velocity and acceleration at the playback speed,
while an animation plays, so the gauges show which servo saturates during the
motion. The model is the cycle model's rigid-body Newton-Euler balance with
ideal massless rods and horns (see [CYCLE_MODEL.md](./CYCLE_MODEL.md)).

**Rejected pose ghost** (on by default) draws the latest request faintly when
the evaluator rejected it (`assessment.reachable` is false), so the pose that
was asked for, or the animation frame that paused playback, is visible next to
the held accepted pose. It reads only the rejected assessment: the platform
outline is the requested `translation` and `rotationMatrix` applied to
`layout.platformAnchors`, which the evaluator always returns, plus the
requested platform axes; horns and rods are drawn only for legs where the
solver produced a horn tip, because it stops at the first structural failure
(a joint-limit failure still solves every leg). The ghost has no markers, and
its colours are dimmed to 35 % of the way from the background colour
(`SCENE_BACKGROUND`, also the canvas clear colour) toward the normal colour,
instead of blending. Legs named in a violation use the failure colour at full
brightness, and a whole-platform failure outlines the ghost platform in the
global-failure colour. An accepted request draws no ghost. Each failure is
coloured once, on the geometry that failed: while the ghost draws a failing leg
or the whole-platform failure, the held accepted pose keeps its normal colours
for it (see Live diagnostics). A request just past a limit puts the ghost almost on top
of the held pose, where equal depths would make the lines flicker between the
two layers as the camera moves. Each ghost line therefore carries a depth bias
of `GHOST_DEPTH_BIAS_MM` (10 mm), which changes only its depth value, not where
it appears: failure-coloured ghost lines are pulled toward the camera so they
show on top, and dimmed ghost lines are pushed away so the held pose shows
wherever the two overlap.

## Rendering checks

The camera is an orbit camera with +Z up. `yaw` is the eye's azimuth about the
target, measured from +X toward +Y; `pitch` is its elevation above the target's
horizontal plane; `distance` is the eye's distance from `target` in mm. The view
is a pinhole projection with a 60° vertical field of view and square pixels, so
the horizontal field follows the canvas aspect ratio. Screen right is the view
direction crossed with +Z, so from any camera above the base plane the world
X axis turns counterclockwise onto the Y axis on screen. `cameraFrame` and
`projectPoint` in `src/simulator/renderer.js` implement this, and the CPU
projection is the only one: the vertex shader passes positions through.

Three layers of checks tie what is drawn to the evaluator's solved geometry:

- `tests/simulator/renderer.test.js` checks that each leg line runs from the
  evaluator's own base anchor, horn tip and platform point objects.
- `tests/simulator/projection-oracle.test.js` compares `cameraFrame` and
  `projectPoint` with an independently written pinhole camera
  (`tests/fixtures/independent-geometry.js`), checks screen handedness, the
  field of view and square pixels, and checks that the vertex buffer the
  renderer uploads puts every horn and rod at the pinhole projection of the
  solved points.
- `npm run test:browser:render` (`scripts/browser-render-check.mjs`) loads a
  tilted, translated pose with markers and overlays off in real WebGL, reads the
  canvas back and requires the rod and horn colours at the pinhole projection of
  points along every rod and horn, and the X, Y and Z axis colours along the
  world axes. The same samples mirrored left-right or top-bottom must mostly
  miss, so a flipped image fails.

The model side of the same chain is checked in
`tests/model/inverse-kinematics-oracle.test.js` (hand-solved legs, and an
independent platform transform, horn-tip formula and root search for general
poses) and `tests/model/rotation-convention.test.js` (the Euler convention).
None of these checks replace validation against a physical mechanism.

## Live diagnostics

The diagnostics panel subscribes to the same controller as the renderer. It reports the six-axis **requested** pose and last valid **rendered accepted** pose separately. A rejected request remains visible with its evaluator failures while the renderer holds the last accepted geometry. Red leg rows mark failures in the requested pose, and a whole-platform conditioning failure has a separate message; the table always lists every failure. A `linkCollision` names two legs, so it is listed on both rows, each against the other leg and link, for example `linkCollision (rod with leg 2 rod)`, and both legs are coloured. In the scene each failure is coloured once, on the geometry that failed. When the **Rejected pose ghost** overlay is on and the solver reached that leg's horn tip (a joint-limit failure), the ghost leg is red and the held accepted leg keeps its normal colours; a whole-platform failure likewise outlines the ghost platform in magenta instead of tinting the held legs. When the ghost is off, or for a structural failure where the solver stopped before that leg's horn tip, the held accepted leg is coloured red (magenta for a whole-platform failure). A red held leg therefore marks a failure in the request, not in the rendered pose, which the evaluator accepted; the rejected pose itself appears only as the dimmed ghost overlay. The servo arcs and joint cones keep their own failure colours either way.

Each leg shows lower and upper socket deflection against their effective limits, maximum deflection, rod-length deviation against the editable tolerance, servo angle, rod force and servo torque. The force and torque columns come from the controller's `loads` at the **accepted** pose, not the request: rod force in newtons, positive in compression; servo torque as signed output-shaft N m against the peak rating with the utilisation percentage, or `unrated`, and the torque cell is red above 100 %. A line above the table (`#simLoadDiagnostic`) says whether the loads are static or from the animation, that no payload or ratings are loaded, that there is no accepted pose, or why the cycle model could not solve the pose. Unavailable measurements show a dash because the evaluator may stop at the first structural failure. Lower and upper joint limits, rod tolerance and link clearance (`linkClearanceMm`, the minimum distance between two legs' horn or rod centre lines; 0 turns the check off) can be edited in the panel; changes reevaluate the current request and last accepted pose, a focused field keeps text the user has typed while an animation plays, an untouched focused field still follows loads and settings changes, and a rejected value is restored. The mandatory reciprocal conditioning cutoff and optional engineering condition limit are displayed, with the link clearance and the closest pair of links of different legs in the requested pose. Simulator JSON saves these effective options with the requested and accepted poses, and loading that JSON restores them.
