# Feature reference

This document inventories what the Stewart Platform Optimizer actually does today, organized by feature area. Each section names the user-facing controls, the headless API surface, the source module that owns the behavior, and the deeper model document where one exists. Values quoted here (defaults, limits, budgets) are taken from the source at the time of writing; when a value here and a value in an older document disagree, this document and `src/` are the reference.

Feature areas:

1. [Requirements input](#1-requirements-input)
2. [Optimization controls](#2-optimization-controls)
3. [Search engine](#3-search-engine)
4. [Candidate evaluation model](#4-candidate-evaluation-model)
5. [Execution, progress and cancellation](#5-execution-progress-and-cancellation)
6. [Results browser and exports](#6-results-browser-and-exports)
7. [Reference-layout import and refinement](#7-reference-layout-import-and-refinement)
8. [Integrated simulator](#8-integrated-simulator)
9. [Browser workspace save](#9-browser-workspace-save)
10. [Headless API and replay](#10-headless-api-and-replay)
11. [Deployment, tooling and verification](#11-deployment-tooling-and-verification)
12. [Explicit non-features and limits](#12-explicit-non-features-and-limits)

## Feature map

| Feature | Where in the UI | Headless entry | Owning modules | Detail doc |
| --- | --- | --- | --- | --- |
| Requirements JSON (flat or nested) with validation, cycle note and cycle-vs-workspace warnings | Requirements textarea, Load Sample, Clear, cycle note | `parseRequirements(text)`, `describeCycle`, `cycleRangeWarnings` | `src/model/requirements.js`, `src/model/cycle-checks.js` | [REQUIREMENTS.md](./REQUIREMENTS.md) |
| Rigid-body payload and 6-DOF trajectories | JSON only | `payload.trajectory`, mass fields | `src/model/trajectory.js`, `mass-properties.js` | [CYCLE_MODEL.md](./CYCLE_MODEL.md) |
| Servo ratings, envelopes, thermal and actuator models | Servo ratings panel (peak, speed, continuous, policy, per-servo peak/speed); curves, durations and actuator via JSON | `servoRatings` option / requirements keys | `src/model/servo-ratings.js` | [CYCLE_MODEL.md](./CYCLE_MODEL.md#actuator-demand-and-servo-capacity) |
| Layout topologies with symmetry-preserving evolution | Layout Topology select, Horn direction (C3) select | `topology`, `hornDirection`, `designSpace` | `src/optimization/topology.js`, `layout-operators.js` | [TOPOLOGIES.md](./TOPOLOGIES.md) |
| NSGA-II multi-objective search, Compact/Full objective sets | Objective set, population, generations, mutation rate, seed | constructor options | `src/optimization/optimizer.js`, `nsga2.js`, `objectives.js` | [architecture.md](./architecture.md) |
| Halton or Cartesian workspace sampling | Workspace sampling select, axis min/max/step | `sampling`, `ranges` | `src/workspace/sampling.js`, `sweep.js` | [RESULTS.md](./RESULTS.md) |
| Strict and soft ball-joint policies, two-socket joint model | Ball Joint Limit, soft-constraints checkbox | `ballJointLimitDeg`, `lower/upperBallJointLimitDeg`, `ballJointClamp` | `src/model/pose.js`, `mounting.js` | [JOINT_MODEL.md](./JOINT_MODEL.md) |
| Link collision check between legs' horns and rods | Link Clearance (mm); simulator diagnostics | `linkClearanceMm`, `link_clearance_mm` | `src/model/collision.js`, `pose.js` | [JOINT_MODEL.md](./JOINT_MODEL.md#link-collisions) |
| Actuator conditioning (mandatory numerical, optional engineering limit) | Always on; engineering limit is headless-only | `conditionLimit` | `src/model/conditioning.js` | [CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md) |
| Newton-Euler cycle demand with adaptive time sampling | Cycle sampling select | `cycleSampling` | `src/model/cycle.js`, `cycle-sampling.js` | [CYCLE_MODEL.md](./CYCLE_MODEL.md) |
| Rod load sharing and actuator utilization | Always on | none | `src/model/load-sharing.js` | [CYCLE_MODEL.md](./CYCLE_MODEL.md#rod-load-sharing) |
| Physical Cartesian stiffness/compliance | JSON `stiffness_model` | same | `src/model/compliance.js` | [COMPLIANCE_MODEL.md](./COMPLIANCE_MODEL.md) |
| Static payload-support check across the workspace | JSON `workspace_payload_support` | same | `src/workspace/payload-support.js` | [RESULTS.md](./RESULTS.md#static-payload-support-across-the-workspace) |
| Module-worker execution, live dashboard, cancel, fallback | Run, Cancel, Run on main thread, dashboard | `Optimizer.run/stop` | `src/ui/worker-*.js`, `run-dashboard.js` | [WORKER_PROTOCOL.md](./WORKER_PROTOCOL.md) |
| Candidate chart, ranking, selection, JSON display and copy | Result browser, Copy JSON | `selectCandidate`, `exportBest` | `src/ui/results-view.js`, `src/io/results.js` | [RESULTS.md](./RESULTS.md) |
| Layout JSON, Fusion script and coordinate CSV downloads | Format select + Download | `exportBest()`, `buildConstructionSkeleton` | `src/io/results.js`, `cad.js` | [CAD.md](./CAD.md) |
| Reference-layout import, exact retention, seeded variations | Reference Layout JSON, Load Layout JSON, Clear Reference | `referenceLayout` | `src/io/layout-import.js`, `src/optimization/reference-seeding.js` | [IMPORT.md](./IMPORT.md) |
| WebGL2 simulator with pose requests, animation, input devices | Simulate tab | `createSimulatorController` | `src/simulator/*` | [SIMULATOR.md](./SIMULATOR.md) |
| Editable geometry copies (parametric or explicit) | Mechanical geometry panel | `editGeometry` | `src/simulator/geometry-editor.js`, `geometry-controls.js` | [SIMULATOR.md](./SIMULATOR.md#mechanical-geometry-controls) |
| Live per-leg diagnostics with editable limits | Pose diagnostics panel | `buildPoseDiagnostics` | `src/simulator/diagnostics.js` | [SIMULATOR.md](./SIMULATOR.md#live-diagnostics) |
| Simulator JSON transfer and optimizer reference hand-off | Use geometry as optimizer reference, Load optimizer reference, Download simulator JSON | none | `src/ui/app.js` | [SIMULATOR.md](./SIMULATOR.md#browser-workflow) |
| Browser-local workspace save/restore | Save in browser, Restore saved workspace, Delete browser save | none | `src/ui/local-workspace.js` | this document |
| Deterministic replay from exported JSON | Downloaded `run.effective_settings` | `Optimizer.fromReplay` | `src/optimization/optimizer.js`, `random.js` | [RESULTS.md](./RESULTS.md#downloaded-layout) |

## 1. Requirements input

The Requirements textarea accepts a JSON document that is either flat or grouped into `payload`, `workspace`, `rotations` and optional `constraints`; in the grouped form, constraint keys left at the top level are merged into `constraints`, and a key present in both places is an error. The bundled sample (`examples/sample-requirements.json`) loads automatically at startup and again from **Load Sample Requirements**; loading the sample resets every derived control (range rows, ball-joint limit, home-height bounds, servo rating fields). **Clear** empties the requirements and discards the current optimizer, results, simulator layout and dashboard. A note under the textarea restates the motion cycle in plain terms (for the legacy cycle: `cycle_mm` is peak-to-peak and centred on home, so 30 mm on Z is ±15 mm) and, in the error colour, warns for each cycle axis that swings past its workspace range, since the sweep and its coverage do not include those poses and the cycle does not widen the workspace; it updates when requirements load, when the JSON is edited and parses, and on Run, and clears with **Clear**. See [REQUIREMENTS.md](./REQUIREMENTS.md#cycle-and-workspace-are-separate-demands).

A collapsible **Requirements reference** panel inside the page repeats the field table from [REQUIREMENTS.md](./REQUIREMENTS.md). The two copies are maintained by hand.

### Payload and motion

| Input | Purpose |
| --- | --- |
| `mass_kg` | Combined platform plus payload mass. Zero removes the modeled load. |
| `cycle_mm`, `frequency_hz`, `cycle_axis` | Legacy single-axis sinusoidal cycle (peak-to-peak stroke along x, y or z). Reported as `trajectorySource: "legacy-cycle"`. |
| `trajectory` | Multi-axis common-frequency sinusoid: `{ frequency_hz, components: [{ axis, amplitude_mm | amplitude_deg, phase_deg }] }`. Each axis may appear once; translation axes take `amplitude_mm`, rotation axes take `amplitude_deg`. Optional `type` must be `"sinusoid"`. Supplying `trajectory` together with any legacy cycle field is an error. |
| `center_of_mass_mm`, `inertia_kg_m2`, `external_force_n`, `external_moment_nm` | Optional rigid-body properties. Any one of them switches the mass model from `legacy-point` to `rigid-body`. The inertia tensor is validated for symmetry, positive semidefiniteness and the principal-moment triangle inequality. |

### Workspace and rotations

Six ranges (`x/y/z_range_mm`, `rx/ry/rz_range_deg`) as `[min, max]`, `{min, max}` or `{from, to}` objects. Equal bounds fix an axis. The parser derives a grid step of half the span per axis (three grid samples: min, midpoint, max); a zero-span axis gets a single sample.

### Constraints and defaults

| Field | Default when omitted |
| --- | --- |
| `ball_joint_max_deg` | 45 |
| `link_clearance_mm` | 6 (minimum distance between the centre lines of two legs' horns or rods; 0 turns the check off) |
| `servo_travel_bounds_deg` | [-120, 120] (or symmetric `servo_max_deg` when supplied) |
| `rod_length_bounds_mm` | [160, 420] |
| `horn_length_bounds_mm` | [30, 110] |
| `home_height_bounds_mm` | [50, 450] |
| `servo_rating_policy` | `enforced` |

The bundled sample overrides the joint limit (52°), rod bounds (180–380 mm) and horn bounds (40–110 mm).

### Servo ratings (JSON)

Shared keys: `servo_torque_rating_nm`, `servo_speed_rating_deg_s`, `servo_continuous_torque_rating_nm`, `servo_torque_speed_curve`, `servo_duration_ratings`, `servo_actuator`. `per_servo_ratings` is a six-entry array whose entries may override any of those per servo with `torque_nm`, `speed_deg_s`, `continuous_torque_nm`, `torque_speed_curve`, `duration_ratings`, `actuator`, or be `null` to inherit. A torque-speed curve lists strictly increasing nonnegative speeds with one torque per speed, `quadrants` of `symmetric` (default) or `motoring-braking`, and an optional `braking_torque_nm` array (same length as the speeds, only valid with `motoring-braking`). Duration ratings are `[{ torque_nm, duration_s }]`. The actuator model takes `output_inertia_kg_m2`, `viscous_nm_s_per_rad` and `coulomb_nm`, each defaulting to zero.

### Physical stiffness model (JSON)

`stiffness_model` requires `servo_torsional_stiffness_nm_per_rad` (one number or six) and exactly one rod description: `rod_axial_stiffness_n_per_m`, `rod_material` (`youngs_modulus_gpa` with `area_mm2` or `diameter_mm`), or `rods: "rigid"`. Optional `characteristic_length_mm`, `test_wrenches` and `use_as_objective`. See [COMPLIANCE_MODEL.md](./COMPLIANCE_MODEL.md).

### Static payload support (JSON)

`workspace_payload_support` is `{ rating: "continuous" | "peak", policy: "enforced" | "advisory" }` with both fields defaulting to the first value. It has no UI control.

### Validation behavior

Errors name the offending field and are raised before any evaluation. Numeric strings, `null` optional fields and out-of-domain values are rejected. Valid input does not imply a feasible design.

## 2. Optimization controls

The **Optimization Parameters** panel is collapsed by default. Explicit edits to a control survive later JSON edits and Run; an untouched control follows the JSON default.

| Control | Options / domain | Default | Notes |
| --- | --- | --- | --- |
| Layout Topology | C3 paired, Circular, Rectangular paired, Free | C3 paired | An imported reference forces its own topology and the select follows it. |
| Horn direction (C3) | Outward, Inward, Search both | Outward | Shown only for C3 paired; other topologies always run outward. Outward keeps seeded results unchanged; Inward and Search both start `beta_offset` anywhere within ±90°, and Search both also evolves the direction. A C3 reference's own direction overrides Outward/Inward. See [TOPOLOGIES.md](./TOPOLOGIES.md#c3-horn-direction-in-a-run). |
| Home Height Min / Max (mm) | positive, max ≥ min | 50 / 450 | UI value wins over `home_height_bounds_mm`, which wins over the design-space default. |
| Generations | integer ≥ 1 | 5 | |
| Population Size | integer ≥ 4 | 12 | |
| Objective set | Compact (4 objectives), Full (9 objectives) | Compact | See [Objectives](#objective-sets). |
| Mutation rate | 0 to 1 inclusive | 0.35 | Probability that each offspring is mutated after crossover. |
| Workspace sampling | Halton 1,024 (default), Halton 256, Halton 4,096, Cartesian grid | Halton 1,024 | Grid uses the axis steps; Halton ignores them. |
| Cycle sampling | Adaptive up to 256 samples at 0.5% (default); Adaptive up to 1,024 at 0.1%; Fixed 64 phases (legacy) | Adaptive 256 | See [Cycle demand](#cycle-demand). |
| Run seed | integer 1 to 4,294,967,295 | 1 | Reused until edited. **Randomize** draws a new 32-bit seed from `crypto.getRandomValues`. |
| Workspace Sweep Ranges | six rows of min / max / step | from requirements | Steps must be positive and apply only to the grid strategy. |
| Ball Joint Limit (deg) | 0 to 180 | from requirements (45, or 52 in the sample) | Shared limit for both sockets. Separate lower/upper limits are headless-only. |
| Link Clearance (mm) | 0 or more | from requirements (6) | Minimum distance between any two legs' horn or rod centre lines; a closer pose is a link collision. 0 turns the check off. An emptied field uses the requirements value. |
| Explore beyond ball-joint limits | checkbox | off | Enables soft exploration; adds `relaxed_coverage` without changing feasible coverage. |
| Servo ratings panel | shared torque, speed, continuous torque, policy; per-servo torque and speed for servos 1–6 | from requirements | Cleared field removes that rating. Curves, duration ratings and actuator models are JSON-only and are preserved when the UI overrides per-servo peak values. |

Not exposed in the UI (headless constructor only): `conditionLimit`, `lowerBallJointLimitDeg`, `upperBallJointLimitDeg`, custom `cycleSampling` values (including `inconclusivePolicy: "advisory"`), `designSpace` overrides, `sampling.sequenceStart`.

## 3. Search engine

### Algorithm

The optimizer is an NSGA-II loop implemented in `src/optimization/optimizer.js` and `nsga2.js`:

1. Build the initial population (random layouts, or the seeded composition when a reference is supplied).
2. Evaluate every layout, then run fast non-dominated sorting and crowding-distance assignment.
3. For each generation, create `populationSize` offspring by binary tournament selection (rank, then crowding, then coin flip), crossover, and probabilistic mutation. Evaluate them, merge with parents, re-sort, and keep `populationSize` survivors by front order and crowding distance.
4. Emit a checkpoint after the initial population and after every completed generation.

Dominance is constraint-first: `compareFeasibility` orders passing candidates ahead of diagnostic ones, and among diagnostics prefers fewer failed categories, then higher feasible coverage, then lower available torque and speed demand. Only candidates that tie on that ordering are compared by their objective vectors. An invalid or nonfinite objective is treated as negative infinity.

Total candidate evaluations per run are `populationSize × (generations + 1)`; the default run evaluates 72 candidates.

### Objective sets

| Set | Objectives (direction) |
| --- | --- |
| `compact` (default) | coverage (max), conditioningQuality (max), torque (min), speedDemand (min) |
| `full` | compact plus dexterity (max), stiffness (max), loadSharing (max), limitMargin (max), fatigue (min) |
| `full-v1` | replay-only; `full` with the legacy `loadBalance` proxy in place of `loadSharing` |
| `legacy-v2` | replay-only ten-slot set from older exports |

When a `stiffness_model` is supplied, the physical `physicalStiffness` metric replaces the scale-free `stiffness` proxy in the Full set unless `use_as_objective` is `false` (the proxy cannot tell a 2× larger mechanism with the same servos is 4× softer). Every metric is still computed and exported regardless of the selected set.

### Layout representation

A layout carries `baseAnchors`, `platformAnchors` (mm), `betaAngles` (rad), shared `hornLength`, `rodLength`, `homeHeight` (mm), `servoRangeRad`, `topology`, `topologyParameters` and `mounting`. Parametric topologies regenerate all six anchors and beta angles from their parameters so mutation and crossover preserve the family invariant; Free layouts evolve anchors and beta angles individually.

### Topologies

| Topology | Invariant | Parameters |
| --- | --- | --- |
| `circular` | Six equally spaced anchors per centered coplanar ring; alternating horn offsets | `base_radius`, `platform_radius`, `base_orientation`, `platform_orientation`, `beta_offset`, `beta_pair_offset` |
| `c3_paired` (default) | Three base pairs every 120°, platform pairs locked 60° between them; each base pair's legs go to the two neighbouring platform pairs, forming three triangles; mirrored servos per pair; pair gap is a chord | `base_radius`, `platform_radius`, `base_orientation`, `beta_offset` (within ±90°), `base_pair_gap`, `platform_pair_gap`, optional `horn_direction` (`outward` default, or `inward`) |
| `rectangular_paired` | Three rows of left/right pairs on a rotated rectangle; radius is the corner distance | circular fields plus `base_aspect`, `platform_aspect` |
| `free` | None | `{}` |

New Circular and Rectangular layouts start `beta_pair_offset` at 30°. A declared topology is validated against the actual coordinates on every import, mutation and crossover; an absent topology means Free.

### Design space and operator constants

These values are fixed in `DEFAULT_DESIGN_SPACE` (`src/optimization/layout-operators.js`) and can only be changed through the headless `designSpace` option. Horn, rod and home-height bounds supplied in the requirements JSON take precedence over the `designSpace` value:

| Constant | Value | Role |
| --- | --- | --- |
| `baseRadius` | [90, 160] mm | Base anchor radius bounds (Free: per anchor; parametric: ring or corner radius) |
| `platformRadius` | [40, 120] mm | Platform anchor radius bounds |
| `hornLengthBounds` | [30, 120] mm headless default; parsed requirements default to [30, 110] | Clamped at finalization |
| `rodLengthBounds` | [160, 420] mm | Clamped at finalization |
| `homeHeightBounds` | [50, 450] mm | Clamped at finalization; overridden by UI or requirements |
| `pairGapBounds` | [12, 45] mm | C3 pair gap; the ceiling per plate is also capped at 1.2 × that plate's radius |
| `rectangularAspectBounds` | [0.6, 1.4] | Width/depth ratio |
| `betaJitterRad` | 20° | Random `beta_offset` range for new parametric layouts (C3 Inward and Search both use ±90°); Free beta jitter |
| `anchorJitter`, `platformJitter` | 6 mm | Gaussian mutation scale for radii, gaps and Free anchor XY |
| `baseZJitter` | 2 mm | Free base anchor Z range and mutation scale |
| `mutationHorn`, `mutationRod`, `mutationHeight` | 4, 6, 15 mm | Gaussian mutation scales |
| `mutationAngle` | 4° | Gaussian scale for orientations, offsets and Free beta angles |

Crossover copies parent A, then for parametric layouts takes each topology parameter from either parent with equal probability; for Free layouts it takes legs from a random split index onward from parent B. Horn and rod lengths are averaged, home height comes from either parent, and the child is re-bounded before finalization so out-of-bounds diagnostic references cannot produce invalid radius/gap combinations.

### Randomness and reproducibility

All stochastic choices use a seeded `mulberry32-v1` generator. The run seed drives candidate evolution and the Halton `sequenceStart`; per-candidate reservoir sampling of example poses uses a generator derived from the seed and the candidate ID. Equal effective settings under the same model version reproduce identical candidate layouts and sampled results.

## 4. Candidate evaluation model

`evaluateLayout` (`src/optimization/evaluate-layout.js`) runs four stages per candidate and assembles metrics, feasibility flags and objectives. Direct callers may omit what the `Optimizer` always supplies: a missing ball-joint limit resolves to the shared 45° default for the sweep, home pose, cycle, limit margin and fatigue alike; a `trajectory` passed without `trajectorySource` is treated as `supplied` (and `stroke`/`frequency` derive from it when omitted), while `trajectorySource: 'supplied'` without a trajectory is rejected. Any supplied `trajectory` is run through `normalizeTrajectory`, so the JSON shape (omitted `phase_deg`, mixed-case axes) evaluates exactly as the `Optimizer` path does and a malformed trajectory throws instead of producing a silently invalid cycle.

### Workspace sweep

Every sampled six-axis pose is checked by the shared pose evaluator (`src/model/pose.js`). For Halton sampling, poses come from radical inverses in bases 2, 3, 5, 7, 11, 13 starting at the recorded `sequenceStart`; a fixed axis stays at its bound and a moving axis never lands exactly on an endpoint. Cartesian grid starts at each minimum and advances by the step. The sweep yields to the event loop before starting and after every 256 poses, checks the abort signal at each yield, and streams statistics with fixed memory. Example poses per class (reachable, unreachable, violations) are reservoir samples capped at 200.

Limits: at most 100,000 workspace poses per layout and 1,000,000 budgeted pose checks per run, both rejected before evaluation and, in the UI, before the previous run's results are cleared.

### Pose evaluator and constraints

For each leg the evaluator solves the single inverse-kinematics branch (`alpha = asin(g / hypot(e, f)) - atan2(f, e)`), then rejects in order: degenerate or invalid geometry, servo travel outside `servoRangeRad`, rod length outside the tolerance (0.5 mm by default), missing socket directions, lower or upper socket deflection beyond its limit, and finally whole-platform conditioning failures. Evaluation stops at the first hard structural failure but checks both sockets for every structurally valid leg.

Reachability levels reported per pose: `geometricallyReachable` (IK, travel, rod length, socket availability), `mechanicallyReachable` (adds both socket limits and link clearance), `reachable` (adds numerical and optional engineering conditioning), and `relaxedReachable` (ignores socket excess only when soft exploration is on; a link collision still fails it).

### Link collisions

Once every leg solves, each leg's horn (base anchor to horn tip) and rod (horn tip to platform joint) are treated as straight segments, and every link of one leg is checked against every link of each other leg. A pair whose centre lines come closer than `linkClearanceMm` (default 6 mm, `link_clearance_mm` in requirements, the **Link Clearance** control in the UI) is a `linkCollision` violation naming both legs and links. The two links of one leg share their joint and are not checked against each other. Servo bodies, joint housings, the plates and the payload are not modelled, and poses between samples are not checked. See [JOINT_MODEL.md](./JOINT_MODEL.md#link-collisions).

### Joint model

Each leg has a lower socket on the horn and an upper socket on the platform. Directions default to alignment with the actual rod at the home pose and can be overridden per leg through `layout.mounting`. Lower directions live in the moving horn frame; upper directions live in the platform frame. Derived directions are recalculated whenever geometry changes. See [JOINT_MODEL.md](./JOINT_MODEL.md).

### Conditioning

A dimensionless six-by-six rotary-actuator Jacobian is formed at every complete pose from actual rod directions, anchor offsets about the platform-anchor centroid, the RMS anchor radius and horn leverage. A one-sided Jacobi SVD supplies its singular values. Any pose with rank deficiency, degenerate leverage or `σmin/σmax ≤ 1e-10` fails the mandatory numerical check. An optional `conditionLimit` (≥ 1) additionally rejects poses whose condition number exceeds it. See [CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md).

### Home pose

The home pose (zero translation and rotation at `home_height`) is evaluated separately from the sweep, counts as one work unit, and supplies `dexterity` (home reciprocal condition), the fallback `stiffness` proxy, the physical compliance operating pose and CAD export geometry.

### Cycle demand

The cycle model (`cycle-newton-euler-v1`, `src/model/cycle.js`) evaluates the requested trajectory over one period:

- **Trajectory**: the legacy fields become a single-axis sinusoid with amplitude `cycle_mm / 2`; a supplied trajectory is a multi-axis sinusoid. Euler rates are converted to true angular velocity and acceleration. Zero frequency or all-zero amplitudes evaluate one stationary home pose.
- **Time sampling**: the `uniform` policy evaluates N equally spaced samples (64 reproduces the original schedule). The default `adaptive` policy evaluates 32 initial samples plus the midpoint of every interval (so the first 64 samples equal the legacy grid), then repeatedly bisects the interval with the largest normalized unresolved estimate across signed torque, signed speed, reciprocal conditioning, both socket angles and servo angles, until every estimate is within the tolerance (`converged`) or the sample budget is reached (`budget-limited`). A budget-limited cycle is inconclusive and, under the default `enforced` policy, adds the `cycle_convergence` failure category.
- **Dynamics**: at each sample the six rod axial forces are solved from the Newton-Euler wrench (gravity, linear and angular acceleration, external force and moment, offset center of mass, inertia tensor). Signed servo speed and acceleration come from the physical rotary mapping and the second derivative of the rod-length constraint; signed torque is rod force times transmission.
- **Actuator torque**: with a reduced actuator model, output-shaft torque adds `J α̈ + b α̇ + c sign(α̇)`; otherwise it equals load torque. Per-servo peak and time-weighted RMS torque are reported.
- **Outputs**: peak absolute torque (N m), speed (rad/s) and acceleration (rad/s²), per-servo peaks, the limiting sample for each, sampling status and evaluated sample count, rod load sharing, conditioning extremes, and a plain-language model description. An invalid cycle reports the failed sample, time, pose, reason and violations with null demand.

Cycle poses always use the strict joint policy, both socket limits, the numerical singularity check and any engineering condition limit, even when workspace exploration is soft.

### Rod load sharing and actuator utilization

`cycle.loadSharing` (`rod-load-sharing-v2`) reports `balanceScore`, the time-weighted mean over loaded samples of the equal-share ratio ideal / max|f| (ideal = |F| / Σ|uᵢ · F̂|, the peak rod force if all six rods carried the required force equally in the same sense; 1 is perfect, opposing tension/compression rods score lower), the worst sample's ratio and time, the magnitude CV as a spread diagnostic, per-rod peak compression and tension, and a `zero-load` status instead of a perfect score when no sample carries load. `actuator_utilization` separately reports per-servo peak actuator torque, its CV, and utilization against peak torque ratings.

### Servo capacity

`servo_capacity` (`servo-capacity-v2`) compares each servo's demand with its effective rating:

| Group | Comparison |
| --- | --- |
| Peak | peak actuator torque vs `torque_nm`; peak speed vs `speed_deg_s`; every sampled signed (torque, speed) point vs the torque-speed envelope with linear interpolation and no extrapolation (`outOfDomain`), braking quadrants only when modeled |
| Continuous | time-weighted RMS actuator torque vs `continuous_torque_nm` |
| Duration-limited | worst moving-window RMS over each `duration_s` (periodic extension) vs its `torque_nm` |

Statuses are `below`, `at`, `above`, `unavailable`, `outOfDomain` or `unrated`; the overall status is the most severe. With the `enforced` policy an exceeded or unavailable rated demand adds the `servo_capacity` failure category; `advisory` keeps the status visible without disqualifying the candidate. An invalid cycle cannot pass an enforced rating. Ratings never change the demand model.

### Static payload support

With `workspace_payload_support`, every strictly feasible workspace pose also gets a zero-velocity holding-torque solve (`static-payload-support-v1`) using the same mass properties, gravity and external wrench as the cycle. Outcomes per pose are `supported`, `exceeded`, `partiallyRated`, `unrated` or `unavailable`; geometrically infeasible poses are `notEvaluated`. The summary reports counts, `qualifiedCoverage` (fully rated and supported poses as a percentage of all sampled poses, null when nothing is rated), per-servo peak holding torque with its pose and headroom, the worst margin, and up to 50 failing samples. The `enforced` policy adds the `payload_support` failure category for exceeded, partially rated or unavailable outcomes. Each check costs one extra work unit per feasible pose.

### Physical stiffness

With `stiffness_model`, the home-pose Cartesian stiffness matrix `K = Aᵀ diag(k_leg) A` is formed from servo torsional springs in series with axial rod springs. The export includes `K`, its inverse, per-leg stiffness and servo share, scaled eigenvalues, the characteristic length and, for each test wrench, the predicted translation, rotation and stored energy. The scalar `physical_stiffness` metric is the minimum eigenvalue of `S K S` with rotations scaled by the characteristic length. Without the model the metric is null and the status `unavailable`. See [COMPLIANCE_MODEL.md](./COMPLIANCE_MODEL.md).

### Metrics

| Internal key | JSON name | Direction | Unit | Meaning |
| --- | --- | --- | --- | --- |
| coverage | coverage | max | percent | Strictly feasible sampled workspace poses |
| relaxedCoverage | relaxed_coverage | max | percent | Soft-exploration coverage; equals coverage in strict mode |
| payloadCoverage | payload_coverage | max | percent | Capacity-qualified static-support coverage; null unless enabled and rated |
| conditioningQuality | conditioning_quality | max | ratio | Worst reciprocal condition among valid home and workspace poses |
| dexterity | dexterity | max | ratio | Home reciprocal condition |
| stiffness | stiffness | max | proxy | Mean minimum singular value over feasible poses, falling back to home |
| physicalStiffness | physical_stiffness | max | N/m | Minimum scaled eigenvalue of the physical stiffness matrix |
| torque | torque | min | N m | Peak sampled absolute servo load torque |
| speedDemand | speed_demand | min | rad/s | Peak sampled absolute servo speed |
| loadSharing | load_sharing | max | ratio | Mean equal-share ratio of solved rod forces (ideal same-sense peak / actual peak) |
| loadBalance | load_balance | max | proxy | Legacy directional leg-share proxy |
| isotropy | isotropy | max | ratio | Mean reciprocal condition over feasible poses |
| limitMargin | limit_margin | max | ratio | 1 − worst reachable socket deflection / limit, clipped to [0, 1] |
| fatigue | fatigue | min | rad/s | Cycle frequency × mean RMS servo excursion over the cycle (servo motion rate) |

Unavailable or nonfinite values export as `null`, never zero.

### Feasibility flags and failure categories

Each candidate carries independent flags: `homePoseSatisfied`, `sampledWorkspaceSatisfied` (coverage exactly 100%), `cycleSatisfied`, `conditionSatisfied`, `collisionSatisfied`, `cycleConvergenceSatisfied`, `servoCapacitySatisfied` with `servoCapacityEnforced`, and `payloadSupportSatisfied` with `payloadSupportEnforced`. `failedCategories` is derived from them:

| Category | Added when |
| --- | --- |
| `home` | Home pose fails |
| `workspace` | Any sampled pose fails |
| `cycle` | Cycle evaluation invalid |
| `conditioning` | Numerical singularity, engineering limit or unavailable conditioning at home, in the sweep or on the cycle |
| `collision` | Links of two legs came closer than the link clearance at home, at a sampled workspace pose or at an evaluated cycle sample |
| `cycle_convergence` | Adaptive sampling was budget-limited under the enforced policy |
| `servo_capacity` | Enforced ratings are exceeded or unavailable |
| `payload_support` | Enforced static support is exceeded, partially rated or unavailable |
| `geometry` | The exact imported reference lies outside the search bounds |

`joint` is declared in `src/contracts.js` but is never produced; socket failures surface through `workspace`, `home` or `cycle`. A candidate with no failed categories is `passing`; every other candidate is `diagnostic` and stays in the population, chart and exports.

## 5. Execution, progress and cancellation

### Work budget

Before a run starts, `estimateWork` computes:

```text
posesPerLayout = workspacePoses + (payloadSupport ? workspacePoses : 0) + cycleBudget + 1
totalPoses     = posesPerLayout × populationSize × (generations + 1)
```

`cycleBudget` is the policy's maximum (256 for the default adaptive policy, 1,024 for the fine preset, 64 for the fixed schedule) or 1 for a stationary cycle. The bundled sample with default settings budgets 72 × (1,024 + 256 + 1) = **92,232** pose checks; the Cartesian grid variant budgets 72 × (729 + 256 + 1) = 70,992. Convergence or early failure can leave actual work below the budget; the dashboard shows both.

### Worker execution

**Run Optimization** creates one module worker per run. The main thread's `WorkerOptimizer` sends `start` with the serialized effective settings and receives `started`, `progress` (bounded summary at most every 100 ms), `checkpoint` (a complete retained population after each generation), `result` and `error`, each tagged with a unique run ID. Worker startup that fails or exceeds 5 seconds terminates the worker and enables **Run on main thread**, an explicit fallback that runs the same optimizer with event-loop yielding. Runtime failures retain the last completed population as partial results.

### Dashboard

The live dashboard shows state, elapsed time, completed/total candidates, generation, front size, approximate remaining time (available after three completed candidates), actual versus budgeted pose work, and a one-line summary of the best completed candidate. It updates at most 10 times per second and shows exact terminal counts on completion. The pose-budget preflight runs before the previous run is replaced, so a rejected budget reports the error in the status line and leaves the previous results, candidate list, simulator candidate and dashboard on screen together.

### Cancellation

**Cancel** aborts the active sweep through an `AbortController` checked at every workspace yield and before every cycle sample. The last fully evaluated population is retained and labelled partial; an unfinished generation is discarded. Overlapping starts are rejected; controls are disabled during a run.

## 6. Results browser and exports

### Chart and candidate list

A Chart.js scatter plot places passing candidates in one series and diagnostic candidates in another, defaulting to torque versus speed demand. Both axes can switch among torque, speed, coverage, payload coverage, conditioning quality, dexterity, stiffness, physical stiffness, load sharing, load balance, isotropy, limit margin and fatigue. Coincident points are spread in a small ring so each remains clickable. The candidate select lists every retained candidate in rank order with its passing state or failed categories, labelling the exact reference when present, and is keyboard accessible. Clicking a point or choosing a list entry selects that candidate for the JSON panel, downloads and the simulator.

Ranking: passing before diagnostic; diagnostics by fewer failed categories, then higher coverage, then lower torque and speed; passing candidates by lower torque, then lower speed, then higher conditioning quality, then ID.

### Candidate summary and JSON panel

The summary line reports passing state, coverage, home/workspace/cycle flags, torque and speed, trajectory identity and mass model, cycle sampling status, rod load sharing, physical stiffness, static support, servo capacity and reference diagnostics. The **Execution JSON output** textarea shows `{ run, result }` for the selected candidate, including bounded example poses, and **Copy JSON** copies it to the clipboard with a selection fallback.

### Downloads

The **Format** select and **Download** button offer:

| Format | File | Availability |
| --- | --- | --- |
| Layout JSON | `optimized_layout.json` | Any selected candidate |
| Fusion Script | `stewart_construction.py` | Candidates whose home pose solves six valid legs |
| Coordinates CSV | `stewart_coordinates.csv` | Same as Fusion Script |

The Layout JSON contains the layout fields (`servo_range` in degrees), `schema_version` 2, `model_version` 2, identity and seed origin, reference diagnostics, `diagnostic` flag, `metadata`, `cycle`, `conditioning`, `servo_capacity`, `actuator_utilization`, `physical_stiffness`, `payload_support`, `feasibility`, `constraint_policy`, `workspace_counts`, `workspace_stats` and `run` with `effective_settings` for replay. The Fusion script creates named construction points, one plane and two construction-line sketches per leg in a new direct-design document, converting millimetres to Fusion's internal centimetres; the CSV lists base anchors, solved horn tips, platform points and both centroids with frames. See [RESULTS.md](./RESULTS.md) and [CAD.md](./CAD.md).

## 7. Reference-layout import and refinement

Paste a layout into **Reference Layout JSON** or pick a file with **Load Layout JSON**; **Clear Reference** returns to a fresh search. Accepted shapes are a plain layout, a downloaded layout with metadata, a `{ layout }` wrapper and the on-screen `{ run, result: { layout } }` wrapper. Import validates six finite anchors per plate, six beta angles, positive lengths and height, servo bounds with max strictly greater than min, supported schema and model versions, and that any declared topology reproduces the actual geometry (`topology_parameters` on a free layout must be an object or absent and is copied). Missing mounting data is derived as home-aligned sockets with a visible migration note; the note reports a legacy upgrade only when the file's `model_version` is older than the current model. Imported metrics are discarded.

Seeding for population N places 1 exact reference, `round((N − 1) / 4)` fresh random layouts and the remainder as mutated variations (12 → 1 + 8 + 3). The exact reference is never finalized or repaired, is re-evaluated under the current model, is forced back into every survivor set, and is labelled **Exact reference** in the results. Its search-bound conflicts (lengths, height, servo range, radii, gaps, aspects, base Z, platform Z) and home-pose violations are recorded as reference diagnostics; bounds conflicts add the `geometry` failure category. Bounds are compared with a relative tolerance of 1e-9 (absolute 1e-9 near zero) so a downloaded candidate, whose `servo_range` degrees do not round-trip exactly through radians, re-imports without a conflict. The run's effective settings store the original reference and seed composition so the run can be replayed. See [IMPORT.md](./IMPORT.md).

## 8. Integrated simulator

The **Simulate** tab shares the retained candidate selection with Optimize and loads the selected candidate automatically. Tab switches keep pose, camera, input mode and animation state. The **Optimizer candidate** select lists the run's candidates plus an **Imported reference** entry after a reference load; the entry reloads that import when chosen again, and the select stays enabled whenever it has an entry, even without a run.

### Pose control

- Six axis rows with paired sliders (±50 mm translation, ±30° rotation) and numeric fields that accept any finite value. The X/Y/Z and Rx/Ry/Rz labels are drawn in the red, green and blue of the axes in the view.
- **Move along** chooses the axes the incremental inputs act along: **Base axes** (default, the fixed world frame) or **Platform axes** (the platform's current rotated frame). It covers the X/Y/Z rows and sliders, the arrow, Page and W/S, A/D, Q/E keys, the mouse drag and the gamepad; rotations turn about the chosen axes through the platform origin, which stays put. The Rx/Ry/Rz rows always show the stored base-frame Euler angles. Switching never moves the platform; poses are always stored and evaluated in the base frame, and the choice is saved as `simulator.inputFrame`.
- **Reset pose** returns to home.
- Keyboard: arrows move X/Y, Page Up/Down move Z, W/S tilt about X, A/D tilt about Y, Q/E rotate about Z; Shift doubles the 1 mm / 1° step. Keys are ignored while typing in a field.
- Mouse: **Orbit camera** (default) drags yaw/pitch; **Move platform** drags X/Y pose requests instead. In either mode the wheel zooms toward the point under the cursor, between 10 and 2,500 mm from the camera target, and Shift+drag pans. The renderer's depth range follows the camera distance so far views keep near/far line ordering, and lines crossing the near plane are clipped rather than dropped in a close-up. Pose changes keep a zoomed or panned view; the target recentres only when a layout with a different home height loads. **Reset camera** restores the default orientation and distance and recentres the target.
- Gamepad (checkbox): left stick moves X/Y, right stick tilts, triggers move Z and shoulder buttons yaw, with a 0.15 dead zone.

Every request is evaluated by the same pose evaluator as the optimizer. An accepted request becomes the rendered pose; a rejected request is reported with its violations while the last accepted pose stays rendered. Home failing at load leaves no accepted pose.

### Animation, markers and traces

Patterns: Wobble, Ping-pong, Rotation, Tilt, Helical, at a speed multiplier of 0.1 to 5. A rejected animation frame pauses playback with the failure reason; any manual request also pauses. Markers toggle anchor, horn-tip and platform points; Traces record up to 300 accepted platform-origin positions and can be cleared.

### Rendering

A native WebGL2 line/point renderer draws the base polygon, servo direction stubs, horns, rods, the platform polygon, platform axes, the world axes, a ground grid, the requirement workspace box, servo travel arcs, ball-joint socket cones, an optional reachability point cloud, an optional translation conditioning ellipsoid, optional rod-force and servo-torque loads and a dimmed ghost of a rejected request (all but the mechanism itself can be hidden under **Overlays**), colouring each failure in the requested pose once (a link collision on both of its legs): on the ghost leg (red) or ghost platform outline (magenta, for a global conditioning failure) when the ghost can draw it, otherwise on the held accepted legs. If WebGL2 is unavailable the tab shows an actionable error and the optimizer remains usable. If the browser loses the WebGL2 context, the pose status reports it, the renderer rebuilds its program and buffers when the context is restored and redraws; pose requests are still evaluated meanwhile. The six pose fields and sliders keep text the user is typing during animation, follow every request when untouched (a committed entry counts as untouched), and are restored after an invalid entry.

### Overlays

The **Overlays** group toggles optional scene layers. Ground grid, rejected pose ghost, platform axes and world axes are on by default; servo arcs, joint cones, workspace box, reachability cloud, conditioning ellipsoid and loads are off: **Ground grid** (dim lines every 25 mm on the base plane, z = 0, out to 1.5 times the base radius; the pitch is fixed), **Servo arcs** (each servo's allowed travel as a horn-length arc about its base anchor, with stop ticks and a marker at the accepted horn angle; grey normally, yellow within 5° of a stop, red when the requested pose breaks that servo's range), **Joint cones** (each lower and upper ball-joint socket's allowed cone at the accepted pose, apex at the rod end, axis along the socket normal, half-angle equal to the effective joint limit and redrawn as soon as a limit is edited; same grey, yellow and red scheme; a limit picture, not a collision check), **Workspace box** (the twelve edges of the requirement X/Y/Z ranges about home, the region the platform origin must reach; taken from the current run when a candidate is selected and from `simulator.workspaceRanges`, else the file's run bounds, when simulator JSON loads; rotation ranges are not drawn and a layout without ranges draws no box), **Reachability cloud** (off by default because it costs evaluator time; Halton samples of the platform origin inside the requirement X/Y/Z ranges, or ±100 mm on an axis without one, at the current requested rotation, each checked by the shared pose evaluator and drawn as a small point, green when reachable and dim red when not; **Cloud samples** chooses 256, 1,024 or 4,096 samples, **Z slice only** with **Slice Z (mm)** samples one plane at that Z offset from home instead; the sweep runs on the page 200 poses per frame, restarts from no points whenever the requested rotation, geometry, limits, ranges or these settings change, and its status line reports evaluated and reachable counts; the settings, not the samples, are saved as `simulator.reachability`, and the cloud shows evaluated samples only, never a continuous envelope), **Conditioning ellipsoid** (off by default; the translation manipulability ellipsoid of the accepted pose at the platform origin, with axes along the singular vectors of the Jacobian translation block and half-lengths proportional to their singular values, the largest drawn at the platform axis length, so it flattens as the pose nears a translational singularity; three axis lines and three great circles, coloured from light blue at reciprocal condition 1 to yellow at 0.01, or at 1 / condition limit when one is set, on a log scale, and magenta when the requested pose fails a conditioning check; the shape covers translation only and the colour the full six-axis condition; see [CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md#translation-conditioning-ellipsoid)), **Loads** (off by default, and disabled with a hint until the layout has a payload: a mass or an external force or moment from the selected candidate's run or from `simulator.loadModel`; rods shaded by their solved force at the accepted pose, toward blue in compression and orange-red in tension, fully coloured when one rod carries the whole static payload weight; each servo with a peak torque rating gets a thick band outside its travel arc that sweeps from mid-travel toward the stop in the torque's direction, reaching it at 100 % of the rating, graded green to yellow and red above 100 %; loads are static for a manual pose and include the pattern's velocity and acceleration at the playback speed while an animation plays, solved by the cycle model's `dynamicsAtPose`; see [SIMULATOR.md](./SIMULATOR.md#scene-builders-and-overlays)), **Rejected pose ghost** (when the latest request is rejected, its platform outline, platform axes and every leg the solver could reach, dimmed and without markers; failing legs in full red and a whole-platform failure in magenta, which then leaves the held pose in its normal colours; the accepted pose stays the solid layer), **Platform axes** (the accepted platform frame) and **World axes** (the fixed X/Y/Z frame at the origin). Each toggle is saved in simulator JSON and browser saves as `simulator.overlays`; a file without it keeps the current toggles. The info button beside the **Overlays** heading opens a panel under the toggles that explains each one in plain terms; it stays open while the page scrolls and the same button closes it. See [SIMULATOR.md](./SIMULATOR.md#scene-builders-and-overlays).

### Mechanical geometry controls

The panel edits an independent copy of the loaded layout; the optimizer's candidate is never changed. Parametric layouts expose their topology parameters (radii, pair spacing, aspect ratios, plate turns, horn direction offset, alternating horn offset for Circular and Rectangular, and a C3 **Horn direction** select, Outward by default or Inward) with paired sliders and numeric fields; all layouts expose horn length, rod length, home height and servo minimum/maximum. **Edit anchors explicitly** switches to Free while preserving coordinates and exposes each anchor coordinate and horn direction; **Generate selected topology** replaces explicit anchors with a suggested parametric layout. **Reset geometry** restores the last externally loaded layout. Invalid edits, including an empty or non-numeric field (never read as 0), leave the last valid layout in place, restore the field and report a field-specific error. A field that has keyboard focus keeps its text while an animation plays, so geometry can be edited mid-animation; a rejected edit still restores the stored value.

### Pose diagnostics

The diagnostics panel shows the requested and rendered accepted poses, editable lower and upper joint limits, rod-length tolerance and link clearance (changes re-evaluate both poses; a focused field keeps text the user has typed during animation, follows loads and settings changes when untouched, and is restored after a rejected edit), the active condition limit and the mandatory 1e-10 cutoff, the link clearance with the closest pair of links in the requested pose, whole-platform failures, and a per-leg table of lower/upper socket deflection against limits, maximum deflection, rod deviation against tolerance, servo angle, rod force (N, positive in compression) and signed servo torque against its peak rating with the utilisation percentage at the accepted pose, and requested-pose failures, where a link collision is listed on both legs it involves with the other leg and link. A line above the table says whether the loads are static or from the animation, or why there are none.

### Transfer

- **Use geometry as optimizer reference** writes the current simulator layout plus a `simulator` snapshot (source, requested and accepted poses, options, animation, markers, traces, overlay toggles, reachability cloud settings, workspace ranges in mm and degrees, load model in requirement keys and units, camera, pointer mode, input frame) and the originating run into the reference textarea and switches to Optimize.
- **Load optimizer reference** loads that textarea into the simulator, replaying saved poses, camera, animation (paused), pointer mode, input frame, markers, traces, overlay toggles, reachability cloud settings, workspace ranges and the load model (else the file run's payload and ratings), with the Markers, Traces and overlay checkboxes synced to the replayed state; an unrecognized or idle animation pattern defaults to Wobble.
- **Download simulator JSON** saves the same document as `stewart_simulator.json`.

## 9. Browser workspace save

**Save in browser** stores, under the `localStorage` key `stewart-optimizer.workspace.v1`, the requirements and reference textareas, every optimization control (topology, C3 horn direction, home-height bounds, population, generations, objective set, mutation rate, workspace and cycle sampling, seed, ball-joint limit and soft checkbox, all shared and per-servo rating fields, every axis min/max/step) and, when a layout is loaded, the full simulator JSON. **Restore saved workspace** and an automatic restore on page load (skipped, with a status note, when an optimization is already running) apply those values, re-populate derived controls when the saved requirements parse, and reload the saved simulator layout without activating the tab. **Delete browser save** removes the entry. Optimization results are not saved and must be regenerated. Saves are validated for shape and version before use, and the saved simulator document (layout, options, poses, camera, animation, markers, traces, overlay toggles, workspace ranges, mouse mode, input frame) is validated before any input or layout changes, so a rejected save leaves the current inputs and loaded layout as they were.

## 10. Headless API and replay

```js
import { Optimizer } from './src/optimization/optimizer.js';
import { parseRequirements } from './src/model/requirements.js';

const { normalized, workspace } = parseRequirements(text);
const optimizer = new Optimizer(normalized, { ranges: workspace, onProgress, onCheckpoint });
const outcome = await optimizer.run();      // { status, completedGenerations, completedEvaluations, partialResults }
const json = optimizer.exportBest();         // JSON string of the selected candidate
optimizer.selectCandidate(id);               // change the selection
optimizer.stop();                            // request cancellation
```

Constructor options: `populationSize` (12), `generations` (5), `ranges`, `sampling` (`{ strategy: 'halton' }`), `cycleSampling`, `seed` (1), `mutationRate` (0.35), `objectiveSet` (`compact`), `designSpace`, `topology` (`c3_paired`), `hornDirection` (`outward`; `inward` or `both` for C3 only), `referenceLayout`, `homeHeightBounds`, `ballJointLimitDeg` (45 when neither the option nor the requirements supply it), `lowerBallJointLimitDeg`, `upperBallJointLimitDeg`, `conditionLimit` (null), `linkClearanceMm` (6 when neither the option nor `link_clearance_mm` supplies it), `ballJointClamp` (false), `servoRatings`, `onProgress`, `onCheckpoint`.

`Optimizer.fromReplay(downloadedJSON)` rebuilds an optimizer from `run.effective_settings`, restoring requirements, bounds, sampling, cycle sampling (fixed 64 for exports that predate adaptive sampling), seed, population, generations, mutation rate, design space, topology, C3 horn direction (`outward` for exports without one), limits, ratings, objective set (mapping older Full runs to `full-v1`) and the original reference. Exports without effective settings or with a different random algorithm are rejected.

Root `math.js`, `workspace.js`, `cycle.js`, `requirements.js` and `optimizer.js` remain as compatibility shims; the root `Optimizer` adds a browser download on `exportBest()`.

## 11. Deployment, tooling and verification

| Command | Purpose |
| --- | --- |
| `npm run dev` | Static development server on 127.0.0.1:8000 (`-- --port N`, 0 for a free port) with path restrictions and no caching |
| `docker compose up -d --build` | nginx image serving the app on host port 8081, restarting after reboot |
| `npm test` | Node test suite under `tests/` (no installation needed) |
| `npm run smoke` | Runs the bundled sample headlessly and asserts 72 evaluations, a 92,232-pose budget, completion and a valid export |
| `npm run test:browser`, `test:browser:geometry`, `test:browser:diagnostics`, `test:browser:render` | Playwright checks in a real browser; they require `npm ci`, launch Google Chrome (the `chrome` channel) by default, fall back to Playwright's bundled Chromium when Chrome is missing, and honour `PLAYWRIGHT_CHANNEL` (a channel name or `bundled`) |
| `npm run build:charts`, `npm run build:icons` | Refresh the vendored Chart.js bundle and the Lucide info icon from devDependencies |
| `npm run legacy:build` | Rebuild the archived p5 simulator bundle after `npm --prefix archive/simulator ci` |

The active app has no build step, backend or runtime dependency other than the vendored Chart.js file.

## 12. Explicit non-features and limits

- Collision detection covers only the centre lines of the legs' horns and rods against each other, with one clearance. Servo bodies, joint housings, the plates and the payload are not checked, and neither is motion between evaluated poses.
- No PSO, hybrid search or surrogate model; the search is a single NSGA-II heuristic.
- Coverage, convergence and capacity statements describe evaluated samples only; nothing establishes continuous feasibility. The simulator's reachability cloud is likewise a set of evaluated samples at one orientation, not a continuous workspace envelope or boundary.
- Rods and horns are rigid and massless in the dynamics; the compliance model is a separate unloaded small-deflection estimate at home only.
- Servo electrical dynamics, control loops, static friction and thermal behavior are not modeled; RMS comparisons are screening metrics.
- Web Serial streaming of servo angles (issue #35) is not implemented.
- The archived p5 simulator uses a different layout format and legacy math; no compatibility is promised.
- Exported geometry and metrics are inputs to independent engineering analysis, not a manufacturing specification.
