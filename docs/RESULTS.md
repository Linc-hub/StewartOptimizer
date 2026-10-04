# Results, feasibility and export

## Coverage and constraint policy

`metadata.coverage` is 0-100 percent of sampled workspace poses satisfying all modeled geometry, servo travel, rod length (0.5 mm tolerance), lower and upper ball-joint, and actuator-conditioning constraints. Both sockets default to alignment with the actual rod at home and can have independent mounting-direction overrides. Their frame and sign conventions are documented in [JOINT_MODEL.md](./JOINT_MODEL.md); the dimensionless Jacobian and conditioning policy are documented in [CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md).

The default workspace strategy evaluates exactly 1,024 six-dimensional Halton poses; the UI also offers 256 and 4,096 poses or the original Cartesian grid. Halton uses radical inverses in bases 2, 3, 5, 7, 11 and 13, indexed from the recorded `sequenceStart` (seed by default). It maps each component into its inclusive input bounds, with rotation bounds converted from degrees to radians. A fixed axis stays exactly at its bound. For a nonfixed axis, Halton fractions are strictly between 0 and 1, so endpoints are not guaranteed samples. Grid starts at each minimum and advances by the recorded step without forcing a nonaligned maximum. The home pose is evaluated separately even if a workspace sample equals home; it is not added to the coverage denominator. Cycle poses are also separate. Coverage is never a claim about the continuous region.

Strict mode is the default. In optional soft-ball-joint mode, `metadata.relaxed_coverage` also counts otherwise valid poses that exceed the joint angle limit. It is an exploration score, not a feasibility claim. No positions or joints are physically clamped. Invalid geometry, servo travel, rod length, link collision and conditioning failures remain excluded. Strict mode's relaxed coverage equals its feasible coverage.

`constraint_policy` records mode, effective lower and upper ball-joint limits, the mandatory reciprocal cutoff, the optional engineering `conditionLimit`, and the `linkClearanceMm` used for link collisions. `workspace_counts` includes reachable, unreachable, relaxedReachable and violationPoses. Counts cover every sampled workspace pose; stored example poses are reservoir samples capped at 200 per class. `workspace_stats.violationCounts` counts observed violations by type, `jointViolationCounts` separates lower and upper failures, and `conditioningCounts` separates numerical, engineering, and unavailable failures. Evaluation may stop at the first hard geometry or servo failure, but checks both sockets for every structurally valid leg. `violationRate` counts poses with any violation, not the number of individual leg failures.

The three feasibility flags report sampled workspace satisfaction, home-pose satisfaction and cycle satisfaction independently. They describe the implemented checks only. `feasibility.failedCategories` lists every failed category from `FAILURE_CATEGORIES` in `src/contracts.js`, and `passing` is true only when that list is empty:

| Category | Added when |
| --- | --- |
| `home` | The home pose is not reachable |
| `workspace` | Sampled coverage is below 100% |
| `cycle` | The cycle evaluation is invalid (violated or unavailable) |
| `conditioning` | A numerical singularity, engineering limit or unavailable conditioning occurs at home, in the sweep or on the cycle |
| `collision` | Links of two legs came closer than the link clearance at home, at a sampled workspace pose or at an evaluated cycle sample (`feasibility.collisionSatisfied` is false) |
| `cycle_convergence` | Adaptive cycle sampling was budget-limited under the enforced inconclusive policy |
| `servo_capacity` | Enforced servo ratings are exceeded or unavailable |
| `payload_support` | Enforced static payload support is exceeded, partially rated or unavailable |
| `geometry` | The exact imported reference lies outside the effective search bounds |

`joint` is declared in the contract list for compatibility but is never produced; socket-limit failures surface through `home`, `workspace` or `cycle`. Diagnostic candidates stay in the population, chart, list and exports with their categories. The candidate browser includes every retained candidate, including diagnostic failures and coincident chart points. Passing candidates rank ahead of diagnostics. Diagnostics rank by fewer failed categories, then greater feasible coverage, then lower available torque/speed demand. Initial selection among passing candidates is lowest torque, then speed, then better conditioning; with no passing candidate, it uses diagnostic order. The X/Y chart defaults to torque versus speed and can use other available metrics. The keyboard-accessible candidate list changes the displayed and downloaded layout. A new run clears the prior selection.

## Static payload support across the workspace

Coverage above is geometric/mechanical feasibility only, and servo capacity is otherwise checked on the cycle about home. With `workspace_payload_support`, every strictly feasible workspace pose also gets a static holding check (`static-payload-support-v1`). It uses the same mass properties, gravity and external wrench as the cycle, with zero velocity and acceleration, the same actual-rod wrench balance and servo transmission, and so the same demand as the cycle evaluator at that pose. Tilting the platform changes both geometry and, with an offset center of mass, the gravity moment.

`payload_support` reports:
- the load case and rating semantics;
- counts of `supported` (all six servos rated and within), `exceeded`, `partiallyRated`, `unrated`, `unavailable` (singular or nonfinite equilibrium) and `notEvaluated` (geometrically infeasible) poses;
- `qualifiedCoverage`, the percentage of all sampled poses that are fully rated and supported, which is null when no applicable rating exists;
- per-servo peak holding torque with its pose, ratings and headroom;
- the worst margin;
- up to 50 failing samples.

`rating: "continuous"` (default) compares the constant holding torque with the continuous (RMS) torque rating; `rating: "peak"` uses the peak/stall rating and is disclosed as not implying sustained capability. A missing applicable rating is unrated, never substituted.

The enforced policy adds the `payload_support` failure category when any evaluated pose is exceeded, partially rated or unavailable. The advisory policy keeps these outcomes visible without disqualifying the candidate. With no applicable rating the check is demand-only. `metadata.payload_coverage` is the qualified coverage; geometric `coverage` is unchanged. This mode covers static support only; dynamic feasibility still comes from the cycle trajectory. The work budget adds one check per workspace pose against the existing 1,000,000 limit, and progress, cancellation and replay (`run.effective_settings.payloadSupport`) include it.

Supplied servo ratings add a separate capacity check to the feasibility flags. Enforced mode is the default: a rated demand that exceeds its limit or is unavailable makes the candidate diagnostic and affects ranking. Advisory mode keeps the capacity status and warning visible without disqualifying the candidate for capacity alone. No ratings means demand-only results.

## Metric meanings

The default Compact search maximizes feasible coverage and worst valid home/workspace reciprocal conditioning, and minimizes peak cycle torque and speed. Full adds home dexterity, geometric stiffness proxy, solved rod-force load sharing (`loadSharing`), sampled joint-limit margin proxy, and fatigue heuristic. When a `stiffness_model` is supplied, Full uses `physicalStiffness` in place of the scale-free stiffness proxy unless `use_as_objective` is `false`, and `objectiveDefinitions` records the choice; a run saved before this default (stiffness model without `use_as_objective`, `stiffness` in `objectiveDefinitions`) replays with the proxy. `full-v1` is the earlier Full set, which used the directional `loadBalance` proxy; exported Full runs whose `objectiveDefinitions` name `loadBalance` replay as `full-v1`. Metrics remain in diagnostic JSON even when they are not objectives. `run.effective_settings.objectiveSet` records one of `compact`, `full`, `full-v1` (a replayed older Full run) or `legacy-v2` (a replayed ten-slot snapshot); `objectiveDefinitions` records each selected key, direction, unit, and approximation label. Older ten-slot snapshots can still replay as `legacy-v2`.

| Export metadata | Calculation / interpretation |
| --- | --- |
| coverage | Feasible sampled workspace percentage |
| relaxed_coverage | Separate workspace exploration percentage |
| payload_coverage | Capacity-qualified static-support coverage percentage when `workspace_payload_support` is enabled and rated; otherwise null |
| conditioning_quality | Worst reciprocal condition among valid home/workspace samples, or null if unavailable |
| torque | Peak sampled absolute servo torque in N m, or null for an invalid cycle |
| speed_demand | Peak sampled absolute servo speed in rad/s, or null for an invalid cycle |
| dexterity | Valid home-pose reciprocal actuator condition, or null if unavailable |
| stiffness | Mean eligible minimum singular value, falling back to the home value; a dimensionless actuator proxy, not N/m |
| physical_stiffness | Minimum eigenvalue (N/m) of the home-pose Cartesian stiffness with rotations scaled by the characteristic length; null unless `stiffness_model` is supplied; see [COMPLIANCE_MODEL.md](./COMPLIANCE_MODEL.md) |
| isotropy | Mean reciprocal actuator condition across feasible workspace poses |
| load_sharing | Time-weighted mean over cycle samples with load of ideal / max(abs(f)), where ideal = abs(F) / Σ abs(uᵢ · F̂) is the peak rod force if all six rods shared the required force equally in the same sense; 1 is equal same-sense sharing, opposing rods score lower; null for zero load or an invalid cycle. See [CYCLE_MODEL.md](./CYCLE_MODEL.md#rod-load-sharing) |
| load_balance | Legacy diagnostic: mean 1/(1 + standard deviation of normalized absolute vertical base-to-platform leg directions); a directional proxy, not the solved cycle loads |
| limit_margin | 1 − (worst lower or upper socket deflection over the reachable sampled poses) / its limit, clipped to [0,1]; 0 with no reachable pose. Violating poses are excluded so the margin measures headroom instead of repeating coverage |
| footprint | Radius in mm of the vertical cylinder about Z holding every base anchor plus its horn length and every platform anchor |
| directional_stiffness | Worst directional stiffness proxy along the run's `stiffnessDirection` axis over home and the reachable sampled poses with satisfied conditioning, on the `stiffness` scale; null when the run set no direction. See [CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md#directional-stiffness) |
| fatigue | Servo motion rate, rad/s: cycle frequency × mean over the six servos of the time-weighted RMS servo-angle excursion about its cycle mean (`cycle.servoExcursionRmsRad`). Independent of the servo travel bounds and joint limits, 0 for a stationary cycle, null for an invalid cycle; a relative heuristic, not service life or a material fatigue model |

Actuator conditioning uses actual rods, servo leverage, centroid-referenced rotations, and RMS-radius-normalized translation. The mandatory numerical cutoff rejects zero or near-zero singular modes instead of omitting them. Failed samples remain in coverage and failure counts. The minimum singular value is a kinematic proxy, not calibrated physical stiffness. The cycle model uses actual rod directions and a separate equilibrium calculation, detailed in [CYCLE_MODEL.md](./CYCLE_MODEL.md).

## Downloaded layout

The download is `optimized_layout.json` with these top-level fields:

| Field | Meaning |
| --- | --- |
| base_anchors | Six [x,y,z] coordinates in the base frame, mm |
| platform_anchors | Six local moving-platform coordinates, mm |
| beta_angles | Six horn-plane orientation angles, radians |
| horn_length / rod_length | Shared horn and rod lengths, mm |
| servo_range | Shared [minimum, maximum] travel, degrees. Generated and evolved candidates carry the run's `servo_travel_bounds_deg` exactly, equal to `run.effective_settings.servoRangeDeg`; an exact imported reference keeps its own imported degrees |
| home_height | Platform home offset along Z, mm |
| schema_version / model_version | Layout format and physical model versions |
| id / topology / topology_parameters / mounting / migration | Selected candidate identity, geometry/model metadata, effective socket directions and sources, and the import migration record (`upgraded` true with the legacy-upgrade note for a pre-version-2 import, false with a derived-sockets note when a current-version import omitted `mounting`) when present |
| seed_origin / reference_diagnostics | Whether the candidate is the exact reference, a variation, fresh, or offspring; search-bound conflicts and home-pose reasons for the reference |
| diagnostic | True when this result fails an enforced requirement |
| metadata | Metrics described above |
| cycle | Validity, legacy axis (null for supplied trajectories), sample count, trajectory definition/identity/source, mass model, model version, peak torque/speed/acceleration, per-servo peaks, limiting samples, failure sample/time/reason when invalid |
| conditioning | Compact home, workspace, cycle, limit, and numerical threshold summary; nonfinite values are null |
| actuator_utilization | Per-servo peak output-shaft torque, its CV across servos, and utilization against peak torque ratings (`rated`, `partial`, `unrated`, `unavailable`); separate from rod-force balance |
| payload_support | Static workspace holding check (when enabled): load case, rating semantics, outcome counts, capacity-qualified coverage, peak holding torque, margins, bounded failing samples |
| physical_stiffness | Compliance model version, status, units, reference, stiffness and compliance matrices, leg stiffness and servo share, characteristic length and source, test-wrench displacements, assumptions |
| servo_capacity | Capacity model version, effective shared and per-servo ratings, source (`shared`, `override`, `unrated`), policy, demand reference, per-servo peak torque/speed, envelope (with limiting operating point), continuous RMS and duration-limited status and margins, grouped peak/continuous/duration status, worst fractional headroom |
| feasibility | Independent sampled-workspace, home-pose and cycle flags, cycle sampling status (`cycleConvergence`) and whether it satisfies its policy, passing status, failed categories, and scope text |
| constraint_policy | Strict/soft workspace policy, effective joint limits, mandatory numerical cutoff, and optional engineering condition limit |
| workspace_counts / workspace_stats | Full sweep counters and aggregated statistics |
| run | completed/cancelled status, completed generation count, partial flag, and effective replay settings |

The on-screen JSON has a `run`/`result` wrapper and includes bounded example poses; it is not byte-for-byte identical to the download. Pose samples store translation offsets in mm and Euler angles in radians. Internal servo angles/statistics are radians. Cycles are always checked strictly, even when workspace exploration is soft. Unavailable demand is null with an explicit reason, never an implicit zero.

Servo-capacity torque demand and rating use N m; speed demand and effective rating use rad/s. Each rated metric reports `below`, `at`, `above`, or `unavailable`, and envelopes may also report `outOfDomain`; an unrated metric reports `unrated`. Torque comparisons use output-shaft actuator torque (load torque unless an actuator model is supplied); see [CYCLE_MODEL.md](./CYCLE_MODEL.md#actuator-demand-and-servo-capacity). Headroom is rating minus demand, and fractional headroom divides by rating. Negative headroom means an exceeded rating. The worst fraction is the minimum across rated metrics when all rated demands are available; otherwise it is null. An invalid cycle cannot pass an enforced rating. The run's effective settings record the final rating policy and all effective per-servo ratings for replay.

`run.effective_settings.cycleModel` records the cycle model version, the rod load-sharing model version (`loadSharingModel`, currently `rod-load-sharing-v2`; a Full run saved with `rod-load-sharing-v1` or without the field was ranked with the older absolute-force CV score and does not replay to the same front), effective trajectory and its identity/source, and the normalized mass properties (SI). Replay uses the saved normalized requirements, bounds, sampling strategy/count and sequence start, seed, random algorithm, population and generation counts, mutation rate, design space, and effective joint/servo policies. `mulberry32-v1` drives candidate evolution and bounded example-pose retention. A seed is reused until changed explicitly; Randomize draws a new 32-bit seed from the browser. Equal effective settings reproduce candidate layouts and sampled results in the same model version. The work budget reserves all workspace poses, one home check and the cycle policy's maximum samples (256 by default, 64 for the legacy schedule) per candidate; convergence or early cycle failure may leave actual work below the budget. `run.effective_settings.cycleSampling` records the cycle schedule; see [CYCLE_MODEL.md](./CYCLE_MODEL.md).

`run.effective_settings.linkClearanceMm` records the link clearance in mm. A run saved before link collision checks has no such field and replays with a clearance of 0, which never reports a collision, so it reproduces its original results.

`run.effective_settings.hornDirection` and `legPairing` record the run's C3 choice modes (`outward`/`inward`/`both` and `triangulated`/`parallel`/`both`); runs saved without them replay as `outward` and `triangulated`. When a run searches more than one C3 configuration, `run.configuration_summary` lists one row per configuration: `configuration` (for example `inward/parallel`), `hornDirection`, `legPairing`, `retained` (layouts in the final population), `feasible` (of those, how many pass every constraint) and `bestCoverage` (the highest feasible coverage among them, percent, or `null`). Single-configuration runs omit the field. See [TOPOLOGIES.md](./TOPOLOGIES.md#searching-several-c3-configurations).

Every candidate's metrics include `footprint` (mm): the radius of the vertical cylinder about Z holding every base anchor plus its horn length and every platform anchor. `run.effective_settings.compactness` is `true` when the run also minimised it as an objective (then the last entry of `objectiveDefinitions`), and `run.effective_settings.boundsRelaxation` is the largest share the run could widen its geometry bounds by (0 for none); runs saved without them replay with neither. A relaxing run's export adds `run.bounds_relaxation`: `maxShare`, `finalShare` (how far it actually widened), `designSpace` (the widened bounds), `outsideNominal` (retained candidates outside the requested bounds) and `history` (one `{ generation, passing, share }` row per generation). Each of its candidates carries `bounds_excursions`, a list of `{ field, value, nominal: [min, max], excessMm }` for every bound it lies outside (negative `excessMm` below the minimum); an empty list means it is inside the requested bounds. See [TOPOLOGIES.md](./TOPOLOGIES.md#compactness-and-relaxed-geometry-bounds).

`run.effective_settings.stiffnessDirection` is `null` or the axis (`x`, `y`, `z`, `rx`, `ry`, `rz`) the run maximised as its last objective; `objectiveDefinitions` then ends with a `directionalStiffness` entry carrying that `axis`. Such a run's candidates also carry `workspace_stats.stiffnessDirection` and `worstDirectionalStiffness` and `conditioning.directional` (`direction`, `home`, `workspaceWorst`, `value`). Runs saved without the field replay with no direction.

An imported reference is reevaluated. Its old metadata is never trusted, and it can remain a selectable diagnostic even if outside search bounds or invalid at home. The downloaded `run.effective_settings.reference_layout` stores the original reference for replay. See [reference import](./IMPORT.md).

The selected candidate can be downloaded as JSON. Candidates with valid solved home geometry can also be exported as a [Fusion construction script and coordinate CSV](./CAD.md), including diagnostic candidates whose other requirements fail. The archived simulator uses a separate legacy format; no general compatibility guarantee is made.
