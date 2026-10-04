# Current optimizer architecture and reproduction guide

This guide describes the implemented root application. The supplemental v3/v4 specifications and earlier research guides are historical design proposals. For launch and input instructions, start with [README.md](../README.md) and [requirements](./REQUIREMENTS.md).

## Runtime

Run `npm run dev` with Node.js >=22 and open the printed URL. This local static server binds to 127.0.0.1; `-- --port 0` chooses a free port. Any other static server also works. The HTML loads a stylesheet, a locally bundled Chart.js script for the results plot, and `src/main.js`, which initializes native JavaScript modules. The sample loader resolves its JSON URL relative to its module. There is no backend, database, bundler or third-party math dependency in the active app.

The p5/quaternion/Stewart scripts belong only to the [archived simulator](../archive/simulator/README.md). Its prebuilt bundle includes separate third-party dependencies and has its own locked build command.

## Module boundaries

| Module | Owns |
| --- | --- |
| `src/ui/app.js` | Event handlers, status display and run/cancel control |
| `src/ui/worker-optimizer.js` | Browser adapter retaining selection/export and completed checkpoints |
| `src/ui/optimizer-worker.js`, `worker-runtime.js`, `worker-protocol.js` | Module worker entry, run lifecycle and serializable message shapes |
| `src/ui/run-dashboard.js` | Live bounded metrics, 10 Hz UI cap and run-state transitions |
| `src/ui/results-view.js` | Shared retained-candidate chart and selection |
| `src/ui/controls.js` | Workspace inputs, sampling/seed/cycle presets and preservation of explicit overrides |
| `src/ui/servo-ratings-controls.js` | Shared and per-servo rating fields with override preservation |
| `src/ui/local-workspace.js` | Capture, validation and restore of the browser-local workspace save |
| `src/ui/tooltips.js`, `download.js` | Browser-only interactions |
| `src/simulator/controller.js` | Copied active layout, requested/accepted poses, animation (with analytic derivatives), trace state and accepted-pose loads |
| `src/simulator/scene.js` | Ordered scene builders (lines and points from accepted geometry) and overlay toggles |
| `src/simulator/reachability.js` | Reachability cloud settings and the chunked, abortable translation sweep through the shared pose evaluator |
| `src/simulator/renderer.js`, `view.js` | Native WebGL2 drawing and camera/input handling |
| `src/simulator/input-frame.js` | Base or platform frame for incremental pose inputs: frame-local translation, rotation steps and Euler extraction |
| `src/simulator/geometry-editor.js`, `geometry-controls.js` | Explicit or parametric editable layout copies |
| `src/simulator/loads.js` | Simulator load model (payload and servo ratings in requirement keys) and rod force / servo torque at a solved pose through the cycle model |
| `src/simulator/diagnostics.js` | Evaluator-driven per-leg joint/rod diagnostics, rod force, servo torque and effective limits |
| `src/contracts.js` | Schema/model versions, topology names, metric keys with JSON names, units and directions, failure categories |
| `src/model/requirements.js` | Parsing, normalization and physical input validation |
| `src/model/trajectory.js`, `mass-properties.js` | Legacy and supplied sinusoidal trajectories with Euler-rate conversion; rigid-body mass, inertia and external wrench |
| `src/model/kinematics.js`, `pose.js` | Horn frame, servo-angle solve, and single-pose inverse kinematics with all constraints |
| `src/model/mounting.js` | Derived or supplied lower/upper socket directions and legacy migration |
| `src/model/conditioning.js` | Dimensionless rotary Jacobian, one-sided SVD, numerical and engineering condition checks |
| `src/model/cycle.js`, `cycle-sampling.js` | Newton-Euler rod-force balance, servo rate/acceleration, uniform and adaptive time schedules, periodic weights |
| `src/model/load-sharing.js` | Streamed rod-force sharing statistics (equal-share ratio, magnitude CV, peaks) and actuator utilization |
| `src/model/servo-ratings.js` | Rating normalization, envelopes, RMS/duration windows and capacity status |
| `src/model/compliance.js` | Physical Cartesian stiffness model and test-wrench predictions |
| `src/workspace/sampling.js` | Halton/grid pose generation, size estimate and the per-layout limit |
| `src/workspace/sweep.js` | Sweep iteration, yields, abort checks, progress and optional payload-support checks |
| `src/workspace/statistics.js` | Running metrics, counters and reservoir samples |
| `src/workspace/payload-support.js` | Static holding-torque outcomes and capacity-qualified coverage |
| `src/optimization/optimizer.js` | Run state and population lifecycle |
| `src/optimization/objectives.js` | Compact, Full and replay-only objective sets |
| `src/optimization/random.js` | Seeded `mulberry32-v1` generator |
| `src/optimization/layout-operators.js` | Generation, finalization, mutation and crossover |
| `src/optimization/reference-seeding.js` | Imported-reference composition, bounds checks and exact seed retention |
| `src/optimization/topology.js` | Symmetric anchor generators and declared-topology validation |
| `src/optimization/nsga2.js` | Ranking, crowding, tournament and survivor selection |
| `src/optimization/evaluate-layout.js` | Workspace/home/cycle scoring and objectives |
| `src/optimization/budget.js` | Run workload estimate and limits |
| `src/io/results.js` | Best-candidate selection and both output formats |
| `src/io/sample-requirements.js` | Fetching the bundled example |
| `src/io/layout-import.js` | Reference-layout parsing, field validation and mounting migration |
| `src/io/cad.js` | Home-pose construction skeleton, Fusion script and coordinate CSV |
| `src/math.js` | Shared numerical primitives |

The UI depends on the optimizer; the optimizer composes search and evaluation functions. The simulator controller, cycle and workspace evaluation all depend on the same pose evaluator, and the simulator's loads reuse the cycle model's `dynamicsAtPose`. The renderer draws accepted pose geometry, plus a dimmed ghost built from a rejected request's own evaluator result, and never solves constraints. Numerical modules never import the UI or manipulate the DOM. Abort signals and progress callbacks cross these boundaries explicitly. NSGA-II intentionally updates evaluation rank/crowding fields; layout mutation and crossover clone their inputs. See the [active simulator guide](./SIMULATOR.md).

## API compatibility

Root `math.js`, `workspace.js`, `cycle.js` and `requirements.js` retain their original exports through compatibility modules. Root `optimizer.js` preserves the original `Optimizer.exportBest()` browser download and overridable `download()` method.

New headless callers use the core directly:

```js
import { Optimizer } from './src/optimization/optimizer.js';
import { parseRequirements } from './src/model/requirements.js';

const { normalized, workspace } = parseRequirements(requirementsText);
const optimizer = new Optimizer(normalized, { ranges: workspace });
const outcome = await optimizer.start();
const json = optimizer.exportBest(); // JSON string, no browser side effects
```

These paths are relative to a caller at the repository root. The core keeps `start`, `run`, `stop`, progress and run-result semantics; its `exportBest()` returns a string when results exist. The UI passes that string to `src/ui/download.js`. Display and download retain their distinct documented JSON shapes while sharing field conversion and selection. Tests import `createApp` directly and await its `ready` promise for sample initialization.

## Data flow and geometry

1. `parseRequirements(text)` normalizes flat/nested input, validates it, and returns `{ normalized, workspace }`. The default sweep uses 1,024 six-dimensional Halton samples; optional Cartesian grid uses the configured axis steps and includes a maximum only when a step lands there.
2. The UI fills defaults, retaining explicit overrides at Run. An explicit optimizer joint-limit option wins over normalized requirements. Loading the sample resets the controls.
3. `Optimizer` defaults to the C3 paired topology. Circular, C3 paired, and rectangular paired layouts are regenerated from symmetry-preserving parameters during mutation and crossover; Free layouts evolve individual anchors. See [layout topologies](./TOPOLOGIES.md) for exact invariants, parameter names, and bounds.
4. `finalizeLayout` clamps shared lengths and the independently chosen home height to effective bounds. It preserves the topology's geometry; it never derives a replacement height from rods and horns.
5. `evaluateLayout` sweeps workspace poses, evaluates home geometry, evaluates the required cycle, and assembles objectives. None of these loops are stubs. Retained candidate selection is shared by the Optimize and Simulate tabs; geometry editing makes an independent simulator copy.

Direct `new Optimizer()` defaults are population 12, generations 5, mutation rate 0.35, joint limit 45 degrees (the shared `DEFAULT_BALL_JOINT_LIMIT_DEG`, also used by `evaluatePose`, `computeCycleDemand` and the requirements parser), horn bounds [30,120] mm and rod bounds [160,420] mm. The UI passes parser-normalized requirements: omitted JSON limits use 45 degrees, [30,110] mm and [160,420] mm. The bundled sample explicitly specifies 52 degrees, [40,110] mm and [180,380] mm. Both entry paths use [-120,120] degrees servo travel unless overridden. Prefer parsing requirements rather than constructing incomplete data by hand.

## Pose evaluator

Internal geometry uses mm and angles use radians. R = Rz * Ry * Rx rotates a local platform anchor. Add translation and the home-height Z offset to obtain q; l = q - base. With horn length h, rod length d and orientation beta:

```text
e = 2 h lz
f = 2 h (cos(beta) lx + sin(beta) ly)
g = dot(l,l) - (d*d - h*h)
alpha = asin(g / sqrt(e*e + f*f)) - atan2(f,e)
```

Reject degenerate/invalid geometry or a servo angle outside its range. Reconstruct the horn tip, check rod length within the effective tolerance (0.5 mm by default), both mounting-frame socket angles and mandatory actuator-conditioning threshold. An optional engineering condition limit can further reject poses. Only one inverse-kinematics branch is evaluated: of the two horn angles that close the rod, the one where raising the horn shortens the gap from horn tip to platform point. `reachable` always excludes joint and conditioning violations; optional soft exploration can mark an otherwise valid joint-limited pose `relaxedReachable`. No geometry is clamped. The simulator calls this same evaluator for every requested pose and keeps the last valid accepted pose when a request fails.

[Results documentation](./RESULTS.md) describes the geometric Jacobian proxies and their limitations. [Cycle documentation](./CYCLE_MODEL.md) describes the separate actual-rod equilibrium, gravity direction and torque/speed formulas. The old equal-load harmonic torque approximation is no longer used.

## Evolution and execution

The initial population and each generation's offspring are evaluated. Non-dominated sorting, crowding distance, tournament selection, crossover and probabilistic mutation form an NSGA-II search. Compact (default) uses coverage, conditioning quality, cycle torque and speed demand; Full adds dexterity, the stiffness proxy (or physical stiffness when a `stiffness_model` is supplied and `use_as_objective` is not `false`), solved rod-force load sharing, limit margin and the fatigue proxy. The earlier Full set with the directional load-balance proxy survives only as the replay-only `full-v1`. All diagnostic metrics remain available in either set. Dominance is constraint-first: passing candidates precede diagnostics, and diagnostics compare by failed-category count, coverage and demand before objectives are consulted. Invalid cycle demand receives the worst demand objective. The UI exposes a recorded seed and mutation rate for replay.

Preflight counts samples arithmetically before allocating range arrays. It rejects more than 100,000 workspace poses per layout or 1,000,000 total budgeted pose evaluations per run. Total work is:

```text
population * (generations + 1) * (workspace poses + payload checks + cycle budget + 1 home pose)
```

The cycle budget is the sampling policy's maximum: 256 for the default adaptive policy, 1,024 for the fine preset, 64 for the fixed legacy schedule, and 1 for a stationary cycle. Payload checks equal the workspace pose count when `workspace_payload_support` is enabled and are otherwise zero. The default sample budgets 72 * (1,024 + 256 + 1) = 92,232 checks (78,408 with the legacy schedule); convergence or early cycle failure can perform fewer actual checks. Progress distinguishes actual completed work from the budget and includes approximate elapsed/remaining time.

Workspace sweeps yield to the event loop before starting and every 256 poses. Statistics use running means; reservoir samples cap retained example poses at 200/class. A run's AbortController is checked during evaluation and after each yield. Cycle checks are bounded and observe the same signal. The browser creates one module worker per run; the headless API still runs directly. See [worker protocol](./WORKER_PROTOCOL.md).

`await optimizer.start()` or `await optimizer.run()` returns a completed/cancelled outcome. `optimizer.stop()` requests cancellation. Overlapping runs reject. Browser worker errors are reported as failed outcomes with any completed checkpoint retained. Cancellation retains the most recent fully evaluated population, not an unfinished sweep/generation. UI and download mark retained results partial. A worker startup failure offers an explicit main-thread fallback.

## Verification and reproduction

Run `npm test` for the checked-in regression suite, organized under `tests/model`, `workspace`, `optimization`, `ui`, `io`, `tooling` and `archive`. It covers normalization, controls, feasible/relaxed accounting, export, work limits, real event-loop yielding, cancellation, selection, cycle force balance and finite-difference speeds, and the archived bundle's custom-layout API. The workspace reference fixture was captured before the refactor at commit `97b7360`; it includes strict/soft/reachable results and capped samples. During the optimizer extraction, six seeded old/new runs across x/y/z cycles in strict/soft modes matched populations, scores, fronts, progress and JSON results exactly.

Those fixtures only guard against change. Independent checks cover the core geometry: `tests/model/inverse-kinematics-oracle.test.js` solves a 3-4-5 layout by hand and, for general poses on three layouts, rebuilds platform points and horn tips from the documented formulas, requires rod closure to 1e-9 mm and finds the servo angle by a numerical root search; `tests/model/rotation-convention.test.js` checks `R = Rz Ry Rx` against hand-written rotations and base-axis turns; `tests/simulator/projection-oracle.test.js` checks the camera against an independent pinhole model. The shared formulas live in `tests/fixtures/independent-geometry.js`, which imports nothing from `src/`.

Run `npm run smoke` for the complete default sample and export checks. After `npm ci`, `npm run test:browser`, `npm run test:browser:geometry`, `npm run test:browser:diagnostics` and `npm run test:browser:render` exercise worker behavior, active simulator transfers/controls and drawn positions in a real browser. Manual desktop checks include camera orbit, explicit platform drag, animations and WebGL2 rendering. The local development server has HTTP checks for entry points, content types and path restrictions.

The search is a heuristic. The UI exposes a seed, and exports include effective settings for replay. Exported geometry and metrics are a starting point for independent engineering analysis, not a manufacturing specification.
