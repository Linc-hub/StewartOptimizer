# Cycle demand model

Model identity: `cycle-newton-euler-v1` (`cycle.modelVersion`), independent of the layout/joint `model_version`.

## Trajectories

A cycle is a periodic trajectory about the home pose. Two inputs are accepted:

- **Legacy single-axis cycle** (`cycle_mm`, `frequency_hz`, `cycle_axis`): peak-to-peak translation along x, y or z. It is exactly the sinusoid with amplitude `cycle_mm / 2`, zero phase. Results report `trajectorySource: "legacy-cycle"` and keep `axis`.
- **Supplied trajectory** (`payload.trajectory`): a common-frequency multi-axis sinusoid. Each component names an axis (`x`, `y`, `z` with `amplitude_mm`; `rx`, `ry`, `rz` with `amplitude_deg`) and an optional `phase_deg`. Component value is `A sin(2 pi f t + phase)`. It replaces the legacy fields; supplying both is an error. Results report `trajectorySource: "supplied"` and `axis: null`. A direct `evaluateLayout` or `evaluateCycle` caller that passes `trajectory` without `trajectorySource` gets the supplied trajectory, not a stationary legacy cycle.

`trajectoryId` is a canonical text identity, for example `sinusoid-v1:f=2Hz;z:15mm@0deg`, recorded in the result and in `run.effective_settings.cycleModel`. Zero frequency or all-zero amplitudes evaluate one stationary home pose (`sampling.status: "stationary"`).

The Simulate tab's **Requirements cycle** animation pattern plays this trajectory with `trajectoryState`, the same poses the evaluation samples; see [SIMULATOR.md](./SIMULATOR.md#controller-boundary).

## Sampling and convergence

`run.effective_settings.cycleSampling` selects the time schedule over one period [0, T):

- `{ "strategy": "uniform", "samples": 64 }` evaluates equally spaced times from t = 0. This is the original schedule; it reports `status: "fixed"` and makes no convergence claim. Replays of runs saved without `cycleSampling` use it.
- `{ "strategy": "adaptive", "initialSamples": 32, "maxSamples": 256, "tolerance": 0.005, "inconclusivePolicy": "enforced" }` is the default for new runs. It evaluates `initialSamples` equally spaced times and then the midpoint of every initial interval, so the first 64 samples are exactly the legacy grid. For each bisected interval, the unresolved variation of each monitored quantity is estimated as one quarter of the midpoint's departure from linear interpolation. The monitored quantities are signed servo torque and speed (scaled by the current peak across servos), reciprocal conditioning (scaled by the worst sampled value, floored at 1e-3), lower and upper socket angles (scaled by their limits), and servo angles (scaled by the travel span). The interval with the largest normalized estimate is bisected next; ties go to the earliest interval. Refinement stops as `converged` when every estimate is at most `tolerance`, or as `budget-limited` when `maxSamples` is reached first.

The result's `sampling` object records the policy, `status`, `evaluatedSamples`, `converged` (true, false for budget-limited, null when not assessed) and `maxUnresolved`. A failed cycle reports `violated` for an observed modeled constraint (joint, travel, conditioning) and `unavailable` for a singular or nonfinite calculation. A budget-limited cycle is inconclusive: with the default enforced policy it adds the `cycle_convergence` failure category; with `inconclusivePolicy: "advisory"` it stays visible without disqualifying the candidate. The UI offers the default, a finer preset (1,024 samples, 0.1%) and the fixed 64-phase schedule. Other values (`initialSamples` from 2 to 512, `maxSamples` up to 1,024, any tolerance in (0, 1], `inconclusivePolicy: "advisory"`, or other uniform sample counts) are available only through the headless `cycleSampling` option.

Sampled convergence is a numerical estimate, not a mathematical guarantee of continuous feasibility: a feature narrower than the inspected intervals can still be missed. No interval-bound method is applied. Nonuniform samples carry trapezoidal periodic time weights (`periodicSampleWeights`) for time-averaged metrics.

### Frames and units

The base frame has +Z up; gravity is `[0, 0, -9.81]` m/s^2. The moving origin is the platform-frame origin (the frame of `platform_anchors`), located at `[x, y, z + home_height]`. Orientation uses the pose evaluator's Euler convention `R = Rz(rz) Ry(ry) Rx(rx)`. Internally, positions are converted from mm to m and angles to radians; derivatives are SI and expressed in base-frame components at the moving origin.

Euler-angle rates are not the physical angular velocity. With Euler rates `(a', b', c')` for `(rx, ry, rz)`, the base-frame angular velocity is `omega = a' Rz Ry e_x + b' Rz e_y + c' e_z`, and the angular acceleration is its analytic time derivative. Tests check both against finite differences of `R(t)`.

## Mass properties

`mass_kg` is the combined moving platform and payload mass. Optional payload fields define a rigid body:

| Field | Meaning |
| --- | --- |
| `center_of_mass_mm` | Center of mass in the platform frame, relative to the moving origin, mm |
| `inertia_kg_m2` | Inertia tensor about the center of mass in platform axes, kg m^2. A 3x3 matrix or `{ixx, iyy, izz, ixy, ixz, iyz}`; off-diagonal values are tensor entries (I_xy = -integral of x y dm) |
| `external_force_n` | Base-frame force applied to the moving body at the center of mass, N |
| `external_moment_nm` | Base-frame moment applied to the moving body, N m |

The tensor must be finite, symmetric, positive semidefinite, and satisfy the principal-moment triangle inequality; a nonzero tensor requires positive mass. With none of these fields, the result reports `massModel: "legacy-point"`: a point mass at the moving origin, which reproduces the earlier model exactly. Any supplied field selects `massModel: "rigid-body"`.

## Newton-Euler balance

With `c = R c_local`, `a_c = a + alpha x c + omega x (omega x c)` and `I_w = R I R^T`, the rods must supply the wrench about the moving origin

- `F = m (a_c - g) - F_ext`
- `M = I_w alpha + omega x (I_w omega) - M_ext + c x F`

At each sample, inverse kinematics gives horn angles and actual rod directions `u_i` (horn tip toward platform). The six equilibrium equations `sum f_i [u_i; r_i x u_i] = [F; M]`, with `r_i = R p_i` in meters, are solved for rod axial forces `f_i`. A positive rod force pushes the platform along `u_i`.

## Servo motion and torque

The physical rotary mapping is `G = D^-1 A`, with `A_i = [u_i^T, (r_i x u_i)^T]`, `D_ii = u_i . h'_i` and `h'_i = dh/dalpha` (m/rad). For the moving-origin twist `V = [v; omega]`, the signed servo rate is `alphaDot = G V`. Signed servo acceleration `alphaDDot = G VDot + GDot V` is evaluated from the second derivative of the rod-length constraint, `alphaDDot_i = (|rhoDot|^2 / L + u . qDDot + (u . h) alphaDot^2) / (u . h')`, using `h'' = -h`. Signed servo torque is `tau_i = f_i D_ii` in the +alpha direction, so `sum tau_i alphaDot_i = F . v + M . omega` (virtual power). `G` is unnormalized and SI; the dimensionless centroid-referenced Jacobian in [CONDITIONING_MODEL.md](./CONDITIONING_MODEL.md) remains a separate conditioning measure.

Results report the largest sampled absolute torque (`torqueNm`, N m), speed (`speedRadPerSec`, rad/s) and acceleration (`accelerationRadPerSec2`, rad/s^2), per-servo peaks, and `limiting` entries giving the servo, sample index, time and signed value of each peak. These sampled maxima need not bound the continuous trajectory.

At rest under gravity the servo torque is nearly independent of rod inclination when the horns are close to horizontal: each rod carries about `m g / (6 u_z)` and its transmission is about `h u_z` (h the horn length), so `tau ≈ m g h / 6` (the paired fixture gives 0.219 N m against `m g h / 6` = 0.204 N m). Rod angle, plate radii and home height change the gravity torque only through the small horn tilt at home, so on a Pareto front the torque objective mostly drives horn length toward its lower bound; the trajectory's inertial and external wrench is what separates geometries.

`servoExcursionRmsRad` reports, per servo, the time-weighted RMS excursion of the servo angle about its cycle mean (rad; 0 for a stationary cycle). The `fatigue` objective is the cycle frequency times the mean of these six values, a servo motion rate in rad/s. It is computed from the cycle alone, so the servo travel bounds and joint limits do not change it, a rotation-only trajectory gives a nonzero value, and an invalid cycle gives null. It is a relative proxy for wear, not a service-life or material fatigue model.

## Rod load sharing

`cycle.loadSharing` (`rod-load-sharing-v2`) is computed from the signed rod forces solved at every cycle sample, so it reflects the requested trajectory, mass properties and external wrench rather than gravity-only vertical sharing. Positive force is compression (the rod pushes the platform); tension is reported as a positive magnitude. At each sample the six rods deliver the required force F = Σ fᵢ uᵢ (uᵢ the unit rod direction). The ideal peak rod force is the one they would carry if all six shared F equally in the same sense along their directions, ideal = |F| / Σ|uᵢ · F̂|, and the sample's share ratio is ideal / max|fᵢ|. Because |F| ≤ Σ|fᵢ||uᵢ · F̂| ≤ max|f| Σ|uᵢ · F̂|, the ratio lies in [0, 1] and equals 1 only when every rod carries the same magnitude in the same sense along F; rods that oppose each other (tension fighting compression) lower it even when their magnitudes match, and a sample whose required force is zero but whose rods carry load (a pure moment) scores 0. `balanceScore` is the time-weighted mean share ratio over samples with load, `worstShareRatio`/`worstTime` the sample with the lowest ratio, and `meanCv` the time-weighted coefficient of variation of the rod-force magnitudes (std(abs(f)) / mean(abs(f)), population standard deviation), reported as a diagnostic of magnitude spread only. Samples whose peak absolute force is at most 1e-9 N count as `zeroLoadSamples`; a cycle with no loaded sample reports `status: "zero-load"` and null scores, never perfect balance. An invalid cycle reports `unavailable`. Per-rod peak compression and tension, and the largest of each with its rod and time, are retained so balanced but excessive forces stay visible. Statistics are streamed with fixed memory from the bounded cycle samples.

For a 2.5 kg platform at rest on the default `circular` topology (base 110 mm, platform 70 mm, plate turn 30°, horn 45 mm, rod 210 mm, home 190 mm) the gravity load is 24.5 N and the equal-share ideal is about 4.2 N per rod, yet every horn pair offset produces three rods in tension against three in compression: a 10° offset gives −19.1 N / +27.1 N (share 0.155), the 30° default −5.8 N / +13.7 N (0.309) and 50° −4.3 N / +12.2 N (0.351), while a fixture whose six rods each carry +4.58 N scores 1. `rod-load-sharing-v1` scored the same three layouts 0.85, 0.71 and 0.68 because it took the coefficient of variation over absolute forces, so it improved as the opposing magnitudes converged; the measure was replaced because a `max` objective on it pulled the Pareto front toward force-fighting geometry. Full runs saved with `run.effective_settings.cycleModel.loadSharingModel` of `rod-load-sharing-v1` (or without the field) were ranked with the old score and are not reproduced by replay.

Actuator utilization is reported separately (`actuator_utilization`): horn leverage and ratings change servo torque for the same rod loads.

The simulator's **Loads** overlay applies the same `dynamicsAtPose` to the pose on screen: statically for a manual pose, and with the animation pattern's analytic velocity and acceleration while it plays. It shades rods by these signed rod forces and draws each rated servo's output-shaft torque (including any `servo_actuator` model) against its peak `torque_nm` rating; at a static pose the forces sum to the payload weight and the torques match a stationary `computeCycleDemand` at the same pose. Only the peak torque rating is compared per frame. See [SIMULATOR.md](./SIMULATOR.md#scene-builders-and-overlays).

## Actuator demand and servo capacity

Each sample also yields an output-shaft actuator torque `tau_act = tau_load + J alphaDDot + b alphaDot + c sign(alphaDot)` when `servo_actuator` (or a per-servo `actuator`) is supplied: `J` is actuator rotor/gear inertia reflected to the output shaft (`J_motor N^2`), `b` viscous friction (N m s/rad) and `c` Coulomb friction (N m, zero at rest; static friction is unmodeled). The load model already accounts for the moving platform, so `J` must not repeat platform inertia; horn inertia about the servo axis is not in the load model and may be included in `J`. Without an actuator model, `cycle.actuator.model` is `ideal`, actuator torque equals load torque and the `cycle.model` text ends with "actuator inertia/friction omitted"; with one, `cycle.actuator.model` is `reduced` and the text names the reduced actuator model instead. `cycle.actuator` reports per-servo peak and time-weighted RMS actuator torque; RMS uses the trapezoidal periodic weights of the (possibly nonuniform) samples.

Capacity (`servo-capacity-v2`) compares, per servo:

- **Peak**: scalar peak torque against peak actuator torque, scalar speed against peak speed, and the torque-speed envelope at every sampled simultaneous signed operating point. The envelope interpolates linearly between strictly increasing supplied speeds and reports `outOfDomain` outside them instead of extrapolating. `symmetric` envelopes apply to both directions and to braking; `motoring-braking` envelopes use `braking_torque_nm` where torque opposes motion, and without it braking points are unmodeled (`unavailable`). Envelope headroom fractions divide by the curve's peak torque, since capacity may reach zero at the top speed.
- **Continuous**: RMS actuator torque over the cycle against `continuous_torque_nm`. RMS is a screening metric for sustained cyclic load, not a calibrated temperature prediction.
- **Duration-limited**: for each `{torque_nm, duration_s}`, the worst moving-window RMS torque over `duration_s` (windows starting at each sample, periodic extension) against `torque_nm`.

Peak, continuous and duration groups report separate statuses and worst headroom fractions; the overall status is the most severe (`unavailable`, `outOfDomain`, `above`, `at`, `below`). Only sampled operating points are checked. Missing curve, thermal or friction data is reported as unrated or unmodeled, never replaced by an assumed manufacturer rating.

## Assumptions and omissions

Rods and horns are ideal, rigid and massless; joints are ideal and frictionless. The moving body is one rigid body with the supplied mass properties. Omitted from the load model: rod and horn inertia, compliance, thermal and material-strength checks. Each cycle sample is still checked for link collisions between legs by the pose evaluator ([JOINT_MODEL.md](./JOINT_MODEL.md#link-collisions)); a collision makes the cycle invalid. Actuator inertia and friction are included only through the optional reduced actuator model below; motor electrical dynamics and control are not modeled. This is not a complete multibody model. The workspace stiffness/fatigue proxies are separate from this force balance. Near-singular designs can have very large demand; the solver's numerical singularity check is not an engineering margin.

Cycle evaluation always enforces the modeled mechanical limits, including both socket deflections in their moving mounting frames, the mandatory actuator numerical-singularity check, and any supplied engineering condition limit, even during soft workspace exploration. An invalid pose reports its failed sample, time and specific violations. An invalid pose or singular equilibrium/transmission yields `valid: false`, a reason, and null demand values; it must not be interpreted as zero demand. Work estimates reserve the policy's maximum cycle samples (`maxSamples`, or `samples` for a uniform schedule) and one home-pose evaluation per candidate; convergence or early failure can perform fewer actual checks, and progress counts the samples actually evaluated. Cancellation is checked before every sample, including during refinement.

Run `npm test` for adaptive-versus-dense agreement, a between-sample violation missed by the 64-phase grid, legacy reproduction, equilibrium, offset-gravity, pure-rotation and combined-motion Newton-Euler, finite-difference rate/acceleration and virtual-power checks. Hardware validation is still required before using these estimates for component selection.
