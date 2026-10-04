# Requirements and validation

Requirements may be flat or grouped into `payload`, `workspace`, `rotations`, and optional `constraints`. Nested payload/workspace/rotations must be objects containing their required fields. In the grouped format, constraint keys may also sit at the top level and are merged into `constraints`; a key present in both places is rejected. Both formats undergo the same validation. Errors identify the invalid field before evaluation begins. Numeric strings and null values are rejected, including for optional sub-fields such as `trajectory.type`, `phase_deg`, `quadrants`, `servo_actuator` terms, `stiffness_model` options and `workspace_payload_support.rating`/`policy`; only `per_servo_ratings` entries may be null. Omission of an optional field uses its default.

| Field | Domain / meaning | Default |
| --- | --- | --- |
| mass_kg | Finite number >= 0; combined moving platform and payload mass, kg | Required |
| cycle_mm | Finite number >= 0; peak-to-peak translation, mm, centred on home (it reaches `cycle_mm / 2` either side); it does not widen the workspace ranges | Required unless `trajectory` |
| frequency_hz | Finite number >= 0; zero means stationary | Required unless `trajectory` |
| cycle_axis | x, y or z, case-insensitive | Required unless `trajectory` |
| trajectory | Optional payload field replacing the three cycle fields: `{ type, frequency_hz, components: [{ axis, amplitude_mm or amplitude_deg, phase_deg }] }`; `type` is optional and must be `sinusoid`, each axis may appear once, and `phase_deg` defaults to 0 | Single-axis cycle |
| center_of_mass_mm | Optional payload field; [x, y, z] platform-frame offset from the moving origin, mm | [0, 0, 0] |
| inertia_kg_m2 | Optional payload field; admissible inertia tensor about the center of mass, kg m^2, as a 3x3 matrix or an `ixx`/`iyy`/`izz`/`ixy`/`ixz`/`iyz` object (omitted products default to zero; null is rejected) | Zero |
| external_force_n / external_moment_nm | Optional payload fields; base-frame external force at the center of mass (N) and moment (N m) | Zero |
| x_range_mm, y_range_mm, z_range_mm | Finite min/max with max >= min; offsets from home, mm | Required |
| rx_range_deg, ry_range_deg, rz_range_deg | Finite min/max with max >= min; roll/pitch/yaw, degrees | Required |
| ball_joint_max_deg | Finite number in [0, 180] | 45 |
| link_clearance_mm | Finite number >= 0; minimum distance between the centre lines of two legs' horns or rods, mm. A closer pose is a link collision; 0 turns the check off | 6 |
| rod_length_bounds_mm | Two positive finite lengths with max >= min | [160, 420] |
| horn_length_bounds_mm | Two positive finite lengths with max >= min | [30, 110] |
| home_height_bounds_mm | Two positive finite home heights, mm, with max >= min | [50, 450] |
| servo_travel_bounds_deg | Two finite angles with max >= min | [-120, 120] |
| servo_max_deg | Optional finite nonnegative symmetric travel limit; used only when explicit travel bounds are omitted | Omitted |
| servo_torque_rating_nm | Optional finite positive shared peak torque rating in N m | Omitted |
| servo_speed_rating_deg_s | Optional finite positive shared peak angular speed rating in deg/s | Omitted |
| per_servo_ratings | Optional six-entry array in servo order; each entry may set positive `torque_nm` and/or `speed_deg_s`, or be null | Omitted |
| servo_continuous_torque_rating_nm | Optional finite positive shared continuous (RMS) torque rating in N m | Omitted |
| servo_torque_speed_curve | Optional `{ speed_deg_s: [...], torque_nm: [...], quadrants, braking_torque_nm: [...] }`; at least two strictly increasing nonnegative speeds (deg/s), one nonnegative torque (N m) per speed with at least one positive, `quadrants` `symmetric` (default) or `motoring-braking`; `braking_torque_nm` is one torque per speed and is allowed only with `motoring-braking` | Omitted |
| servo_duration_ratings | Optional nonempty array of `{ torque_nm, duration_s }`, both positive | Omitted |
| servo_actuator | Optional reduced actuator model: `output_inertia_kg_m2`, `viscous_nm_s_per_rad`, `coulomb_nm`, each finite >= 0, referred to the output shaft | Unmodeled |
| servo_rating_policy | `enforced` or `advisory`; supplied ratings constrain feasibility by default | `enforced` |
| workspace_payload_support | Optional static holding check at every strictly feasible workspace pose: `{ rating: "continuous" (default) or "peak", policy: "enforced" (default) or "advisory" }` | Omitted (no check) |
| stiffness_model | Optional physical compliance inputs: servo torsional stiffness, one rod description, characteristic length, test wrenches, objective choice (physical stiffness replaces the proxy in the Full set unless `use_as_objective` is false); see [COMPLIANCE_MODEL.md](./COMPLIANCE_MODEL.md) | Omitted (physical stiffness unavailable) |

Trajectory, mass-property and external-wrench conventions are defined in [CYCLE_MODEL.md](./CYCLE_MODEL.md). Supplying `trajectory` together with any legacy cycle field is rejected rather than silently choosing one.

### Cycle and workspace are separate demands

The workspace ranges are what the sweep samples and what feasible coverage measures. The cycle (legacy `cycle_mm` or `trajectory`) is a separate motion about home, checked on its own path against every modeled limit (see [CYCLE_MODEL.md](./CYCLE_MODEL.md)). Neither widens the other: a 40 mm `cycle_mm` on Z swings ±20 mm about home whatever `z_range_mm` says, and a wider `z_range_mm` does not lengthen the cycle. When a cycle axis swings past its range (for example ±25 mm against `z_range_mm` [-20, 40]), the browser warns under the requirements box, because coverage then says nothing about those cycle poses. `describeCycle` and `cycleRangeWarnings` in `src/model/cycle-checks.js` produce the note and the warnings.

Ranges accept `[min, max]`, `{ "min": min, "max": max }`, or `{ "from": min, "to": max }`. Arrays must contain exactly two values. Equal bounds are allowed for a fixed length, stationary coordinate or locked servo. A zero payload removes the modeled external load; zero stroke or frequency evaluates a stationary cycle. Valid input need not describe a feasible design. Note that an imported [reference layout](./IMPORT.md) requires a servo range with max strictly greater than min.

The in-app **Requirements reference** panel in `index.html` repeats this table and these paragraphs; edit both together.

The parser initializes grid steps that produce three samples along each nonzero workspace/rotation range (min, midpoint, max). Zero-span ranges contain one grid sample. The default Halton strategy uses the bounds and ignores steps; the user may select 256, 1,024, or 4,096 samples, or select Cartesian grid. Explicit UI overrides, including home-height bounds, take precedence over JSON defaults; untouched controls follow edits to the JSON. Loading the sample resets range and height controls. UI steps must be positive; population and generations must be integers >= 4 and >= 1 respectively. Workload limits apply independently of physical input validation.

Per-servo entries may also set `continuous_torque_nm`, `torque_speed_curve`, `duration_ratings` and `actuator`. For ratings, a per-servo value overrides the shared value for that servo and metric; an omitted per-servo value inherits the shared value. The UI's rating fields use the JSON values as defaults. Edited UI values take precedence for the run; untouched fields refresh after JSON edits. Loading the sample resets rating controls. Clear a UI field to remove that rating. Speed ratings entered in deg/s are converted to rad/s for comparison with cycle demand. The UI edits shared peak torque, speed, continuous torque, policy and per-servo peak torque/speed; curves, duration ratings and actuator models come from the JSON and are kept when UI per-servo fields are edited. Ratings do not change the force-balance or servo-speed models; an actuator model adds its terms to the compared actuator torque.
