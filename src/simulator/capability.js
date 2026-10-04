import { evaluatePose } from '../model/pose.js';

// The capability box: how far the platform origin reaches from home along each
// base X, Y and Z half-axis at one fixed orientation, found through the shared
// pose evaluator. Each half-axis is walked outward in doubling steps until a
// pose fails or CAPABILITY_LIMIT_MM is reached, then the last reachable and
// first failing offsets are bisected to CAPABILITY_TOLERANCE_MM. The result is
// the first boundary met along each axis line through home: reach beyond a gap
// is not reported, and the box corners are not checked, so the box is per-axis
// reach, not a guarantee that every point inside it is reachable.

export const CAPABILITY_AXES = Object.freeze(['x', 'y', 'z']);
// Furthest offset (mm) probed along each half-axis.
export const CAPABILITY_LIMIT_MM = 250;
// First outward step (mm); later steps double until a pose fails.
export const CAPABILITY_FIRST_STEP_MM = 2;
// Bisection stops once the reachable and failing offsets are this close (mm).
export const CAPABILITY_TOLERANCE_MM = 0.5;

// Reach (mm, positive) along one half-axis from home at `orientation`.
function halfAxisReach(reachable, axis, sign) {
  let good = 0;
  let step = CAPABILITY_FIRST_STEP_MM;
  let bad = null;
  while (good < CAPABILITY_LIMIT_MM) {
    const next = Math.min(good + step, CAPABILITY_LIMIT_MM);
    if (!reachable(axis, sign * next)) { bad = next; break; }
    good = next;
    step *= 2;
  }
  if (bad === null) return { reach: good, limited: true };
  while (bad - good > CAPABILITY_TOLERANCE_MM) {
    const middle = (good + bad) / 2;
    if (reachable(axis, sign * middle)) good = middle;
    else bad = middle;
  }
  return { reach: good, limited: false };
}

// Per-axis reach about home at `orientation` ({ rx, ry, rz } in radians):
// `{ homeReachable, extents: { x: { min, max }, ... }, limited: { x: { min, max }, ... }, evaluations }`.
// `extents` are mm offsets from home (min <= 0 <= max); `limited` flags a side
// that reached CAPABILITY_LIMIT_MM without failing. When the orientation itself
// fails at home, `homeReachable` is false and `extents` is null. `evaluate` is
// `evaluatePose` outside tests.
export function capabilityExtents({ layout, options = {}, orientation, evaluate = evaluatePose }) {
  let evaluations = 0;
  const pose = (axis, offset) => ({ x: 0, y: 0, z: 0, rx: orientation.rx, ry: orientation.ry, rz: orientation.rz,
    ...(axis ? { [axis]: offset } : {}) });
  const reachable = (axis, offset) => {
    evaluations++;
    return Boolean(evaluate(layout, pose(axis, offset), options).reachable);
  };
  if (!reachable(null, 0)) return { homeReachable: false, extents: null, limited: null, evaluations };
  const extents = {}, limited = {};
  for (const axis of CAPABILITY_AXES) {
    const low = halfAxisReach(reachable, axis, -1);
    const high = halfAxisReach(reachable, axis, 1);
    extents[axis] = { min: low.reach === 0 ? 0 : -low.reach, max: high.reach };
    limited[axis] = { min: low.limited, max: high.limited };
  }
  return { homeReachable: true, extents, limited, evaluations };
}

// Whether the reach on each axis covers the requirement range for that axis:
// `{ x: true | false | null, ... }`, null where there is no requirement range.
export function capabilityCoverage(extents, workspaceRanges) {
  return Object.fromEntries(CAPABILITY_AXES.map(axis => {
    const required = workspaceRanges?.[axis];
    if (!required || !extents) return [axis, required ? false : null];
    return [axis, extents[axis].min <= required.min && extents[axis].max >= required.max];
  }));
}
