import { rotateVector, vectorAdd, vectorCross, vectorNormalize, vectorScale, vectorSub } from '../math.js';
import { hornFrameAxes } from '../model/kinematics.js';
import { translationSingularSystem } from '../model/conditioning.js';
import { effectiveServoRange, socketNormalsInWorld } from '../model/pose.js';
import { violationLegs } from '../model/collision.js';
import { capabilityCoverage } from './capability.js';

// The scene is a list of builders, each `(state, layout, solved) => { lines, points }`.
// `solved` is the accepted assessment (it may be null); nothing here evaluates a
// pose. Builders run in list order and their output is concatenated, so the
// order below fixes the draw order. A builder with an `overlay` key is drawn
// only when that key is on in `state.overlays`; the rest are always drawn.

export const SCENE_COLORS = Object.freeze({
  base: [0.24, 0.64, 0.94], platform: [0.43, 0.88, 0.72],
  horn: [1, 0.69, 0.31], rod: [0.88, 0.89, 0.94],
  failure: [1, 0.26, 0.32], globalFailure: [1, 0.38, 0.8],
  servo: [1, 0.42, 0.39], trace: [0.72, 0.5, 1],
  x: [1, 0.38, 0.38], y: [0.39, 0.92, 0.47], z: [0.42, 0.62, 1],
  limitRange: [0.5, 0.56, 0.66], nearLimit: [1, 0.88, 0.2],
  grid: [0.16, 0.2, 0.27], workspace: [0.32, 0.7, 0.76], capability: [0.78, 0.8, 0.86],
  reachable: [0.36, 0.9, 0.5], unreachable: [0.5, 0.2, 0.25],
  wellConditioned: [0.42, 0.85, 1],
  compression: [0.3, 0.55, 1], tension: [0.95, 0.4, 0.2], torqueLow: [0.36, 0.9, 0.5],
});
const COLORS = SCENE_COLORS;
// The canvas clear colour; the ghost dims toward it instead of blending.
export const SCENE_BACKGROUND = Object.freeze([0.055, 0.075, 0.11]);
const GHOST_BRIGHTNESS = 0.35;
// A small overshoot puts the ghost almost on top of the held pose, where equal
// depths would flicker between the two as the camera moves. Ghost failure lines
// are drawn this far (mm) toward the camera and dimmed ghost lines this far away
// from it, so there the held pose shows with any failure on top.
export const GHOST_DEPTH_BIAS_MM = 10;
const dim = color => color.map((value, k) => SCENE_BACKGROUND[k] + (value - SCENE_BACKGROUND[k]) * GHOST_BRIGHTNESS);

// Toggleable overlays and whether each is drawn when a state or saved file
// does not say. New overlays default off unless their issue says otherwise.
export const OVERLAY_DEFAULTS = Object.freeze({ groundGrid: true, servoArcs: false, jointCones: false,
  workspaceBox: false, capabilityBox: false, reachabilityCloud: false, conditioningEllipsoid: false, loads: false, requestedGhost: true,
  platformAxes: true, worldAxes: true });
export const OVERLAY_NAMES = Object.freeze(Object.keys(OVERLAY_DEFAULTS));
// Limit overlays (servo travel, socket cones) tint a value this close to its
// limit as a warning before the evaluator rejects it: 5°, in radians.
export const NEAR_LIMIT_MARGIN_RAD = 5 * Math.PI / 180;
// Ground grid line spacing (mm) and its half-width as a multiple of the base
// radius, rounded up to whole cells. The grid lies on z = 0 with the base
// polygon, servo stubs and world axes, so its lines are pushed this far (mm)
// away from the camera to lose every depth tie with them.
export const GROUND_GRID_PITCH_MM = 25;
const GROUND_GRID_EXTENT = 1.5;
export const GROUND_DEPTH_BIAS_MM = 2;
// Reachability samples are drawn smaller than the 6 to 7 px markers.
export const REACHABILITY_POINT_SIZE = 3;
// The conditioning ellipsoid colour runs from wellConditioned at reciprocal
// condition 1 to nearLimit at this floor, on a log scale. With a condition
// limit set, the floor is 1 / limit instead, so full yellow means at the limit.
export const CONDITIONING_GRADE_FLOOR = 0.01;
// The load overlay redraws each rod this far (mm) toward the camera so its
// graded colour wins the depth tie with the plain rod beneath it. Each servo's
// torque gauge is a band of concentric arcs at these multiples of horn length,
// outside the travel arc and its angle marker; WebGL lines are one pixel wide.
export const LOAD_DEPTH_BIAS_MM = 2;
export const TORQUE_BAND_RADII = Object.freeze([1.3, 1.33, 1.36, 1.39, 1.42, 1.45]);
const ELLIPSOID_SEGMENTS = 32;
const SERVO_ARC_SEGMENTS = 24;
const JOINT_CONE_SEGMENTS = 24;
const JOINT_CONE_GENERATRICES = 4;

function polygon(lines, points, color) {
  points.forEach((point, i) => lines.push({ from: point, to: points[(i + 1) % points.length], color }));
}

const hasSolvedLegs = solved => solved?.platformPoints?.length === 6 && solved?.hornTips?.length === 6;
// Length (mm) of the platform axes, which the conditioning ellipsoid also uses
// for its largest semi-axis.
export const platformAxisLength = layout => Math.max(18, layout.hornLength * 0.35);

// The rejected assessment the ghost overlay draws, or null when there is no
// ghost: the request was accepted, or the overlay is off.
function shownGhost(state) {
  const requested = state.assessment;
  if (!requested || requested.reachable !== false || !requested.translation || !requested.rotationMatrix) return null;
  return { ...OVERLAY_DEFAULTS, ...state.overlays }.requestedGhost ? requested : null;
}

// Color reports failures in the requested pose, which may have been rejected by
// the shared evaluator, while the geometry stays at the accepted pose. Each
// failure is coloured once, on the geometry that failed: when the ghost draws a
// failing leg (the solver reached its horn tip) or the whole-platform failure,
// that colour is on the ghost and the held leg keeps its normal colour; a leg
// the ghost cannot draw, or any failure while the ghost is off, colours the held leg.
function failureColor(state) {
  const violations = state.assessment?.violations ?? [];
  const ghost = shownGhost(state);
  const ghostLegs = ghost?.hornTips ?? [];
  const affectedLegs = new Set(violations.flatMap(violationLegs).filter(leg => !ghostLegs[leg]));
  const globalFailure = !ghost && violations.some(violation => !Number.isInteger(violation.leg));
  return index => affectedLegs.has(index) ? COLORS.failure : globalFailure ? COLORS.globalFailure : null;
}

// A square grid on the base plane for scale: lines every GROUND_GRID_PITCH_MM
// through the origin, out to GROUND_GRID_EXTENT times the base radius.
function groundGrid(state, layout) {
  const lines = [];
  const radius = Math.max(...layout.baseAnchors.map(([x, y]) => Math.hypot(x, y)));
  const cells = Math.max(1, Math.ceil(radius * GROUND_GRID_EXTENT / GROUND_GRID_PITCH_MM));
  const half = cells * GROUND_GRID_PITCH_MM;
  for (let k = -cells; k <= cells; k++) {
    const offset = k * GROUND_GRID_PITCH_MM;
    lines.push({ from: [offset, -half, 0], to: [offset, half, 0], color: COLORS.grid, depthBias: -GROUND_DEPTH_BIAS_MM });
    lines.push({ from: [-half, offset, 0], to: [half, offset, 0], color: COLORS.grid, depthBias: -GROUND_DEPTH_BIAS_MM });
  }
  return { lines, points: [] };
}

function base(state, layout) {
  const lines = [], points = [];
  const legColor = failureColor(state);
  polygon(lines, layout.baseAnchors, COLORS.base);
  for (let i = 0; i < 6; i++) {
    const anchor = layout.baseAnchors[i];
    const beta = layout.betaAngles[i];
    const direction = [Math.cos(beta), Math.sin(beta), 0];
    lines.push({ from: anchor, to: vectorAdd(anchor, vectorScale(direction, Math.max(12, layout.hornLength * 0.35))), color: legColor(i) ?? COLORS.servo });
    if (state.markers) points.push({ at: anchor, color: legColor(i) ?? COLORS.servo, size: 7 });
  }
  return { lines, points };
}

function platform(state, layout, solved) {
  const lines = [];
  if (hasSolvedLegs(solved)) polygon(lines, solved.platformPoints, COLORS.platform);
  return { lines, points: [] };
}

function legs(state, layout, solved) {
  const lines = [], points = [];
  if (!hasSolvedLegs(solved)) return { lines, points };
  const legColor = failureColor(state);
  for (let i = 0; i < 6; i++) {
    lines.push({ from: layout.baseAnchors[i], to: solved.hornTips[i], color: legColor(i) ?? COLORS.horn });
    lines.push({ from: solved.hornTips[i], to: solved.platformPoints[i], color: legColor(i) ?? COLORS.rod });
    if (state.markers) {
      points.push({ at: solved.hornTips[i], color: legColor(i) ?? COLORS.horn, size: 6 });
      points.push({ at: solved.platformPoints[i], color: legColor(i) ?? COLORS.platform, size: 7 });
    }
  }
  return { lines, points };
}

// Each servo's allowed travel drawn as an arc of horn-length radius about the
// base anchor, in the horn plane the evaluator uses, with ticks at both stops
// and a marker across the arc at the accepted horn angle. Failure colour when
// the requested pose breaks this servo's range, a warning tint when the
// accepted angle is within NEAR_LIMIT_MARGIN_RAD of a stop.
// A point in servo i's horn plane at horn angle alpha, `scale` horn lengths from its base anchor.
const hornPoint = (layout, i, alpha, scale) => vectorAdd(layout.baseAnchors[i],
  vectorScale(hornFrameAxes(layout.betaAngles[i], alpha)[0], layout.hornLength * scale));

function servoArcs(state, layout, solved) {
  const lines = [];
  const [min, max] = effectiveServoRange(layout, state.options ?? {});
  const violations = state.assessment?.violations ?? [];
  const at = (i, alpha, scale) => hornPoint(layout, i, alpha, scale);
  for (let i = 0; i < 6; i++) {
    const angle = solved?.servoAngles?.[i];
    const color = violations.some(violation => violation.type === 'servoLimit' && violation.leg === i) ? COLORS.failure
      : Number.isFinite(angle) && Math.min(angle - min, max - angle) < NEAR_LIMIT_MARGIN_RAD ? COLORS.nearLimit
        : COLORS.limitRange;
    for (let step = 0; step < SERVO_ARC_SEGMENTS; step++) {
      const alpha = min + (max - min) * step / SERVO_ARC_SEGMENTS;
      const next = step + 1 === SERVO_ARC_SEGMENTS ? max : min + (max - min) * (step + 1) / SERVO_ARC_SEGMENTS;
      lines.push({ from: at(i, alpha, 1), to: at(i, next, 1), color });
    }
    for (const stop of [min, max]) lines.push({ from: at(i, stop, 0.85), to: at(i, stop, 1.15), color });
    if (Number.isFinite(angle)) {
      lines.push({ from: at(i, angle, 0.8), to: at(i, angle, 1.25), color: color === COLORS.limitRange ? COLORS.horn : color });
    }
  }
  return { lines, points: [] };
}

// A cone of the given half-angle about a unit axis, as a ring at a fixed slant
// length from the apex plus a few generatrix lines. A fixed slant rather than a
// fixed height keeps wide limits (up to 180°) bounded.
function cone(lines, apex, axis, halfAngle, slant, color) {
  const u = vectorNormalize(vectorCross(axis, Math.abs(axis[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]));
  const v = vectorCross(axis, u);
  const at = turn => vectorAdd(apex, vectorScale(vectorAdd(vectorScale(axis, Math.cos(halfAngle)),
    vectorScale(vectorAdd(vectorScale(u, Math.cos(turn)), vectorScale(v, Math.sin(turn))), Math.sin(halfAngle))), slant));
  const ring = Array.from({ length: JOINT_CONE_SEGMENTS }, (_, k) => at(2 * Math.PI * k / JOINT_CONE_SEGMENTS));
  polygon(lines, ring, color);
  for (let k = 0; k < JOINT_CONE_GENERATRICES; k++) {
    lines.push({ from: apex, to: ring[k * JOINT_CONE_SEGMENTS / JOINT_CONE_GENERATRICES], color });
  }
}

// Each ball-joint socket's allowed cone at the accepted pose: apex at the rod
// end (horn tip for the lower socket, platform point for the upper), axis along
// the socket normal from the evaluator's construction, half-angle equal to that
// socket's effective limit. Failure colour when the requested pose breaks that
// socket's limit, a warning tint when the accepted angle is within
// NEAR_LIMIT_MARGIN_RAD of it. A limit picture, not a collision check.
function jointCones(state, layout, solved) {
  const lines = [];
  if (!hasSolvedLegs(solved) || !solved.mounting || !solved.jointLimits) return { lines, points: [] };
  const violations = state.assessment?.violations ?? [];
  const slant = Math.max(12, layout.hornLength * 0.35);
  for (let i = 0; i < 6; i++) {
    const mounts = { lower: solved.mounting.lower[i]?.direction, upper: solved.mounting.upper[i]?.direction };
    const alpha = solved.servoAngles?.[i];
    if (!mounts.lower || !mounts.upper || !Number.isFinite(alpha)) continue;
    const normals = socketNormalsInWorld(layout.betaAngles[i], alpha, solved.rotationMatrix, mounts);
    for (const [joint, apex] of [['lower', solved.hornTips[i]], ['upper', solved.platformPoints[i]]]) {
      const limit = solved.jointLimits[joint];
      const angle = solved.jointAngles?.[joint]?.[i];
      const failed = violations.some(violation => violation.type === 'ballJoint' && violation.leg === i
        && violation.joint === joint);
      const color = failed ? COLORS.failure
        : Number.isFinite(angle) && limit - angle < NEAR_LIMIT_MARGIN_RAD ? COLORS.nearLimit : COLORS.limitRange;
      cone(lines, apex, vectorNormalize(normals[joint]), limit, slant, color);
    }
  }
  return { lines, points: [] };
}

// The twelve edges of a box about home from per-axis { min, max } offsets (mm),
// four along each axis, one for each min/max pair of the other two.
// `color(axis)` colours the edges that run along that axis.
function boxEdges({ x, y, z }, layout, color) {
  const lines = [];
  const corner = ([i, j, k]) => [[x.min, x.max][i], [y.min, y.max][j], layout.homeHeight + [z.min, z.max][k]];
  for (const i of [0, 1]) {
    for (const j of [0, 1]) {
      for (const [axis, from, to] of [['x', [0, i, j], [1, i, j]], ['y', [i, 0, j], [i, 1, j]], ['z', [i, j, 0], [i, j, 1]]]) {
        lines.push({ from: corner(from), to: corner(to), color: color(axis) });
      }
    }
  }
  return lines;
}

// The twelve edges of the requirement x/y/z ranges as a box about home: the
// region the platform origin must reach, not the platform's extent. Rotation
// ranges are not drawn; without all three translation ranges there is no box.
function workspaceBox(state, layout) {
  const { x, y, z } = state.workspaceRanges ?? {};
  if (!x || !y || !z) return { lines: [], points: [] };
  return { lines: boxEdges({ x, y, z }, layout, () => COLORS.workspace), points: [] };
}

// The capability box: the layout's own reach from home along each X, Y and Z
// axis at the requested rotation, as the controller found it (`state.capability`;
// nothing is evaluated here). Edges along an axis are green when that reach
// covers the requirement range, yellow when it falls short, and light grey
// without a requirement range. Its corners are not checked poses.
function capabilityBox(state, layout) {
  const extents = state.capability?.extents;
  if (!extents) return { lines: [], points: [] };
  const coverage = capabilityCoverage(extents, state.workspaceRanges);
  const color = axis => coverage[axis] === null ? COLORS.capability : coverage[axis] ? COLORS.reachable : COLORS.nearLimit;
  return { lines: boxEdges(extents, layout, color), points: [] };
}

// One point per evaluated reachability sample at its platform-origin position:
// reachable samples green, unreachable ones a dim red so the reachable region
// stands out. The controller sweeps and publishes them; this only draws them.
function reachabilityCloud(state) {
  const points = (state.reachabilityCloud?.points ?? []).map(point => ({ at: point.at,
    color: point.reachable ? COLORS.reachable : COLORS.unreachable, size: REACHABILITY_POINT_SIZE }));
  return { lines: [], points };
}

// Colour of the conditioning ellipsoid: the whole-platform failure colour when
// the requested pose fails a conditioning check, otherwise graded from
// wellConditioned to nearLimit by the accepted pose's reciprocal condition
// number on a log scale (see CONDITIONING_GRADE_FLOOR).
export function conditioningColor(state, solved) {
  const violations = state.assessment?.violations ?? [];
  if (violations.some(violation => violation.type === 'conditionLimit' || violation.type === 'numericalSingularity')) {
    return COLORS.globalFailure;
  }
  const limit = state.options?.conditionLimit;
  const floor = Number.isFinite(limit) && limit > 1 ? 1 / limit : CONDITIONING_GRADE_FLOOR;
  const reciprocal = solved?.conditioning?.reciprocal;
  const t = Number.isFinite(reciprocal) && reciprocal > 0
    ? Math.min(1, Math.max(0, Math.log(reciprocal) / Math.log(floor))) : 1;
  return COLORS.wellConditioned.map((value, k) => value + (COLORS.nearLimit[k] - value) * t);
}

// The translation manipulability ellipsoid at the accepted pose, centred on the
// platform origin: its principal axes are the right singular vectors of the
// Jacobian's translation block, each half-length proportional to its singular
// value, with the largest equal to the platform axis length. A short axis is a
// direction the servos barely drive or hold, so the ellipsoid flattens as the
// pose nears a translational singularity. Drawn as the three axes plus the three
// great circles through each pair; the axes sit a little behind the platform axes
// they overlap. The colour follows the full six-axis condition number, which
// also covers rotation.
function conditioningEllipsoid(state, layout, solved) {
  const lines = [];
  const system = hasSolvedLegs(solved) ? translationSingularSystem(solved.conditioning?.jacobianRows) : null;
  if (!system || !(system[0].value > 0)) return { lines, points: [] };
  const center = solved.translation;
  const color = conditioningColor(state, solved);
  const semi = system.map(({ value, vector }) => vectorScale(vector, platformAxisLength(layout) * value / system[0].value));
  for (const axis of semi) {
    lines.push({ from: vectorSub(center, axis), to: vectorAdd(center, axis), color, depthBias: -GROUND_DEPTH_BIAS_MM });
  }
  for (const [a, b] of [[0, 1], [1, 2], [2, 0]]) {
    const ring = Array.from({ length: ELLIPSOID_SEGMENTS }, (_, k) => {
      const turn = 2 * Math.PI * k / ELLIPSOID_SEGMENTS;
      return vectorAdd(center, vectorAdd(vectorScale(semi[a], Math.cos(turn)), vectorScale(semi[b], Math.sin(turn))));
    });
    polygon(lines, ring, color);
  }
  return { lines, points: [] };
}

const mix = (from, to, t) => from.map((value, k) => value + (to[k] - value) * t);

// Rod colour by solved force: the plain rod colour at zero, graded toward
// compression blue (positive, the rod pushes) or tension red (negative), fully
// reached at `referenceN`.
export function rodForceColor(forceN, referenceN) {
  const t = referenceN > 0 ? Math.min(1, Math.abs(forceN) / referenceN) : 0;
  return mix(COLORS.rod, forceN >= 0 ? COLORS.compression : COLORS.tension, t);
}

// Torque gauge colour: torqueLow at zero graded to nearLimit at the peak
// torque rating, and the failure colour above it.
export function torqueUtilizationColor(utilization) {
  if (utilization > 1) return COLORS.failure;
  return mix(COLORS.torqueLow, COLORS.nearLimit, Math.max(0, utilization));
}

// Rod forces and servo torques at the accepted pose, as the controller solved
// them with the cycle model (it publishes `state.loads`; nothing is solved here).
// Each rod is redrawn graded by its force, except a rod the held pose already
// failure-colours. Each rated servo gets a torque gauge: a band from mid-travel
// toward the max stop for positive torque or the min stop for negative,
// reaching the stop at the peak torque rating and capped there above it.
function loads(state, layout, solved) {
  const lines = [];
  const result = state.loads;
  if (!result?.valid || !hasSolvedLegs(solved)) return { lines, points: [] };
  const legColor = failureColor(state);
  const reference = result.staticForceN > 0 ? result.staticForceN : Math.max(...result.rodForceN.map(Math.abs));
  for (let i = 0; i < 6; i++) {
    if (legColor(i)) continue;
    lines.push({ from: solved.hornTips[i], to: solved.platformPoints[i],
      color: rodForceColor(result.rodForceN[i], reference), depthBias: LOAD_DEPTH_BIAS_MM });
  }
  const [min, max] = effectiveServoRange(layout, state.options ?? {});
  const middle = (min + max) / 2;
  for (let i = 0; i < 6; i++) {
    const utilization = result.utilization?.[i];
    if (!Number.isFinite(utilization) || utilization <= 0) continue;
    const sweep = Math.sign(result.servoTorqueNm[i]) * Math.min(utilization, 1) * (max - middle);
    const steps = Math.max(1, Math.ceil(SERVO_ARC_SEGMENTS * Math.abs(sweep) / (max - min)));
    const color = torqueUtilizationColor(utilization);
    for (const radius of TORQUE_BAND_RADII) {
      for (let step = 0; step < steps; step++) {
        lines.push({ from: hornPoint(layout, i, middle + sweep * step / steps, radius),
          to: hornPoint(layout, i, middle + sweep * (step + 1) / steps, radius), color });
      }
    }
  }
  return { lines, points: [] };
}

// The rejected request drawn faintly beside the accepted pose: the one layer
// that shows geometry the evaluator did not accept. It uses only what the
// rejected evaluation returned. The platform comes from its translation and
// rotation, which are always set; legs are drawn only where the solver reached
// a horn tip (it stops at the first structural failure). Everything is dimmed
// toward the background and has no markers, except legs named in a violation,
// which use the failure colour at full brightness, and the platform outline,
// which turns the whole-platform failure colour on a conditioning failure.
function requestedGhost(state, layout) {
  const lines = [];
  const requested = shownGhost({ ...state, overlays: { ...state.overlays, requestedGhost: true } });
  if (!requested) return { lines, points: [] };
  const violations = requested.violations ?? [];
  const failedLegs = new Set(violations.flatMap(violationLegs));
  const platformFailure = violations.some(violation => !Number.isInteger(violation.leg));
  const platformPoints = layout.platformAnchors.map(anchor =>
    vectorAdd(requested.translation, rotateVector(requested.rotationMatrix, anchor)));
  polygon(lines, platformPoints, platformFailure ? COLORS.globalFailure : dim(COLORS.platform));
  const hornTips = requested.hornTips ?? [];
  for (let i = 0; i < 6; i++) {
    if (!hornTips[i]) continue;
    const failed = failedLegs.has(i);
    lines.push({ from: layout.baseAnchors[i], to: hornTips[i], color: failed ? COLORS.failure : dim(COLORS.horn) });
    lines.push({ from: hornTips[i], to: platformPoints[i], color: failed ? COLORS.failure : dim(COLORS.rod) });
  }
  const axis = platformAxisLength(layout);
  const column = index => requested.rotationMatrix.map(row => row[index]);
  for (const [index, color] of [[0, COLORS.x], [1, COLORS.y], [2, COLORS.z]]) {
    lines.push({ from: requested.translation, to: vectorAdd(requested.translation, vectorScale(column(index), axis)), color: dim(color) });
  }
  for (const line of lines) {
    const failure = line.color === COLORS.failure || line.color === COLORS.globalFailure;
    line.depthBias = failure ? GHOST_DEPTH_BIAS_MM : -GHOST_DEPTH_BIAS_MM;
  }
  return { lines, points: [] };
}

function platformAxes(state, layout, solved) {
  const lines = [];
  if (!hasSolvedLegs(solved)) return { lines, points: [] };
  const center = solved.translation;
  const axis = platformAxisLength(layout);
  const column = index => solved.rotationMatrix.map(row => row[index]);
  for (const [index, color] of [[0, COLORS.x], [1, COLORS.y], [2, COLORS.z]]) {
    lines.push({ from: center, to: vectorAdd(center, vectorScale(column(index), axis)), color });
  }
  return { lines, points: [] };
}

function worldAxes() {
  const origin = [0, 0, 0];
  return { lines: [[[30, 0, 0], COLORS.x], [[0, 30, 0], COLORS.y], [[0, 0, 30], COLORS.z]]
    .map(([direction, color]) => ({ from: origin, to: direction, color })), points: [] };
}

function trace(state) {
  const positions = state.trace ?? [];
  const lines = [];
  for (let i = 1; i < positions.length; i++) lines.push({ from: positions[i - 1], to: positions[i], color: COLORS.trace });
  return { lines, points: [] };
}

export const SCENE_BUILDERS = Object.freeze([
  { name: 'groundGrid', overlay: 'groundGrid', build: groundGrid },
  { name: 'base', build: base },
  { name: 'platform', build: platform },
  { name: 'legs', build: legs },
  { name: 'servoArcs', overlay: 'servoArcs', build: servoArcs },
  { name: 'jointCones', overlay: 'jointCones', build: jointCones },
  { name: 'workspaceBox', overlay: 'workspaceBox', build: workspaceBox },
  { name: 'capabilityBox', overlay: 'capabilityBox', build: capabilityBox },
  { name: 'reachabilityCloud', overlay: 'reachabilityCloud', build: reachabilityCloud },
  { name: 'conditioningEllipsoid', overlay: 'conditioningEllipsoid', build: conditioningEllipsoid },
  { name: 'loads', overlay: 'loads', build: loads },
  { name: 'requestedGhost', overlay: 'requestedGhost', build: requestedGhost },
  { name: 'platformAxes', overlay: 'platformAxes', build: platformAxes },
  { name: 'worldAxes', overlay: 'worldAxes', build: worldAxes },
  { name: 'trace', build: trace },
]);

export function buildSceneGeometry(state, builders = SCENE_BUILDERS) {
  const lines = [], points = [];
  const { layout, acceptedAssessment: solved } = state;
  if (!layout) return { lines, points };
  const overlays = { ...OVERLAY_DEFAULTS, ...state.overlays };
  const scene = { ...state, overlays, markers: state.markers === undefined ? true : state.markers };
  for (const builder of builders) {
    if (builder.overlay && !overlays[builder.overlay]) continue;
    const part = builder.build(scene, layout, solved ?? null);
    lines.push(...part.lines);
    points.push(...part.points);
  }
  return { lines, points };
}

// Reads a toggle map from saved JSON: known overlay names only, each true or
// false. Unknown names are dropped so a file from a newer version still loads.
export function parseOverlays(value, field = 'overlays') {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${field} must be an object.`);
  const result = {};
  for (const name of OVERLAY_NAMES) {
    if (value[name] === undefined) continue;
    if (typeof value[name] !== 'boolean') throw new TypeError(`${field}.${name} must be true or false.`);
    result[name] = value[name];
  }
  return result;
}
