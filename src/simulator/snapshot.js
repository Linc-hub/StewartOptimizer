import { degToRad, radToDeg } from '../math.js';
import { evaluatePose } from '../model/pose.js';
import { ANIMATION_PATTERNS, HOME_POSE, normalizePose, normalizeWorkspaceRanges } from './controller.js';
import { parseReachability } from './reachability.js';
import { loadModelFromSettings, parseLoadModel } from './loads.js';
import { parseOverlays } from './scene.js';
import { parseCamera } from './view.js';
import { INPUT_FRAMES } from './input-frame.js';

export const POINTER_MODES = Object.freeze(['orbit', 'platform']);
// Everything the simulator JSON `simulator.options` block may carry; other keys are dropped.
export const SIMULATOR_OPTION_KEYS = Object.freeze(['ballJointLimitDeg', 'lowerBallJointLimitDeg',
  'upperBallJointLimitDeg', 'ballJointClamp', 'conditionLimit', 'servoRangeRad', 'rodLengthTolerance',
  'linkClearanceMm']);

function plainObject(value, field) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${field} must be an object.`);
  return value;
}

function pose(value, field) {
  if (value == null) return null;
  try { return normalizePose(plainObject(value, field)); }
  catch (error) { throw new (error.constructor)(`${field}: ${error.message}`); }
}

const ROTATION_AXES = new Set(['rx', 'ry', 'rz']);
const convertRanges = (ranges, convert) => ranges && Object.fromEntries(Object.entries(ranges).map(([axis, { min, max }]) =>
  [axis, ROTATION_AXES.has(axis) ? { min: convert(min), max: convert(max) } : { min, max }]));

// Workspace ranges as user-facing JSON writes them (a run's
// `effective_settings.bounds`, `simulator.workspaceRanges`): mm, and degrees
// for rotations. Returns the controller's mm and radians, or null for none.
export function parseWorkspaceRanges(value, field = 'workspaceRanges') {
  return convertRanges(normalizeWorkspaceRanges(value, field), degToRad);
}

// The inverse, for simulator JSON. Degrees print with at most 12 significant
// digits so a 12° range reads 12 after the radian round trip.
export function workspaceRangesToJSON(ranges) {
  return convertRanges(ranges, value => Number(radToDeg(value).toPrecision(12))) ?? null;
}

// Validates the `simulator` block of simulator JSON (Load optimizer reference,
// browser-save restore) before any of it is applied, so a rejected file leaves
// the current layout, pose, camera and animation untouched. `fallbackOptions`
// are used, and checked, when the block carries no options. `fallbackRanges`
// (the source run's bounds, user units) stand in for missing workspace ranges;
// ranges that do not parse there only mean no box, never a rejected file.
// `fallbackSettings` (the source run's `effective_settings`) likewise supply the
// load model when the block carries none; one that does not parse means no loads.
export function parseSimulatorSnapshot(saved, layout, fallbackOptions, fallbackRanges = null, fallbackSettings = null) {
  const block = saved == null ? {} : plainObject(saved, 'simulator');
  let options;
  if (block.options == null) options = { ...fallbackOptions };
  else {
    plainObject(block.options, 'simulator.options');
    options = Object.fromEntries(SIMULATOR_OPTION_KEYS.filter(key => block.options[key] !== undefined)
      .map(key => [key, structuredClone(block.options[key])]));
    if ('ballJointClamp' in options && typeof options.ballJointClamp !== 'boolean') {
      throw new TypeError('simulator.options.ballJointClamp must be true or false.');
    }
  }
  try { evaluatePose(layout, HOME_POSE, { ...options, recordLegData: true }); }
  catch (error) { throw new (error.constructor)(`simulator.options: ${error.message}`); }

  const result = { options, requested: pose(block.requested, 'simulator.requested'),
    accepted: pose(block.accepted, 'simulator.accepted'), camera: null, animation: null,
    markers: null, tracesEnabled: null, overlays: null, reachability: null, pointerMode: null, inputFrame: null,
    workspaceRanges: null, loadModel: null };
  if (block.loadModel != null) {
    result.loadModel = parseLoadModel(block.loadModel, 'simulator.loadModel')?.input ?? null;
  } else {
    try { result.loadModel = parseLoadModel(loadModelFromSettings(fallbackSettings))?.input ?? null; }
    catch { result.loadModel = null; }
  }
  if (block.workspaceRanges != null) {
    result.workspaceRanges = parseWorkspaceRanges(block.workspaceRanges, 'simulator.workspaceRanges');
  } else {
    try { result.workspaceRanges = parseWorkspaceRanges(fallbackRanges); }
    catch { result.workspaceRanges = null; }
  }
  if (block.camera != null) result.camera = parseCamera(block.camera, 'simulator.camera');
  if (block.animation != null) {
    const animation = plainObject(block.animation, 'simulator.animation');
    // A saved idle or unknown pattern selects the playable default; speed must be usable.
    const pattern = animation.pattern !== 'none' && ANIMATION_PATTERNS.includes(animation.pattern)
      ? animation.pattern : 'wobble';
    const speed = animation.speed == null ? 1 : animation.speed;
    if (typeof speed !== 'number' || !Number.isFinite(speed) || speed <= 0) {
      throw new RangeError('simulator.animation.speed must be a positive finite number.');
    }
    result.animation = { pattern, speed };
  }
  for (const key of ['markers', 'tracesEnabled']) {
    if (block[key] === undefined) continue;
    if (typeof block[key] !== 'boolean') throw new TypeError(`simulator.${key} must be true or false.`);
    result[key] = block[key];
  }
  if (block.overlays != null) result.overlays = parseOverlays(block.overlays, 'simulator.overlays');
  // Cloud settings only; whether it is on is the reachabilityCloud overlay.
  if (block.reachability != null) result.reachability = parseReachability(block.reachability, 'simulator.reachability');
  // An empty mode (a select without a value) counts as not saved.
  if (block.pointerMode != null && block.pointerMode !== '') {
    if (!POINTER_MODES.includes(block.pointerMode)) {
      throw new RangeError(`simulator.pointerMode must be one of ${POINTER_MODES.join(', ')}.`);
    }
    result.pointerMode = block.pointerMode;
  }
  // Likewise an empty frame; a file without one keeps the current frame.
  if (block.inputFrame != null && block.inputFrame !== '') {
    if (!INPUT_FRAMES.includes(block.inputFrame)) {
      throw new RangeError(`simulator.inputFrame must be one of ${INPUT_FRAMES.join(', ')}.`);
    }
    result.inputFrame = block.inputFrame;
  }
  return result;
}
