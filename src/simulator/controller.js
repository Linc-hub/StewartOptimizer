import { evaluatePose, ensureLayout } from '../model/pose.js';
import { OVERLAY_DEFAULTS, OVERLAY_NAMES, parseOverlays } from './scene.js';
import { parseReachability, REACHABILITY_DEFAULTS, sweepReachability } from './reachability.js';
import { parseLoadModel, poseLoads } from './loads.js';
import { capabilityExtents } from './capability.js';
import { staticState } from '../model/cycle.js';
import { eulerRatesToAngular } from '../model/trajectory.js';

export const POSE_AXES = Object.freeze(['x', 'y', 'z', 'rx', 'ry', 'rz']);
export const HOME_POSE = Object.freeze({ x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 });
export const ANIMATION_PATTERNS = Object.freeze(['none', 'wobble', 'pingpong', 'rotate', 'tilt', 'helical']);

const copy = value => value == null ? value : structuredClone(value);
// The reachability sweep yields once per animation frame in a browser, else to a timer.
const nextFrame = callback => typeof globalThis.requestAnimationFrame === 'function'
  ? globalThis.requestAnimationFrame(callback) : setTimeout(callback, 0);

// Options come from the UI, saved JSON and headless callers; a string or array
// would otherwise be spread into the option map character by character.
function plainOptions(value) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw new TypeError('Simulator options must be an object.');
  return value;
}

export function normalizePose(pose = {}) {
  if (!pose || typeof pose !== 'object' || Array.isArray(pose)) {
    throw new TypeError('A requested pose must be an object with six finite coordinates.');
  }
  const result = Object.fromEntries(POSE_AXES.map(axis => [axis, pose[axis] ?? 0]));
  if (!Object.values(result).every(Number.isFinite)) {
    throw new RangeError('A requested pose must have six finite coordinates.');
  }
  return result;
}

// Requirement workspace ranges about home, one `{ min, max }` per pose axis in
// the caller's units (the controller holds mm and radians). Axes may be left
// out, `step` and other keys are dropped, and a map with no axes is null.
export function normalizeWorkspaceRanges(value, field = 'workspaceRanges') {
  if (value == null) return null;
  if (typeof value !== 'object' || Array.isArray(value)) throw new TypeError(`${field} must be an object.`);
  const result = {};
  for (const axis of POSE_AXES) {
    const range = value[axis];
    if (range == null) continue;
    if (typeof range !== 'object' || Array.isArray(range)) throw new TypeError(`${field}.${axis} must be an object.`);
    const { min, max } = range;
    if (typeof min !== 'number' || typeof max !== 'number' || !Number.isFinite(min) || !Number.isFinite(max) || min > max) {
      throw new RangeError(`${field}.${axis} must have finite min and max with min <= max.`);
    }
    result[axis] = { min, max };
  }
  return Object.keys(result).length ? result : null;
}

// Each pattern is a sum of terms `[axis, amplitude, wave, multiple, offset]`,
// one per moving axis: offset + amplitude * wave(multiple * phase) with
// phase = 2π f t, so its derivatives are analytic (mm and radians).
function patternTerms(pattern, { amplitudeMm = 12, rotationRad = Math.PI / 18, apexMm = 20 }) {
  if (!ANIMATION_PATTERNS.includes(pattern)) throw new RangeError(`Unknown animation pattern: ${pattern}`);
  switch (pattern) {
    case 'wobble': return [['rx', rotationRad, 'sin', 1], ['ry', rotationRad, 'cos', 1]];
    case 'pingpong': return [['z', apexMm, 'sin', 1]];
    case 'rotate': return [['rz', rotationRad, 'sin', 1]];
    case 'tilt': return [['rx', rotationRad, 'sin', 1], ['ry', rotationRad, 'sin', 0.5]];
    case 'helical': return [['x', amplitudeMm, 'cos', 1, -amplitudeMm], ['y', amplitudeMm, 'sin', 1],
      ['z', apexMm, 'sin', 0.5], ['rz', rotationRad, 'sin', 1]];
    default: return [];
  }
}

export function animationPose(pattern, seconds, settings = {}) {
  const phase = 2 * Math.PI * (settings.frequencyHz ?? 0.25) * seconds;
  const pose = { ...HOME_POSE };
  for (const [axis, amplitude, wave, multiple, offset = 0] of patternTerms(pattern, settings)) {
    pose[axis] = offset + amplitude * Math[wave](multiple * phase);
  }
  return pose;
}

// The pattern's pose with its velocity and acceleration, in the cycle model's
// trajectory state form (m, m/s, m/s², rad/s, rad/s², base frame). `rate` is
// pattern seconds per real second (the playback speed), so derivatives are
// taken against real time: velocity scales with it and acceleration with its square.
export function animationState(pattern, seconds, settings = {}, rate = 1) {
  const angular = 2 * Math.PI * (settings.frequencyHz ?? 0.25);
  const phase = angular * seconds;
  const first = { ...HOME_POSE }, second = { ...HOME_POSE };
  for (const [axis, amplitude, wave, multiple] of patternTerms(pattern, settings)) {
    const w = multiple * angular * rate;
    const sine = Math.sin(multiple * phase), cosine = Math.cos(multiple * phase);
    first[axis] = amplitude * w * (wave === 'sin' ? cosine : -sine);
    second[axis] = -amplitude * w * w * (wave === 'sin' ? sine : cosine);
  }
  const pose = animationPose(pattern, seconds, settings);
  const meters = (values, axes) => axes.map(axis => values[axis] / 1000);
  const { omega, alpha } = eulerRatesToAngular([pose.rx, pose.ry, pose.rz], [first.rx, first.ry, first.rz],
    [second.rx, second.ry, second.rz]);
  return { pose, velocity: meters(first, ['x', 'y', 'z']), acceleration: meters(second, ['x', 'y', 'z']),
    omega, angularAcceleration: alpha };
}

// `schedule(callback)` runs a callback later; the reachability sweep awaits it
// between chunks. `evaluateReachability` replaces `evaluatePose` for that sweep
// in tests only; the cloud otherwise uses the shared evaluator like everything else.
export function createSimulatorController({ onChange, schedule = nextFrame, evaluateReachability } = {}) {
  const listeners = new Set(onChange ? [onChange] : []);
  let layout = null;
  let source = null;
  let options = {};
  let requested = { ...HOME_POSE };
  let accepted = null;
  let assessment = null;
  let acceptedAssessment = null;
  let requestSource = 'load';
  let animation = { pattern: 'none', playing: false, seconds: 0, speed: 1, pauseReason: null };
  let markers = true;
  let tracesEnabled = false;
  let overlays = { ...OVERLAY_DEFAULTS };
  let workspaceRanges = null;
  let trace = [];
  let reachability = { ...REACHABILITY_DEFAULTS };
  // The published cloud is frozen and shared by every snapshot instead of
  // copied, since it can hold thousands of points and notifies every chunk.
  let reachabilityCloud = null;
  let sweep = null;
  // The parsed load model (null for none), the motion state the accepted pose
  // was reached with, and the loads published for it.
  let loadModel = null;
  let acceptedMotion = null;
  let loads = null;
  // The capability box result (frozen and shared like the cloud) and the
  // orientation it was found at; layout and option changes clear it.
  let capability = null;

  function getState() {
    return { ...copy({ layout, source, options, requested, accepted, assessment, acceptedAssessment,
      requestSource, rejected: Boolean(assessment && !assessment.reachable), animation,
      markers, tracesEnabled, overlays, workspaceRanges, trace, reachability,
      loadModel: loadModel?.input ?? null, loads }), reachabilityCloud, capability };
  }

  // Finds the per-axis reach at the requested orientation when the capability
  // box is on. It is about 70 evaluations, so it runs synchronously, and only
  // again once the orientation differs from the last result's.
  function refreshCapability() {
    if (!layout || !overlays.capabilityBox) {
      capability = null;
      return;
    }
    const orientation = { rx: requested.rx, ry: requested.ry, rz: requested.rz };
    const previous = capability?.orientation;
    if (previous && previous.rx === orientation.rx && previous.ry === orientation.ry && previous.rz === orientation.rz) return;
    const result = capabilityExtents({ layout, options, orientation, evaluate: evaluateReachability });
    capability = Object.freeze({ ...result, orientation: Object.freeze(orientation) });
  }

  // Loads at the accepted pose: dynamic while the animation drives it, static
  // (zero velocity and acceleration) for every other request. Null without a
  // load model or an accepted pose.
  function refreshLoads() {
    loads = loadModel && acceptedAssessment && acceptedMotion
      ? poseLoads(layout, acceptedAssessment, acceptedMotion.state, loadModel, acceptedMotion.kind) : null;
  }

  function stopSweep() {
    sweep?.abort.abort();
    sweep = null;
  }

  // Starts a sweep when the cloud is on and its inputs (requested orientation,
  // settings) differ from the running one; layout, options and range changes
  // call stopSweep first so they always restart it. Off or without a layout,
  // there is no cloud. Points from an older sweep are never mixed in.
  function refreshReachability() {
    if (!layout || !overlays.reachabilityCloud) {
      stopSweep();
      reachabilityCloud = null;
      return;
    }
    const orientation = Object.freeze({ rx: requested.rx, ry: requested.ry, rz: requested.rz });
    const key = JSON.stringify([orientation, reachability]);
    if (sweep?.key === key) return;
    stopSweep();
    const abort = new AbortController();
    sweep = { key, abort };
    const total = reachability.sampleCount;
    const points = [];
    const publish = (error = null) => {
      reachabilityCloud = Object.freeze({ points: Object.freeze(points.slice()), progress: points.length / total,
        total, orientation, error });
    };
    publish();
    sweepReachability({ layout, options, workspaceRanges, orientation, settings: { ...reachability },
      signal: abort.signal, yieldControl: () => new Promise(resolve => schedule(resolve)),
      evaluate: evaluateReachability,
      onChunk(chunk) {
        for (const point of chunk) points.push(Object.freeze({ at: Object.freeze(point.at), reachable: point.reachable }));
        publish();
        notify();
      } }).catch(error => {
      if (abort.signal.aborted) return;
      publish(error.message);
      notify();
    });
  }

  function notify() {
    const snapshot = getState();
    for (const listener of listeners) listener(snapshot);
    return snapshot;
  }

  function requestPose(pose, { source: origin = 'manual' } = {}) {
    return request(pose, origin, null);
  }

  // `motion` is the animation state for an animation frame, null otherwise.
  function request(pose, origin, motion) {
    if (!layout) throw new Error('Load a layout before requesting a pose.');
    if (animation.playing && origin !== 'animation') {
      animation.playing = false;
      animation.pauseReason = 'Manual pose request';
    }
    requested = normalizePose(pose);
    requestSource = origin;
    assessment = evaluatePose(layout, requested, { ...options, recordLegData: true });
    if (assessment.reachable) {
      accepted = { ...requested };
      acceptedAssessment = assessment;
      acceptedMotion = motion ? { kind: 'animation', state: motion } : { kind: 'static', state: staticState(accepted) };
      if (tracesEnabled) {
        const center = assessment.translation;
        trace.push(center.slice());
        if (trace.length > 300) trace.shift();
      }
    } else if (origin === 'animation') {
      animation.playing = false;
      animation.pauseReason = assessment.violations.map(violation => violation.type).join(', ') || 'Pose rejected';
    }
    refreshLoads();
    refreshReachability();
    refreshCapability();
    return notify();
  }

  // `workspaceRanges` (mm and radians, or null for none) replaces the drawn
  // requirement ranges and `loadModel` (requirement keys and units, or null for
  // none) the payload and servo ratings; either left out is kept, so a geometry
  // edit that reloads the layout keeps the box and the loads.
  function loadLayout(nextLayout, { source: nextSource = { kind: 'import' },
    options: nextOptions = {}, workspaceRanges: nextRanges, loadModel: nextLoadModel } = {}) {
    ensureLayout(nextLayout);
    const nextLayoutCopy = copy(nextLayout);
    const nextOptionsCopy = copy(plainOptions(nextOptions));
    const nextRangesCopy = nextRanges === undefined ? workspaceRanges : normalizeWorkspaceRanges(nextRanges);
    const nextLoadModelCopy = nextLoadModel === undefined ? loadModel : parseLoadModel(nextLoadModel);
    // Validate the options against the home pose before touching any state, as
    // setOptions does, so an invalid load leaves the previous layout intact.
    evaluatePose(nextLayoutCopy, HOME_POSE, { ...nextOptionsCopy, recordLegData: true });
    layout = nextLayoutCopy;
    source = copy(nextSource);
    options = nextOptionsCopy;
    workspaceRanges = nextRangesCopy;
    loadModel = nextLoadModelCopy;
    requested = { ...HOME_POSE };
    accepted = null;
    assessment = null;
    acceptedAssessment = null;
    acceptedMotion = null;
    trace = [];
    animation = { ...animation, playing: false, seconds: 0, pauseReason: null };
    stopSweep();
    capability = null;
    return requestPose(HOME_POSE, { source: 'load' });
  }

  function clear() {
    layout = null;
    source = null;
    options = {};
    workspaceRanges = null;
    loadModel = null;
    requested = { ...HOME_POSE };
    accepted = null;
    assessment = null;
    acceptedAssessment = null;
    acceptedMotion = null;
    loads = null;
    trace = [];
    animation = { ...animation, playing: false, seconds: 0, pauseReason: null };
    refreshReachability();
    refreshCapability();
    return notify();
  }

  function setOptions(patch) {
    const nextOptions = { ...options, ...copy(plainOptions(patch)) };
    if (layout) evaluatePose(layout, requested, { ...nextOptions, recordLegData: true });
    options = nextOptions;
    if (!layout) return notify();
    // A settings change must recheck the accepted pose as well as the request.
    if (accepted) {
      const checked = evaluatePose(layout, accepted, { ...options, recordLegData: true });
      if (!checked.reachable) {
        accepted = null;
        acceptedAssessment = null;
        acceptedMotion = null;
      } else acceptedAssessment = checked;
    }
    stopSweep();
    capability = null;
    return requestPose(requested, { source: 'settings' });
  }

  function setAnimation(pattern, playing = false, settings = {}) {
    if (!ANIMATION_PATTERNS.includes(pattern)) throw new RangeError(`Unknown animation pattern: ${pattern}`);
    const speed = settings.speed ?? animation.speed;
    if (!Number.isFinite(speed) || speed <= 0) throw new RangeError('Animation speed must be positive.');
    animation = { ...animation, pattern, playing: Boolean(playing && pattern !== 'none'), speed,
      seconds: settings.reset ? 0 : animation.seconds, pauseReason: null };
    return notify();
  }

  function tick(deltaSeconds, settings = {}) {
    if (!animation.playing || !layout) return getState();
    if (!Number.isFinite(deltaSeconds) || deltaSeconds < 0) throw new RangeError('Animation step must be nonnegative.');
    animation.seconds += Math.min(deltaSeconds, 0.1) * animation.speed;
    const motion = animationState(animation.pattern, animation.seconds, settings, animation.speed);
    return request(motion.pose, 'animation', motion);
  }

  return {
    loadLayout, clear, requestPose, setOptions, setAnimation, tick, getState,
    subscribe(listener) { listeners.add(listener); listener(getState()); return () => listeners.delete(listener); },
    getReferenceLayout() { return copy(layout); },
    setMarkers(enabled) { markers = Boolean(enabled); return notify(); },
    setTraces(enabled) { tracesEnabled = Boolean(enabled); return notify(); },
    // Patches the overlay toggles; names missing from the patch keep their state.
    setOverlays(patch) {
      const known = parseOverlays(patch);
      const unknown = Object.keys(patch).filter(name => !OVERLAY_NAMES.includes(name));
      if (unknown.length) throw new RangeError(`Unknown overlay: ${unknown.join(', ')}.`);
      overlays = { ...overlays, ...known };
      refreshReachability();
      refreshCapability();
      return notify();
    },
    // Patches the reachability cloud: `enabled` is the reachabilityCloud overlay
    // toggle; sampleCount, mode and sliceZ (mm about home) are its settings.
    // Everything is checked before anything changes.
    setReachabilityCloud(patch) {
      const settings = parseReachability(patch);
      if (patch.enabled !== undefined && typeof patch.enabled !== 'boolean') {
        throw new TypeError('reachability.enabled must be true or false.');
      }
      reachability = { ...reachability, ...settings };
      if (patch.enabled !== undefined) overlays = { ...overlays, reachabilityCloud: patch.enabled };
      refreshReachability();
      return notify();
    },
    // Replaces the drawn requirement ranges (mm and radians); null removes them.
    setWorkspaceRanges(ranges) {
      workspaceRanges = normalizeWorkspaceRanges(ranges);
      stopSweep();
      refreshReachability();
      return notify();
    },
    clearTrace() { trace = []; return notify(); },
    dispose() { stopSweep(); listeners.clear(); },
  };
}
