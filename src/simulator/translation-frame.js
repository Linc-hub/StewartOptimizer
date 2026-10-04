import { rotateVector, rotationMatrixFromEuler } from '../math.js';

// The frame the simulator's translation inputs (X/Y/Z fields and sliders,
// arrow keys, Move platform drags, the gamepad sticks) are expressed in.
// `base` is the fixed world frame at the base plate; `platform` is the moving
// platform frame of the requested pose, so X moves along the platform's own red
// axis. Poses are always stored in the base frame; only the inputs convert.
export const TRANSLATION_FRAMES = Object.freeze(['base', 'platform']);
export const DEFAULT_TRANSLATION_FRAME = 'base';

const transpose = m => m[0].map((_, column) => m.map(row => row[column]));
const rotation = pose => rotationMatrixFromEuler(pose.rx ?? 0, pose.ry ?? 0, pose.rz ?? 0);

// An unknown or empty frame (a select without a value) reads as the base frame.
export function resolveTranslationFrame(frame) {
  return TRANSLATION_FRAMES.includes(frame) ? frame : DEFAULT_TRANSLATION_FRAME;
}

// The pose's base-frame translation expressed in `frame`: R^T t for the platform frame.
export function translationInFrame(pose, frame) {
  const t = [pose.x ?? 0, pose.y ?? 0, pose.z ?? 0];
  return resolveTranslationFrame(frame) === 'platform' ? rotateVector(transpose(rotation(pose)), t) : t;
}

// The base-frame translation whose coordinates in `frame`, at the orientation
// of `pose`, are `local`: R local for the platform frame.
export function translationFromFrame(local, pose, frame) {
  return resolveTranslationFrame(frame) === 'platform' ? rotateVector(rotation(pose), local) : local.slice();
}

// `pose` moved by `delta` (mm) along the axes of `frame` at its own orientation.
export function stepTranslation(pose, delta, frame) {
  const [dx, dy, dz] = translationFromFrame(delta, pose, frame);
  return { ...pose, x: (pose.x ?? 0) + dx, y: (pose.y ?? 0) + dy, z: (pose.z ?? 0) + dz };
}
