import { rotateVector, rotationMatrixFromEuler } from '../math.js';

// The frame the simulator's incremental pose inputs act in: the X/Y/Z fields
// and sliders, arrow keys, Move platform drags, the rotation keys and the
// gamepad. `base` is the fixed world frame at the base plate; `platform` is the
// moving platform frame of the requested pose, so X moves along, and W/S tilt
// about, the platform's own red axis. Poses are always stored in the base frame
// as Euler angles; only the inputs convert. The Rx/Ry/Rz fields and sliders
// always hold those stored angles, since an absolute value needs a fixed frame.
export const INPUT_FRAMES = Object.freeze(['base', 'platform']);
export const DEFAULT_INPUT_FRAME = 'base';

const transpose = m => m[0].map((_, column) => m.map(row => row[column]));
const multiply = (a, b) => a.map(row => b[0].map((_, column) => row.reduce((sum, value, k) => sum + value * b[k][column], 0)));
const rotation = pose => rotationMatrixFromEuler(pose.rx ?? 0, pose.ry ?? 0, pose.rz ?? 0);

// An unknown or empty frame (a select without a value) reads as the base frame.
export function resolveInputFrame(frame) {
  return INPUT_FRAMES.includes(frame) ? frame : DEFAULT_INPUT_FRAME;
}

// The pose's base-frame translation expressed in `frame`: R^T t for the platform frame.
export function translationInFrame(pose, frame) {
  const t = [pose.x ?? 0, pose.y ?? 0, pose.z ?? 0];
  return resolveInputFrame(frame) === 'platform' ? rotateVector(transpose(rotation(pose)), t) : t;
}

// The base-frame translation whose coordinates in `frame`, at the orientation
// of `pose`, are `local`: R local for the platform frame.
export function translationFromFrame(local, pose, frame) {
  return resolveInputFrame(frame) === 'platform' ? rotateVector(rotation(pose), local) : local.slice();
}

// `pose` moved by `delta` (mm) along the axes of `frame` at its own orientation.
export function stepTranslation(pose, delta, frame) {
  const [dx, dy, dz] = translationFromFrame(delta, pose, frame);
  return { ...pose, x: (pose.x ?? 0) + dx, y: (pose.y ?? 0) + dy, z: (pose.z ?? 0) + dz };
}

// The rotation matrix of a rotation vector (axis times angle, radians), by Rodrigues.
export function rotationFromVector([x, y, z]) {
  const angle = Math.hypot(x, y, z);
  if (angle === 0) return [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const [kx, ky, kz] = [x / angle, y / angle, z / angle];
  const c = Math.cos(angle), s = Math.sin(angle), v = 1 - c;
  return [
    [c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s],
    [ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s],
    [kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v],
  ];
}

// The angle equal to `angle` modulo 2π that is nearest `reference`, so a
// stepped angle does not jump by a turn when it crosses ±180°.
const nearest = (angle, reference) => angle + 2 * Math.PI * Math.round((reference - angle) / (2 * Math.PI));

// The Euler angles of `rotationMatrixFromEuler` (R = Rz Ry Rx) for matrix `m`,
// each unwrapped toward `previous`. At gimbal lock (|ry| = 90°) only rx + rz or
// rx − rz is defined; rx is held at its previous value.
export function eulerFromRotation(m, previous = { rx: 0, ry: 0, rz: 0 }) {
  const ry = -Math.asin(Math.max(-1, Math.min(1, m[2][0])));
  if (Math.abs(Math.cos(ry)) < 1e-9) {
    const rx = previous.rx ?? 0;
    // ry = +90°: R[0][1] = sin(rx − rz), R[1][1] = cos(rx − rz).
    // ry = −90°: R[0][1] = −sin(rx + rz), R[1][1] = cos(rx + rz).
    const rz = m[2][0] < 0 ? rx - Math.atan2(m[0][1], m[1][1]) : Math.atan2(-m[0][1], m[1][1]) - rx;
    return { rx, ry, rz: nearest(rz, previous.rz ?? 0) };
  }
  return {
    rx: nearest(Math.atan2(m[2][1], m[2][2]), previous.rx ?? 0),
    ry,
    rz: nearest(Math.atan2(m[1][0], m[0][0]), previous.rz ?? 0),
  };
}

// `pose` turned by the rotation vector `delta` (radians) about the axes of
// `frame` through the platform origin, whose position is kept: R' = Rδ R about
// the fixed base axes, R' = R Rδ about the platform's own axes.
export function stepRotation(pose, delta, frame) {
  if (!delta.some(Boolean)) return { ...pose };
  const current = rotation(pose);
  const turn = rotationFromVector(delta);
  const next = resolveInputFrame(frame) === 'platform' ? multiply(current, turn) : multiply(turn, current);
  return { ...pose, ...eulerFromRotation(next, { rx: pose.rx ?? 0, ry: pose.ry ?? 0, rz: pose.rz ?? 0 }) };
}
