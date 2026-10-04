import test from 'node:test';
import assert from 'node:assert/strict';
import { parseSimulatorSnapshot, parseWorkspaceRanges, SIMULATOR_OPTION_KEYS, workspaceRangesToJSON } from '../../src/simulator/snapshot.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';

const layout = asymmetricJointFixture();
const fallback = { ballJointLimitDeg: 180, servoRangeRad: layout.servoRangeRad, rodLengthTolerance: 0.5 };
const parse = simulator => parseSimulatorSnapshot(simulator, layout, fallback);

test('a saved simulator block round-trips its known fields and drops unknown ones', () => {
  const saved = { options: { ballJointLimitDeg: 45, lowerBallJointLimitDeg: 40, upperBallJointLimitDeg: 50,
    ballJointClamp: false, conditionLimit: null, servoRangeRad: [-1, 1], rodLengthTolerance: 0.25, linkClearanceMm: 4, extra: 'dropped' },
  requested: { x: 1, y: 2, z: 3, rx: 0.1, ry: 0.2, rz: 0.3 }, accepted: { z: 3 },
  camera: { yaw: 1, pitch: 0.5, distance: 400, target: [0, 0, 90], junk: 'x' },
  animation: { pattern: 'tilt', speed: 2, playing: true }, markers: false, tracesEnabled: true, pointerMode: 'platform',
  translationFrame: 'platform', trace: 'ignored', source: { kind: 'import' } };
  const result = parse(saved);
  assert.deepEqual(Object.keys(result.options).sort(), SIMULATOR_OPTION_KEYS.filter(key => key !== 'extra').sort());
  assert.equal('extra' in result.options, false);
  assert.deepEqual(result.requested, saved.requested);
  assert.deepEqual(result.accepted, { x: 0, y: 0, z: 3, rx: 0, ry: 0, rz: 0 });
  assert.deepEqual(result.camera, { yaw: 1, pitch: 0.5, distance: 400, target: [0, 0, 90] });
  assert.deepEqual(result.animation, { pattern: 'tilt', speed: 2 });
  assert.equal(result.markers, false);
  assert.equal(result.tracesEnabled, true);
  assert.equal(result.pointerMode, 'platform');
  assert.equal(result.translationFrame, 'platform');
  saved.options.servoRangeRad[0] = -9;
  assert.equal(result.options.servoRangeRad[0], -1, 'options were stored by reference');
  const empty = parse(undefined);
  assert.deepEqual(empty.options, fallback);
  assert.deepEqual([empty.requested, empty.accepted, empty.camera, empty.animation, empty.markers,
    empty.tracesEnabled, empty.pointerMode, empty.translationFrame], [null, null, null, null, null, null, null, null]);
  assert.deepEqual(parse({ animation: { pattern: 'none' }, pointerMode: '' }).animation, { pattern: 'wobble', speed: 1 });
  assert.equal(parse({ translationFrame: '' }).translationFrame, null, 'an empty frame counts as not saved');
});

test('every simulator field is type-checked and the error names the field', () => {
  const cases = [
    ['x', /simulator must be an object/],
    [{ options: 'x' }, /simulator.options must be an object/],
    [{ options: { ballJointLimitDeg: null } }, /simulator.options: ballJointLimitDeg must be a finite angle/],
    [{ options: { linkClearanceMm: -1 } }, /simulator.options: linkClearanceMm must be a finite nonnegative length/],
    [{ options: { ballJointLimitDeg: '45' } }, /ballJointLimitDeg must be a finite angle/],
    [{ options: { ballJointLimitDeg: true } }, /ballJointLimitDeg must be a finite angle/],
    [{ options: { ballJointLimitDeg: [] } }, /ballJointLimitDeg must be a finite angle/],
    [{ options: { lowerBallJointLimitDeg: 181 } }, /lowerBallJointLimitDeg must be a finite angle/],
    [{ options: { upperBallJointLimitDeg: NaN } }, /upperBallJointLimitDeg must be a finite angle/],
    [{ options: { servoRangeRad: 'abc' } }, /servoRangeRad must contain two finite bounds/],
    [{ options: { servoRangeRad: null } }, /servoRangeRad must contain two finite bounds/],
    [{ options: { servoRangeRad: [1, 0] } }, /servoRangeRad must contain two finite bounds/],
    [{ options: { servoRangeRad: ['-1', '1'] } }, /servoRangeRad must contain two finite bounds/],
    [{ options: { rodLengthTolerance: '0.5' } }, /rodLengthTolerance must be a finite nonnegative/],
    [{ options: { conditionLimit: 'abc' } }, /conditionLimit must be a finite number/],
    [{ options: { ballJointClamp: 'yes' } }, /simulator.options.ballJointClamp must be true or false/],
    [{ requested: 'home' }, /simulator.requested must be an object/],
    [{ requested: { x: 'a' } }, /simulator.requested: A requested pose must have six finite coordinates/],
    [{ accepted: [1, 2, 3] }, /simulator.accepted must be an object/],
    [{ camera: 'abc' }, /simulator.camera must be an object/],
    [{ camera: { yaw: 'abc' } }, /simulator.camera.yaw must be a finite number/],
    [{ camera: { distance: -1 } }, /simulator.camera.distance must be a finite number from 10 to 2500/],
    [{ camera: { pitch: 2 } }, /simulator.camera.pitch must be a finite number from -1.4 to 1.4/],
    [{ camera: { target: [0, 0] } }, /simulator.camera.target must contain three finite coordinates/],
    [{ animation: 'wobble' }, /simulator.animation must be an object/],
    [{ animation: { speed: -1 } }, /simulator.animation.speed must be a positive finite number/],
    [{ animation: { speed: 0 } }, /simulator.animation.speed must be a positive/],
    [{ animation: { speed: '2' } }, /simulator.animation.speed must be a positive/],
    [{ markers: 'yes' }, /simulator.markers must be true or false/],
    [{ tracesEnabled: null }, /simulator.tracesEnabled must be true or false/],
    [{ pointerMode: 'fly' }, /simulator.pointerMode must be one of orbit, platform/],
    [{ translationFrame: 'world' }, /simulator.translationFrame must be one of base, platform/],
  ];
  for (const [simulator, expected] of cases) {
    assert.throws(() => parse(simulator), expected, JSON.stringify(simulator));
  }
});

test('saved overlay toggles keep known names, drop unknown ones and are type-checked', () => {
  assert.equal(parse({}).overlays, null);
  assert.equal(parse({ overlays: null }).overlays, null);
  assert.deepEqual(parse({ overlays: { worldAxes: false, ghost: true } }).overlays, { worldAxes: false });
  assert.deepEqual(parse({ overlays: { platformAxes: true, worldAxes: false } }).overlays,
    { platformAxes: true, worldAxes: false });
  assert.throws(() => parse({ overlays: 'all' }), /simulator.overlays must be an object/);
  assert.throws(() => parse({ overlays: [true] }), /simulator.overlays must be an object/);
  assert.throws(() => parse({ overlays: { platformAxes: 1 } }), /simulator.overlays.platformAxes must be true or false/);
});

test('saved workspace ranges read mm and degrees, fall back to the run bounds and write back unchanged', () => {
  const json = { x: { min: -40, max: 40 }, y: { min: -40, max: 40 }, z: { min: -20, max: 40 },
    rx: { min: -12, max: 12 }, ry: { min: -12, max: 12 }, rz: { min: -8, max: 8 } };
  const ranges = parse({ workspaceRanges: { ...json, x: { ...json.x, step: 40 } } }).workspaceRanges;
  assert.deepEqual(ranges.x, { min: -40, max: 40 });
  assert.deepEqual(ranges.z, { min: -20, max: 40 });
  assert.ok(Math.abs(ranges.rx.max - 12 * Math.PI / 180) < 1e-15 && Math.abs(ranges.rz.min + 8 * Math.PI / 180) < 1e-15);
  assert.deepEqual(workspaceRangesToJSON(ranges), json);
  assert.equal(workspaceRangesToJSON(null), null);
  assert.deepEqual(parseWorkspaceRanges(json), ranges);
  // Missing from the block: the source run's bounds, else none.
  const withRun = (simulator, bounds) => parseSimulatorSnapshot(simulator, layout, fallback, bounds).workspaceRanges;
  assert.equal(parse({}).workspaceRanges, null);
  assert.equal(parse(null).workspaceRanges, null);
  assert.deepEqual(withRun({}, json), ranges);
  assert.deepEqual(withRun({ workspaceRanges: null }, json), ranges);
  assert.deepEqual(withRun({ workspaceRanges: { x: json.x } }, json), { x: { min: -40, max: 40 } });
  // Bad run bounds only mean no box; a bad saved block rejects the file by field.
  assert.equal(withRun({}, { x: { min: 1, max: -1 } }), null);
  assert.equal(withRun({}, 'bounds'), null);
  assert.throws(() => parse({ workspaceRanges: [] }), /simulator.workspaceRanges must be an object/);
  assert.throws(() => parse({ workspaceRanges: { z: { min: 4 } } }), /simulator.workspaceRanges.z must have finite min and max/);
});

test('a saved load model is checked by name, and a run supplies one when the block has none', () => {
  assert.equal(parse({}).loadModel, null);
  assert.deepEqual(parse({ loadModel: { mass_kg: 2, servo_torque_rating_nm: 0.5, extra: 1 } }).loadModel,
    { mass_kg: 2, servo_torque_rating_nm: 0.5 });
  assert.throws(() => parse({ loadModel: 'heavy' }), /simulator.loadModel must be an object/);
  assert.throws(() => parse({ loadModel: { mass_kg: -1 } }), /simulator.loadModel: mass_kg/);
  const run = { requirements: { mass_kg: 3 }, servoRatings: { servo_torque_rating_nm: 1 } };
  const withRun = block => parseSimulatorSnapshot(block, layout, fallback, null, run);
  assert.deepEqual(withRun({}).loadModel, { mass_kg: 3, servo_torque_rating_nm: 1 });
  assert.deepEqual(withRun({ loadModel: { mass_kg: 1 } }).loadModel, { mass_kg: 1 });
  // A run whose load does not parse means no loads, never a rejected file.
  assert.equal(parseSimulatorSnapshot({}, layout, fallback, null, { requirements: { mass_kg: -3 } }).loadModel, null);
});
