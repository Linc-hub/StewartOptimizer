import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { trajectoryFromRequirements, trajectoryState } from '../../src/model/trajectory.js';
import { createSimulatorController, cycleFromRequirements, cycleState, NO_CYCLE_MESSAGE, normalizeCycle }
  from '../../src/simulator/controller.js';
import { parseSimulatorSnapshot } from '../../src/simulator/snapshot.js';
import { createSimulatorView, describeCycleNote } from '../../src/simulator/view.js';
import { createFakeDocument } from './helpers.js';

const options = { ballJointLimitDeg: 180 };
const requirements = { cycle_mm: 10, frequency_hz: 0.5, cycle_axis: 'z' };
const multiAxis = { type: 'sinusoid', frequency_hz: 1,
  components: [{ axis: 'x', amplitude_mm: 4 }, { axis: 'rz', amplitude_deg: 3, phase_deg: 90 }] };

test('the cycle comes from requirements the way the optimizer reads it, and a stationary one is none', () => {
  assert.deepEqual(cycleFromRequirements(requirements), trajectoryFromRequirements(requirements).trajectory);
  assert.deepEqual(cycleFromRequirements({ trajectory: multiAxis }), normalizeCycle(multiAxis));
  assert.equal(cycleFromRequirements({ cycle_mm: 10, frequency_hz: 0 }), null);
  assert.equal(cycleFromRequirements({ cycle_mm: 0, frequency_hz: 2 }), null);
  assert.equal(cycleFromRequirements({ trajectory: { frequency_hz: 'fast' } }), null);
  assert.equal(cycleFromRequirements(null), null);
  assert.throws(() => normalizeCycle({ frequency_hz: 1, components: [{ axis: 'q' }] }, 'simulator.cycle'),
    /^RangeError: simulator\.cycle: trajectory\.components\[0\]\.axis/);
});

test('cycle states reproduce trajectoryState poses and scale derivatives with the playback rate', () => {
  const cycle = normalizeCycle(multiAxis);
  for (const t of [0, 0.13, 0.5, 1.7]) {
    const reference = trajectoryState(cycle, t);
    const state = cycleState(cycle, t, 2);
    assert.deepEqual(state.pose, reference.pose);
    state.velocity.forEach((value, k) => assert.ok(Math.abs(value - 2 * reference.velocity[k]) < 1e-12));
    state.acceleration.forEach((value, k) => assert.ok(Math.abs(value - 4 * reference.acceleration[k]) < 1e-12));
    state.omega.forEach((value, k) => assert.ok(Math.abs(value - 2 * reference.omega[k]) < 1e-12));
    state.angularAcceleration.forEach((value, k) => assert.ok(Math.abs(value - 4 * reference.angularAcceleration[k]) < 1e-12));
  }
});

test('the cycle pattern plays the requirements trajectory through requestPose', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options, cycle: cycleFromRequirements(requirements) });
  const cycle = controller.getState().cycle;
  controller.setAnimation('cycle', true, { reset: true });
  let seconds = 0;
  for (let frame = 0; frame < 12; frame++) {
    const state = controller.tick(0.05);
    seconds += 0.05;
    assert.equal(state.requestSource, 'animation');
    assert.equal(state.rejected, false);
    const expected = trajectoryState(cycle, seconds).pose;
    for (const axis of ['x', 'y', 'z', 'rx', 'ry', 'rz']) assert.ok(Math.abs(state.requested[axis] - expected[axis]) < 1e-9, axis);
  }
  assert.ok(Math.abs(controller.getState().requested.z - 5 * Math.sin(Math.PI * 0.6)) < 1e-9);
  // A geometry reload without a cycle keeps it; setCycle(null) pauses playback.
  controller.loadLayout(asymmetricJointFixture(), { options });
  assert.deepEqual(controller.getState().cycle, cycle);
  controller.setAnimation('cycle', true);
  controller.setCycle(null);
  assert.equal(controller.getState().animation.playing, false);
  assert.equal(controller.getState().animation.pauseReason, NO_CYCLE_MESSAGE);
});

test('without a cycle the pattern can be chosen but not played', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options });
  assert.equal(controller.getState().cycle, null);
  controller.setAnimation('cycle', false);
  assert.equal(controller.getState().animation.pattern, 'cycle');
  assert.throws(() => controller.setAnimation('cycle', true), { message: NO_CYCLE_MESSAGE });
  assert.equal(controller.getState().animation.playing, false);
  assert.equal(controller.loadLayout(asymmetricJointFixture(), { options, cycle: { frequency_hz: 0, components: [] } }).cycle, null);
});

test('simulator JSON stores the cycle, falls back to the run requirements and rejects a malformed one', () => {
  const layout = asymmetricJointFixture();
  const saved = parseSimulatorSnapshot({ cycle: multiAxis, animation: { pattern: 'cycle', speed: 1.5 } }, layout, options);
  assert.deepEqual(saved.cycle, normalizeCycle(multiAxis));
  assert.deepEqual(saved.animation, { pattern: 'cycle', speed: 1.5 });
  assert.deepEqual(parseSimulatorSnapshot({}, layout, options, null, { requirements }).cycle, cycleFromRequirements(requirements));
  assert.equal(parseSimulatorSnapshot({}, layout, options, null, { requirements: { cycle_mm: 'x' } }).cycle, null);
  assert.equal(parseSimulatorSnapshot(null, layout, options).cycle, null);
  assert.throws(() => parseSimulatorSnapshot({ cycle: { frequency_hz: -1, components: [] } }, layout, options),
    /simulator\.cycle: trajectory\.frequency_hz must be >= 0/);
});

test('the Motion group describes the cycle and reports why it cannot play', () => {
  assert.equal(describeCycleNote(normalizeCycle(multiAxis)),
    'Requirements cycle: X ±4 mm, Rz ±3° (phase 90°) about home at 1 Hz, the motion the optimizer scored.');
  const document = createFakeDocument();
  const controller = createSimulatorController();
  const renderer = { available: true, contextLost: false, render() {}, dispose() {} };
  createSimulatorView({ document, window: { addEventListener() {} }, controller, createRenderer: () => renderer });
  const input = id => document.getElementById(id);
  controller.loadLayout(asymmetricJointFixture(), { options });
  assert.match(input('simCycleNote').textContent, /^Requirements cycle: none loaded\./);
  input('simSpeed').value = '1';
  input('simPattern').value = 'cycle';
  input('simPattern').dispatch('change');
  input('simPlay').dispatch('click');
  assert.equal(input('simPoseStatus').textContent, NO_CYCLE_MESSAGE);
  controller.setCycle(cycleFromRequirements(requirements));
  assert.equal(input('simCycleNote').textContent,
    'Requirements cycle: Z ±5 mm about home at 0.5 Hz, the motion the optimizer scored.');
  input('simPlay').dispatch('click');
  assert.equal(controller.getState().animation.playing, true);
});
