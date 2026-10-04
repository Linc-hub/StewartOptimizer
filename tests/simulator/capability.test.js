import test from 'node:test';
import assert from 'node:assert/strict';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { evaluatePose } from '../../src/model/pose.js';
import { CAPABILITY_AXES, CAPABILITY_LIMIT_MM, CAPABILITY_TOLERANCE_MM, capabilityCoverage, capabilityExtents }
  from '../../src/simulator/capability.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { parseSimulatorSnapshot } from '../../src/simulator/snapshot.js';
import { createSimulatorView } from '../../src/simulator/view.js';
import { createFakeDocument } from './helpers.js';

const settings = { ballJointLimitDeg: 180 };
const level = { rx: 0, ry: 0, rz: 0 };

// A box-shaped fake: reachable inside the given half-widths, counting calls.
function boxEvaluator(bounds) {
  const calls = { count: 0 };
  const evaluate = (layout, pose) => {
    calls.count++;
    const inside = CAPABILITY_AXES.every(axis => pose[axis] >= bounds[axis].min && pose[axis] <= bounds[axis].max)
      && Math.abs(pose.rx) < 1;
    return { reachable: inside, translation: [pose.x, pose.y, pose.z], violations: [] };
  };
  return { evaluate, calls };
}

test('capability extents bracket each half-axis boundary of the shared evaluator', () => {
  const layout = asymmetricJointFixture();
  const result = capabilityExtents({ layout, options: settings, orientation: level });
  assert.equal(result.homeReachable, true);
  assert.ok(result.evaluations < 120, `${result.evaluations} evaluations`);
  const reachable = (axis, offset) => evaluatePose(layout, { x: 0, y: 0, z: 0, ...level, [axis]: offset }, settings).reachable;
  for (const axis of CAPABILITY_AXES) {
    const { min, max } = result.extents[axis];
    assert.ok(min < 0 && max > 0, axis);
    assert.equal(reachable(axis, max), true, `${axis} max`);
    assert.equal(reachable(axis, max + CAPABILITY_TOLERANCE_MM + 1e-9), false, `${axis} past max`);
    assert.equal(reachable(axis, min), true, `${axis} min`);
    assert.equal(reachable(axis, min - CAPABILITY_TOLERANCE_MM - 1e-9), false, `${axis} past min`);
  }
});

test('capability extents follow a known boundary, flag the probe limit and fail at an unreachable home', () => {
  const { evaluate } = boxEvaluator({ x: { min: -40.3, max: 17.8 }, y: { min: -1000, max: 1000 }, z: { min: -3, max: 9.2 } });
  const result = capabilityExtents({ layout: null, orientation: level, evaluate });
  assert.ok(result.extents.x.max <= 17.8 && result.extents.x.max > 17.8 - CAPABILITY_TOLERANCE_MM);
  assert.ok(result.extents.x.min >= -40.3 && result.extents.x.min < -40.3 + CAPABILITY_TOLERANCE_MM);
  assert.ok(result.extents.z.min >= -3 && result.extents.z.min < -3 + CAPABILITY_TOLERANCE_MM);
  assert.deepEqual(result.extents.y, { min: -CAPABILITY_LIMIT_MM, max: CAPABILITY_LIMIT_MM });
  assert.deepEqual(result.limited.y, { min: true, max: true });
  assert.deepEqual(result.limited.x, { min: false, max: false });
  const tilted = capabilityExtents({ layout: null, orientation: { rx: 1.2, ry: 0, rz: 0 }, evaluate });
  assert.deepEqual(tilted, { homeReachable: false, extents: null, limited: null, evaluations: 1 });
});

test('coverage compares the reach on each axis with its requirement range', () => {
  const extents = { x: { min: -30, max: 30 }, y: { min: -10, max: 40 }, z: { min: -5, max: 5 } };
  assert.deepEqual(capabilityCoverage(extents, { x: { min: -30, max: 25 }, y: { min: -20, max: 20 } }),
    { x: true, y: false, z: null });
  assert.deepEqual(capabilityCoverage(extents, null), { x: null, y: null, z: null });
  assert.deepEqual(capabilityCoverage(null, { z: { min: -1, max: 1 } }), { x: null, y: null, z: false });
});

test('the controller finds capability only with the overlay on and again only for a new orientation or limits', () => {
  const { evaluate, calls } = boxEvaluator({ x: { min: -20, max: 20 }, y: { min: -20, max: 20 }, z: { min: -10, max: 10 } });
  const controller = createSimulatorController({ evaluateReachability: evaluate });
  controller.loadLayout(asymmetricJointFixture(), { options: settings });
  assert.equal(controller.getState().capability, null);
  assert.equal(controller.getState().overlays.capabilityBox, false);
  controller.setOverlays({ capabilityBox: true });
  const first = controller.getState().capability;
  assert.equal(first.homeReachable, true);
  assert.deepEqual(first.orientation, level);
  const count = calls.count;
  controller.requestPose({ x: 5, z: 3 });
  assert.equal(calls.count, count, 'a translation keeps the result');
  assert.equal(controller.getState().capability, first);
  controller.requestPose({ rz: 0.1 });
  assert.ok(calls.count > count, 'a new orientation probes again');
  assert.equal(controller.getState().capability.orientation.rz, 0.1);
  const afterRotation = calls.count;
  controller.setOptions({ ballJointLimitDeg: 170 });
  assert.ok(calls.count > afterRotation, 'new limits probe again');
  controller.setOverlays({ capabilityBox: false });
  assert.equal(controller.getState().capability, null);
  controller.setOverlays({ capabilityBox: true });
  controller.clear();
  assert.equal(controller.getState().capability, null);
});

test('saved simulator JSON keeps the capability box toggle', () => {
  const parsed = parseSimulatorSnapshot({ overlays: { capabilityBox: true } }, asymmetricJointFixture(), settings);
  assert.deepEqual(parsed.overlays, { capabilityBox: true });
  assert.throws(() => parseSimulatorSnapshot({ overlays: { capabilityBox: 'yes' } }, asymmetricJointFixture(), settings),
    /simulator\.overlays\.capabilityBox must be true or false/);
});

test('the view reports per-axis reach and names axes short of the requirement', () => {
  const document = createFakeDocument();
  const { evaluate } = boxEvaluator({ x: { min: -20, max: 20 }, y: { min: -20, max: 20 }, z: { min: -10, max: 10 } });
  const controller = createSimulatorController({ evaluateReachability: evaluate });
  const renderer = { available: true, contextLost: false, render() {}, dispose() {} };
  createSimulatorView({ document, window: { addEventListener() {} }, controller, createRenderer: () => renderer });
  const input = id => document.getElementById(id);
  assert.equal(input('simCapabilityStatus').textContent, 'Capability box off.');
  input('simOverlayCapabilityBox').checked = true;
  input('simOverlayCapabilityBox').dispatch('change');
  assert.equal(input('simCapabilityStatus').textContent, 'Load a layout to find its capability.');
  controller.loadLayout(asymmetricJointFixture(), { options: settings,
    workspaceRanges: { x: { min: -15, max: 15 }, y: { min: -15, max: 15 }, z: { min: -15, max: 15 } } });
  assert.match(input('simCapabilityStatus').textContent,
    /^Capability at Rx 0, Ry 0, Rz 0°: X -(19\.\d+|20) to (19\.\d+|20), Y -(19\.\d+|20) to (19\.\d+|20), Z -(9\.\d+|10) to (9\.\d+|10) mm along each axis from home\. Short of the requirement on Z \(needs -15 to 15\)\.$/);
  controller.setWorkspaceRanges({ x: { min: -15, max: 15 } });
  assert.match(input('simCapabilityStatus').textContent, /Covers the requirement ranges\.$/);
  controller.requestPose({ rx: 1.2 });
  assert.equal(input('simCapabilityStatus').textContent, 'Capability: home is not reachable at Rx 68.75, Ry 0, Rz 0°.');
});
