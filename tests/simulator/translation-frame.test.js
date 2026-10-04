import test from 'node:test';
import assert from 'node:assert/strict';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { axisCssColor, createSimulatorView } from '../../src/simulator/view.js';
import { SCENE_COLORS } from '../../src/simulator/scene.js';
import { resolveTranslationFrame, stepTranslation, TRANSLATION_FRAMES, translationFromFrame,
  translationInFrame } from '../../src/simulator/translation-frame.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createFakeDocument } from './helpers.js';

// Platform-frame translation inputs, checked against hand-written rotations
// rather than the rotation helper they use.
const deg = value => value * Math.PI / 180;
const close = (actual, expected, label, tolerance = 1e-9) => expected.forEach((value, k) =>
  assert.ok(Math.abs(actual[k] - value) < tolerance, `${label}: ${actual} vs ${expected}`));

test('the base frame passes translations through and unknown frames read as base', () => {
  assert.deepEqual(TRANSLATION_FRAMES, ['base', 'platform']);
  for (const frame of ['', null, undefined, 'world']) assert.equal(resolveTranslationFrame(frame), 'base');
  const pose = { x: 3, y: -4, z: 5, rx: 0.2, ry: -0.1, rz: 0.7 };
  assert.deepEqual(translationInFrame(pose, 'base'), [3, -4, 5]);
  assert.deepEqual(translationFromFrame([3, -4, 5], pose, 'base'), [3, -4, 5]);
  assert.deepEqual(stepTranslation(pose, [1, 2, 3], 'base'), { ...pose, x: 4, y: -2, z: 8 });
});

test('the platform frame follows the platform axes for single-axis turns', () => {
  const t = deg(30), c = Math.cos(t), s = Math.sin(t);
  // About Z: the platform X axis is (cos, sin, 0) and its Y axis is (-sin, cos, 0).
  close(translationFromFrame([10, 0, 0], { rz: t }, 'platform'), [10 * c, 10 * s, 0], 'Rz: local X');
  close(translationFromFrame([0, 10, 0], { rz: t }, 'platform'), [-10 * s, 10 * c, 0], 'Rz: local Y');
  // About X: the platform Z axis tilts toward -Y.
  close(translationFromFrame([0, 0, 10], { rx: t }, 'platform'), [0, -10 * s, 10 * c], 'Rx: local Z');
  // About Y: the platform X axis tilts toward -Z.
  close(translationFromFrame([10, 0, 0], { ry: t }, 'platform'), [10 * c, 0, -10 * s], 'Ry: local X');
  // Reading a base-frame position back in the platform frame is the inverse.
  close(translationInFrame({ x: 10 * c, y: 10 * s, z: 0, rz: t }, 'platform'), [10, 0, 0], 'Rz: read back');
});

test('platform-frame conversion round-trips for a general orientation', () => {
  const pose = { x: 0, y: 0, z: 0, rx: deg(12), ry: deg(-7), rz: deg(41) };
  const local = [5, -3, 8];
  const world = translationFromFrame(local, pose, 'platform');
  close(translationInFrame({ ...pose, x: world[0], y: world[1], z: world[2] }, 'platform'), local, 'round trip');
  // A rotation preserves length.
  assert.ok(Math.abs(Math.hypot(...world) - Math.hypot(...local)) < 1e-9);
  const stepped = stepTranslation({ ...pose, x: 1, y: 2, z: 3 }, local, 'platform');
  close([stepped.x, stepped.y, stepped.z], [1 + world[0], 2 + world[1], 3 + world[2]], 'step');
  assert.deepEqual([stepped.rx, stepped.ry, stepped.rz], [pose.rx, pose.ry, pose.rz], 'a step never rotates');
});

function mount({ window = { addEventListener() {} }, documentElement } = {}) {
  const document = createFakeDocument();
  if (documentElement) document.documentElement = documentElement;
  const controller = createSimulatorController();
  const renderer = { available: false, contextLost: false, render() {}, dispose() {} };
  const view = createSimulatorView({ document, window, controller, createRenderer: () => renderer });
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const input = id => document.getElementById(id);
  return { document, controller, view, input, requested: () => controller.getState().requested };
}

test('pose labels take the canvas axis colours through CSS variables', () => {
  const set = {};
  mount({ documentElement: { style: { setProperty: (name, value) => { set[name] = value; } } } });
  assert.deepEqual(set, { '--axis-x': axisCssColor(SCENE_COLORS.x), '--axis-y': axisCssColor(SCENE_COLORS.y),
    '--axis-z': axisCssColor(SCENE_COLORS.z) });
  // The stylesheet fallbacks repeat these values.
  assert.equal(axisCssColor(SCENE_COLORS.x), 'rgb(255, 97, 97)');
  assert.equal(axisCssColor(SCENE_COLORS.y), 'rgb(99, 235, 120)');
  assert.equal(axisCssColor(SCENE_COLORS.z), 'rgb(107, 158, 255)');
  // A document without a root element (the test harness) still mounts.
  assert.doesNotThrow(() => mount());
});

test('switching frames re-reads the fields without moving the platform', () => {
  const { controller, view, input, requested } = mount();
  const t = deg(20);
  controller.requestPose({ x: 10, rz: t });
  const before = { ...requested() };
  assert.equal(view.getTranslationFrame(), 'base');
  assert.equal(input('simXInput').value, '10');
  view.setTranslationFrame('platform');
  assert.equal(view.getTranslationFrame(), 'platform');
  assert.equal(input('simTranslationFrame').value, 'platform');
  assert.equal(input('simXInput').value, String(Number((10 * Math.cos(t)).toFixed(2))));
  assert.equal(input('simYInput').value, String(Number((-10 * Math.sin(t)).toFixed(2))));
  assert.deepEqual(requested(), before, 'a frame switch moved the platform');
  // The select's own change event does the same.
  input('simTranslationFrame').value = 'base';
  input('simTranslationFrame').dispatch('change');
  assert.equal(input('simXInput').value, '10');
  assert.deepEqual(requested(), before);
  view.setTranslationFrame('nonsense');
  assert.equal(view.getTranslationFrame(), 'base', 'an unknown frame falls back to base');
});

test('in the platform frame a translation field moves along the platform axis and a rotation edit keeps the origin', () => {
  const { controller, view, input, requested } = mount();
  const t = deg(20);
  controller.requestPose({ rz: t });
  view.setTranslationFrame('platform');
  input('simXInput').value = '5';
  input('simXInput').dispatch('change');
  close([requested().x, requested().y, requested().z], [5 * Math.cos(t), 5 * Math.sin(t), 0], 'local X field');
  assert.ok(Math.abs(requested().rz - t) < 1e-12);
  // The slider path converts the same way.
  input('simYSlider').value = '4';
  input('simYSlider').dispatch('input');
  close([requested().x, requested().y], [5 * Math.cos(t) - 4 * Math.sin(t), 5 * Math.sin(t) + 4 * Math.cos(t)], 'local Y slider');
  const origin = [requested().x, requested().y, requested().z];
  input('simRZInput').value = '35';
  input('simRZInput').dispatch('change');
  close([requested().x, requested().y, requested().z], origin, 'rotation edit kept the origin');
  assert.ok(Math.abs(requested().rz - deg(35)) < 1e-12);
});

test('keyboard, Move platform drags and the gamepad step along the platform axes', () => {
  const frames = [];
  const pad = { axes: [0, 0, 0, 0], buttons: [] };
  const window = { addEventListener() {}, requestAnimationFrame: fn => { frames.push(fn); return frames.length; },
    navigator: { getGamepads: () => [pad] } };
  const { document, controller, view, input, requested } = mount({ window });
  const t = deg(30), c = Math.cos(t), s = Math.sin(t);
  controller.requestPose({ rz: t });
  view.setTranslationFrame('platform');
  const press = key => document.dispatch('keydown', { key, target: { tagName: 'BODY' }, preventDefault() {} });
  press('ArrowRight');
  close([requested().x, requested().y, requested().z], [c, s, 0], 'ArrowRight');
  press('ArrowUp');
  close([requested().x, requested().y], [c - s, s + c], 'ArrowUp');
  controller.requestPose({ rz: t });
  input('simPointerMode').value = 'platform';
  const canvas = input('simCanvas');
  canvas.dispatch('pointerdown', { button: 0, clientX: 0, clientY: 0, pointerId: 1 });
  canvas.dispatch('pointermove', { clientX: 20, clientY: 0 });
  canvas.dispatch('pointerup', {});
  close([requested().x, requested().y], [20 * 0.35 * c, 20 * 0.35 * s], 'drag along local X');
  controller.requestPose({ rz: t });
  input('simGamepad').checked = true;
  pad.axes = [1, 0, 0, 0];
  frames.at(-1)(0);
  frames.at(-1)(100);
  close([requested().x, requested().y], [25 * 0.1 * c, 25 * 0.1 * s], 'left stick X along local X');
  assert.ok(Math.abs(requested().rz - t) < 1e-12, 'translation input never rotates');
});
