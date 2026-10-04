import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { buildSceneGeometry, cameraFrame, createWebGLRenderer, NEAR_PLANE_MM, projectPoint,
  projectSegment } from '../../src/simulator/renderer.js';
import { GHOST_DEPTH_BIAS_MM, GROUND_DEPTH_BIAS_MM, GROUND_GRID_PITCH_MM, NEAR_LIMIT_MARGIN_RAD, OVERLAY_DEFAULTS,
  OVERLAY_NAMES, REACHABILITY_POINT_SIZE, SCENE_BACKGROUND, SCENE_BUILDERS, SCENE_COLORS } from '../../src/simulator/scene.js';
import { parseWorkspaceRanges } from '../../src/simulator/snapshot.js';
import { parseRequirements } from '../../src/model/requirements.js';
import { computeHornTip, hornLocalToWorld } from '../../src/model/kinematics.js';
import { rotateVector, vectorAdd } from '../../src/math.js';
import { resolveMounting } from '../../src/model/mounting.js';
import { mountSimulatorDiagnostics } from '../../src/simulator/diagnostics.js';
import { createSimulatorView } from '../../src/simulator/view.js';
import { createFakeDocument } from './helpers.js';
import { effectiveServoRange } from '../../src/model/pose.js';

// The sample requirements' ranges as the simulator holds them (mm and radians):
// X and Y ±40 mm, Z −20 to 40 mm about home.
const sampleRanges = () => parseWorkspaceRanges(parseRequirements(fs.readFileSync(
  new URL('../../examples/sample-requirements.json', import.meta.url), 'utf8')).workspace);

test('scene uses solved asymmetric anchor, horn, rod and platform frames', () => {
  const controller = createSimulatorController();
  const state = controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const scene = buildSceneGeometry(state);
  assert.ok(scene.lines.some(line => line.from === undefined) === false);
  for (let leg = 0; leg < 6; leg++) {
    const base = state.layout.baseAnchors[leg];
    const horn = state.acceptedAssessment.hornTips[leg];
    const point = state.acceptedAssessment.platformPoints[leg];
    assert.ok(scene.lines.some(line => line.from === base && line.to === horn));
    assert.ok(scene.lines.some(line => line.from === horn && line.to === point));
    assert.ok(scene.points.some(marker => marker.at === horn));
  }
  assert.equal(state.acceptedAssessment.translation[2], state.layout.homeHeight);
  assert.ok(projectPoint(state.layout.baseAnchors[0], { target: [0, 0, 100] }, 800, 500));
  const rejected = controller.loadLayout(asymmetricJointFixture(),
    { options: { ballJointLimitDeg: 180, conditionLimit: 1 } });
  assert.equal(rejected.accepted, null);
  // Base, servo directions and world axes; the grid, arcs and the ghost of the rejected home are switched off.
  assert.equal(buildSceneGeometry({ ...rejected, overlays: { ...rejected.overlays, groundGrid: false, servoArcs: false,
    requestedGhost: false } }).lines.length, 15);
});

test('unavailable WebGL2 leaves renderer inactive with actionable error', () => {
  const renderer = createWebGLRenderer({ getContext: () => null });
  assert.equal(renderer.available, false);
  assert.match(renderer.error, /WebGL2.*hardware acceleration.*optimization remains available/i);
  renderer.render({});
});

test('projected depth keeps near/far ordering for a zoomed-out camera', () => {
  const target = [0, 0, 100];
  const yaw = 0.7, pitch = 0.4;
  const towardEye = [Math.cos(pitch) * Math.cos(yaw), Math.cos(pitch) * Math.sin(yaw), Math.sin(pitch)];
  const offset = (sign, units) => target.map((value, i) => value + sign * units * towardEye[i]);
  for (const distance of [600, 1900, 2500]) {
    const camera = { target, yaw, pitch, distance };
    const [nearer, middle, farther] = [offset(1, 300), target, offset(-1, 300)]
      .map(point => projectPoint(point, camera, 800, 500)[2]);
    assert.ok(nearer < middle && middle < farther, `distance ${distance}: ${nearer} ${middle} ${farther}`);
    assert.ok(farther < 0.999 && nearer > -0.999, `distance ${distance} saturates the depth range`);
  }
  // The depth mapping is unchanged for the default camera distance.
  const home = projectPoint(target, { target, yaw, pitch, distance: 600 }, 800, 500)[2];
  assert.ok(Math.abs(home - (599 / 2000 * 2 - 1)) < 1e-12);
});

function fakeGL() {
  const calls = { createProgram: 0, drawArrays: 0, deleteProgram: 0, draws: [] };
  let lost = false;
  let size = null;
  const noop = () => {};
  return {
    calls, setLost(value) { lost = value; },
    VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, COMPILE_STATUS: 3, LINK_STATUS: 4, DEPTH_TEST: 5,
    ARRAY_BUFFER: 6, DYNAMIC_DRAW: 7, FLOAT: 8, LINES: 9, POINTS: 10, COLOR_BUFFER_BIT: 16, DEPTH_BUFFER_BIT: 32,
    createShader: () => ({}), shaderSource: noop, compileShader: noop, getShaderParameter: () => true,
    createProgram() { calls.createProgram++; return {}; }, attachShader: noop, linkProgram: noop,
    getProgramParameter: () => true, createBuffer: () => ({}), getAttribLocation: () => 0,
    getUniformLocation: () => ({}), enable: noop, clearColor: noop, viewport: noop, clear: noop,
    useProgram: noop, bindBuffer: noop, bufferData: noop, vertexAttribPointer: noop,
    enableVertexAttribArray: noop, uniform1f(location, value) { size = value; },
    drawArrays(primitive, first, count) { calls.drawArrays++; calls.draws.push({ primitive, count, size }); },
    deleteBuffer: noop, deleteProgram() { calls.deleteProgram++; }, isContextLost: () => lost,
  };
}

test('a lost context cancels the default, stops drawing, and restore rebuilds the program and redraws', () => {
  const gl = fakeGL();
  const handlers = {};
  const canvas = { width: 300, height: 200, getContext: () => gl,
    addEventListener(type, handler) { handlers[type] = handler; },
    removeEventListener(type) { delete handlers[type]; } };
  const changes = [];
  const renderer = createWebGLRenderer(canvas, { onContextChange: () => changes.push(renderer.contextLost) });
  const controller = createSimulatorController();
  const state = controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  renderer.render(state, {});
  assert.equal(gl.calls.createProgram, 1);
  assert.ok(gl.calls.drawArrays > 0);
  const drawn = gl.calls.drawArrays;
  const event = { defaultPrevented: false, preventDefault() { this.defaultPrevented = true; } };
  handlers.webglcontextlost(event);
  assert.equal(event.defaultPrevented, true, 'the browser is only allowed to restore a cancelled lost event');
  assert.equal(renderer.contextLost, true);
  gl.setLost(true);
  renderer.render(state, {});
  assert.equal(gl.calls.drawArrays, drawn, 'drew while the context was lost');
  gl.setLost(false);
  handlers.webglcontextrestored();
  assert.equal(renderer.contextLost, false);
  assert.equal(gl.calls.createProgram, 2, 'program was not rebuilt after restore');
  renderer.render(state, {});
  assert.ok(gl.calls.drawArrays > drawn);
  assert.deepEqual(changes, [true, false]);
  renderer.dispose();
  assert.deepEqual(Object.keys(handlers), []);
  assert.equal(gl.calls.deleteProgram, 1);
});

test('the reachability cloud draws one small point per sample, coloured by its reachable flag', () => {
  const controller = createSimulatorController();
  const state = controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const at = k => [k, -k, state.layout.homeHeight + k];
  const points = [...Array(7)].map((_, k) => ({ at: at(k), reachable: k % 3 !== 0 }));
  const reachable = points.filter(point => point.reachable).length;
  const cloudState = { ...state, overlays: { ...state.overlays, reachabilityCloud: true },
    reachabilityCloud: { points, progress: 1, total: 7, orientation: { rx: 0, ry: 0, rz: 0 }, error: null } };
  const cloud = SCENE_BUILDERS.find(builder => builder.name === 'reachabilityCloud').build(cloudState, state.layout, null);
  assert.equal(cloud.lines.length, 0);
  assert.equal(cloud.points.length, reachable + (points.length - reachable));
  assert.equal(cloud.points.filter(point => point.color === SCENE_COLORS.reachable).length, reachable);
  assert.equal(cloud.points.filter(point => point.color === SCENE_COLORS.unreachable).length, points.length - reachable);
  assert.deepEqual(cloud.points.map(point => point.at), points.map(point => point.at));
  const markers = buildSceneGeometry(state).points;
  assert.ok(cloud.points.every(point => point.size === REACHABILITY_POINT_SIZE));
  assert.ok(markers.every(marker => marker.size > REACHABILITY_POINT_SIZE), 'samples are smaller than markers');
  // Off, or with no published cloud, nothing is drawn.
  assert.deepEqual(buildSceneGeometry({ ...cloudState, overlays: state.overlays }).points, markers);
  assert.deepEqual(buildSceneGeometry({ ...cloudState, reachabilityCloud: null }).points, markers);

  // The renderer draws each point size in its own call at that size.
  const gl = fakeGL();
  const canvas = { width: 300, height: 200, getContext: () => gl, addEventListener() {}, removeEventListener() {} };
  createWebGLRenderer(canvas).render(cloudState, {});
  const pointDraws = gl.calls.draws.filter(draw => draw.primitive === gl.POINTS);
  const drawnAt = size => pointDraws.filter(draw => draw.size === size).reduce((sum, draw) => sum + draw.count, 0);
  assert.equal(drawnAt(REACHABILITY_POINT_SIZE), points.length);
  assert.equal(pointDraws.reduce((sum, draw) => sum + draw.count, 0), points.length + markers.length);
  assert.equal(new Set(pointDraws.map(draw => draw.size)).size, pointDraws.length, 'one draw per size');
});

// Scene states whose output was frozen from the single-function scene before it
// was split into builders (tests/fixtures/scene-geometry.json). With only the
// overlays that scene drew switched on, the builders must keep drawing the same
// lines and points in the same order; overlays added later are switched off.
const FROZEN_OVERLAYS = { platformAxes: true, worldAxes: true };
const onlyFrozenOverlays = () => Object.fromEntries(OVERLAY_NAMES.map(name => [name, FROZEN_OVERLAYS[name] ?? false]));
function frozenSceneStates() {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 },
    workspaceRanges: sampleRanges() });
  controller.setTraces(true);
  for (const z of [2, 4, 6]) controller.requestPose({ z, rx: 0.02 * z, rz: 0.01 });
  const accepted = controller.requestPose({ x: 3, y: -2, z: 5, rx: 0.05, ry: -0.03, rz: 0.04 });
  const legFailure = controller.requestPose({ z: 200 });
  const globalFailure = { ...accepted, markers: false,
    assessment: { ...accepted.assessment, violations: [{ type: 'conditioning' }] } };
  const noAcceptedPose = createSimulatorController().loadLayout(asymmetricJointFixture(),
    { options: { ballJointLimitDeg: 180, conditionLimit: 1 } });
  return { accepted, legFailure, globalFailure, noAcceptedPose };
}
const frozenScene = JSON.parse(fs.readFileSync(new URL('../fixtures/scene-geometry.json', import.meta.url), 'utf8'));
const plain = value => JSON.parse(JSON.stringify(value));

test('overlay builders reproduce the frozen single-function scene exactly', () => {
  const states = frozenSceneStates();
  assert.equal(states.legFailure.rejected, true);
  assert.deepEqual(states.accepted.overlays, OVERLAY_DEFAULTS);
  for (const [name, state] of Object.entries(states)) {
    assert.deepEqual(plain(buildSceneGeometry({ ...state, overlays: onlyFrozenOverlays() })), frozenScene[name], name);
    // A state without an overlay map (older callers) draws the defaults.
    const { overlays, ...withoutOverlays } = state;
    assert.deepEqual(plain(buildSceneGeometry(withoutOverlays)),
      plain(buildSceneGeometry({ ...state, overlays: OVERLAY_DEFAULTS })), `${name} without overlays`);
  }
});

test('each overlay toggle removes only its own builder output', () => {
  // A rejected request so the ghost has output; the accepted pose is still drawn.
  // Two published cloud samples give the default-off reachability cloud output,
  // and the default-off conditioning ellipsoid is switched on, as are the
  // default-off loads with a published solution and the default-off servo
  // arcs, joint cones, workspace box and capability box (with a published result).
  const { legFailure: rejected } = frozenSceneStates();
  const state = { ...rejected,
    overlays: { ...rejected.overlays, reachabilityCloud: true, conditioningEllipsoid: true, loads: true,
      servoArcs: true, jointCones: true, workspaceBox: true, capabilityBox: true },
    capability: { homeReachable: true, orientation: { rx: 0, ry: 0, rz: 0 },
      extents: { x: { min: -30, max: 30 }, y: { min: -20, max: 25 }, z: { min: -10, max: 12 } } },
    loads: { valid: true, motion: 'static', reason: null, rodForceN: [3, -2, 4, -1, 2, 0.5],
      servoTorqueNm: [0.2, -0.1, 0.3, -0.05, 0.1, 0], servoSpeedRadPerSec: new Array(6).fill(0),
      ratedTorqueNm: new Array(6).fill(0.4), utilization: [0.5, 0.25, 0.75, 0.125, 0.25, 0], staticForceN: 10 },
    reachabilityCloud: { points: [{ at: [0, 0, 100], reachable: true }, { at: [10, 0, 100], reachable: false }],
      progress: 1, total: 2, orientation: { rx: 0, ry: 0, rz: 0 }, error: null } };
  const full = buildSceneGeometry(state);
  const parts = Object.fromEntries(SCENE_BUILDERS.map(builder =>
    [builder.name, builder.build(state, state.layout, state.acceptedAssessment)]));
  assert.deepEqual(SCENE_BUILDERS.map(builder => builder.name),
    ['groundGrid', 'base', 'platform', 'legs', 'servoArcs', 'jointCones', 'workspaceBox', 'capabilityBox', 'reachabilityCloud',
      'conditioningEllipsoid', 'loads', 'requestedGhost', 'platformAxes', 'worldAxes', 'trace']);
  assert.deepEqual(SCENE_BUILDERS.filter(builder => builder.overlay).map(builder => builder.overlay), OVERLAY_NAMES);
  assert.equal(parts.platformAxes.lines.length, 3);
  assert.equal(parts.worldAxes.lines.length, 3);
  assert.equal(parts.trace.lines.length, state.trace.length - 1);
  for (const name of OVERLAY_NAMES) {
    assert.ok(parts[name].lines.length + parts[name].points.length > 0, name);
    const scene = buildSceneGeometry({ ...state, overlays: { ...state.overlays, [name]: false } });
    for (const kind of ['lines', 'points']) {
      const removed = new Set(parts[name][kind].map(item => JSON.stringify(item)));
      const expected = full[kind].filter(item => !removed.has(JSON.stringify(item)));
      assert.equal(scene[kind].length, full[kind].length - parts[name][kind].length, `${name} ${kind}`);
      assert.deepEqual(plain(scene[kind]), plain(expected), `${name} ${kind}`);
    }
  }
  const bare = buildSceneGeometry({ ...state, overlays: Object.fromEntries(OVERLAY_NAMES.map(name => [name, false])) });
  assert.equal(bare.lines.length, full.lines.length - OVERLAY_NAMES.reduce((sum, name) => sum + parts[name].lines.length, 0));
  // Unknown names in a hand-built state are ignored rather than drawn.
  assert.deepEqual(plain(buildSceneGeometry({ ...state, overlays: { ...state.overlays, ghost: true } })), plain(full));
});

test('a custom builder list is drawn in order and overlay-gated', () => {
  const { accepted } = frozenSceneStates();
  const marker = { at: [0, 0, 0], color: [1, 1, 1], size: 3 };
  const builders = [{ name: 'dot', build: () => ({ lines: [], points: [marker] }) },
    { name: 'gated', overlay: 'worldAxes', build: () => ({ lines: [], points: [marker, marker] }) }];
  assert.equal(buildSceneGeometry(accepted, builders).points.length, 3);
  assert.equal(buildSceneGeometry({ ...accepted, overlays: { worldAxes: false } }, builders).points.length, 1);
  assert.deepEqual(buildSceneGeometry({ ...accepted, layout: null }, builders), { lines: [], points: [] });
});

test('servo arcs span the effective servo range in the evaluator horn plane and mark the accepted angle', () => {
  const { accepted } = frozenSceneStates();
  const { layout, acceptedAssessment: solved } = accepted;
  const arcs = SCENE_BUILDERS.find(builder => builder.name === 'servoArcs');
  const perLeg = 24 + 2 + 1; // Arc segments, two stop ticks, current-angle marker.
  const close = (actual, expected, label) => actual.forEach((value, i) =>
    assert.ok(Math.abs(value - expected[i]) < 1e-9, `${label}: ${actual} vs ${expected}`));
  for (const options of [{}, { servoRangeRad: [-0.4, 0.9] }]) {
    const state = { ...accepted, options: { ...accepted.options, ...options } };
    const [min, max] = effectiveServoRange(layout, state.options);
    const { lines } = arcs.build(state, layout, solved);
    assert.equal(lines.length, 6 * perLeg);
    for (let leg = 0; leg < 6; leg++) {
      const own = lines.slice(leg * perLeg, (leg + 1) * perLeg);
      const tip = alpha => computeHornTip(layout.baseAnchors[leg], layout.hornLength, layout.betaAngles[leg], alpha);
      close(own[0].from, tip(min), `leg ${leg + 1} min`);
      close(own[23].to, tip(max), `leg ${leg + 1} max`);
      for (let i = 1; i < 24; i++) assert.deepEqual(own[i].from, own[i - 1].to, 'the arc is continuous');
      // Every arc point is one horn length from the anchor.
      for (const line of own.slice(0, 24)) {
        assert.ok(Math.abs(Math.hypot(...line.to.map((v, k) => v - layout.baseAnchors[leg][k])) - layout.hornLength) < 1e-9);
      }
      // The marker crosses the arc exactly at the evaluator's horn tip.
      const marker = own[26];
      const crossing = marker.from.map((v, k) => v + (marker.to[k] - v) * (0.2 / 0.45));
      close(crossing, solved.hornTips[leg], `leg ${leg + 1} marker`);
    }
  }
  assert.equal(arcs.build({ ...accepted }, layout, null).lines.length, 6 * (perLeg - 1), 'no accepted angle, no marker');
});

test('servo arcs are neutral, tinted near a stop and failure-coloured on a requested servo violation', () => {
  const { accepted } = frozenSceneStates();
  const { layout, acceptedAssessment: solved } = accepted;
  const arcs = SCENE_BUILDERS.find(builder => builder.name === 'servoArcs');
  const legColors = state => {
    const { lines } = arcs.build(state, layout, solved);
    return [0, 1, 2, 3, 4, 5].map(leg => lines.slice(leg * 27, leg * 27 + 26).map(line => line.color));
  };
  const angles = solved.servoAngles;
  const [min, max] = effectiveServoRange(layout, accepted.options);
  assert.ok(angles.every(angle => Math.min(angle - min, max - angle) > NEAR_LIMIT_MARGIN_RAD), 'fixture starts clear of the stops');
  assert.ok(legColors(accepted).flat().every(color => color === SCENE_COLORS.limitRange));
  assert.equal(arcs.build(accepted, layout, solved).lines[26].color, SCENE_COLORS.horn);
  // Put the lowest leg 3° above its stop; the next lowest is then 5.5° clear.
  const lowest = angles.indexOf(Math.min(...angles));
  const near = { ...accepted, options: { ...accepted.options,
    servoRangeRad: [angles[lowest] - NEAR_LIMIT_MARGIN_RAD * 0.6, Math.max(...angles) + 1] } };
  legColors(near).forEach((colors, leg) => {
    const expected = leg === lowest ? SCENE_COLORS.nearLimit : SCENE_COLORS.limitRange;
    assert.ok(colors.every(color => color === expected), `leg ${leg + 1}`);
  });
  assert.equal(arcs.build(near, layout, solved).lines[lowest * 27 + 26].color, SCENE_COLORS.nearLimit);
  const failing = { ...accepted, assessment: { ...accepted.assessment, violations: [{ type: 'servoLimit', leg: 2, value: 2 }] } };
  legColors(failing).forEach((colors, leg) =>
    assert.ok(colors.every(color => color === (leg === 2 ? SCENE_COLORS.failure : SCENE_COLORS.limitRange)), `leg ${leg + 1}`));
  // A non-servo failure on the same leg leaves its arc neutral.
  const otherFailure = { ...accepted, assessment: { ...accepted.assessment, violations: [{ type: 'ballJoint', leg: 2 }] } };
  assert.ok(legColors(otherFailure).flat().every(color => color === SCENE_COLORS.limitRange));
  assert.equal(NEAR_LIMIT_MARGIN_RAD, 5 * Math.PI / 180);
});

// Lower 30°, upper 45°: distinct so a swapped socket cannot pass.
function coneState() {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180,
    lowerBallJointLimitDeg: 30, upperBallJointLimitDeg: 45 } });
  return { controller, state: controller.requestPose({ x: 3, y: -2, z: 5, rx: 0.05, ry: -0.03, rz: 0.04 }) };
}
const conesOf = state => SCENE_BUILDERS.find(builder => builder.name === 'jointCones')
  .build(state, state.layout, state.acceptedAssessment).lines;
const CONE_LINES = 24 + 4; // Ring segments and generatrices per socket.
const normalize = vector => { const length = Math.hypot(...vector); return vector.map(v => v / length); };
const dot = (a, b) => a.reduce((sum, v, k) => sum + v * b[k], 0);
// Apex, unit axis and half-angle recovered from one socket's drawn lines.
function readCone(lines) {
  const ring = lines.slice(0, 24).map(line => line.from);
  const apex = lines[24].from;
  const center = [0, 1, 2].map(k => ring.reduce((sum, point) => sum + point[k], 0) / ring.length);
  const axis = normalize(center.map((v, k) => v - apex[k]));
  const edge = normalize(ring[0].map((v, k) => v - apex[k]));
  return { apex, axis, halfAngle: Math.acos(dot(axis, edge)), ring };
}

test('joint cones sit on the socket normals of the mounting model with the effective limit as half-angle', () => {
  const { state } = coneState();
  const solved = state.acceptedAssessment;
  assert.equal(state.rejected, false);
  const lines = conesOf(state);
  assert.equal(lines.length, 12 * CONE_LINES);
  const { mounting } = resolveMounting(state.layout);
  const close = (actual, expected, label) => actual.forEach((value, i) =>
    assert.ok(Math.abs(value - expected[i]) < 1e-9, `${label}: ${actual} vs ${expected}`));
  for (let leg = 0; leg < 6; leg++) {
    const lower = readCone(lines.slice((2 * leg) * CONE_LINES, (2 * leg + 1) * CONE_LINES));
    const upper = readCone(lines.slice((2 * leg + 1) * CONE_LINES, (2 * leg + 2) * CONE_LINES));
    close(lower.apex, solved.hornTips[leg], `leg ${leg + 1} lower apex`);
    close(upper.apex, solved.platformPoints[leg], `leg ${leg + 1} upper apex`);
    close(lower.axis, hornLocalToWorld(state.layout.betaAngles[leg], solved.servoAngles[leg],
      mounting.lower[leg].direction), `leg ${leg + 1} lower axis`);
    close(upper.axis, rotateVector(solved.rotationMatrix, mounting.upper[leg].direction), `leg ${leg + 1} upper axis`);
    assert.ok(Math.abs(lower.halfAngle - 30 * Math.PI / 180) < 1e-9, `leg ${leg + 1} lower half-angle ${lower.halfAngle}`);
    assert.ok(Math.abs(upper.halfAngle - 45 * Math.PI / 180) < 1e-9, `leg ${leg + 1} upper half-angle ${upper.halfAngle}`);
    // The evaluator's joint angle is the rod's angle from these same axes.
    const rod = normalize(solved.rodVectors[leg]);
    const angle = (axis, direction) => Math.acos(Math.max(-1, Math.min(1, dot(axis, direction))));
    assert.ok(Math.abs(angle(lower.axis, rod) - solved.jointAngles.lower[leg]) < 1e-9);
    assert.ok(Math.abs(angle(upper.axis, rod.map(v => -v)) - solved.jointAngles.upper[leg]) < 1e-9);
  }
  // A wide 180° limit stays bounded: every ring point is one slant length away.
  const wide = createSimulatorController().loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const slant = Math.max(12, wide.layout.hornLength * 0.35);
  const first = readCone(conesOf(wide).slice(0, CONE_LINES));
  for (const point of first.ring) assert.ok(Math.abs(Math.hypot(...point.map((v, k) => v - first.apex[k])) - slant) < 1e-9);
  assert.deepEqual(conesOf({ ...wide, acceptedAssessment: null }), []);
});

test('joint cones are neutral, tinted near the limit and failure-coloured per socket', () => {
  const { state } = coneState();
  const solved = state.acceptedAssessment;
  const socketColors = target => {
    const lines = conesOf(target);
    return Array.from({ length: 12 }, (_, socket) =>
      lines.slice(socket * CONE_LINES, (socket + 1) * CONE_LINES).map(line => line.color));
  };
  const uniform = (colors, expected, label) => assert.ok(colors.every(color => color === expected), label);
  socketColors(state).forEach((colors, socket) => uniform(colors, SCENE_COLORS.limitRange, `socket ${socket}`));
  // A 12° lower limit puts some lower sockets (2.5° to 9.4° here) within the 5°
  // margin and leaves the rest, and every upper socket, neutral.
  const limit = 12 * Math.PI / 180;
  const near = { ...state, acceptedAssessment: { ...solved, jointLimits: { ...solved.jointLimits, lower: limit } } };
  const tinted = solved.jointAngles.lower.map(angle => limit - angle < NEAR_LIMIT_MARGIN_RAD);
  assert.ok(tinted.includes(true) && tinted.includes(false), `fixture spread: ${tinted}`);
  socketColors(near).forEach((colors, socket) => uniform(colors,
    socket % 2 === 0 && tinted[socket / 2] ? SCENE_COLORS.nearLimit : SCENE_COLORS.limitRange, `near socket ${socket}`));
  // Only the named socket turns red; the same leg's other socket and other failure types do not.
  const failing = { ...state, assessment: { ...state.assessment,
    violations: [{ type: 'ballJoint', leg: 1, joint: 'upper' }, { type: 'servoLimit', leg: 4 }] } };
  socketColors(failing).forEach((colors, socket) => uniform(colors,
    socket === 3 ? SCENE_COLORS.failure : SCENE_COLORS.limitRange, `failing socket ${socket}`));
});

test('a joint-limit edit in the diagnostics panel redraws the cones in the same notification', () => {
  const document = createFakeDocument();
  const controller = createSimulatorController();
  const rendered = [];
  const renderer = { available: true, contextLost: false, render(state) { rendered.push(state); }, dispose() {} };
  createSimulatorView({ document, window: { addEventListener() {} }, controller, createRenderer: () => renderer });
  mountSimulatorDiagnostics({ document, controller });
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180,
    lowerBallJointLimitDeg: 30, upperBallJointLimitDeg: 45 } });
  const upperHalfAngle = state => readCone(conesOf(state).slice(CONE_LINES, 2 * CONE_LINES)).halfAngle;
  assert.ok(Math.abs(upperHalfAngle(rendered.at(-1)) - 45 * Math.PI / 180) < 1e-9);
  const before = rendered.length;
  const field = document.getElementById('simUpperJointLimit');
  field.value = '20';
  field.dispatch('change');
  assert.ok(rendered.length > before, 'the edit did not redraw');
  assert.ok(Math.abs(upperHalfAngle(rendered.at(-1)) - 20 * Math.PI / 180) < 1e-9);
});

const ghostOf = state => SCENE_BUILDERS.find(builder => builder.name === 'requestedGhost')
  .build(state, state.layout, state.acceptedAssessment).lines;
const positions = lines => plain(lines.map(line => [line.from, line.to]));
const FULL_COLORS = new Set(Object.values(SCENE_COLORS));
const isDimmed = color => !FULL_COLORS.has(color) && color.every((value, k) => value > SCENE_BACKGROUND[k] - 1e-12);

test('a rejected request adds a ghost while the accepted geometry stays put; an accepted one adds none', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const before = controller.requestPose({ x: 3, rx: 0.05 });
  assert.equal(before.rejected, false);
  assert.deepEqual(ghostOf(before), [], 'an accepted request drew a ghost');
  const withoutGhost = state => buildSceneGeometry({ ...state, overlays: { ...state.overlays, requestedGhost: false } });
  const rejected = controller.requestPose({ z: 200 }); // Beyond the workspace: leg 1 cannot close.
  assert.equal(rejected.rejected, true);
  assert.deepEqual(rejected.accepted, before.accepted);
  assert.deepEqual(positions(withoutGhost(rejected).lines), positions(withoutGhost(before).lines),
    'the accepted geometry moved');
  const ghost = ghostOf(rejected);
  assert.equal(ghost.length, 6 + 3, 'platform outline and axes only: the solver reached no horn tip');
  assert.equal(buildSceneGeometry(rejected).lines.length, withoutGhost(rejected).lines.length + ghost.length);
  assert.deepEqual(buildSceneGeometry(rejected).points, withoutGhost(rejected).points, 'the ghost drew markers');
  // The ghost platform is the requested translation and rotation applied to the anchors.
  const { translation, rotationMatrix } = rejected.assessment;
  assert.deepEqual(translation, [0, 0, rejected.layout.homeHeight + 200]);
  rejected.layout.platformAnchors.forEach((anchor, i) =>
    assert.deepEqual(ghost[i].from, vectorAdd(translation, rotateVector(rotationMatrix, anchor))));
  assert.ok(ghost.every(line => isDimmed(line.color)), 'an unsolved ghost used a full-brightness colour');
  const back = controller.requestPose({});
  assert.deepEqual(ghostOf(back), [], 'the ghost outlived an accepted request');
});

test('ghost legs follow the solver and failing legs use the full failure colour', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180,
    lowerBallJointLimitDeg: 180, upperBallJointLimitDeg: 3 } });
  const state = controller.requestPose({ x: 3, y: -2, z: 5, rx: 0.05, ry: -0.03, rz: 0.04 });
  const requested = state.assessment;
  const failed = new Set(requested.violations.map(violation => violation.leg));
  assert.ok(requested.violations.length > 0 && requested.violations.every(violation => violation.type === 'ballJoint'));
  assert.ok(failed.size > 0 && failed.size < 6, `fixture should fail some legs, not all: ${[...failed]}`);
  assert.equal(requested.hornTips.length, 6, 'a joint failure still records every horn tip');
  const ghost = ghostOf(state);
  assert.equal(ghost.length, 6 + 12 + 3);
  const legs = ghost.slice(6, 18);
  for (let leg = 0; leg < 6; leg++) {
    const [horn, rod] = legs.slice(2 * leg, 2 * leg + 2);
    assert.deepEqual(horn.from, state.layout.baseAnchors[leg]);
    assert.deepEqual(horn.to, requested.hornTips[leg]);
    assert.deepEqual(rod.to, requested.platformPoints[leg]);
    for (const line of [horn, rod]) {
      if (failed.has(leg)) assert.equal(line.color, SCENE_COLORS.failure, `leg ${leg + 1}`);
      else assert.ok(isDimmed(line.color), `leg ${leg + 1}`);
    }
  }
  assert.deepEqual(plain(ghost.slice(0, 6).map(line => line.from)), plain(requested.platformPoints));
  // A whole-platform failure outlines the ghost platform in its own colour.
  const global = { ...state, assessment: { ...requested, violations: [{ type: 'conditionLimit' }] } };
  const outline = ghostOf(global).slice(0, 6);
  assert.ok(outline.every(line => line.color === SCENE_COLORS.globalFailure));
  assert.ok(ghostOf(global).slice(6).every(line => isDimmed(line.color)));
  // The ghost is gated like any overlay.
  assert.equal(buildSceneGeometry({ ...state, overlays: { ...state.overlays, requestedGhost: false } }).lines.length,
    buildSceneGeometry(state).lines.length - ghost.length);
});

// Colours of the held (accepted) pose's leg geometry: servo stub, horn, rod and markers.
function heldLegColors(state) {
  const scene = buildSceneGeometry(state);
  const { layout, acceptedAssessment: solved } = state;
  const line = (from, to) => scene.lines.find(item => item.from === from && item.to === to).color;
  return [0, 1, 2, 3, 4, 5].map(leg => {
    const base = layout.baseAnchors[leg];
    const stub = scene.lines.find(item => item.from === base && item.to !== solved.hornTips[leg]
      && !layout.baseAnchors.includes(item.to));
    return { stub: stub.color,
      horn: line(base, solved.hornTips[leg]), rod: line(solved.hornTips[leg], solved.platformPoints[leg]),
      markers: scene.points.filter(point => point.at === base || point.at === solved.hornTips[leg]
        || point.at === solved.platformPoints[leg]).map(point => point.color) };
  });
}
const NORMAL_LEG = { stub: SCENE_COLORS.servo, horn: SCENE_COLORS.horn, rod: SCENE_COLORS.rod,
  markers: [SCENE_COLORS.servo, SCENE_COLORS.horn, SCENE_COLORS.platform] };
const paintedLeg = color => ({ stub: color, horn: color, rod: color, markers: [color, color, color] });
const withGhost = (state, on) => ({ ...state, overlays: { ...state.overlays, requestedGhost: on } });

test('a failure is coloured once: on the ghost when it can draw it, otherwise on the held pose', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180,
    lowerBallJointLimitDeg: 180, upperBallJointLimitDeg: 3 } });
  const jointFailure = controller.requestPose({ x: 3, y: -2, z: 5, rx: 0.05, ry: -0.03, rz: 0.04 });
  const failed = new Set(jointFailure.assessment.violations.map(violation => violation.leg));
  assert.ok(failed.size > 0 && failed.size < 6);
  // Ghost on: the joint failures are red on the ghost legs, so the held legs keep their colours.
  assert.deepEqual(heldLegColors(withGhost(jointFailure, true)), Array(6).fill(NORMAL_LEG));
  assert.ok(ghostOf(jointFailure).some(line => line.color === SCENE_COLORS.failure));
  // Ghost off: the held legs carry the failure colour, as before the ghost existed.
  assert.deepEqual(heldLegColors(withGhost(jointFailure, false)),
    [0, 1, 2, 3, 4, 5].map(leg => failed.has(leg) ? paintedLeg(SCENE_COLORS.failure) : NORMAL_LEG));
  // A structural failure leaves no ghost leg to colour, so the held leg stays red with the ghost on.
  const structural = controller.requestPose({ z: 200 });
  assert.deepEqual(structural.assessment.violations.map(violation => violation.leg), [0]);
  assert.deepEqual(heldLegColors(withGhost(structural, true)),
    [paintedLeg(SCENE_COLORS.failure), ...Array(5).fill(NORMAL_LEG)]);
  // A whole-platform failure outlines the ghost; the held legs turn magenta only without it.
  const global = { ...jointFailure, assessment: { ...jointFailure.assessment, violations: [{ type: 'conditionLimit' }] } };
  assert.deepEqual(heldLegColors(withGhost(global, true)), Array(6).fill(NORMAL_LEG));
  assert.ok(ghostOf(global).slice(0, 6).every(line => line.color === SCENE_COLORS.globalFailure));
  assert.deepEqual(heldLegColors(withGhost(global, false)), Array(6).fill(paintedLeg(SCENE_COLORS.globalFailure)));
  // An accepted request has nothing to colour.
  assert.deepEqual(heldLegColors(controller.requestPose({})), Array(6).fill(NORMAL_LEG));
});

test('ghost failure lines win depth ties with the held pose and dimmed ghost lines lose them', () => {
  const controller = createSimulatorController();
  controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180,
    lowerBallJointLimitDeg: 180, upperBallJointLimitDeg: 3 } });
  const state = controller.requestPose({ x: 3, y: -2, z: 5, rx: 0.05, ry: -0.03, rz: 0.04 });
  const ghost = ghostOf(state);
  assert.ok(ghost.some(line => line.color === SCENE_COLORS.failure) && ghost.some(line => isDimmed(line.color)));
  for (const line of ghost) {
    assert.equal(line.depthBias, line.color === SCENE_COLORS.failure ? GHOST_DEPTH_BIAS_MM : -GHOST_DEPTH_BIAS_MM);
  }
  const global = { ...state, assessment: { ...state.assessment, violations: [{ type: 'conditionLimit' }] } };
  assert.ok(ghostOf(global).slice(0, 6).every(line => line.depthBias === GHOST_DEPTH_BIAS_MM));
  assert.ok(ghostOf(global).slice(6).every(line => line.depthBias === -GHOST_DEPTH_BIAS_MM));
  // No other builder but the ground grid biases its lines.
  assert.ok(buildSceneGeometry({ ...state, overlays: { ...state.overlays, requestedGhost: false, groundGrid: false } })
    .lines.every(line => line.depthBias === undefined));
  // The bias shifts depth by exactly its length in the depth mapping, either
  // way, and leaves x and y alone.
  const camera = { target: [0, 0, 100], yaw: 0.7, pitch: 0.4, distance: 600 };
  const plainPoint = projectPoint([10, -20, 150], camera, 800, 500);
  for (const bias of [GHOST_DEPTH_BIAS_MM, -GHOST_DEPTH_BIAS_MM]) {
    const biased = projectPoint([10, -20, 150], camera, 800, 500, bias);
    assert.deepEqual(biased.slice(0, 2), plainPoint.slice(0, 2));
    assert.ok(Math.abs((plainPoint[2] - biased[2]) - bias * 2 / 2000) < 1e-12);
  }
});

test('lines crossing the near plane are clipped there instead of dropped', () => {
  const camera = { target: [0, 0, 100], yaw: 0.7, pitch: 0.4, distance: 50 };
  const frame = cameraFrame(camera);
  const along = (depth, side) => frame.eye.map((value, k) => value + frame.forward[k] * depth + frame.right[k] * side);
  // Both ends in front: identical to projecting each end.
  const front = [along(20, -5), along(80, 5)];
  assert.deepEqual(projectSegment(...front, camera, 800, 500),
    [projectPoint(front[0], camera, 800, 500), projectPoint(front[1], camera, 800, 500)]);
  // One end behind the eye: the visible part is kept, ending at the near plane.
  const behind = along(-30, -5), ahead = along(30, 5);
  assert.equal(projectPoint(behind, camera, 800, 500), null);
  const [start, end] = projectSegment(behind, ahead, camera, 800, 500);
  assert.deepEqual(end, projectPoint(ahead, camera, 800, 500));
  assert.ok(start[2] < -0.99, `the clipped end sits at the near plane: ${start[2]}`);
  const t = (30 - NEAR_PLANE_MM) / 60; // Fraction of the way from ahead toward behind at the near plane.
  const onLine = ahead.map((value, k) => value + (behind[k] - value) * t);
  const expected = projectPoint(onLine.map((value, k) => value + frame.forward[k] * 1e-3), camera, 800, 500);
  assert.ok(Math.abs(start[0] - expected[0]) < 1e-3 && Math.abs(start[1] - expected[1]) < 1e-3, `${start} vs ${expected}`);
  assert.deepEqual(projectSegment(ahead, behind, camera, 800, 500), [end, start], 'order is kept');
  assert.equal(projectSegment(behind, along(-5, 3), camera, 800, 500), null, 'wholly behind');
  // The draw path uses the clipping: a close-up keeps lines that pass beside the eye.
  const gl = fakeGL();
  let lineVertices = 0;
  gl.bufferData = (_target, data) => { lineVertices = lineVertices || data.length / 6; };
  const canvas = { width: 800, height: 500, clientWidth: 800, clientHeight: 500, getContext: () => gl, addEventListener() {}, removeEventListener() {} };
  const state = createSimulatorController().loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const closeUp = { target: [0, 0, state.layout.homeHeight / 2], yaw: 0.7, pitch: 0.4, distance: 10 };
  createWebGLRenderer(canvas).render(state, closeUp);
  const lines = buildSceneGeometry(state).lines;
  const bothEnds = lines.filter(line => projectPoint(line.from, closeUp, 800, 500) && projectPoint(line.to, closeUp, 800, 500)).length;
  const clipped = lines.filter(line => projectSegment(line.from, line.to, closeUp, 800, 500)).length;
  assert.ok(clipped > bothEnds, `the close-up should have lines with one end behind the eye: ${clipped} vs ${bothEnds}`);
  assert.equal(lineVertices, 2 * clipped);
});

const buildOf = name => SCENE_BUILDERS.find(builder => builder.name === name).build;
const key = point => point.map(value => Number(value.toFixed(9))).join(',');

test('the workspace box is the twelve edges of the sample x/y/z ranges about home', () => {
  const controller = createSimulatorController();
  const state = controller.loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 },
    workspaceRanges: sampleRanges() });
  const home = state.layout.homeHeight;
  const edges = buildOf('workspaceBox')(state, state.layout, state.acceptedAssessment).lines;
  assert.equal(edges.length, 12);
  assert.ok(edges.every(edge => edge.color === SCENE_COLORS.workspace && edge.depthBias === undefined));
  const corners = [];
  for (const x of [-40, 40]) for (const y of [-40, 40]) for (const z of [home - 20, home + 40]) corners.push([x, y, z]);
  const drawn = new Set(edges.flatMap(edge => [key(edge.from), key(edge.to)]));
  assert.deepEqual([...drawn].sort(), corners.map(key).sort());
  // Every edge runs along one axis for that axis's full span, four per axis.
  const spans = [80, 80, 60];
  const perAxis = [0, 0, 0];
  for (const edge of edges) {
    const moved = [0, 1, 2].filter(k => Math.abs(edge.to[k] - edge.from[k]) > 1e-9);
    assert.equal(moved.length, 1);
    assert.ok(Math.abs(Math.abs(edge.to[moved[0]] - edge.from[moved[0]]) - spans[moved[0]]) < 1e-9);
    perAxis[moved[0]]++;
  }
  assert.deepEqual(perAxis, [4, 4, 4]);
  // Rotation ranges are not drawn, and the box stays put when the pose moves.
  const { rx, ry, rz, ...translations } = state.workspaceRanges;
  assert.ok(rx && ry && rz);
  const moved = controller.requestPose({ x: 5, z: 10, rz: 0.1 });
  assert.deepEqual(plain(buildOf('workspaceBox')({ ...moved, workspaceRanges: translations }, moved.layout, null).lines),
    plain(edges));
  // No ranges, or no Z range, draws no box.
  assert.deepEqual(buildOf('workspaceBox')({ ...state, workspaceRanges: null }, state.layout, null).lines, []);
  assert.deepEqual(buildOf('workspaceBox')({ ...state, workspaceRanges: { x: translations.x, y: translations.y } },
    state.layout, null).lines, []);
  const boxOn = scene => ({ ...scene, overlays: { ...scene.overlays, workspaceBox: true } });
  assert.equal(buildSceneGeometry(boxOn(controller.setWorkspaceRanges(null))).lines.length,
    buildSceneGeometry(boxOn(moved)).lines.length - 12);
});

test('the ground grid spans the base on z = 0 at the fixed pitch, behind anything it touches', () => {
  const state = createSimulatorController().loadLayout(asymmetricJointFixture(), { options: { ballJointLimitDeg: 180 } });
  const lines = buildOf('groundGrid')(state, state.layout, null).lines;
  const radius = Math.max(...state.layout.baseAnchors.map(([x, y]) => Math.hypot(x, y)));
  const half = Math.max(...lines.map(line => line.to[0]));
  const cells = half / GROUND_GRID_PITCH_MM;
  assert.equal(GROUND_GRID_PITCH_MM, 25);
  assert.ok(Number.isInteger(cells) && half >= 1.5 * radius && half < 1.5 * radius + GROUND_GRID_PITCH_MM, `${half} vs ${radius}`);
  assert.equal(lines.length, 2 * (2 * cells + 1));
  const offsets = new Set();
  for (const line of lines) {
    assert.equal(line.from[2], 0);
    assert.equal(line.to[2], 0);
    assert.equal(line.color, SCENE_COLORS.grid);
    assert.equal(line.depthBias, -GROUND_DEPTH_BIAS_MM);
    const along = line.from[0] === line.to[0] ? 1 : 0;
    assert.deepEqual([line.from[along], line.to[along]], [-half, half]);
    offsets.add(line.from[1 - along]);
  }
  assert.deepEqual([...offsets].sort((a, b) => a - b),
    Array.from({ length: 2 * cells + 1 }, (_, k) => (k - cells) * GROUND_GRID_PITCH_MM));
  // Pushed back only in depth: a grid line under a world axis loses the tie.
  const camera = { target: [0, 0, 100], yaw: 0.7, pitch: 0.4, distance: 600 };
  const [gridStart] = projectSegment([0, 0, 0], [30, 0, 0], camera, 800, 500, -GROUND_DEPTH_BIAS_MM);
  const [axisStart] = projectSegment([0, 0, 0], [30, 0, 0], camera, 800, 500);
  assert.deepEqual(gridStart.slice(0, 2), axisStart.slice(0, 2));
  assert.ok(gridStart[2] > axisStart[2]);
});
