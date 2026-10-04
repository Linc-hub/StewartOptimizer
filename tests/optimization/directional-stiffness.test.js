import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { evaluateLayout } from '../../src/optimization/evaluate-layout.js';
import { objectiveDefinitions, objectiveValues } from '../../src/optimization/objectives.js';
import { directionalStiffness, STIFFNESS_DIRECTIONS, validateStiffnessDirection }
  from '../../src/model/conditioning.js';
import { evaluatePose } from '../../src/model/pose.js';
import { optionsFromEffectiveSettings } from '../../src/ui/worker-protocol.js';
import { pairedFixture } from '../fixtures/layout.js';

const RUN = { seed: 3, populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' } };
const HOME = { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };
const RANGES = { z: { min: -10, max: 10, step: 10 }, rx: { min: -0.05, max: 0.05, step: 0.05 } };

test('directional stiffness reads the compliance diagonal of a known Jacobian', () => {
  // A diagonal Jacobian: axis i alone has stiffness proxy |d_i|.
  const diagonal = [2, 3, 4, 5, 6, 7];
  const rows = diagonal.map((value, i) => diagonal.map((_, j) => (i === j ? value : 0)));
  STIFFNESS_DIRECTIONS.forEach((axis, i) => assert.ok(Math.abs(directionalStiffness(rows, axis) - diagonal[i]) < 1e-12));
  // Coupling the first two axes lowers both below their uncoupled values.
  const coupled = rows.map(row => row.slice());
  coupled[0][1] = 2;
  const expected = 1 / Math.hypot(1 / 2, -2 / 6);
  assert.ok(Math.abs(directionalStiffness(coupled, 'x') - expected) < 1e-12);
  // Singular and malformed Jacobians give null.
  const singular = rows.map(row => row.slice());
  singular[5] = singular[4].slice();
  assert.equal(directionalStiffness(singular, 'z'), null);
  assert.equal(directionalStiffness(rows.slice(0, 5), 'x'), null);
  assert.equal(directionalStiffness(rows, 'w'), null);
  assert.equal(validateStiffnessDirection(null), null);
  assert.throws(() => validateStiffnessDirection('w'), /stiffnessDirection must be one of x, y, z, rx, ry, rz/);
});

test('every directional value lies between sigmaMin and sigmaMax at a real pose', () => {
  const layout = pairedFixture();
  const { conditioning } = evaluatePose(layout, HOME, { recordLegData: true });
  assert.ok(conditioning.satisfied);
  for (const axis of STIFFNESS_DIRECTIONS) {
    const value = directionalStiffness(conditioning.jacobianRows, axis);
    assert.ok(value >= conditioning.sigmaMin * (1 - 1e-9) && value <= conditioning.sigmaMax * (1 + 1e-9), axis);
  }
});

test('the evaluator reports the worst of home and the workspace only when a direction is set', async () => {
  const plain = await evaluateLayout(pairedFixture(), { ranges: RANGES });
  assert.equal(plain.directionalStiffness, null);
  assert.equal(plain.workspace.stats.worstDirectionalStiffness, undefined);
  assert.equal(plain.conditioning.directional, undefined);
  const result = await evaluateLayout(pairedFixture(), { ranges: RANGES, stiffnessDirection: 'z' });
  const { home, workspaceWorst, value, direction } = result.conditioning.directional;
  assert.equal(direction, 'z');
  assert.ok(Number.isFinite(home) && Number.isFinite(workspaceWorst));
  assert.equal(value, Math.min(home, workspaceWorst));
  assert.equal(result.directionalStiffness, value);
  assert.deepEqual(result.objectives, plain.objectives, 'objectives follow the variant, not the option');
});

test('the stiffness emphasis variant appends a maximised directional objective', () => {
  const plain = objectiveDefinitions('compact');
  const emphasis = objectiveDefinitions('compact', { directionalStiffness: 'rx' });
  assert.deepEqual(emphasis.slice(0, -1), plain);
  assert.deepEqual({ ...emphasis.at(-1), approximation: null },
    { key: 'directionalStiffness', direction: 'max', unit: 'proxy', approximation: null, axis: 'rx' });
  assert.deepEqual(objectiveDefinitions('compact', { footprint: true, directionalStiffness: 'x' }).slice(-2).map(entry => entry.key),
    ['footprint', 'directionalStiffness']);
  const metrics = { coverage: 100, conditioningQuality: 0.2, torque: 3, speedDemand: 4, directionalStiffness: 0.7 };
  assert.deepEqual(objectiveValues(metrics, 'compact', { directionalStiffness: 'x' }), [100, 0.2, -3, -4, 0.7]);
});

test('a stiffness emphasis run ranks by it and replays with it; the default run does not', async () => {
  const plain = new Optimizer({}, RUN);
  await plain.run();
  assert.equal(plain.effectiveSettings().stiffnessDirection, null);
  assert.ok(plain.fitness.every(evaluation => evaluation.objectives.length === 4 && evaluation.directionalStiffness === null));
  const run = async () => {
    const optimizer = new Optimizer({}, { ...RUN, stiffnessDirection: 'y' });
    await optimizer.run();
    return optimizer;
  };
  const emphasis = await run();
  const settings = emphasis.effectiveSettings();
  assert.equal(settings.objectiveDefinitions.at(-1).key, 'directionalStiffness');
  assert.ok(emphasis.fitness.every(evaluation => evaluation.objectives.length === 5
    && (evaluation.directionalStiffness == null ? evaluation.objectives[4] === -Infinity
      : evaluation.objectives[4] === evaluation.directionalStiffness)));
  assert.ok(emphasis.fitness.some(evaluation => Number.isFinite(evaluation.directionalStiffness)));
  const exported = JSON.parse(emphasis.exportBest());
  assert.ok('directional_stiffness' in exported.metadata);
  assert.equal(exported.run.effective_settings.stiffnessDirection, 'y');
  const replay = new Optimizer(settings.requirements, optionsFromEffectiveSettings(settings));
  assert.equal(replay.stiffnessDirection, 'y');
  // Deterministic for a seed, and the same layouts as the default run: the option draws no random numbers.
  const again = await run();
  assert.deepEqual(again.fitness.map(evaluation => evaluation.objectives), emphasis.fitness.map(evaluation => evaluation.objectives));
  assert.throws(() => new Optimizer({}, { ...RUN, stiffnessDirection: 'up' }), /stiffnessDirection must be one of/);
});
