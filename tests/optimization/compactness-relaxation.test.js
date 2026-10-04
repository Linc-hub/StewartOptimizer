import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE } from '../../src/optimization/layout-operators.js';
import { layoutFootprint } from '../../src/optimization/compactness.js';
import { objectiveDefinitions, objectiveValues } from '../../src/optimization/objectives.js';
import { boundsExcursions, describeRelaxation, RELAXABLE_FIELDS, relaxedDesignSpace, validateBoundsRelaxation }
  from '../../src/optimization/relaxation.js';
import { optionsFromEffectiveSettings } from '../../src/ui/worker-protocol.js';
import { asymmetricJointFixture, pairedFixture } from '../fixtures/layout.js';

const RUN = { seed: 3, populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' } };
// Bounds no layout can pass with, so a relaxing run never finds a passing candidate.
const IMPOSSIBLE = { horn_length_bounds_mm: [8, 10], rod_length_bounds_mm: [60, 70] };
const HARD_RUN = { ...RUN, generations: 2, ranges: { z: { min: -20, max: 20, step: 20 } } };

test('the footprint is the cylinder holding every horn reach and platform anchor', () => {
  const layout = asymmetricJointFixture();
  const expected = Math.max(...layout.baseAnchors.map(([x, y]) => Math.hypot(x, y) + layout.hornLength),
    ...layout.platformAnchors.map(([x, y]) => Math.hypot(x, y)));
  assert.equal(layoutFootprint(layout), expected);
  assert.equal(layoutFootprint({ ...layout, hornLength: layout.hornLength + 5 }), expected + 5);
});

test('the compactness variant appends a minimised footprint objective to any set', () => {
  for (const set of ['compact', 'full']) {
    const plain = objectiveDefinitions(set);
    const compact = objectiveDefinitions(set, { footprint: true });
    assert.deepEqual(compact.slice(0, -1), plain);
    assert.deepEqual({ ...compact.at(-1), approximation: null },
      { key: 'footprint', direction: 'min', unit: 'mm', approximation: null });
  }
  const metrics = { coverage: 100, conditioningQuality: 0.2, torque: 3, speedDemand: 4, footprint: 180 };
  assert.deepEqual(objectiveValues(metrics, 'compact', { footprint: true }), [100, 0.2, -3, -4, -180]);
});

test('a compactness run ranks by footprint and replays with it; the default run does not', async () => {
  const plain = new Optimizer({}, RUN);
  await plain.run();
  assert.equal(plain.effectiveSettings().compactness, false);
  assert.ok(plain.fitness.every(evaluation => evaluation.objectives.length === 4 && Number.isFinite(evaluation.footprint)));
  const compact = new Optimizer({}, { ...RUN, compactness: true });
  await compact.run();
  const settings = compact.effectiveSettings();
  assert.equal(settings.objectiveDefinitions.at(-1).key, 'footprint');
  assert.ok(compact.fitness.every(evaluation => evaluation.objectives.length === 5
    && evaluation.objectives[4] === -evaluation.footprint));
  const replay = new Optimizer(settings.requirements, optionsFromEffectiveSettings(settings));
  assert.equal(replay.compactness, true);
  assert.throws(() => new Optimizer({}, { ...RUN, compactness: 'yes' }), /compactness must be true or false/);
});

test('relaxed design spaces widen every relaxable bound by the share of its value', () => {
  assert.equal(relaxedDesignSpace(DEFAULT_DESIGN_SPACE, 0), DEFAULT_DESIGN_SPACE);
  const relaxed = relaxedDesignSpace(DEFAULT_DESIGN_SPACE, 0.25);
  for (const field of RELAXABLE_FIELDS) {
    const [min, max] = DEFAULT_DESIGN_SPACE[field];
    assert.deepEqual(relaxed[field], [min * 0.75, max * 1.25], field);
  }
  assert.deepEqual(relaxed.rectangularAspectBounds, DEFAULT_DESIGN_SPACE.rectangularAspectBounds);
  assert.equal(validateBoundsRelaxation(0.1), 0.1);
  assert.throws(() => validateBoundsRelaxation(0.3), /boundsRelaxation must be one of 0, 0.1, 0.25, 0.5/);
});

test('bounds excursions name each field outside the nominal space and by how much', () => {
  const layout = pairedFixture();
  assert.deepEqual(boundsExcursions(layout, relaxedDesignSpace(DEFAULT_DESIGN_SPACE, 0.5))
    .filter(entry => entry.field === 'hornLengthBounds'), []);
  const space = { ...DEFAULT_DESIGN_SPACE, hornLengthBounds: [10, layout.hornLength - 2], rodLengthBounds: [layout.rodLength + 3, 500] };
  const found = boundsExcursions(layout, space);
  const horn = found.find(entry => entry.field === 'hornLengthBounds');
  const rod = found.find(entry => entry.field === 'rodLengthBounds');
  assert.ok(Math.abs(horn.excessMm - 2) < 1e-9);
  assert.ok(Math.abs(rod.excessMm + 3) < 1e-9);
  assert.deepEqual(rod.nominal, [layout.rodLength + 3, 500]);
});

test('a relaxing run widens step by step while nothing passes, adds fresh layouts and reports it', async () => {
  const run = async () => {
    const optimizer = new Optimizer(IMPOSSIBLE, { ...HARD_RUN, boundsRelaxation: 0.5 });
    await optimizer.run();
    return optimizer;
  };
  const optimizer = await run();
  assert.deepEqual(optimizer.relaxationHistory, [
    { generation: 0, passing: 0, share: 0.125 },
    { generation: 1, passing: 0, share: 0.25 },
    { generation: 2, passing: 0, share: 0.375 },
  ]);
  assert.ok(optimizer.fitness.every(evaluation => Array.isArray(evaluation.boundsExcursions)));
  assert.ok(optimizer.fitness.some(evaluation => evaluation.layout.seedOrigin === 'relaxation'));
  const exported = JSON.parse(optimizer.exportBest());
  assert.equal(exported.run.bounds_relaxation.finalShare, 0.375);
  assert.equal(exported.run.effective_settings.boundsRelaxation, 0.5);
  assert.deepEqual(exported.run.bounds_relaxation.designSpace.hornLengthBounds, [8 * 0.625, 10 * 1.375]);
  assert.match(describeRelaxation(exported.run.bounds_relaxation), /^Geometry bounds relaxed by 38% of up to 50%; \d+ retained candidates? lies? outside/);
  // Deterministic for a seed.
  const again = await run();
  assert.deepEqual(again.fitness.map(evaluation => evaluation.objectives), optimizer.fitness.map(evaluation => evaluation.objectives));
});

test('a run without relaxation never widens or reports excursions', async () => {
  const optimizer = new Optimizer(IMPOSSIBLE, HARD_RUN);
  await optimizer.run();
  assert.equal(optimizer.relaxationShare, 0);
  assert.deepEqual(optimizer.relaxationHistory, []);
  assert.equal(optimizer.relaxationSummary(), null);
  assert.ok(optimizer.fitness.every(evaluation => evaluation.boundsExcursions === undefined));
  assert.ok(optimizer.fitness.every(evaluation => evaluation.layout.seedOrigin !== 'relaxation'));
  assert.equal(JSON.parse(optimizer.exportBest()).run.bounds_relaxation, undefined);
  assert.equal(describeRelaxation({ maxShare: 0.25, finalShare: 0, outsideNominal: 0 }),
    'Geometry bounds were not relaxed (allowed up to 25%; a candidate passed within them).');
});
