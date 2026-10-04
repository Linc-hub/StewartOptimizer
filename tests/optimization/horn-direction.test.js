import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE, HORN_DIRECTION_MODES, createRandomLayout, crossoverLayouts, finalizeLayout,
  mutateLayout } from '../../src/optimization/layout-operators.js';
import { C3_HORN_DIRECTIONS, c3HornDirection, topologyGeometry, validateTopology, wrapAngle } from '../../src/optimization/topology.js';
import { createRandom } from '../../src/optimization/random.js';
import { layoutToJSON } from '../../src/io/results.js';
import { importLayout } from '../../src/io/layout-import.js';
import { editGeometry } from '../../src/simulator/geometry-editor.js';
import { createSimulatorController } from '../../src/simulator/controller.js';
import { createGeometryControls } from '../../src/simulator/geometry-controls.js';
import { createFakeDocument } from '../simulator/helpers.js';

const deg = value => value * Math.PI / 180;
const PARAMETERS = { base_radius: 120, platform_radius: 70, base_orientation: deg(10), beta_offset: 0,
  base_pair_gap: 30, platform_pair_gap: 25 };
const context = { designSpace: DEFAULT_DESIGN_SPACE, servoRangeRad: [-2, 2] };

function c3Layout(parameters = PARAMETERS) {
  return { topology: 'c3_paired', topologyParameters: structuredClone(parameters),
    ...topologyGeometry('c3_paired', parameters), hornLength: 50, rodLength: 200, homeHeight: 180, servoRangeRad: [-2, 2] };
}

// Counts the draws a random source serves, so a test can prove an operator
// consumed exactly the stream it did before horn directions existed.
function counted(seed) {
  const random = createRandom(seed);
  const wrapped = () => { wrapped.calls++; return random(); };
  wrapped.calls = 0;
  return wrapped;
}

test('outward C3 horns point away from their pair partner and inward horns toward it', () => {
  const outward = topologyGeometry('c3_paired', PARAMETERS);
  const inward = topologyGeometry('c3_paired', { ...PARAMETERS, horn_direction: 'inward' });
  assert.deepEqual(inward.baseAnchors, outward.baseAnchors, 'the anchors do not depend on the horn direction');
  assert.deepEqual(inward.platformAnchors, outward.platformAnchors);
  for (let leg = 0; leg < 6; leg++) {
    const partner = leg ^ 1;
    const toPartner = [0, 1].map(k => outward.baseAnchors[partner][k] - outward.baseAnchors[leg][k]);
    const along = beta => Math.cos(beta) * toPartner[0] + Math.sin(beta) * toPartner[1];
    assert.ok(along(outward.betaAngles[leg]) < 0, `leg ${leg + 1} outward horn points away from its partner`);
    assert.ok(along(inward.betaAngles[leg]) > 0, `leg ${leg + 1} inward horn points toward its partner`);
    // At zero offset both are tangent: the inward horn is the outward one turned half a circle.
    assert.ok(Math.abs(wrapAngle(inward.betaAngles[leg] - outward.betaAngles[leg] - Math.PI)) < 1e-12);
  }
  // A missing direction is outward, so earlier layouts are unchanged.
  assert.deepEqual(topologyGeometry('c3_paired', { ...PARAMETERS, horn_direction: 'outward' }), outward);
  assert.equal(c3HornDirection({}), 'outward');
  assert.deepEqual(C3_HORN_DIRECTIONS, ['outward', 'inward']);
  assert.throws(() => topologyGeometry('c3_paired', { ...PARAMETERS, horn_direction: 'sideways' }),
    /horn_direction must be one of outward, inward/);
});

test('the two horn directions meet at a radial horn at the +/-90 degree offset limits', () => {
  for (const [offset, radialTurn] of [[90, Math.PI], [-90, 0]]) {
    const parameters = { ...PARAMETERS, beta_offset: deg(offset) };
    const outward = topologyGeometry('c3_paired', parameters);
    const inward = topologyGeometry('c3_paired', { ...parameters, horn_direction: 'inward' });
    outward.betaAngles.forEach((beta, leg) => {
      const [x, y] = outward.baseAnchors[leg];
      assert.ok(Math.abs(wrapAngle(beta - inward.betaAngles[leg])) < 1e-12, `leg ${leg + 1} at ${offset}°`);
      // +90° points at the plate centre and -90° straight away from it.
      assert.ok(Math.abs(wrapAngle(beta - Math.atan2(y, x) - radialTurn)) < 1e-12, `leg ${leg + 1} radial at ${offset}°`);
    });
  }
});

test('an inward layout validates, exports and re-imports with its horn direction', () => {
  const layout = c3Layout({ ...PARAMETERS, horn_direction: 'inward', beta_offset: deg(40) });
  assert.equal(validateTopology(layout), 'c3_paired');
  const json = layoutToJSON({ ...layout, id: 7 });
  assert.equal(json.topology_parameters.horn_direction, 'inward');
  const { layout: imported } = importLayout(json);
  assert.equal(imported.topologyParameters.horn_direction, 'inward');
  imported.betaAngles.forEach((beta, leg) => assert.ok(Math.abs(wrapAngle(beta - layout.betaAngles[leg])) < 1e-9));
  // Angles that only an outward layout produces conflict with a declared inward direction.
  const outwardAngles = { ...layout, betaAngles: topologyGeometry('c3_paired', { ...PARAMETERS, beta_offset: deg(40) }).betaAngles };
  assert.throws(() => validateTopology(outwardAngles), /betaAngles\[0\] conflicts/);
});

test('outward runs consume the random stream they always have and add no horn_direction', () => {
  const plain = counted(11), outward = counted(11);
  const a = createRandomLayout({ ...context, topology: 'c3_paired', random: plain });
  const b = createRandomLayout({ ...context, topology: 'c3_paired', hornDirection: 'outward', random: outward });
  assert.deepEqual(b, a);
  assert.equal(outward.calls, plain.calls);
  assert.equal('horn_direction' in b.topologyParameters, false);
  // The same seeded mutation and crossover with and without the outward mode
  // are identical, draw for draw, and the children carry no horn_direction.
  const left = counted(5), right = counted(5);
  const mutated = mutateLayout(a, { ...context, hornDirection: 'outward', random: right });
  assert.deepEqual(mutated, mutateLayout(a, { ...context, random: left }));
  assert.equal(left.calls, right.calls);
  assert.equal('horn_direction' in mutated.topologyParameters, false);
  const crossLeft = counted(6), crossRight = counted(6);
  const child = crossoverLayouts(a, mutated, { ...context, hornDirection: 'outward', random: crossRight });
  assert.deepEqual(child, crossoverLayouts(a, mutated, { ...context, random: crossLeft }));
  assert.equal(crossLeft.calls, crossRight.calls);
  assert.equal('horn_direction' in child.topologyParameters, false);
});

test('inward and both runs set the direction and draw the offset over the full +/-90 degrees', () => {
  const random = createRandom(21);
  const inward = Array.from({ length: 40 }, () => createRandomLayout({ ...context, topology: 'c3_paired', hornDirection: 'inward', random }));
  assert.ok(inward.every(layout => layout.topologyParameters.horn_direction === 'inward'));
  const offsets = inward.map(layout => Math.abs(layout.topologyParameters.beta_offset));
  assert.ok(offsets.every(offset => offset <= Math.PI / 2 + 1e-12));
  assert.ok(Math.max(...offsets) > deg(45), `offsets reach past the 20° outward jitter: ${Math.max(...offsets)}`);
  const both = Array.from({ length: 40 }, () => createRandomLayout({ ...context, topology: 'c3_paired', hornDirection: 'both', random }));
  const directions = new Set(both.map(layout => layout.topologyParameters.horn_direction));
  assert.deepEqual([...directions].sort(), ['inward', 'outward']);
  // Other topologies ignore the mode and never carry a horn direction.
  const circular = createRandomLayout({ ...context, topology: 'circular', hornDirection: 'inward', random });
  assert.equal('horn_direction' in circular.topologyParameters, false);
});

test('fixed modes hold the direction, both mode flips it by mutation and crossover inherits it', () => {
  const inwardLayout = c3Layout({ ...PARAMETERS, horn_direction: 'inward' });
  const outwardLayout = c3Layout({ ...PARAMETERS, horn_direction: 'outward' });
  assert.equal(finalizeLayout(structuredClone(outwardLayout), { ...context, hornDirection: 'inward' }).topologyParameters.horn_direction, 'inward');
  assert.equal(finalizeLayout(structuredClone(inwardLayout), { ...context, hornDirection: 'outward' }).topologyParameters.horn_direction, 'outward');
  assert.equal(finalizeLayout(structuredClone(inwardLayout), context).topologyParameters.horn_direction, 'inward',
    'without a mode a layout keeps its own direction');
  const finalized = finalizeLayout(structuredClone(inwardLayout), { ...context, hornDirection: 'outward' });
  assert.deepEqual(finalized.betaAngles, topologyGeometry('c3_paired', finalized.topologyParameters).betaAngles,
    'the horn angles follow the enforced direction');

  const random = createRandom(8);
  const flips = mode => Array.from({ length: 300 }, () => mutateLayout(inwardLayout, { ...context, hornDirection: mode, random }))
    .filter(layout => layout.topologyParameters.horn_direction === 'outward').length;
  assert.equal(flips('inward'), 0);
  const flipped = flips('both');
  assert.ok(flipped > 10 && flipped < 60, `about one in ten mutations flips: ${flipped}`);

  const children = Array.from({ length: 60 }, () => crossoverLayouts(inwardLayout, outwardLayout, { ...context, hornDirection: 'both', random }));
  const inherited = new Set(children.map(layout => layout.topologyParameters.horn_direction));
  assert.deepEqual([...inherited].sort(), ['inward', 'outward']);
  const same = crossoverLayouts(inwardLayout, inwardLayout, { ...context, hornDirection: 'both', random });
  assert.equal(same.topologyParameters.horn_direction, 'inward');
  assert.deepEqual([...HORN_DIRECTION_MODES], ['outward', 'inward', 'both']);
});

const RUN = { seed: 4, populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' } };

test('the optimizer validates the mode, records it and replays an inward run', async () => {
  assert.throws(() => new Optimizer({}, { ...RUN, topology: 'c3_paired', hornDirection: 'sideways' }),
    /hornDirection must be one of outward, inward, both/);
  assert.throws(() => new Optimizer({}, { ...RUN, topology: 'circular', hornDirection: 'inward' }),
    /applies to c3_paired layouts only/);
  assert.equal(new Optimizer({}, RUN).hornDirection, 'outward');
  const optimizer = new Optimizer({}, { ...RUN, topology: 'c3_paired', hornDirection: 'inward' });
  await optimizer.run();
  assert.equal(optimizer.fitness.length, RUN.populationSize);
  assert.ok(optimizer.fitness.every(candidate => candidate.layout.topologyParameters.horn_direction === 'inward'));
  assert.equal(optimizer.effectiveSettings().hornDirection, 'inward');
  const replay = Optimizer.fromReplay({ ...layoutToJSON(optimizer.getSelectedCandidate().layout),
    run: { effective_settings: optimizer.effectiveSettings() } });
  assert.equal(replay.hornDirection, 'inward');
  await replay.run();
  assert.deepEqual(replay.fitness.map(candidate => layoutToJSON(candidate.layout)),
    optimizer.fitness.map(candidate => layoutToJSON(candidate.layout)));
});

test('a C3 reference keeps its horn direction unless the run searches both', () => {
  const reference = layoutToJSON({ ...c3Layout({ ...PARAMETERS, horn_direction: 'inward' }), id: 1 });
  assert.equal(new Optimizer({}, { ...RUN, referenceLayout: reference, hornDirection: 'outward' }).hornDirection, 'inward');
  assert.equal(new Optimizer({}, { ...RUN, referenceLayout: reference, hornDirection: 'both' }).hornDirection, 'both');
  const outwardReference = layoutToJSON({ ...c3Layout(), id: 1 });
  assert.equal(new Optimizer({}, { ...RUN, referenceLayout: outwardReference, hornDirection: 'inward' }).hornDirection, 'outward');
  // A reference of another topology has no horn direction to apply.
  const circular = createRandomLayout({ ...context, topology: 'circular', random: createRandom(2) });
  const circularReference = layoutToJSON({ ...circular, id: 1 });
  assert.equal(new Optimizer({}, { ...RUN, referenceLayout: circularReference, hornDirection: 'inward' }).hornDirection, 'outward');
});

test('the simulator geometry panel toggles a C3 layout between outward and inward horns', () => {
  const source = c3Layout();
  const edited = editGeometry(source, { type: 'hornDirection', value: 'inward' });
  assert.equal(edited.topologyParameters.horn_direction, 'inward');
  assert.deepEqual(edited.betaAngles, topologyGeometry('c3_paired', { ...PARAMETERS, horn_direction: 'inward' }).betaAngles);
  assert.deepEqual(edited.baseAnchors, source.baseAnchors);
  assert.throws(() => editGeometry(source, { type: 'hornDirection', value: 'up' }), /must be one of outward, inward/);
  const circular = createRandomLayout({ ...context, topology: 'circular', random: createRandom(2) });
  assert.throws(() => editGeometry(circular, { type: 'hornDirection', value: 'inward' }), /c3_paired parameter/);

  const document = createFakeDocument();
  const controller = createSimulatorController();
  controller.loadLayout(source, { options: { ballJointLimitDeg: 180 } });
  createGeometryControls({ document, container: document.createElement('div'), controller });
  const select = document.getElementById('sim-horn-direction');
  assert.equal(select.value, 'outward');
  select.value = 'inward';
  select.dispatch('change');
  assert.equal(controller.getState().layout.topologyParameters.horn_direction, 'inward');
  assert.equal(select.value, 'inward');
  select.value = 'outward';
  select.dispatch('change');
  assert.deepEqual(controller.getState().layout.betaAngles, source.betaAngles, 'toggling back restores the outward horns');
});
