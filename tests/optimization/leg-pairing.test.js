import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE, LEG_PAIRING_MODES, createRandomLayout, crossoverLayouts, finalizeLayout,
  mutateLayout } from '../../src/optimization/layout-operators.js';
import { C3_LEG_PAIRINGS, c3LegPairing, topologyGeometry, validateTopology, wrapAngle } from '../../src/optimization/topology.js';
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
const angleOf = ([x, y]) => Math.atan2(y, x);

function c3Layout(parameters = PARAMETERS) {
  return { topology: 'c3_paired', topologyParameters: structuredClone(parameters),
    ...topologyGeometry('c3_paired', parameters), hornLength: 50, rodLength: 200, homeHeight: 180, servoRangeRad: [-2, 2] };
}

// Counts the draws a random source serves, so a test can prove an operator
// consumed exactly the stream it did before leg pairing existed.
function counted(seed) {
  const random = createRandom(seed);
  const wrapped = () => { wrapped.calls++; return random(); };
  wrapped.calls = 0;
  return wrapped;
}

test('triangulated legs part to neighbouring platform pairs and parallel legs rise to the pair above', () => {
  const triangulated = topologyGeometry('c3_paired', PARAMETERS);
  const parallel = topologyGeometry('c3_paired', { ...PARAMETERS, leg_pairing: 'parallel' });
  assert.deepEqual(parallel.baseAnchors, triangulated.baseAnchors, 'the base does not depend on the pairing');
  assert.deepEqual(parallel.betaAngles, triangulated.betaAngles, 'nor do the horns');
  for (let pair = 0; pair < 3; pair++) {
    const axis = PARAMETERS.base_orientation + pair * deg(120);
    for (const leg of [2 * pair, 2 * pair + 1]) {
      const side = leg % 2 ? 1 : -1;
      // Parallel: the platform anchor sits on the base pair's own axis, on the same side as its base anchor.
      const offset = wrapAngle(angleOf(parallel.platformAnchors[leg]) - axis);
      assert.ok(Math.sign(offset) === side && Math.abs(offset) < deg(30), `leg ${leg + 1} rises to the pair above`);
      // Triangulated: it leans 60 degrees toward the neighbouring platform pair.
      const lean = wrapAngle(angleOf(triangulated.platformAnchors[leg]) - axis);
      assert.ok(Math.sign(lean) === side && Math.abs(lean) > deg(30), `leg ${leg + 1} leans to a neighbouring pair`);
    }
    // The platform pair gap is the same chord in both pairings.
    const chord = anchors => Math.hypot(anchors[2 * pair][0] - anchors[2 * pair + 1][0], anchors[2 * pair][1] - anchors[2 * pair + 1][1]);
    assert.ok(Math.abs(chord(parallel.platformAnchors) - PARAMETERS.platform_pair_gap) < 1e-9);
  }
  // A missing pairing is triangulated, so earlier layouts are unchanged.
  assert.deepEqual(topologyGeometry('c3_paired', { ...PARAMETERS, leg_pairing: 'triangulated' }), triangulated);
  assert.equal(c3LegPairing({}), 'triangulated');
  assert.deepEqual(C3_LEG_PAIRINGS, ['triangulated', 'parallel']);
  assert.throws(() => topologyGeometry('c3_paired', { ...PARAMETERS, leg_pairing: 'crossed' }),
    /leg_pairing must be one of triangulated, parallel/);
});

test('a parallel layout validates, exports and re-imports with its pairing', () => {
  const layout = c3Layout({ ...PARAMETERS, leg_pairing: 'parallel', horn_direction: 'inward', beta_offset: deg(30) });
  assert.equal(validateTopology(layout), 'c3_paired');
  const json = layoutToJSON({ ...layout, id: 3 });
  assert.equal(json.topology_parameters.leg_pairing, 'parallel');
  const { layout: imported } = importLayout(json);
  assert.equal(imported.topologyParameters.leg_pairing, 'parallel');
  assert.equal(imported.topologyParameters.horn_direction, 'inward');
  // Triangulated platform anchors conflict with a declared parallel pairing.
  const wrong = { ...layout, platformAnchors: topologyGeometry('c3_paired', PARAMETERS).platformAnchors };
  assert.throws(() => validateTopology(wrong), /platformAnchors\[0\] conflicts/);
});

test('triangulated runs consume the random stream they always have and add no leg_pairing', () => {
  const plain = counted(11), fixed = counted(11);
  const a = createRandomLayout({ ...context, topology: 'c3_paired', random: plain });
  const b = createRandomLayout({ ...context, topology: 'c3_paired', legPairing: 'triangulated', random: fixed });
  assert.deepEqual(b, a);
  assert.equal(fixed.calls, plain.calls);
  assert.equal('leg_pairing' in b.topologyParameters, false);
  const left = counted(5), right = counted(5);
  const mutated = mutateLayout(a, { ...context, legPairing: 'triangulated', random: right });
  assert.deepEqual(mutated, mutateLayout(a, { ...context, random: left }));
  assert.equal(left.calls, right.calls);
  const crossLeft = counted(6), crossRight = counted(6);
  const child = crossoverLayouts(a, mutated, { ...context, legPairing: 'triangulated', random: crossRight });
  assert.deepEqual(child, crossoverLayouts(a, mutated, { ...context, random: crossLeft }));
  assert.equal(crossLeft.calls, crossRight.calls);
  assert.equal('leg_pairing' in child.topologyParameters, false);
});

test('a parallel run makes no extra draws and keeps the horn offset jitter', () => {
  const plain = counted(13), parallel = counted(13);
  const a = createRandomLayout({ ...context, topology: 'c3_paired', random: plain });
  const b = createRandomLayout({ ...context, topology: 'c3_paired', legPairing: 'parallel', random: parallel });
  assert.equal(parallel.calls, plain.calls);
  assert.equal(b.topologyParameters.leg_pairing, 'parallel');
  assert.equal(b.topologyParameters.beta_offset, a.topologyParameters.beta_offset);
  assert.equal(b.homeHeight, a.homeHeight);
});

test('both mode draws, flips and inherits the pairing; fixed modes hold it', () => {
  const random = createRandom(31);
  const drawn = Array.from({ length: 40 }, () => createRandomLayout({ ...context, topology: 'c3_paired', legPairing: 'both', random }));
  assert.deepEqual([...new Set(drawn.map(layout => layout.topologyParameters.leg_pairing))].sort(), ['parallel', 'triangulated']);
  const circular = createRandomLayout({ ...context, topology: 'circular', legPairing: 'parallel', random });
  assert.equal('leg_pairing' in circular.topologyParameters, false, 'other topologies ignore the mode');

  const parallel = c3Layout({ ...PARAMETERS, leg_pairing: 'parallel' });
  const triangulated = c3Layout({ ...PARAMETERS, leg_pairing: 'triangulated' });
  assert.equal(finalizeLayout(structuredClone(triangulated), { ...context, legPairing: 'parallel' }).topologyParameters.leg_pairing, 'parallel');
  const held = finalizeLayout(structuredClone(parallel), { ...context, legPairing: 'triangulated' });
  assert.equal(held.topologyParameters.leg_pairing, 'triangulated');
  assert.deepEqual(held.platformAnchors, topologyGeometry('c3_paired', PARAMETERS).platformAnchors);
  assert.equal(finalizeLayout(structuredClone(parallel), context).topologyParameters.leg_pairing, 'parallel',
    'without a mode a layout keeps its own pairing');

  const flips = mode => Array.from({ length: 300 }, () => mutateLayout(parallel, { ...context, legPairing: mode, random }))
    .filter(layout => layout.topologyParameters.leg_pairing === 'triangulated').length;
  assert.equal(flips('parallel'), 0);
  const flipped = flips('both');
  assert.ok(flipped > 10 && flipped < 60, `about one in ten mutations flips: ${flipped}`);

  const children = Array.from({ length: 60 }, () => crossoverLayouts(parallel, triangulated, { ...context, legPairing: 'both', random }));
  assert.deepEqual([...new Set(children.map(layout => layout.topologyParameters.leg_pairing))].sort(), ['parallel', 'triangulated']);
  assert.deepEqual([...LEG_PAIRING_MODES], ['triangulated', 'parallel', 'both']);
});

test('searching both pairings and both horn directions keeps the two choices independent', () => {
  const random = createRandom(41);
  const layouts = Array.from({ length: 60 }, () => createRandomLayout({ ...context, topology: 'c3_paired',
    hornDirection: 'both', legPairing: 'both', random }));
  const combos = new Set(layouts.map(({ topologyParameters: p }) => `${p.horn_direction}/${p.leg_pairing}`));
  assert.equal(combos.size, 4, `every combination appears: ${[...combos].join(', ')}`);
});

const RUN = { seed: 4, populationSize: 4, generations: 1, ranges: {}, sampling: { strategy: 'grid' } };

test('the optimizer validates the pairing mode, records it and replays a parallel run', async () => {
  assert.throws(() => new Optimizer({}, { ...RUN, topology: 'c3_paired', legPairing: 'crossed' }),
    /legPairing must be one of triangulated, parallel, both/);
  assert.throws(() => new Optimizer({}, { ...RUN, topology: 'circular', legPairing: 'parallel' }),
    /legPairing applies to c3_paired layouts only/);
  assert.equal(new Optimizer({}, RUN).legPairing, 'triangulated');
  const optimizer = new Optimizer({}, { ...RUN, topology: 'c3_paired', legPairing: 'parallel' });
  await optimizer.run();
  assert.ok(optimizer.fitness.every(candidate => candidate.layout.topologyParameters.leg_pairing === 'parallel'));
  assert.equal(optimizer.effectiveSettings().legPairing, 'parallel');
  const replay = Optimizer.fromReplay({ ...layoutToJSON(optimizer.getSelectedCandidate().layout),
    run: { effective_settings: optimizer.effectiveSettings() } });
  assert.equal(replay.legPairing, 'parallel');
  await replay.run();
  assert.deepEqual(replay.fitness.map(candidate => layoutToJSON(candidate.layout)),
    optimizer.fitness.map(candidate => layoutToJSON(candidate.layout)));
  // A run saved before leg pairing existed replays triangulated.
  const { legPairing, ...older } = optimizer.effectiveSettings();
  assert.equal(legPairing, 'parallel');
  assert.equal(Optimizer.fromReplay({ ...layoutToJSON(optimizer.getSelectedCandidate().layout),
    run: { effective_settings: older } }).legPairing, 'triangulated');
});

test('a C3 reference keeps its leg pairing unless the run searches both', () => {
  const reference = layoutToJSON({ ...c3Layout({ ...PARAMETERS, leg_pairing: 'parallel' }), id: 1 });
  assert.equal(new Optimizer({}, { ...RUN, referenceLayout: reference }).legPairing, 'parallel');
  assert.equal(new Optimizer({}, { ...RUN, referenceLayout: reference, legPairing: 'both' }).legPairing, 'both');
  const plain = layoutToJSON({ ...c3Layout(), id: 1 });
  assert.equal(new Optimizer({}, { ...RUN, referenceLayout: plain, legPairing: 'parallel' }).legPairing, 'triangulated');
  const circular = createRandomLayout({ ...context, topology: 'circular', random: createRandom(2) });
  assert.equal(new Optimizer({}, { ...RUN, referenceLayout: layoutToJSON({ ...circular, id: 1 }), legPairing: 'parallel' }).legPairing,
    'triangulated');
});

test('the simulator geometry panel switches a C3 layout between triangulated and parallel legs', () => {
  const source = c3Layout();
  const edited = editGeometry(source, { type: 'legPairing', value: 'parallel' });
  assert.equal(edited.topologyParameters.leg_pairing, 'parallel');
  assert.deepEqual(edited.platformAnchors, topologyGeometry('c3_paired', { ...PARAMETERS, leg_pairing: 'parallel' }).platformAnchors);
  assert.throws(() => editGeometry(source, { type: 'legPairing', value: 'crossed' }), /must be one of triangulated, parallel/);
  const circular = createRandomLayout({ ...context, topology: 'circular', random: createRandom(2) });
  assert.throws(() => editGeometry(circular, { type: 'legPairing', value: 'parallel' }), /c3_paired parameter/);

  const document = createFakeDocument();
  const controller = createSimulatorController();
  controller.loadLayout(source, { options: { ballJointLimitDeg: 180 } });
  createGeometryControls({ document, container: document.createElement('div'), controller });
  const select = document.getElementById('sim-leg-pairing');
  assert.equal(select.value, 'triangulated');
  select.value = 'parallel';
  select.dispatch('change');
  assert.equal(controller.getState().layout.topologyParameters.leg_pairing, 'parallel');
  assert.equal(document.getElementById('sim-leg-pairing').value, 'parallel');
  select.value = 'triangulated';
  select.dispatch('change');
  assert.deepEqual(controller.getState().layout.platformAnchors, source.platformAnchors, 'switching back restores the triangles');
});
