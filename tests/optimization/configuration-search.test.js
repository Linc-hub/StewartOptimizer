import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { DEFAULT_DESIGN_SPACE, createRandomLayout } from '../../src/optimization/layout-operators.js';
import { CONFIGURATION_SHARE, configurationKey, configurationSummary, describeConfigurationSummary,
  layoutConfiguration, retainConfigurations, searchedConfigurations } from '../../src/optimization/configurations.js';
import { createRandom } from '../../src/optimization/random.js';

const RUN = { seed: 9, populationSize: 8, generations: 2, ranges: {}, sampling: { strategy: 'grid' } };
const keyOf = evaluation => configurationKey(layoutConfiguration(evaluation.layout));
const fake = (horn_direction, leg_pairing, coverage = 50) => ({ coverage, feasibility: { passing: true },
  layout: { topology: 'c3_paired', topologyParameters: { horn_direction, leg_pairing } } });

test('a run searches every combination its C3 modes allow', () => {
  assert.deepEqual(searchedConfigurations({ topology: 'c3_paired' }), [{ hornDirection: 'outward', legPairing: 'triangulated' }]);
  assert.equal(searchedConfigurations({ topology: 'c3_paired', hornDirection: 'both' }).length, 2);
  assert.deepEqual(searchedConfigurations({ topology: 'c3_paired', hornDirection: 'both', legPairing: 'both' }).map(configurationKey),
    ['outward/triangulated', 'outward/parallel', 'inward/triangulated', 'inward/parallel']);
  assert.deepEqual(searchedConfigurations({ topology: 'circular', hornDirection: 'both' }), []);
  assert.equal(layoutConfiguration({ topology: 'free' }), null);
  assert.equal(configurationKey(layoutConfiguration({ topology: 'c3_paired', topologyParameters: {} })), 'outward/triangulated');
});

test('retention swaps a crowded-out configuration back in for the lowest-ranked surplus survivor', () => {
  const configurations = searchedConfigurations({ topology: 'c3_paired', legPairing: 'both' });
  const triangles = Array.from({ length: 6 }, (_, i) => fake('outward', 'triangulated', 90 - i));
  const parallels = [fake('outward', 'parallel', 40), fake('outward', 'parallel', 30)];
  const quota = Math.floor(6 * CONFIGURATION_SHARE / 2);
  assert.equal(quota, 1);
  const kept = retainConfigurations(triangles, [...triangles, ...parallels], configurations, 6);
  assert.equal(kept.length, 6);
  assert.equal(kept.filter(evaluation => keyOf(evaluation) === 'outward/parallel').length, quota);
  assert.equal(kept[5], parallels[0], 'the best parallel layout replaces the last survivor');
  assert.deepEqual(kept.slice(0, 5), triangles.slice(0, 5));
  // The protected reference is never the one replaced.
  const protectedKept = retainConfigurations(triangles, [...triangles, ...parallels], configurations, 6, triangles[5]);
  assert.ok(protectedKept.includes(triangles[5]));
  assert.ok(!protectedKept.includes(triangles[4]));
  // A single-configuration run is untouched.
  assert.equal(retainConfigurations(triangles, triangles, [configurations[0]], 6), triangles);
  // A configuration with no evaluated members cannot be retained, and nothing else is lost.
  assert.deepEqual(retainConfigurations(triangles, triangles, configurations, 6), triangles);
});

test('fresh layouts are dealt out evenly across the searched configurations', async () => {
  let first;
  const optimizer = new Optimizer({}, { ...RUN, generations: 1, topology: 'c3_paired', hornDirection: 'both', legPairing: 'both',
    onCheckpoint: checkpoint => { first ??= checkpoint.fitness.map(keyOf); } });
  await optimizer.run();
  const counts = {};
  for (const key of first) counts[key] = (counts[key] ?? 0) + 1;
  assert.deepEqual(counts, { 'outward/triangulated': 2, 'outward/parallel': 2, 'inward/triangulated': 2, 'inward/parallel': 2 });
});

test('a multi-configuration run keeps every configuration and reports a per-configuration summary', async () => {
  const optimizer = new Optimizer({}, { ...RUN, generations: 3, topology: 'c3_paired', hornDirection: 'outward', legPairing: 'both' });
  await optimizer.run();
  const quota = Math.floor(RUN.populationSize * CONFIGURATION_SHARE / 2);
  const summary = optimizer.configurationSummary();
  assert.deepEqual(summary.map(row => row.configuration), ['outward/triangulated', 'outward/parallel']);
  for (const row of summary) {
    assert.ok(row.retained >= quota, `${row.configuration} keeps at least ${quota}: ${row.retained}`);
    assert.ok(row.feasible <= row.retained);
  }
  assert.equal(summary.reduce((total, row) => total + row.retained, 0), RUN.populationSize);
  const exported = JSON.parse(optimizer.exportBest());
  assert.deepEqual(exported.run.configuration_summary, summary);
  assert.match(describeConfigurationSummary(summary), /^outward\/triangulated \d+ kept, \d+ feasible/);
  // Replay reproduces the same configuration search.
  const replay = Optimizer.fromReplay(exported);
  await replay.run();
  assert.deepEqual(replay.configurationSummary(), summary);
});

test('a single-configuration run has no summary and exports what it always did', async () => {
  const optimizer = new Optimizer({}, { ...RUN, generations: 1, topology: 'c3_paired' });
  await optimizer.run();
  assert.equal(optimizer.configurationSummary(), null);
  assert.equal('configuration_summary' in JSON.parse(optimizer.exportBest()).run, false);
});

test('new C3 layouts draw the horn offset over the full +/-90 degrees in every mode', () => {
  const random = createRandom(17);
  const context = { designSpace: DEFAULT_DESIGN_SPACE, servoRangeRad: [-2, 2], topology: 'c3_paired', random };
  const offsets = Array.from({ length: 60 }, () => createRandomLayout(context).topologyParameters.beta_offset);
  assert.ok(offsets.every(offset => Math.abs(offset) <= Math.PI / 2 + 1e-12));
  assert.ok(Math.max(...offsets.map(Math.abs)) > Math.PI / 4, 'outward offsets now reach past the old 20 degree jitter');
  const circular = Array.from({ length: 60 }, () => createRandomLayout({ ...context, topology: 'circular' }).topologyParameters.beta_offset);
  assert.ok(circular.every(offset => Math.abs(offset) <= DEFAULT_DESIGN_SPACE.betaJitterRad + 1e-12), 'other topologies keep the jitter');
});

test('the summary counts feasible members and the best coverage per configuration', () => {
  const configurations = searchedConfigurations({ topology: 'c3_paired', hornDirection: 'both' });
  const failing = { ...fake('inward', undefined, 99), feasibility: { passing: false, failedCategories: ['workspace'] } };
  const summary = configurationSummary([fake('outward', undefined, 70), fake('outward', undefined, 80), failing], configurations);
  assert.deepEqual(summary.map(({ configuration, retained, feasible, bestCoverage }) => [configuration, retained, feasible, bestCoverage]),
    [['outward/triangulated', 2, 2, 80], ['inward/triangulated', 1, 0, 99]]);
});
