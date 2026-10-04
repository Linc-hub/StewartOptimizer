import { readFileSync } from 'node:fs';
import test from 'node:test';
import assert from 'node:assert/strict';
import { Optimizer } from '../../src/optimization/optimizer.js';
import { resolveMounting } from '../../src/model/mounting.js';
import { asymmetricJointFixture } from '../fixtures/layout.js';
import { loadUI } from './helpers.js';

// A subclass keeps the app on the headless path; the default class selects the worker adapter.
class HeadlessOptimizer extends Optimizer {}

// Page actions that no other test touched: Clear Requirements, Copy JSON, the
// CSV and Fusion download formats, the simulator download, and the remaining
// workspace range and per-servo rating inputs.

function stubOptimizer(capture) {
  return class StubOptimizer {
    constructor(requirements, options) { capture({ requirements, options }); this.pareto = []; this.fitness = []; }
    estimateWork() { return { totalPoses: 8 }; }
    start(done) { done?.(); return Promise.resolve({ status: 'completed' }); }
  };
}

test('every workspace range input and per-servo rating reaches the optimizer options', async () => {
  let captured;
  const element = await loadUI(stubOptimizer(value => { captured = value; }));
  const ranges = { X: [-11, 12, 1], Y: [-13, 14, 3], Z: [-15, 16, 4], Rx: [-5, 6, 2], Ry: [-7, 8, 5], Rz: [-9, 10, 19] };
  for (const [prefix, [min, max, step]] of Object.entries(ranges)) {
    element(`opt${prefix}Min`).value = String(min);
    element(`opt${prefix}Max`).value = String(max);
    element(`opt${prefix}Step`).value = String(step);
  }
  element('servoSpeedRating').value = '400';
  element('servoContinuousTorqueRating').value = '0.9';
  for (let servo = 1; servo <= 6; servo++) {
    element(`servoTorque${servo}`).value = String(servo);
    element(`servoSpeed${servo}`).value = String(100 * servo);
  }
  await element('runOptimization').handlers.click();
  for (const [prefix, [min, max, step]] of Object.entries(ranges)) {
    assert.deepEqual(captured.options.ranges[prefix.toLowerCase()], { min, max, step }, `${prefix} range`);
  }
  const ratings = captured.options.servoRatings;
  assert.equal(ratings.servo_speed_rating_deg_s, 400);
  assert.equal(ratings.servo_continuous_torque_rating_nm, 0.9);
  assert.deepEqual(ratings.per_servo_ratings, Array.from({ length: 6 }, (_, i) => ({ torque_nm: i + 1, speed_deg_s: 100 * (i + 1) })));
  element('servoSpeed3').value = '-1';
  await element('runOptimization').handlers.click();
  assert.match(element('optStatus').textContent, /Servo 3 speed rating must be a finite positive number/);
});

test('the C3 horn direction control shows only for C3 and reaches the optimizer', async () => {
  let captured;
  const element = await loadUI(stubOptimizer(value => { captured = value; }));
  element('optTopology').value = 'c3_paired';
  element('optTopology').handlers.change();
  assert.equal(element('optHornDirectionField').hidden, false);
  element('optHornDirection').value = 'both';
  await element('runOptimization').handlers.click();
  assert.equal(captured.options.topology, 'c3_paired');
  assert.equal(captured.options.hornDirection, 'both');
  // Another topology hides the control and runs outward whatever it still shows.
  element('optTopology').value = 'circular';
  element('optTopology').handlers.change();
  assert.equal(element('optHornDirectionField').hidden, true);
  await element('runOptimization').handlers.click();
  assert.equal(captured.options.hornDirection, 'outward');
});

test('the requirements cycle note explains the cycle and warns when it leaves the workspace', async () => {
  const element = await loadUI(stubOptimizer(() => {}));
  const requirements = JSON.parse(readFileSync(new URL('../../examples/sample-requirements.json', import.meta.url), 'utf8'));
  element('requirementsInput').value = JSON.stringify(requirements);
  element('requirementsInput').handlers.change();
  assert.match(element('requirementsCycleNote').textContent, /cycle_mm is peak-to-peak and centred on home\. Cycle at 2 Hz: Z 30 mm/);
  requirements.payload.cycle_mm = 60;
  element('requirementsInput').value = JSON.stringify(requirements);
  element('requirementsInput').handlers.change();
  assert.match(element('requirementsCycleNote').textContent, /Z cycle swings ±30 mm about home, beyond z_range_mm/);
  // Text that does not parse yet keeps the last note.
  element('requirementsInput').value = '{';
  element('requirementsInput').handlers.change();
  assert.match(element('requirementsCycleNote').textContent, /swings ±30 mm/);
});

test('Clear Requirements empties the inputs and results and disables copy and simulator download', async () => {
  let constructed = 0;
  const element = await loadUI(stubOptimizer(() => { constructed++; }));
  await element('runOptimization').handlers.click();
  assert.equal(constructed, 1);
  element('clearRequirements').handlers.click();
  assert.equal(element('requirementsInput').value, '');
  assert.equal(element('resultOutput').value, '');
  assert.equal(element('copyResultOutput').disabled, true);
  assert.equal(element('simDownload').disabled, true);
  assert.equal(element('optStatus').textContent, 'Requirements cleared.');
  await element('runOptimization').handlers.click();
  assert.equal(constructed, 1, 'a run with empty requirements is rejected before constructing an optimizer');
  assert.match(element('optStatus').textContent, /JSON|requirements/i);
});

test('Copy JSON reports success through the clipboard API and a blocked copy otherwise', async () => {
  const written = [];
  const window = { addEventListener() {}, navigator: { clipboard: { writeText: async text => { written.push(text); } } } };
  const element = await loadUI(HeadlessOptimizer, { window });
  assert.equal(element('copyResultOutput').disabled, true, 'nothing to copy before a run');
  await element('runOptimization').handlers.click();
  assert.equal(element('copyResultOutput').disabled, false);
  await element('copyResultOutput').handlers.click();
  assert.deepEqual(written, [element('resultOutput').value]);
  assert.ok(JSON.parse(written[0]).result.layout.id >= 1);
  assert.equal(element('copyResultStatus').textContent, 'Copied to clipboard.');
  const blocked = await loadUI(HeadlessOptimizer, { window: { addEventListener() {} } });
  await blocked('runOptimization').handlers.click();
  await blocked('copyResultOutput').handlers.click();
  assert.equal(blocked('copyResultStatus').textContent, 'Copy blocked. Select the JSON and copy it manually.');
});

test('the CSV and Fusion download formats export the construction skeleton of the selected candidate', async () => {
  const downloads = [];
  const element = await loadUI(HeadlessOptimizer, { downloadFile: (data, name, type) => downloads.push({ data, name, type }) });
  // CAD formats need a home pose; seed 3's selected candidate has one.
  element('optSeed').value = '3';
  await element('runOptimization').handlers.click();
  const selectedId = JSON.parse(element('resultOutput').value).result.layout.id;
  element('downloadFormat').value = 'csv';
  element('downloadFormat').handlers.change();
  assert.equal(element('downloadSelected').disabled, false);
  element('downloadSelected').handlers.click();
  assert.equal(downloads.at(-1).name, 'stewart_coordinates.csv');
  assert.equal(downloads.at(-1).type, 'text/csv');
  const rows = downloads.at(-1).data.trim().split('\r\n');
  assert.equal(rows[0], 'candidate_id,name,kind,frame,x_mm,y_mm,z_mm');
  assert.ok(rows.length > 12, 'six legs contribute at least two points each');
  assert.ok(rows.slice(1).every(row => row.startsWith(`"${selectedId}",`)), "every row carries the quoted candidate id");
  element('downloadFormat').value = 'fusion';
  element('downloadFormat').handlers.change();
  element('downloadSelected').handlers.click();
  assert.equal(downloads.at(-1).name, 'stewart_construction.py');
  assert.equal(downloads.at(-1).type, 'text/x-python');
  assert.match(downloads.at(-1).data, /^# Stewart Optimizer construction skeleton/);
  assert.match(downloads.at(-1).data, /import adsk\.core, adsk\.fusion/);
  assert.match(downloads.at(-1).data, new RegExp(`# Candidate ${selectedId};`));
  element('downloadFormat').value = 'json';
  element('downloadFormat').handlers.change();
  element('downloadSelected').handlers.click();
  assert.equal(downloads.at(-1).name, 'optimized_layout.json');
  assert.equal(JSON.parse(downloads.at(-1).data).id, selectedId);
});

test('the simulator download carries the loaded layout, pose state, camera and pointer mode', async () => {
  const source = { ...asymmetricJointFixture(), id: 23, topology: 'free', topologyParameters: {} };
  source.mounting = resolveMounting(source).mounting;
  const candidate = { layout: source, torque: 2, speedDemand: 3, coverage: 100,
    feasibility: { passing: true, homePoseSatisfied: true, sampledWorkspaceSatisfied: true, cycleSatisfied: true } };
  class FixtureOptimizer {
    constructor() { this.fitness = [candidate]; this.pareto = [candidate]; }
    estimateWork() { return { totalPoses: 1 }; }
    async start() { return { status: 'completed' }; }
    getSelectedCandidate() { return candidate; }
    selectCandidate() {}
    effectiveSettings() { return { ballJointLimitDeg: 180, conditionLimit: null, requirements: { mass_kg: 1 }, populationSize: 4, generations: 1 }; }
  }
  const downloads = [];
  const element = await loadUI(FixtureOptimizer, { downloadFile: (data, name, type) => downloads.push({ data, name, type }) });
  await element('runOptimization').handlers.click();
  assert.equal(element('simDownload').disabled, false);
  element('simulateTab').handlers.click();
  element('simXInput').value = '4';
  element('simXInput').handlers.change();
  element('simPointerMode').value = 'platform';
  element('simInputFrame').value = 'platform';
  element('simDownload').handlers.click();
  assert.equal(downloads.length, 1);
  assert.equal(downloads[0].name, 'stewart_simulator.json');
  assert.equal(downloads[0].type, 'application/json');
  const saved = JSON.parse(downloads[0].data);
  assert.equal(saved.id, 23);
  assert.equal(saved.run.effective_settings.populationSize, 4);
  assert.equal(saved.simulator.requested.x, 4);
  assert.equal(saved.simulator.pointerMode, 'platform');
  assert.equal(saved.simulator.inputFrame, 'platform');
  assert.deepEqual(Object.keys(saved.simulator.camera).sort(), ['distance', 'pitch', 'target', 'yaw']);
  assert.equal(saved.simulator.source.candidateId, 23);
});
