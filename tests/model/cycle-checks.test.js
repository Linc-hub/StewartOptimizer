import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { cycleRangeWarnings, describeCycle } from '../../src/model/cycle-checks.js';
import { parseRequirements } from '../../src/model/requirements.js';

const sample = parseRequirements(readFileSync(new URL('../../examples/sample-requirements.json', import.meta.url), 'utf8')).normalized;

test('the legacy cycle is described as peak-to-peak and centred on home', () => {
  assert.equal(describeCycle(sample),
    'cycle_mm is peak-to-peak and centred on home. Cycle at 2 Hz: Z 30 mm peak-to-peak (±15 mm about home).');
  assert.deepEqual(cycleRangeWarnings(sample), [], 'the sample cycle stays inside z_range_mm [-20, 40]');
});

test('a cycle that swings past its workspace range is flagged, on the side it exceeds', () => {
  const warnings = cycleRangeWarnings({ ...sample, cycle_mm: 50 });
  assert.equal(warnings.length, 1);
  assert.deepEqual({ axis: warnings[0].axis, amplitude: warnings[0].amplitude, key: warnings[0].key },
    { axis: 'z', amplitude: 25, key: 'z_range_mm' });
  assert.match(warnings[0].message, /Z cycle swings ±25 mm about home, beyond z_range_mm \[-20, 40\]/);
  assert.match(warnings[0].message, /does not widen the workspace/);
  // Exactly at the range edge is inside.
  assert.deepEqual(cycleRangeWarnings({ ...sample, cycle_mm: 40 }), []);
});

test('a multi-axis trajectory is described and checked per axis, rotations in degrees', () => {
  const requirements = { ...sample, trajectory: { frequency_hz: 1.5, components: [
    { axis: 'x', amplitude_mm: 10 }, { axis: 'rz', amplitude_deg: 9 }, { axis: 'y', amplitude_mm: 0 }] } };
  assert.equal(describeCycle(requirements),
    'Cycle at 1.5 Hz: X 20 mm peak-to-peak (±10 mm about home), RZ 18° peak-to-peak (±9° about home).');
  const warnings = cycleRangeWarnings(requirements);
  assert.deepEqual(warnings.map(warning => warning.key), ['rz_range_deg']);
  assert.match(warnings[0].message, /RZ cycle swings ±9° about home, beyond rz_range_deg \[-8, 8\]/);
});

test('a stationary payload has no cycle and no warnings', () => {
  assert.equal(describeCycle({ ...sample, frequency_hz: 0 }), 'No motion cycle: the payload is held at home.');
  assert.deepEqual(cycleRangeWarnings({ ...sample, cycle_mm: 500, frequency_hz: 0 }), []);
});
