import { isStationary, trajectoryFromRequirements } from './trajectory.js';

const RANGE_KEYS = Object.freeze({ x: 'x_range_mm', y: 'y_range_mm', z: 'z_range_mm',
  rx: 'rx_range_deg', ry: 'ry_range_deg', rz: 'rz_range_deg' });
const unit = component => ('amplitude_mm' in component ? ' mm' : '°');
const amplitudeOf = component => component.amplitude_mm ?? component.amplitude_deg;
const round = value => Number(value.toFixed(3));

// The motion cycle in plain terms. Every amplitude is about the home pose, and
// the legacy cycle_mm is the peak-to-peak stroke, so it reaches cycle_mm / 2
// either side of home.
export function describeCycle(requirements) {
  const { trajectory, source } = trajectoryFromRequirements(requirements);
  if (isStationary(trajectory)) return 'No motion cycle: the payload is held at home.';
  const parts = trajectory.components.filter(component => amplitudeOf(component) > 0).map(component => {
    const amplitude = amplitudeOf(component);
    return `${component.axis.toUpperCase()} ${round(2 * amplitude)}${unit(component)} peak-to-peak `
      + `(±${round(amplitude)}${unit(component)} about home)`;
  });
  const origin = source === 'legacy-cycle' ? 'cycle_mm is peak-to-peak and centred on home. ' : '';
  return `${origin}Cycle at ${round(trajectory.frequency_hz)} Hz: ${parts.join(', ')}.`;
}

// Cycle axes whose swing leaves the matching workspace range. The cycle is
// evaluated on its own path with every modeled limit, but the workspace sweep
// samples only the ranges, so such a cycle asks for motion the coverage figure
// never measured; neither widens the other.
export function cycleRangeWarnings(requirements) {
  const { trajectory } = trajectoryFromRequirements(requirements);
  if (isStationary(trajectory)) return [];
  const warnings = [];
  for (const component of trajectory.components) {
    const amplitude = amplitudeOf(component);
    const key = RANGE_KEYS[component.axis];
    const range = requirements[key];
    if (!(amplitude > 0) || !Array.isArray(range)) continue;
    if (-amplitude >= range[0] - 1e-9 && amplitude <= range[1] + 1e-9) continue;
    warnings.push({ axis: component.axis, amplitude, range: range.slice(), key,
      message: `The ${component.axis.toUpperCase()} cycle swings ±${round(amplitude)}${unit(component)} about home, `
        + `beyond ${key} [${range[0]}, ${range[1]}]. The cycle is still checked on its own path, but the workspace `
        + `sweep and its coverage do not include those poses, and the cycle does not widen the workspace.` });
  }
  return warnings;
}
