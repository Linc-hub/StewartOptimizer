import { METRICS } from '../contracts.js';

export const OBJECTIVE_SETS = Object.freeze({
  compact: Object.freeze(['coverage', 'conditioningQuality', 'torque', 'speedDemand']),
  full: Object.freeze(['coverage', 'conditioningQuality', 'torque', 'speedDemand',
    'dexterity', 'stiffness', 'loadSharing', 'limitMargin', 'fatigue']),
  // Full before solved rod-force load sharing; kept to replay older runs.
  'full-v1': Object.freeze(['coverage', 'conditioningQuality', 'torque', 'speedDemand',
    'dexterity', 'stiffness', 'loadBalance', 'limitMargin', 'fatigue']),
});

const LEGACY_KEYS = ['coverage', 'relaxedCoverage', 'dexterity', 'stiffness',
  'loadBalance', 'isotropy', 'limitMargin', 'torque', 'speedDemand', 'fatigue'];
const APPROXIMATION = Object.freeze({ stiffness: 'geometric proxy',
  physicalStiffness: 'unloaded small-deflection stiffness at home; minimum eigenvalue with rotations scaled by the characteristic length',
  loadBalance: 'legacy directional proxy', loadSharing: 'solved rod-force balance over the cycle',
  limitMargin: 'sampled proxy', fatigue: 'heuristic',
  footprint: 'radius of the Z-axis cylinder holding every horn reach and platform anchor' });

export function normalizeObjectiveSet(input = 'compact') {
  if (Array.isArray(input) && input.length === LEGACY_KEYS.length
      && input.every((key, index) => key === LEGACY_KEYS[index])) return 'legacy-v2';
  if (input === 'legacy-v2' || Object.hasOwn(OBJECTIVE_SETS, input)) return input;
  throw new RangeError('objectiveSet must be compact, full or full-v1.');
}

// `stiffnessMetric: 'physicalStiffness'` substitutes the physical stiffness for the proxy;
// `footprint: true` (the optimizer's compactness option) appends the footprint to any set.
export function objectiveDefinitions(input = 'compact', { stiffnessMetric = 'stiffness', footprint = false } = {}) {
  const name = normalizeObjectiveSet(input);
  if (!['stiffness', 'physicalStiffness'].includes(stiffnessMetric)) throw new RangeError('stiffnessMetric must be stiffness or physicalStiffness.');
  const keys = (name === 'legacy-v2' ? LEGACY_KEYS : OBJECTIVE_SETS[name])
    .map(key => key === 'stiffness' ? stiffnessMetric : key);
  if (footprint) keys.push('footprint');
  return keys.map(key => ({ key, direction: METRICS[key].direction,
    unit: METRICS[key].unit, approximation: APPROXIMATION[key] ?? null }));
}

export function objectiveValues(evaluation, input = 'compact', variant = {}) {
  return objectiveDefinitions(input, variant).map(({ key, direction }) => {
    const value = evaluation[key];
    return Number.isFinite(value) ? direction === 'min' ? -value : value : -Infinity;
  });
}
