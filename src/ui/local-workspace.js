export const LOCAL_WORKSPACE_KEY = 'stewart-optimizer.workspace.v1';

const INPUT_IDS = [
  'requirementsInput', 'referenceLayoutInput', 'optTopology', 'homeHeightMin',
  'homeHeightMax', 'optPopulation', 'optGenerations', 'optObjectiveSet',
  'optMutationRate', 'optSampling', 'optSeed', 'ballJointLimit',
  'servoTorqueRating', 'servoSpeedRating', 'servoRatingPolicy',
  ...['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'].flatMap(axis =>
    ['Min', 'Max', 'Step'].map(suffix => `opt${axis}${suffix}`)),
  ...Array.from({ length: 6 }, (_, index) =>
    [`servoTorque${index + 1}`, `servoSpeed${index + 1}`]).flat(),
];
// Controls added after version 1 was saved; older saves keep the control's current value.
const OPTIONAL_INPUT_IDS = ['optCycleSampling', 'servoContinuousTorqueRating', 'linkClearance', 'optHornDirection'];

export function captureLocalWorkspace(document, simulator = null) {
  return {
    version: 1,
    inputs: Object.fromEntries([...INPUT_IDS, ...OPTIONAL_INPUT_IDS].map(id => [id, document.getElementById(id).value])),
    ballJointClamp: document.getElementById('ballJointClamp').checked,
    simulator,
  };
}

export function parseLocalWorkspace(raw) {
  const saved = JSON.parse(raw);
  if (saved?.version !== 1 || !saved.inputs || typeof saved.inputs !== 'object'
    || INPUT_IDS.some(id => typeof saved.inputs[id] !== 'string')
    || OPTIONAL_INPUT_IDS.some(id => id in saved.inputs && typeof saved.inputs[id] !== 'string')
    || typeof saved.ballJointClamp !== 'boolean'
    || (saved.simulator !== null && (typeof saved.simulator !== 'object' || !saved.simulator))) {
    throw new Error('The local workspace has an unsupported or incomplete format.');
  }
  return saved;
}

export function applyLocalWorkspace(document, saved) {
  for (const id of INPUT_IDS) document.getElementById(id).value = saved.inputs[id];
  for (const id of OPTIONAL_INPUT_IDS) if (id in saved.inputs) document.getElementById(id).value = saved.inputs[id];
  document.getElementById('ballJointClamp').checked = saved.ballJointClamp;
}
