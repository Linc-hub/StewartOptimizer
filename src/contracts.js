// Shared names for the implementation streams tracked by issue #36.
// Internal metric values use these camel-case keys. JSON preserves the existing
// snake-case metadata names for compatibility.
export const SCHEMA_VERSION = 2;
export const MODEL_VERSION = 2;
// Cycle dynamics identity, separate from the layout/joint MODEL_VERSION.
export const CYCLE_MODEL_VERSION = 'cycle-newton-euler-v1';

export const TOPOLOGIES = Object.freeze(['circular', 'c3_paired', 'rectangular_paired', 'free']);
export const DEFAULT_TOPOLOGY = 'c3_paired';
// Shared ball-joint limit used when neither an option nor the requirements supply one.
export const DEFAULT_BALL_JOINT_LIMIT_DEG = 45;
// Minimum distance (mm) between the centre lines of two legs' horns and rods,
// standing for the sum of their half-widths: a 4 mm rod beside an 8 mm horn arm.
export const DEFAULT_LINK_CLEARANCE_MM = 6;

export const METRICS = Object.freeze({
  coverage: { json: 'coverage', direction: 'max', unit: 'percent' },
  relaxedCoverage: { json: 'relaxed_coverage', direction: 'max', unit: 'percent' },
  payloadCoverage: { json: 'payload_coverage', direction: 'max', unit: 'percent' },
  conditioningQuality: { json: 'conditioning_quality', direction: 'max', unit: 'ratio' },
  dexterity: { json: 'dexterity', direction: 'max', unit: 'ratio' },
  stiffness: { json: 'stiffness', direction: 'max', unit: 'proxy' },
  physicalStiffness: { json: 'physical_stiffness', direction: 'max', unit: 'N/m' },
  torque: { json: 'torque', direction: 'min', unit: 'N m' },
  speedDemand: { json: 'speed_demand', direction: 'min', unit: 'rad/s' },
  loadBalance: { json: 'load_balance', direction: 'max', unit: 'proxy' },
  loadSharing: { json: 'load_sharing', direction: 'max', unit: 'ratio' },
  isotropy: { json: 'isotropy', direction: 'max', unit: 'ratio' },
  limitMargin: { json: 'limit_margin', direction: 'max', unit: 'ratio' },
  fatigue: { json: 'fatigue', direction: 'min', unit: 'rad/s' },
  footprint: { json: 'footprint', direction: 'min', unit: 'mm' },
});

export const FAILURE_CATEGORIES = Object.freeze([
  'geometry', 'home', 'workspace', 'cycle', 'cycle_convergence', 'joint', 'conditioning', 'servo_capacity',
  'payload_support', 'collision',
]);
