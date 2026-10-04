import { DEFAULT_TOPOLOGY, TOPOLOGIES } from '../contracts.js';

export const PAIRED_HORN_TOPOLOGIES = Object.freeze(['circular', 'rectangular_paired']);
export const DEFAULT_BETA_PAIR_OFFSET = Math.PI / 6;

export function wrapAngle(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function point(radius, angle) {
  return [radius * Math.cos(angle), radius * Math.sin(angle), 0];
}

function rotate([x, y, z], angle) {
  return [x * Math.cos(angle) - y * Math.sin(angle),
    x * Math.sin(angle) + y * Math.cos(angle), z];
}

function ring(radius, orientation) {
  return Array.from({ length: 6 }, (_, i) => point(radius, orientation + i * Math.PI / 3));
}

// The classic triangulated hexapod. Base pair k sits on the axis
// base_orientation + k * 120 deg, and its two legs part to the platform pairs
// 60 deg either side, so neighbouring base pairs meet at each platform pair and
// the six legs zig-zag into three triangles. The two servos of a pair are
// mirror images: each horn starts tangent and points away from its partner,
// toward its leg's lean (horn_direction outward, the default), or toward its
// partner (inward), and beta_offset turns both by the same mirrored angle.
function c3PairedGeometry(p) {
  const baseHalf = Math.asin(p.base_pair_gap / (2 * p.base_radius));
  const platformHalf = Math.asin(p.platform_pair_gap / (2 * p.platform_radius));
  const turn = c3HornDirection(p) === 'inward' ? -1 : 1;
  const baseAnchors = [], platformAnchors = [], betaAngles = [];
  for (let i = 0; i < 6; i++) {
    const side = i % 2 ? 1 : -1;
    const axis = p.base_orientation + Math.floor(i / 2) * 2 * Math.PI / 3;
    const baseAngle = axis + side * baseHalf;
    baseAnchors.push(point(p.base_radius, baseAngle));
    platformAnchors.push(point(p.platform_radius, axis + side * (Math.PI / 3 - platformHalf)));
    betaAngles.push(wrapAngle(baseAngle + turn * side * (Math.PI / 2 + p.beta_offset)));
  }
  return { baseAnchors, platformAnchors, betaAngles };
}

// beta_offset is held within +/-90 deg. At the limits both directions point
// the horn radially (+90 deg toward the plate centre, -90 deg away from it), so
// the two horn directions together reach every angle once, continuously.
// Inward horns can cross their partner across the pair gap; the link collision
// check rejects such poses rather than the parametrisation.
export const C3_BETA_OFFSET_LIMIT = Math.PI / 2;
export const C3_HORN_DIRECTIONS = Object.freeze(['outward', 'inward']);
export const DEFAULT_C3_HORN_DIRECTION = 'outward';

// A C3 layout without horn_direction keeps the outward horns every earlier
// layout had.
export function c3HornDirection(parameters) {
  return parameters?.horn_direction ?? DEFAULT_C3_HORN_DIRECTION;
}

function rectangle(radius, aspect, orientation) {
  const halfDepth = radius / Math.hypot(aspect, 1);
  const halfWidth = aspect * halfDepth;
  return [-1, 0, 1].flatMap(row => [-1, 1].map(column =>
    rotate([column * halfWidth, row * halfDepth, 0], orientation)));
}

const COMMON_FIELDS = ['base_radius', 'platform_radius', 'base_orientation', 'platform_orientation', 'beta_offset'];
const TOPOLOGY_FIELDS = Object.freeze({
  circular: Object.freeze([...COMMON_FIELDS, 'beta_pair_offset']),
  // The C3 platform is locked 60 deg from the base, so it has no turn of its own.
  c3_paired: Object.freeze([...COMMON_FIELDS.filter(field => field !== 'platform_orientation'),
    'base_pair_gap', 'platform_pair_gap']),
  rectangular_paired: Object.freeze([...COMMON_FIELDS, 'beta_pair_offset', 'base_aspect', 'platform_aspect']),
  free: Object.freeze([]),
});

// The parameter fields of a topology in a fixed order (the order generated
// layouts have always used), independent of the key order of an imported
// topology_parameters object. beta_pair_offset is optional and defaults to 0.
export function topologyFields(topology) {
  if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
  return TOPOLOGY_FIELDS[topology];
}

export function topologyGeometry(topology, parameters) {
  if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
  if (topology === 'free') throw new Error('Free topology has no parametric geometry.');
  if (!parameters || typeof parameters !== 'object' || Array.isArray(parameters)) {
    throw new Error('topology_parameters must be an object.');
  }
  const fields = topologyFields(topology).filter(field => field !== 'beta_pair_offset');
  for (const field of fields) {
    if (!Number.isFinite(parameters[field])) throw new Error(`topology_parameters.${field} must be finite.`);
  }
  // Missing offsets retain the exact horn directions of previously exported layouts.
  const pairOffset = PAIRED_HORN_TOPOLOGIES.includes(topology)
    ? (parameters.beta_pair_offset === undefined ? 0 : parameters.beta_pair_offset) : 0;
  if (!Number.isFinite(pairOffset)) throw new Error('topology_parameters.beta_pair_offset must be finite.');
  for (const field of ['base_radius', 'platform_radius']) {
    if (parameters[field] <= 0) throw new Error(`topology_parameters.${field} must be positive.`);
  }
  if (topology === 'c3_paired') {
    for (const [gap, radius] of [['base_pair_gap', 'base_radius'], ['platform_pair_gap', 'platform_radius']]) {
      if (parameters[gap] <= 0 || parameters[gap] >= 2 * parameters[radius]) {
        throw new Error(`topology_parameters.${gap} must be positive and less than twice ${radius}.`);
      }
    }
    if (Math.abs(wrapAngle(parameters.beta_offset)) > C3_BETA_OFFSET_LIMIT + 1e-12) {
      throw new Error('topology_parameters.beta_offset must be within +/-90 degrees for c3_paired; '
        + 'use horn_direction to turn the horns toward their partner.');
    }
    if (parameters.horn_direction !== undefined && !C3_HORN_DIRECTIONS.includes(parameters.horn_direction)) {
      throw new Error(`topology_parameters.horn_direction must be one of ${C3_HORN_DIRECTIONS.join(', ')}.`);
    }
    return c3PairedGeometry(parameters);
  }
  if (topology === 'rectangular_paired') {
    for (const field of ['base_aspect', 'platform_aspect']) {
      if (parameters[field] <= 0) throw new Error(`topology_parameters.${field} must be positive.`);
    }
  }
  const make = (radius, orientation, aspect) => topology === 'circular'
    ? ring(radius, orientation) : rectangle(radius, aspect, orientation);
  const baseAnchors = make(parameters.base_radius, parameters.base_orientation, parameters.base_aspect);
  const platformAnchors = make(parameters.platform_radius, parameters.platform_orientation,
    parameters.platform_aspect);
  // Alternating horn directions preserve the plate geometry without forcing
  // the Circular and Rectangular home Jacobians to have dependent rows.
  const betaAngles = baseAnchors.map(([x, y], i) =>
    wrapAngle(Math.atan2(y, x) + Math.PI / 2 + parameters.beta_offset
      + (i % 2 ? pairOffset : -pairOffset)));
  return { baseAnchors, platformAnchors, betaAngles };
}

// A declared family is a claim about the actual anchors, never an instruction
// to reshape imported coordinates. The importer in #22 calls this before use.
export function validateTopology(layout) {
  const topology = layout.topology ?? 'free';
  if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
  if (topology === 'free') return topology;
  const expected = topologyGeometry(topology, layout.topologyParameters ?? layout.topology_parameters);
  for (const [field, values] of Object.entries(expected)) {
    const actual = layout[field] ?? layout[{ baseAnchors: 'base_anchors',
      platformAnchors: 'platform_anchors', betaAngles: 'beta_angles' }[field]];
    if (!Array.isArray(actual) || actual.length !== 6) {
      throw new Error(`${field} must contain six entries for declared topology ${topology}.`);
    }
    for (let i = 0; i < 6; i++) {
      const a = actual[i];
      const b = values[i];
      const good = field === 'betaAngles'
        ? Number.isFinite(a) && Math.abs(wrapAngle(a - b)) <= 1e-7
        : Array.isArray(a) && a.length === 3 && a.every((value, j) =>
          Number.isFinite(value) && Math.abs(value - b[j]) <= 1e-7 * Math.max(1, Math.abs(b[j])));
      if (!good) throw new Error(`${field}[${i}] conflicts with declared topology ${topology} and topology_parameters.`);
    }
  }
  return topology;
}

export { DEFAULT_TOPOLOGY };
