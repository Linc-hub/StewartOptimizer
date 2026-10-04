import { clamp, randomNormal, degToRad } from '../math.js';
import { DEFAULT_TOPOLOGY, TOPOLOGIES } from '../contracts.js';
import { topologyGeometry, topologyFields, validateTopology, wrapAngle, c3HornDirection,
  PAIRED_HORN_TOPOLOGIES, DEFAULT_BETA_PAIR_OFFSET, C3_BETA_OFFSET_LIMIT, C3_HORN_DIRECTIONS } from './topology.js';

export const DEFAULT_DESIGN_SPACE = {
  baseRadius: [90, 160], platformRadius: [40, 120], homeHeightBounds: [50, 450],
  hornLengthBounds: [30, 120], rodLengthBounds: [160, 420],
  pairGapBounds: [12, 45], rectangularAspectBounds: [0.6, 1.4],
  betaJitterRad: degToRad(20), anchorJitter: 6, platformJitter: 6, baseZJitter: 2,
  mutationHorn: 4, mutationRod: 6, mutationHeight: 15, mutationAngle: degToRad(4),
};

// The C3 horn directions an optimizer run may generate: one fixed direction,
// or `both`, where each new layout draws one, crossover inherits it and
// mutation occasionally flips it.
export const HORN_DIRECTION_MODES = Object.freeze([...C3_HORN_DIRECTIONS, 'both']);
export const DEFAULT_HORN_DIRECTION_MODE = 'outward';
export const HORN_DIRECTION_FLIP_PROBABILITY = 0.1;

export function validateHornDirectionMode(mode, topology) {
  if (!HORN_DIRECTION_MODES.includes(mode)) {
    throw new RangeError(`hornDirection must be one of ${HORN_DIRECTION_MODES.join(', ')}.`);
  }
  if (topology !== 'c3_paired' && mode !== DEFAULT_HORN_DIRECTION_MODE) {
    throw new RangeError('hornDirection applies to c3_paired layouts only.');
  }
  return mode;
}

// Holds a C3 layout's horn direction to a fixed mode. Outward leaves a missing
// direction missing, so default runs export exactly what they always have; an
// undefined mode (a caller outside an optimizer run) changes nothing.
function applyHornDirection(p, mode) {
  if (mode === 'inward') p.horn_direction = 'inward';
  else if (mode === 'outward') { if (p.horn_direction !== undefined) p.horn_direction = 'outward'; }
  else if (mode === 'both') p.horn_direction ??= 'outward';
}

const randomInRange = ([min, max], random) => min + random() * (max - min);

// Wraps an angle; a C3 horn offset is also held inside the range that keeps
// the two horns of a pair apart.
function boundAngle(topology, field, value) {
  const angle = wrapAngle(value);
  return topology === 'c3_paired' && field === 'beta_offset'
    ? clamp(angle, -C3_BETA_OFFSET_LIMIT, C3_BETA_OFFSET_LIMIT) : angle;
}

function pairGapRange(radius, space) {
  const [min, max] = space.pairGapBounds;
  const ceiling = Math.min(max, 1.2 * radius);
  if (ceiling < min) throw new Error('pairGapBounds cannot fit the selected radius bounds.');
  return [min, ceiling];
}

function regenerateBoundedTopology(layout, space, hornDirection) {
  const { topology, topologyParameters: p } = layout;
  const minimum = topology === 'c3_paired' ? space.pairGapBounds[0] / 1.2 : 0;
  p.base_radius = clamp(p.base_radius, Math.max(space.baseRadius[0], minimum), space.baseRadius[1]);
  p.platform_radius = clamp(p.platform_radius, Math.max(space.platformRadius[0], minimum), space.platformRadius[1]);
  for (const field of ['base_orientation', 'platform_orientation', 'beta_offset']) {
    if (topologyFields(topology).includes(field)) p[field] = boundAngle(topology, field, p[field]);
  }
  if (PAIRED_HORN_TOPOLOGIES.includes(topology)) {
    p.beta_pair_offset = wrapAngle(p.beta_pair_offset ?? 0);
  }
  if (topology === 'c3_paired') {
    p.base_pair_gap = clamp(p.base_pair_gap, ...pairGapRange(p.base_radius, space));
    p.platform_pair_gap = clamp(p.platform_pair_gap, ...pairGapRange(p.platform_radius, space));
    applyHornDirection(p, hornDirection);
  }
  if (topology === 'rectangular_paired') {
    p.base_aspect = clamp(p.base_aspect, ...space.rectangularAspectBounds);
    p.platform_aspect = clamp(p.platform_aspect, ...space.rectangularAspectBounds);
  }
  Object.assign(layout, topologyGeometry(topology, p));
}

function clampPointRadius(anchor, bounds) {
  const radius = Math.hypot(anchor[0], anchor[1]);
  if (radius < 1e-12) {
    anchor[0] = bounds[0]; anchor[1] = 0;
  } else {
    const scale = clamp(radius, ...bounds) / radius;
    anchor[0] *= scale; anchor[1] *= scale;
  }
}

// Inward and `both` runs draw the C3 horn offset over the full +/-90 deg:
// inward horns near tangent usually cross their partner across the pair gap.
// Every extra draw comes after the existing ones and only in those modes, so an
// outward (default) run consumes exactly the random stream it always has.
function randomParameters(topology, space, random, hornDirection = DEFAULT_HORN_DIRECTION_MODE) {
  const radiusBounds = bounds => topology === 'c3_paired'
    ? [Math.max(bounds[0], space.pairGapBounds[0] / 1.2), bounds[1]] : bounds;
  const p = {
    base_radius: randomInRange(radiusBounds(space.baseRadius), random),
    platform_radius: randomInRange(radiusBounds(space.platformRadius), random),
    base_orientation: randomInRange([-Math.PI, Math.PI], random),
  };
  if (topology !== 'c3_paired') p.platform_orientation = randomInRange([-Math.PI, Math.PI], random);
  const betaRange = topology === 'c3_paired' && hornDirection !== 'outward' ? C3_BETA_OFFSET_LIMIT : space.betaJitterRad;
  p.beta_offset = randomInRange([-betaRange, betaRange], random);
  if (PAIRED_HORN_TOPOLOGIES.includes(topology)) p.beta_pair_offset = DEFAULT_BETA_PAIR_OFFSET;
  if (topology === 'c3_paired') {
    p.base_pair_gap = randomInRange(pairGapRange(p.base_radius, space), random);
    p.platform_pair_gap = randomInRange(pairGapRange(p.platform_radius, space), random);
    if (hornDirection === 'inward') p.horn_direction = 'inward';
    else if (hornDirection === 'both') p.horn_direction = random() < 0.5 ? 'outward' : 'inward';
  }
  if (topology === 'rectangular_paired') {
    p.base_aspect = randomInRange(space.rectangularAspectBounds, random);
    p.platform_aspect = randomInRange(space.rectangularAspectBounds, random);
  }
  return p;
}

export function validateDesignSpace(space) {
  for (const field of ['baseRadius', 'platformRadius', 'homeHeightBounds', 'hornLengthBounds',
    'rodLengthBounds', 'pairGapBounds', 'rectangularAspectBounds']) {
    const range = space[field];
    if (!Array.isArray(range) || range.length !== 2 || !range.every(Number.isFinite)
      || range[0] <= 0 || range[1] < range[0]) {
      throw new Error(`${field} must contain two positive finite bounds with max >= min.`);
    }
  }
  if (space.pairGapBounds[0] >= 1.2 * Math.min(space.baseRadius[1], space.platformRadius[1])) {
    throw new Error('pairGapBounds cannot fit baseRadius and platformRadius.');
  }
}

export const cloneLayout = layout => JSON.parse(JSON.stringify(layout));

export function createRandomLayout({ designSpace: space, servoRangeRad, servoRangeDeg, id, topology = DEFAULT_TOPOLOGY,
  hornDirection, random = Math.random }) {
  if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
  const layout = {
    id, topology, topologyParameters: topology === 'free' ? {} : randomParameters(topology, space, random, hornDirection),
    baseAnchors: [], platformAnchors: [], betaAngles: [],
    hornLength: randomInRange(space.hornLengthBounds, random), rodLength: randomInRange(space.rodLengthBounds, random),
    servoRangeRad: servoRangeRad.slice(), homeHeight: randomInRange(space.homeHeightBounds, random),
  };
  if (topology === 'free') {
    const baseOffset = random() * 2 * Math.PI;
    for (let i = 0; i < 6; i++) {
      const angle = baseOffset + i * Math.PI / 3 + randomNormal(random) * degToRad(3);
      const upperAngle = baseOffset + Math.PI / 6 + i * Math.PI / 3 + randomNormal(random) * degToRad(3);
      const br = randomInRange(space.baseRadius, random);
      const pr = randomInRange(space.platformRadius, random);
      layout.baseAnchors.push([br * Math.cos(angle), br * Math.sin(angle),
        randomNormal(random) * space.baseZJitter]);
      layout.platformAnchors.push([pr * Math.cos(upperAngle), pr * Math.sin(upperAngle), 0]);
      layout.betaAngles.push(wrapAngle(angle + Math.PI / 2
        + randomNormal(random) * space.betaJitterRad));
    }
  } else Object.assign(layout, topologyGeometry(topology, layout.topologyParameters));
  return finalizeLayout(layout, { designSpace: space, servoRangeRad, servoRangeDeg, hornDirection });
}

// servoRangeDeg is the degree form of servoRangeRad when the caller has one (the
// Optimizer's servo_travel_bounds_deg); exported servo_range then matches the run
// settings exactly instead of the radians converted back. hornDirection is the
// run's C3 horn direction mode; without one a layout keeps its own direction.
export function finalizeLayout(layout, { designSpace: space, servoRangeRad, servoRangeDeg = null, hornDirection }) {
  const topology = validateTopology(layout);
  layout.topology = topology;
  layout.topologyParameters ??= topology === 'free' ? {} : layout.topology_parameters;
  layout.hornLength = clamp(layout.hornLength, ...space.hornLengthBounds);
  layout.rodLength = clamp(layout.rodLength, ...space.rodLengthBounds);
  layout.homeHeight = clamp(layout.homeHeight, ...space.homeHeightBounds);
  if (topology === 'free') {
    for (const point of layout.baseAnchors) {
      clampPointRadius(point, space.baseRadius);
      point[2] = clamp(point[2], -space.baseZJitter, space.baseZJitter);
    }
    // Generated platform anchors stay in the platform plane, so an out-of-plane
    // reference anchor is not inherited by its variations or offspring.
    for (const point of layout.platformAnchors) {
      clampPointRadius(point, space.platformRadius);
      point[2] = 0;
    }
  } else {
    regenerateBoundedTopology(layout, space, hornDirection);
  }
  layout.servoRangeRad = servoRangeRad.slice();
  // The imported-degree copy would otherwise shadow the finalized range on export.
  if (servoRangeDeg) layout.servoRangeDeg = servoRangeDeg.slice();
  else delete layout.servoRangeDeg;
  return layout;
}

export function mutateLayout(source, { designSpace: space, servoRangeRad, servoRangeDeg, hornDirection, random = Math.random }) {
  validateTopology(source);
  const layout = cloneLayout(source);
  if ((layout.topology ?? 'free') === 'free') {
    for (let i = 0; i < 6; i++) {
      layout.baseAnchors[i][0] += randomNormal(random) * space.anchorJitter;
      layout.baseAnchors[i][1] += randomNormal(random) * space.anchorJitter;
      layout.baseAnchors[i][2] += randomNormal(random) * space.baseZJitter;
      layout.platformAnchors[i][0] += randomNormal(random) * space.platformJitter;
      layout.platformAnchors[i][1] += randomNormal(random) * space.platformJitter;
      layout.betaAngles[i] = wrapAngle(layout.betaAngles[i]
        + randomNormal(random) * space.mutationAngle);
    }
  } else {
    const p = layout.topologyParameters;
    const minimum = layout.topology === 'c3_paired' ? space.pairGapBounds[0] / 1.2 : 0;
    p.base_radius = clamp(p.base_radius + randomNormal(random) * space.anchorJitter,
      Math.max(space.baseRadius[0], minimum), space.baseRadius[1]);
    p.platform_radius = clamp(p.platform_radius + randomNormal(random) * space.platformJitter,
      Math.max(space.platformRadius[0], minimum), space.platformRadius[1]);
    p.base_orientation += randomNormal(random) * space.mutationAngle;
    if (layout.topology !== 'c3_paired') p.platform_orientation += randomNormal(random) * space.mutationAngle;
    p.beta_offset += randomNormal(random) * space.mutationAngle;
    // Bound the C3 offset now: topologyGeometry rejects one beyond the limit.
    if (layout.topology === 'c3_paired') p.beta_offset = boundAngle('c3_paired', 'beta_offset', p.beta_offset);
    if (PAIRED_HORN_TOPOLOGIES.includes(layout.topology)) {
      p.beta_pair_offset = (p.beta_pair_offset ?? 0) + randomNormal(random) * space.mutationAngle;
    }
    if (layout.topology === 'c3_paired') {
      p.base_pair_gap = clamp(p.base_pair_gap + randomNormal(random) * space.anchorJitter,
        ...pairGapRange(p.base_radius, space));
      p.platform_pair_gap = clamp(p.platform_pair_gap + randomNormal(random) * space.platformJitter,
        ...pairGapRange(p.platform_radius, space));
      // Only a `both` run draws here, so fixed-direction runs keep their stream.
      if (hornDirection === 'both' && random() < HORN_DIRECTION_FLIP_PROBABILITY) {
        p.horn_direction = c3HornDirection(p) === 'inward' ? 'outward' : 'inward';
      }
    }
    if (layout.topology === 'rectangular_paired') {
      p.base_aspect = clamp(p.base_aspect + randomNormal(random) * 0.05,
        ...space.rectangularAspectBounds);
      p.platform_aspect = clamp(p.platform_aspect + randomNormal(random) * 0.05,
        ...space.rectangularAspectBounds);
    }
    Object.assign(layout, topologyGeometry(layout.topology, p));
  }
  layout.hornLength += randomNormal(random) * space.mutationHorn;
  layout.rodLength += randomNormal(random) * space.mutationRod;
  layout.homeHeight += randomNormal(random) * space.mutationHeight;
  return finalizeLayout(layout, { designSpace: space, servoRangeRad, servoRangeDeg, hornDirection });
}

export function crossoverLayouts(a, b, { designSpace: space, servoRangeRad, servoRangeDeg, hornDirection, random = Math.random }) {
  validateTopology(a); validateTopology(b);
  if ((a.topology ?? 'free') !== (b.topology ?? 'free')) {
    throw new Error('Cannot cross layouts with different topologies.');
  }
  const layout = cloneLayout(a);
  if ((layout.topology ?? 'free') === 'free') {
    const split = Math.floor(random() * 6);
    for (let i = split; i < 6; i++) {
      layout.baseAnchors[i] = b.baseAnchors[i].slice();
      layout.platformAnchors[i] = b.platformAnchors[i].slice();
      layout.betaAngles[i] = b.betaAngles[i];
    }
  } else {
    // Draw in a fixed field order so a seeded run does not depend on the key
    // order of an imported reference's topology_parameters.
    layout.topologyParameters = {};
    for (const field of topologyFields(layout.topology)) {
      const fallback = field === 'beta_pair_offset' ? 0 : undefined;
      layout.topologyParameters[field] = random() < 0.5
        ? a.topologyParameters[field] ?? fallback : b.topologyParameters[field] ?? fallback;
    }
    // The child takes the parents' common horn direction; only parents that
    // differ draw, so outward-only runs consume the stream they always have.
    const [hornA, hornB] = [a.topologyParameters.horn_direction, b.topologyParameters.horn_direction];
    if (layout.topology === 'c3_paired' && (hornA !== undefined || hornB !== undefined)) {
      const [first, second] = [c3HornDirection(a.topologyParameters), c3HornDirection(b.topologyParameters)];
      layout.topologyParameters.horn_direction = first === second || random() < 0.5 ? first : second;
    }
    // A valid diagnostic parent may be outside the search bounds. Bound coupled
    // radius/gap choices before constructing the child, leaving both parents intact.
    regenerateBoundedTopology(layout, space, hornDirection);
  }
  layout.hornLength = (a.hornLength + b.hornLength) / 2;
  layout.rodLength = (a.rodLength + b.rodLength) / 2;
  layout.homeHeight = random() < 0.5 ? a.homeHeight : b.homeHeight;
  return finalizeLayout(layout, { designSpace: space, servoRangeRad, servoRangeDeg, hornDirection });
}
