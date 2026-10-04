import { validateDesignSpace } from './layout-operators.js';

// Opt-in widening of the geometry bounds an optimizer run draws and clamps
// layouts to. A run with `boundsRelaxation` > 0 starts at its nominal design
// space and, after each generation in which no retained candidate passes,
// widens every relaxable range by another RELAXATION_STEPS-th of that share,
// as a fraction of each bound (min * (1 - share), max * (1 + share)), up to the
// full share. The next generation then replaces RELAXATION_IMMIGRANT_SHARE of
// its offspring with fresh layouts drawn in the widened space, so the search
// can actually reach it. It never narrows again. The nominal ranges stay the
// reference: every candidate reports how far it lies outside them
// (`boundsExcursions`), so a relaxed result is never mistaken for one inside
// the requested bounds.

export const RELAXABLE_FIELDS = Object.freeze(['baseRadius', 'platformRadius', 'homeHeightBounds',
  'hornLengthBounds', 'rodLengthBounds', 'pairGapBounds']);
export const BOUNDS_RELAXATION_CHOICES = Object.freeze([0, 0.1, 0.25, 0.5]);
export const RELAXATION_STEPS = 4;
// Share of a generation's offspring drawn fresh after a widening step.
export const RELAXATION_IMMIGRANT_SHARE = 0.25;

export function validateBoundsRelaxation(value) {
  if (!BOUNDS_RELAXATION_CHOICES.includes(value)) {
    throw new RangeError(`boundsRelaxation must be one of ${BOUNDS_RELAXATION_CHOICES.join(', ')}.`);
  }
  return value;
}

// The design space with each relaxable [min, max] widened to
// [min * (1 - share), max * (1 + share)]. A share of 0 returns the space unchanged.
export function relaxedDesignSpace(space, share) {
  if (!(share > 0)) return space;
  const relaxed = { ...space };
  for (const field of RELAXABLE_FIELDS) {
    const [min, max] = space[field];
    relaxed[field] = [min * (1 - share), max * (1 + share)];
  }
  validateDesignSpace(relaxed);
  return relaxed;
}

const radii = anchors => anchors.map(([x, y]) => Math.hypot(x, y));

// The layout's value for each relaxable field: the topology radius and pair gap
// parameters where the topology has them, else the anchor radii (min and max).
function layoutValues(layout) {
  const p = layout.topologyParameters ?? {};
  const values = [];
  const radius = (field, parameter, anchors) => {
    if (Number.isFinite(p[parameter])) values.push([field, p[parameter]]);
    else if (anchors?.length) for (const value of [Math.min(...radii(anchors)), Math.max(...radii(anchors))]) values.push([field, value]);
  };
  radius('baseRadius', 'base_radius', layout.baseAnchors);
  radius('platformRadius', 'platform_radius', layout.platformAnchors);
  values.push(['homeHeightBounds', layout.homeHeight], ['hornLengthBounds', layout.hornLength],
    ['rodLengthBounds', layout.rodLength]);
  for (const parameter of ['base_pair_gap', 'platform_pair_gap']) {
    if (Number.isFinite(p[parameter])) values.push(['pairGapBounds', p[parameter]]);
  }
  return values;
}

// Every place a layout lies outside the nominal design space:
// `[{ field, value, nominal: [min, max], excessMm }]`, where `excessMm` is
// negative below the minimum and positive above the maximum. Empty inside.
export function boundsExcursions(layout, space) {
  const result = [];
  for (const [field, value] of layoutValues(layout)) {
    const [min, max] = space[field];
    const tolerance = 1e-9 * Math.max(1, Math.abs(max));
    if (value < min - tolerance) result.push({ field, value, nominal: [min, max], excessMm: value - min });
    else if (value > max + tolerance) result.push({ field, value, nominal: [min, max], excessMm: value - max });
  }
  return result;
}

const FIELD_NAMES = Object.freeze({ baseRadius: 'base radius', platformRadius: 'platform radius',
  homeHeightBounds: 'home height', hornLengthBounds: 'horn length', rodLengthBounds: 'rod length',
  pairGapBounds: 'pair gap' });
const percent = share => `${Math.round(share * 100)}%`;

// One sentence for the completion status about a run's bounds relaxation.
export function describeRelaxation(summary) {
  if (!summary) return '';
  if (!(summary.finalShare > 0)) return `Geometry bounds were not relaxed (allowed up to ${percent(summary.maxShare)}; a candidate passed within them).`;
  return `Geometry bounds relaxed by ${percent(summary.finalShare)} of up to ${percent(summary.maxShare)}; `
    + `${summary.outsideNominal} retained candidate${summary.outsideNominal === 1 ? '' : 's'} lie outside the requested bounds (see bounds_excursions).`;
}

