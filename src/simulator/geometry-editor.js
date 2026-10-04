import { degToRad } from '../math.js';
import { ensureLayout } from '../model/pose.js';
import { resolveMounting } from '../model/mounting.js';
import { C3_HORN_DIRECTIONS, topologyFields, topologyGeometry, validateTopology } from '../optimization/topology.js';
import { TOPOLOGIES } from '../contracts.js';

const copy = value => structuredClone(value);
const loadKey = state => state.layout == null ? null
  : JSON.stringify({ layout: state.layout, source: state.source });

// The editable parameters of each parametric topology are the shared
// `topologyFields`, shown in the editor's order: sizes (radii, then C3 gaps or
// rectangular aspects), then plate turns, then horn direction offsets.
const DISPLAY_GROUPS = [/_radius$/, /_(gap|aspect)$/, /_orientation$/, /^beta_/];
const displayOrder = fields => DISPLAY_GROUPS.flatMap(group => fields.filter(field => group.test(field)));
export const PARAMETER_FIELDS = Object.freeze(Object.fromEntries(TOPOLOGIES.filter(topology => topology !== 'free')
  .map(topology => [topology, Object.freeze(displayOrder(topologyFields(topology)))])));

export function geometryMode(layout) {
  if (!layout) return null;
  return (layout.topology ?? 'free') === 'free' ? 'explicit' : 'parametric';
}

function finite(value, field) {
  if (!Number.isFinite(value)) throw new RangeError(`${field} must be finite.`);
  return value;
}

function positive(value, field) {
  if (finite(value, field) <= 0) throw new RangeError(`${field} must be positive.`);
  return value;
}

function index(value, field) {
  if (!Number.isInteger(value) || value < 0 || value >= 6) {
    throw new RangeError(`${field} must be a leg index from 0 to 5.`);
  }
  return value;
}

function regenerate(layout) {
  Object.assign(layout, topologyGeometry(layout.topology, layout.topologyParameters));
}

export function editGeometry(source, edit) {
  ensureLayout(source);
  const layout = copy(source);
  switch (edit.type) {
    case 'parameter': {
      const fields = PARAMETER_FIELDS[layout.topology];
      if (!fields) throw new Error('Switch explicitly to a parametric topology before editing parameters.');
      if (!fields.includes(edit.field)) throw new RangeError(`${edit.field} is not a ${layout.topology} parameter.`);
      layout.topologyParameters[edit.field] = finite(edit.value, edit.field);
      regenerate(layout);
      break;
    }
    case 'hornDirection': {
      if (layout.topology !== 'c3_paired') throw new RangeError('Horn direction is a c3_paired parameter.');
      if (!C3_HORN_DIRECTIONS.includes(edit.value)) {
        throw new RangeError(`Horn direction must be one of ${C3_HORN_DIRECTIONS.join(', ')}.`);
      }
      layout.topologyParameters.horn_direction = edit.value;
      regenerate(layout);
      break;
    }
    case 'scalar': {
      if (!['hornLength', 'rodLength', 'homeHeight'].includes(edit.field)) {
        throw new RangeError(`${edit.field} is not an editable geometry scalar.`);
      }
      layout[edit.field] = positive(edit.value, edit.field);
      break;
    }
    case 'servoRange': {
      const bounds = edit.degrees;
      if (!Array.isArray(bounds) || bounds.length !== 2 || !bounds.every(Number.isFinite)
          || bounds[1] < bounds[0]) {
        throw new RangeError('servoRange must have finite minimum and maximum degrees, with max >= min.');
      }
      layout.servoRangeRad = bounds.map(degToRad);
      delete layout.servoRangeDeg;
      break;
    }
    case 'anchor': {
      if (geometryMode(layout) !== 'explicit') throw new Error('Switch to explicit-anchor mode before editing an anchor.');
      const collection = edit.plate === 'base' ? 'baseAnchors'
        : edit.plate === 'platform' ? 'platformAnchors' : null;
      if (!collection) throw new RangeError('Anchor plate must be base or platform.');
      const leg = index(edit.leg, 'Anchor leg');
      if (!Number.isInteger(edit.axis) || edit.axis < 0 || edit.axis > 2) {
        throw new RangeError('Anchor axis must be 0, 1, or 2.');
      }
      layout[collection][leg][edit.axis] = finite(edit.value, `${collection}[${leg}][${edit.axis}]`);
      break;
    }
    case 'betaAngle': {
      if (geometryMode(layout) !== 'explicit') throw new Error('Switch to explicit-anchor mode before editing a horn direction.');
      const leg = index(edit.leg, 'Horn leg');
      layout.betaAngles[leg] = degToRad(finite(edit.degrees, `betaAngles[${leg}]`));
      break;
    }
    case 'explicitMode':
      layout.topology = 'free';
      layout.topologyParameters = {};
      break;
    case 'generateParametric': {
      if (!TOPOLOGIES.includes(edit.topology) || edit.topology === 'free') {
        throw new RangeError('Choose circular, c3_paired, or rectangular_paired topology.');
      }
      layout.topology = edit.topology;
      layout.topologyParameters = copy(edit.parameters);
      regenerate(layout);
      break;
    }
    default:
      throw new RangeError(`Unknown geometry edit: ${edit.type}`);
  }
  ensureLayout(layout);
  validateTopology(layout);
  // Derived socket directions follow the home geometry; supplied ones stay fixed.
  // Without this the exported JSON would carry the pre-edit directions.
  if (layout.mounting != null) layout.mounting = resolveMounting(layout).mounting;
  return layout;
}

// The controller owns an independent layout. A geometry edit loads another copy
// and rechecks home through its shared pose evaluator; the optimizer's candidate
// never changes. An external controller.loadLayout becomes the new reset point.
export function createGeometryEditor(controller) {
  let baseline = null;
  let observedKey = null;
  let applying = false;
  const listeners = new Set();
  const unsubscribe = controller.subscribe(state => {
    const key = loadKey(state);
    if (key !== observedKey) {
      observedKey = key;
      if (!applying) baseline = state.layout ? {
        layout: copy(state.layout), source: copy(state.source),
      } : null;
    }
    for (const listener of listeners) listener(state);
  });

  function load(nextLayout, source, options) {
    applying = true;
    try { return controller.loadLayout(nextLayout, { source, options }); }
    finally { applying = false; }
  }

  return {
    edit(action) {
      const state = controller.getState();
      if (!state.layout) throw new Error('Load a layout before editing geometry.');
      const layout = editGeometry(controller.getReferenceLayout(), action);
      const candidateId = state.source?.candidateId ?? null;
      return load(layout, { kind: 'editable', candidateId },
        { ...state.options, servoRangeRad: layout.servoRangeRad.slice() });
    },
    reset() {
      if (!baseline) throw new Error('No loaded geometry to reset.');
      return load(copy(baseline.layout), copy(baseline.source),
        { ...controller.getState().options, servoRangeRad: baseline.layout.servoRangeRad.slice() });
    },
    getBaseline() { return copy(baseline); },
    subscribe(listener) { listeners.add(listener); listener(controller.getState()); return () => listeners.delete(listener); },
    dispose() { unsubscribe(); listeners.clear(); },
  };
}
