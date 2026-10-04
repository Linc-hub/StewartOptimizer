import { radToDeg } from '../math.js';
import { topologyGeometry, PAIRED_HORN_TOPOLOGIES, DEFAULT_BETA_PAIR_OFFSET,
  C3_BETA_OFFSET_LIMIT, C3_HORN_DIRECTIONS, c3HornDirection } from '../optimization/topology.js';
import { createGeometryEditor, geometryMode, PARAMETER_FIELDS } from './geometry-editor.js';

const ANGLE_FIELDS = new Set(['base_orientation', 'platform_orientation', 'beta_offset', 'beta_pair_offset']);
const LABELS = {
  base_radius: 'Base radius (mm)', platform_radius: 'Platform radius (mm)',
  base_pair_gap: 'Base anchor spacing (mm)', platform_pair_gap: 'Platform anchor spacing (mm)',
  base_aspect: 'Base width/depth ratio', platform_aspect: 'Platform width/depth ratio',
  base_orientation: 'Base turn (deg)', platform_orientation: 'Platform turn (deg)',
  beta_offset: 'Horn direction offset (deg)',
  beta_pair_offset: 'Alternating horn offset (deg)',
  hornLength: 'Horn length (mm)', rodLength: 'Rod length (mm)', homeHeight: 'Home height (mm)',
};

function element(document, tag, { text, className, id, type } = {}) {
  const node = document.createElement(tag);
  if (text != null) node.textContent = text;
  if (className) node.className = className;
  if (id) node.id = id;
  if (type) node.type = type;
  return node;
}

function meanRadius(anchors) {
  return anchors.reduce((sum, [x, y]) => sum + Math.hypot(x, y), 0) / anchors.length;
}

export function suggestedParameters(layout, topology) {
  const base = Math.max(1, meanRadius(layout.baseAnchors));
  const platform = Math.max(1, meanRadius(layout.platformAnchors));
  const parameters = {
    base_radius: base, platform_radius: platform,
    base_orientation: 0, platform_orientation: 0, beta_offset: 0,
  };
  if (PAIRED_HORN_TOPOLOGIES.includes(topology)) parameters.beta_pair_offset = DEFAULT_BETA_PAIR_OFFSET;
  if (topology === 'c3_paired') {
    delete parameters.platform_orientation;
    parameters.base_pair_gap = Math.min(30, base);
    parameters.platform_pair_gap = Math.min(30, platform);
  }
  if (topology === 'rectangular_paired') {
    parameters.base_aspect = 1;
    parameters.platform_aspect = 1;
  }
  // Validate the suggested generator before showing its explicit mode switch.
  topologyGeometry(topology, parameters);
  return parameters;
}

// Writes the layout's value into an input unless the user is editing it: the
// input is focused and its text has moved away from the value last written to
// it. A focused but untouched field still follows a load or reset, which
// matters in browsers where clicking a button does not move focus.
// A committed entry records the typed text as the baseline, so a stored value
// that prints differently from what was typed (77.0 for 77) still counts as
// untouched and follows the next reset, load or settings change.
export function markCommitted(input, synced) {
  synced.set(input, input.value);
}

// Values print with at most 12 significant digits, so a degree that round-trips
// through radians shows 30 rather than 29.999999999999996.
export function displayText(value) {
  return Number.isFinite(value) ? String(Number(value.toPrecision(12))) : String(value);
}

// Only the exact text counts as untouched. Comparing numbers instead would
// rewrite in-progress typing that happens to parse to the shown value
// (50.0 on the way to 50.05) on the next animation frame.
export function syncInput(document, input, shown, synced, force = false) {
  const untouched = input.value === synced.get(input) || input.value === shown;
  if (!force && input === document.activeElement && !untouched) return false;
  input.value = shown;
  synced.set(input, shown);
  return true;
}

export function createGeometryControls({ document, container, controller }) {
  if (!container) throw new Error('A simulator geometry controls container is required.');
  const editor = createGeometryEditor(controller);
  const controls = new Map();
  const synced = new WeakMap();
  let structure = null;
  let status = null;
  let unsubscribe;

  function report(message, error = false) {
    if (!status) return;
    // The status is aria-live and sync runs every animation frame.
    if (status.textContent !== message) status.textContent = message;
    status.classList.toggle('error', error);
  }

  function perform(action) {
    try {
      editor.edit(action);
      report('Geometry changed. Home pose and diagnostics were reevaluated.');
    } catch (error) {
      sync(controller.getState(), { force: true });
      report(error.message, true);
    }
  }

  // Number('') is 0, so a cleared field must be rejected explicitly instead of
  // silently moving an anchor, horn direction or servo bound to zero.
  function numeric(text, label) {
    const trimmed = String(text ?? '').trim();
    const value = trimmed === '' ? NaN : Number(trimmed);
    if (Number.isFinite(value)) return value;
    sync(controller.getState(), { force: true });
    report(`${label} must be a finite number.`, true);
    return null;
  }

  function addButton(parent, label, id, handler) {
    const button = element(document, 'button', { text: label, id, type: 'button' });
    button.addEventListener('click', handler);
    parent.appendChild(button);
    return button;
  }

  function addPair(parent, { key, label, value, edit, min = 0, max = 500, step = 0.1 }) {
    const row = element(document, 'label', { className: 'sim-geometry-row' });
    row.appendChild(element(document, 'span', { text: label }));
    const sliders = element(document, 'span', { className: 'sim-geometry-inputs' });
    const range = element(document, 'input', { id: `sim-${key}-range`, type: 'range' });
    const number = element(document, 'input', { id: `sim-${key}-number`, type: 'number' });
    range.min = String(Math.min(min, value));
    range.max = String(Math.max(max, value));
    range.step = String(step);
    number.step = String(step);
    const change = event => {
      markCommitted(event.target, synced);
      const value = numeric(event.target.value, label);
      if (value !== null) edit(value);
    };
    range.addEventListener('input', change);
    number.addEventListener('change', change);
    sliders.append(range, number);
    row.appendChild(sliders);
    parent.appendChild(row);
    controls.set(key, { range, number, value });
  }

  function addNumber(parent, { key, label, value, edit, step = 0.1 }) {
    const row = element(document, 'label', { className: 'sim-geometry-row' });
    row.appendChild(element(document, 'span', { text: label }));
    const number = element(document, 'input', { id: `sim-${key}-number`, type: 'number' });
    number.step = String(step);
    number.addEventListener('change', event => {
      markCommitted(event.target, synced);
      const value = numeric(event.target.value, label);
      if (value !== null) edit(value);
    });
    row.appendChild(number);
    parent.appendChild(row);
    controls.set(key, { number, value });
  }

  function parameter(parent, layout, field) {
    const angle = ANGLE_FIELDS.has(field);
    const storedValue = layout.topologyParameters[field] ?? 0;
    const value = angle ? radToDeg(storedValue) : storedValue;
    const isAspect = field.endsWith('aspect');
    const angleLimit = layout.topology === 'c3_paired' && field === 'beta_offset'
      ? radToDeg(C3_BETA_OFFSET_LIMIT) : 180;
    addPair(parent, {
      key: field.replaceAll('_', '-'), label: LABELS[field], value,
      min: angle ? -angleLimit : isAspect ? 0.2 : 1,
      max: angle ? angleLimit : isAspect ? 3 : field.includes('gap') ? 200 : 500,
      step: angle ? 0.1 : isAspect ? 0.01 : 0.1,
      edit: number => perform({ type: 'parameter', field,
        value: angle ? number * Math.PI / 180 : number }),
    });
  }

  // C3 horns point away from (outward) or toward (inward) their pair partner.
  function hornDirection(parent, layout) {
    const row = element(document, 'label', { className: 'sim-geometry-row' });
    row.appendChild(element(document, 'span', { text: 'Horn direction' }));
    const select = element(document, 'select', { id: 'sim-horn-direction' });
    for (const value of C3_HORN_DIRECTIONS) {
      const option = element(document, 'option', { text: value === 'inward' ? 'Inward (toward partner)' : 'Outward (away from partner)' });
      option.value = value;
      select.appendChild(option);
    }
    select.value = c3HornDirection(layout.topologyParameters);
    select.addEventListener('change', event => {
      markCommitted(event.target, synced);
      perform({ type: 'hornDirection', value: event.target.value });
    });
    row.appendChild(select);
    parent.appendChild(row);
    controls.set('horn-direction', { number: select });
  }

  function scalar(parent, layout, field) {
    addPair(parent, {
      key: field, label: LABELS[field], value: layout[field], min: 0.1,
      max: field === 'homeHeight' ? 1000 : 500,
      edit: number => perform({ type: 'scalar', field, value: number }),
    });
  }

  function explicitAnchors(parent, layout) {
    for (const plate of ['base', 'platform']) {
      parent.appendChild(element(document, 'h4', { text: `${plate === 'base' ? 'Base' : 'Platform'} anchors (mm)` }));
      for (let leg = 0; leg < 6; leg++) {
        const group = element(document, 'div', { className: 'sim-anchor-row' });
        for (let axis = 0; axis < 3; axis++) {
          const key = `${plate}-${leg}-${'xyz'[axis]}`;
          addNumber(group, { key, label: `Leg ${leg + 1} ${'XYZ'[axis]}`,
            value: layout[plate === 'base' ? 'baseAnchors' : 'platformAnchors'][leg][axis],
            edit: value => perform({ type: 'anchor', plate, leg, axis, value }) });
        }
        parent.appendChild(group);
      }
    }
    parent.appendChild(element(document, 'h4', { text: 'Horn directions (deg)' }));
    for (let leg = 0; leg < 6; leg++) {
      addNumber(parent, { key: `beta-${leg}`, label: `Leg ${leg + 1} β`, value: radToDeg(layout.betaAngles[leg]),
        edit: degrees => perform({ type: 'betaAngle', leg, degrees }) });
    }
  }

  function render(state) {
    container.replaceChildren();
    controls.clear();
    const title = element(document, 'h3', { text: 'Mechanical geometry' });
    container.appendChild(title);
    if (!state.layout) {
      container.appendChild(element(document, 'p', { text: 'Select a candidate or load a layout to edit geometry.' }));
      status = null;
      return;
    }
    const layout = state.layout;
    const mode = geometryMode(layout);
    container.appendChild(element(document, 'p', { className: 'sim-geometry-note',
      text: mode === 'parametric'
        ? `${layout.topology} parameters regenerate anchors and horn directions together.`
        : 'Explicit anchors retain each entered coordinate. Generating a topology replaces them only when requested.' }));
    const actions = element(document, 'div', { className: 'button-row' });
    if (mode === 'parametric') {
      addButton(actions, 'Edit anchors explicitly', 'sim-switch-explicit',
        () => perform({ type: 'explicitMode' }));
    } else {
      const select = element(document, 'select', { id: 'sim-generate-topology' });
      for (const [value, label] of [['c3_paired', 'C3 paired'], ['circular', 'Circular'],
        ['rectangular_paired', 'Rectangular paired']]) {
        const option = element(document, 'option', { text: label });
        option.value = value;
        select.appendChild(option);
      }
      actions.appendChild(select);
      addButton(actions, 'Generate selected topology', 'sim-generate-parametric', () => {
        const current = controller.getState().layout;
        perform({ type: 'generateParametric', topology: select.value,
          parameters: suggestedParameters(current, select.value) });
      });
    }
    addButton(actions, 'Reset geometry', 'sim-reset-geometry', () => {
      try {
        editor.reset();
        report('Original loaded geometry restored.');
      } catch (error) { report(error.message, true); }
    });
    container.appendChild(actions);

    if (mode === 'parametric') {
      for (const field of PARAMETER_FIELDS[layout.topology]) {
        if (layout.topology === 'c3_paired' && field === 'beta_offset') hornDirection(container, layout);
        if (LABELS[field]) parameter(container, layout, field);
      }
    } else explicitAnchors(container, layout);
    for (const field of ['hornLength', 'rodLength', 'homeHeight']) scalar(container, layout, field);
    for (let bound = 0; bound < 2; bound++) {
      const key = `servo-${bound === 0 ? 'min' : 'max'}`;
      const degrees = layout.servoRangeRad.map(radToDeg);
      addPair(container, { key, label: `Servo ${bound === 0 ? 'minimum' : 'maximum'} (deg)`,
        value: degrees[bound], min: -180, max: 180, step: 0.1,
        edit: value => {
          const current = controller.getState().layout.servoRangeRad.map(radToDeg);
          current[bound] = value;
          perform({ type: 'servoRange', degrees: current });
        } });
    }
    status = element(document, 'p', { className: 'status', id: 'sim-geometry-status' });
    status.setAttribute('aria-live', 'polite');
    container.appendChild(status);
  }

  // Animation ticks notify every frame; an input the user is typing into keeps
  // its text unless the caller forces a rewrite after a rejected edit.
  function sync(state, { force = false } = {}) {
    if (!state.layout) return;
    const layout = state.layout;
    const values = { hornLength: layout.hornLength, rodLength: layout.rodLength,
      homeHeight: layout.homeHeight,
      'servo-min': radToDeg(layout.servoRangeRad[0]),
      'servo-max': radToDeg(layout.servoRangeRad[1]) };
    if (geometryMode(layout) === 'parametric') {
      for (const field of PARAMETER_FIELDS[layout.topology]) {
        const value = layout.topologyParameters[field] ?? 0;
        values[field.replaceAll('_', '-')] = ANGLE_FIELDS.has(field) ? radToDeg(value) : value;
      }
      if (layout.topology === 'c3_paired') values['horn-direction'] = c3HornDirection(layout.topologyParameters);
    } else {
      for (const plate of ['base', 'platform']) for (let leg = 0; leg < 6; leg++) {
        for (let axis = 0; axis < 3; axis++) {
          values[`${plate}-${leg}-${'xyz'[axis]}`] =
            layout[plate === 'base' ? 'baseAnchors' : 'platformAnchors'][leg][axis];
        }
      }
      for (let leg = 0; leg < 6; leg++) values[`beta-${leg}`] = radToDeg(layout.betaAngles[leg]);
    }
    for (const [key, control] of controls) {
      const value = values[key];
      if (value == null) continue;
      syncInput(document, control.number, displayText(value), synced, force);
      if (control.range) {
        control.range.min = String(Math.min(Number(control.range.min), value));
        control.range.max = String(Math.max(Number(control.range.max), value));
        syncInput(document, control.range, displayText(value), synced, force);
      }
    }
    report(state.source?.kind === 'editable'
      ? `Editable copy${state.source.candidateId == null ? '' : ` of candidate ${state.source.candidateId}`}; home ${state.assessment?.reachable ? 'valid' : 'invalid'}.`
      : `Loaded ${state.source?.kind ?? 'layout'}; home ${state.assessment?.reachable ? 'valid' : 'invalid'}.`);
  }

  unsubscribe = editor.subscribe(state => {
    const nextStructure = state.layout ? `${geometryMode(state.layout)}:${state.layout.topology}` : 'empty';
    if (nextStructure !== structure) {
      structure = nextStructure;
      render(state);
    }
    sync(state);
  });
  return { editor, dispose() { unsubscribe(); editor.dispose(); container.replaceChildren(); } };
}
