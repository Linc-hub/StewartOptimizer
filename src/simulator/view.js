import { clamp, degToRad, radToDeg, vectorDot, vectorSub } from '../math.js';
import { HOME_POSE, POSE_AXES } from './controller.js';
import { cameraFrame, createWebGLRenderer, projectSegment, VIEW_HALF_TANGENT } from './renderer.js';
import { buildSceneGeometry, OVERLAY_NAMES, SCENE_COLORS } from './scene.js';
import { displayText, markCommitted, syncInput } from './geometry-controls.js';
import { REACHABILITY_SAMPLE_COUNTS } from './reachability.js';
import { hasLoad } from './loads.js';
import { resolveTranslationFrame, stepTranslation, translationFromFrame, translationInFrame } from './translation-frame.js';

const RADIAN_AXES = new Set(['rx', 'ry', 'rz']);
const TRANSLATION_AXES = Object.freeze(['x', 'y', 'z']);
// The CSS form of a scene colour. The pose labels take the canvas axis colours
// through --axis-x/y/z, so the two cannot drift apart.
export const axisCssColor = ([r, g, b]) => `rgb(${[r, g, b].map(value => Math.round(value * 255)).join(', ')})`;
const axisInput = (document, axis) => document.getElementById(`sim${axis.toUpperCase()}Input`);
const axisSlider = (document, axis) => document.getElementById(`sim${axis.toUpperCase()}Slider`);
const displayValue = (axis, value) => RADIAN_AXES.has(axis) ? radToDeg(value) : value;
const modelValue = (axis, value) => RADIAN_AXES.has(axis) ? degToRad(value) : value;
const fmt = value => Number.isFinite(value) ? Number(value.toFixed(2)) : '—';
// The near end lets a close-up separate lines a few millimetres apart; the
// renderer clips lines at its 1 mm near plane rather than dropping them.
export const CAMERA_DISTANCE_RANGE = Object.freeze([10, 2500]);
export const CAMERA_PITCH_LIMIT = 1.4;
// The wheel zooms toward a drawn line within this many CSS pixels of the cursor.
export const ZOOM_PICK_RADIUS_PX = 30;
// Each overlay toggle is a checkbox named after its builder: worldAxes -> simOverlayWorldAxes.
export const overlayInputId = name => `simOverlay${name[0].toUpperCase()}${name.slice(1)}`;

// Only the known camera fields are taken, each checked, so a saved camera from
// hand-edited JSON cannot spread characters or store a non-finite view.
export function parseCamera(next, field = 'camera') {
  if (!next || typeof next !== 'object' || Array.isArray(next)) throw new TypeError(`${field} must be an object.`);
  const camera = {};
  const finite = (key, min = -Infinity, max = Infinity) => {
    if (next[key] === undefined) return;
    const value = next[key];
    if (typeof value !== 'number' || !Number.isFinite(value) || value < min || value > max) {
      throw new RangeError(`${field}.${key} must be a finite number${min === -Infinity ? '' : ` from ${min} to ${max}`}.`);
    }
    camera[key] = value;
  };
  finite('yaw');
  finite('pitch', -CAMERA_PITCH_LIMIT, CAMERA_PITCH_LIMIT);
  finite('distance', CAMERA_DISTANCE_RANGE[0], CAMERA_DISTANCE_RANGE[1]);
  if (next.target !== undefined) {
    if (!Array.isArray(next.target) || next.target.length !== 3
      || !next.target.every(value => typeof value === 'number' && Number.isFinite(value))) {
      throw new RangeError(`${field}.target must contain three finite coordinates.`);
    }
    camera.target = next.target.slice();
  }
  return camera;
}

export function createSimulatorView({ document, window, controller, isActive = () => true,
  createRenderer = createWebGLRenderer }) {
  const canvas = document.getElementById('simCanvas');
  const status = document.getElementById('simPoseStatus');
  const summary = document.getElementById('simCandidateSummary');
  const acceptedText = document.getElementById('simAcceptedPose');
  const pattern = document.getElementById('simPattern');
  const play = document.getElementById('simPlay');
  const pointerMode = document.getElementById('simPointerMode');
  const translationFrame = document.getElementById('simTranslationFrame');
  const frameOf = () => resolveTranslationFrame(translationFrame.value);
  for (const axis of TRANSLATION_AXES) {
    document.documentElement?.style?.setProperty?.(`--axis-${axis}`, axisCssColor(SCENE_COLORS[axis]));
  }
  const markers = document.getElementById('simMarkers');
  const traces = document.getElementById('simTraces');
  const overlayInputs = OVERLAY_NAMES.map(name => [name, document.getElementById(overlayInputId(name))]);
  const reachSamples = document.getElementById('simReachabilitySamples');
  const reachSlice = document.getElementById('simReachabilitySlice');
  const reachSliceZ = document.getElementById('simReachabilitySliceZ');
  const reachStatus = document.getElementById('simReachabilityStatus');
  const loadsInput = document.getElementById(overlayInputId('loads'));
  const loadsHint = document.getElementById('simLoadsHint');
  // The sample-count choices are the shared workspace sampling presets.
  reachSamples.innerHTML = REACHABILITY_SAMPLE_COUNTS
    .map(count => `<option value="${count}">${count.toLocaleString('en-US')}</option>`).join('');
  const renderer = createRenderer(canvas, { window, onContextChange: () => show(controller.getState()) });
  const synced = new WeakMap();
  let camera = { yaw: 0.7, pitch: 0.38, distance: 600, target: [0, 0, 100] };
  // The layout-centred target; the view recentres only when this changes (a
  // layout with a new home height), so zooming and panning survive pose updates.
  let centre = null;
  let drag = null;
  let previousFrame = null;
  let frameHandle = null;
  let disposed = false;

  const describePose = pose => pose ? `X ${fmt(pose.x)}, Y ${fmt(pose.y)}, Z ${fmt(pose.z)} mm; `
    + `Rx ${fmt(radToDeg(pose.rx))}, Ry ${fmt(radToDeg(pose.ry))}, Rz ${fmt(radToDeg(pose.rz))}°` : 'None';

  function show(state) {
    const layoutCentre = state.layout ? [0, 0, state.layout.homeHeight / 2] : null;
    if (layoutCentre && layoutCentre.some((value, k) => value !== centre?.[k])) {
      centre = layoutCentre;
      camera.target = layoutCentre.slice();
    }
    summary.textContent = state.layout
      ? `${state.source?.kind === 'candidate' ? `Candidate ${state.source.candidateId}` : state.source?.kind || 'Layout'} · ${state.layout.topology || 'free'} · home ${fmt(state.layout.homeHeight)} mm`
      : 'Select an optimizer candidate or import a layout to simulate.';
    if (renderer.contextLost) {
      status.textContent = 'WebGL2 context lost. The scene redraws when the browser restores it; pose requests are still evaluated.';
      status.classList.toggle('error', true);
    } else if (state.assessment) {
      const reason = state.assessment.violations.map(v => `${v.type}${v.leg === undefined ? ''
        : Number.isInteger(v.otherLeg) ? ` (legs ${v.leg + 1} and ${v.otherLeg + 1})` : ` (leg ${v.leg + 1})`}`).join(', ');
      status.textContent = state.rejected
        ? `Rejected request: ${describePose(state.requested)}. ${reason || 'Pose failed the active evaluator.'} Accepted pose held.`
        : `Accepted request: ${describePose(state.accepted)}.`;
      status.classList.toggle('error', state.rejected);
    } else {
      status.textContent = renderer.available ? 'No layout loaded.' : renderer.error;
      status.classList.toggle('error', !renderer.available);
    }
    acceptedText.textContent = `Rendered pose: ${describePose(state.accepted)}`;
    syncPoseFields(state);
    play.textContent = state.animation.playing ? 'Pause' : 'Play';
    play.setAttribute('aria-pressed', String(state.animation.playing));
    // Snapshot loads and browser-save restores set these on the controller directly.
    markers.checked = state.markers;
    traces.checked = state.tracesEnabled;
    for (const [name, input] of overlayInputs) input.checked = state.overlays[name];
    // The loads toggle needs a payload or external load to solve against.
    const loadable = hasLoad(state.loadModel);
    loadsInput.disabled = !loadable;
    loadsHint.hidden = loadable;
    showReachability(state);
    if (renderer.available) renderer.render(state, camera);
  }

  // Animation ticks notify every frame; a pose field the user is typing into
  // keeps its text unless an invalid entry forces the requested value back.
  function syncPoseFields(state, force = false) {
    const local = translationInFrame(state.requested, frameOf());
    for (const axis of POSE_AXES) {
      const index = TRANSLATION_AXES.indexOf(axis);
      const value = index >= 0 ? local[index] : displayValue(axis, state.requested[axis]);
      const slider = axisSlider(document, axis);
      syncInput(document, axisInput(document, axis), String(fmt(value)), synced, force);
      syncInput(document, slider, String(clamp(value, Number(slider.min), Number(slider.max))), synced, force);
    }
  }

  // The cloud settings follow the controller. The status counts evaluated
  // samples only; it never describes a continuous envelope. A rejected entry
  // stays reported until the next accepted cloud change.
  function showReachability(state, force = false) {
    const { sampleCount, mode, sliceZ } = state.reachability;
    reachSamples.value = String(sampleCount);
    reachSlice.checked = mode === 'slice';
    reachSliceZ.disabled = mode !== 'slice';
    syncInput(document, reachSliceZ, displayText(sliceZ), synced, force);
    const cloud = state.reachabilityCloud;
    let message;
    if (!state.overlays.reachabilityCloud) message = 'Reachability cloud off.';
    else if (!cloud) message = 'Load a layout to sweep reachability.';
    else if (cloud.error) message = `Reachability sweep stopped after ${cloud.points.length} samples: ${cloud.error}`;
    else {
      const reachable = cloud.points.filter(point => point.reachable).length;
      const { rx, ry, rz } = cloud.orientation;
      message = `${cloud.points.length < cloud.total ? 'Sweeping' : 'Swept'} ${cloud.points.length} of ${cloud.total} samples`
        + `${mode === 'slice' ? ` on the Z ${fmt(sliceZ)} mm plane` : ''} at Rx ${fmt(radToDeg(rx))}, Ry ${fmt(radToDeg(ry))},`
        + ` Rz ${fmt(radToDeg(rz))}°: ${reachable} reachable. Evaluated samples only, not a continuous envelope.`;
    }
    if (reachabilityError) message = `${reachabilityError} ${message}`;
    if (reachStatus.textContent !== message) reachStatus.textContent = message;
    reachStatus.classList.toggle('error', Boolean(reachabilityError || cloud?.error));
  }
  let reachabilityError = null;

  const unsubscribe = controller.subscribe(show);
  if (!renderer.available) {
    document.getElementById('simRendererError').textContent = renderer.error;
    document.getElementById('simRendererError').hidden = false;
  }

  // Controller calls throw on invalid input (no layout, bad speed); surface the
  // message in the pose status instead of leaving an uncaught error and a dead control.
  function guarded(action) {
    return (...args) => {
      try { action(...args); }
      catch (error) {
        status.textContent = error.message;
        status.classList.add('error');
        syncPoseFields(controller.getState(), true);
      }
    };
  }
  // In the platform frame a translation field holds platform-frame coordinates,
  // mapped back to the base frame at the requested orientation. A rotation edit
  // there keeps the platform origin in place instead of re-reading the
  // platform-frame translation at the new orientation, which would swing it.
  const requestFields = guarded(changed => {
    const fields = Object.fromEntries(POSE_AXES.map(axis => [axis,
      modelValue(axis, Number(axisInput(document, axis).value))]));
    for (const axis of POSE_AXES) markCommitted(axisInput(document, axis), synced);
    let pose = fields;
    if (frameOf() === 'platform') {
      const [x, y, z] = TRANSLATION_AXES.includes(changed)
        ? translationFromFrame([fields.x, fields.y, fields.z], fields, 'platform')
        : TRANSLATION_AXES.map(axis => controller.getState().requested[axis]);
      pose = { ...fields, x, y, z };
    }
    controller.requestPose(pose);
  });
  for (const axis of POSE_AXES) {
    axisInput(document, axis).addEventListener('change', () => requestFields(axis));
    axisSlider(document, axis).addEventListener('input', () => {
      axisInput(document, axis).value = axisSlider(document, axis).value;
      requestFields(axis);
    });
  }
  // Switching frames only changes how the fields read the same pose.
  translationFrame.addEventListener('change', () => syncPoseFields(controller.getState(), true));
  document.getElementById('simResetPose').addEventListener('click', guarded(() => controller.requestPose(HOME_POSE)));
  document.getElementById('simResetCamera').addEventListener('click', () => {
    camera = { yaw: 0.7, pitch: 0.38, distance: 600, target: centre ? centre.slice() : camera.target };
    if (renderer.available) renderer.render(controller.getState(), camera);
  });
  markers.addEventListener('change', () => controller.setMarkers(markers.checked));
  traces.addEventListener('change', () => controller.setTraces(traces.checked));
  for (const [name, input] of overlayInputs) {
    input.addEventListener('change', () => controller.setOverlays({ [name]: input.checked }));
  }
  function changeReachability(patch) {
    try {
      controller.setReachabilityCloud(patch());
      reachabilityError = null;
    } catch (error) { reachabilityError = error.message; }
    showReachability(controller.getState(), true);
  }
  reachSamples.addEventListener('change', () => changeReachability(() => ({ sampleCount: Number(reachSamples.value) })));
  reachSlice.addEventListener('change', () => changeReachability(() => ({ mode: reachSlice.checked ? 'slice' : 'cloud' })));
  // An empty field is an error, never 0.
  reachSliceZ.addEventListener('change', () => changeReachability(() => {
    const text = reachSliceZ.value.trim();
    return { sliceZ: text === '' ? NaN : Number(text) };
  }));
  document.getElementById('simClearTrace').addEventListener('click', () => controller.clearTrace());
  pattern.addEventListener('change', guarded(() => controller.setAnimation(pattern.value, false)));
  play.addEventListener('click', guarded(() => {
    const state = controller.getState();
    controller.setAnimation(pattern.value, !state.animation.playing,
      { speed: Number(document.getElementById('simSpeed').value) });
  }));
  document.getElementById('simSpeed').addEventListener('change', guarded(event => {
    const state = controller.getState();
    controller.setAnimation(state.animation.pattern, state.animation.playing, { speed: Number(event.target.value) });
  }));

  canvas.addEventListener('pointerdown', event => {
    // A right click or a second touch must not reset the drag origin.
    if ((event.button ?? 0) !== 0 || event.isPrimary === false) return;
    drag = { x: event.clientX, y: event.clientY, pan: Boolean(event.shiftKey) };
    canvas.setPointerCapture?.(event.pointerId);
    event.preventDefault?.();
  });
  canvas.addEventListener('pointermove', event => {
    if (!drag) return;
    const dx = event.clientX - drag.x, dy = event.clientY - drag.y;
    drag = { ...drag, x: event.clientX, y: event.clientY };
    if (drag.pan) {
      // Shift+drag pans in either mouse mode: the point under the cursor follows it.
      const { height } = viewport();
      if (height > 0) {
        const frame = cameraFrame(camera);
        const perPixel = 2 * camera.distance * VIEW_HALF_TANGENT / height;
        camera.target = camera.target.map((value, k) => value - frame.right[k] * dx * perPixel + frame.up[k] * dy * perPixel);
        if (renderer.available) renderer.render(controller.getState(), camera);
      }
    } else if (pointerMode.value === 'platform') {
      const state = controller.getState();
      if (state.layout) controller.requestPose(stepTranslation(state.requested, [dx * 0.35, -dy * 0.35, 0], frameOf()),
        { source: 'pointer' });
    } else {
      camera.yaw += dx * 0.006;
      camera.pitch = clamp(camera.pitch + dy * 0.006, -CAMERA_PITCH_LIMIT, CAMERA_PITCH_LIMIT);
      if (renderer.available) renderer.render(controller.getState(), camera);
    }
    event.preventDefault?.();
  });
  const releaseDrag = () => { drag = null; };
  canvas.addEventListener('pointerup', releaseDrag);
  canvas.addEventListener('pointercancel', releaseDrag);
  // The canvas box in CSS pixels; a canvas without layout reports zero size.
  function viewport() {
    const box = canvas.getBoundingClientRect?.();
    return { left: box?.left ?? 0, top: box?.top ?? 0,
      width: box?.width || canvas.clientWidth || 0, height: box?.height || canvas.clientHeight || 0 };
  }
  // Depth (mm along the view direction) of the drawn line nearest the cursor
  // within ZOOM_PICK_RADIUS_PX, or null over empty space.
  function depthUnderCursor(clientX, clientY, { left, top, width, height }, frame) {
    const toPage = ([x, y]) => [left + (x + 1) / 2 * width, top + (1 - y) / 2 * height];
    let best = null;
    for (const line of buildSceneGeometry(controller.getState()).lines) {
      const segment = projectSegment(line.from, line.to, camera, width, height, 0, frame);
      if (!segment) continue;
      const [a, b] = segment.map(toPage);
      const run = [b[0] - a[0], b[1] - a[1]];
      const length2 = run[0] * run[0] + run[1] * run[1];
      const s = length2 > 0 ? clamp(((clientX - a[0]) * run[0] + (clientY - a[1]) * run[1]) / length2, 0, 1) : 0;
      const gap = Math.hypot(a[0] + run[0] * s - clientX, a[1] + run[1] * s - clientY);
      if (gap > ZOOM_PICK_RADIUS_PX || (best && gap >= best.gap)) continue;
      // Depth varies with 1/depth across the screen; interpolate it that way.
      const depths = [line.from, line.to].map(point => Math.max(vectorDot(vectorSub(point, frame.eye), frame.forward), 1));
      best = { gap, depth: 1 / ((1 - s) / depths[0] + s / depths[1]) };
    }
    return best?.depth ?? null;
  }
  // The wheel zooms toward what is under the cursor and keeps it there: the
  // nearest drawn line's point, or over empty space the point on the plane
  // through the target facing the camera.
  canvas.addEventListener('wheel', event => {
    const distance = clamp(camera.distance * Math.exp(event.deltaY * 0.001), CAMERA_DISTANCE_RANGE[0], CAMERA_DISTANCE_RANGE[1]);
    const box = viewport();
    const { left, top, width, height } = box;
    if (width > 0 && height > 0 && Number.isFinite(event.clientX) && Number.isFinite(event.clientY)) {
      const frame = cameraFrame(camera);
      const x = (event.clientX - left) / width * 2 - 1, y = 1 - (event.clientY - top) / height * 2;
      const depth = depthUnderCursor(event.clientX, event.clientY, box, frame) ?? camera.distance;
      // The cursor ray reaches this point at that depth.
      const pointed = frame.eye.map((value, k) => value + depth * (frame.forward[k]
        + frame.right[k] * x * VIEW_HALF_TANGENT * width / height + frame.up[k] * y * VIEW_HALF_TANGENT));
      const ratio = distance / camera.distance;
      camera.target = pointed.map((value, k) => value + (camera.target[k] - value) * ratio);
    }
    camera.distance = distance;
    if (renderer.available) renderer.render(controller.getState(), camera);
    event.preventDefault?.();
  }, { passive: false });

  function keydown(event) {
    if (!isActive() || /^(INPUT|TEXTAREA|SELECT)$/.test(event.target?.tagName || '')) return;
    const state = controller.getState();
    if (!state.layout) return;
    const pose = { ...state.requested };
    const move = [0, 0, 0];
    const step = event.shiftKey ? 2 : 1;
    let handled = true;
    switch (event.key) {
      case 'ArrowLeft': move[0] -= step; break;
      case 'ArrowRight': move[0] += step; break;
      case 'ArrowUp': move[1] += step; break;
      case 'ArrowDown': move[1] -= step; break;
      case 'PageUp': move[2] += step; break;
      case 'PageDown': move[2] -= step; break;
      case 'q': case 'Q': pose.rz -= degToRad(step); break;
      case 'e': case 'E': pose.rz += degToRad(step); break;
      case 'w': case 'W': pose.rx += degToRad(step); break;
      case 's': case 'S': pose.rx -= degToRad(step); break;
      case 'a': case 'A': pose.ry -= degToRad(step); break;
      case 'd': case 'D': pose.ry += degToRad(step); break;
      default: handled = false;
    }
    if (handled) {
      controller.requestPose(stepTranslation(pose, move, frameOf()), { source: 'keyboard' });
      event.preventDefault?.();
    }
  }
  document.addEventListener('keydown', keydown);

  function frame(timestamp) {
    if (disposed) return;
    const delta = previousFrame === null ? 0 : Math.min((timestamp - previousFrame) / 1000, 0.1);
    previousFrame = timestamp;
    if (isActive()) {
      const state = controller.getState();
      if (state.animation.playing) controller.tick(delta);
      if (document.getElementById('simGamepad').checked && state.layout) {
        const pad = Array.from(window.navigator?.getGamepads?.() || []).find(Boolean);
        if (pad) {
          const axis = index => Math.abs(pad.axes?.[index] || 0) < 0.15 ? 0 : pad.axes[index];
          const button = index => pad.buttons?.[index]?.value || 0;
          const movement = [axis(0), axis(1), axis(2), axis(3), button(7) - button(6), button(5) - button(4)];
          if (movement.some(Boolean)) {
            const moved = stepTranslation(state.requested, [movement[0] * 25 * delta,
              -movement[1] * 25 * delta, movement[4] * 25 * delta], frameOf());
            const next = { ...moved,
              rx: state.requested.rx + movement[2] * 0.3 * delta,
              ry: state.requested.ry - movement[3] * 0.3 * delta,
              rz: state.requested.rz + movement[5] * 0.3 * delta };
            controller.requestPose(next, { source: 'gamepad' });
          }
        }
      }
    }
    frameHandle = window.requestAnimationFrame(frame);
  }
  if (typeof window.requestAnimationFrame === 'function') frameHandle = window.requestAnimationFrame(frame);
  window.addEventListener?.('resize', () => {
    if (renderer.available) renderer.render(controller.getState(), camera);
  });

  return { renderer, getCamera: () => structuredClone(camera),
    getTranslationFrame: frameOf,
    setTranslationFrame(frame) {
      translationFrame.value = resolveTranslationFrame(frame);
      syncPoseFields(controller.getState(), true);
    },
    setCamera(next) {
      camera = { ...camera, ...parseCamera(next) };
      if (renderer.available) renderer.render(controller.getState(), camera);
    },
    render() { show(controller.getState()); },
    dispose() { disposed = true; unsubscribe(); if (frameHandle != null) window.cancelAnimationFrame?.(frameHandle);
      document.removeEventListener?.('keydown', keydown); renderer.dispose(); } };
}
