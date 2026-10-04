import { METRICS } from '../contracts.js';
import { failureCategories, isPassing, rankCandidates } from '../io/results.js';

const AXES = ['torque', 'speedDemand', 'coverage', 'payloadCoverage', 'conditioningQuality', 'dexterity',
  'stiffness', 'physicalStiffness', 'loadSharing', 'loadBalance', 'isotropy', 'limitMargin', 'fatigue', 'footprint',
  'directionalStiffness'];
const label = key => `${key.replace(/([A-Z])/g, ' $1')} (${METRICS[key].unit})`;
const printable = value => Number.isFinite(value) ? Number(value).toPrecision(4) : 'unavailable';
const escapeHtml = value => String(value).replace(/[&<>"']/g, char => ({
  '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
})[char]);

export function chartData(candidates, selectedId, xKey = 'torque', yKey = 'speedDemand') {
  return {
    datasets: [
      { label: 'Passing', passing: true, color: '#4dabf7' },
      { label: 'Diagnostic', passing: false, color: '#ffad5c' },
    ].map(group => {
      const data = candidates.filter(candidate => isPassing(candidate) === group.passing
        && Number.isFinite(candidate[xKey]) && Number.isFinite(candidate[yKey]))
        .map(candidate => ({ x: candidate[xKey], y: candidate[yKey], id: candidate.layout.id }));
      return {
        label: group.label,
        data,
        backgroundColor: group.color,
        borderColor: data.map(point => String(point.id) === String(selectedId) ? '#fff' : group.color),
        borderWidth: data.map(point => String(point.id) === String(selectedId) ? 2.5 : 0),
        pointRadius: data.map(point => String(point.id) === String(selectedId) ? 8 : 6),
        pointHoverRadius: 10,
      };
    }),
  };
}

// Keep coincident candidates separately clickable without changing their metric values.
const spreadCoincidentPoints = {
  id: 'spreadCoincidentCandidates',
  afterDatasetsUpdate(chart) {
    const groups = new Map();
    chart.data.datasets.forEach((dataset, datasetIndex) => {
      const elements = chart.getDatasetMeta(datasetIndex).data;
      dataset.data.forEach((point, index) => {
        const key = `${point.x}:${point.y}`;
        if (!groups.has(key)) groups.set(key, []);
        groups.get(key).push(elements[index]);
      });
    });
    for (const elements of groups.values()) {
      if (elements.length < 2) continue;
      elements.forEach((point, index) => {
        const angle = 2 * Math.PI * index / elements.length;
        point.x += 9 * Math.cos(angle);
        point.y += 9 * Math.sin(angle);
      });
    }
  },
};

export function createResultsView(document, onSelect, ChartClass = globalThis.Chart) {
  const candidateSelect = document.getElementById('candidateSelect');
  const xAxis = document.getElementById('chartXAxis');
  const yAxis = document.getElementById('chartYAxis');
  const canvas = document.getElementById('paretoChart');
  const summary = document.getElementById('candidateSummary');
  let candidates = [];
  let selectedId = null;

  for (const [element, initial] of [[xAxis, 'torque'], [yAxis, 'speedDemand']]) {
    element.innerHTML = AXES.map(key => `<option value="${key}">${escapeHtml(label(key))}</option>`).join('');
    element.value = initial;
  }

  const chart = new ChartClass(canvas, {
    type: 'scatter',
    data: chartData([], null),
    plugins: [spreadCoincidentPoints],
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: { mode: 'nearest', intersect: true },
      onClick(_event, elements, instance) {
        const hit = elements[0];
        if (hit) choose(instance.data.datasets[hit.datasetIndex].data[hit.index].id);
      },
      plugins: {
        legend: { position: 'top', align: 'end', labels: { color: '#ddd', usePointStyle: true } },
        tooltip: {
          callbacks: {
            title: items => `Candidate ${items[0]?.raw.id ?? ''}`,
            label: item => [
              `${label(xAxis.value)}: ${printable(item.raw.x)}`,
              `${label(yAxis.value)}: ${printable(item.raw.y)}`,
              item.dataset.label,
            ],
          },
        },
      },
      scales: {
        x: {
          type: 'linear',
          grace: '8%',
          title: { display: true, text: label(xAxis.value), color: '#ddd' },
          ticks: { color: '#aaa' }, grid: { color: '#303030' }, border: { color: '#777' },
        },
        y: {
          type: 'linear',
          grace: '8%',
          title: { display: true, text: label(yAxis.value), color: '#ddd' },
          ticks: { color: '#aaa' }, grid: { color: '#303030' }, border: { color: '#777' },
        },
      },
    },
  });

  function redraw() {
    chart.data = chartData(candidates, selectedId, xAxis.value, yAxis.value);
    chart.options.scales.x.title.text = label(xAxis.value);
    chart.options.scales.y.title.text = label(yAxis.value);
    canvas.setAttribute('aria-label', `Candidate chart, ${label(xAxis.value)} versus ${label(yAxis.value)}. Select a candidate from the list for keyboard access.`);
    chart.update();
    const selected = candidates.find(candidate => String(candidate.layout.id) === String(selectedId));
    if (!selected) {
      summary.textContent = 'No completed candidate is available.';
      return;
    }
    const flags = selected.feasibility || {};
    const failures = failureCategories(selected);
    const reference = selected.layout.seedOrigin === 'reference' ? selected.referenceDiagnostics : null;
    const details = reference
      ? ` Exact reference. Bounds conflicts: ${reference.boundsConflicts.map(item => item.field).join(', ') || 'none'}. Home violations: ${reference.homePoseViolations.map(item => `${item.type} leg ${item.leg + 1}`).join(', ') || 'none'}.`
      : '';
    const capacity = selected.servoCapacity;
    const capacityText = capacity?.hasRatings
      ? ` Servo capacity ${capacity.status} (${capacity.policy}${capacity.policy === 'advisory' && ['above', 'unavailable', 'outOfDomain'].includes(capacity.status) ? ' warning' : ''}); peak ${capacity.peak?.status ?? capacity.status}, continuous RMS ${capacity.continuous?.status ?? 'unrated'}; worst headroom ${printable(capacity.worstHeadroomFraction)} of rating.`
      : '';
    const cycle = selected.cycle;
    const cycleText = cycle?.trajectoryId ? ` Cycle ${cycle.trajectorySource === 'supplied' ? 'trajectory' : 'single-axis'} ${cycle.trajectoryId}; ${cycle.massModel === 'rigid-body' ? 'rigid-body mass properties' : 'legacy centered point mass'}.` : '';
    const sharing = cycle?.loadSharing;
    const sharingText = sharing ? ` Rod load sharing ${sharing.status === 'available' ? `score ${printable(sharing.balanceScore)} (worst sample ${printable(sharing.worstShareRatio)})` : sharing.status}.` : '';
    const physical = selected.compliance;
    const physicalText = physical?.status === 'available' ? ` Physical stiffness ${printable(physical.minScaledStiffnessNPerM)} N/m (L = ${printable(physical.characteristicLengthM * 1000)} mm).`
      : physical?.status === 'singular' ? ' Physical stiffness singular.' : '';
    const support = selected.workspace?.payloadSupport;
    const supportText = support ? ` Static payload support ${support.status} (${support.rating} rating, ${support.policy}); capacity-qualified coverage ${printable(support.qualifiedCoverage)}%.` : '';
    const sampled = cycle?.sampling;
    const samplingText = sampled ? ` Cycle sampling ${sampled.status} after ${sampled.evaluatedSamples} samples${sampled.status === 'budget-limited' ? ' (inconclusive)' : ''}.` : '';
    summary.textContent = `Candidate ${selectedId}: ${isPassing(selected) ? 'passing' : `diagnostic (${failures.join(', ') || 'requirements failed'})`}. Feasible sampled coverage ${printable(selected.coverage)}%. Home ${flags.homePoseSatisfied ? 'pass' : 'fail'}; workspace ${flags.sampledWorkspaceSatisfied ? 'pass' : 'fail'}; cycle ${flags.cycleSatisfied ? 'pass' : 'fail'}. Torque ${printable(selected.torque)} N m; speed ${printable(selected.speedDemand)} rad/s.${cycleText}${samplingText}${sharingText}${physicalText}${supportText}${capacityText}${details}`;
  }

  function choose(id) {
    const candidate = candidates.find(item => String(item.layout.id) === String(id));
    if (!candidate) return;
    selectedId = candidate.layout.id;
    candidateSelect.value = String(selectedId);
    redraw();
    onSelect(candidate);
  }

  candidateSelect.addEventListener('change', () => choose(candidateSelect.value));
  xAxis.addEventListener('change', redraw);
  yAxis.addEventListener('change', redraw);

  return {
    select: choose,
    render(retained, initialId) {
      candidates = rankCandidates(retained);
      selectedId = initialId ?? candidates[0]?.layout.id ?? null;
      candidateSelect.innerHTML = candidates.map(candidate => {
        const id = escapeHtml(candidate.layout.id);
        const state = isPassing(candidate) ? 'passing' : `diagnostic: ${failureCategories(candidate).join(', ')}`;
        const prefix = candidate.layout.seedOrigin === 'reference' ? 'Exact reference' : `Candidate ${id}`;
        return `<option value="${id}">${escapeHtml(prefix)} — ${escapeHtml(state)}</option>`;
      }).join('');
      candidateSelect.disabled = !candidates.length;
      candidateSelect.value = selectedId === null ? '' : String(selectedId);
      redraw();
    },
    clear() { this.render([], null); },
  };
}
