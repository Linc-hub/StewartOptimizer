import { radToDeg } from '../math.js';
import { SCHEMA_VERSION, MODEL_VERSION, METRICS } from '../contracts.js';

const finite = value => Number.isFinite(value) ? value : null;
const demand = value => Number.isFinite(value) ? value : Infinity;
const coverage = value => Number.isFinite(value) ? value : -Infinity;

export function failureCategories(evaluation) {
  const flags = evaluation?.feasibility || {};
  const categories = new Set(flags.failedCategories || flags.failureCategories || []);
  if (flags.homePoseSatisfied === false) categories.add('home');
  if (flags.sampledWorkspaceSatisfied === false) categories.add('workspace');
  if (flags.cycleSatisfied === false || evaluation?.cycle?.valid === false) categories.add('cycle');
  if (flags.conditionSatisfied === false) categories.add('conditioning');
  if (flags.collisionSatisfied === false) categories.add('collision');
  if (flags.cycleConvergenceSatisfied === false) categories.add('cycle_convergence');
  if (flags.servoCapacityEnforced && flags.servoCapacitySatisfied === false) categories.add('servo_capacity');
  if (flags.payloadSupportEnforced && flags.payloadSupportSatisfied === false) categories.add('payload_support');
  return [...categories].sort();
}

export function isPassing(evaluation) {
  if (failureCategories(evaluation).length) return false;
  return evaluation?.feasibility?.passing !== false;
}

function compareAvailableDemand(a, b) {
  const torqueA = demand(a.torque);
  const torqueB = demand(b.torque);
  if (torqueA !== torqueB) return torqueA < torqueB ? -1 : 1;
  const speedA = demand(a.speedDemand);
  const speedB = demand(b.speedDemand);
  return speedA === speedB ? 0 : speedA < speedB ? -1 : 1;
}

// Constraint rank precedes numerical Pareto ranking and selected-result ordering.
export function compareFeasibility(a, b) {
  const passingA = isPassing(a);
  const passingB = isPassing(b);
  if (passingA !== passingB) return passingA ? -1 : 1;
  if (passingA) return 0;
  const failures = failureCategories(a).length - failureCategories(b).length;
  if (failures) return failures;
  const feasibleCoverage = coverage(b.coverage) - coverage(a.coverage);
  if (feasibleCoverage) return feasibleCoverage;
  return compareAvailableDemand(a, b);
}

// Stable policy for the chart, initial selection, exports, and progress summaries.
export function compareCandidates(a, b) {
  const constraintRank = compareFeasibility(a, b);
  if (constraintRank) return constraintRank;
  const passingA = isPassing(a);
  if (!passingA) return String(a.layout?.id ?? '').localeCompare(String(b.layout?.id ?? ''), undefined, { numeric: true });
  const load = compareAvailableDemand(a, b);
  if (load) return load;
  const quality = coverage(b.conditioningQuality ?? b.dexterity) - coverage(a.conditioningQuality ?? a.dexterity);
  if (quality) return quality;
  return String(a.layout?.id ?? '').localeCompare(String(b.layout?.id ?? ''), undefined, { numeric: true });
}

export function rankCandidates(candidates = []) {
  return candidates.slice().sort(compareCandidates);
}

export function selectBest(pareto, fitness) {
  // The retained population is authoritative: a numerical Pareto front may omit
  // a feasible candidate while keeping a high-scoring diagnostic candidate.
  return rankCandidates(fitness?.length ? fitness : pareto || [])[0];
}

export function layoutToJSON(layout, toDegrees = radToDeg) {
  return {
    schema_version: SCHEMA_VERSION,
    model_version: MODEL_VERSION,
    id: layout.id ?? null,
    seed_origin: layout.seedOrigin ?? null,
    reference_diagnostics: layout.referenceDiagnostics ?? null,
    topology: layout.topology ?? 'free',
    topology_parameters: layout.topologyParameters ?? null,
    mounting: layout.mounting ?? null,
    migration: layout.migration ?? null,
    base_anchors: layout.baseAnchors.map(a => a.slice()),
    platform_anchors: layout.platformAnchors.map(a => a.slice()),
    beta_angles: layout.betaAngles.slice(),
    horn_length: layout.hornLength,
    rod_length: layout.rodLength,
    servo_range: layout.servoRangeDeg?.slice() ?? layout.servoRangeRad.map(toDegrees),
    home_height: layout.homeHeight,
  };
}

function metricValues(evaluation, json = false) {
  return Object.fromEntries(Object.entries(METRICS).map(([key, info]) => [json ? info.json : key, finite(evaluation[key])]));
}

function conditioningSummary(evaluation) {
  if (!evaluation.conditioning) return null;
  const { jacobianRows: _rows, ...home } = evaluation.conditioning.home ?? {};
  return JSON.parse(JSON.stringify({ ...evaluation.conditioning, home }));
}

export function resultFeasibility(evaluation) {
  return {
    ...evaluation.feasibility,
    passing: isPassing(evaluation),
    failedCategories: failureCategories(evaluation),
  };
}

export function displayResult(evaluation) {
  return {
    id: evaluation.layout?.id ?? null,
    diagnostic: !isPassing(evaluation),
    reference_diagnostics: evaluation.referenceDiagnostics ?? null,
    metrics: metricValues(evaluation),
    layout: layoutToJSON(evaluation.layout),
    cycle: evaluation.cycle,
    conditioning: conditioningSummary(evaluation),
    servo_capacity: evaluation.servoCapacity ?? null,
    actuator_utilization: evaluation.actuatorUtilization ?? null,
    physical_stiffness: evaluation.compliance ?? null,
    payload_support: evaluation.workspace?.payloadSupport ?? null,
    ...(evaluation.boundsExcursions ? { bounds_excursions: evaluation.boundsExcursions } : {}),
    feasibility: resultFeasibility(evaluation),
    constraint_policy: evaluation.workspace?.constraintPolicy,
    workspace_stats: evaluation.workspace?.stats,
    workspace_counts: evaluation.workspace?.counts,
    workspace_samples: evaluation.workspace?.samples,
  };
}

export function exportResult(evaluation, run) {
  return {
    ...layoutToJSON(evaluation.layout),
    diagnostic: !isPassing(evaluation),
    reference_diagnostics: evaluation.referenceDiagnostics ?? null,
    metadata: metricValues(evaluation, true),
    cycle: evaluation.cycle ?? null,
    conditioning: conditioningSummary(evaluation),
    servo_capacity: evaluation.servoCapacity ?? null,
    actuator_utilization: evaluation.actuatorUtilization ?? null,
    physical_stiffness: evaluation.compliance ?? null,
    payload_support: evaluation.workspace?.payloadSupport ?? null,
    ...(evaluation.boundsExcursions ? { bounds_excursions: evaluation.boundsExcursions } : {}),
    feasibility: resultFeasibility(evaluation),
    constraint_policy: evaluation.workspace?.constraintPolicy ?? null,
    workspace_counts: evaluation.workspace?.counts ?? null,
    workspace_stats: evaluation.workspace?.stats ?? null,
    run,
  };
}
