import { validatePhysicalRequirements } from '../model/requirements.js';
import { degToRad } from '../math.js';
import { DEFAULT_DESIGN_SPACE, DEFAULT_HORN_DIRECTION_MODE, DEFAULT_LEG_PAIRING_MODE, cloneLayout, createRandomLayout, finalizeLayout, mutateLayout,
  crossoverLayouts, validateDesignSpace, validateHornDirectionMode, validateLegPairingMode } from './layout-operators.js';
import { c3HornDirection, c3LegPairing } from './topology.js';
import { configurationSummary, retainConfigurations, searchedConfigurations } from './configurations.js';
import { dominates, fastNonDominatedSort, assignCrowdingDistance, tournamentSelect, selectFromFronts } from './nsga2.js';
import { evaluateLayout, evaluateCycle, computeFatigue } from './evaluate-layout.js';
import { estimateWork } from './budget.js';
import { selectBest, exportResult, layoutToJSON, isPassing } from '../io/results.js';
import { boundsExcursions, relaxedDesignSpace, RELAXATION_IMMIGRANT_SHARE, RELAXATION_STEPS, validateBoundsRelaxation }
  from './relaxation.js';
import { DEFAULT_TOPOLOGY, TOPOLOGIES, DEFAULT_BALL_JOINT_LIMIT_DEG, DEFAULT_LINK_CLEARANCE_MM } from '../contracts.js';
import { normalizeSampling } from '../workspace/sampling.js';
import { createRandom, normalizeSeed, RANDOM_ALGORITHM } from './random.js';
import { validateConditionLimit } from '../model/conditioning.js';
import { validateLinkClearance } from '../model/collision.js';
import { importLayout } from '../io/layout-import.js';
import { evaluatePose } from '../model/pose.js';
import { initialPopulation, referenceBoundsConflicts, seedComposition } from './reference-seeding.js';
import { normalizeServoRatings, SERVO_RATING_KEYS } from '../model/servo-ratings.js';
import { normalizeObjectiveSet, objectiveDefinitions } from './objectives.js';
import { trajectoryFromRequirements, trajectoryIdentity, trajectorySummary } from '../model/trajectory.js';
import { normalizeMassProperties, RIGID_BODY_FIELDS } from '../model/mass-properties.js';
import { normalizeStiffnessModel } from '../model/compliance.js';
import { normalizePayloadSupport } from '../workspace/payload-support.js';
import { CYCLE_MODEL_VERSION } from '../contracts.js';
import { LOAD_SHARING_MODEL } from '../model/load-sharing.js';
import { DEFAULT_CYCLE_SAMPLING, LEGACY_CYCLE_SAMPLING, normalizeCycleSampling } from '../model/cycle-sampling.js';

// Runs saved before the physical stiffness became the default Full objective
// carry a stiffness model without use_as_objective and name the proxy in
// objectiveDefinitions; replay keeps the proxy they were ranked with.
function replayRequirements(settings) {
  const requirements = settings.requirements ?? {};
  const model = requirements.stiffness_model;
  const keys = Array.isArray(settings.objectiveDefinitions) ? settings.objectiveDefinitions.map(definition => definition?.key) : [];
  if (model && typeof model === 'object' && !Array.isArray(model) && !('use_as_objective' in model) && keys.includes('stiffness')) {
    return { ...requirements, stiffness_model: { ...model, use_as_objective: false } };
  }
  return requirements;
}

// Owns run state and population lifecycle. Numerical work and browser I/O live elsewhere.
export class Optimizer {
  static fromReplay(input, { onProgress } = {}) {
    const { sourceRun } = importLayout(input);
    const settings = sourceRun?.effective_settings;
    if (!settings || typeof settings !== 'object' || Array.isArray(settings)) {
      throw new TypeError('run.effective_settings is required to replay an exported result.');
    }
    if (settings.randomAlgorithm !== RANDOM_ALGORITHM) {
      throw new RangeError(`run.effective_settings.randomAlgorithm must be ${RANDOM_ALGORITHM}.`);
    }
    return new Optimizer(replayRequirements(settings), {
      ...settings,
      // Full runs saved before solved load sharing optimized the directional proxy.
      objectiveSet: settings.objectiveSet === 'full'
        && settings.objectiveDefinitions?.some(definition => definition.key === 'loadBalance') ? 'full-v1' : settings.objectiveSet,
      // Runs saved before adaptive sampling used the fixed 64-sample schedule.
      cycleSampling: settings.cycleSampling ?? LEGACY_CYCLE_SAMPLING,
      // Runs saved before link collision checks had none; a zero clearance never collides.
      linkClearanceMm: settings.linkClearanceMm ?? 0,
      ranges: settings.bounds,
      referenceLayout: settings.reference_layout ?? null,
      onProgress,
    });
  }

  constructor(requirements = {}, {
    populationSize = 12,
    generations = 5,
    ranges = {},
    sampling = { strategy: 'halton' },
    cycleSampling = DEFAULT_CYCLE_SAMPLING,
    seed = 1,
    mutationRate = 0.35,
    objectiveSet = 'compact',
    designSpace = {},
    topology = DEFAULT_TOPOLOGY,
    hornDirection = DEFAULT_HORN_DIRECTION_MODE,
    legPairing = DEFAULT_LEG_PAIRING_MODE,
    compactness = false,
    boundsRelaxation = 0,
    referenceLayout = null,
    homeHeightBounds,
    ballJointLimitDeg,
    lowerBallJointLimitDeg,
    upperBallJointLimitDeg,
    conditionLimit = null,
    linkClearanceMm,
    ballJointClamp = false,
    servoRatings,
    onProgress,
    onCheckpoint,
  } = {}) {
    if (!Number.isSafeInteger(populationSize) || populationSize < 4
        || !Number.isSafeInteger(generations) || generations < 1) {
      throw new RangeError('Population must be an integer >= 4 and generations an integer >= 1.');
    }
    this.onProgress = onProgress;
    this.onCheckpoint = onCheckpoint;
    this.requirements = requirements;
    this.populationSize = Math.max(4, populationSize);
    this.generations = Math.max(1, generations);
    this.ranges = ranges;
    this.seed = normalizeSeed(seed);
    this.sampling = normalizeSampling(sampling.strategy === 'halton'
      ? { ...sampling, sequenceStart: sampling.sequenceStart ?? this.seed } : sampling);
    this.random = createRandom(this.seed);
    this.cycleSampling = normalizeCycleSampling(cycleSampling);
    if (!Number.isFinite(mutationRate) || mutationRate < 0 || mutationRate > 1) {
      throw new RangeError('mutationRate must be a finite probability in [0, 1].');
    }
    this.mutationRate = mutationRate;
    this.objectiveSet = normalizeObjectiveSet(objectiveSet);
    const imported = referenceLayout == null ? null : importLayout(referenceLayout);
    this.referenceLayout = imported?.layout ?? null;
    this.referenceSourceRun = imported?.sourceRun ?? null;
    if (this.referenceLayout) topology = this.referenceLayout.topology;
    if (!TOPOLOGIES.includes(topology)) throw new Error(`topology must be one of ${TOPOLOGIES.join(', ')}.`);
    this.topology = topology;
    // A C3 reference's horn direction and leg pairing win over fixed modes, as
    // its topology does; `both` still searches both values from it. A reference
    // of another topology has neither, so the settings do not apply.
    const referenceTopology = this.referenceLayout?.topology;
    if (referenceTopology === 'c3_paired') {
      if (hornDirection !== 'both') hornDirection = c3HornDirection(this.referenceLayout.topologyParameters);
      if (legPairing !== 'both') legPairing = c3LegPairing(this.referenceLayout.topologyParameters);
    } else if (referenceTopology) {
      hornDirection = DEFAULT_HORN_DIRECTION_MODE;
      legPairing = DEFAULT_LEG_PAIRING_MODE;
    }
    this.hornDirection = validateHornDirectionMode(hornDirection, this.topology);
    this.legPairing = validateLegPairingMode(legPairing, this.topology);
    this.configurations = searchedConfigurations(this);
    this.ballJointLimitDeg = ballJointLimitDeg ?? requirements.ball_joint_max_deg ?? DEFAULT_BALL_JOINT_LIMIT_DEG;
    this.lowerBallJointLimitDeg = lowerBallJointLimitDeg ?? this.ballJointLimitDeg;
    this.upperBallJointLimitDeg = upperBallJointLimitDeg ?? this.ballJointLimitDeg;
    this.conditionLimit = validateConditionLimit(conditionLimit);
    this.linkClearanceMm = validateLinkClearance(linkClearanceMm ?? requirements.link_clearance_mm
      ?? DEFAULT_LINK_CLEARANCE_MM);
    this.ballJointClamp = ballJointClamp;
    this.servoRatingsInput = Object.fromEntries(SERVO_RATING_KEYS
      .filter(key => Object.hasOwn(servoRatings ?? {}, key) || Object.hasOwn(requirements, key))
      .map(key => [key, Object.hasOwn(servoRatings ?? {}, key) ? servoRatings[key] : requirements[key]]));
    // UI overrides own the per-servo keys they supply; curves, continuous/duration
    // ratings and actuator models present only in the requirements JSON are kept.
    if (Array.isArray(servoRatings?.per_servo_ratings) && Array.isArray(requirements.per_servo_ratings)) {
      this.servoRatingsInput.per_servo_ratings = servoRatings.per_servo_ratings.map((entry, index) => {
        const json = requirements.per_servo_ratings[index];
        return json == null ? entry : { ...json, ...entry };
      });
    }
    this.servoRatings = normalizeServoRatings(this.servoRatingsInput);
    this.payload = requirements.mass_kg ?? 0;
    const cycleInput = Object.fromEntries(['trajectory', ...RIGID_BODY_FIELDS]
      .filter(key => requirements[key] != null).map(key => [key, requirements[key]]));
    const { trajectory, source: trajectorySource } = trajectoryFromRequirements(requirements);
    this.trajectory = trajectory;
    this.trajectorySource = trajectorySource;
    // Stroke/frequency feed the fatigue heuristic; supplied trajectories use the largest translation.
    ({ stroke: this.stroke, frequency: this.frequency } = trajectorySummary(trajectory));
    this.cycleAxis = requirements.cycle_axis ?? 'z';

    // Requirement bounds win, then the headless designSpace option, then the defaults;
    // the same order as homeHeightBounds below.
    const hornBounds = requirements.horn_length_bounds_mm ?? designSpace.hornLengthBounds
      ?? DEFAULT_DESIGN_SPACE.hornLengthBounds;
    const rodBounds = requirements.rod_length_bounds_mm ?? designSpace.rodLengthBounds
      ?? DEFAULT_DESIGN_SPACE.rodLengthBounds;
    this.servoRangeDeg = requirements.servo_travel_bounds_deg || [-120, 120];

    this.designSpace = {
      ...DEFAULT_DESIGN_SPACE,
      ...designSpace,
      hornLengthBounds: hornBounds,
      rodLengthBounds: rodBounds,
      homeHeightBounds: homeHeightBounds ?? requirements.home_height_bounds_mm
        ?? designSpace.homeHeightBounds ?? DEFAULT_DESIGN_SPACE.homeHeightBounds,
    };
    validateDesignSpace(this.designSpace);
    if (typeof compactness !== 'boolean') throw new TypeError('compactness must be true or false.');
    this.compactness = compactness;
    this.boundsRelaxation = validateBoundsRelaxation(boundsRelaxation);
    this.relaxationShare = 0;
    this.relaxationHistory = [];

    validatePhysicalRequirements({ ...cycleInput, mass_kg: this.payload, cycle_mm: this.stroke,
      frequency_hz: this.frequency, cycle_axis: this.cycleAxis,
      ball_joint_max_deg: this.ballJointLimitDeg, servo_travel_bounds_deg: this.servoRangeDeg,
      horn_length_bounds_mm: hornBounds, rod_length_bounds_mm: rodBounds,
      home_height_bounds_mm: this.designSpace.homeHeightBounds });
    for (const value of [this.lowerBallJointLimitDeg, this.upperBallJointLimitDeg]) {
      if (!Number.isFinite(value) || value < 0 || value > 180) {
        throw new RangeError('Ball-joint limits must be finite angles from 0 to 180 degrees.');
      }
    }

    this.massProperties = normalizeMassProperties({ ...cycleInput, mass_kg: this.payload });
    this.stiffnessModel = normalizeStiffnessModel(requirements.stiffness_model);
    this.payloadSupport = normalizePayloadSupport(requirements.workspace_payload_support);
    this.objectiveVariant = { stiffnessMetric: this.stiffnessModel?.useAsObjective ? 'physicalStiffness' : 'stiffness',
      ...(this.compactness ? { footprint: true } : {}) };
    this.servoRangeRad = this.servoRangeDeg.map((deg) => degToRad(deg));
    this.referenceDiagnostics = null;
    if (this.referenceLayout) {
      const home = evaluatePose(this.referenceLayout,
        { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 }, {
          ballJointLimitDeg: this.ballJointLimitDeg,
          lowerBallJointLimitDeg: this.lowerBallJointLimitDeg,
          upperBallJointLimitDeg: this.upperBallJointLimitDeg,
          conditionLimit: this.conditionLimit,
          linkClearanceMm: this.linkClearanceMm,
          servoRangeRad: this.referenceLayout.servoRangeRad,
          mounting: this.referenceLayout.mounting,
          recordLegData: true,
        });
      this.referenceDiagnostics = {
        boundsConflicts: referenceBoundsConflicts(this.referenceLayout, this.designSpace, this.servoRangeRad),
        homePoseSatisfied: home.reachable,
        homePoseViolations: home.violations,
      };
      this.referenceLayout.referenceDiagnostics = this.referenceDiagnostics;
    }

    this.population = [];
    this.fitness = [];
    this.pareto = [];
    this.generation = 0;
    this.running = false;
    this.runStatus = 'idle';
    this.nextLayoutId = 1;
    this.selectedCandidateId = null;
  }

  // A run that searches several C3 configurations deals its fresh layouts out
  // in turn, so every configuration starts with an equal share.
  createRandomLayout() {
    const options = this.layoutOptions();
    if (this.configurations.length > 1) {
      Object.assign(options, this.configurations[this.freshLayouts++ % this.configurations.length]);
    }
    return createRandomLayout({ ...options, id: this.nextLayoutId++ });
  }

  // The space layouts are drawn and clamped in: the nominal design space,
  // widened by the current relaxation share in a run that allows it.
  activeDesignSpace() { return relaxedDesignSpace(this.designSpace, this.relaxationShare); }

  // After each generation without a passing retained candidate, widen the
  // bounds by another step, up to the run's boundsRelaxation share.
  updateRelaxation(evaluations, generation) {
    if (!(this.boundsRelaxation > 0)) return;
    const passing = evaluations.filter(isPassing).length;
    if (!passing && this.relaxationShare < this.boundsRelaxation) {
      this.relaxationShare = Math.min(this.boundsRelaxation, this.relaxationShare + this.boundsRelaxation / RELAXATION_STEPS);
      this.pendingImmigrants = Math.max(1, Math.ceil(this.populationSize * RELAXATION_IMMIGRANT_SHARE));
    }
    this.relaxationHistory.push({ generation, passing, share: this.relaxationShare });
  }

  // The relaxation record for a run that allows it, else null.
  relaxationSummary() {
    if (!(this.boundsRelaxation > 0)) return null;
    const relaxed = this.fitness.filter(evaluation => evaluation.boundsExcursions?.length).length;
    return { maxShare: this.boundsRelaxation, finalShare: this.relaxationShare,
      designSpace: this.activeDesignSpace(), outsideNominal: relaxed, history: this.relaxationHistory };
  }

  layoutOptions() { return { designSpace: this.activeDesignSpace(), servoRangeRad: this.servoRangeRad,
    servoRangeDeg: this.servoRangeDeg, topology: this.topology, hornDirection: this.hornDirection,
      legPairing: this.legPairing, random: this.random }; }
  finalizeLayout(layout) { return finalizeLayout(layout, this.layoutOptions()); }
  mutateLayout(layout) { return mutateLayout(layout, this.layoutOptions()); }
  crossoverLayouts(a, b) { return crossoverLayouts(a, b, this.layoutOptions()); }

  evaluationOptions() {
    return { ranges: this.ranges, signal: this.abortController?.signal,
      payload: this.payload, stroke: this.stroke, frequency: this.frequency, cycleAxis: this.cycleAxis,
      trajectory: this.trajectory, trajectorySource: this.trajectorySource, massProperties: this.massProperties,
      cycleSampling: this.cycleSampling,
      ballJointLimitDeg: this.ballJointLimitDeg,
      lowerBallJointLimitDeg: this.lowerBallJointLimitDeg,
      upperBallJointLimitDeg: this.upperBallJointLimitDeg,
      conditionLimit: this.conditionLimit,
      linkClearanceMm: this.linkClearanceMm,
      ballJointClamp: this.ballJointClamp,
      servoRangeRad: this.servoRangeRad, sampling: this.sampling,
      servoRatings: this.servoRatings, objectiveSet: this.objectiveSet,
      stiffnessModel: this.stiffnessModel,
      payloadSupport: this.payloadSupport, objectiveVariant: this.objectiveVariant,
      payloadSupport: this.payloadSupport };
  }

  evaluateLayout(layout) {
    const random = createRandom((this.seed ^ Math.imul(layout.id ?? 0, 0x9e3779b9)) >>> 0 || 1);
    let workspaceCompleted = 0;
    let extraCompleted = 0;
    const report = () => this.onProgress?.({
      completed: (this.completedPoseWork || 0) + workspaceCompleted + extraCompleted,
      total: this.workEstimate?.totalPoses ?? this.workEstimate?.posesPerLayout ?? workspaceCompleted + extraCompleted,
      budgeted: this.workEstimate?.totalPoses ?? null,
      generation: this.activeGeneration ?? this.generation,
    });
    return evaluateLayout(layout, { ...this.evaluationOptions(), random,
      onProgress: ({ completed, total }) => this.onProgress?.({
        completed: (this.completedPoseWork || 0) + (workspaceCompleted = completed) + extraCompleted,
        total: this.workEstimate?.totalPoses ?? total,
        budgeted: this.workEstimate?.totalPoses ?? null,
        generation: this.activeGeneration ?? this.generation,
      }),
      onPoseWork: () => { extraCompleted++; report(); },
    });
  }

  evaluateCycle(layout) { return evaluateCycle(layout, this.evaluationOptions()); }
  computeTorque(layout) { return this.evaluateCycle(layout).torqueNm; }
  computeSpeedDemand(layout) { return this.evaluateCycle(layout).speedRadPerSec; }
  computeFatigue(stats) { return computeFatigue(stats, this.evaluationOptions()); }
  dominates(a, b) { return dominates(a, b); }
  fastNonDominatedSort(evaluations) { return fastNonDominatedSort(evaluations); }
  assignCrowdingDistance(fronts, evaluations) { return assignCrowdingDistance(fronts, evaluations); }
  tournamentSelect(evaluations) { return tournamentSelect(evaluations, this.random); }

  // After a relaxation step the last few offspring are fresh layouts drawn in
  // the widened space (seedOrigin 'relaxation'); otherwise all are bred.
  createOffspring(evaluations) {
    const offspring = [];
    const immigrants = this.pendingImmigrants ?? 0;
    this.pendingImmigrants = 0;
    while (offspring.length < this.populationSize - immigrants) {
      const parentA = this.tournamentSelect(evaluations);
      const parentB = this.tournamentSelect(evaluations);
      let child = this.crossoverLayouts(parentA.layout, parentB.layout);
      if (this.random() < this.mutationRate) {
        child = this.mutateLayout(child);
      }
      child.id = this.nextLayoutId++;
      child.seedOrigin = 'offspring';
      delete child.referenceDiagnostics;
      delete child.migration;
      offspring.push(child);
    }
    while (offspring.length < this.populationSize) {
      const layout = this.createRandomLayout();
      layout.seedOrigin = 'relaxation';
      offspring.push(layout);
    }
    return offspring;
  }

  selectFromFronts(evaluations, fronts) {
    const survivors = selectFromFronts(evaluations, fronts, this.populationSize);
    if (this.referenceEvaluation && !survivors.includes(this.referenceEvaluation)) {
      survivors[survivors.length - 1] = this.referenceEvaluation;
    }
    if (this.configurations.length < 2) return survivors;
    const ranked = fronts.flatMap(front => front.map(index => evaluations[index])
      .sort((a, b) => (b.crowding ?? -Infinity) - (a.crowding ?? -Infinity)));
    return retainConfigurations(survivors, ranked, this.configurations, this.populationSize, this.referenceEvaluation);
  }

  updateState(evaluations, fronts) {
    this.population = evaluations.map((ev) => cloneLayout(ev.layout));
    this.fitness = evaluations;
    this.pareto = fronts[0]?.map((idx) => evaluations[idx]) || [];
    if (this.running || !this.fitness.some(ev => ev.layout.id === this.selectedCandidateId)) {
      this.selectedCandidateId = selectBest(this.pareto, this.fitness)?.layout.id ?? null;
    }
  }

  emitCheckpoint() {
    this.onCheckpoint?.({ generation: this.generation, completedEvaluations: this.completedEvaluations,
      fitness: this.fitness, pareto: this.pareto });
  }

  // Per-configuration counts for a run that searches several C3 configurations.
  configurationSummary() {
    return this.configurations.length > 1 ? configurationSummary(this.fitness, this.configurations) : null;
  }

  getSelectedCandidate() {
    return this.fitness.find(ev => ev.layout.id === this.selectedCandidateId)
      || selectBest(this.pareto, this.fitness);
  }

  selectCandidate(id) {
    const selected = this.fitness.find(ev => String(ev.layout.id) === String(id));
    if (!selected) throw new RangeError(`Candidate ${id} is not in the retained population.`);
    this.selectedCandidateId = selected.layout.id;
    return selected;
  }

  async evaluatePopulation(layouts) {
    const results = [];
    for (const layout of layouts) {
      this.abortController?.signal.throwIfAborted();
      const result = await this.evaluateLayout(layout);
      if (layout.seedOrigin === 'reference') {
        result.referenceDiagnostics = this.referenceDiagnostics;
        if (this.referenceDiagnostics.boundsConflicts.length) {
          result.feasibility.failedCategories = [...new Set([
            ...(result.feasibility.failedCategories ?? []), 'geometry',
          ])];
          result.feasibility.passing = false;
        }
        this.referenceEvaluation = result;
      }
      if (this.boundsRelaxation > 0) result.boundsExcursions = boundsExcursions(layout, this.designSpace);
      results.push(result);
      this.completedPoseWork += (result.workspace.workUnits ?? result.workspace.total) + 1 + result.cycle.samples;
      this.completedEvaluations += 1;
      this.onProgress?.({ completed: this.completedPoseWork,
        total: this.workEstimate.totalPoses, budgeted: this.workEstimate.totalPoses, generation: this.activeGeneration });
    }
    return results;
  }

  estimateWork() {
    return estimateWork({ ranges: this.ranges, sampling: this.sampling, trajectory: this.trajectory,
      cycleSampling: this.cycleSampling, payloadSupport: this.payloadSupport,
      populationSize: this.populationSize, generations: this.generations });
  }

  effectiveSettings() {
    return JSON.parse(JSON.stringify({
      requirements: this.requirements,
      bounds: this.ranges,
      sampling: this.sampling,
      cycleSampling: this.cycleSampling,
      seed: this.seed,
      randomAlgorithm: RANDOM_ALGORITHM,
      populationSize: this.populationSize,
      generations: this.generations,
      mutationRate: this.mutationRate,
      designSpace: this.designSpace,
      topology: this.topology,
      hornDirection: this.hornDirection,
      legPairing: this.legPairing,
      compactness: this.compactness,
      boundsRelaxation: this.boundsRelaxation,
      homeHeightBounds: this.designSpace.homeHeightBounds,
      ballJointLimitDeg: this.ballJointLimitDeg,
      lowerBallJointLimitDeg: this.lowerBallJointLimitDeg,
      upperBallJointLimitDeg: this.upperBallJointLimitDeg,
      conditionLimit: this.conditionLimit,
      linkClearanceMm: this.linkClearanceMm,
      ballJointClamp: this.ballJointClamp,
      servoRangeDeg: this.servoRangeDeg,
      servoRatings: this.servoRatingsInput,
      effectiveServoRatings: this.servoRatings,
      servoRatingPolicy: this.servoRatings.policy,
      // The load-sharing measure ranks Full runs, so its version travels with the export.
      cycleModel: { modelVersion: CYCLE_MODEL_VERSION, loadSharingModel: LOAD_SHARING_MODEL, trajectory: this.trajectory,
        trajectoryId: trajectoryIdentity(this.trajectory), trajectorySource: this.trajectorySource,
        massProperties: this.massProperties },
      objectiveSet: this.objectiveSet,
      objectiveDefinitions: objectiveDefinitions(this.objectiveSet, this.objectiveVariant),
      stiffnessModel: this.stiffnessModel,
      payloadSupport: this.payloadSupport,
      reference_layout: this.referenceLayout ? layoutToJSON(this.referenceLayout) : null,
      seed_composition: this.referenceLayout ? seedComposition(this.populationSize) : null,
    }));
  }

  async executeRun() {
    this.workEstimate = this.estimateWork();
    this.completedEvaluations = 0;
    this.completedPoseWork = 0;
    this.nextLayoutId = 1;
    this.freshLayouts = 0;
    this.relaxationShare = 0;
    this.relaxationHistory = [];
    this.pendingImmigrants = 0;
    this.random = createRandom(this.seed);
    this.population = initialPopulation(this);
    let evaluations = await this.evaluatePopulation(this.population);
    let fronts = this.fastNonDominatedSort(evaluations);
    this.assignCrowdingDistance(fronts, evaluations);
    this.updateState(evaluations, fronts);
    this.updateRelaxation(evaluations, 0);
    this.emitCheckpoint();

    for (let gen = 0; gen < this.generations; gen++) {
      this.abortController?.signal.throwIfAborted();
      this.activeGeneration = gen + 1;
      const offspringLayouts = this.createOffspring(evaluations);
      const offspringEvaluations = await this.evaluatePopulation(offspringLayouts);
      const combined = evaluations.concat(offspringEvaluations);
      fronts = this.fastNonDominatedSort(combined);
      this.assignCrowdingDistance(fronts, combined);
      evaluations = this.selectFromFronts(combined, fronts);
      fronts = this.fastNonDominatedSort(evaluations);
      this.assignCrowdingDistance(fronts, evaluations);
      this.updateState(evaluations, fronts);
      this.updateRelaxation(evaluations, gen + 1);
      this.generation = gen + 1;
      this.emitCheckpoint();
    }
  }

  async run() {
    if (this.running) throw new Error('An optimization is already running.');
    this.running = true;
    this.runStatus = 'running';
    this.abortController = new AbortController();
    this.population = [];
    this.fitness = [];
    this.pareto = [];
    this.selectedCandidateId = null;
    this.generation = 0;
    this.activeGeneration = 0;
    this.completedEvaluations = 0;
    this.nextLayoutId = 1;
    this.referenceEvaluation = null;
    try {
      await this.executeRun();
      this.runStatus = 'completed';
    } catch (error) {
      if (error.name !== 'AbortError') {
        this.runStatus = 'failed';
        throw error;
      }
      this.runStatus = 'cancelled';
    } finally {
      this.running = false;
      this.abortController = null;
    }
    return { status: this.runStatus, completedGenerations: this.generation,
      completedEvaluations: this.completedEvaluations,
      partialResults: this.runStatus === 'cancelled' && this.fitness.length > 0 };
  }

  start(callback) {
    return this.run().then(outcome => {
      callback?.(this, outcome);
      return outcome;
    });
  }

  stop() {
    this.abortController?.abort();
  }

  exportBest(format = 'json') {
    if (this.running) throw new Error('Wait for completion or cancellation before exporting.');
    if (!this.fitness.length) {
      console.warn('No evaluated layouts available for export.');
      return;
    }
    const best = this.getSelectedCandidate();
    if (!best) {
      console.warn('Unable to determine best layout.');
      return;
    }
    if (format !== 'json') {
      console.warn('Only JSON export is currently supported.');
      return;
    }
    return JSON.stringify(exportResult(best, {
      status: this.runStatus, completedGenerations: this.generation, partial: this.runStatus !== 'completed',
      effective_settings: this.effectiveSettings(),
      ...(this.configurations.length > 1 ? { configuration_summary: this.configurationSummary() } : {}),
      ...(this.boundsRelaxation > 0 ? { bounds_relaxation: this.relaxationSummary() } : {}),
    }), null, 2);
  }
}
