import { computeCycleDemand } from '../model/cycle.js';
import { evaluatePose } from '../model/pose.js';
import { resolveMounting } from '../model/mounting.js';
import { validateLinkClearance } from '../model/collision.js';
import { validateConditionLimit, NUMERICAL_RECIPROCAL_CUTOFF } from '../model/conditioning.js';
import { evaluateServoCapacity, normalizeServoRatings } from '../model/servo-ratings.js';
import { massPropertiesDescription, normalizeMassProperties } from '../model/mass-properties.js';
import { payloadSupportSatisfied } from '../workspace/payload-support.js';
import { actuatorUtilization } from '../model/load-sharing.js';
import { evaluateCompliance } from '../model/compliance.js';
import { computeWorkspace } from '../workspace/sweep.js';
import { failureCategories } from '../io/results.js';
import { DEFAULT_BALL_JOINT_LIMIT_DEG, DEFAULT_LINK_CLEARANCE_MM, MODEL_VERSION } from '../contracts.js';
import { normalizeTrajectory, trajectorySummary } from '../model/trajectory.js';
import { average, clamp, degToRad } from '../math.js';
import { objectiveValues } from './objectives.js';
import { layoutFootprint } from './compactness.js';

// Direct callers may omit the limits and the cycle identity that the Optimizer
// always supplies; resolve them once so every stage, the limit margin and the
// fatigue proxy share the same values.
export function resolveEvaluationOptions(options) {
  const ballJointLimitDeg = options.ballJointLimitDeg ?? DEFAULT_BALL_JOINT_LIMIT_DEG;
  const trajectorySource = options.trajectorySource ?? (options.trajectory ? 'supplied' : 'legacy-cycle');
  if (trajectorySource === 'supplied' && !options.trajectory) {
    throw new TypeError("trajectorySource 'supplied' requires a trajectory.");
  }
  // A direct caller may pass the JSON shape (no phase_deg, mixed-case axes); normalize it
  // so the cycle identity and sampling see the same trajectory the Optimizer would build.
  const trajectory = options.trajectory ? normalizeTrajectory(options.trajectory) : options.trajectory;
  const summary = trajectorySource === 'supplied' ? trajectorySummary(trajectory) : null;
  return { ...options,
    trajectory,
    ballJointLimitDeg,
    lowerBallJointLimitDeg: options.lowerBallJointLimitDeg ?? ballJointLimitDeg,
    upperBallJointLimitDeg: options.upperBallJointLimitDeg ?? ballJointLimitDeg,
    linkClearanceMm: options.linkClearanceMm ?? DEFAULT_LINK_CLEARANCE_MM,
    trajectorySource,
    stroke: options.stroke ?? summary?.stroke,
    frequency: options.frequency ?? summary?.frequency,
  };
}

export async function evaluateLayout(layout, rawOptions) {
  const options = resolveEvaluationOptions(rawOptions);
  const { ranges, signal, onProgress, payload, stroke, frequency, ballJointLimitDeg, ballJointClamp,
    lowerBallJointLimitDeg, upperBallJointLimitDeg, linkClearanceMm, sampling, random, onPoseWork } = options;
  const conditionLimit = validateConditionLimit(options.conditionLimit);
  validateLinkClearance(linkClearanceMm);
  const mounting = resolveMounting(layout).mounting;
  layout.mounting = mounting;
  const massProperties = options.massProperties ?? normalizeMassProperties({ mass_kg: payload ?? 0 });
  const payloadSupport = options.payloadSupport ? { settings: options.payloadSupport, massProperties,
    ratings: options.servoRatings ?? normalizeServoRatings(),
    loadCase: `${massPropertiesDescription(massProperties)}; ${massProperties.massKg} kg; `
      + `center of mass ${massProperties.centerOfMassM.map(v => v * 1000).join(', ')} mm (platform frame); `
      + `external force ${massProperties.externalForceN.join(', ')} N and moment ${massProperties.externalMomentNm.join(', ')} N m (base frame); `
      + 'gravity along -Z; zero velocity and acceleration' } : null;
  const workspaceResult = await computeWorkspace(layout, ranges, {
    signal, onProgress,
    payload, stroke, frequency, ballJointLimitDeg, lowerBallJointLimitDeg,
    upperBallJointLimitDeg, ballJointClamp, mounting, sampling, random, conditionLimit, linkClearanceMm,
    payloadSupport,
  });

  const coverage = Number.isFinite(workspaceResult.coverage) ? workspaceResult.coverage : 0;
  const relaxedCoverage = workspaceResult.relaxedCoverage ?? coverage;
  const stats = workspaceResult.stats || {};

  const homeResult = evaluatePose(layout, {
    x: 0,
    y: 0,
    z: 0,
    rx: 0,
    ry: 0,
    rz: 0,
  }, {
    ballJointLimitDeg, lowerBallJointLimitDeg, upperBallJointLimitDeg, ballJointClamp, mounting,
    conditionLimit, linkClearanceMm,
    servoRangeRad: layout.servoRangeRad,
    recordLegData: true,
  });
  onPoseWork?.();

  const dexterity = homeResult.reachable ? homeResult.conditioning.reciprocal : null;
  const stiffness = homeResult.reachable ? homeResult.conditioning.sigmaMin : null;
  const condition = homeResult.conditioning.condition;
  const conditioningQuality = [dexterity, stats.worstReciprocal]
    .filter(Number.isFinite).reduce((worst, value) => Math.min(worst, value), Infinity);
  const availableQuality = Number.isFinite(conditioningQuality) ? conditioningQuality : null;

  const cycle = evaluateCycle(layout, { ...options, mounting, onPose: onPoseWork });
  const servoCapacity = evaluateServoCapacity(cycle, options.servoRatings);
  const torque = cycle.torqueNm;
  const speedDemand = cycle.speedRadPerSec;
  const loadBalance = stats.loadBalanceScore ?? 0;
  const loadSharing = cycle.loadSharing?.balanceScore ?? null;
  const compliance = evaluateCompliance(layout, homeResult, options.stiffnessModel ?? null);
  const physicalStiffness = compliance.minScaledStiffnessNPerM;
  const support = workspaceResult.payloadSupport;
  const payloadCoverage = support?.qualifiedCoverage ?? null;
  const utilization = actuatorUtilization(cycle, options.servoRatings);
  const isotropy = stats.averageIsotropy ?? 0;
  const stiffnessScore = stats.averageStiffness > 0 ? stats.averageStiffness : stiffness;
  const marginFor = (maxAngle, limitDeg) => {
    const limit = degToRad(limitDeg);
    if (!Number.isFinite(maxAngle)) return 0;
    return limit > 0 ? 1 - maxAngle / limit : (maxAngle <= 1e-6 ? 1 : 0);
  };
  // Headroom of the worst reachable socket deflection against its limit. Only
  // reachable poses count: a violating pose already lowers coverage, and
  // including it collapsed the margin to 0 whenever any sampled pose failed.
  const limitMargin = !(stats.reachableCount > 0) ? 0 : clamp(Math.min(
    marginFor(Math.max(0, ...(stats.reachableLowerJointMax ?? [])), lowerBallJointLimitDeg),
    marginFor(Math.max(0, ...(stats.reachableUpperJointMax ?? [])), upperBallJointLimitDeg),
  ), 0, 1);
  const fatigue = computeFatigue(cycle);
  const footprint = layoutFootprint(layout);
  const objectives = objectiveValues({ coverage, relaxedCoverage,
    conditioningQuality: availableQuality, dexterity, stiffness: stiffnessScore,
    physicalStiffness, loadBalance, loadSharing, isotropy, limitMargin, torque, speedDemand, fatigue, footprint },
  options.objectiveSet, options.objectiveVariant);

  const feasibility = {
    cycleSatisfied: cycle.valid,
    sampledWorkspaceSatisfied: coverage === 100,
    homePoseSatisfied: homeResult.reachable,
    conditionSatisfied: !homeResult.violations.some(v => ['numericalSingularity', 'conditionLimit'].includes(v.type))
      && !(stats.conditioningCounts?.numericalSingularity || stats.conditioningCounts?.engineeringLimit
        || stats.conditioningCounts?.unavailable)
      && !cycle.violations?.some(v => ['numericalSingularity', 'conditionLimit'].includes(v.type)),
    // Budget exhaustion before convergence is inconclusive, never a convergence pass.
    // Links of different legs kept their clearance at home, at every sampled
    // workspace pose and at every evaluated cycle sample.
    collisionSatisfied: !homeResult.violations.some(v => v.type === 'linkCollision')
      && !stats.violationCounts?.linkCollision
      && !cycle.violations?.some(v => v.type === 'linkCollision'),
    cycleConvergence: cycle.sampling?.status ?? null,
    cycleConvergenceSatisfied: cycle.sampling?.status !== 'budget-limited'
      || cycle.sampling?.inconclusivePolicy === 'advisory',
    servoCapacitySatisfied: !servoCapacity.hasRatings || servoCapacity.compliant === true,
    servoCapacityEnforced: servoCapacity.hasRatings && servoCapacity.policy === 'enforced',
    payloadSupport: support?.status ?? null,
    payloadSupportSatisfied: payloadSupportSatisfied(support),
    payloadSupportEnforced: support?.policy === 'enforced',
    scope: 'Sampled poses under the modeled geometry, servo, rod, ball-joint, link-clearance and conditioning constraints',
  };
  feasibility.failedCategories = failureCategories({ feasibility, cycle });
  feasibility.passing = feasibility.failedCategories.length === 0;

  return {
    layout,
    workspace: workspaceResult,
    coverage,
    relaxedCoverage,
    payloadCoverage,
    cycle,
    feasibility,
    dexterity,
    stiffness: stiffnessScore,
    physicalStiffness,
    compliance,
    conditioningQuality: availableQuality,
    conditioning: {
      modelVersion: MODEL_VERSION,
      home: homeResult.conditioning,
      workspace: { worstReciprocal: stats.worstReciprocal ?? null,
        worstCondition: stats.worstCondition ?? null, counts: stats.conditioningCounts ?? null },
      cycle: cycle.conditioning ?? null,
      limit: conditionLimit,
      numericalThreshold: NUMERICAL_RECIPROCAL_CUTOFF,
    },
    torque,
    speedDemand,
    servoCapacity,
    loadBalance,
    loadSharing,
    actuatorUtilization: utilization,
    isotropy,
    limitMargin,
    fatigue,
    footprint,
    condition,
    objectives,
    homePose: homeResult,
    rank: Infinity,
    crowding: 0,
  };
}

export function evaluateCycle(layout, rawOptions) {
  const { payload, stroke, frequency, cycleAxis, trajectory, trajectorySource,
    massProperties, cycleSampling, servoRatings, ballJointLimitDeg,
    lowerBallJointLimitDeg, upperBallJointLimitDeg, conditionLimit, linkClearanceMm, mounting, signal,
    onPose } = resolveEvaluationOptions(rawOptions);
  // A legacy-cycle trajectory keeps the single-axis identity in exported results.
  const supplied = trajectorySource === 'supplied' ? trajectory : undefined;
  return computeCycleDemand(layout, { mass: payload, stroke,
    frequency, axis: cycleAxis, trajectory: supplied, trajectorySource, massProperties, sampling: cycleSampling,
    actuators: servoRatings?.perServo?.map(servo => servo.actuator) ?? null,
    ballJointLimitDeg, lowerBallJointLimitDeg,
    upperBallJointLimitDeg, conditionLimit, linkClearanceMm, mounting, signal, onPose });
}

// Servo motion rate (rad/s): cycle frequency times the mean over the six servos
// of the RMS servo excursion about its cycle mean. It comes from the cycle
// alone, so it is unchanged by the servo travel bounds or the ball-joint limit
// and is nonzero for rotation-only trajectories. A relative heuristic, not
// service life; null for an invalid cycle.
export function computeFatigue(cycle) {
  if (!cycle?.valid || !Array.isArray(cycle.servoExcursionRmsRad)) return null;
  if (!(cycle.periodS > 0)) return 0;
  return average(cycle.servoExcursionRmsRad) / cycle.periodS;
}
