import { average, standardDeviation } from '../math.js';
import { directionalStiffness } from '../model/conditioning.js';

function runningMean() {
  let count = 0;
  let total = 0;
  return { add(value) { total += value; count++; }, value() { return count ? total / count : 0; } };
}

// Owns aggregation and reservoir sampling; it never schedules or evaluates poses.
// `stiffnessDirection` (x, y, z, rx, ry or rz) also records the worst
// directional stiffness proxy; it draws no random numbers.
export function createWorkspaceStatistics({ totalPoses, sampleLimit = 200, violationSampleLimit = sampleLimit, random = Math.random,
  stiffnessDirection = null }) {
  const normalizedSampleLimit = Math.min(200, Math.max(0, Math.floor(sampleLimit)));
  const normalizedViolationSampleLimit = Math.min(200, Math.max(0, Math.floor(violationSampleLimit)));
  const reachableSamples = [];
  const unreachableSamples = [];
  const violationSamples = [];
  const violationCounts = {};
  const isotropySamples = runningMean();
  const stiffnessSamples = runningMean();
  const loadShareSamples = runningMean();
  const ballJointSamples = runningMean();
  const servoRanges = Array.from({ length: 6 }, () => ({ min: Infinity, max: -Infinity }));
  const ballJointMax = new Array(6).fill(0);
  const lowerJointMax = new Array(6).fill(0);
  const upperJointMax = new Array(6).fill(0);
  // Socket maxima over reachable poses only; the all-pose maxima above include
  // violating poses and exceed the limit whenever any sampled pose fails.
  const reachableLowerJointMax = new Array(6).fill(0);
  const reachableUpperJointMax = new Array(6).fill(0);
  const jointViolationCounts = { lower: 0, upper: 0 };
  const conditioningCounts = { valid: 0, numericalSingularity: 0,
    engineeringLimit: 0, unavailable: 0 };
  let worstReciprocal = null;
  let worstCondition = null;
  let worstDirectionalStiffness = null;

  const recordSample = (collection, limit, seenCount, sample) => {
    if (limit <= 0) return;
    if (collection.length < limit) {
      collection.push(sample);
    } else {
      const replaceIndex = Math.floor(random() * seenCount);
      if (replaceIndex < limit) {
        collection[replaceIndex] = sample;
      }
    }
  };

  let relaxedReachableCount = 0;
  let reachableCount = 0;
  let unreachableCount = 0;
  let violationPoseCount = 0;
  let reachableSeen = 0;
  let unreachableSeen = 0;
  let violationSeen = 0;


  function add(pose, result) {
    const hasViolations = Array.isArray(result.violations) && result.violations.length > 0;

    if (Array.isArray(result.ballJointAngles) && result.ballJointAngles.length) {
      ballJointSamples.add(Math.max(...result.ballJointAngles));
      for (let leg = 0; leg < result.ballJointAngles.length; leg++) {
        ballJointMax[leg] = Math.max(ballJointMax[leg], result.ballJointAngles[leg]);
        lowerJointMax[leg] = Math.max(lowerJointMax[leg], result.jointAngles?.lower[leg] ?? 0);
        upperJointMax[leg] = Math.max(upperJointMax[leg], result.jointAngles?.upper[leg] ?? 0);
      }
    }

    if (result.geometricallyReachable) {
      if (!result.conditioning?.available) conditioningCounts.unavailable++;
      else if (result.conditioning.numericalSingularity) conditioningCounts.numericalSingularity++;
      else if (result.conditioning.engineeringFailure) conditioningCounts.engineeringLimit++;
      else conditioningCounts.valid++;
    }

    if (result.relaxedReachable) relaxedReachableCount += 1;
    if (result.reachable) {
      reachableCount += 1;
      reachableSeen += 1;
      recordSample(reachableSamples, normalizedSampleLimit, reachableSeen, { pose });
      if (result.jointAngles) {
        for (let leg = 0; leg < 6; leg++) {
          reachableLowerJointMax[leg] = Math.max(reachableLowerJointMax[leg], result.jointAngles.lower?.[leg] ?? 0);
          reachableUpperJointMax[leg] = Math.max(reachableUpperJointMax[leg], result.jointAngles.upper?.[leg] ?? 0);
        }
      }

      if (result.conditioning?.satisfied) {
        const { reciprocal, condition, sigmaMin } = result.conditioning;
        isotropySamples.add(reciprocal);
        stiffnessSamples.add(sigmaMin);
        worstReciprocal = worstReciprocal == null ? reciprocal : Math.min(worstReciprocal, reciprocal);
        worstCondition = worstCondition == null ? condition : Math.max(worstCondition, condition);
        if (stiffnessDirection) {
          const value = directionalStiffness(result.conditioning.jacobianRows, stiffnessDirection);
          if (value != null) worstDirectionalStiffness = worstDirectionalStiffness == null
            ? value : Math.min(worstDirectionalStiffness, value);
        }
      }

      if (result.legDirections.length === 6) {
        const shares = result.legDirections.map((dir) => Math.abs(dir[2]));
        const sumShares = shares.reduce((acc, value) => acc + value, 0) || 1;
        const normalized = shares.map((value) => value / sumShares);
        const loadStd = standardDeviation(normalized);
        const loadScore = 1 / (1 + loadStd);
        loadShareSamples.add(loadScore);
      }

      if (Array.isArray(result.servoAngles)) {
        for (let iLeg = 0; iLeg < Math.min(6, result.servoAngles.length); iLeg++) {
          const angle = result.servoAngles[iLeg];
          const servo = servoRanges[iLeg];
          if (angle < servo.min) servo.min = angle;
          if (angle > servo.max) servo.max = angle;
        }
      }

    } else {
      unreachableCount += 1;
      unreachableSeen += 1;
      recordSample(unreachableSamples, normalizedSampleLimit, unreachableSeen, { pose });
    }

    if (hasViolations) {
      violationPoseCount += 1;
      violationSeen += 1;
      recordSample(
        violationSamples,
        normalizedViolationSampleLimit,
        violationSeen,
        { pose, violations: result.violations.map((violation) => ({ ...violation })) },
      );
      for (const violation of result.violations) {
        violationCounts[violation.type] = (violationCounts[violation.type] || 0) + 1;
        if (violation.type === 'ballJoint' && violation.joint in jointViolationCounts) {
          jointViolationCounts[violation.joint]++;
        }
      }
    }
  }

  function finish() {
    const coverage = (reachableCount / totalPoses) * 100;
    const relaxedCoverage = (relaxedReachableCount / totalPoses) * 100;

    const servoUsage = servoRanges.map((range) => {
      if (range.min === Infinity || range.max === -Infinity) return 0;
      return range.max - range.min;
    });
    const servoUsageAvg = average(servoUsage);
    const servoUsagePeak = Math.max(0, ...servoUsage);

    const violationRate = totalPoses > 0 ? violationPoseCount / totalPoses : 0;

    const stats = {
      reachableCount,
      averageIsotropy: isotropySamples.value(),
      averageStiffness: stiffnessSamples.value(),
      loadBalanceScore: loadShareSamples.value(),
      servoUsage,
      servoUsageAvg,
      servoUsagePeak,
      ballJointMax,
      lowerJointMax,
      upperJointMax,
      reachableLowerJointMax,
      reachableUpperJointMax,
      ballJointOverallMax: Math.max(0, ...ballJointMax),
      ballJointAverage: ballJointSamples.value(),
      jointViolationCounts,
      conditioningCounts,
      worstReciprocal,
      worstCondition,
      ...(stiffnessDirection ? { stiffnessDirection, worstDirectionalStiffness } : {}),
      violationCounts,
      violationRate,
    };

    return {
      coverage,
      relaxedCoverage,
      total: totalPoses,
      reachable: reachableSamples,
      unreachable: unreachableSamples,
      violations: violationSamples,
      stats,
      counts: {
        relaxedReachable: relaxedReachableCount,
        reachable: reachableCount,
        unreachable: unreachableCount,
        violationPoses: violationPoseCount,
      },
      samples: {
        reachable: reachableSamples,
        unreachable: unreachableSamples,
        violations: violationSamples,
        limits: {
          reachable: normalizedSampleLimit,
          unreachable: normalizedSampleLimit,
          violations: normalizedViolationSampleLimit,
        },
      },
    };
  }

  return { add, finish };
}
