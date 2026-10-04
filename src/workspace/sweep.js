import { ensureLayout, evaluatePose } from '../model/pose.js';
import { resolveMounting } from '../model/mounting.js';
import { NUMERICAL_RECIPROCAL_CUTOFF, validateConditionLimit } from '../model/conditioning.js';
import { createWorkspaceStatistics } from './statistics.js';
import { createRandom } from '../optimization/random.js';
import { estimateWorkspaceSize, normalizeSampling, workspacePoses } from './sampling.js';
import { createPayloadSupportStatistics } from './payload-support.js';
import { dynamicsAtPose, staticState } from '../model/cycle.js';
import { DEFAULT_BALL_JOINT_LIMIT_DEG, DEFAULT_LINK_CLEARANCE_MM } from '../contracts.js';
import { validateLinkClearance } from '../model/collision.js';

export { MAX_WORKSPACE_POSES, estimateWorkspaceSize } from './sampling.js';

export const yieldToEventLoop = () => new Promise(resolve => setTimeout(resolve, 0));

export async function computeWorkspace(layout, ranges = {}, options = {}) {
  ensureLayout(layout);
  const {
    ballJointLimitDeg = DEFAULT_BALL_JOINT_LIMIT_DEG,
    lowerBallJointLimitDeg = ballJointLimitDeg,
    upperBallJointLimitDeg = ballJointLimitDeg,
    conditionLimit = null,
    linkClearanceMm = DEFAULT_LINK_CLEARANCE_MM,
    ballJointClamp = false,
    payload = 0,
    stroke = 0,
    frequency = 0,
    sampleLimit = 200,
    violationSampleLimit = sampleLimit,
    sampling = { strategy: 'grid' },
    random,
    onProgress,
    signal,
    payloadSupport = null,
    stiffnessDirection = null,
  } = options;

  signal?.throwIfAborted();
  validateConditionLimit(conditionLimit);
  validateLinkClearance(linkClearanceMm);
  const mounting = options.mounting ?? resolveMounting(layout).mounting;
  const effectiveSampling = normalizeSampling(sampling);
  const totalPoses = estimateWorkspaceSize(ranges, effectiveSampling);

  // normalizeSampling admits any safe-integer sequence start; fold it into the 32-bit seed range.
  const reservoirSeed = effectiveSampling.strategy === 'halton'
    ? ((effectiveSampling.sequenceStart - 1) % 0xffffffff) + 1 : null;
  const statistics = createWorkspaceStatistics({ totalPoses, sampleLimit, violationSampleLimit, stiffnessDirection,
    random: random ?? (reservoirSeed ? createRandom(reservoirSeed) : Math.random) });

  // Optional static holding check at every strictly feasible pose; each check is one extra work unit.
  const support = payloadSupport ? createPayloadSupportStatistics(payloadSupport) : null;
  let completed = 0, checks = 0;
  const totalWork = support ? 2 * totalPoses : totalPoses;
  onProgress?.({ completed, total: totalWork });
  await yieldToEventLoop();
  signal?.throwIfAborted();
  for (const pose of workspacePoses(ranges, effectiveSampling)) {
    const result = evaluatePose(layout, pose, {
      ballJointLimitDeg, lowerBallJointLimitDeg, upperBallJointLimitDeg, ballJointClamp,
      conditionLimit, linkClearanceMm, mounting, servoRangeRad: layout.servoRangeRad, recordLegData: Boolean(support),
    });
    statistics.add(pose, result);
    if (support && result.reachable) {
      support.add(pose, dynamicsAtPose(layout, result, staticState(pose), payloadSupport.massProperties));
      checks++;
    }
    completed++;
    if (completed % 256 === 0) {
      onProgress?.({ completed: completed + checks, total: totalWork });
      await yieldToEventLoop();
      signal?.throwIfAborted();
    }
  }
  onProgress?.({ completed: completed + checks, total: totalWork });

  return {
    ...statistics.finish(),
    constraintPolicy: { mode: ballJointClamp ? 'soft-ball-joint' : 'strict', ballJointLimitDeg,
      lowerBallJointLimitDeg, upperBallJointLimitDeg, conditionLimit, linkClearanceMm,
      numericalReciprocalCutoff: NUMERICAL_RECIPROCAL_CUTOFF },
    sampling: effectiveSampling,
    payload, stroke, frequency,
    workUnits: completed + checks,
    payloadSupport: support ? support.finish(totalPoses) : null,
  };
}
