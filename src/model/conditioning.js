import { vectorCross, vectorDot, vectorMagnitude, vectorSub } from '../math.js';

export const NUMERICAL_RECIPROCAL_CUTOFF = 1e-10;
const LEVERAGE_CUTOFF = 1e-10;

export function validateConditionLimit(conditionLimit) {
  if (conditionLimit == null) return null;
  if (!Number.isFinite(conditionLimit) || conditionLimit < 1) {
    throw new RangeError('conditionLimit must be a finite number >= 1.');
  }
  return conditionLimit;
}

// One-sided Jacobi SVD acts on the matrix itself. Forming J^T J loses the
// small singular values needed at the 1e-10 reciprocal-condition boundary.
export function singularValuesOneSided(matrix) {
  const n = matrix.length;
  if (!n || matrix.some(row => !Array.isArray(row) || row.length !== n
      || row.some(value => !Number.isFinite(value)))) return null;
  const scale = Math.max(...matrix.flat().map(Math.abs));
  if (scale === 0 || !Number.isFinite(scale)) return new Array(n).fill(0);
  const columns = Array.from({ length: n }, (_, col) => matrix.map(row => row[col] / scale));
  const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
  let converged = false;
  for (let sweep = 0; sweep < 80; sweep++) {
    let rotated = false;
    for (let p = 0; p < n - 1; p++) {
      for (let q = p + 1; q < n; q++) {
        const a = dot(columns[p], columns[p]);
        const b = dot(columns[q], columns[q]);
        const c = dot(columns[p], columns[q]);
        if (a === 0 || b === 0 || Math.abs(c) <= 4 * Number.EPSILON * Math.sqrt(a * b)) continue;
        rotated = true;
        const tau = (b - a) / (2 * c);
        const t = (tau >= 0 ? 1 : -1) / (Math.abs(tau) + Math.hypot(1, tau));
        const cosine = 1 / Math.hypot(1, t);
        const sine = t * cosine;
        const left = columns[p], right = columns[q];
        for (let row = 0; row < n; row++) {
          const lp = left[row], rq = right[row];
          left[row] = cosine * lp - sine * rq;
          right[row] = sine * lp + cosine * rq;
        }
      }
    }
    if (!rotated) { converged = true; break; }
  }
  if (!converged) return null;
  return columns.map(col => Math.hypot(...col) * scale).sort((a, b) => b - a);
}

// Singular values and right singular vectors of the translation block (the
// first three columns) of a 6x6 actuator Jacobian, for the simulator's
// conditioning ellipsoid. The same one-sided Jacobi rotations as above, applied
// to the 6x3 block while accumulating them into V, so each vector is a world
// translation direction and its value is how strongly the servos couple to
// motion along it. Values are sorted largest first, with their vectors; the
// pose condition number still comes from singularValuesOneSided on all six columns.
export function translationSingularSystem(rows) {
  if (!Array.isArray(rows) || rows.length !== 6 || rows.some(row => !Array.isArray(row) || row.length !== 6
      || row.some(value => !Number.isFinite(value)))) return null;
  const columns = [0, 1, 2].map(col => rows.map(row => row[col]));
  const scale = Math.max(...columns.flat().map(Math.abs));
  if (scale === 0 || !Number.isFinite(scale)) return null;
  for (const column of columns) column.forEach((value, row) => { column[row] = value / scale; });
  const vectors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
  const rotate = (left, right, cosine, sine) => {
    for (let k = 0; k < left.length; k++) {
      const lp = left[k], rq = right[k];
      left[k] = cosine * lp - sine * rq;
      right[k] = sine * lp + cosine * rq;
    }
  };
  let converged = false;
  for (let sweep = 0; sweep < 80; sweep++) {
    let rotated = false;
    for (let p = 0; p < 2; p++) {
      for (let q = p + 1; q < 3; q++) {
        const a = dot(columns[p], columns[p]);
        const b = dot(columns[q], columns[q]);
        const c = dot(columns[p], columns[q]);
        if (a === 0 || b === 0 || Math.abs(c) <= 4 * Number.EPSILON * Math.sqrt(a * b)) continue;
        rotated = true;
        const tau = (b - a) / (2 * c);
        const t = (tau >= 0 ? 1 : -1) / (Math.abs(tau) + Math.hypot(1, tau));
        const cosine = 1 / Math.hypot(1, t);
        const sine = t * cosine;
        rotate(columns[p], columns[q], cosine, sine);
        rotate(vectors[p], vectors[q], cosine, sine);
      }
    }
    if (!rotated) { converged = true; break; }
  }
  if (!converged) return null;
  return columns.map((column, k) => ({ value: Math.hypot(...column) * scale, vector: vectors[k] }))
    .sort((a, b) => b.value - a.value);
}

export function assessJacobian(rows, conditionLimit = null) {
  validateConditionLimit(conditionLimit);
  const singularValues = singularValuesOneSided(rows);
  if (!singularValues) return { available: false, satisfied: false,
    numericalSingularity: true, engineeringFailure: false, reason: 'nonfiniteOrNonconvergent',
    singularValues: null, condition: null, reciprocal: null };
  const sigmaMax = singularValues[0];
  const sigmaMin = singularValues.at(-1);
  const reciprocal = sigmaMax > 0 ? sigmaMin / sigmaMax : 0;
  const condition = reciprocal > 0 ? 1 / reciprocal : null;
  const numericalSingularity = !Number.isFinite(reciprocal)
    || reciprocal <= NUMERICAL_RECIPROCAL_CUTOFF;
  const engineeringFailure = !numericalSingularity && conditionLimit != null
    && condition > conditionLimit;
  return { available: true, satisfied: !numericalSingularity && !engineeringFailure,
    numericalSingularity, engineeringFailure, singularValues,
    sigmaMax, sigmaMin, condition, reciprocal };
}

export function rotaryActuatorJacobian(platformPoints, rodVectors, servoAngles,
  betaAngles, hornLength) {
  if ([platformPoints, rodVectors, servoAngles, betaAngles].some(values => values.length !== 6)) {
    return { rows: null, reason: 'incompletePose' };
  }
  const centroid = [0, 1, 2].map(axis => platformPoints.reduce((sum, q) => sum + q[axis], 0) / 6);
  const offsets = platformPoints.map(q => vectorSub(q, centroid));
  const radius = Math.sqrt(offsets.reduce((sum, r) => sum + vectorDot(r, r), 0) / 6);
  if (!Number.isFinite(radius) || radius <= 0) return { rows: null, centroid, radius, reason: 'anchorRadius' };
  const rows = [];
  for (let leg = 0; leg < 6; leg++) {
    const rod = rodVectors[leg];
    const rodLength = vectorMagnitude(rod);
    if (!Number.isFinite(rodLength) || rodLength <= 0) {
      return { rows: null, centroid, radius, leg, reason: 'rodDirection' };
    }
    const u = rod.map(value => value / rodLength);
    const alpha = servoAngles[leg], beta = betaAngles[leg];
    const tangent = [
      -hornLength * Math.sin(alpha) * Math.cos(beta),
      -hornLength * Math.sin(alpha) * Math.sin(beta),
      hornLength * Math.cos(alpha),
    ];
    const leverage = vectorDot(u, tangent);
    if (!Number.isFinite(leverage) || Math.abs(leverage) <= LEVERAGE_CUTOFF * hornLength) {
      return { rows: null, centroid, radius, leg, leverage,
        reason: 'servoLeverage' };
    }
    rows.push([...u.map(value => radius * value), ...vectorCross(offsets[leg], u)]
      .map(value => value / leverage));
  }
  return { rows, centroid, radius };
}

export function assessPoseConditioning(platformPoints, rodVectors, servoAngles,
  betaAngles, hornLength, conditionLimit = null) {
  const geometry = rotaryActuatorJacobian(platformPoints, rodVectors, servoAngles,
    betaAngles, hornLength);
  if (!geometry.rows) return { available: false, satisfied: false,
    numericalSingularity: true, engineeringFailure: false, reason: geometry.reason,
    leg: geometry.leg ?? null, leverage: geometry.leverage ?? null,
    condition: null, reciprocal: null, singularValues: null, jacobianRows: null,
    centroid: geometry.centroid ?? null, radius: geometry.radius ?? null };
  const assessment = assessJacobian(geometry.rows, conditionLimit);
  return { ...assessment, reason: assessment.numericalSingularity ? 'singularValues'
    : assessment.engineeringFailure ? 'engineeringLimit' : null,
    jacobianRows: geometry.rows, centroid: geometry.centroid, radius: geometry.radius };
}

// Platform twist axes in Jacobian column order: translations (scaled by the
// characteristic radius) then rotations.
export const STIFFNESS_DIRECTIONS = Object.freeze(['x', 'y', 'z', 'rx', 'ry', 'rz']);

export function validateStiffnessDirection(direction) {
  if (direction == null) return null;
  if (!STIFFNESS_DIRECTIONS.includes(direction)) {
    throw new RangeError(`stiffnessDirection must be one of ${STIFFNESS_DIRECTIONS.join(', ')}.`);
  }
  return direction;
}

// Stiffness proxy along one twist axis, per unit servo stiffness. With every
// servo equally stiff, K is proportional to J^T J, so the compliance is
// C = J^-1 J^-T and a load along axis i deflects that axis by C[i][i] while the
// other axes are free to comply. The proxy is 1 / sqrt(C[i][i]), the norm
// reciprocal of row i of J^-1, which puts it on the same scale as sigmaMin:
// sigmaMin <= value <= sigmaMax. Null for a missing or singular Jacobian.
export function directionalStiffness(rows, direction) {
  const axis = STIFFNESS_DIRECTIONS.indexOf(direction);
  if (axis < 0 || !Array.isArray(rows) || rows.length !== 6
      || rows.some(row => !Array.isArray(row) || row.length !== 6 || row.some(value => !Number.isFinite(value)))) return null;
  // Gauss-Jordan with partial pivoting on [J | I].
  const work = rows.map((row, i) => [...row, ...Array.from({ length: 6 }, (_, j) => (i === j ? 1 : 0))]);
  const scale = Math.max(...rows.flat().map(Math.abs));
  if (!(scale > 0)) return null;
  for (let col = 0; col < 6; col++) {
    let pivot = col;
    for (let row = col + 1; row < 6; row++) {
      if (Math.abs(work[row][col]) > Math.abs(work[pivot][col])) pivot = row;
    }
    if (Math.abs(work[pivot][col]) <= NUMERICAL_RECIPROCAL_CUTOFF * scale) return null;
    [work[col], work[pivot]] = [work[pivot], work[col]];
    const lead = work[col][col];
    for (let k = 0; k < 12; k++) work[col][k] /= lead;
    for (let row = 0; row < 6; row++) {
      if (row === col || work[row][col] === 0) continue;
      const factor = work[row][col];
      for (let k = 0; k < 12; k++) work[row][k] -= factor * work[col][k];
    }
  }
  const compliance = work[axis].slice(6).reduce((sum, value) => sum + value * value, 0);
  return compliance > 0 && Number.isFinite(compliance) ? 1 / Math.sqrt(compliance) : null;
}
