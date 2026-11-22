import type { Cloud, Point } from './types';

// Use the recorded path on axes where it spans the main cloud. A snapshot may
// carry only the final localized segment, so keep the cloud as a fallback.
function fitAxis(values: number[], pathMin: number, pathMax: number): [number, number] {
  if (!values.length) return [pathMin, pathMax];
  values.sort((a, b) => a - b);
  const q1 = values[Math.floor(values.length / 4)];
  const q3 = values[Math.ceil(3 * values.length / 4) - 1];
  if (pathMin < pathMax && pathMin <= q1 && pathMax >= q3) return [pathMin, pathMax];

  const fence = 1.5 * (q3 - q1);
  const core = values.filter(value => value >= q1 - fence && value <= q3 + fence);
  // A diffuse halo can still leave isolated points just inside the fences.
  const trim = Math.floor(core.length * .02);
  return [Math.min(core[trim], pathMin), Math.max(core[core.length - 1 - trim], pathMax)];
}

export function cloudFitCorners(cloud: Cloud): Point[] {
  if (cloud.frame !== 'map') return [];
  const xs: number[] = [], ys: number[] = [];
  const stride = Math.max(1, Math.ceil(cloud.points.length / 10_000));
  for (let i = 0; i < cloud.points.length; i += stride) {
    const [x, y] = cloud.points[i];
    if (Number.isFinite(x) && Number.isFinite(y)) {
      xs.push(x);
      ys.push(y);
    }
  }

  let xMin = Infinity, xMax = -Infinity, yMin = Infinity, yMax = -Infinity;
  if (Array.isArray(cloud.trajectory)) {
    for (const pose of cloud.trajectory) {
      if (!Array.isArray(pose) || !Number.isFinite(pose[0]) || !Number.isFinite(pose[1])) continue;
      xMin = Math.min(xMin, pose[0]); xMax = Math.max(xMax, pose[0]);
      yMin = Math.min(yMin, pose[1]); yMax = Math.max(yMax, pose[1]);
    }
  }
  if (!xs.length && xMin === Infinity) return [];
  const [left, right] = fitAxis(xs, xMin, xMax);
  const [bottom, top] = fitAxis(ys, yMin, yMax);
  return [[left, bottom], [right, top]];
}
