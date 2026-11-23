import type { Line, Point } from './types';

export type SpeedScale = { min: number; max: number };

// Equally spaced stops from the slowest point on the saved line to the fastest.
const palette = [[38, 94, 212], [15, 153, 188], [27, 146, 71], [207, 145, 19], [208, 48, 44]];
const rgb = (channels: number[]) => `rgb(${channels.join(', ')})`;
export const speedRamp = `linear-gradient(to right, ${palette.map(rgb).join(', ')})`;

export function savedSpeeds(line?: Line): number[] | undefined {
  if (!line || line.speeds !== undefined) return line?.speeds;
  // HDMaps can store just the planner's [s, x, y, yaw, curvature, v, a] rows.
  const profile = line.profile;
  if (!Array.isArray(profile) || profile.length !== line.points.length) return;
  const speeds: number[] = [];
  for (let i = 0; i < profile.length; ++i) {
    const row = profile[i], [x, y] = line.points[i];
    if (!Array.isArray(row) || row.length !== 7 || !row.every(Number.isFinite)
      || Math.abs(row[1] - x) > 1e-6 || Math.abs(row[2] - y) > 1e-6) return;
    speeds.push(row[5]);
  }
  return speeds;
}

export function speedScale(speeds: number[] | undefined, pointCount: number): SpeedScale | null {
  if (!Array.isArray(speeds) || pointCount < 2 || speeds.length !== pointCount) return null;
  let min = Infinity, max = -Infinity;
  for (const speed of speeds) {
    if (!Number.isFinite(speed) || speed < 0) return null;
    min = Math.min(min, speed);
    max = Math.max(max, speed);
  }
  return { min, max };
}

export function speedColor(value: number, { min, max }: SpeedScale): string {
  const fraction = min === max ? .5 : Math.max(0, Math.min(1, (value - min) / (max - min)));
  const index = fraction * (palette.length - 1);
  const left = Math.min(Math.floor(index), palette.length - 2);
  return rgb(palette[left].map((channel, i) => Math.round(channel + (palette[left + 1][i] - channel) * (index - left))));
}

type ColorStop = { offset: number; color: string };
type SpeedStroke = { points: Point[]; color: string; stops?: ColorStop[] };

// Uniform stretches share a stroke. A segment that crosses e.g. blue to red
// includes the intervening palette stops even when the saved points are far apart.
export function speedStrokes(points: Point[], speeds: number[], scale: SpeedScale, closed: boolean): SpeedStroke[] {
  const strokes: SpeedStroke[] = [];
  const colors = speeds.map(speed => speedColor(speed, scale));
  const count = closed && points.length > 2 ? points.length : points.length - 1;
  for (let i = 0; i < count; ++i) {
    const next = (i + 1) % points.length;
    const from = points[i], to = points[next];
    if (from[0] === to[0] && from[1] === to[1]) continue;
    if (colors[i] === colors[next]) {
      const last = strokes.at(-1);
      if (last && !last.stops && last.color === colors[i]) last.points.push(to);
      else strokes.push({ points: [from, to], color: colors[i] });
      continue;
    }
    const stops = [{ offset: 0, color: colors[i] }, { offset: 1, color: colors[next] }];
    for (let j = 1; j < palette.length - 1; ++j) {
      const threshold = scale.min + (scale.max - scale.min) * j / (palette.length - 1);
      const offset = (threshold - speeds[i]) / (speeds[next] - speeds[i]);
      if (offset > 0 && offset < 1) stops.push({ offset, color: rgb(palette[j]) });
    }
    strokes.push({ points: [from, to], color: colors[i], stops: stops.sort((a, b) => a.offset - b.offset) });
  }
  return strokes;
}
