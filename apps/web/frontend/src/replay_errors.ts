export interface ReplayRun {
  id: string;
  bag: string;
  circuit: string | null;
  created_at: string;
  status: string;
  reason: string | null;
  sample_count: number;
  slam_max_map_size?: number;
  laps: number | null;
}

interface CoursePosition {
  lap: number | null;
  station_m: number | null;
  distance_m: number;
  time_s: number;
}

export interface Reference extends CoursePosition {
  x_m: number;
  y_m: number;
}

export interface ErrorSample extends CoursePosition {
  gt_x_m: number;
  gt_y_m: number;
  est_x_m: number;
  est_y_m: number;
  along_m: number;
  lateral_m: number;
  heading_deg: number;
  segment: number;
}

export interface Section { id: string; label: string; start_m: number; end_m: number }
export interface ReplayDetail extends ReplayRun {
  lap_length_m: number | null;
  sections: Section[];
  reference: Reference[];
  samples: ErrorSample[];
}

export interface Located<T> { at: number; point: T }
export type Metric = 'along_m' | 'lateral_m' | 'heading_deg';

export function recordedLaps(run: ReplayDetail): number[] {
  if (!(run.lap_length_m && run.lap_length_m > 0)) return [];
  return [...new Set(run.reference.flatMap(point => point.lap === null ? [] : [point.lap]))].sort((a,b) => a-b);
}

// A finish-line passage connects the end of one lap with the beginning of the next.
export function selectWindow(run: ReplayDetail, section: Section | undefined, lap: number | null) {
  const length = run.lap_length_m;
  const wrap = Boolean(section && section.start_m > section.end_m && length);
  function locate<T extends CoursePosition>(points: T[]): Located<T>[] {
    const visible: Located<T>[] = [];
    for (const point of points) {
      let at = point.distance_m;
      if (lap !== null && length) {
        if (point.station_m === null) continue;
        if (wrap && section) {
          if (point.lap === lap && point.station_m >= section.start_m) at = point.station_m - length;
          else if (point.lap === lap + 1 && point.station_m <= section.end_m) at = point.station_m;
          else continue;
        } else {
          if (point.lap !== lap) continue;
          at = point.station_m;
          if (section && (at < section.start_m || at > section.end_m)) continue;
        }
      }
      visible.push({ at, point });
    }
    return visible;
  }
  const reference = locate(run.reference);
  const samples = locate(run.samples);
  const bounds = [reference[0]?.at, reference.at(-1)?.at, samples[0]?.at, samples.at(-1)?.at]
    .filter((n): n is number => n !== undefined);
  const min = bounds.length ? Math.min(...bounds) : 0;
  const max = bounds.length ? Math.max(...bounds) : min;
  const gaps: [number, number][] = [];
  if (!samples.length) gaps.push([min, max]);
  else {
    if (samples[0].at > min) gaps.push([min, samples[0].at]);
    for (let i = 1; i < samples.length; i++) {
      if (samples[i-1].point.segment !== samples[i].point.segment) gaps.push([samples[i-1].at, samples[i].at]);
    }
    if (samples[samples.length-1].at < max) gaps.push([samples[samples.length-1].at, max]);
  }
  return {reference, samples, min, max, gaps, axis: wrap ? '計測線からの距離' : lap === null ? '累積走行距離' : '周回内距離'};
}

export type ReplayWindow = ReturnType<typeof selectWindow>;

export function nearest<T>(rows: Located<T>[], at: number): Located<T> | undefined {
  if (!rows.length) return undefined;
  let left = 0, right = rows.length - 1;
  while (left < right) {
    const mid = (left + right) >>> 1;
    if (rows[mid].at < at) left = mid + 1;
    else right = mid;
  }
  if (left > 0 && at - rows[left - 1].at <= rows[left].at - at) return rows[left - 1];
  return rows[left];
}

export function errorAt(view: ReplayWindow, at: number) {
  const values = view.samples;
  if (!values.length || at < values[0].at || at > values[values.length-1].at ||
      view.gaps.some(([start, end]) => at > start && at < end)) return undefined;
  return nearest(values, at);
}

export function seriesPath<T>(data: Located<T>[], xy: (item: Located<T>) => number[],
                              split: (prev: T, next: T) => boolean = () => false) {
  return data.map((row, i) => {
    const [x,y] = xy(row);
    return (i === 0 || split(data[i-1].point, row.point) ? 'M' : 'L') + x.toFixed(2) + ',' + y.toFixed(2);
  }).join(' ');
}
