import type { Job } from './types';

export interface PoseSource { map_id: string; base_frame: string | null; pose_frame?: string | null }
export interface CalibrationRun {
  id: string;
  bag: string;
  eligible_samples: number;
  timeseries_samples: number;
  metadata_sha256: string;
  exclusion_reasons?: Record<string, number> | null;
}
export interface CalibrationOptions {
  poses: PoseSource[];
  warnings?: string[];
  runs: CalibrationRun[];
  metadata_sha256?: string | null;
}

export function validPoseFrame(value: string) {
  return /^[A-Za-z_][A-Za-z_0-9]*(?:\/[A-Za-z_0-9]+)*$/.test(value.trim());
}

/** undefined lets the worker use its verified frame; null means more user input is needed. */
export function requestedPoseFrame(poses: PoseSource[], source: PoseSource | undefined, entered: string) {
  if (!poses.length || source?.pose_frame) return undefined;
  return source && validPoseFrame(entered) ? entered.trim() : null;
}

export function displayedRun(options: CalibrationOptions | null, bag: string, selected: string, completed: string) {
  if (!options) return undefined;
  const chosen = options.runs.find(run => run.id === selected);
  if (chosen) return chosen;
  const candidates = bag ? options.runs.filter(run => run.bag === bag && run.metadata_sha256 === options.metadata_sha256) : options.runs;
  return candidates.find(run => run.id === completed) ?? candidates[0];
}

export function topExclusions(run: CalibrationRun) {
  return Object.entries(run.exclusion_reasons ?? {})
    .filter(([, count]) => Number.isInteger(count) && count > 0)
    .sort(([a, countA], [b, countB]) => countB - countA || a.localeCompare(b)).slice(0, 3);
}

export function referenceCandidates(options: CalibrationOptions) {
  if (!options.metadata_sha256) return [];
  return options.runs.filter(run => run.metadata_sha256 && run.metadata_sha256 !== options.metadata_sha256 && run.eligible_samples > 0);
}

export function wheelbaseMetres(mm: string) {
  if (!mm.trim()) return undefined;
  const metres = Number(mm) / 1000;
  return Number.isFinite(metres) && metres > 0 ? metres : null;
}

export function receivedBag(job: Job | undefined): string {
  return job?.action === 'pull' && job.status === 'succeeded' && job.result?.verified === true &&
    typeof job.result.record === 'string' ? job.result.record : '';
}

export function calibrationHash(bag: string) {
  return '#calibration' + (bag ? '?' + new URLSearchParams({bag}).toString() : '');
}

export function bagFromHash(hash: string) {
  const [page, query] = hash.split('?');
  return page === '#calibration' ? new URLSearchParams(query).get('bag') ?? '' : '';
}
