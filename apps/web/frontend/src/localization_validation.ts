import type { MapDocument, Point } from './types';

export interface ValidationOptions {
  maps: {id: string; title: string; revision: number; source_bag: string; has_raceline: boolean; ready: boolean; issue: string | null}[];
  bags: {id: string; eligible: boolean; issue: string | null}[];
  engine_dirs: string[];
  available: boolean;
  unavailable_reason: string;
}
export interface ValidationRun {id: string; map_id: string; bag: string; outcome: string; saved_at?: number}
export interface Observation {time_s: number; point: Point}
export interface TrackingGap {
  kind: 'lost' | 'stale' | 'gap';
  reason: string; start_s: number; end_s: number;
  before: Observation | null; after: Observation | null;
}
export interface ValidationResult extends ValidationRun {
  error: string | null; pose_samples: number | null; displayed_samples: number;
  tracking_time_fraction: number | null; first_tracking_delay_s: number | null; stereo_duration_s: number | null;
  segments: {epoch: number; start_s: number; end_s: number; samples: number; points: Point[]}[];
  states: {time_s: number; state: string; reason: string; epoch: number}[];
  gaps: TrackingGap[]; document: MapDocument | null; map_issue: string | null; source_bag: string;
}
export interface GapMarker {point: Point; gapIndex: number; side: 'before' | 'after'}
export interface TrajectoryOverlay {segments: Point[][]; markers: GapMarker[]}

export const OUTCOMES: Record<string, string> = {
  tracked_to_end: '末尾まで追跡', partial_tracking: '一部区間で追跡', no_tracking: '追跡姿勢なし',
  failed: '検証失敗', cancelled: '中止',
};
export const GAP_LABELS = {lost: '追跡喪失', stale: '更新途絶', gap: '観測の空白'};

export function resultMapIssue(result: ValidationResult): string {
  if (result.map_issue) return result.map_issue;
  if (!result.document || result.document.id !== result.map_id || result.document.frame !== 'map')
    return '検証時の地図を照合できないため、軌跡を重ねられません。';
  return '';
}

export function validationOverlay(result: ValidationResult): TrajectoryOverlay {
  const markers: GapMarker[] = [];
  for (const [gapIndex, gap] of result.gaps.entries()) {
    for (const side of ['before', 'after'] as const) {
      const observation = gap[side];
      if (observation) markers.push({point: observation.point, gapIndex, side});
    }
  }
  return {segments: result.segments.map(segment => segment.points), markers};
}

export function seconds(time: number | null): string { return time === null ? '—' : time.toFixed(2) + ' s'; }
