import type { Cloud, GenerationSettings, Job, LineKind, MapDocument } from './types';

export const DEFAULT_GENERATION_SETTINGS: GenerationSettings = {
  centerline_method: 'region', region_resolution: .02, centerline_max_points: 800,
  optimizer: 'mincurv', spacing: .1, width: .19, margin: .05,
  vehicle_length: .35, rear_axle_to_rear: .1065, curvature_limit: 1,
  max_speed: 1.5, lateral_accel: 2.5, accel: 1.5, decel: 2.5,
};
export const ACTIVE_MAP_JOBS = new Set(['starting', 'queued', 'running', 'stopping', 'cancel_requested']);
const SUCCEEDED = new Set(['succeeded', 'completed']);
export function preparationJobs(jobs: Job[], mapId: string | null) {
  const offlineJob = jobs.find(job => job.resources?.includes('offline-mapping') &&
    (ACTIVE_MAP_JOBS.has(job.status) || job.status === 'orphaned'));
  const appliesToMap = mapId === null
    ? offlineJob?.action === 'map_build'
    : offlineJob?.resources?.includes('map:' + mapId);
  const isPreparation = offlineJob?.action === 'map_build' || offlineJob?.action === 'vgl' || offlineJob?.action === 'capture';
  return { offlineJob, stageJob: appliesToMap && isPreparation ? offlineJob : undefined };
}
export function nvidiaMap(doc: MapDocument | null): boolean { return doc?.source?.kind === 'nvidia_offline'; }
export function editingIssue(doc: MapDocument | null, cloud: Cloud | null): string {
  if (!doc) return '地図を選択してください';
  if (!nvidiaMap(doc)) return '';
  if (doc.source?.pointcloud_status !== 'ready') return '保存地図から点群を取得するとHDMapを編集できます';
  if (cloud?.frame !== 'map' || !cloud.points.length) return '表示用点群を確認できません。点群の状態を確認してください';
  return '';
}
export function generationIssue(doc: MapDocument | null, kind: LineKind, cloud: Cloud | null): string {
  const issue = editingIssue(doc, cloud);
  if (issue || !doc) return issue;
  if (kind !== 'custom' && !doc.closed) return 'Centerline・Racelineは閉路で生成します';
  if (kind === 'raceline' && !doc.lines.centerline?.points.length) return '先にCenterlineを生成してください';
  if (kind === 'raceline' && doc.lines.centerline.method !== 'region_equal_distance_v1') return '領域方式でCenterlineを再生成してください';
  return '';
}
export interface TrackedMapJob { mapId: string; selectionId: string | null; catalogOnly?: boolean }
export interface CompletionUpdate { mapId: string; action: 'refresh' | 'open' | 'defer' | 'catalog' }
export function completionUpdate(job: Job, previous: string | undefined, tracked: TrackedMapJob | undefined, currentMapId: string | null, dirty: boolean): CompletionUpdate | null {
  if (!SUCCEEDED.has(job.status) || (!tracked && (!previous || !ACTIVE_MAP_JOBS.has(previous)))) return null;
  const mapId = tracked?.mapId ?? job.result?.map ?? job.resources?.find(item => item.startsWith('map:'))?.slice(4);
  if (!mapId) return null;
  if (tracked?.catalogOnly) return { mapId, action: 'catalog' };
  if (mapId === currentMapId) return { mapId, action: dirty ? 'defer' : 'refresh' };
  if (tracked && tracked.selectionId === currentMapId && !dirty) return { mapId, action: 'open' };
  return { mapId, action: 'catalog' };
}
export function restoredSettings(doc: MapDocument, kind: LineKind): GenerationSettings {
  const saved = doc.lines[kind]?.settings;
  const adopted = kind === 'raceline' ? doc.raceline_calibration?.limits : undefined;
  const result = { ...DEFAULT_GENERATION_SETTINGS, ...saved, ...adopted, centerline_method: 'region' as const };
  if (result.optimizer !== 'mincurv' && result.optimizer !== 'mincurv_iqp') result.optimizer = 'mincurv';
  return result;
}
export function captureCloudNotes(provenance: unknown): { scope: string; tail: string | null } {
  const scope = '可視化点群（全地図の網羅性は未確認）';
  if (!provenance || typeof provenance !== 'object' || Array.isArray(provenance)) return { scope, tail: null };
  const data = provenance as Record<string, unknown>;
  const age = data.cloud_tail_age_s;
  const tail = data.cloud_within_tail_tolerance === false && typeof age === 'number' && Number.isFinite(age) && age >= 0
    ? `入力末尾より${age.toFixed(2)}秒前の点群` : null;
  return { scope, tail };
}
