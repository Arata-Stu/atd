import type { MapDocument, RacelineLimitKey } from './types';

export const LIMIT_LABELS: Record<RacelineLimitKey, string> = {
  curvature_limit: '曲率上限',
  accel: '加速上限',
  decel: '減速上限',
  lateral_accel: '横加速度上限',
};
export const LIMIT_KEYS = Object.keys(LIMIT_LABELS) as RacelineLimitKey[];

export interface ObservedInterval {
  episode_id: number;
  start_s: number;
  end_s: number;
  speed_min_mps: number;
  speed_max_mps: number;
  side: 'left' | 'right' | 'straight';
  value: number;
  samples: number;
}

export interface LimitCandidate {
  value: number | null;
  unit: string;
  reason: string;
  episodes: number;
  samples: number;
  evidence: ObservedInterval[];
}

export interface VehicleCandidates {
  run_id: string;
  bag: string;
  proposal_sha256: string;
  limits: Record<RacelineLimitKey, LimitCandidate>;
  feedforward: {
    points: {speed_mps: number; throttle_raw: number; speed_min_mps: number; speed_max_mps: number; samples: number; episodes: number}[];
    usable: boolean;
    reason: string;
    direction: 'increasing' | 'decreasing' | null;
    speed_min_mps: number | null;
    speed_max_mps: number | null;
  };
}

export function applicable(candidate: LimitCandidate): boolean {
  return candidate.value !== null && Number.isFinite(candidate.value) && candidate.value > 0;
}

export function adoptionRequest(proposal: VehicleCandidates, map: MapDocument | undefined, checked: RacelineLimitKey[]) {
  const fields = LIMIT_KEYS.filter(key => checked.includes(key) && applicable(proposal.limits[key]));
  if (!map || !fields.length) return null;
  const adopted = map.raceline_calibration;
  if (adopted?.run_id === proposal.run_id && adopted.proposal_sha256 === proposal.proposal_sha256 &&
      Object.keys(adopted.limits).length === fields.length &&
      fields.every(key => adopted.limits[key] === proposal.limits[key].value)) return null;
  return {revision: map.revision, run_id: proposal.run_id, proposal_sha256: proposal.proposal_sha256, fields};
}
