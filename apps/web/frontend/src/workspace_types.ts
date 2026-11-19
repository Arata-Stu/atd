import type { AutoObservation } from './auto_drive_view';
import type { LiveControl } from './control_view';
import type { LivePlan } from './planning_view';
import type { LiveSpeed } from './speed_view';
export type { Cloud, MapDocument, Parameter } from './types';
export interface LiveStatus {
  connected: boolean; identity?: { hostname?: string; instance_id?: string };
  pose?: { x: number; y: number; z: number; yaw: number; frame: string; stamp: unknown; age_s?: number; stale: boolean } | null;
  velocity?: { linear_mps: number; angular_rps: number; frame: string; stamp: unknown; age_s?: number; stale: boolean } | null;
  localization?: Localization | null; planning?: LivePlan | null; control?: LiveControl | null; speed_control?: LiveSpeed | null;
  auto_drive?: AutoObservation | null;
  topics?: unknown[]; parameter_nodes?: string[]; availability?: unknown;
}

export type TrackingState = 'waiting_hint' | 'localizing' | 'tracking' | 'lost' | 'stale';
export interface Localization {
  map_id: string; map_sha256: string; hdmap_sha256: string; session_id: string; epoch: number;
  state: TrackingState; reason: string; stamp: number; stale: boolean; age_s: number; remaining_s: number;
}
