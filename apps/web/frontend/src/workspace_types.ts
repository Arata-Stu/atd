import type { AutoObservation } from './auto_drive_view';
import type { LiveControl } from './control_view';
import type { LivePlan } from './planning_view';
import type { LiveSpeed } from './speed_view';
import type { LineKind } from './types';
export type { Cloud, MapDocument, Parameter } from './types';
export interface LiveStatus {
  connected: boolean; identity?: { hostname?: string; instance_id?: string; environment?: string }; message?: string;
  pose?: { x: number; y: number; z: number; yaw: number; frame: string; stamp: unknown; age_s?: number; stale: boolean } | null;
  velocity?: { linear_mps: number; angular_rps: number; frame: string; stamp: unknown; age_s?: number; stale: boolean } | null;
  localization?: Localization | null; planning?: LivePlan | null; control?: LiveControl | null; speed_control?: LiveSpeed | null;
  auto_drive?: AutoObservation | null;
  gazebo?: {now_s: number; current: GazeboDriveSample | null; history: GazeboDriveSample[]; ground_truth?: GazeboGroundTruth | null};
  topics?: unknown[]; parameter_nodes?: string[]; availability?: unknown;
}

export interface GazeboGroundTruth {
  frame: string; stamp_s: number; remaining_s: number; x: number; y: number; yaw: number;
  map_id: string; map_sha256: string; hdmap_sha256: string;
  footprint: {vehicle_length: number; vehicle_width: number; rear_axle_to_rear: number; margin: number};
}

export interface GazeboDriveSample {
  stamp_s: number; state: 'waiting' | 'waiting_observation' | 'driving' | 'stopped';
  reason: string; map_id: string; line: LineKind;
  target_mps: number | null; estimated_mps: number | null;
  steering_rad: number | null; throttle: number | null;
}

export type TrackingState = 'waiting_hint' | 'localizing' | 'tracking' | 'lost' | 'stale';
export interface Localization {
  map_id: string; map_sha256: string; hdmap_sha256: string; session_id: string; epoch: number;
  state: TrackingState; reason: string; stamp: number; stale: boolean; age_s: number; remaining_s: number;
}
