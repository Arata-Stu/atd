export type { Cloud, MapDocument, Parameter } from './types';
export interface LiveStatus {
  connected: boolean; identity?: { hostname?: string; instance_id?: string };
  pose?: { x: number; y: number; z: number; yaw: number; frame: string; stamp: unknown; age_s?: number; stale: boolean } | null;
  velocity?: { linear_mps: number; angular_rps: number; frame: string; stamp: unknown; age_s?: number; stale: boolean } | null;
  topics?: unknown[]; parameter_nodes?: string[]; availability?: unknown;
}
