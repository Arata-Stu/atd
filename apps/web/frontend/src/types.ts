export type Point = [number, number];
export type LineKind = 'centerline' | 'raceline' | 'custom';
export type EditTarget = 'left' | 'right' | 'custom';
export interface GenerationSettings {
  centerline_method: 'region'; region_resolution: number; centerline_max_points: number;
  optimizer: 'mincurv' | 'mincurv_iqp'; spacing: number; width: number; margin: number;
  vehicle_length: number; rear_axle_to_rear: number; curvature_limit: number;
  max_speed: number; lateral_accel: number; accel: number; decel: number;
}
export type RacelineLimitKey = 'curvature_limit' | 'accel' | 'decel' | 'lateral_accel';
export interface RacelineCalibration {
  run_id: string; bag: string; proposal_sha256: string; limits: Partial<Record<RacelineLimitKey, number>>; applied_at: number;
}
export interface Line { points: Point[]; speeds?: number[]; method?: string; settings?: Partial<GenerationSettings>; raceline_calibration?: RacelineCalibration; [key: string]: unknown }
export interface MapSource { kind?: string; bag?: string; pointcloud_status?: string; [key: string]: unknown }
export interface MapDocument {
  id: string; title: string; revision: number; frame: string; closed: boolean;
  left: Point[]; right: Point[]; custom: Point[]; lines: Record<string, Line>;
  source?: MapSource; raceline_calibration?: RacelineCalibration;
  [key: string]: unknown;
}
export interface Cloud { points: [number, number, number][]; frame: string; source_count: number; truncated: boolean; status?: string; message?: string; trajectory?: unknown; provenance?: unknown }
export interface CaptureOptions {
  ready: boolean; issues: string[]; source_bag: string; bag: string; model_dir: string;
  revision: number; pointcloud_status: string; vgl_ready: boolean; source_kind: string;
  capabilities: { ros: boolean; collector: boolean }; input_shape?: number[];
  [key: string]: unknown;
}
export interface Bag { id: string; name: string; bytes: number }
export interface Job {
  id: string; title?: string; kind?: string; status: string; message?: string;
  created_at?: string; started_at?: string; finished_at?: string; progress?: number;
  cancel_requested?: boolean;
  action?: string; resources?: string[]; result?: { map?: string; revision?: number; pointcloud_ready?: boolean; [key: string]: unknown } | null;
  [key: string]: unknown;
}
export interface Connection { host: string; user: string; port: number; repo_root?: string; record_root?: string; result_root?: string; [key: string]: unknown }
export interface RemoteEntry { name: string; relative: string; is_dir: boolean; is_bag: boolean; bytes: number }
export interface LiveStatus {
  connected: boolean; identity?: unknown; topics?: unknown; availability?: unknown;
  pose?: { x?: number; y?: number; yaw?: number; frame?: string; [key: string]: unknown } | null;
  velocity?: { linear_mps?: number; speed_mps?: number; angular_rps?: number; [key: string]: unknown } | number | null;
  [key: string]: unknown;
}
export interface Parameter { name: string; value: unknown; type?: string | number; read_only?: boolean; description?: string; [key: string]: unknown }
export interface InputStatus { connected: boolean; raw?: unknown; normalized?: unknown; profile?: unknown; configuring?: boolean; ready?: boolean; availability?: unknown; [key: string]: unknown }
