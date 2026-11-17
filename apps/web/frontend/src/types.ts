export type Point = [number, number];
export type LineKind = 'centerline' | 'raceline' | 'custom';
export type EditTarget = 'left' | 'right' | 'custom';
export interface Line { points: Point[]; speeds?: number[]; [key: string]: unknown }
export interface MapDocument {
  id: string; title: string; revision: number; frame: string; closed: boolean;
  left: Point[]; right: Point[]; custom: Point[]; lines: Record<string, Line>;
  [key: string]: unknown;
}
export interface Cloud { points: [number, number, number][]; frame: string; source_count: number; truncated: boolean }
export interface Bag { id: string; name: string; bytes: number }
export interface Job {
  id: string; title?: string; kind?: string; status: string; message?: string;
  created_at?: string; started_at?: string; finished_at?: string; progress?: number;
  cancel_requested?: boolean;
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
