export const VEHICLE_DIMENSIONS = ['wheelbase', 'front_track', 'rear_track', 'body_length', 'body_width', 'body_height', 'rear_overhang'] as const;
export const VEHICLE_XYZ = ['x', 'y', 'z'] as const;
export const VEHICLE_RPY = ['roll', 'pitch', 'yaw'] as const;
export type VehicleDimension = typeof VEHICLE_DIMENSIONS[number];
export type VehicleSource = 'unmeasured' | 'manual' | 'cad_candidate';
export interface VehicleProfile {
  version: 1;
  dimensions: Record<VehicleDimension, { value_m: number | null; source: VehicleSource }>;
  camera: { enabled: boolean; translation_m: Record<typeof VEHICLE_XYZ[number], number | null>;
    rotation_rpy_rad: Record<typeof VEHICLE_RPY[number], number | null>; source: VehicleSource };
}
export interface VehicleDraft {
  dimensions: Record<VehicleDimension, { value_mm: string; source: VehicleSource }>;
  camera: { enabled: boolean; translation_mm: Record<typeof VEHICLE_XYZ[number], string>;
    rotation_rpy_deg: Record<typeof VEHICLE_RPY[number], string>; source: VehicleSource };
}
export interface VehicleIdentity { instance_id?: string; hostname?: string }
export interface VehicleCadReference {
  source: 'cad_candidate'; path: string; sha256: string; axis_extent_m: { x: number; y: number; z: number }; caveat: string;
}
export interface VehicleProfileResult {
  connected: boolean; identity?: VehicleIdentity; profile: VehicleProfile; revision: string; saved: boolean;
  path: string; applied: boolean; pending_restart: boolean; active_profile: VehicleProfile | null;
  frames: { base: string; rear_axle: string; front_axle: string; camera: string };
  dimension_fields: string[]; sources: VehicleSource[];
  cad_reference?: VehicleCadReference;
}
export interface VehicleState {
  connected: boolean; identity?: VehicleIdentity; ready: boolean;
  status: { profile_valid: boolean; profile_path: string; active_profile: VehicleProfile | null;
    tf_published: boolean; published_frames: string[]; instance_id: string; reason: string;
    stamp?: number; age_s: number; stale: boolean } | null;
  availability?: unknown;
}
export const VEHICLE_LABELS: Record<VehicleDimension, { label: string; hint: string }> = {
  wheelbase: { label: 'ホイールベース', hint: '後輪と前輪の車軸中心間' },
  front_track: { label: '前輪トレッド', hint: '前輪の左右回転中心間' },
  rear_track: { label: '後輪トレッド', hint: '後輪の左右回転中心間' },
  body_length: { label: '車体全長', hint: '車体外形の前端から後端まで' },
  body_width: { label: '車体全幅', hint: '車体外形の左右端の幅' },
  body_height: { label: '車体全高', hint: '路面から車体上端まで。車軸の高さとは別' },
  rear_overhang: { label: '後方オーバーハング', hint: '後輪車軸中心から車体後端までの正の長さ' },
};

function sameKeys(value: unknown, keys: readonly string[]): boolean {
  return !!value && typeof value === 'object' && !Array.isArray(value)
    && Object.keys(value).length === keys.length && keys.every(key => Object.hasOwn(value, key));
}
function sourceIssue(source: unknown, known: boolean): boolean {
  return known ? source !== 'manual' && source !== 'cad_candidate' : source !== 'unmeasured';
}
export function vehicleProfileIssue(profile: VehicleProfile): string {
  if (!sameKeys(profile, ['version', 'dimensions', 'camera']) || profile.version !== 1
      || !sameKeys(profile.dimensions, VEHICLE_DIMENSIONS)) return '車両設定の形式を確認してください。';
  for (const key of VEHICLE_DIMENSIONS) {
    const item = profile.dimensions[key], value = item?.value_m;
    if (!sameKeys(item, ['value_m', 'source']) || sourceIssue(item.source, value !== null)) return '寸法と測定元の設定を確認してください。';
    if (value !== null && (typeof value !== 'number' || !Number.isFinite(value) || value > 20
        || (key === 'rear_overhang' ? value < 0 : value <= 0))) return `${VEHICLE_LABELS[key].label}の値を確認してください。`;
  }
  const camera = profile.camera;
  if (!sameKeys(camera, ['enabled', 'translation_m', 'rotation_rpy_rad', 'source']) || typeof camera.enabled !== 'boolean'
      || !sameKeys(camera.translation_m, VEHICLE_XYZ) || !sameKeys(camera.rotation_rpy_rad, VEHICLE_RPY)) return 'カメラ配置の形式を確認してください。';
  const xyz = VEHICLE_XYZ.map(key => camera.translation_m[key]), rpy = VEHICLE_RPY.map(key => camera.rotation_rpy_rad[key]);
  if (xyz.some(value => value !== null && (typeof value !== 'number' || !Number.isFinite(value) || Math.abs(value) > 20))
      || rpy.some(value => value !== null && (typeof value !== 'number' || !Number.isFinite(value) || Math.abs(value) > Math.PI))) return 'カメラ位置は ±20000 mm、姿勢は ±180° の範囲にしてください。';
  if (sourceIssue(camera.source, [...xyz, ...rpy].some(value => value !== null))) return 'カメラ配置の測定元を確認してください。';
  if (camera.enabled && [...xyz, ...rpy].some(value => value === null)) return 'カメラ TF を使うには位置と姿勢の6項目をすべて入力してください。';
  return '';
}

const displayValue = (value: number | null, scale: number) => value === null ? '' : String(Number((value * scale).toPrecision(12)));
export function vehicleDraft(profile: VehicleProfile): VehicleDraft {
  return {
    dimensions: Object.fromEntries(VEHICLE_DIMENSIONS.map(key => [key, { value_mm: displayValue(profile.dimensions[key].value_m, 1000), source: profile.dimensions[key].source }])) as VehicleDraft['dimensions'],
    camera: { enabled: profile.camera.enabled, source: profile.camera.source,
      translation_mm: Object.fromEntries(VEHICLE_XYZ.map(key => [key, displayValue(profile.camera.translation_m[key], 1000)])) as VehicleDraft['camera']['translation_mm'],
      rotation_rpy_deg: Object.fromEntries(VEHICLE_RPY.map(key => [key, displayValue(profile.camera.rotation_rpy_rad[key], 180 / Math.PI)])) as VehicleDraft['camera']['rotation_rpy_deg'] },
  };
}

/** 空欄は null のまま保持し、入力途中・不正値をプレビューの原点へ補完しない。 */
export function vehicleDraftProfile(draft: VehicleDraft): { profile: VehicleProfile; issue: string } {
  let parseError = false;
  function parse(text: string, scale: number, minimum: number, maximum: number, positive = false): number | null {
    if (!text.trim()) return null;
    const number = /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?$/i.test(text.trim()) ? Number(text) / scale : NaN;
    if (!Number.isFinite(number) || number < minimum || number > maximum || positive && number === 0) { parseError = true; return null; }
    return number;
  }
  const dimensions = Object.fromEntries(VEHICLE_DIMENSIONS.map(key => {
    const value_m = parse(draft.dimensions[key].value_mm, 1000, 0, 20, key !== 'rear_overhang');
    return [key, { value_m, source: value_m === null ? 'unmeasured' : draft.dimensions[key].source }];
  })) as VehicleProfile['dimensions'];
  const translation_m = Object.fromEntries(VEHICLE_XYZ.map(key => [key, parse(draft.camera.translation_mm[key], 1000, -20, 20)])) as VehicleProfile['camera']['translation_m'];
  const rotation_rpy_rad = Object.fromEntries(VEHICLE_RPY.map(key => [key, parse(draft.camera.rotation_rpy_deg[key], 180 / Math.PI, -Math.PI, Math.PI)])) as VehicleProfile['camera']['rotation_rpy_rad'];
  const known = [...Object.values(translation_m), ...Object.values(rotation_rpy_rad)].some(value => value !== null);
  const profile: VehicleProfile = { version: 1, dimensions, camera: { enabled: draft.camera.enabled, translation_m, rotation_rpy_rad, source: known ? draft.camera.source : 'unmeasured' } };
  return { profile, issue: parseError ? '寸法は 0 ～ 20000 mm（後方オーバーハング以外は正の値）、位置は ±20000 mm、姿勢は ±180° の数値で入力してください。' : vehicleProfileIssue(profile) };
}
export const sameVehicleDraft = (a: VehicleDraft | null, b: VehicleDraft | null) => !!a && !!b && JSON.stringify(a) === JSON.stringify(b);
export function sameVehicleProfile(a: VehicleProfile | null | undefined, b: VehicleProfile | null | undefined): boolean {
  return !!a && !!b && !vehicleProfileIssue(a) && !vehicleProfileIssue(b)
    && VEHICLE_DIMENSIONS.every(key => a.dimensions[key].value_m === b.dimensions[key].value_m && a.dimensions[key].source === b.dimensions[key].source)
    && a.camera.enabled === b.camera.enabled && a.camera.source === b.camera.source
    && VEHICLE_XYZ.every(key => a.camera.translation_m[key] === b.camera.translation_m[key])
    && VEHICLE_RPY.every(key => a.camera.rotation_rpy_rad[key] === b.camera.rotation_rpy_rad[key]);
}
export function vehicleIdentityMatches(target: VehicleIdentity | null, current: VehicleIdentity | undefined): boolean {
  return !!target?.instance_id && target.instance_id === current?.instance_id;
}
export function vehicleStatusFresh(state: VehicleState | null, receivedAt: number, now: number): boolean {
  return !!state?.connected && receivedAt > 0 && now >= receivedAt && now - receivedAt <= 3000
    && state.status?.stale === false && typeof state.status.age_s === 'number' && Number.isFinite(state.status.age_s)
    && state.status.age_s >= 0 && state.status.age_s + (now - receivedAt) / 1000 <= 3;
}
export function vehicleApplied(state: VehicleState | null, receivedAt: number, now: number, profile: VehicleProfile | null, path: string): boolean {
  return vehicleStatusFresh(state, receivedAt, now) && state?.status?.profile_valid === true && state.status.tf_published === true
    && !!path && state.status.profile_path === path && sameVehicleProfile(profile, state.status.active_profile);
}
export function vehicleSourceText(source: VehicleSource): string {
  return source === 'manual' ? '手入力' : source === 'cad_candidate' ? 'CAD 候補' : '未測定';
}
export function vehicleCadReference(value: VehicleCadReference | undefined): VehicleCadReference | null {
  return value?.source === 'cad_candidate' && typeof value.path === 'string' && typeof value.caveat === 'string'
    && typeof value.sha256 === 'string' && /^[a-f0-9]{64}$/.test(value.sha256)
    && VEHICLE_XYZ.every(key => typeof value.axis_extent_m?.[key] === 'number' && Number.isFinite(value.axis_extent_m[key]) && value.axis_extent_m[key] > 0)
    ? value : null;
}

export type VehicleVector = [number, number, number];
/** 固定軸 RPY: Rz(yaw) Ry(pitch) Rx(roll)。camera_link の基底を base_link で表す。 */
export function vehicleCameraAxes(profile: VehicleProfile): VehicleVector[] | null {
  const { roll, pitch, yaw } = profile.camera.rotation_rpy_rad;
  if (roll === null || pitch === null || yaw === null) return null;
  const cr = Math.cos(roll), sr = Math.sin(roll), cp = Math.cos(pitch), sp = Math.sin(pitch), cy = Math.cos(yaw), sy = Math.sin(yaw);
  return [[cy * cp, sy * cp, -sp], [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr], [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr]];
}
export function vehicleBodyPlan(profile: VehicleProfile): { rear: number; front: number; halfWidth: number } | null {
  const length = profile.dimensions.body_length.value_m, width = profile.dimensions.body_width.value_m, rear = profile.dimensions.rear_overhang.value_m;
  return length === null || width === null || rear === null ? null : { rear: -rear, front: length - rear, halfWidth: width / 2 };
}
