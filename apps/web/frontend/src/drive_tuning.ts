import type { LineKind, Parameter } from './types';

export type TuningTarget = 'jetson' | 'gazebo';
export const tuningNodes: Record<TuningTarget, string> = {
  gazebo: '/atd_gazebo_control', jetson: '/atd_auto_drive',
};

export const tuningFields = {
  max_speed_mps: {label: '速度上限', unit: 'm/s'},
  min_lookahead_m: {label: '最小距離', unit: 'm'},
  lookahead_time_s: {label: '先読み時間', unit: 's'},
  max_lookahead_m: {label: '最大距離', unit: 'm'},
  speed_kp: {label: '比例 Kp', unit: 'raw/(m/s)'},
  speed_ki: {label: '積分 Ki', unit: 'raw/m'},
  speed_kd: {label: '微分 Kd', unit: 'raw/(m/s²)'},
} as const;
export type NumericTuningKey = keyof typeof tuningFields;
export type TuningKey = NumericTuningKey | 'line';
export type TuningDraft = Partial<Record<TuningKey, string>>;

export interface TuningSnapshot {
  node: string;
  identity: {instance_id: string; environment: string};
  items: Parameter[];
}

export function matchesTuningSession(snapshot: TuningSnapshot, target: TuningTarget, instanceId: string) {
  return !!instanceId && snapshot.identity?.instance_id === instanceId &&
    snapshot.identity.environment === target && snapshot.node === tuningNodes[target];
}

export function tuningLines(parameters: Parameter[]): LineKind[] {
  // This list is validated against the controller's deployed HDMap at launch.
  const lines = parameters.find(p => p.name === 'available_lines')?.value;
  return Array.isArray(lines) ? lines.filter((line): line is LineKind =>
    line === 'centerline' || line === 'raceline' || line === 'custom') : [];
}

export function speedRange(parameters: Parameter[]) {
  const parameter = parameters.find(p => p.name === 'max_speed_mps');
  const range = Array.isArray(parameter?.floating_point_range) ? parameter.floating_point_range[0] : null;
  const cap = parameters.find(p => p.name === 'launch_speed_cap_mps')?.value;
  return {
    min: Number.isFinite(range?.from_value) ? Number(range.from_value) : undefined,
    max: typeof cap === 'number' && Number.isFinite(cap) ? cap : undefined,
  };
}

export function tuningChanges(items: Parameter[], draft: TuningDraft) {
  const values: Record<string, number | string> = {};
  const params = new Map(items.map(p => [p.name, p]));
  let issue = '';
  const range = speedRange(items);
  for (const [name, text] of Object.entries(draft)) {
    const parameter = params.get(name);
    if (parameter?.allowed !== true || parameter.read_only) {
      issue = 'この項目の変更は許可されていません。';
      break;
    }
    if (name === 'line') {
      if (!tuningLines(items).some(key => key === text)) {
        issue = '走行地図の速度付きラインを選択してください。';
        break;
      }
      if (text !== parameter.value) values[name] = text;
      continue;
    }
    const field = tuningFields[name as NumericTuningKey];
    const n = text.trim() ? Number(text) : NaN;
    if (!field || !Number.isFinite(n) || n < 0 ||
        (['min_lookahead_m','max_lookahead_m','max_speed_mps'].includes(name) && n === 0)) {
      issue = (field?.label ?? name) + 'の値を確認してください。';
      break;
    }
    if (name === 'max_speed_mps' && range.min !== undefined && n < range.min) {
      issue = `速度上限は ${range.min} m/s 以上にしてください。`;
      break;
    }
    if (name === 'max_speed_mps' && range.max !== undefined && n > range.max) {
      issue = `速度上限は起動時の ${range.max} m/s までです。`;
      break;
    }
    if (n !== parameter.value) values[name] = n;
  }
  const min = values.min_lookahead_m ?? params.get('min_lookahead_m')?.value;
  const max = values.max_lookahead_m ?? params.get('max_lookahead_m')?.value;
  if (!issue && typeof min === 'number' && typeof max === 'number' && min > max)
    issue = '先読みの最大距離は最小距離以上にしてください。';
  return {values, issue};
}
