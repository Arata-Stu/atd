/** Jetson が現在有効と判定した観測。入力と基板のフレームは独立して届く。 */
export interface ManualDriveBoard {
  source: 'cdc0' | 'cdc1'; serial: string; stamp: number; age_s: number;
  ch3: 'propo' | 'jetson' | 'middle' | 'unknown'; rx_mode_us: number | null; ch3_raw: number | null;
  active_path: number; control_source: 'propo' | 'dualsense' | 'host' | 'standby' | 'fault' | 'unknown';
  pwm_enabled: boolean | null; servo_us: number | null; esc_us: number | null; fault_bits: number;
}
export interface ManualDriveJoy {
  steering: number; throttle: number; enable_requested: boolean; stamp: number; age_s: number;
}
export interface ManualDriveObservation {
  board: ManualDriveBoard | null; joy: ManualDriveJoy | null; board_reason: string; joy_reason: string;
}

// SSH／ブラウザからの更新途絶を検出する。ROS 側の元フレームの有効期限とは別。
export const MONITOR_RESPONSE_MS = 1000;

function inRange(value: unknown, lo: number, hi: number): value is number {
  return typeof value === 'number' && Number.isFinite(value) && value >= lo && value <= hi;
}
function pulse(value: unknown) {
  return inRange(value, 1000, 2000) && Number.isInteger(value) ? value : null;
}

export function driveReadout(observation: ManualDriveObservation | undefined | null) {
  const board = observation?.board, joy = observation?.joy;
  const reported = board?.source === 'cdc0';
  const ch3Pulse = reported ? pulse(board.rx_mode_us) : null;
  const ch3Raw = board?.source === 'cdc1' && inRange(board.ch3_raw, 0, 2047) && Number.isInteger(board.ch3_raw)
    ? board.ch3_raw : null;
  let ch3 = '未確認';
  if (ch3Pulse !== null && board) {
    ch3 = {propo:'プロポ側', jetson:'Jetson 側', middle:'中間位置', unknown:'未確認'}[board.ch3] ?? '未確認';
  } else if (reported && board.ch3 === 'unknown' && (board.rx_mode_us === null || board.rx_mode_us === 0)) {
    ch3 = '未受信';
  }
  let route = '未確認';
  if (board?.control_source === 'fault') {
    route = '基板異常';
  } else if (board) {
    switch (board.active_path) {
      case 0: if (board.control_source === 'standby') route = '待機'; break;
      case 1: if (board.control_source === 'propo') route = 'プロポ'; break;
      case 2: route = reported && board.control_source === 'dualsense' ? 'DualSense' : 'HOST（操作元未確認）'; break;
      case 3: route = '基板異常'; break;
    }
  }
  const outputEnabled = reported && board.pwm_enabled === true;
  const steering = inRange(joy?.steering, -1, 1) ? joy.steering : null;
  const throttle = inRange(joy?.throttle, -1, 1) ? joy.throttle : null;
  return {
    ch3, ch3Pulse, ch3Raw, route, steering, throttle,
    servo: outputEnabled ? pulse(board.servo_us) : null,
    esc: outputEnabled ? pulse(board.esc_us) : null,
  };
}
