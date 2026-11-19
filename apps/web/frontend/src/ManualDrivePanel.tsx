import { useEffect, useState } from 'react';
import { driveReadout, MONITOR_RESPONSE_MS } from './drive_monitor';
import type { ManualDriveObservation } from './drive_monitor';
import './drive_monitor.css';

function axisText(value: number | null) {
  return value === null ? '—' : (value > 0 ? '+' : '') + value.toFixed(2);
}

export function ManualDrivePanel({connected, observation, requestedAt}: {
  connected: boolean; observation?: ManualDriveObservation; requestedAt?: number;
}) {
  const [, refresh] = useState(0);
  useEffect(() => {
    if (!connected || requestedAt === undefined) return;
    const delay = requestedAt + MONITOR_RESPONSE_MS - performance.now();
    if (delay <= 0) return;
    const timer = setTimeout(() => refresh(n => n + 1), Math.ceil(delay) + 1);
    return () => clearTimeout(timer);
  }, [connected, requestedAt]);
  const expired = requestedAt !== undefined && performance.now() - requestedAt >= MONITOR_RESPONSE_MS;
  const current = connected && !expired ? observation : undefined;
  const row = driveReadout(current);
  const board = current?.board;
  const output = board?.source === 'cdc0';
  const issue = !connected ? 'Jetson の監視接続を待っています。' : expired ? '監視応答の更新が途切れました。'
    : !current ? '手動走行の状態を待っています。' : '';
  const delta = output && current?.joy && Number.isFinite(board.stamp) && Number.isFinite(current.joy.stamp)
    ? Math.round(Math.abs(board.stamp - current.joy.stamp) * 1000) : null;

  return <section className="content-card manual-drive-panel" aria-label="手動走行モニター">
    <div className="manual-drive-heading"><h3>手動走行モニター</h3>
      {delta !== null && <small>入力と基板の観測時刻差 {delta} ms</small>}
    </div>
    <div className="manual-drive-sources">
      <div><span>プロポ CH3 の選択</span><strong>{row.ch3}</strong>
        {row.ch3Pulse !== null && <small>{row.ch3Pulse} µs</small>}
        {row.ch3Raw !== null && <small>{row.ch3Raw} S.BUS raw</small>}
      </div>
      <div><span>現在の操作元</span><strong>{row.route}</strong></div>
    </div>
    <table className="manual-drive-signals">
      <thead><tr><th scope="col">操作</th><th scope="col">DualSense 入力 <small>正規化 −1〜+1</small></th>
        <th scope="col">Pico 報告 PWM <small>パルス幅 µs</small></th></tr></thead>
      <tbody>{[
        {axis:'操舵', input:row.steering, actuator:'サーボ', pwm:row.servo},
        {axis:'スロットル', input:row.throttle, actuator:'ESC', pwm:row.esc},
      ].map(({axis,input,actuator,pwm}) => <tr key={axis}><th scope="row">{axis}</th>
        <td>{axisText(input)}</td><td><span>{actuator}</span> {pwm === null ? '—' : pwm + ' µs'}</td></tr>)}</tbody>
    </table>
    {issue && <p className="drive-note">{issue}</p>}
    {current?.joy_reason && !current.joy && <p className="drive-note">{current.joy_reason}</p>}
    {current?.board_reason && <p className="drive-note">{output && 'Jetson の走行ノード · '}{current.board_reason}</p>}
    {output && board.pwm_enabled === false && <p className="drive-note">基板の PWM 出力は無効です。</p>}
    {current && !output && <p className="drive-note">PWM を見るには Jetson で <code>./scripts/bringup.sh joy-drive</code> または <code>manual</code> を起動してください。</p>}
  </section>;
}
