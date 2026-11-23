import { useEffect, useMemo, useState } from 'react';
import type { ReactNode } from 'react';
import { AUTO_HISTORY_S, autoReadout, autoSegments, autoStates, commandLabels, controlSources,
  switchPositions } from './auto_drive_view';
import type { AutoObservation, AutoSeries } from './auto_drive_view';
import { LINE_NAMES } from './planning_view';
import { captureTiming, clock, clockText } from './live_timing';
import './auto_drive.css';

function value(v?: number | null, digits = 0) {
  return typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : '—';
}
interface PlotAxis {
  name: string; unit: string; keys: [AutoSeries, AutoSeries]; labels: [string, string];
}
const axes: PlotAxis[] = [
  {name:'速度', unit:'m/s', keys:['target_mps','estimated_mps'], labels:['目標速度','推定速度']},
  {name:'操舵', unit:'µs', keys:['command_servo','board_servo'], labels:['送信指令相当','サーボ報告']},
  {name:'スロットル', unit:'µs', keys:['command_esc','board_esc'], labels:['送信指令相当','ESC 報告']},
];

function AutoPlot({data, axis, last}: {data: AutoObservation; axis: PlotAxis; last: boolean}) {
  const end = data.now_s;
  const lines = useMemo(() => axis.keys.map(key => autoSegments(data.history, key, end)), [axis, data.history, end]);
  const speed = axis.unit === 'm/s';
  const points = lines.flat(2);
  const values = points.map(p => p[1]);
  const maxSpeed = Math.max(.5, data.current?.max_speed_mps ?? 0, ...values.map(Math.abs));
  // 低速時の小さな PWM 差も読めるよう、中立を含む報告範囲に合わせる。
  const deviation = Math.max(0, ...values.map(v => Math.abs(v - 1500)));
  const pwmRange = Math.min(500, Math.ceil((deviation + 10) / 50) * 50);
  const high = speed ? Math.ceil(maxSpeed * 2) / 2 : 1500 + pwmRange;
  const low = speed ? (values.some(v => v < 0) ? -high : 0) : 1500 - pwmRange;
  return <LiveTrace name={axis.name} unit={axis.unit} labels={axis.labels} lines={lines} end={end}
    low={low} high={high} last={last} stopTime={data.last_stop?.stamp_s}/>;
}

export function LiveTrace({name, unit, labels, lines, end, low, high, last, stopTime}: {
  name: string; unit: string; labels: string[]; lines: [number,number][][][];
  end: number; low: number; high: number; last: boolean; stopTime?: number;
}) {
  const y = (v: number) => 4 + 92 * (high-v) / (high-low);
  const x = (t: number) => 1000 * (1 - (end-t) / AUTO_HISTORY_S);
  return <figure className="auto-trace">
    <figcaption><strong>{name} <small>[{unit}]</small></strong>
      {labels.map((label, i) => <span className={'auto-key auto-key-' + i} key={label}>{label}</span>)}
    </figcaption>
    <div className="auto-trace-grid">
      <div className="auto-ticks-y">{[high,(high+low)/2,low].map(t => <span key={t}>{Number(t.toFixed(2))}</span>)}</div>
      <div className="auto-trace-area">
        <svg viewBox="0 0 1000 100" preserveAspectRatio="none" role="img" aria-label={name + 'の直近15秒の折れ線グラフ'}>
          {[4,50,96].map(h => <line key={h} x1={0} x2={1000} y1={h} y2={h} className="auto-gridline"/>)}
          {[0,1000/3,2000/3,1000].map(w => <line key={w} x1={w} x2={w} y1={4} y2={96} className="auto-gridline"/>)}
          {lines.map((_,i)=>i).reverse().map(i => <g className={'auto-line auto-line-' + i} key={i}>
            {lines[i].map((segment, k) => segment.length === 1
              ? <line key={k} x1={x(segment[0][0])} x2={x(segment[0][0])+.3} y1={y(segment[0][1])} y2={y(segment[0][1])}/>
              : <polyline key={k} points={segment.map(([t,v]) => x(t).toFixed(2) + ',' + y(v).toFixed(2)).join(' ')}/>)}
          </g>)}
          {stopTime !== undefined && stopTime >= end - AUTO_HISTORY_S && stopTime <= end &&
            <line className="auto-stop-line" x1={x(stopTime)} x2={x(stopTime)} y1={0} y2={100}><title>停止要求 {clockText(stopTime)}</title></line>}
        </svg>
        {!lines.some(series => series.length) && <span className="auto-trace-empty">この区間は未観測</span>}
      </div>
      {last && <div className="auto-ticks-x"><span>−15 s</span><span>−10 s</span><span>−5 s</span><span>0 s</span></div>}
    </div>
  </figure>;
}

export function AutoDrivePanel({data, enabled, connected, requestStartedAt, capture, receivedAtMs, roundTripMs, captureMap, fallbackPlot}: {
  data?: AutoObservation | null; enabled: boolean; connected: boolean; requestStartedAt: number;
  capture: boolean; receivedAtMs?: number; roundTripMs?: number; captureMap: ReactNode; fallbackPlot: ReactNode;
}) {
  const [, refresh] = useState(0);
  // Jetson と Notebook の時計を比較せず、通信時間を含む経過の上限で現在値を判定する。
  const elapsed = enabled && connected ? Math.max(0, performance.now() - requestStartedAt) / 1000 : Infinity;
  const observed = enabled && connected;
  const timing = captureTiming(data, observed ? roundTripMs ?? null : null);
  const live = autoReadout(data, elapsed);
  const {refreshIn} = live;
  // SSH 経由の観測は採取時刻も表示し、次の応答を待つ間に古くなった項目を明示する。
  const {state, board, source, command, speed} = autoReadout(data, enabled && connected ? 0 : Infinity);
  useEffect(() => {
    if (refreshIn === null) return;
    const timer = setTimeout(() => refresh(t => t + 1), Math.ceil(refreshIn * 1000) + 1);
    return () => clearTimeout(timer);
  }, [refreshIn, data]);
  const history = data?.publishers === 1 && !!data.session_id && data.history.length > 0;
  const stop = data?.publishers === 1 && data.session_id ? data.last_stop : null;
  const atStop = live.state?.state === 'latched_stop' || live.state?.state === 'braking';
  const stopMessage = (atStop || stop) && <div className={'auto-stop' + (atStop ? '' : ' past')} role="status">
    <strong>{atStop ? '停止要求' : '直近の停止要求'}</strong>
    {stop && <time>{clockText(stop.stamp_s)}</time>}<span>{stop?.reason || state?.reason}</span>
  </div>;
  const issue = !enabled ? '表示を一時停止しています。' : !connected ? 'Jetson の観測接続を待っています。'
    : data?.issue || (!data?.publishers ? 'Jetson で autonomous を起動すると観測できます。'
      : !live.state ? '新しい自動走行状態を待っています。' : '');
  return <section className="content-card auto-drive-panel" aria-label="自動走行モニター">
    <header className="card-heading"><h3>自動走行モニター</h3>
      {state && <span>{!capture && <>Jetson 取得 {clockText(data?.now_s)} · </>}{state.map_id} · {LINE_NAMES[state.line]} · 上限 {value(state.max_speed_mps,2)} m/s</span>}</header>
    {capture && <dl className="capture-timing" aria-label="観測時刻と取得時間">
      <div><dt>{!enabled ? 'Jetson 前回取得' : 'Jetson 取得'}</dt><dd>{clockText(data?.now_s)}</dd></div>
      <div><dt>Notebook 受信</dt><dd>{observed && receivedAtMs !== undefined ? clock.format(receivedAtMs) : '—'}</dd></div>
      <div title="Notebook のブラウザが取得を要求してから応答を読み終えるまで"><dt>取得往復</dt>
        <dd>{timing.roundTripMs ?? '—'} <small>ms</small></dd></div>
      <div title="Pico 報告を Jetson が USB で受信してから、Notebook が応答を受け取るまでの時間の範囲">
        <dt>基板報告の経過（受信時）</dt><dd>{timing.boardAgeMs?.join('–') ?? '—'} <small>ms</small></dd></div>
    </dl>}
    {issue && <p className="auto-notice" role="status">{issue}</p>}
    {stopMessage}
    <div className="auto-body">
      {(history || state) && <div className="auto-details">
      <dl className="auto-authority">
        <div><dt>プロポ CH3</dt><dd>{board ? (live.board ? '' : '前回 ') + switchPositions[board.ch3] : '未確認'} <small>{board?.rx_mode_us != null && board.rx_mode_us + ' µs'}</small></dd></div>
        <div><dt>現在の操作元</dt><dd>{board && !live.board && '前回 '}{controlSources[live.board ? live.source : source]}</dd></div>
        <div><dt>自動走行ノード</dt><dd>{state ? (live.state ? '' : '前回 ') + autoStates[state.state] : '未確認'}</dd></div>
      </dl>
      <div className="auto-readings">
        <dl className="auto-speed-current">
          <div><dt>目標速度</dt><dd>{value(speed?.target_mps,2)} <small>m/s</small></dd></div>
          <div><dt>推定速度</dt><dd>{value(speed?.estimated_mps,2)} <small>m/s</small></dd></div>
          <div className="auto-speed-stamp"><dt>速度の観測時刻</dt><dd>{clockText(speed?.stamp_s)}{speed && !live.speed && ' · 前回取得'}</dd></div>
        </dl>
        <div className="auto-table-wrap"><table className="auto-signals">
          <thead><tr><th scope="col">操作</th><th scope="col">Jetson 送信 <small>−1〜+1</small></th>
            <th scope="col">指令相当 <small>µs</small></th><th scope="col">Pico 報告 <small>µs</small></th></tr></thead>
          <tbody><tr><th scope="row">操舵 <small>サーボ</small></th><td>{value(command?.steering,3)}</td>
            <td>{value(command?.servo_us)}</td><td>{value(board?.pwm_enabled ? board.servo_us : null)}</td></tr>
            <tr><th scope="row">スロットル <small>ESC</small></th><td>{value(command?.throttle,3)}</td>
            <td>{value(command?.esc_us)}</td><td>{value(board?.pwm_enabled ? board.esc_us : null)}</td></tr></tbody>
        </table>
        <div className="auto-stamps"><span>送信 {clockText(command?.stamp_s)} {command && commandLabels[command.kind]}{command && !live.command && ' · 前回取得'}</span>
          <span>基板報告の受信 {clockText(board?.stamp_s)}{board?.pwm_enabled === false && ' · PWM 無効'}{board && !live.board && ' · 前回取得'}</span></div></div>
      </div>
      {state?.reason && !['driving','propo','braking','latched_stop'].includes(state.state) && <p className="auto-reason">{state.reason}</p>}
      </div>}
      {captureMap}
      {history ? <div className="auto-timeline"><div className="auto-timeline-title"><h4>直近 {AUTO_HISTORY_S} 秒</h4>
        <span>右端 {clockText(data.now_s)}</span>
        {stop && stop.stamp_s >= data.now_s - AUTO_HISTORY_S && <span className="auto-stop-key">停止要求</span>}</div>
        {axes.map((axis, i) => <AutoPlot key={axis.name} axis={axis} data={data} last={i === axes.length-1}/>)}</div> : fallbackPlot}
    </div>
  </section>;
}
