import { useMemo, type MouseEvent, type ReactNode } from 'react';
import { atTime, ch3Labels, ch3State, escPwm, motionLabels, motionState, numericPath, servoPwm, sourceLabels,
  stateBands, STOP_BAND_MPS, symmetricLimit } from './manual_replay';
import type { DriveWindow, Span } from './manual_replay';

const colors: Record<string, string> = {
  propo:'#4f8866', jetson:'#4472a3', dualsense:'#4472a3', middle:'#a47335', host:'#7965a0',
  standby:'#87928b', fault:'#bc514b', unknown:'#8d8891', absent:'#a69683',
  forward:'#387a9a', near_stop:'#9d8533', reverse:'#96527b',
};

function StateBand({rows, labels, bounds}: {
  rows: ReturnType<typeof stateBands>; labels: Record<string,string>; bounds: Span;
}) {
  const x = (v: number) => 1000 * (v - bounds.start_s) / (bounds.end_s - bounds.start_s);
  return <svg viewBox="0 0 1000 20" preserveAspectRatio="none" aria-hidden="true">
    {rows.map(row => <rect key={row.start_s} x={x(row.start_s)} width={Math.max(0, x(row.end_s) - x(row.start_s))}
      height="20" fill={colors[row.state]}><title>{labels[row.state]}</title></rect>)}
  </svg>;
}

function SignalLines({paths}: {paths: string[]}) {
  return <svg viewBox="0 0 1000 100" preserveAspectRatio="none" aria-hidden="true">
    <path d="M0 4 H1000 M0 50 H1000 M0 96 H1000" fill="none" stroke="#d8e1da" vectorEffect="non-scaling-stroke"/>
    {paths.map((d, i) => <path key={i} d={d} className={'drive-signal signal-' + i} fill="none" vectorEffect="non-scaling-stroke"/>)}
  </svg>;
}
function signed(value: number | undefined, digits = 2) {
  return value === undefined ? '—' : (value > 0 ? '+' : '') + value.toFixed(digits);
}
const pct = (value: number) => Math.min(1, Math.max(0, value)) * 100;

function Trace({title, unit, paths, labels, scale, marker, onSeek}: {
  title: string; unit: string; paths: string[]; labels?: string[]; scale: number[]; marker: ReactNode;
  onSeek: (event: MouseEvent<HTMLDivElement>) => void;
}) {
  return <>
    <div className="drive-trace-heading"><strong>{title}</strong><small>{unit}</small>
      {labels && <div>{labels.map((label, i) => <span key={label}><i className={i === 0 ? 'line-first' : 'line-second'}/>{label}</span>)}</div>}
    </div>
    <div className="drive-y-labels">{scale.map(n => <span key={n}>{n}</span>)}</div>
    <div className="drive-trace" aria-label={title} onClick={onSeek}><SignalLines paths={paths}/>{marker}</div>
  </>;
}

export function DriveTimeline({data, cursor, seek}: {data: DriveWindow; cursor: number; seek: (s: number) => void}) {
  const {start_s, end_s, joy, board, speed, imu, propo} = data;
  const shapes = useMemo(() => {
    const speedMax = symmetricLimit(speed, r => r.speed_mps, .1);
    const accelMax = symmetricLimit(imu, r => r.accel_mps2, .5);
    return {
      ch3: stateBands(board, ch3State), route: stateBands(board, row => row.control_source),
      direction: stateBands(speed, row => motionState(row.speed_mps)),
      joy: [numericPath(joy, r => r.steering, data, -1, 1), numericPath(joy, r => r.throttle, data, -1, 1)],
      propo: [numericPath(propo, r => r.throttle_raw, data, 0, 2047)],
      pwm: [numericPath(board, servoPwm, data, 1000, 2000), numericPath(board, escPwm, data, 1000, 2000)],
      speed: [numericPath(speed, r => r.speed_mps, data, -speedMax, speedMax, r => r.segment)], speedMax,
      accel: [numericPath(imu, r => r.accel_mps2, data, -accelMax, accelMax)], accelMax,
    };
  }, [data, board, joy, speed, imu, propo]);
  const b = atTime(board, cursor), j = atTime(joy, cursor);
  const v = atTime(speed, cursor), a = atTime(imu, cursor), ch2 = atTime(propo, cursor);
  const ch3 = b ? ch3Labels[ch3State(b)] : '欠測';
  let ch3Value = '';
  if (b?.rx_mode_us != null) ch3Value = b.rx_mode_us + ' µs';
  else if (b?.ch3_raw != null) ch3Value = b.ch3_raw + ' S.BUS raw';
  const source = b ? sourceLabels[b.control_source] : '欠測';
  const servo = b ? servoPwm(b) : null, esc = b ? escPwm(b) : null;
  const right = end_s - start_s;
  function seekPlot(event: MouseEvent<HTMLDivElement>) {
    const rect = event.currentTarget.getBoundingClientRect();
    const time = start_s + right * pct((event.clientX - rect.left) / rect.width) / 100;
    seek(Math.min(end_s, Math.max(start_s, Math.round(time * 1000) / 1000)));
  }
  const legend = [...new Set([...shapes.ch3.map(r => r.state), ...shapes.route.map(r => r.state)])];
  const names: Record<string,string> = {...ch3Labels, ...sourceLabels};
  const ticks = [0, .25, .5, .75, 1];
  const marker = <i className="drive-time-marker" style={{left:pct(right > 0 ? (cursor-start_s)/right : 0) + '%'}}/>;
  return <>
    {data.speed_issue && <p className="drive-note" role="status">{data.speed_issue}</p>}
    {right > 0 && <div className="drive-tracks" role="group" aria-label="手動走行の時系列">
      <span>CH3 の位置</span><div className="drive-band" onClick={seekPlot}><StateBand rows={shapes.ch3} labels={ch3Labels} bounds={data}/>{marker}</div>
      <span>操作元</span><div className="drive-band" onClick={seekPlot}><StateBand rows={shapes.route} labels={sourceLabels} bounds={data}/>{marker}</div>
      <div className="drive-state-key">{legend.map(state => <span key={state}><i style={{background:colors[state]}}/>{names[state]}</span>)}<span><i className="drive-gap"/>欠測</span></div>
      <Trace title="DualSense 入力" unit="正規化 −1〜+1" labels={['操舵','スロットル']} scale={[1,0,-1]}
        paths={shapes.joy} marker={marker} onSeek={seekPlot}/>
      <Trace title="プロポのスロットル入力" unit="CH2 · S.BUS raw" scale={[2047,1024,0]}
        paths={shapes.propo} marker={marker} onSeek={seekPlot}/>
      <Trace title="Pico 報告 PWM" unit="パルス幅 µs" labels={['サーボ','ESC']} scale={[2000,1500,1000]}
        paths={shapes.pwm} marker={marker} onSeek={seekPlot}/>
      <Trace title="VSLAM 推定速度" unit="車体前後 m/s · 前進が正" scale={[shapes.speedMax,0,-shapes.speedMax]}
        paths={shapes.speed} marker={marker} onSeek={seekPlot}/>
      <span>速度からの判定</span><div className="drive-band" onClick={seekPlot}><StateBand rows={shapes.direction} labels={motionLabels} bounds={data}/>{marker}</div>
      <div className="drive-state-key">{Object.entries(motionLabels).map(([state,label]) =>
        <span key={state}><i style={{background:colors[state]}}/>{label}{state === 'near_stop' && `（±${STOP_BAND_MPS} m/s 以内）`}</span>)}</div>
      <Trace title="IMU 前後加速度（比力）" unit="車体 x 軸 · m/s² · センサ位置、重力未補正"
        scale={[shapes.accelMax,0,-shapes.accelMax]} paths={shapes.accel} marker={marker} onSeek={seekPlot}/>
      <small>経過時間 [s]</small><div className="drive-ticks">{ticks.map(f => <span key={f}>{(start_s + f * right).toFixed(right < 1 ? 3 : 2)}</span>)}</div>
    </div>}
    <div className="drive-inspector" aria-label="選択時刻の値">
      <h4>選択時刻 <strong>{cursor.toFixed(3)} s</strong></h4>
      <dl><div><dt>CH3 の位置</dt><dd>{ch3} <small>{ch3Value}</small></dd></div><div><dt>操作元</dt><dd>{source}</dd></div></dl>
      <table className="manual-drive-signals"><thead><tr><th scope="col">操作</th><th scope="col">DualSense <small>−1〜+1</small></th><th scope="col">プロポ <small>S.BUS raw</small></th><th scope="col">Pico 報告 PWM <small>µs</small></th></tr></thead>
        <tbody><tr><th scope="row">操舵</th><td>{signed(j?.steering)}</td><td>—</td><td><span>サーボ</span> {servo == null ? '—' : servo + ' µs'}</td></tr>
          <tr><th scope="row">スロットル</th><td>{signed(j?.throttle)}</td><td><span>CH2</span> {ch2?.throttle_raw ?? '—'}</td><td><span>ESC</span> {esc == null ? '—' : esc + ' µs'}</td></tr></tbody></table>
      <dl className="drive-motion-values"><div><dt>VSLAM 推定速度</dt><dd>{v ? signed(v.speed_mps,3) + ' m/s' : '欠測'}
        {v && <small> · {motionLabels[motionState(v.speed_mps)]}</small>}</dd></div>
        <div><dt>IMU 前後加速度</dt><dd>{a ? signed(a.accel_mps2,3) + ' m/s²' : '欠測'}</dd></div></dl>
      {b?.source === 'cdc1' && <p className="drive-note">この時刻は CDC1 の受信データのみです。出力 PWM は未記録です。</p>}
      {b?.pwm_enabled === false && <p className="drive-note">この時刻は基板の PWM 出力が無効です。</p>}
    </div>
  </>;
}
