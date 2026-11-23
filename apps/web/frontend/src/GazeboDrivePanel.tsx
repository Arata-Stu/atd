import type { ReactNode } from 'react';
import { LiveTrace } from './AutoDrivePanel';
import { AUTO_HISTORY_S } from './auto_drive_view';
import { LINE_NAMES } from './planning_view';
import type { GazeboDriveSample, LiveStatus } from './workspace_types';

const states = {waiting:'発進待ち', waiting_observation:'自己位置・計画待ち', driving:'走行中', stopped:'停止'};
type Metric = 'target_mps' | 'estimated_mps' | 'steering_rad' | 'throttle';

function series(history: GazeboDriveSample[], key: Metric, end: number) {
  const segments: [number, number][][] = [];
  let segment: [number, number][] | null = null;
  let previous = -Infinity;
  for (const sample of history) {
    const t = sample.stamp_s, value = sample[key];
    if (t < end - AUTO_HISTORY_S || t > end || value === null || !Number.isFinite(value)) {
      segment = null;
      continue;
    }
    if (!segment || t < previous || t - previous > .30) {
      segment = [];
      segments.push(segment);
    }
    segment.push([t, key === 'steering_rad' ? value * 180 / Math.PI : value]);
    previous = t;
  }
  return segments;
}

export function GazeboDrivePanel({data, enabled, captureMap}: {
  data: LiveStatus['gazebo']; enabled: boolean; captureMap?: ReactNode;
}) {
  const row = data?.current, end = data?.now_s ?? 0, history = data?.history ?? [];
  const speeds = [series(history,'target_mps',end), series(history,'estimated_mps',end)];
  const speedValues = speeds.flat(2).map(p => p[1]);
  const steering = series(history,'steering_rad',end);
  const steerExtent = Math.max(5, Math.ceil(Math.max(0,...steering.flat().map(p => Math.abs(p[1]))) / 5) * 5);
  const value = (v: number | null | undefined, digits = 2) => v == null ? '—' : v.toFixed(digits);
  return <section className="content-card auto-drive-panel" aria-label="Gazebo 自動走行モニター">
    <header className="card-heading"><h3>Gazebo 自動走行</h3>
      <span>{!enabled ? '表示停止' : row ? states[row.state] : '制御状態を待っています'}
        {row && <> · {row.map_id} · {LINE_NAMES[row.line]}</>}
        {data && <> · シミュレーション時刻 {end.toFixed(2)} s</>}</span>
    </header>
    {row?.reason && <p className="auto-notice">{row.reason}</p>}
    <div className="auto-body">
      <dl className="auto-speed-current">
        <div><dt>目標速度</dt><dd>{value(row?.target_mps)} <small>m/s</small></dd></div>
        <div><dt>推定速度</dt><dd>{value(row?.estimated_mps)} <small>m/s</small></dd></div>
        <div><dt>操舵指令</dt><dd>{value(row?.steering_rad == null ? null : row.steering_rad * 180 / Math.PI, 1)} <small>°</small></dd></div>
        <div><dt>スロットル指令</dt><dd>{value(row?.throttle)} <small>/ 1</small></dd></div>
      </dl>
      {captureMap}
      <div className="auto-timeline"><div className="auto-timeline-title"><h4>直近 {AUTO_HISTORY_S} 秒</h4></div>
        <LiveTrace name="速度" unit="m/s" labels={['目標速度','推定速度']} lines={speeds} end={end}
          low={Math.floor(Math.min(0,...speedValues)*2)/2} high={Math.max(.5,Math.ceil(Math.max(0,...speedValues)*2)/2)} last={false}/>
        <LiveTrace name="操舵" unit="°" labels={['指令角（左が＋）']} lines={[steering]} end={end}
          low={-steerExtent} high={steerExtent} last={false}/>
        <LiveTrace name="スロットル" unit="0〜1" labels={['正規化指令']} lines={[series(history,'throttle',end)]} end={end}
          low={0} high={1} last/>
      </div>
    </div>
  </section>;
}
