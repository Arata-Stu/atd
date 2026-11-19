import { useMemo } from 'react';
import { modeColor, modeLabel, sampleAt } from './simulation';
import type { Comparison, Point, SimRun, SimSample } from './simulation';

function xyLine(points: Point[], x: (v: number) => number, y: (v: number) => number) {
  return points.map(p => x(p.x_m).toFixed(2) + ',' + y(p.y_m).toFixed(2)).join(' ');
}

export function CourseOverlay({ result, time }: { result: Comparison; time: number }) {
  const view = useMemo(() => {
    const w = 500, h = 324, pad = 30;
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
    function bound(point: Point) {
      xmin = Math.min(xmin, point.x_m); xmax = Math.max(xmax, point.x_m);
      ymin = Math.min(ymin, point.y_m); ymax = Math.max(ymax, point.y_m);
    }
    result.course.points.forEach(bound);
    result.course.left?.forEach(bound); result.course.right?.forEach(bound);
    result.runs.forEach(run => run.samples.forEach(bound));
    const scale = Math.min((w - pad * 2) / Math.max(xmax - xmin, 0.1), (h - pad * 2) / Math.max(ymax - ymin, 0.1));
    const x = (v: number) => w/2 + (v - (xmax + xmin)/2) * scale;
    const y = (v: number) => h/2 - (v - (ymax + ymin)/2) * scale;
    const barMeters = 10 ** Math.floor(Math.log10(90 / scale));
    return { x, y, scale, barMeters, reference: xyLine(result.course.points, x, y),
      edges: [result.course.left, result.course.right].map(edge => edge ? xyLine(edge, x, y) : ''),
      traces: result.runs.map(run => ({ mode: run.mode, line: xyLine(run.samples, x, y) })) };
  }, [result]);
  return <svg className="sim-map" viewBox="0 0 500 324" role="img" aria-label="基準ラインと2方式の走行軌跡。車両の向きと先読み点を同じ経過時刻で表示">
    <defs><pattern id="sim-grid" width="22" height="22" patternUnits="userSpaceOnUse"><circle cx="1" cy="1" r="0.8" fill="#c8d5cc"/></pattern></defs>
    <rect width="500" height="324" fill="url(#sim-grid)"/>
    {view.edges.map((edge,i) => edge && <polygon key={i} points={edge} fill="none" stroke="#66786c" strokeWidth="1"/>)}
    <polygon points={view.reference} fill="none" stroke="#77857c" strokeWidth="6" strokeOpacity=".32"/>
    <polygon points={view.reference} fill="none" stroke="#5e7063" strokeWidth="1.6" strokeDasharray="4 4"/>
    {view.traces.map(trace => <polyline key={trace.mode} points={trace.line} fill="none" stroke={modeColor[trace.mode]}
      strokeWidth={trace.mode === 'fixed' ? 3.3 : 2.2} strokeDasharray={trace.mode === 'adaptive' ? '7 3' : undefined} strokeLinejoin="round"/>)}
    {result.runs.map(run => run.collision && <path key={run.mode} d="M -4 -4 L 4 4 M 4 -4 L -4 4"
      transform={'translate(' + view.x(run.collision.x_m) + ' ' + view.y(run.collision.y_m) + ')'}
      stroke={modeColor[run.mode]} strokeWidth="2.4"><title>{modeLabel[run.mode]}の接触 · {run.collision.t_s.toFixed(2)} s</title></path>)}
    {result.runs.map(run => {
      const p = sampleAt(run.samples, time);
      if (!p) return null;
      const x = view.x(p.x_m), y = view.y(p.y_m), tx = view.x(p.target_x_m), ty = view.y(p.target_y_m);
      return <g key={run.mode} stroke={modeColor[run.mode]}><title>{modeLabel[run.mode]} · 先読み {p.lookahead_m.toFixed(2)} m</title>
        <line x1={x} y1={y} x2={tx} y2={ty} strokeWidth="1.6" strokeDasharray="2 3"/>
        <circle cx={tx} cy={ty} r="3.5" fill="white" strokeWidth="2"/>
        <path d="M 8 0 L -5 4 L -3 0 L -5 -4 Z" transform={'translate(' + x + ' ' + y + ') rotate(' + (-p.yaw_rad * 180/Math.PI) + ')'}
          fill={modeColor[run.mode]} stroke="white" strokeWidth="1.1"/>
      </g>;
    })}
    <g transform="translate(15 307)" fill="#4d6355">
      <path d={'M 0 -4 V 0 H ' + (view.barMeters * view.scale) + ' V -4'} fill="none" stroke="currentColor" strokeWidth="1.3"/>
      <text x="0" y="-7" fontSize="11">{view.barMeters} m</text>
    </g>
  </svg>;
}

type SeriesKey = 'speed_mps' | 'target_speed_mps' | 'lateral_error_m' | 'steering_deg' | 'steering_command_deg';
type ChartSpec = { title: string; unit: string; key: SeriesKey; reference?: SeriesKey; guide?: string; decimals: number };
const charts: ChartSpec[] = [
  { title: '横ずれ', unit: 'm', key: 'lateral_error_m', guide: 'ライン進行方向の左が正', decimals: 2 },
  { title: '速度', unit: 'm/s', key: 'speed_mps', reference: 'target_speed_mps', guide: '破線は目標速度', decimals: 2 },
  { title: '操舵角', unit: '°', key: 'steering_deg', reference: 'steering_command_deg', guide: '左旋回が正 · 破線は指令角', decimals: 1 },
];
const left = 46, right = 524, top = 9, bottom = 72;
function TimePlot({ runs, spec, end, time, select }: { runs: SimRun[]; spec: ChartSpec; end: number; time: number; select: (t: number) => void }) {
  const { lines, ticks } = useMemo(() => {
    let low = 0, high = 0;
    const series: { samples: SimSample[]; mode: SimRun['mode']; field: SeriesKey; dashed: boolean }[] = [];
    for (const run of runs) {
      if (spec.reference) series.push({ samples: run.samples, mode: run.mode, field: spec.reference, dashed: true });
      series.push({ samples: run.samples, mode: run.mode, field: spec.key, dashed: false });
    }
    for (const { samples, field } of series) for (const s of samples) {
      low = Math.min(low, s[field]); high = Math.max(high, s[field]);
    }
    // Signed axes stay symmetric, so left/right errors and turns share one scale.
    const span = Math.max(10 ** -spec.decimals, Math.abs(low), Math.abs(high));
    high = span; low = spec.key === 'speed_mps' ? Math.min(0, low) : -span;
    const y = (value: number) => bottom - (value - low) / (high - low) * (bottom - top);
    const lines = series.map(({samples, mode, field, dashed}) => ({ mode, field, dashed,
      points: samples.map(s => ((left + s.t_s/end * (right-left)).toFixed(2) + ',' + y(s[field]).toFixed(2))).join(' ') }));
    const ticks = [low, (low + high)/2, high].map(v => ({ y: y(v), value: v }));
    return { lines, ticks };
  }, [runs, spec, end]);
  return <div className="sim-chart">
    <header><h3>{spec.title} <small>{spec.unit}</small></h3><span>{spec.guide}</span></header>
    <svg viewBox="0 0 546 95" role="img" aria-label={spec.title + 'の時系列'} onClick={event => {
      const rect = event.currentTarget.getScreenCTM();
      if (!rect) return;
      const pt = new DOMPoint(event.clientX, event.clientY).matrixTransform(rect.inverse());
      select(Math.min(end, Math.max(0, (pt.x-left) / (right-left) * end)));
    }}>
      {ticks.map(tick => <g key={tick.y}><line x1={left} x2={right} y1={tick.y} y2={tick.y} stroke="#dfe6df"/>
        <text x={left-6} y={tick.y+4} textAnchor="end">{tick.value.toFixed(spec.decimals)}</text></g>)}
      {[0,.25,.5,.75,1].map(f => <g key={f}><line x1={left + f*(right-left)} x2={left + f*(right-left)} y1={top} y2={bottom} stroke="#e9eee9"/>
        <text x={left + f*(right-left)} y="89" textAnchor="middle">{(end*f).toFixed(1)}</text></g>)}
      {lines.map(line => <polyline key={line.mode+line.field} points={line.points} fill="none" stroke={modeColor[line.mode]} strokeWidth={line.dashed ? 1.4 : 2.1}
        strokeDasharray={line.dashed ? '5 3' : undefined} strokeLinejoin="round"/>)}
      <line x1={left + time/end*(right-left)} x2={left + time/end*(right-left)} y1={top} y2={bottom} stroke="#304d3e" strokeWidth="1.3"/>
    </svg>
  </div>;
}

export function TimePlots({ result, time, end, select }: { result: Comparison; time: number; end: number; select: (t: number) => void }) {
  return <div className="sim-charts">{charts.map(spec => <TimePlot key={spec.key} runs={result.runs} spec={spec} time={time} end={end} select={select}/>)}</div>;
}
