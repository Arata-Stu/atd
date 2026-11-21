import { useMemo } from 'react';
import type { PointerEvent } from 'react';
import { seriesPath } from './replay_errors';
import type { ErrorSample, Located, Metric, Reference, ReplayWindow } from './replay_errors';

const estimateColor = '#ad511c';
const truthColor = '#1878a0';
export const metrics: { key: Metric; label: string; positive: string; unit: string }[] = [
  { key: 'along_m', label: '進行方向のずれ', positive: '前方が＋', unit: 'm' },
  { key: 'lateral_m', label: '横方向のずれ', positive: '左が＋', unit: 'm' },
  { key: 'heading_deg', label: '向きの誤差', positive: '反時計回りが＋', unit: '°' },
];
function pointer(event: PointerEvent<SVGSVGElement>) {
  const matrix = event.currentTarget.getScreenCTM();
  return matrix && new DOMPoint(event.clientX, event.clientY).matrixTransform(matrix.inverse());
}

export function CourseMap({course, view, zoom, selected, reference, select}: {
  course: Reference[]; view: ReplayWindow; zoom: boolean; selected?: ErrorSample; reference?: Reference;
  select: (distance: number) => void;
}) {
  const picture = useMemo(() => {
    let xmin = Infinity, ymin = Infinity, xmax = -Infinity, ymax = -Infinity;
    function include(x: number, y: number) {
      xmin = Math.min(xmin,x); ymin = Math.min(ymin,y); xmax = Math.max(xmax,x); ymax = Math.max(ymax,y);
    }
    if (!zoom) course.forEach(p => include(p.x_m,p.y_m));
    view.reference.forEach(({point:p}) => include(p.x_m,p.y_m));
    view.samples.forEach(({point:p}) => { include(p.gt_x_m,p.gt_y_m); include(p.est_x_m,p.est_y_m); });
    if (!Number.isFinite(xmin)) { xmin = ymin = 0; xmax = ymax = 1; }
    const scale = Math.min(458 / Math.max(xmax - xmin, .2), 274 / Math.max(ymax - ymin, .2));
    const x = (value: number) => 250 + scale * (value - (xmax + xmin) / 2);
    const y = (value: number) => 155 - scale * (value - (ymax + ymin) / 2);
    const meters = 10 ** Math.floor(Math.log10(85 / scale));
    const background = course.map((p, i) =>
      (i === 0 || p.lap !== course[i-1].lap ? 'M' : 'L') + x(p.x_m).toFixed(2) + ',' + y(p.y_m).toFixed(2)).join(' ');
    const gt = seriesPath(view.reference, ({point:p}) => [x(p.x_m), y(p.y_m)]);
    const estimate = seriesPath(view.samples, ({point:p}) => [x(p.est_x_m), y(p.est_y_m)],
      (a,b) => a.segment !== b.segment);
    return {x, y, background, gt, estimate, meters, scale};
  }, [course, view, zoom]);
  const {x,y} = picture;
  const gt = selected ? [selected.gt_x_m,selected.gt_y_m] : reference ? [reference.x_m,reference.y_m] : null;
  return <svg className="replay-map" viewBox="0 0 500 335" role="img" aria-label="コース図。青が Gazebo、橙の破線が cuVSLAM。コース上をクリックして地点を選択"
    onPointerDown={event => {
      const pt = pointer(event);
      if (!pt) return;
      let best: Located<Reference> | undefined, distance = Infinity;
      for (const row of view.reference) {
        const dx = x(row.point.x_m) - pt.x, dy = y(row.point.y_m) - pt.y;
        const d = dx*dx + dy*dy;
        if (d < distance) { distance = d; best = row; }
      }
      if (best) select(best.at);
    }}>
    <defs><pattern id="replay-map-grid" width="22" height="22" patternUnits="userSpaceOnUse"><circle cx="1" cy="1" r="0.8" fill="#b8c8c1"/></pattern></defs>
    <rect width="500" height="335" fill="url(#replay-map-grid)"/>
    <path d={picture.background} fill="none" stroke="#c9d4ce" strokeWidth="6" strokeLinejoin="round"/>
    <path d={picture.gt} fill="none" stroke={truthColor} strokeWidth="3.2" strokeLinejoin="round"/>
    <path d={picture.estimate} fill="none" stroke={estimateColor} strokeWidth="2.3" strokeDasharray="6 2" strokeLinejoin="round"/>
    {gt && <g>{selected && <line x1={x(gt[0])} y1={y(gt[1])} x2={x(selected.est_x_m)} y2={y(selected.est_y_m)} stroke="#293d34" strokeWidth="1.8"/>}
      <circle cx={x(gt[0])} cy={y(gt[1])} r="5.2" stroke={truthColor} fill="white" strokeWidth="2.7"/>
      {selected && <circle cx={x(selected.est_x_m)} cy={y(selected.est_y_m)} r="3.6" fill={estimateColor} stroke="white" strokeWidth="1.2"/>}
    </g>}
    <g transform="translate(15 322)" fill="#55675d">
      <path d={'M0 -4 V0 H' + picture.meters*picture.scale + ' V-4'} fill="none" stroke="currentColor"/>
      <text x="0" y="-7" fontSize="11">{picture.meters} m</text>
    </g>
  </svg>;
}

const left = 53, right = 505, top = 5, bottom = 71;
export function ErrorChart({metric, view, at, selection, select}: {
  metric: typeof metrics[number]; view: ReplayWindow; at: number; selection?: Located<ErrorSample>;
  select: (distance: number) => void;
}) {
  const drawing = useMemo(() => {
    const extent = view.samples.reduce((acc, row) => Math.max(acc,Math.abs(row.point[metric.key])), 0);
    const half = Math.max(metric.unit === '°' ? .1 : .01,extent);
    const x = (d: number) => left + (right-left)*(d-view.min)/(view.max-view.min || 1);
    const y = (v: number) => (bottom+top)/2 - (bottom-top)/2*v/half;
    const line = seriesPath(view.samples, row => [x(row.at), y(row.point[metric.key])],
      (a,b) => a.segment !== b.segment || (metric.key === 'heading_deg' && Math.abs(a.heading_deg-b.heading_deg)>180));
    return {x, y, half, line};
  }, [view,metric]);
  const {x,y,half} = drawing;
  function pick(event: PointerEvent<SVGSVGElement>) {
    const pt = pointer(event);
    if (pt) select(view.min + Math.max(0,Math.min(1,(pt.x-left)/(right-left))) * (view.max-view.min));
  }
  return <figure className="replay-chart">
    <figcaption><strong>{metric.label} <small>({metric.unit})</small></strong><span>{metric.positive}</span></figcaption>
    <svg viewBox="0 0 530 92" role="img" aria-label={metric.label + 'と' + view.axis + 'のグラフ'} onPointerDown={pick}
      onPointerMove={e => { if (e.buttons === 1) pick(e); }}>
      <defs><pattern id={'replay-gap-' + metric.key} width="6" height="6" patternUnits="userSpaceOnUse">
        <path d="M-1 1L1 -1M0 6L6 0M5 7L7 5" stroke="#a6b1a9" strokeWidth=".9"/>
      </pattern></defs>
      {[0,.25,.5,.75,1].map(t => {
        const s = view.min + (view.max-view.min)*t;
        return <g key={t}><line x1={x(s)} x2={x(s)} y1={top} y2={bottom} stroke="#e6ece8"/>
          <text x={x(s)} y="88" textAnchor="middle">{s.toFixed(1)}</text></g>;
      })}
      {[-half,0,half].map(v => <g key={v}><line x1={left} x2={right} y1={y(v)} y2={y(v)} stroke={v === 0 ? '#9caea3' : '#e3e9e4'}/>
        <text x={left-6} y={y(v)+3} textAnchor="end">{Number(v.toPrecision(3))}</text></g>)}
      {view.gaps.map(([a,b],i) => <rect key={i} x={x(a)} y={top} width={Math.max(0,x(b)-x(a))} height={bottom-top} fill={'url(#replay-gap-'+metric.key+')'}/>)}
      <path d={drawing.line} fill="none" stroke="#256c54" strokeWidth="1.6" strokeLinejoin="round"/>
      <line x1={x(at)} x2={x(at)} y1={top} y2={bottom} stroke="#30473d" strokeDasharray="3 2"/>
      {selection && <circle cx={x(selection.at)} cy={y(selection.point[metric.key])} r="3.7" fill="#1a6349" stroke="white" strokeWidth="1.1"/>}
    </svg>
  </figure>;
}
