import { useMemo, type PointerEvent } from 'react';
import { maxGap, poseAt, valueAt, windowRows } from './closed_loop';
import type { Chart, ClosedLoopDetail, Pose, Series, Span, XY } from './closed_loop';
import { RacelineSpeedPath } from './RacelineSpeedPath';
import type { SpeedScale } from './raceline_speed';

const truth = '#1878a0', estimate = '#ad511c', reference = '#577743', contact = '#a73532';
const between = (n:number, [lo,hi]:Span) => n >= lo && n <= hi;

function pointer(event: PointerEvent<SVGSVGElement>) {
  const matrix = event.currentTarget.getScreenCTM();
  return matrix && new DOMPoint(event.clientX, event.clientY).matrixTransform(matrix.inverse());
}
function point(x:number, y:number) { return x.toFixed(2) + ' ' + y.toFixed(2); }
function format(value:number|null, digits:number, unit:string) {
  return value === null ? '—' : value.toFixed(digits) + ' ' + unit;
}

export function ClosedLoopMap({run,span,focus,showLine,lineScale,at,select}: {
  run:ClosedLoopDetail; span:Span; focus:boolean; showLine:boolean; lineScale:SpeedScale | null;
  at:number; select:(t:number)=>void;
}) {
  const picture = useMemo(() => {
    const gt = windowRows(run.samples,span), est = windowRows(run.estimates,span);
    const line = showLine ? run.reference : [];
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
    const include = (x:number, y:number) => {
      xmin = Math.min(xmin,x); xmax = Math.max(xmax,x); ymin = Math.min(ymin,y); ymax = Math.max(ymax,y);
    };
    for (const rows of [gt,est]) for (const p of rows) include(p.x,p.y);
    if (!focus || !Number.isFinite(xmin)) {
      for (const rows of [line,run.boundaries.left,run.boundaries.right]) for (const [x,y] of rows) include(x,y);
    }
    if (!Number.isFinite(xmin)) return null;
    const scale = Math.min(460 / Math.max(xmax-xmin+.25,1), 310 / Math.max(ymax-ymin+.25,1));
    const x = (v:number) => 250 + (v-(xmax+xmin)/2)*scale;
    const y = (v:number) => 174 - (v-(ymax+ymin)/2)*scale;
    const course = (rows:XY[]) => rows.map(([a,b],i) => (i === 0 ? 'M' : 'L') + point(x(a),y(b))).join(' ') + (rows.length ? ' Z' : '');
    const trace = (rows:Pose[]) => rows.map((p,i) => {
      const last = rows[i-1];
      const newSegment = !last || last.segment !== p.segment || p.t-last.t > maxGap;
      return (newSegment ? 'M' : 'L') + point(x(p.x),y(p.y)) + (newSegment ? 'l0 0' : '');
    }).join(' ');
    return {x,y,scale,bar:10 ** Math.floor(Math.log10(80/scale)), gt,est, gtPath:trace(gt), estPath:trace(est),
      line:course(line), linePoints:line.map(([a,b]):XY=>[x(a),y(b)]),
      walls:[course(run.boundaries.left),course(run.boundaries.right)]};
  },[run,span,focus,showLine]);

  if (!picture) return <p className="cl-missing">この範囲に描画できる座標がありません。</p>;
  const {x,y,scale} = picture, gt = poseAt(run.samples,at), est = poseAt(run.estimates,at);
  // Interpolate sine and cosine to preserve heading at ±π.
  const sin = valueAt(run.samples,at,p=>Math.sin(p.yaw_rad)), cos = valueAt(run.samples,at,p=>Math.cos(p.yaw_rad));
  const yaw = sin === null || cos === null ? null : Math.atan2(sin,cos);
  const footprint = run.footprint;
  const hit = run.contact_s !== null && between(run.contact_s,span) ? poseAt(run.samples,run.contact_s) : null;
  return <svg className="replay-map cl-map" viewBox="0 0 500 350" role="img" aria-label="GT・推定・走行ラインの軌跡。地点を選ぶと時系列も移動します"
    onPointerDown={event => {
      const pt = pointer(event);
      if (!pt) return;
      let time:number|undefined, best = Infinity;
      for (const rows of [picture.gt,picture.est]) for (const p of rows) {
        const d = (x(p.x)-pt.x)**2 + (y(p.y)-pt.y)**2;
        if (d < best || (d === best && Math.abs(p.t-at) < Math.abs((time ?? at)-at))) { best = d; time = p.t; }
      }
      if (time !== undefined) select(time);
    }}>
    <defs><pattern id="cl-map-grid" width="20" height="20" patternUnits="userSpaceOnUse"><circle cx="1" cy="1" r=".8" fill="#c6d2c9"/></pattern></defs>
    <rect width="500" height="350" fill="url(#cl-map-grid)"/>
    {picture.walls.map((d,i)=><path key={i} d={d} fill="none" stroke="#8b9990" strokeWidth="1.4"/>)}
    {lineScale && run.reference_speeds_mps
      ? <RacelineSpeedPath points={picture.linePoints} speeds={run.reference_speeds_mps} scale={lineScale} width={5.5}/>
      : <path d={picture.line} fill="none" stroke={reference} strokeWidth="2.3" strokeDasharray="2 4" strokeLinecap="round"/>}
    <path d={picture.gtPath} fill="none" stroke={truth} strokeWidth="3" strokeLinejoin="round" strokeLinecap="round"/>
    <path d={picture.estPath} fill="none" stroke={estimate} strokeWidth="2.2" strokeDasharray="7 3" strokeLinecap="round"/>
    {gt && est && <line x1={x(gt[0])} y1={y(gt[1])} x2={x(est[0])} y2={y(est[1])} stroke="#243b32" strokeWidth="1.5"/>}
    {gt && footprint && yaw !== null && <g transform={'translate('+point(x(gt[0]),y(gt[1]))+') rotate('+(-yaw*180/Math.PI)+')'}>
      <rect x={-footprint.rear_axle_to_rear*scale} y={-footprint.vehicle_width*scale/2} width={footprint.vehicle_length*scale}
        height={footprint.vehicle_width*scale} fill={truth} fillOpacity=".13" stroke={truth} strokeWidth="1.7"/></g>}
    {gt && <circle cx={x(gt[0])} cy={y(gt[1])} r="4.6" fill="white" stroke={truth} strokeWidth="2.4"/>}
    {est && <circle cx={x(est[0])} cy={y(est[1])} r="3.5" fill={estimate} stroke="white" strokeWidth="1"/>}
    {hit && <path transform={'translate('+point(x(hit[0]),y(hit[1]))+')'} d="M-6 -6L6 6 M6 -6L-6 6" stroke={contact} strokeWidth="2.6"><title>接触判定時の後輪軸</title></path>}
    <g transform="translate(14 338)" fill="#42584b"><path d={'M0 -4V0H'+picture.bar*scale+'v-4'} stroke="currentColor" fill="none"/>
      <text y="-7" fontSize="11">{picture.bar} m</text></g>
  </svg>;
}

const left = 49, right = 491, top = 6, bottom = 61;

export function TimelinePlot({chart,span,at,contactTime,select}: {
  chart:Chart; span:Span; at:number; contactTime:number|null; select:(t:number)=>void;
}) {
  const drawing = useMemo(() => {
    const x = (t:number) => left + (right-left)*(t-span[0])/(span[1]-span[0] || 1);
    let low = 0, high = 0;
    const sets = chart.series.map(s=>windowRows(s.rows,span,true));
    let hasData = false;
    const extend = (v:number|null) => {
      if (v !== null) { low = Math.min(low,v); high = Math.max(high,v); hasData = true; }
    };
    chart.series.forEach((s,i)=>{
      for (const p of sets[i]) if (between(p.t,span)) extend(s.get(p));
      for (const t of span) extend(valueAt(s.rows,t,s.get,s.mode));
    });
    if (high === low) high += 10 ** -chart.digits;
    const y = (v:number) => bottom - (bottom-top)*(v-low)/(high-low);
    const paths = sets.map((rows,i)=>{
      const s = chart.series[i];
      let d = '', previous:number|null = null;
      rows.forEach((p,j)=>{
        const v = s.get(p), last = rows[j-1];
        if (v !== null) {
          const start = previous === null || !last || last.segment !== p.segment || p.t-last.t > maxGap ||
            (s.mode === 'degrees' && Math.abs(previous-v) > 180);
          if (start) d += 'M'+point(x(p.t),y(v))+'l0 0';
          else if (s.mode === 'step') d += 'H'+x(p.t).toFixed(2)+'V'+y(v).toFixed(2);
          else d += 'L'+point(x(p.t),y(v));
        }
        previous = v;
      });
      return d;
    });
    return {x,y,paths,low,high,hasData};
  },[chart,span]);
  const {x,y,paths,low,high,hasData} = drawing;
  const reading = (s:Series) => valueAt(s.rows,at,s.get,s.mode);
  function pick(event:PointerEvent<SVGSVGElement>) {
    const pt = pointer(event);
    if (pt) select(span[0] + Math.max(0,Math.min(1,(pt.x-left)/(right-left))) * (span[1]-span[0]));
  }
  return <figure className="replay-chart cl-chart"><figcaption><strong>{chart.label} <small>({chart.unit})</small></strong></figcaption>
    <div className="cl-chart-legend">{chart.series.map(s=><span key={s.label}><i style={{borderColor:s.color,borderTopStyle:s.dashed?'dashed':'solid'}}/>{s.label} <b>{format(reading(s),chart.digits,chart.unit)}</b></span>)}</div>
    {hasData ? <svg viewBox="0 0 510 82" role="img" aria-label={chart.label+'の時系列'} onPointerDown={pick} onPointerMove={e=>{if(e.buttons===1)pick(e);}}>
      {(span[0] === span[1] ? [0] : [0,.25,.5,.75,1]).map(f=>{
        const t=span[0]+f*(span[1]-span[0]);
        return <g key={f}><line x1={x(t)} x2={x(t)} y1={top} y2={bottom} stroke="#e5ece7"/><text x={x(t)} y="76" textAnchor="middle">{t.toFixed(2)}</text></g>;
      })}
      {[low,low<0&&high>0?0:(high+low)/2,high].map(v=><g key={v}><line x1={left} x2={right} y1={y(v)} y2={y(v)} stroke={v===0?'#aabbb1':'#e4ebe6'}/>
        <text x={left-5} y={y(v)+3} textAnchor="end">{Number(v.toPrecision(3))}</text></g>)}
      <svg x={left} y={top-1} width={right-left} height={bottom-top+2} viewBox={left+' '+(top-1)+' '+(right-left)+' '+(bottom-top+2)}>
        {chart.series.map((s,i)=><path key={s.label} d={paths[i]} fill="none" stroke={s.color} strokeWidth="1.8" strokeDasharray={s.dashed?'5 3':undefined} strokeLinecap="round"/>)}
      </svg>
      {contactTime !== null && between(contactTime,span) && <line x1={x(contactTime)} x2={x(contactTime)} y1={top} y2={bottom} stroke={contact} strokeDasharray="4 2" strokeWidth="2"/>}
      <line x1={x(at)} x2={x(at)} y1={top} y2={bottom} stroke="#293e34" strokeWidth="1.25"/>
      {chart.series.map(s=>{
        const value=reading(s);
        return value===null?null:<circle key={s.label} cx={x(at)} cy={y(value)} r="3" fill={s.color} stroke="white"/>;
      })}
    </svg> : <p className="cl-missing">この範囲に記録なし</p>}
    <small className="cl-guide">{chart.hint}</small>
  </figure>;
}
