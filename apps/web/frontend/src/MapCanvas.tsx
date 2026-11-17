import { useCallback, useEffect, useRef, useState } from 'react';
import type { Cloud, EditTarget, MapDocument, Point } from './types';
import { Icon } from './ui';

const COLORS = { left: '#40aa96', right: '#d78d51', custom: '#9c7be2', centerline: '#40aa96', raceline: '#538ee4', customline: '#9c7be2' };
interface Props {
  doc: MapDocument | null; cloud?: Cloud | null; target?: EditTarget; mode?: 'select' | 'add' | 'pan';
  visible?: Record<string, boolean>; onChange?: (target: EditTarget, points: Point[]) => void;
  pose?: { x?: number; y?: number; yaw?: number } | null; fitKey?: number; trail?: Point[];
}
export function MapCanvas({ doc, cloud, target = 'left', mode = 'select', visible = {}, onChange, pose, fitKey = 0, trail = [] }: Props) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ width: 500, height: 500 });
  const [view, setView] = useState({ x: 0, y: 0, scale: 38 });
  const [selected, setSelected] = useState(-1);
  const [draft, setDraft] = useState<Point[] | null>(null);
  const [pointerWorld, setPointerWorld] = useState<Point | null>(null);
  const drag = useRef<{ kind: 'pan' | 'point' | 'click'; index: number; start: Point; origin: Point; moved: boolean; points: Point[] } | null>(null);
  const project = useCallback((point: readonly number[]): Point => [size.width / 2 + (point[0] - view.x) * view.scale, size.height / 2 - (point[1] - view.y) * view.scale], [size, view]);
  const world = (x: number, y: number): Point => [(x - size.width / 2) / view.scale + view.x, -(y - size.height / 2) / view.scale + view.y];
  const fit = useCallback(() => {
    const points: Point[] = doc ? [...doc.left, ...doc.right, ...doc.custom, ...Object.values(doc.lines).flatMap(line => line.points)] : [];
    if (points.length === 0 && cloud) points.push(...cloud.points.filter((_, i) => i % Math.max(1, Math.ceil(cloud.points.length / 10000)) === 0).map(p => [p[0], p[1]] as Point));
    if (!points.length) { setView({ x: 0, y: 0, scale: 38 }); return; }
    const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
    const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    setView({ x: (x0 + x1) / 2, y: (y0 + y1) / 2, scale: Math.min(size.width / Math.max(x1 - x0 + 2, 4), size.height / Math.max(y1 - y0 + 2, 4)) * .84 });
  }, [doc, cloud, size]);
  useEffect(() => {
    const node = canvas.current!;
    const observer = new ResizeObserver(entries => { const { width, height } = entries[0].contentRect; setSize({ width, height }); });
    observer.observe(node); return () => observer.disconnect();
  }, []);
  useEffect(() => { fit(); setSelected(-1); setDraft(null); }, [doc?.id, fitKey]);
  useEffect(() => { setSelected(-1); setDraft(null); }, [target]);
  useEffect(() => {
    const node = canvas.current!, ctx = node.getContext('2d')!;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    node.width = size.width * dpr; node.height = size.height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#f1f4f3'; ctx.fillRect(0, 0, size.width, size.height);
    const step = view.scale > 75 ? .5 : view.scale > 16 ? 1 : view.scale > 4 ? 5 : 10;
    const lo = world(0, size.height), hi = world(size.width, 0);
    ctx.lineWidth = 1;
    for (let x = Math.ceil(lo[0] / step) * step; x < hi[0]; x += step) {
      const px = project([x, 0])[0]; ctx.strokeStyle = Math.abs(x) < step / 100 ? '#c6d2ce' : '#e0e7e3';
      ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, size.height); ctx.stroke();
    }
    for (let y = Math.ceil(lo[1] / step) * step; y < hi[1]; y += step) {
      const py = project([0, y])[1]; ctx.strokeStyle = Math.abs(y) < step / 100 ? '#c6d2ce' : '#e0e7e3';
      ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(size.width, py); ctx.stroke();
    }
    if (cloud && visible.cloud !== false) {
      ctx.fillStyle = '#718d7e66';
      const stride = Math.max(1, Math.ceil(cloud.points.length / 60000));
      for (let i = 0; i < cloud.points.length; i += stride) { const [x, y] = project(cloud.points[i]); ctx.fillRect(x, y, 1.5, 1.5); }
    }
    const path = (points: Point[], color: string, width: number, dashed = false, close = false) => {
      if (!points.length) return;
      ctx.beginPath(); points.forEach((p, i) => { const [x, y] = project(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      if (close && points.length > 2) ctx.closePath(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dashed ? [5, 5] : []); ctx.stroke(); ctx.setLineDash([]);
    };
    if (doc) {
      for (const side of ['left', 'right'] as const) path(target === side && draft ? draft : doc[side], COLORS[side], 2, false, doc.closed);
      for (const [kind, line] of Object.entries(doc.lines)) if (visible[kind] !== false) path(line.points, COLORS[kind as keyof typeof COLORS] || '#538ee4', 2.5, false, doc.closed);
      if (visible.custom !== false) path(target === 'custom' && draft ? draft : doc.custom, COLORS.custom, 2, true, doc.closed);
      if (onChange && mode !== 'pan') (draft || doc[target]).forEach((p, i) => {
        const [x, y] = project(p); ctx.beginPath(); ctx.arc(x, y, i === selected ? 6.5 : 4, 0, Math.PI * 2);
        ctx.fillStyle = i === selected ? COLORS[target] : '#fff'; ctx.fill(); ctx.strokeStyle = COLORS[target]; ctx.lineWidth = 2; ctx.stroke();
      });
    }
    if (doc && trail.length) path(trail, '#16835e77', 2, false, false);
    if (pose && typeof pose.x === 'number' && typeof pose.y === 'number') {
      const [x, y] = project([pose.x, pose.y]); ctx.save(); ctx.translate(x, y); ctx.rotate(-(pose.yaw || 0));
      ctx.shadowColor = '#2a806755'; ctx.shadowBlur = 12; ctx.fillStyle = '#16835e';
      ctx.beginPath(); ctx.moveTo(13, 0); ctx.lineTo(-9, -8); ctx.lineTo(-5, 0); ctx.lineTo(-9, 8); ctx.closePath(); ctx.fill(); ctx.restore();
    }
    const scaleWidth = Math.max(1, Math.floor(90 / view.scale / step)) * step * view.scale;
    ctx.strokeStyle = '#65796d'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(24, size.height - 31); ctx.lineTo(24 + scaleWidth, size.height - 31); ctx.stroke();
    ctx.fillStyle = '#65796d'; ctx.font = '11px ui-monospace, monospace'; ctx.fillText(`${(scaleWidth / view.scale).toFixed(step < 1 ? 1 : 0)} m`, 24, size.height - 40);
  }, [doc, cloud, view, size, target, mode, selected, draft, visible, pose, project, onChange, trail]);
  const pointAt = (e: React.PointerEvent | React.WheelEvent): Point => { const rect = canvas.current!.getBoundingClientRect(); return [e.clientX - rect.left, e.clientY - rect.top]; };
  return <div className="map-surface">
    <canvas ref={canvas} tabIndex={0} aria-label={onChange ? '地図編集キャンバス。点をクリックして選択、Deleteで削除。右ドラッグで移動、ホイールで拡大縮小。' : '地図閲覧キャンバス。右ドラッグで移動、ホイールで拡大縮小。'} style={{ cursor: mode === 'pan' ? 'grab' : mode === 'add' ? 'crosshair' : 'default' }}
      onContextMenu={e => e.preventDefault()}
      onWheel={e => { const [x, y] = pointAt(e), at = world(x, y), scale = Math.min(800, Math.max(.5, view.scale * Math.exp(-e.deltaY * .001))); setView({ scale, x: at[0] - (x - size.width / 2) / scale, y: at[1] + (y - size.height / 2) / scale }); }}
      onPointerDown={e => {
        canvas.current!.focus(); canvas.current!.setPointerCapture(e.pointerId); const at = pointAt(e);
        let index = -1;
        if (doc && onChange && mode !== 'pan' && e.button === 0) index = doc[target].findIndex(p => { const screen = project(p); return Math.hypot(screen[0] - at[0], screen[1] - at[1]) < 11; });
        setSelected(index); drag.current = { kind: e.button !== 0 || mode === 'pan' ? 'pan' : index >= 0 ? 'point' : 'click', index, start: at, origin: [view.x, view.y], moved: false, points: doc?.[target].map(p => [...p] as Point) || [] };
      }}
      onPointerMove={e => {
        const at = pointAt(e); setPointerWorld(world(...at)); const active = drag.current; if (!active) return;
        active.moved ||= Math.hypot(at[0] - active.start[0], at[1] - active.start[1]) > 3;
        if (active.kind === 'pan') setView(v => ({ ...v, x: active.origin[0] - (at[0] - active.start[0]) / v.scale, y: active.origin[1] + (at[1] - active.start[1]) / v.scale }));
        if (active.kind === 'point') { active.points[active.index] = world(...at); setDraft([...active.points]); }
      }}
      onPointerUp={e => {
        const active = drag.current; drag.current = null; if (!active) return;
        if (active.kind === 'point' && active.moved) onChange?.(target, active.points);
        else if (active.kind === 'click' && !active.moved && mode === 'add' && doc) { const next = [...doc[target], world(...pointAt(e))]; onChange?.(target, next); setSelected(next.length - 1); }
        setDraft(null);
      }}
      onPointerCancel={() => { drag.current = null; setDraft(null); }}
      onKeyDown={e => { if ((e.key === 'Delete' || e.key === 'Backspace') && selected >= 0 && doc && onChange) { e.preventDefault(); onChange(target, doc[target].filter((_, i) => i !== selected)); setSelected(-1); } }}
    />
    <div className="map-corner-label"><span className="dot"/> {doc?.frame || 'map'} <span>座標 / m</span></div>
    <div className="canvas-controls"><button title="全体を表示" aria-label="全体を表示" onClick={fit}><Icon name="expand"/></button><button aria-label="拡大" onClick={() => setView(v => ({ ...v, scale: Math.min(800, v.scale * 1.4) }))}>＋</button><button aria-label="縮小" onClick={() => setView(v => ({ ...v, scale: Math.max(.5, v.scale / 1.4) }))}>−</button></div>
    {pointerWorld && <span className="map-coordinates">X {pointerWorld[0].toFixed(2)} · Y {pointerWorld[1].toFixed(2)}</span>}
    {!doc && <div className="canvas-empty"><span className="empty-icon"><Icon name="map" size={30}/></span><h3>地図から、走行の準備を。</h3><p>保存済みの地図を選ぶか、<br/>rosbag から新しい地図を作成してください。</p></div>}
  </div>;
}
