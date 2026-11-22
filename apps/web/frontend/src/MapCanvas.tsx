import { useCallback, useEffect, useRef, useState } from 'react';
import type { Cloud, EditTarget, LineKind, MapDocument, Point } from './types';
import { Icon } from './ui';
import { cloudFitCorners } from './map_cloud_fit';
import type { ControlOverlay } from './control_view';
import type { TrajectoryOverlay } from './localization_validation';

const COLORS = { left: '#40aa96', right: '#d78d51', custom: '#9c7be2', centerline: '#40aa96', raceline: '#538ee4', customline: '#9c7be2' };
interface Props {
  doc: MapDocument | null; cloud?: Cloud | null; target?: EditTarget; mode?: 'select' | 'add' | 'pan';
  visible?: Record<string, boolean>; onChange?: (target: EditTarget, points: Point[]) => void;
  referenceLine?: LineKind | null; forwardPath?: Point[]; controlTarget?: ControlOverlay;
  overlay?: TrajectoryOverlay; selectedGap?: {index: number} | null; onSelectGap?: (index: number | null) => void;
  pose?: { x?: number; y?: number; yaw?: number } | null; fitKey?: number; autoFit?: boolean; trail?: Point[];
}
export function MapCanvas({ doc, cloud, target = 'left', mode = 'select', visible = {}, onChange, pose, fitKey = 0, autoFit = false, trail = [], overlay, selectedGap, onSelectGap, referenceLine, forwardPath, controlTarget }: Props) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const manualView = useRef(false);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [view, setView] = useState({ x: 0, y: 0, scale: 38 });
  const [selected, setSelected] = useState(-1);
  const [draft, setDraft] = useState<Point[] | null>(null);
  const [pointerWorld, setPointerWorld] = useState<Point | null>(null);
  const drag = useRef<{ kind: 'pan' | 'point' | 'click'; index: number; start: Point; origin: Point; moved: boolean; points: Point[] } | null>(null);
  const project = useCallback((point: readonly number[]): Point => [size.width / 2 + (point[0] - view.x) * view.scale, size.height / 2 - (point[1] - view.y) * view.scale], [size, view]);
  const world = (x: number, y: number): Point => [(x - size.width / 2) / view.scale + view.x, -(y - size.height / 2) / view.scale + view.y];
  const fitPoints = useCallback((points: Point[]) => {
    const { width, height } = size;
    if (!width || !height) return;
    if (!points.length) { setView({ x: 0, y: 0, scale: 38 }); return; }
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const [x, y] of points) {
      x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
    }
    const scale = autoFit
      ? Math.min(width / Math.max(x1 - x0, 1), height / Math.max(y1 - y0, 1)) * .84
      : Math.min(width / Math.max(x1 - x0 + 2, 4), height / Math.max(y1 - y0 + 2, 4)) * .84;
    setView({ x: (x0 + x1) / 2, y: (y0 + y1) / 2, scale });
  }, [size, autoFit]);
  const fit = useCallback(() => {
    const layers: Point[][] = [];
    if (doc) {
      layers.push(doc.left, doc.right);
      if (!autoFit || referenceLine === undefined) {
        layers.push(doc.custom, ...Object.values(doc.lines).map(line => line.points));
      } else if (referenceLine && doc.lines[referenceLine]) {
        layers.push(doc.lines[referenceLine].points);
      }
    }
    const points = layers.flat();
    if (points.length === 0 && cloud) points.push(...cloudFitCorners(cloud));
    if (overlay) {
      for (const segment of overlay.segments) for (const point of segment) points.push(point);
    }
    fitPoints(points);
  }, [doc, cloud, fitPoints, overlay, autoFit, referenceLine]);
  useEffect(() => {
    const node = canvas.current!;
    const observer = new ResizeObserver(entries => { const { width, height } = entries[0].contentRect; setSize({ width, height }); });
    observer.observe(node); return () => observer.disconnect();
  }, []);
  useEffect(() => {
    manualView.current = false;
    setSelected(-1); setDraft(null);
  }, [doc?.id, fitKey, autoFit]);
  // Loading the map can change the canvas dimensions after the first fit.
  // Refit on layout changes until the user chooses their own viewport.
  useEffect(() => {
    if (!autoFit && !manualView.current) fit();
  }, [doc?.id, fitKey, autoFit, size.width, size.height]);
  // 録画用の地図は実際の表示枠に合わせる。走行中の姿勢更新では縮尺を変えない。
  useEffect(() => { if (autoFit) fit(); }, [autoFit, fit, fitKey]);
  useEffect(() => { setSelected(-1); setDraft(null); }, [target]);
  useEffect(() => {
    if (!overlay) return;
    const endpoints = overlay.markers.filter(marker => marker.gapIndex === selectedGap?.index);
    if (endpoints.length) fitPoints(endpoints.map(marker => marker.point));
    else fit();
  }, [overlay, selectedGap, fitPoints, fit]);
  useEffect(() => {
    const node = canvas.current!, ctx = node.getContext('2d')!;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    node.width = size.width * dpr; node.height = size.height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#f1f4f3'; ctx.fillRect(0, 0, size.width, size.height);
    if (!(view.scale > 0) || !Number.isFinite(view.scale)) return;
    const step = view.scale > 75 ? .5 : view.scale > 16 ? 1 : view.scale > 4 ? 5 :
      Math.max(10, 10 ** Math.ceil(Math.log10(12 / view.scale)));
    const lo = world(0, size.height);
    ctx.lineWidth = 1;
    for (let i = 0; i < Math.ceil(size.width / (step * view.scale)); ++i) {
      const x = (Math.ceil(lo[0] / step) + i) * step;
      const px = project([x, 0])[0]; ctx.strokeStyle = Math.abs(x) < step / 100 ? '#c6d2ce' : '#e0e7e3';
      ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, size.height); ctx.stroke();
    }
    for (let i = 0; i < Math.ceil(size.height / (step * view.scale)); ++i) {
      const y = (Math.ceil(lo[1] / step) + i) * step;
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
      if (referenceLine) {
        const line = doc.lines[referenceLine];
        if (line) path(line.points, '#538ee4', 2.5, false, doc.closed);
      } else if (referenceLine === undefined) {
        for (const [kind, line] of Object.entries(doc.lines)) if (visible[kind] !== false)
          path(line.points, COLORS[kind as keyof typeof COLORS] || '#538ee4', 2.5, false, doc.closed);
      }
      if (referenceLine === undefined && visible.custom !== false) path(target === 'custom' && draft ? draft : doc.custom, COLORS.custom, 2, true, doc.closed);
      if (onChange && mode !== 'pan') (draft || doc[target]).forEach((p, i) => {
        const [x, y] = project(p); ctx.beginPath(); ctx.arc(x, y, i === selected ? 6.5 : 4, 0, Math.PI * 2);
        ctx.fillStyle = i === selected ? COLORS[target] : '#fff'; ctx.fill(); ctx.strokeStyle = COLORS[target]; ctx.lineWidth = 2; ctx.stroke();
      });
    }
    if (doc && trail.length) path(trail, '#16835e77', 2, false, false);
    if (overlay && doc) {
      for (const points of overlay.segments) {
        if (points.length === 1) {
          const [x, y] = project(points[0]);
          ctx.fillStyle = '#16835e'; ctx.beginPath(); ctx.arc(x, y, 2, 0, 2 * Math.PI); ctx.fill();
        } else {
          path(points, '#16835e', 2.5);
        }
      }
      // Draw the selected endpoints last so overlapping observations remain inspectable.
      const ordered = selectedGap == null ? overlay.markers : [
        ...overlay.markers.filter(m => m.gapIndex !== selectedGap.index), ...overlay.markers.filter(m => m.gapIndex === selectedGap.index),
      ];
      for (const marker of ordered) {
        const [x, y] = project(marker.point), active = marker.gapIndex === selectedGap?.index;
        ctx.save(); ctx.translate(x,y);
        ctx.lineWidth = active ? 3 : 2; ctx.strokeStyle = marker.side === 'before' ? '#bc5315' : '#673cab';
        ctx.fillStyle = 'white'; ctx.beginPath();
        if (marker.side === 'before') ctx.arc(0, 0, active ? 8 : 6, 0, Math.PI * 2);
        else { ctx.rotate(Math.PI / 4); const r = active ? 4 : 3.5; ctx.rect(-r, -r, 2*r, 2*r); }
        ctx.fill(); ctx.stroke(); ctx.restore();
      }
    }
    if (doc && forwardPath?.length) {
      path(forwardPath, '#ffffff', 6);
      path(forwardPath, '#a04715', 3.5);
      const projection = forwardPath[0];
      if (typeof pose?.x === 'number' && typeof pose?.y === 'number')
        path([[pose.x, pose.y], projection], '#a04715', 1.5, true);
      const [x, y] = project(projection);
      ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fillStyle = '#fff'; ctx.fill(); ctx.strokeStyle = '#a04715'; ctx.lineWidth = 2; ctx.stroke();
    }
    if (doc && controlTarget) {
      path([controlTarget.origin, controlTarget.target], '#5234aa', 2, true);
      const [x, y] = project(controlTarget.target);
      ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2);
      ctx.strokeStyle = 'white'; ctx.lineWidth = 5; ctx.stroke();
      ctx.strokeStyle = '#5234aa'; ctx.lineWidth = 2.5; ctx.stroke();
      ctx.beginPath(); ctx.arc(x, y, 2, 0, Math.PI * 2); ctx.fillStyle = '#5234aa'; ctx.fill();
    }
    if (pose && typeof pose.x === 'number' && typeof pose.y === 'number') {
      const [x, y] = project([pose.x, pose.y]); ctx.save(); ctx.translate(x, y); ctx.rotate(-(pose.yaw || 0));
      ctx.shadowColor = '#2a806755'; ctx.shadowBlur = 12; ctx.fillStyle = '#16835e';
      ctx.beginPath(); ctx.moveTo(13, 0); ctx.lineTo(-9, -8); ctx.lineTo(-5, 0); ctx.lineTo(-9, 8); ctx.closePath(); ctx.fill(); ctx.restore();
    }
    const scaleWidth = Math.max(1, Math.floor(90 / view.scale / step)) * step * view.scale;
    ctx.strokeStyle = '#65796d'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(24, size.height - 31); ctx.lineTo(24 + scaleWidth, size.height - 31); ctx.stroke();
    ctx.fillStyle = '#65796d'; ctx.font = '11px ui-monospace, monospace'; ctx.fillText(`${(scaleWidth / view.scale).toFixed(step < 1 ? 1 : 0)} m`, 24, size.height - 40);
  }, [doc, cloud, view, size, target, mode, selected, draft, visible, pose, project, onChange, trail, overlay, selectedGap, referenceLine, forwardPath, controlTarget]);
  const pointAt = (e: React.PointerEvent | React.WheelEvent): Point => { const rect = canvas.current!.getBoundingClientRect(); return [e.clientX - rect.left, e.clientY - rect.top]; };
  function zoomBy(factor: number) {
    manualView.current = true;
    setView(v => ({ ...v, scale: Math.min(800, Math.max(.5, v.scale * factor)) }));
  }
  return <div className="map-surface">
    <canvas ref={canvas} tabIndex={0} aria-label={onChange ? '地図編集キャンバス。点をクリックして選択、Deleteで削除。右ドラッグで移動、ホイールで拡大縮小。' : '地図閲覧キャンバス。右ドラッグで移動、ホイールで拡大縮小。'} style={{ cursor: mode === 'pan' ? 'grab' : mode === 'add' ? 'crosshair' : 'default' }}
      onContextMenu={e => e.preventDefault()}
      onWheel={e => {
        manualView.current = true;
        const [x, y] = pointAt(e), at = world(x, y);
        const scale = Math.min(800, Math.max(.5, view.scale * Math.exp(-e.deltaY * .001)));
        setView({ scale, x: at[0] - (x - size.width / 2) / scale, y: at[1] + (y - size.height / 2) / scale });
      }}
      onPointerDown={e => {
        canvas.current!.focus(); canvas.current!.setPointerCapture(e.pointerId); const at = pointAt(e);
        let index = -1;
        if (doc && onChange && mode !== 'pan' && e.button === 0) index = doc[target].findIndex(p => { const screen = project(p); return Math.hypot(screen[0] - at[0], screen[1] - at[1]) < 11; });
        setSelected(index); drag.current = { kind: e.button !== 0 || mode === 'pan' ? 'pan' : index >= 0 ? 'point' : 'click', index, start: at, origin: [view.x, view.y], moved: false, points: doc?.[target].map(p => [...p] as Point) || [] };
      }}
      onPointerMove={e => {
        const at = pointAt(e); setPointerWorld(world(...at)); const active = drag.current; if (!active) return;
        active.moved ||= Math.hypot(at[0] - active.start[0], at[1] - active.start[1]) > 3;
        if (active.kind === 'pan') {
          manualView.current = true;
          setView(v => ({ ...v, x: active.origin[0] - (at[0] - active.start[0]) / v.scale, y: active.origin[1] + (at[1] - active.start[1]) / v.scale }));
        }
        if (active.kind === 'point') { active.points[active.index] = world(...at); setDraft([...active.points]); }
      }}
      onPointerUp={e => {
        const active = drag.current; drag.current = null; if (!active) return;
        if (active.kind === 'point' && active.moved) onChange?.(target, active.points);
        else if (active.kind === 'click' && !active.moved && mode === 'add' && doc) { const next = [...doc[target], world(...pointAt(e))]; onChange?.(target, next); setSelected(next.length - 1); }
        else if (overlay && onSelectGap && !active.moved && e.button === 0) {
          const [x, y] = pointAt(e);
          let closest = 12, index = -1;
          for (const marker of overlay.markers) {
            const [mx, my] = project(marker.point), distance = Math.hypot(x - mx, y - my);
            if (distance < closest) { closest = distance; index = marker.gapIndex; }
          }
          if (index >= 0) onSelectGap(index);
        }
        setDraft(null);
      }}
      onPointerCancel={() => { drag.current = null; setDraft(null); }}
      onKeyDown={e => { if ((e.key === 'Delete' || e.key === 'Backspace') && selected >= 0 && doc && onChange) { e.preventDefault(); onChange(target, doc[target].filter((_, i) => i !== selected)); setSelected(-1); } }}
    />
    <div className="map-corner-label"><span className="dot"/> {doc?.frame || 'map'} <span>座標 / m</span></div>
    <div className="canvas-controls">
      <button title="全体を表示" aria-label="全体を表示" onClick={() => { manualView.current = false; onSelectGap?.(null); fit(); }}><Icon name="expand"/></button>
      <button aria-label="拡大" onClick={() => zoomBy(1.4)}>＋</button>
      <button aria-label="縮小" onClick={() => zoomBy(1 / 1.4)}>−</button>
    </div>
    {pointerWorld && <span className="map-coordinates">X {pointerWorld[0].toFixed(2)} · Y {pointerWorld[1].toFixed(2)}</span>}
    {!doc && <div className="canvas-empty"><span className="empty-icon"><Icon name="map" size={30}/></span><h3>地図から、走行の準備を。</h3><p>保存済みの地図を選ぶか、<br/>rosbag から新しい地図を作成してください。</p></div>}
  </div>;
}
