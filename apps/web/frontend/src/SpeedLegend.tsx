import { useMemo, type ReactNode } from 'react';
import { savedSpeeds, speedColor, speedRamp, speedScale } from './raceline_speed';
import type { SpeedScale } from './raceline_speed';
import type { Line } from './types';
import './raceline_speed.css';

export function SpeedLegend({ scale }: { scale: SpeedScale | null }) {
  if (!scale) return null;
  const { min, max } = scale;
  const constant = min === max;
  const ticks = constant ? [min] : [min, min + (max - min) / 2, max];
  return <span className="raceline-speed-key">
    <span>Raceline 目標速度 (m/s)</span>
    <span className="raceline-speed-values">
      <span className="raceline-speed-bar" aria-hidden="true" style={{ background: constant ? speedColor(min, scale) : speedRamp }}/>
      <span className="raceline-speed-ticks">{ticks.map((v, i) => <span key={i}>{constant && '一定 '}{v.toFixed(2)}</span>)}</span>
    </span>
  </span>;
}

export function ReferenceLineLegend({ raceline, children }: { raceline?: Line; children: ReactNode }) {
  const scale = useMemo(() => speedScale(savedSpeeds(raceline), raceline?.points.length ?? 0), [raceline]);
  return scale ? <SpeedLegend scale={scale}/> : <>{children}</>;
}
