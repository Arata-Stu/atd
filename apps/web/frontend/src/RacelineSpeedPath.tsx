import { memo, useId, useMemo } from 'react';
import { speedStrokes } from './raceline_speed';
import type { SpeedScale } from './raceline_speed';
import type { Point } from './types';

export const RacelineSpeedPath = memo(function RacelineSpeedPath({ points, speeds, scale, width = 3 }: {
  points: Point[]; speeds: number[]; scale: SpeedScale; width?: number;
}) {
  const id = useId();
  const strokes = useMemo(() => speedStrokes(points, speeds, scale, true), [points, speeds, scale]);
  return <g fill="none" strokeWidth={width} strokeLinecap="round" strokeLinejoin="round">
    <defs>{strokes.map(({ points: [from, to], stops }, i) => stops &&
      <linearGradient key={i} id={`${id}-${i}`} gradientUnits="userSpaceOnUse" x1={from[0]} y1={from[1]} x2={to[0]} y2={to[1]}>
        {stops.map(stop => <stop key={stop.offset} offset={stop.offset} stopColor={stop.color}/>)}
      </linearGradient>)}</defs>
    {strokes.map((stroke, i) => <polyline key={i} points={stroke.points.join(' ')}
      stroke={stroke.stops ? `url(#${id}-${i})` : stroke.color}/>)}
  </g>;
});
