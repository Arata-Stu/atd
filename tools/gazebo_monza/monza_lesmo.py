"""Lesmo run-off barriers and marshal lights on the Monza GP lap."""

from math import ceil, cos, dist, pi, sin

from monza_marshal import PANEL_FACE_COLOR, PANEL_FRAME_COLOR, panel
from monza_trackside import _beam


# OSM ways in monza_lap_scenery.geojson. These consecutive vertices trace
# the back of each gravel bed clockwise, with gravel to the right. FIA's
# 2017 Italian GP preview documents tyres and conveyor belts at T6 and T7.
# The actual barrier centreline, lengths and sections were not surveyed.
GRAVEL_BACKS = {231025694: (1, 7), 231025696: (3, 9)}

# The 2025 FIA Monza circuit diagram puts light 8 outside on approach to T6,
# 9 outside on approach to T7 and 10 inside after T7. Anchor the estimated
# positions to vertices on the project's pinned OSM GP boundaries.
# Values are (light number, edge, vertex, outward clearance in RC metres).
MARSHAL_LIGHTS = ((8, 'left', 160, 1.32), (9, 'left', 203, .86),
                  (10, 'right', 317, 1.0))


def _tyre_bank(path, belts, tyres):
    # The track-facing belt obscures most of the stack. Only emit the upper
    # tyre shoulders and the recess in each top tyre. Batch the whole sector.
    directions = [(cos(i*pi/3), sin(i*pi/3)) for i in range(6)]
    for a, b in zip(path, path[1:]):
        length = dist(a, b)
        dx, dy = (b[0]-a[0])/length, (b[1]-a[1])/length

        def at(s, outward):
            return a[0]+dx*s-dy*outward, a[1]+dy*s+dx*outward

        # Belts meet in lengths spanning several tyre stacks. Their end
        # faces and the narrow exposed lips help indicate the turn radius.
        panels = ceil(length/.48)
        for i in range(panels):
            start, end = i*length/panels+.002, (i+1)*length/panels-.002
            belts.extend(_beam(at(start, .048), at(end, .048), .055, .002, .097))
        count = max(1, round(length/.073))
        radius = min(.037, length/(2*count))
        for i in range(count):
            x, y = at((i+.5)*length/count, .065)
            outer = [(x+rx*radius, y+ry*radius, .098) for rx, ry in directions]
            inner = [(x+rx*radius*.46, y+ry*radius*.46, .105) for rx, ry in directions]
            for j in range(6):
                tyres.append([outer[j-1], outer[j], inner[j], inner[j-1]])


def details(source, data):
    """Yield visual-only equipment outside the two right-handers."""
    belts, tyres = [], []
    for props, (ring,) in data.polygons('sand'):
        indices = GRAVEL_BACKS.get(props['osm_id'])
        if indices:
            start, end = indices
            _tyre_bank(ring[start:end+1], belts, tyres)
    yield 'lesmo_runoff_belts', (.20, .23, .205), belts
    yield 'lesmo_tyre_shoulders', (.095, .10, .095), tyres

    frames, screens = [], []
    for _, side, index, clearance in MARSHAL_LIGHTS:
        edge = source[side]
        before, after = edge[index-1], edge[index+1]
        length = dist(before, after)
        dx, dy = (after[0]-before[0])/length, (after[1]-before[1])/length
        signed = clearance if side == 'left' else -clearance
        xy = (edge[index][0]-signed*dy, edge[index][1]+signed*dx)
        frame, screen = panel(xy, (dx, dy))
        frames.extend(frame)
        screens.extend(screen)
    yield 'lesmo_marshal_frames', PANEL_FRAME_COLOR, frames
    yield 'lesmo_marshal_screens', PANEL_FACE_COLOR, screens
