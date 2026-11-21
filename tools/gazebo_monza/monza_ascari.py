"""Ascari and Alboreto approaches, modelled at 1:10 from OSM and FIA references."""

from collections import defaultdict
from math import cos, dist, sin

from monza_marshal import PANEL_FACE_COLOR, PANEL_FRAME_COLOR, panel
from monza_trackside import COLORS, REFERENCE as BARRIERS, _beam, _grandstand_fence, _post, _rail_splices
from scenery import Features, MONZA_ASCARI as REFERENCE

MATERIALS = {
    'catch_mesh': COLORS['catch_mesh'], 'posts': COLORS['posts'], 'fence': COLORS['fence'],
    'metal': PANEL_FRAME_COLOR, 'panel_face': PANEL_FACE_COLOR,
    'pale': (.82, .83, .81), 'black': (.035, .04, .048), 'blue': (.055, .14, .34),
    'marshal_base': (.58, .56, .53), 'marshal_orange': (.81, .26, .055),
    'lap_steel': COLORS['lap_steel'], 'bolt_heads': (.35, .38, .39),
}


def _stairs(batch, bottom, top, steps):
    # All four mapped flights join the existing 0.60 m footbridge decks. The
    # covered bridge has tagged metal treads and step counts. Other profiles,
    # counts, heights and handrail dimensions are indicative at RC scale.
    length = dist(bottom, top)
    dx, dy = ((b - a) / length for a, b in zip(bottom, top))

    def at(fraction, offset=0.):
        along = fraction * length
        return bottom[0] + along * dx - offset * dy, bottom[1] + along * dy + offset * dx

    for i in range(steps):
        z = .60 * (i + 1) / steps
        batch.extend(_beam(at(i / steps), at((i + 1) / steps), .27, z - .008, z))
    section = ((-.005, 0), (.005, 0), (.005, .009), (-.005, .009))
    for side in (-.135, .135):
        x0, y0 = at(0, side)
        x1, y1 = at(1, side)
        for lift in (0., .11):
            # Sloping stringer and handrail share the flight's longitudinal slope.
            a = [(x0 + dy * w, y0 - dx * w, lift + h) for w, h in section]
            b = [(x1 + dy * w, y1 - dx * w, .60 + lift + h) for w, h in section]
            for j in range(len(section)):
                following = (j + 1) % len(section)
                batch.append([a[j], b[j], b[following], a[following]])
        for t in (0., .33, .66, 1.):
            x, y = at(t, side)
            _post(batch, x, y, dx, dy, .009, .009, .60 * t, .60 * t + .11)


def _marshal_station(batches, center, direction, side):
    # FIA specifies the location of each flag post. Approximate a protected bay
    # with a low front barrier and an open flagging window facing the track.
    x, y = center
    dx, dy = direction

    def at(along, offset):
        return x + along * dx - side * offset * dy, y + along * dy + side * offset * dx

    base = batches['marshal_base']
    base.extend(_beam(at(-.13, -.07), at(.13, -.07), .030, 0., .115))
    for end in (-.118, .118):
        base.extend(_beam(at(end, -.064), at(end, .078), .024, 0., .115))
        for offset in (-.061, .069):
            _post(batches['metal'], *at(end, offset), dx, dy, .010, .010, .11, .255)
    batches['metal'].extend(_beam(at(-.138, .003), at(.138, .003), .171, .251, .261))
    batches['marshal_orange'].extend(_beam(at(-.097, -.087), at(.097, -.087), .008, .044, .103))


# Stroke paths for the three numeral shapes on physical brake-distance boards.
# Coordinates are normalized to a 1x1 glyph. A 0 is a closed ring with a hole.
_DIGITS = {
    '1': [(.16, .72), (.55, .97), (.55, .04)],
    '5': [(.92, .95), (.12, .95), (.09, .55), (.73, .55),
          (.91, .41), (.87, .18), (.72, .05), (.15, .05)],
    '0': [(.24, .95), (.71, .95), (.88, .77), (.88, .22), (.71, .05),
          (.24, .05), (.08, .22), (.08, .77), (.24, .95)],
}


def _brake_board(batches, center, direction, label):
    x, y = center
    dx, dy = direction

    def at(u, ahead=0.):
        # u goes left-to-right in the image seen by an approaching driver.
        return x + u * dy + ahead * dx, y - u * dx + ahead * dy

    ink, ground = ('pale', 'blue') if label == '100' else ('black', 'pale')
    batches[ground].extend(_beam(at(-.081), at(.081), .011, .005, .079))
    glyph_w, glyph_h, advance = .032, .052, .042
    for idx, char in enumerate(label):
        left = (idx - (len(label) - 1) / 2) * advance - glyph_w / 2
        strokes = [(left + u * glyph_w, .016 + v * glyph_h) for u, v in _DIGITS[char]]
        for (u0, z0), (u1, z1) in zip(strokes, strokes[1:]):
            span = dist((u0, z0), (u1, z1))
            nu, nz = -.0034 * (z1 - z0) / span, .0034 * (u1 - u0) / span
            corners = ((u0 - nu, z0 - nz), (u1 - nu, z1 - nz),
                       (u1 + nu, z1 + nz), (u0 + nu, z0 + nz))
            batches[ink].append([(*at(u, -.0058), z) for u, z in corners])


def details(source, data):
    """Yield named, static mesh batches for the full Monza lap."""
    barriers = Features([BARRIERS], data.project)
    batches = defaultdict(list)
    # FIA 2026 section 27 identifies new fences on the left from T10 to T11.
    # Reuse the segmented left-hand rail alignment, preserving its access gaps.
    for props, points in barriers.lines('guard_rail'):
        extent = props.get('sector_rc_m')
        if extent in ([395, 484], [395, 482]):
            _rail_splices(batches, points, props['side'])
        if props.get('side') == 'left' and extent == [395, 484]:
            _grandstand_fence(batches, points, setback=.025)
    # The FIA photograph approaching T11 shows an upright fence on the
    # right too. Fill the existing posts and cables on the two modelled runs.
    for props, points in barriers.lines('fence'):
        if props.get('side') == 'right' and props.get('sector_rc_m') in ([462, 476], [480, 485]):
            for a, b in zip(points, points[1:]):
                batches['catch_mesh'].append([(*a, .028), (*b, .028), (*b, .28), (*a, .28)])
    for props, (bottom, top) in data.lines('steps'):
        count = int(props['tags'].get('step_count', round(dist(bottom, top) / .025)))
        _stairs(batches['metal'], bottom, top, count)
    for kind in ('panel', 'marshal', 'distance_board'):
        for feature in data.by_kind[kind]:
            props = feature['properties']
            center = data.project(*feature['geometry']['coordinates'])
            direction = cos(props['heading_rad']), sin(props['heading_rad'])
            if kind == 'panel':
                frame, face = panel(center, direction)
                batches['metal'].extend(frame)
                batches['panel_face'].extend(face)
            elif kind == 'marshal':
                side = 1 if props['side'] == 'left' else -1
                _marshal_station(batches, center, direction, side)
            else:
                _brake_board(batches, center, direction, props['label'])
    for name, faces in batches.items():
        yield 'ascari_' + name, MATERIALS[name], faces
