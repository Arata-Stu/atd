"""Fixed Monza pit-wall fittings in 1:10 RC metres.

FIA Monza 2026 F3 event notes §§19–20 identify two car extraction gates
beside grid slots 6 and 21 and a 2 m orange section for a vehicle exit.
Geobrugg describes regular pit-board openings and a 4.5 m vehicle gate.
That width is representative; Monza's panel spacing and hardware are estimated.
https://www.fia.com/system/files/documents/doc_10_-_2026_monza_event_-_f3_monaco_competition_notes_2026_v2.pdf
https://www.geobrugg.com/en/Pit-Wall-196996.html
"""

from math import ceil, dist
from monza_trackside import _beam, _post

WALL_WAY = 243090150
GATE_WIDTH = .45
ORANGE_WIDTH = .20


def grid_station(slot):
    """Station of the front paint line for a one-based grid slot."""
    return -.24 - (slot - 1) * .8


def sections(straight, points):
    """Split the surveyed wall where the two closed steel gates replace concrete."""
    centers = [grid_station(slot) for slot in (21, 6)]
    for a, b in zip(points, points[1:]):
        start, end = straight.station(a), straight.station(b)
        if end <= start:
            raise ValueError("Monza pit wall must be ordered in driving direction")
        cuts = [start, *(s for c in centers for s in (c - GATE_WIDTH/2, c + GATE_WIDTH/2)
                         if start < s < end), end]

        def at(s):
            t = (s - start) / (end - start)
            return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))

        for low, high in zip(cuts, cuts[1:]):
            is_gate = any(abs((low + high)/2 - c) < GATE_WIDTH/2 for c in centers)
            yield is_gate, at(low), at(high)


def fittings(straight, sections_with_height):
    """Upper fencing, open signalling windows and closed removal gates on OSM 243090150."""
    mesh, steel, leaves, markings = [], [], [], []
    ends = set()
    for is_gate, a, b, sill in sections_with_height:
        length = dist(a, b)
        dx, dy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
        top = sill + .165
        # Coordinates along the wall and towards the circuit (driver's left).
        def at(s, offset=0.):
            return a[0] + dx*s - dy*offset, a[1] + dy*s + dx*offset

        def face(lo, hi, z0, z1, offset=0.):
            x, y = at(lo, offset)
            u, v = at(hi, offset)
            return [(x, y, z0), (x, y, z1), (u, v, z1), (u, v, z0)]

        def rail(lo, hi, z):
            steel.extend(_beam(at(lo), at(hi), .007, z - .0035, z + .0035))

        def upright(s, z0, z1):
            _post(steel, *at(s), dx, dy, .009, .009, z0, z1)

        if is_gate:
            # Two lower leaves replace the concrete, with a slim seam between leaves.
            # A slightly narrower steel body keeps its outer face behind the wall face.
            mid = length / 2
            for lo, hi in ((.002, mid - .001), (mid + .001, length - .002)):
                leaves.extend(_beam(at(lo), at(hi), .044, .002, sill))
                mesh.append(face(lo, hi, sill, top))
                rail(lo, hi, top)
            upright(mid, sill, top)
            markings.append(face(mid - ORANGE_WIDTH - .004, mid - .004,
                                 sill * .32, sill * .86, .0225))
        else:
            bays = max(1, ceil(length / .42))
            pitch = length / bays
            rail(0., length, top)
            for index in range(bays):
                lo, hi = pitch * index, pitch * (index + 1)
                if index:
                    upright(lo, sill, top)
                # One empty signalling hatch per ~2.4 m RC, cut through the wire.
                # Fix stations along the wall so the pattern continues past a gate.
                center = (lo + hi)/2
                center_s = straight.station(at(center))
                window_s = round(center_s / 2.4) * 2.4
                if pitch > .20 and -pitch/2 <= center_s - window_s < pitch/2:
                    x0, x1 = center - .063, center + .063
                    z0, z1 = sill + .026, sill + .119
                    mesh.extend((face(lo, x0, sill, top), face(x1, hi, sill, top),
                                 face(x0, x1, sill, z0), face(x0, x1, z1, top)))
                    rail(x0, x1, z0)
                    rail(x0, x1, z1)
                    upright(x0, z0, z1)
                    upright(x1, z0, z1)
                else:
                    mesh.append(face(lo, hi, sill, top))
        for s in (0., length):
            key = tuple(round(v, 6) for v in at(s))
            if key not in ends:
                upright(s, sill, top)
                ends.add(key)

    yield 'pit_wall_board_mesh', (1., 1., 1.), mesh
    yield 'pit_wall_board_frames', (.61, .63, .62), steel
    yield 'pit_wall_gate_leaves', (.12, .28, .19), leaves
    yield 'pit_wall_exit_orange', (.98, .36, .055), markings
