"""Trackside features in the same 1:10 local frame as the mapped GP circuit."""

from collections import defaultdict
from functools import lru_cache
import json
from math import ceil, cos, dist, pi, sin
from pathlib import Path
from random import Random

from geography import project

SCENERY_FILE = Path(__file__).resolve().parent / 'reference' / 'monza_scenery.geojson'
SCENERY_LAP_FILE = SCENERY_FILE.with_name('monza_lap_scenery.geojson')
MONZA_BACKGROUND = SCENERY_FILE.with_name('monza_background.geojson')


class Features:
    """GeoJSON features projected into one circuit's local RC frame."""
    def __init__(self, files, projection):
        self.project = projection
        self.by_kind = defaultdict(list)
        for path in files:
            for feature in json.loads(path.read_bytes())['features']:
                self.by_kind[feature['properties']['kind']].append(feature)

    def polygons(self, kind):
        for feature in self.by_kind[kind]:
            geom = feature['geometry']
            rings = geom['coordinates']
            for polygon in ([rings] if geom['type'] == 'Polygon' else rings):
                yield feature['properties'], [[self.project(*p[:2]) for p in ring[:-1]] for ring in polygon]

    def lines(self, kind):
        for feature in self.by_kind[kind]:
            yield feature['properties'], [self.project(*p[:2]) for p in feature['geometry']['coordinates']]


@lru_cache(maxsize=2)
def monza_features(lap=False):
    files = [SCENERY_FILE, SCENERY_LAP_FILE, MONZA_BACKGROUND] if lap else [SCENERY_FILE]
    return Features(files, project)


def polygons(kind, lap=False):
    return monza_features(lap).polygons(kind)


def flat_triangles(data, kind, z):
    for props, (exterior,) in data.polygons(kind):
        for triangle in props['triangles']:
            yield [(*exterior[i], z) for i in triangle]


def sand_triangles(lap=False):
    return flat_triangles(monza_features(lap), 'sand', -.002)


def _in_ring(x, y, ring):
    inside = False
    ax, ay = ring[-1]
    for bx, by in ring:
        if (ay > y) != (by > y) and x < ax + (bx - ax) * (y - ay) / (by - ay):
            inside = not inside
        ax, ay = bx, by
    return inside


def _inside(x, y, rings):
    return _in_ring(x, y, rings[0]) and not any(_in_ring(x, y, hole) for hole in rings[1:])


def _crosses_ring(a, b, ring):
    """Detect intersections even when both endpoints lie inside a concave boundary."""
    def side(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])

    c = ring[-1]
    for d in ring:
        if side(a, b, c)*side(a, b, d) < 0 and side(c, d, a)*side(c, d, b) < 0:
            return True
        c = d
    return False


def woodland(lap=False, *, data=None):
    """Deterministic stand-ins within mapped woodland. Neither stems nor species were surveyed."""
    aspects = {'oak': 1.14, 'pine': .75, 'poplar': .29}
    by_species = defaultdict(list)
    # Monza's additional forest polygons were already clipped around structures.
    stand_rings = [rings for _, rings in (data or monza_features()).polygons('grandstand')]
    data = data or monza_features(lap)
    for tree in data.by_kind['tree']:
        props = tree['properties']
        rng = Random(props['osm_id'])
        x, y = data.project(*tree['geometry']['coordinates'][:2])
        height = float(props.get('height_m') or rng.uniform(9, 18)) * .1
        species = rng.choice(('oak', 'poplar', 'pine'))
        angle = rng.uniform(0, pi)
        radius = height * aspects[species] / 2
        for a in (angle, angle + pi/2):
            dx, dy = radius*cos(a), radius*sin(a)
            by_species[species].append([(x-dx, y-dy, 0), (x+dx, y+dy, 0),
                                        (x+dx, y+dy, height), (x-dx, y-dy, height)])
    for props, rings in data.polygons('woodland'):
        rng = Random(props['osm_id'])
        xs, ys = zip(*rings[0])
        xmin, ymin = min(xs), min(ys)
        # The lap supplement is already clipped around raceways, stands and
        # buildings; thin out trees beyond the detailed Rettifilo section.
        spacing = props.get('tree_spacing_rc_m', 1.55)
        for iy in range(ceil((max(ys) - ymin) / spacing)):
            for ix in range(ceil((max(xs) - xmin) / spacing)):
                x = xmin + (ix + rng.uniform(.10, .90)) * spacing
                y = ymin + (iy + rng.uniform(.10, .90)) * spacing
                species = rng.choice(('oak', 'oak', 'pine', 'poplar'))
                height = rng.uniform(1.25, 2.35)
                angle = rng.uniform(0, pi)
                if not _inside(x, y, rings):
                    continue
                radius = height * aspects[species] / 2
                directions = [(radius*cos(a), radius*sin(a)) for a in (angle, angle + pi/2)]
                ends = [(x + sign*dx, y + sign*dy) for dx, dy in directions for sign in (-1, 1)]
                if not all(_inside(px, py, rings) for px, py in ends):
                    continue
                if any(_crosses_ring(a, b, ring) for a, b in zip(ends[::2], ends[1::2]) for ring in rings):
                    continue
                if any(_inside(px, py, footprint) for px, py in [(x, y), *ends] for footprint in stand_rings):
                    continue
                for dx, dy in directions:
                    by_species[species].append([(x-dx, y-dy, 0), (x+dx, y+dy, 0),
                                                (x+dx, y+dy, height), (x-dx, y-dy, height)])
    for species, faces in by_species.items():
        yield 'tree_' + species, faces


def _box(front, back, u0, u1, v0, v1, z0, z1, *, rise_u=0, rise_v=0):
    # The four mapped corners may form a slightly skewed quadrilateral.
    bottom = []
    for u, v, rise in ((u0, v0, 0), (u1, v0, rise_u),
                       (u1, v1, rise_u + rise_v), (u0, v1, rise_v)):
        xy = [(1-v)*(a + (b-a)*u) + v*(d + (c-d)*u)
              for a, b, d, c in zip(*front, *back)]
        bottom.append((*xy, z0 + rise))
    top = [(x, y, z + (z1 - z0)) for x, y, z in bottom]
    yield bottom[::-1]
    yield top
    for i in range(4):
        j = (i+1) % 4
        yield [bottom[i], bottom[j], top[j], top[i]]


def grandstand(lap=False, *, data=None):
    """Fit stepped seating to each mapped footprint. Vertical layout is approximate."""
    decks, benches, supports, canopies = [], [], [], []
    data = data or monza_features(lap)
    for props, _ in data.polygons('grandstand'):
        a, b, c, d = (data.project(*p[:2]) for p in props['model_corners'])
        if (b[0]-a[0])*(d[1]-a[1]) < (b[1]-a[1])*(d[0]-a[0]):
            a, b, c, d = b, a, d, c
        front, back = (a, b), (d, c)
        length, depth = dist(a, b), (dist(a, d) + dist(b, c)) / 2
        rows, sections = max(3, round(depth/.086)), max(1, round(length/1.7))
        for row in range(rows):
            v0, v1, top = row/rows, (row+1)/rows, .15 + .035*row
            decks.extend(_box(front, back, 0, 1, v0, v1, top-.035, top))
            # Narrow continuous bench sections leave aisles through the terraces.
            for section in range(sections):
                margin = .038 / length
                benches.extend(_box(front, back, section/sections + margin, (section+1)/sections - margin,
                                    (row+.46)/rows, (row+.9)/rows, top+.021, top+.033))
        # Three rows of posts carry the terraces. In each bay, inclined beams
        # follow the stepped deck, and braces stiffen the rear and the ends.
        half_u, half_v = .013/length, .013/depth
        columns = [half_u + (1 - 2*half_u)*i/sections for i in range(sections+1)]
        frames = [(v, .115 + .035*int(rows*v)) for v in (half_v, .48, 1-half_v)]
        for column, u in enumerate(columns):
            for v, height in frames:
                supports.extend(_box(front, back, u-half_u, u+half_u, v-half_v, v+half_v, 0, height))
            for (v0, h0), (v1, h1) in zip(frames, frames[1:]):
                supports.extend(_box(front, back, u-half_u, u+half_u, v0, v1,
                                     h0-.025, h0, rise_v=h1-h0))
                if column in (0, sections):
                    supports.extend(_box(front, back, u-half_u, u+half_u, v0, v1,
                                         .035, .055, rise_v=h1-.06))
                    supports.extend(_box(front, back, u-half_u, u+half_u, v0, v1,
                                         h0-.025, h0-.005, rise_v=.06-h0))
        v, height = frames[-1]
        for u0, u1 in zip(columns, columns[1:]):
            supports.extend(_box(front, back, u0, u1, v-half_v, v+half_v,
                                 .035, .055, rise_u=height-.06))
            supports.extend(_box(front, back, u0, u1, v-half_v, v+half_v,
                                 height-.025, height-.005, rise_u=.06-height))
        if props.get('covered'):
            roof = height + .36
            for u in columns:
                supports.extend(_box(front, back, u-half_u, u+half_u, v-half_v, v+half_v,
                                     height, roof+.04))
            canopies.extend(_box(front, back, 0, 1, .02, 1, roof, roof+.035, rise_v=.04))
    yield 'stand_decks', (.55, .55, .53), decks
    yield 'stand_benches', (.34, .39, .42), benches
    yield 'stand_steel', (.24, .27, .28), supports
    if canopies:
        yield 'stand_canopies', (.59, .60, .59), canopies


def buildings(data, kind):
    """Extrude a recorded footprint at its tagged height, or an indicative height."""
    walls, roofs = [], []
    for props, (outline,) in data.polygons(kind):
        height = float(props.get('height_m') or (12.5 if kind == 'pit_building' else 6)) * .1
        area2 = sum(a[0]*b[1] - b[0]*a[1] for a, b in zip(outline, outline[1:] + outline[:1]))
        for a, b in zip(outline, outline[1:] + outline[:1]):
            if area2 < 0:
                a, b = b, a
            walls.append([(*a, 0), (*b, 0), (*b, height), (*a, height)])
        for triangle in props['triangles']:
            a, b, c = (outline[i] for i in triangle)
            if (b[0]-a[0])*(c[1]-a[1]) < (b[1]-a[1])*(c[0]-a[0]):
                b, c = c, b
            roofs.append([(*a, height), (*b, height), (*c, height)])
    prefix = 'pit' if kind == 'pit_building' else 'building'
    yield f'{prefix}_walls', (.59, .58, .55), walls
    yield f'{prefix}_roofs', (.40, .42, .44), roofs


def pit_facades(data, road_edge):
    """Give long pit-facing walls garage openings and an upper window band at RC scale."""
    lanes = [p for _, line in data.lines('pit_lane') for p in line]
    reference = lanes or road_edge[::10]
    garages, windows = [], []
    for props, (outline,) in data.polygons('pit_building'):
        height = float(props.get('height_m') or 12.5) * .1
        area2 = sum(a[0]*b[1] - b[0]*a[1] for a, b in zip(outline, outline[1:] + outline[:1]))
        for a, b in zip(outline, outline[1:] + outline[:1]):
            if area2 < 0:
                a, b = b, a
            length = dist(a, b)
            if length < .9:
                continue
            dx, dy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
            mx, my = (a[0]+b[0])/2, (a[1]+b[1])/2
            near = min(reference, key=lambda p: (p[0]-mx)**2 + (p[1]-my)**2)
            if (near[0]-mx)*dy - (near[1]-my)*dx <= 0:
                continue

            def wall_panel(start, end, bottom, top):
                return [(a[0] + dx*s + .0015*dy, a[1] + dy*s - .0015*dx, z)
                        for s, z in ((start, bottom), (end, bottom), (end, top), (start, top))]

            bays = max(1, round(length / .52))
            for i in range(bays):
                start, end = length*i/bays, length*(i+1)/bays
                garages.append(wall_panel(start+.045, end-.045, .009, min(.36, .42*height)))
                windows.append(wall_panel(start+.015, end-.015, height*.55, height*.81))
    yield 'pit_garages', (.25, .27, .28), garages
    yield 'pit_windows', (.18, .27, .32), windows


def observation_towers(data):
    """COTA tower landmark. The OSM point and tagged height anchor an indicative silhouette."""
    core, deck, red_steel = [], [], []
    for feature in data.by_kind['observation_tower']:
        x, y = data.project(*feature['geometry']['coordinates'][:2])
        height = .1 * feature['properties']['height_m']
        front = ((x-.65, y-.45), (x+.65, y-.45))
        back = ((x-.65, y+.45), (x+.65, y+.45))
        core.extend(_box(front, back, .38, .62, .32, .68, 0, .97*height))
        # Design elevation of the deck is 230 ft on a 251 ft tower.
        deck_z = height * 230 / 251
        deck.extend(_box(front, back, 0, 1, 0, 1, deck_z-.09, deck_z))
        for u in (.26, .38, .5, .62, .74):
            red_steel.extend(_box(front, back, u-.018, u+.018, .69, .74, 0, height))
            red_steel.extend(_box(front, back, u-.018, u+.018, .10, .74, height-.15, height-.10,
                                  rise_v=.1))
    yield 'tower_core', (.69, .72, .73), core
    yield 'tower_deck', (.37, .40, .42), deck
    yield 'tower_steel', (.70, .055, .04), red_steel
