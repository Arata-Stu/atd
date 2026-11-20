"""Monza guardrails, walls, fences and overhead crossings in 1:10 RC metres.

The companion GeoJSON distinguishes OSM ways from estimated roadside sections.
Heights tagged by OSM are preserved. Other height, span and section details are
indicative. All faces with a common material share one visual per lap.
"""

from collections import defaultdict
from math import ceil, cos, dist, pi, sin
from pathlib import Path

from scenery import Features, _in_ring, _inside

REFERENCE = Path(__file__).resolve().parent / 'reference' / 'monza_trackside.geojson'
# Mapped left-hand Armco (ordered in the driving direction) in front of the
# Laterale, Centrale and Piscina stands. The catch fence behind the Armco is
# apparent in the track-level reference; its offset and section are estimates.
SPECTATOR_RAILS = {243090144, 243090146, 243090151, 1569594729, 243090145}
COLORS = {
    'guardrail': (.58, .62, .63), 'posts': (.27, .29, .29),
    'jointed_concrete': (.57, .57, .54), 'weathered_concrete': (.45, .46, .44),
    'pit_wall_green': (.10, .27, .18),
    'historic_wall': (.49, .42, .35), 'fence': (.35, .37, .36),
    'catch_mesh': (1., 1., 1.),
    'bridge_deck': (.48, .49, .47), 'bridge_steel': (.24, .29, .30),
    'bridge_roof': (.54, .57, .58),
}


def _section(a, b, profile):
    """Extrude a counterclockwise lateral/height profile along one span."""
    length = dist(a, b)
    nx, ny = (a[1] - b[1]) / length, (b[0] - a[0]) / length
    rings = [[(x+nx*w, y+ny*w, z) for w, z in profile] for x, y in (a, b)]
    start, end = rings
    for i in range(len(profile)):
        j = (i + 1) % len(profile)
        yield [start[i], start[j], end[j], end[i]]
    # Convex beams have end caps; the thin folded steel is open at the ends.
    if len(profile) == 4:
        yield start[::-1]
        yield end


def _beam(a, b, width, low, high):
    half = width / 2
    return _section(a, b, [(-half, low), (half, low), (half, high), (-half, high)])


def _post(batch, x, y, dx, dy, width, depth, low, high):
    a = (x-dx*depth/2, y-dy*depth/2)
    b = (x+dx*depth/2, y+dy*depth/2)
    batch.extend(_beam(a, b, width, low, high))


def _grandstand_fence(batches, rail):
    """Track-facing catch net behind the left Armco, open at rail ends."""
    columns = []
    for i, (x, y) in enumerate(rail):
        prev, following = rail[max(0, i-1)], rail[min(i+1, len(rail)-1)]
        length = dist(prev, following)
        nx, ny = (prev[1]-following[1])/length, (following[0]-prev[0])/length
        # At full scale the net stands 1.3 m behind the rail. The top
        # cantilevers 0.55 m towards the track while clearing the Armco.
        knee = (x+.13*nx, y+.13*ny)
        tip = (x+.075*nx, y+.075*ny)
        columns.append([(*knee, .035), (*knee, .32), (*tip, .38)])

    next_post = 0.
    for start, end in zip(columns, columns[1:]):
        for j in (0, 1):
            batches['catch_mesh'].append([start[j], end[j], end[j+1], start[j+1]])
        for j in (1, 2):
            a, b = start[j][:2], end[j][:2]
            batches['fence'].extend(_beam(a, b, .005, start[j][2]-.0025, start[j][2]+.0025))

        a, b = start[0][:2], end[0][:2]
        length = dist(a, b)
        dx, dy = (b[0]-a[0])/length, (b[1]-a[1])/length
        while next_post <= length:
            x, y = a[0]+dx*next_post, a[1]+dy*next_post
            _post(batches['posts'], x, y, dx, dy, .014, .014, 0., .32)
            front, back = (x-dx*.007, y-dy*.007), (x+dx*.007, y+dy*.007)
            batches['posts'].extend(_section(front, back, [
                (-.007, .32), (.007, .32), (-.048, .38), (-.062, .38)]))
            next_post += .42
        next_post -= length


def _barrier(batches, kind, points, props):
    tags = props.get('tags', {})
    height = .1 * float(tags.get('height', {'guard_rail': 1.1, 'fence': 2.8, 'wall': 1.15}[kind]))
    pit_wall = props.get('osm_way') == 243090150
    historic = kind == 'wall' and props['placement'] == 'mapped' and not pit_wall
    if historic and 'height' not in tags:
        height = .19
    # Posts and joints continue across vertices of each mapped way.
    station = 0.
    next_post = .055
    post_index = 0
    for a, b in zip(points, points[1:]):
        length = dist(a, b)
        if length < .0002:
            continue
        dx, dy = (b[0] - a[0])/length, (b[1] - a[1])/length

        def at(s):
            return a[0]+dx*s, a[1]+dy*s

        if kind == 'wall':
            # A small recess at each panel joint is visible from either side.
            pitch = .65 if historic else .47
            start = station
            while start < station + length - 1e-8:
                panel = int((start + 1e-8) // pitch)
                stop = min((panel+1)*pitch, station+length)
                joint = min(stop, panel*pitch+.009)
                if joint > start:
                    batches['weathered_concrete'].extend(
                        _beam(at(start-station), at(joint-station), .039, -.002, height-.004))
                start = max(joint, start)
                if stop > start + 1e-8:
                    if pit_wall:
                        tint = 'pit_wall_green'
                    elif historic:
                        tint = 'historic_wall'
                    else:
                        tint = 'weathered_concrete' if panel % 11 in (4, 8) else 'jointed_concrete'
                    batches[tint].extend(_beam(at(start-station), at(stop-station), .048, -.002, height))
                start = stop
        else:
            if kind == 'guard_rail':
                # Folded galvanized rails have alternating light/dark slopes in mono.
                for level in (.43, .82):
                    low, high = height*(level-.16), height*(level+.16)
                    shape = [(-.008, low), (.008, low), (.012, low+.006),
                             (.006, (low+high)/2), (.012, high-.006), (.008, high),
                             (-.008, high), (-.012, high-.006), (-.006, (low+high)/2),
                             (-.012, low+.006)]
                    batches['guardrail'].extend(_section(a, b, shape))
                spacing, post_height = .46, height
            else:
                # Horizontal tension wires suggest the fence without a subpixel
                # alpha grid; posts and open spans leave the trees visible.
                for level in (.26, .64, .98):
                    batches['fence'].extend(_beam(a, b, .005, height*level-.004, height*level+.004))
                spacing, post_height = .67, height + .012
            while next_post <= station + length:
                x, y = at(next_post-station)
                _post(batches['posts'], x, y, dx, dy, .021, .023, -.003, post_height)
                if kind == 'fence' and post_index % 5 == 0 and next_post+.23 < station+length:
                    end = at(next_post-station+.23)
                    brace = [(x, y, height*.76), (*end, .016), (*end, .035), (x, y, height*.76+.019)]
                    batches['posts'].extend([brace, brace[::-1]])
                next_post += spacing
                post_index += 1
        station += length


def _bridge(batches, points, props, source, sand):
    """The old oval and two pedestrian bridges cross on OSM layer +1."""
    a, b = points[0], points[-1]
    length = dist(a, b)
    dx, dy = (b[0]-a[0])/length, (b[1]-a[1])/length
    oval = props['tags']['highway'] == 'raceway'
    covered = props['tags'].get('covered') == 'yes'
    width, clearance = (1., .49) if oval else (.29, .55)

    def at(s, lateral):
        return a[0]+dx*s-dy*lateral, a[1]+dy*s+dx*lateral

    def member(batch, start, end, w, low, high):
        batches[batch].extend(_beam(start, end, w, low, high))

    deck_z = clearance + (.09 if oval else .05)
    member('bridge_deck', a, b, width, clearance, deck_z)
    for sign in (-1, 1):
        lateral = sign*(width/2-.012)
        p, q = at(0, lateral), at(length, lateral)
        if oval:
            member('bridge_deck', p, q, .03, deck_z, deck_z+.07)
            member('bridge_steel', p, q, .035, clearance-.025, clearance)
        else:
            height = .24 if covered else .12
            member('bridge_steel', p, q, .016, deck_z+height-.014, deck_z+height)
            member('bridge_steel', p, q, .009, deck_z+.045, deck_z+.052)
            count = ceil(length/.32)
            for i in range(count+1):
                x, y = at(length*i/count, lateral)
                _post(batches['bridge_steel'], x, y, dx, dy, .013, .017, deck_z, deck_z+height)
    if covered:
        member('bridge_roof', a, b, width+.042, deck_z+.24, deck_z+.258)

    # The supports stand near mapped span ends. Check the whole base against the
    # driven ribbon and gravel. Along-track geometry does not intersect the rig.
    for s in (.045, length-.045):
        for lateral in (-(width/2-.055), width/2-.055):
            corners = [at(s+ds, lateral+dn) for ds in (-.026, .026) for dn in (-.027, .027)]
            if any((_in_ring(x, y, source['left']) != _in_ring(x, y, source['right']))
                   or any(_inside(x, y, rings) for rings in sand) for x, y in corners):
                continue
            x, y = at(s, lateral)
            _post(batches['bridge_deck'], x, y, dx, dy, .052, .05, -.004, clearance)


def _radio_masts(features):
    # OSM provides the sites and mobile-phone use. The tapered shaft and
    # antenna brackets are approximate, 2.5 m tall at the model scale.
    faces = []
    for feature in features.by_kind['radio_mast']:
        x, y = features.project(*feature['geometry']['coordinates'])
        directions = [(cos(i*pi/3), sin(i*pi/3)) for i in range(6)]
        foot = [(x+.040*dx, y+.040*dy, 0) for dx, dy in directions]
        head = [(x+.024*dx, y+.024*dy, 2.5) for dx, dy in directions]
        for i in range(6):
            faces.append([foot[i-1], foot[i], head[i], head[i-1]])
        faces.append(head)
        for dx, dy in directions[::2]:
            panel = (x+.095*dx, y+.095*dy)
            for height in (2.08, 2.37):
                faces.extend(_beam((x, y), panel, .016, height, height+.013))
            _post(faces, *panel, dx, dy, .056, .024, 2.02, 2.43)
    return faces


def geometry(source, data, *, lap=True):
    """Yield named colored meshes. Short-section runs retain their small scenery set."""
    if not lap:
        return
    features = Features([REFERENCE], data.project)
    batches = defaultdict(list)
    for kind in ('guard_rail', 'wall', 'fence'):
        for props, points in features.lines(kind):
            _barrier(batches, kind, points, props)
            if kind == 'guard_rail' and props.get('osm_way') in SPECTATOR_RAILS:
                _grandstand_fence(batches, points)
    sand = [rings for _, rings in data.polygons('sand')]
    for props, points in features.lines('overpass'):
        _bridge(batches, points, props, source, sand)
    yield 'monza_radio_masts', (.59, .61, .62), _radio_masts(features)
    for name, faces in batches.items():
        yield 'monza_' + name, COLORS[name], faces
