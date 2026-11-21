"""Monza pit-straight details in RC metres. All geometry is visual only.

The checked-in OSM start-light node and home-straight edges locate the grid.
FIA Italian GP competition visa: control line 309 m before the start; FIA pit
lane drawing: pole on the driver's left. Individual paint, repairs, garage and
grandstand construction details are approximations based on the reference view.
"""
from bisect import bisect_right
from math import ceil, cos, dist, floor, pi, sin

from geography import LAP_START_LON_LAT, project
from monza_pit_wall import grid_station
from monza_trackside import _beam, _post
from scenery import _box


# © OpenStreetMap contributors, ODbL 1.0. Pit Lane, way 38168747 v12
# https://www.openstreetmap.org/way/38168747/history/12 (retrieved 2026-11-20).
# The way gives the centre of the paved route, not its outer edge.
PIT_LANE_LON_LAT = (
    (9.2807176, 45.6139645), (9.2807806, 45.6143784), (9.2807878, 45.6144353),
    (9.2807940, 45.6145061), (9.2808031, 45.6146404), (9.2808051, 45.6146639),
    (9.2808057, 45.6146707), (9.2808098, 45.6147039), (9.2808137, 45.6147384),
    (9.2808189, 45.6147757), (9.2808635, 45.6150604), (9.2809221, 45.6153955),
    (9.2809782, 45.6160014), (9.2809959, 45.6161212), (9.2813286, 45.6184074),
    (9.2813702, 45.6186927), (9.2814306, 45.6191387), (9.2814355, 45.6192836),
    (9.2814345, 45.6192983), (9.2814180, 45.6195366), (9.2814231, 45.6202502),
    (9.2814088, 45.6206107),
)

# © OpenStreetMap contributors, ODbL 1.0. Selected nodes of the circular
# Podio (way 219396377 v3), CCW, starting at its elevated footway entrance.
# Way 219396384 v1 ends on the mapped Palazzina Box facade.
PODIUM_LON_LAT = (
    (9.2810617, 45.6164881), (9.2810594, 45.6165074), (9.2810496, 45.6165215),
    (9.2810348, 45.6165322), (9.2810104, 45.6165405), (9.2809841, 45.6165418),
    (9.2809584, 45.6165359), (9.2809383, 45.6165241), (9.2809261, 45.6165083),
    (9.2809227, 45.6164926), (9.2809315, 45.6164695), (9.2809466, 45.6164565),
    (9.2809717, 45.6164467), (9.2810032, 45.616445), (9.2810353, 45.6164547),
    (9.2810498, 45.6164653), (9.2810566, 45.6164739),
)
PODIUM_GARAGE_ENTRANCE = (9.2811574, 45.6164842)

# FIA 2026 Italian GP pit-lane drawing p.3 and F3 competition notes §§12,16
# show the dotted track edge, blue entry island, yellow bollard at SC1, and
# the exit line ending at SC2 by the 200 m boards. Stations are RC metres
# from the starting lights along the home straight. The points where way
# 38168747 (7 m assumed width) splits from/rejoins the GP edge locate the
# dotted lines. The mapped upstream pit-wall endpoint fixes the island tail.
# https://www.fia.com/system/files/decision-document/2026_italian_grand_prix_-_competition_notes_-_circuit_map_pit_lane_drawing_emergency_exits_map_and_red_zone.pdf
# https://www.fia.com/system/files/documents/doc_10_-_2026_monza_event_-_f3_monaco_competition_notes_2026_v2.pdf
PIT_ENTRY_DASHES = (-53.0, -45.35)
PIT_EXIT_DASHES = (7.7, 18.4)
PIT_WALL_TIP = (9.2808511, 45.6153406)  # OSM node 2505270990 on way 243090150 v2
PIT_EXIT_SC2 = 41.2  # 200 m before T1 (~61.2 RC m)
PIT_SPEED_LIMIT_STATION = -38.7  # Between SC1 and control line in the FIA pit plan


class PitStraight:
    """Stations along the grid and pit frontage, spanning the seam in both OSM rings."""

    def __init__(self, edges):
        self.origin = project(*LAP_START_LON_LAT)
        left, right = edges['left'], edges['right']
        a, b, c, d = left[-1], left[0], right[-1], right[0]
        dx, dy = (b[i] - a[i] + d[i] - c[i] for i in range(2))
        length = dist((0, 0), (dx, dy))
        self.forward = (dx / length, dy / length)
        self.left = (-dy / length, dx / length)
        self.sides = []
        for edge in (left, right):
            back = -1
            while self.station(edge[back]) > -56.:
                back -= 1
            points = edge[back:]
            for point in edge:
                points.append(point)
                if self.station(point) > PIT_EXIT_SC2:
                    break
            self.sides.append(([self.station(point) for point in points], points))

    def station(self, point):
        return sum((v - o) * t for v, o, t in zip(point, self.origin, self.forward))

    def sides_at(self, s):
        for stations, points in self.sides:
            i = max(0, min(bisect_right(stations, s) - 1, len(points) - 2))
            a, b = points[i:i+2]
            f = (s - stations[i]) / (stations[i+1] - stations[i])
            yield tuple(x + f * (y - x) for x, y in zip(a, b))

    def ground(self, s, offset, z):
        left, right = self.sides_at(s)
        return (*(0.5 * (x + y) + offset * n for x, y, n in zip(left, right, self.left)), z)

    def right_paint_spans(self, a, b):
        """Painted fractions of a right GP edge, including the gaps at pit accesses."""
        s0, s1 = self.station(a), self.station(b)
        lateral = sum((p-o)*v for p, o, v in zip(a, self.origin, self.left))
        if s1 <= s0 or abs(lateral) > 4.:
            yield 0., 1.
            return
        for start, end in (PIT_ENTRY_DASHES, PIT_EXIT_DASHES):
            low, high = max(start, s0), min(end, s1)
            if high <= low:
                continue
            span = s1 - s0
            if s0 < low:
                yield 0., (low-s0)/span
            # Dash dimensions estimated from the pit entry and exit photographs.
            for i in range(floor((low-start)/.36), ceil((high-start)/.36)):
                lo = max(low, start + i*.36)
                hi = min(high, start + i*.36 + .18)
                if hi > lo:
                    yield (lo-s0)/span, (hi-s0)/span
            if s1 > high:
                yield (high-s0)/span, 1.
            return
        yield 0., 1.


def _speed_limit_signs(straight, faces, ink, red):
    """Two freestanding 80 boards face the approach on opposite pit-lane edges."""
    # Locate the sign pair on the same centreline as the rendered pit road.
    lane = [project(*xy) for xy in PIT_LANE_LON_LAT]
    stations = [straight.station(p) for p in lane]
    i = bisect_right(stations, PIT_SPEED_LIMIT_STATION) - 1
    a, b = lane[i:i+2]
    f = (PIT_SPEED_LIMIT_STATION - stations[i]) / (stations[i+1] - stations[i])
    x, y = (p + f*(q-p) for p, q in zip(a, b))
    dx, dy = ((q-p)/dist(a, b) for p,q in zip(a, b))
    # The two boards in the FIA pit-lane photo stand beside the transverse
    # line at the beginning of the limit. The line spans the 7 m pit road.
    faces.append([(x+u*dy+t*dx, y-u*dx+t*dy, .0007)
                  for u,t in ((.35, -.008), (.35, .008), (-.35, .008), (-.35, -.008))])
    for side in (-1, 1):
        cx, cy = x + side*.405*dy, y - side*.405*dx
        def point(u, z, ahead):
            return cx + u*dy + ahead*dx, cy - u*dx + ahead*dy, z
        faces.extend(_beam(point(-.052, 0, 0)[:2], point(.052, 0, 0)[:2], .008, .001, .108))
        # Faceted ring and digits are visible from the pit-entry approach only.
        for batch, u, z, rx, rz, width in (
                (red, 0., .055, .041, .041, .006),
                (ink, -.012, .064, .008, .009, .0026),
                (ink, -.012, .046, .009, .011, .0026),
                (ink, .011, .055, .009, .019, .0028)):
            def loop(t, r):
                return point(u+(rx-r)*cos(t), z+(rz-r)*sin(t), -.0046)
            for index in range(16):
                t0, t1 = index*pi/8, (index+1)*pi/8
                batch.append([loop(t0,width), loop(t0,0), loop(t1,0), loop(t1,width)])


def _pit_access(straight):
    """Trackside evidence from FIA photographs; widths, tapers and height estimated."""
    white, blue, dark, yellow, red = [], [], [], [], []
    def edge(s, offset=0., z=.0010):
        right = list(straight.sides_at(s))[1]
        return (*(p + n*offset for p, n in zip(right, straight.left)), z)

    def band(first, last, width=.015):
        # Input in driving order. The normal points towards the driver's left.
        dx, dy = last[0]-first[0], last[1]-first[1]
        scale = width/(2*dist(first[:2], last[:2]))
        return [(x+side*dy*scale, y-side*dx*scale, z) for (x,y,z), side
                in ((first,1), (last,1), (last,-1), (first,-1))]

    tip = project(*PIT_WALL_TIP)
    start, stop = PIT_ENTRY_DASHES[1], straight.station(tip)
    nose = edge(start, -.020)
    # Outside the GP road. End the blue at the crash cushion in front of
    # the wall, leaving the pit lane itself clear for a car to pass.
    left, right = edge(stop, -.04), edge(stop, -.53)
    blue.append([(x, y, .0003) for x, y, _ in (nose, right, left)])
    white.extend((band(nose, right), band(nose, left)))

    def cross_line(s, past_edge=0.):
        inside = list(straight.sides_at(s))[0]
        near, far = edge(s, -past_edge, .0014), (*inside, .0014)
        white.append(band(near, far, .014))
    cross_line(start, .70)  # SC1 crosses both the GP road and pit entry.
    cross_line(PIT_EXIT_SC2)

    # The FIA photo shows a second, continuous line on the GP side of the
    # dashed edge; in 2026 it ends at SC2. Blend onto the road from the pit wall.
    exit_offsets = ((2.12, -.62), (6.53, -.21), (10.5, .11), (14.43, .32),
                    (18.4, .34), (PIT_EXIT_SC2, .34))
    points = [edge(s, d) for s,d in exit_offsets]
    white.extend(band(a, b) for a,b in zip(points, points[1:]))

    # Ribbed flexible yellow post on the tip of the island. The dark covered
    # crash cushion behind it is visible in the F1 pit-lane photograph.
    x, y = nose[:2]
    rings = []
    for r, z in ((.017,.001),(.017,.008),(.012,.009),(.012,.035),(.014,.037),
                 (.014,.042),(.012,.044),(.012,.075),(.014,.077),(.014,.082),
                 (.012,.084),(.012,.102),(.010,.107)):
        rings.append([(x+r*cos(j*pi/4), y+r*sin(j*pi/4), z) for j in range(8)])
    for lower, upper in zip(rings, rings[1:]):
        yellow.extend([lower[i],lower[(i+1)%8],upper[(i+1)%8],upper[i]] for i in range(8))
    yellow.append(rings[-1])
    dx, dy = straight.forward
    for d in (-.085, .085):
        x, y = (p+n*d for p,n in zip(tip, straight.left))
        _post(dark, x-.09*dx, y-.09*dy, dx, dy, .166, .25, .001, .155)
    _speed_limit_signs(straight, white, dark, red)
    yield 'pit_access_white', (.85,.86,.82), white
    yield 'pit_access_blue', (.055,.36,.66), blue
    yield 'pit_entry_bollard', (.94,.72,.09), yellow
    yield 'pit_access_dark', (.085,.085,.085), dark
    yield 'pit_speed_limit_red', (.73,.035,.030), red


def _road_details(straight):
    white, yellow, repairs = [], [], []

    def stripe(s0, s1, u0, u1):
        return [straight.ground(s, u, .0008) for s, u in
                ((s0, u0), (s1, u0), (s1, u1), (s0, u1))]

    for s in (-30.9, 0.):
        a, b = straight.sides_at(s)
        half = dist(a, b) / 2 - .024
        white.append(stripe(s - .015, s + .015, -half, half))

    # 22 staggered slots, full-scale front-to-front pitch 8 m. The white
    # line turns back on both sides. Yellow guides extend to the driver's right.
    for slot in range(22):
        s = grid_station(slot + 1)
        u = .30 if slot % 2 == 0 else -.30
        white.append(stripe(s - .009, s, u - .125, u + .125))
        for side in (u - .125, u + .116):
            white.append(stripe(s - .12, s - .009, side, side + .009))
        yellow.append(stripe(s - .029, s - .015, u - .265, u - .125))

    # Shallow irregular-edged repairs between the grid lanes as in the photo.
    # Their positions, lengths and ages are illustrative, not surveyed.
    for s, u, length, width in ((-24.1, .11, .34, .28), (-18.95, -.02, .18, .31),
                                (-15.4, .01, .19, .26), (-12.32, -.04, .24, .31),
                                (-10.0, .03, .16, .23), (-6.35, -.015, .21, .28),
                                (-3.16, .015, .17, .26), (-.94, -.04, .12, .19)):
        repairs.append([straight.ground(s + a * length, u + b * width, .00035)
                        for a, b in ((-.48, -.40), (.32, -.50), (.51, -.33), (.47, .39),
                                     (.34, .50), (-.42, .48), (-.50, .28))])
    yield 'grid_white_paint', (.80, .80, .76), white
    yield 'grid_yellow_guides', (.85, .62, .29), yellow
    yield 'grid_asphalt_repairs', (.77, .77, .77), repairs


def _pit_details(straight, data):
    apron, doors, glass, metal, shades = [], [], [], [], []
    for props, (ring,) in data.polygons('pit_building'):
        # Palazzina Box fronts the garages; the separate Hospitality Building
        # gets continuous glazed storeys. Keep the existing mapped footprint.
        is_garage = props['tags']['name'] == 'Palazzina Box'
        area2 = sum(a[0]*b[1] - a[1]*b[0] for a, b in zip(ring, ring[1:] + ring[:1]))
        for a, b in zip(ring, ring[1:] + ring[:1]):
            if area2 < 0:
                a, b = b, a
            length = dist(a, b)
            outward = ((b[1]-a[1])/length, (a[0]-b[0])/length)
            if length < .8 or sum(v*n for v, n in zip(outward, straight.left)) < .80:
                continue

            # Continuous paving from the garages to the racing surface with
            # the pit wall standing on top. Extend just beneath the GP road at
            # its edge, preserving the existing boundary and its painted line.
            va, vb = [list(straight.sides_at(straight.station(p)))[1] for p in (a, b)]
            apron.append([(*p, -.0016) for p in (b, a, va, vb)])

            def point(f, gap, z):
                return (*(v + (w-v)*f + n*gap for v, w, n in zip(a, b, outward)), z)

            def panel(start, end, gap, bottom, top):
                return [point(f, gap, z) for f, z in
                        ((start, bottom), (end, bottom), (end, top), (start, top))]

            bays = max(1, round(length / (.63 if is_garage else .52)))
            for i in range(bays):
                u, v = i / bays, (i + 1) / bays
                if is_garage:
                    doors.append(panel(u+.035/length, v-.035/length, .003, .015, .38))
                    metal.append(panel(u+.035/length, v-.035/length, .005, .35, .374))
                else:
                    glass.append(panel(u+.014/length, v-.014/length, .003, .22, .60))
                glass.append(panel(u+.014/length, v-.014/length, .003, .72, 1.10))
                metal.append(panel((u+v)/2-.006/length, (u+v)/2+.006/length,
                                   .005, .72, 1.10))
            # Cantilever eaves and a narrower balcony floor: the projecting
            # soffits and edge bands remain visible from the low stereo rig.
            for z, projection in ((.66, .10), (1.25, .18 if is_garage else .27)):
                front = (point(0, projection, 0)[:2], point(1, projection, 0)[:2])
                back = (a, b)
                shades.extend(_box(front, back, 0, 1, 0, 1, z-.034, z))
                metal.append(panel(0, 1, projection+.001, z-.016, z+.002))
    # Include the gap between the two buildings and the pit entry / exit.
    # Assumed width 7 m full scale. Render below the GP surface at the merge.
    lane = [data.project(lon, lat) for lon, lat in PIT_LANE_LON_LAT]
    lane_pairs = []
    for i, xy in enumerate(lane):
        before, after = lane[max(i-1, 0)], lane[min(i+1, len(lane)-1)]
        length = dist(before, after)
        nx, ny = (before[1]-after[1])/length, (after[0]-before[0])/length
        lane_pairs.append([(xy[0]+d*nx, xy[1]+d*ny, -.001) for d in (-.35, .35)])
    yield 'pit_straight_lane', (.38, .37, .36), [
        [a, c, d, b] for (a, b), (c, d) in zip(lane_pairs, lane_pairs[1:])]
    yield 'pit_straight_apron', (.40, .39, .38), apron
    yield 'pit_straight_shutters', (.23, .25, .26), doors
    yield 'pit_straight_glazing', (.16, .25, .29), glass
    yield 'pit_straight_frames', (.63, .65, .64), metal
    yield 'pit_straight_eaves', (.30, .32, .34), shades


def _podium(data):
    """Permanent podium above the pit wall; elevations and rail details are approximate."""
    ring = [data.project(*p) for p in PODIUM_LON_LAT]
    entrance, garage = ring[0], data.project(*PODIUM_GARAGE_ENTRANCE)
    cx, cy = [(min(v) + max(v)) / 2 for v in zip(*ring)]
    # The underside flares from the central column up to the platform rim.
    # Split each ring into outward-facing quads so it is visible from track level.
    concrete = []
    previous = None
    for scale, z in ((.18, 0), (.18, .44), (.95, .55), (1, .56), (1, .61)):
        section = [(cx + scale*(x-cx), cy + scale*(y-cy), z) for x, y in ring]
        if previous:
            for i in range(len(ring)):
                j = (i+1) % len(ring)
                concrete.append([previous[i], previous[j], section[j], section[i]])
        previous = section
    concrete.append(section)

    length = dist(entrance, garage)
    dx, dy = [(b-a)/length for a, b in zip(entrance, garage)]
    # Allow the rectangular walkway to meet the round deck across its width.
    inside = (entrance[0]-.025*dx, entrance[1]-.025*dy)
    concrete.extend(_beam(inside, garage, .19, .557, .609))
    rails = []
    openings = []
    for neighbor in (ring[1], ring[-1]):
        vx, vy = neighbor[0]-entrance[0], neighbor[1]-entrance[1]
        span = -dy*vx + dx*vy
        side = .091 if span > 0 else -.091
        a = (entrance[0] + vx*side/span, entrance[1] + vy*side/span)
        b = (garage[0]-side*dy, garage[1]+side*dx)
        openings.append(a)
        rails.extend(_beam(a, b, .008, .711, .72))
        for t in (0, .5, 1):
            _post(rails, a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]),
                  dx, dy, .009, .009, .609, .715)
    # Leave the bridge entrance open in the perimeter handrail.
    perimeter = [openings[0], *ring[1:], openings[1]]
    for a, b in zip(perimeter, perimeter[1:]):
        rails.extend(_beam(a, b, .009, .711, .72))
    for x, y in ring[1:]:
        _post(rails, x, y, dx, dy, .009, .009, .61, .715)
    yield 'pit_podium_concrete', (.60, .59, .56), concrete
    yield 'pit_podium_handrails', (.35, .38, .39), rails


def _stand_details(data):
    pale, blue, columns, fascia = [], [], [], []
    for props, _ in data.polygons('grandstand'):
        if props['tags'].get('name') not in (
                'Tribuna Centrale', 'Tribuna Laterale Destra', 'Tribuna Laterale Sinistra'):
            continue
        a, b, c, d = (data.project(*p) for p in props['model_corners'])
        if (b[0]-a[0])*(d[1]-a[1]) < (b[1]-a[1])*(d[0]-a[0]):
            a, b, c, d = b, a, d, c
        front, back = (a, b), (d, c)
        length, depth = dist(a, b), (dist(a, d) + dist(b, c)) / 2
        # Match the terraces and canopy height in scenery.grandstand().
        rows, bays = max(3, round(depth/.086)), max(1, round(length/1.7))
        roof = .115 + .035*(rows-1) + .36
        fascia.extend(_box(front, back, 0, 1, -.004, .022, roof-.041, roof+.036))
        for i in range(bays + 1):
            u = .015/length + (1-.03/length)*i/bays
            near, far = u-.014/length, u+.014/length
            columns.extend(_box(front, back, near, far, .115, .115+.030/depth, 0, roof))
            columns.extend(_box(front, back, near, far, .02, .96,
                                roof-.07, roof-.045, rise_v=.04))
            columns.extend(_box(front, back, near, far, .115, .34,
                                roof-.25, roof-.226, rise_v=.18))
        for row in range(rows):
            seats = blue if rows*0.69 <= row < rows*0.80 else pale
            for bay in range(bays):
                # Only the top and near riser of the existing bench are visible
                # here; leave aisles clear and use two material batches per stand.
                t0, t1 = bay/bays+.039/length, (bay+1)/bays-.039/length
                v = (row+.46)/rows
                z = .15 + .035*row
                shell = list(_box(front, back, t0, t1, v-.0005, (row+.9)/rows,
                                  z+.020, z+.034))
                seats.extend((shell[1], shell[2]))
    yield 'pit_stands_pale_seats', (.78, .79, .79), pale
    yield 'pit_stands_blue_seats', (.31, .46, .61), blue
    yield 'pit_stands_roof_frames', (.52, .54, .53), columns
    yield 'pit_stands_roof_fascia', (.83, .84, .83), fascia


def _start_lights(straight):
    """OSM node 1828499259 v4 is tagged as a forward-facing traffic signal.

    Monza's 2025 hospitality brochure (Tower photographs) shows the overhead
    lights. The node fixes the station; the supporting portal, dimensions and
    lamp assembly are indicative. Keep the lamps unlit throughout the lap.
    """
    steel, housings, lenses = [], [], []
    a, b = straight.sides_at(0)
    half_width = dist(a, b)/2
    right, left = -half_width-.70, half_width+.63

    def beam(u0, u1, s0, s1, low, high, rise=0):
        front = [straight.ground(s1, u, 0)[:2] for u in (u0, u1)]
        back = [straight.ground(s0, u, 0)[:2] for u in (u0, u1)]
        return _box(front, back, 0, 1, 0, 1, low, high, rise_u=rise)

    for u in (right, left):
        steel.extend(beam(u-.027, u+.027, -.036, .036, 0, .76))
    for z in (.62, .744):
        steel.extend(beam(right, left, -.028, .028, z, z+.016))
    for i in range(10):
        u0, u1 = (right + (left-right)*j/10 for j in (i, i+1))
        z, rise = (.636, .097) if i % 2 == 0 else (.733, -.097)
        steel.extend(beam(u0, u1, -.013, .013, z, z+.012, rise))

    midpoint = straight.ground(0, 0, 0)
    signal_u = sum((p-m)*n for p, m, n in zip(straight.origin, midpoint, straight.left))
    housings.extend(beam(signal_u-.185, signal_u+.185, -.024, .024, .603, .627))
    for i in range(-2, 3):
        u = signal_u+i*.068
        housings.extend(beam(u-.031, u+.031, -.025, .025, .504, .607))
        for z in (.532, .581):
            # Clockwise as seen along +station: front faces point towards the grid.
            lenses.append([straight.ground(-.026, u+.020*cos(-j*pi/4), z+.020*sin(-j*pi/4))
                           for j in range(8)])
    yield 'start_gantry_steel', (.53, .55, .54), steel
    yield 'start_gantry_housings', (.12, .13, .13), housings
    yield 'start_gantry_unlit_lenses', (.17, .065, .056), lenses


def details(edges, data):
    """Yield named material batches for the Monza GP lap in the existing world frame."""
    straight = PitStraight(edges)
    yield from _road_details(straight)
    yield from _pit_access(straight)
    yield from _pit_details(straight, data)
    yield from _podium(data)
    yield from _stand_details(data)
    yield from _start_lights(straight)
