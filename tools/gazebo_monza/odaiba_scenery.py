"""Track furniture for the Odaiba SKY TRACK in the local 1:5 camera world."""

from bisect import bisect_right
from collections import defaultdict
import json
from math import ceil, cos, dist, hypot, sin, tau
from pathlib import Path

REFERENCE = Path(__file__).resolve().parent / "reference" / "circuits" / "odaiba" / "scenery.json"
COLORS = {
    "barrier_blue": (.045, .13, .45), "barrier_white": (.83, .84, .82),
    "pit_roof": (.81, .82, .81), "pit_walls": (.64, .66, .66),
    "pit_columns": (.50, .54, .55), "pit_interior": (.21, .24, .24),
    "hall_sign": (.06, .41, .42),
    "pit_bay_paint": (.82, .62, .15), "grid_paint": (.82, .82, .77),
    "terrace_base": (.65, .66, .65), "terrace_deck": (.47, .30, .20),
    "terrace_rails": (.65, .69, .69), "terrace_glass": (.18, .24, .27),
    "terrace_furniture": (.18, .20, .19), "briefing": (.54, .56, .56),
    "tyres": (.075, .078, .079), "boundary_steel": (.41, .44, .45),
    "floodlights": (.76, .76, .72),
    "truss": (.68, .70, .69), "equipment": (.29, .31, .32), "island_rails": (.15, .16, .17),
    "tree_bark": (.28, .23, .19), "leaves": (.24, .36, .20), "shaded_leaves": (.13, .24, .14),
}


def _beam(a, b, width, low, high, *, rise=0.):
    """Rectangular beam along a horizontal span, including both end faces."""
    length = dist(a, b)
    nx, ny = (a[1] - b[1]) / length, (b[0] - a[0]) / length
    half = width / 2
    profile = [(-half, low), (half, low), (half, high), (-half, high)]
    start, end = [[(x+nx*w, y+ny*w, z+dz) for w, z in profile] for (x, y), dz in ((a, 0.), (b, rise))]
    for i in range(4):
        j = (i + 1) % 4
        yield [start[i], start[j], end[j], end[i]]
    yield start[::-1]
    yield end


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


class Frontage:
    """Place building details in full-size metres along/across a plan footprint."""
    def __init__(self, spec, project):
        a, b = spec["front_m"]
        self.length = dist(a, b)
        self.depth = spec["depth_m"]
        self.height = spec["height_m"]
        dx, dy = (b[0]-a[0])/self.length, (b[1]-a[1])/self.length
        self.front = (project(*a), project(*b))
        self.back = tuple(project(x-dy*self.depth, y+dx*self.depth) for x, y in (a, b))
        self.scale = dist(*self.front)/self.length

    def front_point(self, u, z, offset=0.):
        """Point on the track-facing elevation; positive offset faces the circuit."""
        (ax, ay), (bx, by) = self.front
        return (ax+((bx-ax)*u+(by-ay)*offset)/self.length,
                ay+((by-ay)*u-(bx-ax)*offset)/self.length, z*self.scale)

    def box(self, batch, u0, u1, v0, v1, low, high, *, rise=0):
        batch.extend(_box(self.front, self.back, u0/self.length, u1/self.length,
                          v0/self.depth, v1/self.depth, low*self.scale, high*self.scale,
                          rise_u=rise*self.scale))


def _pit(hall, groups):
    """Light shed with pitched roof, open ground-floor bays and recessed partitions."""
    length, depth, height = hall.length, hall.depth, hall.height
    roof, walls = groups["pit_roof"], groups["pit_walls"]
    posts, inside = groups["pit_columns"], groups["pit_interior"]
    # Gable faces the driving area. Keep the shadowed bays physically open,
    # with a second row of columns and a set-back wall, as in the pit photos.
    for u0, u1, z, rise in [(-.3, length/2, height, 1.6), (length/2, length+.3, height+1.6, -1.6)]:
        hall.box(roof, u0, u1, -.35, depth+.3, z, z+.14, rise=rise)
        hall.box(walls, u0, u1, -.08, .08, z-.32, z, rise=rise)
    hall.box(walls, 0, .14, 2.8, depth, 0, height)
    hall.box(walls, length-.15, length, 3.6, depth, 0, height)
    hall.box(walls, 0, length, depth-.20, depth, 0, height)
    hall.box(walls, .39*length, .66*length, 0, .18, 0, height-.28)
    hall.box(inside, .2, length-.2, .40*depth, .40*depth+.18, 0, height)
    for u in (.20, length*.21, length*.40, length*.66, length*.84, length-.20):
        for v in (0., .22*depth):
            hall.box(posts, u-.09, u+.09, v, v+.18, 0, height)
        hall.box(posts, u-.10, u+.10, 0, .40*depth, height-.30, height-.10)
    hall.box(posts, 0, length, .05, .20, height-.45, height-.26)
    walls.append([hall.front_point(0, height-.28), hall.front_point(length, height-.28),
                  hall.front_point(length/2, height+1.60)])
    # Only the venue name from the operator's outdoor photograph is reproduced.
    # Plain strokes keep the sign readable in mono without copying a logo image.
    letters = {
        "C": [[(1,1),(0,1),(0,0),(1,0)]], "I": [[(.5,1),(.5,0)]],
        "T": [[(0,1),(1,1)],[(.5,1),(.5,0)]], "Y": [[(0,1),(.5,.55),(1,1)],[(.5,.55),(.5,0)]],
        "R": [[(0,0),(0,1),(.85,1),(1,.85),(1,.65),(.85,.5),(0,.5)],[(.4,.5),(1,0)]],
        "U": [[(0,1),(0,.16),(.18,0),(.82,0),(1,.16),(1,1)]],
    }
    for word, base in (("CITY", 3.9), ("CIRCUIT", 2.35)):
        for i, letter in enumerate(word):
            # Viewed from the front, the 'u' axis points to the right.
            u = length*.52 + (i-(len(word)-1)/2)*1.02 - .39
            for line in letters[letter]:
                path = [(u+.78*a, base+1.17*b) for a, b in line]
                for (ax, az), (bx, bz) in zip(path, path[1:]):
                    stroke = .075/hypot(bx-ax, bz-az)
                    du, dz = -(bz-az)*stroke, (bx-ax)*stroke
                    corners = ((ax-du,az-dz),(bx-du,bz-dz),(bx+du,bz+dz),(ax+du,az+dz))
                    groups["hall_sign"].append([hall.front_point(x, z, .006) for x, z in corners])


def _terrace(deck, groups):
    """Occupied ground level, timber upper deck, metal railings and an enclosed VIP bay."""
    length, depth, height = deck.length, deck.depth, deck.height
    deck.box(groups["terrace_base"], 0, length, 0, depth, 0, height-.15)
    deck.box(groups["terrace_deck"], -.14, length+.14, -.18, depth+.16, height-.15, height)
    rails = groups["terrace_rails"]
    for v in (-.07, depth):
        deck.box(rails, 0, length, v, v+.07, height+1.01, height+1.08)
        count = ceil(length/.65)
        for i in range(count+1):
            u = length*i/count
            deck.box(rails, u-.022, u+.022, v, v+.045, height, height+1.04)
    for u in (0., length):
        deck.box(rails, u-.035, u+.035, 0, depth, height+1.01, height+1.08)
    for u in (1.1, 5.2, 12.3, 17.4):
        deck.box(groups["terrace_glass"], u, u+1.45, -.016, -.004, .72, 2.10)
    deck.box(groups["terrace_base"], .6, .29*length, .9, depth-.1, height, height+2.25)
    deck.box(groups["terrace_glass"], .85, .29*length-.25, .88, .90, height+.3, height+2.06)
    deck.box(groups["terrace_deck"], .32, .29*length+.22, .68, depth+.05, height+2.25, height+2.39)
    for u in (.49*length, .74*length, .90*length):
        deck.box(groups["terrace_furniture"], u, u+.85, .40*depth, .40*depth+.65, height+.66, height+.72)
        deck.box(groups["terrace_furniture"], u+.38, u+.46, .40*depth+.27, .40*depth+.35, height, height+.66)


def _barriers(edges, project, pit_lane, groups):
    """Interlocking blue/white boards, with two actual openings onto the pit apron."""
    for side, edge in edges.items():
        ring = edge + edge[:1]
        stations = [0.]
        for a, b in zip(ring, ring[1:]):
            stations.append(stations[-1] + dist(a, b))
        length = stations[-1]
        gate_stations = []
        if side == "left":
            for label in ("entry_m", "exit_m"):
                location = project(*pit_lane[label])
                nearest = min(range(len(edge)), key=lambda i: dist(edge[i], location))
                gate_stations.append(stations[nearest])

        def point(s):
            i = min(bisect_right(stations, s)-1, len(edge)-1)
            f = (s-stations[i])/(stations[i+1]-stations[i])
            return tuple(a+(b-a)*f for a, b in zip(ring[i], ring[i+1]))

        count = max(2, 2*round(length/.56))
        for n in range(count):
            lo, hi = n*length/count, (n+1)*length/count
            center = (lo+hi)/2
            if any(min(abs(center-s), length-abs(center-s)) < .44 for s in gate_stations):
                continue
            material = "barrier_blue" if n % 2 else "barrier_white"
            groups[material].extend(_beam(point(lo+.001), point(hi-.001), .050, 0., .10))


def _paint(edges, project, lane, groups):
    bays = groups["pit_bay_paint"]
    columns = lane["bay_columns_m"]
    lo, hi = lane["bay_y_range_m"]
    for column in columns:
        bays.extend(_beam(project(column, lo), project(column, hi), .018, .0002, .0008))
    for row in range(9):
        y = lo + row*(hi-lo)/8
        bays.extend(_beam(project(columns[0], y), project(columns[-1], y), .018, .0002, .0008))

    # Paired edges share their station zero exactly at the timing line.
    a, b = edges["left"][0], edges["right"][0]
    count = max(2, round(dist(a, b)/.095))
    for row in range(2):
        for col in range(row, count, 2):
            y0, y1 = sorted((a[1]+(b[1]-a[1])*col/count, a[1]+(b[1]-a[1])*(col+1)/count))
            x = (a[0]+b[0])/2 + row*.085
            groups["grid_paint"].extend(_beam((x, y0), (x, y1), .085, .0004, .0012))
    # Two staggered columns of L-shaped boxes in the grid upstream of the line.
    for n in range(6):
        for side in (-1, 1):
            x, y = .49 + n*.65 + (side+1)*.12, side*.29
            paint = groups["grid_paint"]
            paint.extend(_beam((x, y), (x+.15, y), .014, .0004, .0012))
            paint.extend(_beam((x, y), (x, y+side*.11), .014, .0004, .0012))


def _turn_equipment(frame, groups):
    """Open truss with diagonal webs, a recessed dark rack and a louvred cabinet."""
    length, depth, height = frame.length, frame.depth, frame.height
    metal = groups["truss"]
    for u in (0., length-.10):
        for v in (0., depth-.10):
            frame.box(metal, u, u+.10, v, v+.10, 0., height)
    a, b = frame.front
    d, c = frame.back
    scale = frame.scale
    for p, q in ((a,b),(b,c),(c,d),(d,a)):
        for z in (height-.35, height-.08):
            metal.extend(_beam(p, q, .08*scale, z*scale, (z+.08)*scale))
        for i in range(6):
            xy0, xy1 = [tuple(x+(y-x)*n/6 for x, y in zip(p,q)) for n in (i,i+1)]
            z0 = height-.31 if i % 2 == 0 else height-.08
            rise = (height-.08 if i % 2 == 0 else height-.31) - z0
            metal.extend(_beam(xy0, xy1, .05*scale, z0*scale, (z0+.05)*scale, rise=rise*scale))
    frame.box(groups["equipment"], .22, 1.32, .45, 1.40, 0, 1.40)
    frame.box(metal, .19, 1.35, .41, 1.44, 1.40, 1.46)
    for z in (.35, .49, .63):
        frame.box(metal, .33, 1.21, .43, .45, z, z+.045)
    frame.box(groups["pit_interior"], 1.76, length-.15, depth-.85, depth-.13, .02, .9)


def _start_trees(data, project, groups):
    """Staggered trunks with forked crowns behind the perimeter fence."""
    scale = dist(project(0,0), project(1,0))
    for i, (px, py, height, radius) in enumerate(data["start_verge_trees_xy_height_radius_m"]):
        x, y = project(px, py)
        h, r = height*scale, radius*scale
        bark = groups["tree_bark"]
        bark.extend(_beam((x-r*.08,y), (x+r*.08,y), r*.16, 0., h*.74))
        for branch, fraction in enumerate((.62, .68, .80)):
            angle = 2.4*(branch+i)
            bx, by = x+.42*r*cos(angle), y+.42*r*sin(angle)
            bark.extend(_beam((x,y), (bx,by), r*.09, h*.38, h*.38+r*.09, rise=(fraction-.38)*h))
            crown = groups["leaves" if branch != 1 else "shaded_leaves"]
            rings = []
            for level, (width, z) in enumerate(((.18,-.18),(.78,-.11),(.92,.01),(.67,.13),(.14,.20))):
                ring = []
                for k in range(9):
                    az = tau*k/9 + angle + level*.16
                    spread = r*width*(1+.15*sin(3*az+level+i))
                    ring.append((bx+spread*cos(az), by+spread*sin(az),
                                 h*(fraction+z+.026*sin(2*az+level))))
                rings.append(ring)
            crown.append(rings[0][::-1])
            for low, high in zip(rings, rings[1:]):
                for k in range(9):
                    j = (k+1) % 9
                    crown.append([low[k], low[j], high[j], high[k]])
            crown.append(rings[-1])


def _perimeter(data, project, groups):
    steel = groups["boundary_steel"]
    for a, b in data["north_gate_fence_m"] + data["start_verge_fence_m"]:
        a, b = project(*a), project(*b)
        for z in (.08, .37):
            steel.extend(_beam(a, b, .012, z, z+.012))
        bays = ceil(dist(a, b)/.50)
        for i in range(bays+1):
            x, y = (start+(end-start)*i/bays for start, end in zip(a, b))
            steel.extend(_beam((x-.01, y), (x+.01, y), .018, 0., .40))
        # Braces at the ends leave the gate opening unobstructed.
        for i in (0, bays-1):
            p, q = [tuple(x+(y-x)*j/bays for x, y in zip(a,b)) for j in (i,i+1)]
            z = .09 if i == 0 else .36
            steel.extend(_beam(p, q, .012, z, z+.012, rise=.27 if i == 0 else -.27))
    for location in data["lighting_masts_m"]:
        x, y = project(*location)
        steel.extend(_beam((x-.018, y), (x+.018, y), .036, 0., 1.5))
        steel.extend(_beam((x-.14, y), (x+.14, y), .020, 1.47, 1.50))
        for dx in (-.10, .10):
            groups["floodlights"].extend(_beam((x+dx-.04, y), (x+dx+.04, y), .051, 1.50, 1.55))
    # Short, low rings inside the barriers; the dark hollow and sidewall are
    # both visible in the circuit operator's outdoor photograph.
    circle = [(cos(i*tau/12), sin(i*tau/12)) for i in range(12)]
    for a, b in data["tyre_rows_m"]:
        a, b = project(*a), project(*b)
        count = max(2, round(dist(a, b)/.14)+1)
        for i in range(count):
            x, y = (start+(end-start)*i/(count-1) for start, end in zip(a, b))
            for p, q in zip(circle, circle[1:] + circle[:1]):
                sector = []
                for radius, z in ((.071, 0.), (.071, .065), (.038, .065), (.038, 0.)):
                    sector.append([(x+radius*u, y+radius*v, z) for u, v in (p, q)])
                for lo, hi in zip(sector, sector[1:]):
                    groups["tyres"].append([*lo, *reversed(hi)])

    # Short open frame at the island tip, as in the operator's track photo.
    rails = groups["island_rails"]
    a, b = (project(*p) for p in data["turn_island_rail_m"])
    for f in (0., .5, 1.):
        x, y = (u+(v-u)*f for u,v in zip(a,b))
        rails.extend(_beam((x-.006,y), (x+.006,y), .014, .00375, .1815))
    for z in (.09225, .174):
        rails.extend(_beam(a, b, .012, z, z+.0075))


def geometry(edges, project):
    """Yield batched outward-facing meshes in RC metres, sharing the track projection."""
    data = json.loads(REFERENCE.read_bytes())
    groups = defaultdict(list)
    facilities = data["facilities"]
    _pit(Frontage(facilities["pit"], project), groups)
    _terrace(Frontage(facilities["terrace"], project), groups)
    room = Frontage(facilities["briefing"], project)
    room.box(groups["briefing"], 0, room.length, 0, room.depth, 0, room.height)
    room.box(groups["pit_roof"], -.15, room.length+.15, -.14, room.depth+.14,
             room.height, room.height+.12)
    _barriers(edges, project, data["pit_lane"], groups)
    _paint(edges, project, data["pit_lane"], groups)
    _perimeter(data, project, groups)
    _turn_equipment(Frontage(data["first_turn_equipment"], project), groups)
    _start_trees(data, project, groups)
    for name, faces in groups.items():
        yield "odaiba_"+name, COLORS[name], faces
