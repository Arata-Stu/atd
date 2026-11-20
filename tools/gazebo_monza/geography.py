"""Project the Monza OSM reference to an east/north, one-tenth-scale RC world."""

from bisect import bisect_right
from hashlib import sha256
import json
from math import atan2, ceil, cos, dist, pi, radians, sin, sqrt
from pathlib import Path

TRACK_FILE = Path(__file__).resolve().parent / "reference" / "monza-track.geojson"
ORIGIN_LON = 9.2818939
ORIGIN_LAT = 45.6244492
SCALE = 0.1
APPROACH_FULL_M = 120.0
EXIT_FULL_M = 40.0
# OSM GP relation 284565 assigns both "start" and "finish" to node 1828499259.
LAP_START_LON_LAT = (9.2811729, 45.6189632)
# Inferred from the curvature of the inner edge of each turn. These vertex
# indices on the pinned rings anchor the paint near the apex, not a kerb survey.
# T1 nodes 14298513142 / 14298513151; T2 nodes 14298365021 / 14298365028.
_CHICANE_KERB_VERTICES = {"right": (43, 52), "left": (49, 56)}
# Inside of the final right-hand bend, Curva Alboreto. Its extent on the
# mapped right edge (way 1569604642 v1) is an estimate, as for Rettifilo.
_ALBORETO_KERB_VERTICES = (571, 598)


def local_projection(origin_lon, origin_lat):
    """Return WGS84 lon/lat to local east/north in 1:10 RC metres."""
    lat = radians(origin_lat)
    e2 = 6.6943799901413165e-3
    d = 1.0 - e2 * sin(lat)**2
    east = 6378137.0 * cos(lat) / sqrt(d)
    north = 6378137.0 * (1.0 - e2) / d**1.5

    def to_local(lon, lat):
        return (radians(lon - origin_lon) * east * SCALE,
                radians(lat - origin_lat) * north * SCALE)
    return to_local


project = local_projection(ORIGIN_LON, ORIGIN_LAT)


def reference_metadata(route="chicane"):
    """Identify the exact data and coordinate frame used for a recorded run."""
    raw = TRACK_FILE.read_bytes()
    data = json.loads(raw)
    metadata = {
        "file": f"reference/{TRACK_FILE.name}",
        "sha256": sha256(raw).hexdigest(),
        "retrieved": data["retrieved"],
        "attribution": data["attribution"],
        "copyright_url": "https://www.openstreetmap.org/copyright",
        "osm_ways": {f["properties"]["role"]: {
            "id": f["properties"]["osm_way"], "version": f["properties"]["version"],
        } for f in data["features"]},
        "origin_lon_lat_deg": [ORIGIN_LON, ORIGIN_LAT],
        "axes": ["east", "north"],
        "horizontal_scale": SCALE,
    }
    if route == "lap":
        metadata["route_source"] = "Smoothed centre of paired GP boundary cross-sections"
        metadata["start_finish"] = {
            "osm_node": 1828499259, "version": 4, "lon_lat_deg": list(LAP_START_LON_LAT),
            "osm_relation": 284565, "relation_version": 33, "roles": ["start", "finish"],
        }
    elif route == "chicane":
        metadata["approach_full_m"] = APPROACH_FULL_M
        metadata["exit_full_m"] = EXIT_FULL_M
    else:
        raise ValueError(f"Unknown Monza route: {route}")
    return metadata


def _stations(points):
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + dist(a, b))
    return distances


def kerb_spans(edges):
    """RC arc length along the individual mapped edges for the chicane kerbs."""
    spans = {}
    for side, (start, end) in _CHICANE_KERB_VERTICES.items():
        stations = _stations(edges[side])
        spans[side] = (stations[start], stations[end])
    return spans


def kerb_intervals(edges, lap):
    spans = {side: [interval] for side, interval in kerb_spans(edges).items()}
    if lap:
        stations = _stations(edges["right"])
        start, end = _ALBORETO_KERB_VERTICES
        spans["right"].append((stations[start], stations[end]))
    return spans


def _point_at(points, stations, s):
    s = min(max(s, stations[0]), stations[-1])
    i = min(bisect_right(stations, s) - 1, len(points) - 2)
    fraction = (s - stations[i]) / (stations[i + 1] - stations[i])
    a, b = points[i:i + 2]
    return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))


def cross_sections(left, right):
    """Walk across the triangulated pavement, moving one boundary vertex per step."""
    left = left + left[:1]
    right = right + right[:1]
    i = j = 0
    yield left[i], right[j]
    while i + 1 < len(left) or j + 1 < len(right):
        if j + 1 == len(right) or (i + 1 < len(left)
                                   and dist(left[i + 1], right[j]) <= dist(left[i], right[j + 1])):
            i += 1
        else:
            j += 1
        yield left[i], right[j]


def closed_trajectory(midline, start):
    """Sample a closed drive path, smoothing through the timing line as well as every bend."""
    stations = _stations(midline)
    length = stations[-1]

    def smoothed(s):
        # Wrap the 0.4 m wide smoothing kernel through the lap seam.
        a, b, c = (_point_at(midline, stations, (s + ds) % length) for ds in (-.2, 0, .2))
        return ((a[0] + 2*b[0] + c[0]) / 4, (a[1] + 2*b[1] + c[1]) / 4)

    first = smoothed(0)
    # The paired rings begin ~0.16 RC metres from the OSM timing node. Shift
    # the centre path smoothly onto the timing node on this straight only.
    spacing = .10
    count = ceil(length / spacing)
    points = []
    for i in range(count):
        s = length * i / count
        x, y = smoothed(s)
        taper = .5 * (1 + cos(pi * min(1, min(s, length - s) / 3.0)))
        points.append((x + (start[0] - first[0])*taper, y + (start[1] - first[1])*taper))
    stations = _stations(points + points[:1])
    poses = []
    for i, (x, y) in enumerate(points):
        a, b = points[i - 1], points[(i + 1) % count]
        poses.append([stations[i], x, y, atan2(b[1] - a[1], b[0] - a[0])])
    poses.append([stations[-1], *poses[0][1:]])
    return poses


def load_track(route="chicane"):
    """Return full GP left/right edges, then [s,x,y,yaw] poses for the driven section.

    The outline ways run in the race direction, starting together on the main
    straight. The short ride follows the car-specific way through both bends;
    the complete lap follows the midline of the GP pavement.
    """
    features = json.loads(TRACK_FILE.read_text())["features"]
    paths = {f["properties"]["role"]: [project(*p) for p in f["geometry"]["coordinates"]]
             for f in features}
    edges = {role: paths[role][:-1] for role in ("left", "right")}
    if route == "lap":
        midline = [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
                   for a, b in cross_sections(edges["left"], edges["right"])]
        return edges, closed_trajectory(midline, project(*LAP_START_LON_LAT))
    if route != "chicane":
        raise ValueError(f"Unknown Monza route: {route}")
    straight, bend, exit_way = (paths[k] for k in ("straight", "chicane", "exit"))
    if straight[-1] != bend[0] or bend[-1] != exit_way[0]:
        raise ValueError("Monza GP reference ways must share their chicane end nodes")
    path = straight[:-1] + bend + exit_way[1:]
    stations = _stations(path)
    start = stations[len(straight)-1] - APPROACH_FULL_M * SCALE
    end = stations[len(straight)+len(bend)-2] + EXIT_FULL_M * SCALE
    # A short 3-point filter rounds polyline vertices locally for continuous
    # camera headings. The 20 cm kernel refers to the RC model. It does not
    # alter either mapped boundary.
    n = ceil((end - start) / .05)
    poses = []
    for i in range(n + 1):
        s = start + (end - start) * i / n
        a, b, c = (_point_at(path, stations, s + ds) for ds in (-.10, 0, .10))
        poses.append(((a[0] + 2*b[0] + c[0]) / 4, (a[1] + 2*b[1] + c[1]) / 4))
    track_s = _stations(poses)
    trajectory = []
    for i, (x, y) in enumerate(poses):
        before = poses[max(i-1, 0)]
        after = poses[min(i+1, n)]
        trajectory.append([track_s[i], x, y, atan2(after[1]-before[1], after[0]-before[0])])
    return edges, trajectory
