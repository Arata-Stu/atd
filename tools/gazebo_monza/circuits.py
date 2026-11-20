"""Offline geographical inputs for full-lap Gazebo scenes."""
from functools import lru_cache
from hashlib import sha256
import json
from math import ceil, dist, hypot, isfinite, remainder, tau
from pathlib import Path

from geography import SCALE, closed_trajectory, local_projection

ROOT = Path(__file__).resolve().parent / "reference" / "circuits"


def circuit_ids():
    return sorted(["monza", *(p.parent.name for p in ROOT.glob("*/track.geojson")
                             if (p.parent / "scenery.geojson").is_file())])


@lru_cache(maxsize=24)
def track_data(circuit):
    if circuit not in circuit_ids() or circuit == "monza":
        raise ValueError(f"No full-lap reference for circuit {circuit!r}")
    data = json.loads((ROOT / circuit / "track.geojson").read_bytes())
    if data["circuit_id"] != circuit:
        raise ValueError(f"Mismatched circuit ID in {circuit}/track.geojson")
    return data


def resolve_route(circuit, route):
    if circuit not in circuit_ids():
        raise ValueError(f"Unknown circuit: {circuit}")
    if route is None:
        return "chicane" if circuit == "monza" else "lap"
    if route != "lap" and not (route == "chicane" and circuit == "monza"):
        raise ValueError("The chicane section is available for Monza only")
    return route


def load_lap(circuit):
    data = track_data(circuit)
    project = local_projection(*data["origin_lon_lat"])
    course, = [f for f in data["features"] if f["properties"]["kind"] == "course"]
    midline = []
    for point in course["geometry"]["coordinates"]:
        xy = project(*point[:2])
        if not midline or dist(midline[-1], xy) > 1e-7:
            midline.append(xy)
    if len(midline) < 4 or midline[0] != midline[-1]:
        raise ValueError(f"{circuit}: the mapped GP lap must form a closed line")
    points = closed_trajectory(midline, midline[0])
    width = float(course["properties"]["width_m"]) * SCALE
    if not isfinite(width) or width <= 0:
        raise ValueError(f"{circuit}: invalid road width")

    edges = {"left": [], "right": []}
    count = len(points) - 1
    # A tangent from one pavement width on each side spans map-vertex kinks.
    # Adjacent inside-edge sections can fold over if offset from instantaneous yaw.
    window = ceil(width * count / points[-1][0])
    for i, (_, x, y, _) in enumerate(points[:-1]):
        before = points[(i - window) % count]
        after = points[(i + window) % count]
        east, north = after[1] - before[1], after[2] - before[2]
        chord = hypot(east, north)
        if chord < 1e-7:
            raise ValueError(f"{circuit}: ambiguous road direction at sample {i}")
        dx, dy = -north * width / (2 * chord), east * width / (2 * chord)
        edges["left"].append((x + dx, y + dy))
        edges["right"].append((x - dx, y - dy))
    return edges, points, project


def estimated_kerbs(edges, points):
    """Approximate inside kerbs where heading changes over a 1.2 RC metre window."""
    count = len(points) - 1
    half_window = max(1, round(.6 * count / points[-1][0]))
    turns = [remainder(points[(i+half_window) % count][3] - points[(i-half_window) % count][3], tau)
             for i in range(count)]
    result = {}
    for side, sign in (("left", 1), ("right", -1)):
        edge = edges[side]
        start = None
        distance = 0.
        spans = []
        for i, turn in enumerate(turns):
            stop = distance + dist(edge[i], edge[(i+1) % count])
            if sign * turn > .16:
                if start is None:
                    start = distance
            elif start is not None:
                if distance-start >= .4:
                    spans.append((start, distance))
                start = None
            distance = stop
        if start is not None and distance-start >= .4:
            spans.append((start, distance))
        result[side] = spans
    return result


def metadata(circuit):
    data = track_data(circuit)
    folder = ROOT / circuit
    track, surroundings = folder / "track.geojson", folder / "scenery.geojson"
    scene_data = json.loads(surroundings.read_bytes())
    course, = [f for f in data["features"] if f["properties"]["kind"] == "course"]
    return {
        "name": data["name"], "origin_lon_lat_deg": data["origin_lon_lat"],
        "axes": ["east", "north"], "horizontal_scale": SCALE, "official_length_m": data["official_length_m"],
        "file": f"reference/circuits/{circuit}/{track.name}", "sha256": sha256(track.read_bytes()).hexdigest(),
        "scenery_file": f"reference/circuits/{circuit}/{surroundings.name}",
        "scenery_sha256": sha256(surroundings.read_bytes()).hexdigest(),
        "attribution": data["attribution"], "license": data["license"], "retrieved": data["retrieved"],
        "scenery_attribution": scene_data["attribution"], "scenery_license": scene_data["license"],
        "track_source": data["source"], "official_reference": data["official_reference"],
        **{key: data[key] for key in ("layout_reference", "layout_note", "origin_note", "direction",
                                      "start_reference_attribution", "start_reference_license")
           if key in data},
        "road_width_m": course["properties"]["width_m"], "width_source": course["properties"]["width_source"],
        "default_pit_lane_width_rc_m": .65,
        "modelled": ["road edges at representative width", "inside kerbs estimated from curvature",
                     "level terrain, no banking", "unmapped ground cover", "stand structure",
                     "unmeasured building heights, pit facades and tower silhouette", "individual tree appearance"],
    }


if __name__ == "__main__":
    print("\n".join(circuit_ids()))
