"""Offline geographical inputs for full-lap Gazebo scenes."""
import argparse
from functools import lru_cache
from hashlib import sha256
import json
from math import ceil, dist, hypot, isfinite
from pathlib import Path

from geography import closed_trajectory

ROOT = Path(__file__).resolve().parent / "reference" / "circuits"


def circuit_ids():
    return ["odaiba"]


def reference_files(circuit):
    if circuit not in circuit_ids():
        raise ValueError(f"Unknown circuit: {circuit}")
    folder = ROOT / circuit
    return folder / "track.json", folder / "scenery.json"


@lru_cache(maxsize=1)
def track_data(circuit):
    track, _ = reference_files(circuit)
    data = json.loads(track.read_bytes())
    if data["circuit_id"] != circuit:
        raise ValueError(f"Mismatched circuit ID in {track}")
    return data


def resolve_route(circuit, route):
    if circuit not in circuit_ids():
        raise ValueError(f"Unknown circuit: {circuit}")
    if route not in (None, "lap"):
        raise ValueError(f"Unknown route: {route}")
    return "lap"


def default_travel_seconds(route, circuit="odaiba"):
    resolve_route(circuit, route)
    data = track_data(circuit)
    return round(data["official_length_m"] * data["horizontal_scale"] / 2.4)


def route_timing(circuit, route, length, travel_seconds, speed_mps):
    if speed_mps is None:
        if travel_seconds is None:
            travel_seconds = default_travel_seconds(route, circuit)
        if not isfinite(travel_seconds) or travel_seconds <= 0:
            raise ValueError("travel_seconds must be positive and finite")
        speed_mps = length / travel_seconds
    else:
        if travel_seconds is not None:
            raise ValueError("choose speed_mps or travel_seconds")
        if not isfinite(speed_mps) or speed_mps <= 0:
            raise ValueError("speed_mps must be positive and finite")
        travel_seconds = length / speed_mps
    if not isfinite(speed_mps) or not isfinite(travel_seconds):
        raise ValueError("requested speed and run duration must be finite")
    return travel_seconds, speed_mps


def load_lap(circuit):
    data = track_data(circuit)
    scale = data["horizontal_scale"]

    def project(x, y):
        return x * scale, y * scale

    course, = [f for f in data["features"] if f["properties"]["kind"] == "course"]
    midline = []
    for point in course["geometry"]["coordinates"]:
        xy = project(*point[:2])
        if not midline or dist(midline[-1], xy) > 1e-7:
            midline.append(xy)
    if len(midline) < 4 or midline[0] != midline[-1]:
        raise ValueError(f"{circuit}: the mapped lap must form a closed line")
    points = closed_trajectory(midline, midline[0])
    width = float(course["properties"]["width_m"]) * scale
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


def metadata(circuit):
    data = track_data(circuit)
    track, surroundings = reference_files(circuit)
    scene_data = json.loads(surroundings.read_bytes())
    course, = [f for f in data["features"] if f["properties"]["kind"] == "course"]
    return {
        "name": data["name"], "axes": data["axes"],
        "horizontal_scale": data["horizontal_scale"], "official_length_m": data["official_length_m"],
        "file": f"reference/circuits/{circuit}/{track.name}", "sha256": sha256(track.read_bytes()).hexdigest(),
        "scenery_file": f"reference/circuits/{circuit}/{surroundings.name}",
        "scenery_sha256": sha256(surroundings.read_bytes()).hexdigest(),
        "attribution": data["attribution"], "license": data["license"], "retrieved": data["retrieved"],
        "scenery_attribution": scene_data["attribution"],
        "track_source": data["source"], "official_reference": data["official_reference"],
        "layout_reference": data["layout_reference"], "layout_note": data["layout_note"],
        "origin_note": data["origin_note"], "direction": data["direction"],
        "road_width_m": course["properties"]["width_m"], "width_source": course["properties"]["width_source"],
        "scenery_sources": scene_data["sources"], "modelled": scene_data["modelled"],
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lap-label", metavar="CIRCUIT", choices=circuit_ids())
    parser.add_argument("--laps", type=int, default=1)
    timing = parser.add_mutually_exclusive_group()
    timing.add_argument("--speed", type=float)
    timing.add_argument("--duration", type=float)
    args = parser.parse_args()
    if args.lap_label is None:
        print("\n".join(circuit_ids()))
    else:
        if args.laps < 1:
            parser.error("--laps は1以上を指定してください")
        points = load_lap(args.lap_label)[1]
        try:
            _, speed = route_timing(args.lap_label, "lap", points[-1][0], args.duration, args.speed)
        except ValueError as exc:
            parser.error(str(exc))
        label = format(speed, ".4g") if args.speed is None else str(speed).removesuffix(".0")
        print(f"{args.laps}{'lap' if args.laps == 1 else 'laps'}_{label.replace('e+', 'e')}mps")
