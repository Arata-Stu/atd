"""Generate a scaled circuit and its stereo camera route from local map data."""

import argparse
from hashlib import sha256
import json
from itertools import chain
from math import ceil, cos, degrees, dist, hypot, isfinite, lcm, radians, tan
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from geography import ascari_entry_kerb, cross_sections, kerb_intervals, load_track, reference_metadata
import circuits
from circuits import default_travel_seconds
import monza_ascari
import monza_lesmo
import monza_start_grid
import monza_trackside
import odaiba_scenery
import scenery
from rig_imu import GRAVITY_M_S2
from sensor_profile import CAMERA_RATES, DEFAULT_CAMERA_HZ, IMU_FRAME, IMU_MODES, IMU_TOPIC, RESOLUTIONS, imu_streams

HERE = Path(__file__).resolve().parent
BASELINE_M = 0.095
HFOV = radians(87)
FORWARD_M = 0.18
OPTICAL_FRAMES = {"left": "camera_infra1_optical_frame", "right": "camera_infra2_optical_frame"}
KERB_BLOCK_M = .20
# Full widths in RC metres. The OSM ways provide only the centre of these pavements.
ALTERNATE_WIDTH_M = {"bypass_centerline": .50, "motorcycle_centerline": .72}
LAP_ALTERNATE_WIDTH_M = {**ALTERNATE_WIDTH_M, "junior_merge_centerline": .70}
CAMERA_FAR_M = 45
MONZA_LAP_FAR_M = 100
# The two Lesmo traps and three Ascari traps in the Monza lap OSM snapshot.
LESMO_ASCARI_GRAVEL = {231025694, 231025696, 231026023, 231026026, 231026028}


def field(parent, name, value):
    element = ET.SubElement(parent, name)
    element.text = str(value)
    return element


def vector(*values):
    return " ".join(f"{value:.9g}" for value in values)


def model(world, name, fixed=True):
    element = ET.SubElement(world, "model", name=name)
    if fixed:
        field(element, "static", "true")
    return element, ET.SubElement(element, "link", name="link")


def surface(link, output, name, polygons, rgb, texture_tile_m=None, *, face_up=True, cutout=False, texture_name=None):
    """Batch static polygons. Ground textures tile in metres; tree quads use the full image."""
    polygons = iter(polygons)
    first = next(polygons, None)
    if first is None:
        return
    meshfile = output / f"{name}.obj"
    lines = ["# metres, metre-based planar UVs"]
    vertex = 1
    for face_id, corners in enumerate(chain([first], polygons), start=1):
        # Ground and kerbs face up; vertical scenery retains its source winding.
        a, b, c = corners[:3]
        dx, dy, dz = (b[k] - a[k] for k in range(3))
        ex, ey, ez = (c[k] - a[k] for k in range(3))
        normal = (dy*ez-dz*ey, dz*ex-dx*ez, dx*ey-dy*ex)
        if face_up and normal[2] < 0:
            corners = list(reversed(corners))
            normal = tuple(-n for n in normal)
        magnitude = hypot(*normal)
        # Project vertical walls in metres along their face and height.
        # XY alone gives both top and bottom the same texture coordinate.
        nx, ny, nz = normal
        horizontal = hypot(nx, ny)
        lines.append("vn " + vector(*(n / magnitude if n else 0 for n in normal)))
        for corner, (x, y, z) in enumerate(corners):
            lines.append("v " + vector(x, y, z))
            if texture_tile_m:
                u, v = ((-ny*x + nx*y) / horizontal, z) if horizontal > abs(nz) else (x, y)
                lines.append("vt " + vector(u / texture_tile_m, v / texture_tile_m))
            elif cutout:
                u, v = ((0, 0), (1, 0), (1, 1), (0, 1))[corner]
                lines.append(f"vt {u} {v}")
        refs = [f"{v}/{v if texture_tile_m or cutout else ''}/{face_id}"
                for v in range(vertex, vertex + len(corners))]
        for i in range(1, len(refs)-1):
            lines.append(f"f {refs[0]} {refs[i]} {refs[i+1]}")
        vertex += len(corners)
    meshfile.write_text("\n".join(lines) + "\n")
    visual = ET.SubElement(link, "visual", name=name)
    mesh = ET.SubElement(ET.SubElement(visual, "geometry"), "mesh")
    field(mesh, "uri", str(meshfile))
    material = ET.SubElement(visual, "material")
    field(material, "ambient", vector(*rgb, 1))
    field(material, "diffuse", vector(*rgb, 1))
    if texture_tile_m or cutout:
        if cutout:
            field(material, "double_sided", "true")
        target = output / (texture_name or (name + (".png" if cutout else ".jpg")))
        shutil.copyfile(HERE / "materials" / target.name, target)
        metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
        field(metal, "metalness", 0)
        field(metal, "roughness", .96)
        field(metal, "albedo_map", str(target))


def pavement(left, right):
    """Triangulate the ribbon between two independently simplified closed edges."""
    sections = iter(cross_sections(left, right))
    a, b = next(sections)
    for c, d in sections:
        yield [(x, y, 0) for x, y in (a, b, c if a != c else d)]
        a, b = c, d


def offset_normals(points, side, closed=True):
    """Outward normals with a miter shared by the two paint segments at a corner."""
    normals = []
    sign = 1 if side == "left" else -1
    for a, b in zip(points, points[1:] + (points[:1] if closed else [])):
        length = dist(a, b)
        normals.append((sign * (a[1]-b[1])/length, sign * (b[0]-a[0])/length))
    if not closed:
        yield normals[0]
    previous = normals[-1:] + normals[:-1] if closed else normals[:-1]
    following = normals if closed else normals[1:]
    for (ax, ay), (bx, by) in zip(previous, following):
        scale = 1 + ax*bx + ay*by
        yield ((ax+bx)/scale, (ay+by)/scale)

    if not closed:
        yield normals[-1]


def alternate_pavement(points, width):
    sides = []
    for (x, y), (nx, ny) in zip(points, offset_normals(points, "left", closed=False)):
        dx, dy = nx * width / 2, ny * width / 2
        sides.append(((x + dx, y + dy, -.001), (x - dx, y - dy, -.001)))
    for (a, b), (c, d) in zip(sides, sides[1:]):
        yield [a, b, d, c]


def road_ribbon(edges):
    """Join left/right edges sampled at the same centreline stations."""
    left, right = edges["left"], edges["right"]
    for i, (a, b) in enumerate(zip(left, right)):
        j = (i + 1) % len(left)
        yield [(*a, 0), (*b, 0), (*right[j], 0), (*left[j], 0)]


def ground_extent(source, lap, margin):
    if lap:
        xs, ys = zip(*(source["left"] + source["right"]))
        west, east = min(xs)-margin, max(xs)+margin
        south, north = min(ys)-margin, max(ys)+margin
    else:
        west, east, south, north = -65, 65, -65, 65
    return [(x, y, -.004) for x, y in ((west, south), (east, south), (east, north), (west, north))]


def odaiba_track(world, output, source, project, far):
    _, link = model(world, "circuit")
    # The facility is paved across the infield and the open pit bays.
    ground = ground_extent(source, True, far / cos(HFOV/2) + FORWARD_M + BASELINE_M/2)
    surface(link, output, "apron", [ground], (.8, .8, .8), 2.4, texture_name="monza_asphalt.jpg")
    surface(link, output, "floor", road_ribbon(source), (1, 1, 1), 2.4, texture_name="monza_asphalt.jpg")
    for name, color, faces in odaiba_scenery.geometry(source, project):
        surface(link, output, name, faces, color, face_up=False)


def track(world, output, source, lap, circuit, data, kerb_intervals, far):
    _, link = model(world, "circuit")
    # At an oblique heading, the corner of the front clipping plane extends farther than its centre.
    margin = far / cos(HFOV/2) + FORWARD_M + BASELINE_M/2 if circuit == "monza" else far
    grass = ground_extent(source, lap, margin)
    if circuit in ("sakhir", "yasmarina"):
        surface(link, output, "dry_ground", [grass], (.62, .53, .38))
    else:
        surface(link, output, "grass", [grass], (1, 1, 1), 1.0)
    for kind, color, z in (("grass", (.36, .51, .14), -.0035), ("water", (.11, .32, .40), -.003),
                           ("sand", (.61, .53, .39), -.002)):
        polygons = data.polygons(kind)
        if kind == "sand" and circuit == "monza" and lap:
            gravel, other = [], []
            for item in polygons:
                target = gravel if item[0]['osm_id'] in LESMO_ASCARI_GRAVEL else other
                target.append(item)
            surface(link, output, "lesmo_ascari_gravel", scenery.polygon_triangles(gravel, z),
                    (1, 1, 1), .20, texture_name="runoff_gravel.jpg")
            polygons = other
        surface(link, output, "mapped_" + kind if kind == "grass" else kind,
                scenery.polygon_triangles(polygons, z), color)
    if circuit == "monza" and lap:
        secondary_widths = LAP_ALTERNATE_WIDTH_M
        asphalt_texture, asphalt_span = "monza_asphalt.jpg", 3.2
    else:
        secondary_widths = ALTERNATE_WIDTH_M if circuit == "monza" else {"pit_lane": .65}
        asphalt_texture, asphalt_span = "floor.jpg", .8
    for kind, width in secondary_widths.items():
        faces = [face for props, points in data.lines(kind)
                 for face in alternate_pavement(points, props.get("width_m", width * 10) * .1)]
        surface(link, output, kind, faces, (1, 1, 1), asphalt_span, texture_name=asphalt_texture)
    left, right = source["left"], source["right"]
    if circuit == "monza":
        road = pavement(left, right)
    else:
        road = road_ribbon(source)
    surface(link, output, "floor", road, (1, 1, 1), asphalt_span, texture_name=asphalt_texture)
    paint = []
    pit_straight = monza_start_grid.PitStraight(source) if circuit == "monza" and lap else None
    kerbs = [[], []]
    recessed_faces = []
    for side, spans in kerb_intervals.items():
        span_index = 0
        points = source[side]
        recessed_span = ascari_entry_kerb(points) if circuit == "monza" and lap and side == "right" else None
        normals = list(offset_normals(points, side))
        distance = 0.
        for i, a in enumerate(points):
            j = (i+1) % len(points)
            b = points[j]
            length = dist(a, b)
            end = distance + length
            nx, ny = normals[i]
            mx, my = normals[j]

            def border(s, offset, z):
                t = (s - distance) / length
                x = (a[0] + offset*nx)*(1-t) + (b[0] + offset*mx)*t
                y = (a[1] + offset*ny)*(1-t) + (b[1] + offset*my)*t
                return x, y, z

            paint_spans = pit_straight.right_paint_spans(a, b) if pit_straight and side == "right" else [(0., 1.)]
            for f0, f1 in paint_spans:
                lo, hi = distance + f0*length, distance + f1*length
                paint.append([border(lo, -.022, .0007), border(hi, -.022, .0007),
                              border(hi, -.002, .0007), border(lo, -.002, .0007)])
            if recessed_span:
                start = max(distance, recessed_span[0])
                stop = min(end, recessed_span[1])
                if stop > start:
                    # A shallow concrete channel beside the pavement. The low
                    # point stays above the mapped grass at z=-0.0035.
                    for (u, h), (v, g) in (((0, 0), (.065, -.003)), ((.065, -.003), (.10, -.001))):
                        recessed_faces.append([border(start, u, h), border(stop, u, h),
                                               border(stop, v, g), border(start, v, g)])
            # Divide the kerb at fixed arc-length stations; the stripe pattern
            # continues through each vertex of the mapped edge, even on a tight bend.
            while span_index < len(spans) and spans[span_index][1] <= distance:
                span_index += 1
            for index in range(span_index, len(spans)):
                kerb_start, kerb_end = spans[index]
                if kerb_start >= end:
                    break
                first = max(0, int((distance - kerb_start) // KERB_BLOCK_M))
                last = ceil((min(end, kerb_end) - kerb_start) / KERB_BLOCK_M)
                for block in range(first, last):
                    start = max(distance, kerb_start + block*KERB_BLOCK_M)
                    stop = min(end, kerb_end, kerb_start + (block+1)*KERB_BLOCK_M)
                    if stop - start < 1e-8:
                        continue
                    # Shallow crown falls back into the grass on its outer edge.
                    for (u, h), (v, g) in (((0, .0005), (.03, .006)), ((.03, .006), (.09, -.002))):
                        kerbs[block % 2].append([border(start, u, h), border(stop, u, h),
                                                 border(stop, v, g), border(start, v, g)])
            distance = end
    surface(link, output, "edge_paint", paint, (.85, .86, .82))
    surface(link, output, "kerb_red", kerbs[0], (.60, .028, .022))
    surface(link, output, "kerb_white", kerbs[1], (.90, .89, .82))
    surface(link, output, "ascari_recessed_kerb", recessed_faces, (.52, .51, .48))

    if circuit == "monza" and lap:
        for name, color, faces in monza_start_grid.details(source, data):
            if name == "grid_asphalt_repairs":
                surface(link, output, name, faces, color, asphalt_span, texture_name=asphalt_texture)
            elif name in ("pit_straight_apron", "pit_straight_lane"):
                surface(link, output, name, faces, (1, 1, 1), asphalt_span, texture_name=asphalt_texture)
            else:
                surface(link, output, name, faces, color, face_up=False)
        _, trackside = model(world, "monza_trackside")
        for name, color, faces in chain(monza_trackside.geometry(source, data),
                                        monza_lesmo.details(source, data),
                                        monza_ascari.details(source, data)):
            if name in ("monza_catch_mesh", "ascari_catch_mesh", "pit_wall_board_mesh"):
                surface(trackside, output, name, faces, color, .10, face_up=False,
                        cutout=True, texture_name="monza_catch_mesh.png")
            elif name in ("monza_weathered_concrete", "monza_jointed_concrete", "monza_historic_wall"):
                surface(trackside, output, name, faces, color, .55, face_up=False, texture_name="wall.jpg")
            else:
                surface(trackside, output, name, faces, color, face_up=False)

    if data.by_kind["grandstand"]:
        _, stands = model(world, "grandstand")
        for name, color, faces in scenery.grandstand(data=data):
            surface(stands, output, name, faces, color, face_up=False)
    for kind in ("pit_building", "building"):
        if data.by_kind[kind]:
            _, buildings = model(world, kind + "s")
            for name, color, faces in scenery.buildings(data, kind):
                surface(buildings, output, name, faces, color, face_up=False)
            if kind == "pit_building" and circuit != "monza":
                for name, color, faces in scenery.pit_facades(data, source["left"]):
                    surface(buildings, output, name, faces, color, face_up=False)
    if data.by_kind["observation_tower"]:
        _, tower = model(world, "observation_towers")
        for name, color, faces in scenery.observation_towers(data):
            surface(tower, output, name, faces, color, face_up=False)
    if data.by_kind["woodland"] or data.by_kind["tree"]:
        _, trees = model(world, "woodland")
        # Keep the original Rettifilo foreground identical in both Monza modes.
        for name, faces in scenery.woodland(lap=lap, data=None if circuit == "monza" else data):
            surface(trees, output, name, faces, (1, 1, 1), face_up=False, cutout=True)


def cameras(link, height, pitch, circuit, far, width, image_height, camera_hz):
    for side, sign in (("left", 1), ("right", -1)):
        sensor = ET.SubElement(link, "sensor", type="camera", name=side)
        field(sensor, "pose", vector(FORWARD_M, sign*BASELINE_M/2, height, 0, pitch, 0))
        field(sensor, "always_on", "true")
        field(sensor, "update_rate", camera_hz)
        field(sensor, "topic", f"/{circuit}/{side}/image")
        field(sensor, "frame_id", OPTICAL_FRAMES[side])
        camera = ET.SubElement(sensor, "camera")
        field(camera, "camera_info_topic", f"/{circuit}/{side}/camera_info")
        field(camera, "horizontal_fov", HFOV)
        image = ET.SubElement(camera, "image")
        field(image, "width", width)
        field(image, "height", image_height)
        field(image, "format", "L_INT8")
        clip = ET.SubElement(camera, "clip")
        field(clip, "near", .025)
        field(clip, "far", far)


def build(output, height=.12, pitch_deg=0, travel_seconds=None, repeat=False, *,
          circuit="monza", route=None, wait_for_capture=False, speed_mps=None, laps=1, record_ground_truth=False,
          resolution="640x480", imu_hz=0, camera_hz=DEFAULT_CAMERA_HZ):
    if resolution not in RESOLUTIONS:
        raise ValueError("unsupported stereo resolution: " + str(resolution))
    if type(camera_hz) is not int or camera_hz not in CAMERA_RATES:
        raise ValueError("ステレオ周波数は 30、60、90 Hz から指定してください")
    width, image_height = RESOLUTIONS[resolution]
    streams = imu_streams(imu_hz)
    # Each exposure falls on a physics tick. Preserve the original 90 Hz rig
    # motion grid, and the 900 Hz pose samples used to derive the optional IMU.
    physics_hz = lcm(900 if imu_hz else 90, camera_hz)
    physics_step_s = (1_000_000_000 // physics_hz) / 1_000_000_000
    if type(laps) is not int or laps < 1:
        raise ValueError("laps must be a positive integer")
    if route is None and laps > 1:
        route = "lap"
    route = circuits.resolve_route(circuit, route)
    if laps > 1 and (route != "lap" or repeat):
        raise ValueError("multiple laps require a finite full-circuit run")
    if wait_for_capture and (route != "lap" or repeat):
        raise ValueError("wait_for_capture requires a finite full-circuit run")
    if not .02 <= height <= 1.0 or not -45 <= pitch_deg <= 45:
        raise ValueError("camera height must be 0.02–1.0 m and pitch −45…+45 degrees")
    output = Path(output).resolve()
    lap = route == "lap"
    far = MONZA_LAP_FAR_M if lap and circuit == "monza" else CAMERA_FAR_M
    if circuit == "monza":
        source, points = load_track(route)
        kerbs = kerb_intervals(source, lap)
        data = scenery.monza_features(lap)
        geography = {
            **reference_metadata(route),
            "scenery_file": f"reference/{scenery.SCENERY_FILE.name}",
            "scenery_sha256": sha256(scenery.SCENERY_FILE.read_bytes()).hexdigest(),
            "modelled": ["stand elevations and seats", "individual trees", "level terrain",
                         "kerb extents and profile", "width of alternate pavements", "surface appearance"],
            "alternate_width_rc_m": LAP_ALTERNATE_WIDTH_M if lap else ALTERNATE_WIDTH_M,
        }
        if lap:
            geography.update(lap_scenery_file=f"reference/{scenery.SCENERY_LAP_FILE.name}",
                             lap_scenery_sha256=sha256(scenery.SCENERY_LAP_FILE.read_bytes()).hexdigest(),
                             background_file=f"reference/{scenery.MONZA_BACKGROUND.name}",
                             background_sha256=sha256(scenery.MONZA_BACKGROUND.read_bytes()).hexdigest())
            geography.update(trackside_file=f"reference/{monza_trackside.REFERENCE.name}",
                             trackside_sha256=sha256(monza_trackside.REFERENCE.read_bytes()).hexdigest(),
                             ascari_file=f"reference/{monza_ascari.REFERENCE.name}",
                             ascari_sha256=sha256(monza_ascari.REFERENCE.read_bytes()).hexdigest(),
                             forest_margins_file=f"reference/{scenery.MONZA_FOREST_MARGINS.name}",
                             forest_margins_sha256=sha256(scenery.MONZA_FOREST_MARGINS.read_bytes()).hexdigest())
            geography["modelled"].extend(["grid and pavement repairs", "pit and stand facades",
                                         "asphalt tone at RC scale", "grandstand catch fencing",
                                         "podium elevations and railings", "radio mast profiles",
                                         "barrier sections and bridge elevations",
                                         "home straight rail overlaps, refuge and pit wall gates",
                                         "pit access markings, bollard and speed limit signs",
                                         "Lesmo and Ascari kerb extents and gravel appearance",
                                         "Lesmo tyre banks and marshal light positions",
                                         "Ascari and back-straight trackside equipment",
                                         "tree and scrub placement in mapped forest margins",
                                         "Junior circuit merge width"])
    else:
        source, points, project = circuits.load_lap(circuit)
        if circuit != "odaiba":
            kerbs = circuits.estimated_kerbs(source, points)
            data = scenery.Features([circuits.ROOT / circuit / "scenery.geojson"], project)
        geography = circuits.metadata(circuit)
    length = points[-1][0]
    travel_seconds, speed_mps = circuits.route_timing(circuit, route, length, travel_seconds, speed_mps)
    try:
        total_seconds = travel_seconds * laps
    except OverflowError as exc:
        raise ValueError("requested run duration is too large") from exc
    if not isfinite(total_seconds):
        raise ValueError("requested speed and run duration must be finite")
    focal = width / (2 * tan(HFOV/2))
    scene = {
        "course": circuit, "environment": "outdoor_circuit",
        "scenery": {"monza": "monza_gp_osm" if lap else "monza_prima_variante_osm",
                    "odaiba": "odaiba_sky_track"}.get(circuit, circuit + "_gp"),
        "geography": geography,
        "trajectory": {"route": route, "wait_for_capture": wait_for_capture, "physics_step_s": physics_step_s,
                       "reference": "Rear axle in local map, metres and radians; axes in geography.axes",
                       "samples_s_x_y_yaw": points, "end_s_m": length, "speed_mps": speed_mps,
                       "travel_seconds": travel_seconds, "repeat": repeat,
                       "laps": None if repeat else laps,
                       "total_travel_seconds": None if repeat else total_seconds},
        "camera": {"height_above_floor_m": height, "pitch_down_deg": pitch_deg,
                   "far_clip_m": far,
                   "midpoint_forward_from_rear_axle_m": FORWARD_M, "baseline_m": BASELINE_M,
                   "horizontal_fov_deg": degrees(HFOV), "nominal_focal_px": focal,
                   "width": width, "height": image_height, "rate_hz": camera_hz, "optical_frames": OPTICAL_FRAMES,
                   "mount_source": "Simulation assumption; actual lens height, pose and intrinsics unmeasured."},
        "imu": {"rate_hz": imu_hz, "frame_id": IMU_FRAME, "topic": IMU_TOPIC,
                "streams": {topic: {"frame_id": frame, "rate_hz": rate} for topic, (frame, rate) in streams.items()},
                "model": "Ideal specific force and angular velocity from post-physics planar rig poses. "
                         "Both optical IMU frames are at the stereo midpoint. No sensor noise or bias. "
                         "Combined acceleration is linearly interpolated between 200 Hz samples."},
    }
    if record_ground_truth:
        scene["trajectory"]["trace_interval_s"] = min(.05, travel_seconds / 120)
    output.mkdir(exist_ok=True, parents=True)
    scene_file = output / "scene.json"
    scene_file.write_text(json.dumps(scene, indent=2) + "\n")
    root = ET.Element("sdf", version="1.12")
    world = ET.SubElement(root, "world", name=circuit)
    if imu_hz:
        field(world, "gravity", vector(0, 0, -GRAVITY_M_S2))
    physics = ET.SubElement(world, "physics", name="camera_step", type="ignored")
    field(physics, "max_step_size", physics_step_s)
    field(physics, "real_time_factor", 1.0)
    for filename, system in (("physics", "Physics"), ("user-commands", "UserCommands"),
                             ("scene-broadcaster", "SceneBroadcaster"), ("sensors", "Sensors")):
        plugin = ET.SubElement(world, "plugin", filename=f"gz-sim-{filename}-system",
                               name=f"gz::sim::systems::{system}")
        if system == "Sensors":
            field(plugin, "render_engine", "ogre2")
    lighting = ET.SubElement(world, "scene")
    field(lighting, "ambient", "0.66 0.70 0.75 1")
    field(lighting, "background", "0.50 0.72 0.91 1")
    light = ET.SubElement(world, "light", type="directional", name="sun")
    field(light, "cast_shadows", "false")
    field(light, "pose", "0 0 12 0 0 0")
    field(light, "diffuse", "0.99 0.96 0.90 1")
    field(light, "intensity", 1.25)
    field(light, "specular", "0.20 0.20 0.19 1")
    field(light, "direction", "0.42 -0.52 -0.74")
    if circuit == "odaiba":
        odaiba_track(world, output, source, project, far)
    else:
        track(world, output, source, lap, circuit, data, kerbs, far)
    rig, link = model(world, "d455_rig", fixed=False)
    _, x, y, yaw = points[0]
    field(rig, "pose", vector(x, y, 0, 0, 0, yaw))
    field(link, "gravity", "false")
    cameras(link, height, radians(pitch_deg), circuit, far, width, image_height, camera_hz)
    motion = ET.SubElement(rig, "plugin", filename="gz-sim-python-system-loader-system",
                           name="gz::sim::systems::PythonSystemLoader")
    field(motion, "module_name", "monza_motion")
    field(motion, "scene_file", str(scene_file))
    ET.indent(root)
    ET.ElementTree(root).write(output / "world.sdf", encoding="UTF-8", xml_declaration=True)
    return scene


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--camera-height", type=float, default=.12, help="Optical midpoint above floor in m")
    parser.add_argument("--camera-pitch-deg", type=float, default=0, help="Positive pitches down")
    parser.add_argument("--resolution", choices=RESOLUTIONS, default="640x480")
    parser.add_argument("--camera-hz", type=int, choices=CAMERA_RATES, default=DEFAULT_CAMERA_HZ)
    parser.add_argument("--imu", choices=IMU_MODES, default="off")
    parser.add_argument("--circuit", choices=circuits.circuit_ids(), default="monza")
    parser.add_argument("--route", choices=("chicane", "lap"))
    timing = parser.add_mutually_exclusive_group()
    timing.add_argument("--travel-seconds", type=float, help="Simulation seconds per pass")
    timing.add_argument("--speed", type=float, help="Speed along the mapped route in RC metres per simulation second")
    parser.add_argument("--laps", type=int, help="Drive this many complete forward laps")
    parser.add_argument("--repeat", action="store_true", help="Repeat the chicane out and back or continue lapping forwards")
    args = parser.parse_args()
    if args.laps is not None and (args.repeat or args.route == "chicane"):
        parser.error("--laps requires a finite lap route")
    route = "lap" if args.laps is not None and args.route is None else args.route
    try:
        build(args.output, args.camera_height, args.camera_pitch_deg, args.travel_seconds, args.repeat,
              circuit=args.circuit, route=route, speed_mps=args.speed, laps=1 if args.laps is None else args.laps,
              resolution=args.resolution, imu_hz=IMU_MODES[args.imu], camera_hz=args.camera_hz)
    except ValueError as exc:
        parser.error(str(exc))
