"""Generate the RC Monza starter section, its OGRE 2 materials and stereo rig."""

import argparse
from hashlib import sha256
import json
from math import atan2, ceil, degrees, dist, hypot, isfinite, radians, tan
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from monza_motion import interpolate

HERE = Path(__file__).resolve().parent
MAP = HERE.parents[1] / "samples/f1tenth/maps/monza"
END_M = 12.5  # Beyond the right-left Rettifilo on this compact, smoothed layout.
BASELINE_M = 0.095
HFOV = radians(87)
FORWARD_M = 0.18
OPTICAL_FRAMES = {"left": "camera_infra1_optical_frame", "right": "camera_infra2_optical_frame"}
HALL = (-3.4, -4.0, 10.8, 20.5, 3.5)


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


def write_obj(path, quads, tile_m):
    """Quads face the camera side; texture coordinates have a fixed metric scale."""
    lines = ["# Metres; tile coordinates are in units of the source photograph."]
    for i, corners in enumerate(quads):
        u = dist(corners[0], corners[1]) / tile_m
        v = dist(corners[1], corners[2]) / tile_m
        for vertex in corners:
            lines.append("v " + vector(*vertex))
        for uv in ((0, 0), (u, 0), (u, v), (0, v)):
            lines.append("vt " + vector(*uv))
        a0, b0, c0 = corners[:3]
        dx, dy, dz = (b0[k] - a0[k] for k in range(3))
        ex, ey, ez = (c0[k] - b0[k] for k in range(3))
        normal = (dy*ez - dz*ey, dz*ex - dx*ez, dx*ey - dy*ex)
        magnitude = hypot(*normal)
        lines.append("vn " + vector(*(n / magnitude for n in normal)))
        a, b, c, d = (f"{j}/{j}/{i+1}" for j in range(4 * i + 1, 4 * i + 5))
        lines.extend((f"f {a} {b} {c}", f"f {a} {c} {d}"))
    path.write_text("\n".join(lines) + "\n")


def photo_surface(link, output, name, quads, tile_m):
    meshfile = output / f"{name}.obj"
    write_obj(meshfile, quads, tile_m)
    visual = ET.SubElement(link, "visual", name=name)
    geom = ET.SubElement(visual, "geometry")
    field(ET.SubElement(geom, "mesh"), "uri", str(meshfile))
    mat = ET.SubElement(visual, "material")
    field(mat, "diffuse", "0.9 0.9 0.9 1")
    metal = ET.SubElement(ET.SubElement(mat, "pbr"), "metal")
    field(metal, "metalness", 0)
    field(metal, "roughness", 0.83)
    for suffix, key in ((".jpg", "albedo_map"), ("_normal.png", "normal_map"),
                        ("_roughness.png", "roughness_map")):
        asset = HERE / "materials" / (name + suffix)
        if key == "albedo_map" or asset.is_file():
            target = output / asset.name
            shutil.copyfile(asset, target)
            field(metal, key, str(target))


def box(link, name, position, size, rgb, yaw=0, emissive=False):
    visual = ET.SubElement(link, "visual", name=name)
    field(visual, "pose", vector(*position, 0, 0, yaw))
    field(ET.SubElement(ET.SubElement(visual, "geometry"), "box"), "size", vector(*size))
    material = ET.SubElement(visual, "material")
    field(material, "ambient", vector(*rgb, 1))
    field(material, "diffuse", vector(*rgb, 1))
    if emissive:
        field(material, "emissive", vector(*rgb, 1))


def room(world, output):
    _, link = model(world, "hall")
    xmin, ymin, xmax, ymax, height = HALL
    corners = [(xmin, ymin, 0), (xmax, ymin, 0), (xmax, ymax, 0), (xmin, ymax, 0)]
    photo_surface(link, output, "floor", [corners], 2.3)
    # Floor edges run counterclockwise when viewed from above.
    walls = [[a, b, (*b[:2], height), (*a[:2], height)]
             for a, b in zip(corners, corners[1:] + corners[:1])]
    # For floor edge a->b, a horizontal tangent crossed with +z points outwards.
    photo_surface(link, output, "wall", [[q[1], q[0], q[3], q[2]] for q in walls], 4.0)
    box(link, "ceiling", ((xmax+xmin)/2, (ymax+ymin)/2, height+.05),
        (xmax-xmin, ymax-ymin, .1), (.55, .54, .52))
    for i, y in enumerate((-1.7, 3.1, 7.9, 12.7, 17.5)):
        box(link, f"beam_{i}", ((xmax+xmin)/2, y, height-.14),
            (xmax-xmin, .15, .28), (.33, .32, .30))
        for j, x in enumerate((xmin+.10, xmax-.10)):
            box(link, f"column_{i}_{j}", (x, y, height/2), (.20, .26, height), (.45, .43, .40))
        box(link, f"lamp_{i}", (.2, y+.3, height-.12), (2.1, .07, .05), (.92, .89, .82), emissive=True)
        light = ET.SubElement(world, "light", type="point", name=f"ceiling_light_{i}")
        field(light, "pose", vector(.2, y+.3, height-.16, 0, 0, 0))
        field(light, "diffuse", "0.95 0.94 0.91 1")
        field(light, "intensity", 3.0)
        field(light, "specular", "0.5 0.49 0.47 1")
        attenuation = ET.SubElement(light, "attenuation")
        for key, value in (("range", 9), ("constant", .6), ("linear", .12), ("quadratic", .11)):
            field(attenuation, key, value)
    # A service door and shutter give the distant wall depth and break up repetition.
    box(link, "service_door", (-1.9, ymax-.035, 1.06), (.91, .07, 2.12), (.27, .31, .29))
    box(link, "door_handle", (-1.58, ymax-.08, .96), (.11, .04, .025), (.69, .67, .64))
    box(link, "shutter", (2.2, ymax-.045, 1.32), (3, .09, 2.64), (.48, .49, .47))
    for i in range(14):
        box(link, f"slat_{i}", (2.2, ymax-.095, .10+.18*i), (3, .014, .016), (.34, .35, .33))


def track(world, source, last_xy):
    _, link = model(world, "course_edges")
    for side in ("left", "right"):
        # Use the independently edited HDMap edges; keep sight lines past the exit.
        points = source[side]
        end = min(range(len(points)), key=lambda i: dist(points[i], last_xy))
        sign = 1 if side == "left" else -1
        for index, (a, b) in enumerate(zip(points[:end], points[1:end+1])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = hypot(dx, dy)
            yaw = atan2(dy, dx)
            nx, ny = -dy/length*sign, dx/length*sign
            # Low rubber barriers sit outside the drawn drivable boundary.
            count = ceil(length / 1.2)
            for piece in range(count):
                fraction = (piece + .5) / count
                x, y = a[0]+fraction*dx, a[1]+fraction*dy
                h = .075
                box(link, f"kerb_{side}_{index}_{piece}",
                    (x+nx*.033, y+ny*.033, h/2), (length/count-.002, .066, h),
                    (.12, .13, .125), yaw)
            # Worn off-white edge tape sits on asphalt just inside the boundary.
            box(link, f"tape_{side}_{index}",
                ((a[0]+b[0])/2-nx*.025, (a[1]+b[1])/2-ny*.025, .0008),
                (length, .018, .0016), (.62, .60, .54), yaw)


def cameras(link, height, pitch):
    for side, sign in (("left", 1), ("right", -1)):
        sensor = ET.SubElement(link, "sensor", type="camera", name=side)
        field(sensor, "pose", vector(FORWARD_M, sign*BASELINE_M/2, height, 0, pitch, 0))
        field(sensor, "always_on", "true")
        field(sensor, "update_rate", 90)
        field(sensor, "topic", f"/monza/{side}/image")
        field(sensor, "frame_id", OPTICAL_FRAMES[side])
        camera = ET.SubElement(sensor, "camera")
        field(camera, "camera_info_topic", f"/monza/{side}/camera_info")
        field(camera, "horizontal_fov", HFOV)
        image = ET.SubElement(camera, "image")
        field(image, "width", 640)
        field(image, "height", 480)
        field(image, "format", "L_INT8")
        clip = ET.SubElement(camera, "clip")
        field(clip, "near", .025)
        field(clip, "far", 45)


def build(output, height=.12, pitch_deg=0, travel_seconds=10.0, repeat=False):
    if not .02 <= height <= 1.0 or not -45 <= pitch_deg <= 45:
        raise ValueError("camera height must be 0.02–1.0 m and pitch −45…+45 degrees")
    if not isfinite(travel_seconds) or travel_seconds <= 0:
        raise ValueError("travel_seconds must be positive and finite")
    output = Path(output).resolve()
    output.mkdir(exist_ok=True, parents=True)
    mapfile, linefile = MAP / "hdmap/map.json", MAP / "lines/raceline.json"
    source = json.loads(mapfile.read_bytes())
    line = json.loads(linefile.read_bytes())
    samples = [row[:4] for row in line["profile"]]
    distances = [row[0] for row in samples]
    if line["map_id"] != "monza" or len(distances) < 2 or distances[-1] < END_M+6:
        raise ValueError("Monza raceline is missing or too short")
    points = [row for row in samples if row[0] < END_M]
    points.append([END_M, *interpolate(samples, distances, END_M)])
    focal = 320 / tan(HFOV/2)
    scene = {
        "course": "monza",
        "map_sha256": sha256(mapfile.read_bytes()).hexdigest(),
        "raceline_sha256": sha256(linefile.read_bytes()).hexdigest(),
        "trajectory": {"reference": "map, rear axle x forward / y left, metres and radians",
                       "samples_s_x_y_yaw": points, "end_s_m": END_M,
                       "travel_seconds": travel_seconds, "repeat": repeat},
        "camera": {"height_above_floor_m": height, "pitch_down_deg": pitch_deg,
                   "midpoint_forward_from_rear_axle_m": FORWARD_M, "baseline_m": BASELINE_M,
                   "horizontal_fov_deg": degrees(HFOV), "nominal_focal_px": focal,
                   "width": 640, "height": 480, "rate_hz": 90, "optical_frames": OPTICAL_FRAMES,
                   "mount_source": "Simulation assumption; actual lens height, pose and intrinsics unmeasured."},
    }
    scene_file = output / "scene.json"
    scene_file.write_text(json.dumps(scene, indent=2) + "\n")
    root = ET.Element("sdf", version="1.12")
    world = ET.SubElement(root, "world", name="monza")
    # A 1/90 s step rounded to gz's nanosecond duration. Motion and both exposures
    # use this simulation clock; a slower renderer is detected on the ROS side.
    physics = ET.SubElement(world, "physics", name="camera_step", type="ignored")
    field(physics, "max_step_size", .011111111)
    field(physics, "real_time_factor", 1.0)
    for filename, system in (("physics", "Physics"), ("user-commands", "UserCommands"),
                             ("scene-broadcaster", "SceneBroadcaster"), ("sensors", "Sensors")):
        plugin = ET.SubElement(world, "plugin", filename=f"gz-sim-{filename}-system",
                               name=f"gz::sim::systems::{system}")
        if system == "Sensors":
            field(plugin, "render_engine", "ogre2")
    lighting = ET.SubElement(world, "scene")
    field(lighting, "ambient", "0.72 0.72 0.70 1")
    field(lighting, "background", "0.24 0.24 0.25 1")
    light = ET.SubElement(world, "light", type="directional", name="window_light")
    # Broad bounced daylight inside the hall; exterior occlusion would black
    # this out in the enclosed prototype. The fixtures supply local variation.
    field(light, "cast_shadows", "false")
    field(light, "pose", "0 0 3 0 0 0")
    field(light, "diffuse", "0.93 0.93 0.90 1")
    field(light, "intensity", 2.4)
    field(light, "specular", "0.60 0.60 0.58 1")
    field(light, "direction", "0.42 -0.52 -0.74")
    room(world, output)
    track(world, source, interpolate(samples, distances, END_M+6)[:2])
    rig, link = model(world, "d455_rig", fixed=False)
    _, x, y, yaw = points[0]
    field(rig, "pose", vector(x, y, 0, 0, 0, yaw))
    field(link, "gravity", "false")
    cameras(link, height, radians(pitch_deg))
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
    parser.add_argument("--travel-seconds", type=float, default=10)
    parser.add_argument("--repeat", action="store_true", help="Travel forward then backward for sustained measurements")
    args = parser.parse_args()
    build(args.output, args.camera_height, args.camera_pitch_deg, args.travel_seconds, args.repeat)
