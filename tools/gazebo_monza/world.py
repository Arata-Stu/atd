"""Generate a scaled circuit and its stereo camera route from local map data."""

import argparse
import json
from itertools import chain
from math import cos, degrees, hypot, isfinite, lcm, radians, tan
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import circuits
from circuits import default_travel_seconds
import odaiba_scenery
from rig_imu import GRAVITY_M_S2
from sensor_profile import CAMERA_RATES, DEFAULT_CAMERA_HZ, IMU_FRAME, IMU_MODES, IMU_TOPIC, RESOLUTIONS, imu_streams

HERE = Path(__file__).resolve().parent
BASELINE_M = 0.095
HFOV = radians(87)
FORWARD_M = 0.18
OPTICAL_FRAMES = {"left": "camera_infra1_optical_frame", "right": "camera_infra2_optical_frame"}
CAMERA_FAR_M = 45


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


def surface(link, output, name, polygons, rgb, texture_tile_m=None, *, face_up=True, texture_name=None):
    """Batch static polygons, tiling ground textures in metres."""
    polygons = iter(polygons)
    first = next(polygons, None)
    if first is None:
        return
    meshfile = output / f"{name}.obj"
    lines = ["# metres, metre-based planar UVs"]
    vertex = 1
    for face_id, corners in enumerate(chain([first], polygons), start=1):
        # Ground faces point up; vertical scenery retains its source winding.
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
        for x, y, z in corners:
            lines.append("v " + vector(x, y, z))
            if texture_tile_m:
                u, v = ((-ny*x + nx*y) / horizontal, z) if horizontal > abs(nz) else (x, y)
                lines.append("vt " + vector(u / texture_tile_m, v / texture_tile_m))
        refs = [f"{v}/{v if texture_tile_m else ''}/{face_id}"
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
    if texture_tile_m:
        target = output / (texture_name or f"{name}.jpg")
        shutil.copyfile(HERE / "materials" / target.name, target)
        metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
        field(metal, "metalness", 0)
        field(metal, "roughness", .96)
        field(metal, "albedo_map", str(target))


def road_ribbon(edges):
    """Join left/right edges sampled at the same centreline stations."""
    left, right = edges["left"], edges["right"]
    for i, (a, b) in enumerate(zip(left, right)):
        j = (i + 1) % len(left)
        yield [(*a, 0), (*b, 0), (*right[j], 0), (*left[j], 0)]


def ground_extent(source, margin):
    xs, ys = zip(*(source["left"] + source["right"]))
    west, east = min(xs)-margin, max(xs)+margin
    south, north = min(ys)-margin, max(ys)+margin
    return [(x, y, -.004) for x, y in ((west, south), (east, south), (east, north), (west, north))]


def odaiba_track(world, output, source, project, far):
    _, link = model(world, "circuit")
    # The facility is paved across the infield and the open pit bays.
    ground = ground_extent(source, far / cos(HFOV/2) + FORWARD_M + BASELINE_M/2)
    surface(link, output, "apron", [ground], (.8, .8, .8), 2.4, texture_name="asphalt.jpg")
    surface(link, output, "floor", road_ribbon(source), (1, 1, 1), 2.4, texture_name="asphalt.jpg")
    for name, color, faces in odaiba_scenery.geometry(source, project):
        surface(link, output, name, faces, color, face_up=False)


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


def depth_camera(link, camera, circuit):
    from depth_ftg import FAR_M, NEAR_M, OPTICAL_FRAME, topic

    sensor = ET.SubElement(link, "sensor", type="depth_camera", name="depth")
    field(sensor, "pose", vector(FORWARD_M, 0, camera["height_above_floor_m"], 0,
                                 radians(camera["pitch_down_deg"]), 0))
    field(sensor, "always_on", "true")
    field(sensor, "update_rate", camera["rate_hz"])
    field(sensor, "topic", topic(circuit))
    field(sensor, "gz_frame_id", OPTICAL_FRAME)
    optics = ET.SubElement(sensor, "camera")
    field(optics, "horizontal_fov", HFOV)
    image = ET.SubElement(optics, "image")
    field(image, "width", camera["width"])
    field(image, "height", camera["height"])
    clip = ET.SubElement(optics, "clip")
    field(clip, "near", NEAR_M)
    field(clip, "far", FAR_M)
    depth = ET.SubElement(optics, "depth_camera")
    depth_clip = ET.SubElement(depth, "clip")
    field(depth_clip, "near", NEAR_M)
    field(depth_clip, "far", FAR_M)


def build(output, height=.12, pitch_deg=0, travel_seconds=None, repeat=False, *,
          circuit="odaiba", route=None, wait_for_capture=False, speed_mps=None, laps=1, record_ground_truth=False,
          resolution="640x480", imu_hz=0, camera_hz=DEFAULT_CAMERA_HZ, command_driven=False,
          full_throttle_speed_mps=None, sensor_ingress=False, ftg_speed=None, ftg_visualize=False):
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
    if type(laps) is not int or laps < 0:
        raise ValueError("laps must be a non-negative integer")
    if laps == 0 and not command_driven:
        repeat = True
    route = circuits.resolve_route(circuit, route)
    if laps > 1 and repeat:
        raise ValueError("multiple laps require a finite full-circuit run")
    if wait_for_capture and repeat:
        raise ValueError("wait_for_capture requires a finite full-circuit run")
    if not .02 <= height <= 1.0 or not -45 <= pitch_deg <= 45:
        raise ValueError("camera height must be 0.02–1.0 m and pitch −45…+45 degrees")
    output = Path(output).resolve()
    source, points, project = circuits.load_lap(circuit)
    geography = circuits.metadata(circuit)
    far = CAMERA_FAR_M
    length = points[-1][0]
    if command_driven:
        if travel_seconds is not None or speed_mps is not None or repeat or wait_for_capture:
            raise ValueError("指令走行の速度と姿勢は制御入力から決まります")
        from closed_loop_motion import FULL_SPEED_MPS
        if full_throttle_speed_mps is None:
            full_throttle_speed_mps = FULL_SPEED_MPS
        elif not isfinite(full_throttle_speed_mps) or full_throttle_speed_mps < .2:
            raise ValueError("全開時の車速には 0.2 m/s 以上の有限値を指定してください")
        total_seconds = None
    else:
        if full_throttle_speed_mps is not None:
            raise ValueError("全開時の車速は closed-loop で指定してください")
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
        "scenery": "odaiba_sky_track",
        "geography": geography,
        "trajectory": {"route": route, "wait_for_capture": wait_for_capture, "physics_step_s": physics_step_s,
                       "reference": "Rear axle in local map, metres and radians; axes in geography.axes",
                       "samples_s_x_y_yaw": points, "end_s_m": length, "speed_mps": speed_mps,
                       "travel_seconds": travel_seconds, "repeat": repeat,
                       "laps": None if repeat and laps != 0 else laps,
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
    if ftg_speed is not None:
        from depth_ftg import BODY_HEIGHT_M, OPTICAL_FRAME, topic

        if not command_driven or not isfinite(ftg_speed) or not .2 <= ftg_speed <= full_throttle_speed_mps:
            raise ValueError("3D FTG には車両モデルの範囲内で 0.2 m/s 以上の速度上限が必要です")
        scene["depth"] = dict(topic=topic(circuit), optical_frame=OPTICAL_FRAME, encoding="R_FLOAT32",
                              body_height_m=BODY_HEIGHT_M, speed_limit_mps=ftg_speed, visualize=ftg_visualize)
    if command_driven:
        from closed_loop_eval import FOOTPRINT
        scene["course_evaluation"] = {"frame": "gazebo_world", **source, "footprint": FOOTPRINT.copy()}
        scene["actuation"] = {"topic": "/atd/gazebo/actuators",
                              "spawn_x_y_yaw": points[0][1:], "model": "rear_axle_kinematic_bicycle",
                              "full_throttle_speed_mps": full_throttle_speed_mps,
                              "drive_timeout_s": 0.30, "trace_interval_s": 0.05}
    elif record_ground_truth:
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
    if sensor_ingress:
        sender = ET.SubElement(world, "plugin", filename="gz-sim-python-system-loader-system",
                               name="gz::sim::systems::PythonSystemLoader")
        field(sender, "module_name", "sensor_ingress")
        field(sender, "course", circuit)
        field(sender, "imu_hz", imu_hz)
        field(sender, "ready_file", str(output / "sensor_ingress.ready"))
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
    odaiba_track(world, output, source, project, far)
    rig, link = model(world, "d455_rig", fixed=False)
    _, x, y, yaw = points[0]
    field(rig, "pose", vector(x, y, 0, 0, 0, yaw))
    field(link, "gravity", "false")
    if ftg_speed is None:
        cameras(link, height, radians(pitch_deg), circuit, far, width, image_height, camera_hz)
    else:
        depth_camera(link, scene["camera"], circuit)
    motion = ET.SubElement(rig, "plugin", filename="gz-sim-python-system-loader-system",
                           name="gz::sim::systems::PythonSystemLoader")
    field(motion, "module_name", "closed_loop_motion" if command_driven else "monza_motion")
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
    parser.add_argument("--circuit", choices=circuits.circuit_ids(), default="odaiba")
    parser.add_argument("--route", choices=("lap",))
    timing = parser.add_mutually_exclusive_group()
    timing.add_argument("--travel-seconds", type=float, help="Simulation seconds per pass")
    timing.add_argument("--speed", type=float, help="Speed along the mapped route in RC metres per simulation second")
    parser.add_argument("--laps", type=int, help="Drive this many complete forward laps (0 to keep lapping)")
    parser.add_argument("--repeat", action="store_true", help="Continue lapping forwards")
    args = parser.parse_args()
    if args.laps is not None and args.repeat:
        parser.error("--laps requires a finite lap route")
    try:
        build(args.output, args.camera_height, args.camera_pitch_deg, args.travel_seconds, args.repeat,
              circuit=args.circuit, route=args.route, speed_mps=args.speed, laps=1 if args.laps is None else args.laps,
              resolution=args.resolution, imu_hz=IMU_MODES[args.imu], camera_hz=args.camera_hz)
    except ValueError as exc:
        parser.error(str(exc))
