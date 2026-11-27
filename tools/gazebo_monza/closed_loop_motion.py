"""Gazebo system for the D455 on a command-driven, forward-only RC vehicle."""

import json
import math
from pathlib import Path
import sys

# The Gazebo image mounts the checkout, independently of the Isaac ROS overlay.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws/src/atd_control"))
from atd_control.bicycle import Bicycle, FULL_SPEED_MPS, SPEED_TAU_S, STEERING_LIMIT, STEERING_TAU_S
from rig_imu import ImuPublisher
from sensor_profile import REAR_AXLE_HEIGHT_M

JOINTS = ("steering", "throttle", "enable")
ACTUATOR_RESPONSE = {"steering_tau_s": STEERING_TAU_S, "speed_tau_s": SPEED_TAU_S}


class CommandPlant:
    def __init__(self, spawn, timeout_s, *, full_throttle_speed_mps=FULL_SPEED_MPS):
        self.vehicle = Bicycle(*spawn, full_throttle_speed_mps=full_throttle_speed_mps)
        self.timeout_ns = round(timeout_s * 1e9)
        self.state = "waiting"
        self.reason = ""
        self.first_active_ns = None
        self.command_stamp_ns = None
        self.command_applied_ns = None
        self.hold_stamp_ns = None
        self.hold_applied_ns = None

    def stop(self, reason, *, before_start=False, wait_for_command=False):
        self.vehicle.steering_setpoint = 0.
        self.vehicle.throttle_setpoint = 0.
        if self.state in ("driving", "waiting_observation", "held") or (before_start and self.state == "waiting"):
            self.state = "waiting_observation" if wait_for_command else "stopped"
            self.reason = reason

    def receive(self, now_ns, stamp_ns, names, values):
        if self.state == "stopped":
            return
        if tuple(names) != JOINTS or len(values) != 3 or not all(math.isfinite(v) for v in values):
            self.stop("invalid_command")
            return
        steering, throttle, enabled = values
        if enabled == 0.:
            if self.hold_stamp_ns is not None and stamp_ns < self.hold_stamp_ns:
                # A pre-launch disable queued before an operator hold is obsolete.
                return
            self.stop("controller_disabled")
            return
        if enabled == 2. and steering == throttle == 0.:
            if (not 0 <= stamp_ns <= now_ns or now_ns - stamp_ns > self.timeout_ns
                    or self.command_stamp_ns is not None and stamp_ns < self.command_stamp_ns
                    or self.hold_stamp_ns is not None and stamp_ns <= self.hold_stamp_ns):
                return
            # This is a simulator position hold. Continue publishing the same pose
            # at new sensor timestamps, without integrating a coast-down.
            self.vehicle.speed = self.vehicle.steering = 0.
            self.vehicle.steering_setpoint = self.vehicle.throttle_setpoint = 0.
            self.hold_stamp_ns, self.hold_applied_ns = stamp_ns, now_ns
            self.state, self.reason = "held", "operator_paused"
            return
        if (enabled != 1. or abs(steering) > STEERING_LIMIT + 1e-7 or not 0. <= throttle <= 1.
                or stamp_ns <= 0 or stamp_ns > now_ns):
            self.stop("invalid_or_stale_command")
            return
        if (now_ns - stamp_ns > self.timeout_ns
                or self.hold_applied_ns is not None and stamp_ns <= self.hold_applied_ns
                or self.command_stamp_ns is not None and stamp_ns <= self.command_stamp_ns):
            return
        if self.first_active_ns is None:
            self.first_active_ns = now_ns
        self.command_stamp_ns = stamp_ns
        self.command_applied_ns = now_ns
        self.state, self.reason = "driving", ""
        self.vehicle.steering_setpoint = steering
        self.vehicle.throttle_setpoint = throttle

    def advance(self, now_ns, dt):
        if self.state == "driving" and now_ns - self.command_stamp_ns > self.timeout_ns:
            self.stop("command_timeout", wait_for_command=True)
        self.vehicle.advance(dt, ACTUATOR_RESPONSE)


class ClosedLoopMotion:
    def configure(self, entity, sdf, ecm, event_mgr):
        from gz.math import Pose3d
        from gz.sim import Link, Model
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

        scene_file = Path(sdf.get_string("scene_file"))
        scene = json.loads(scene_file.read_text())
        drive = scene["actuation"]
        self.plant = CommandPlant(drive["spawn_x_y_yaw"], drive["drive_timeout_s"],
                                  full_throttle_speed_mps=drive.get("full_throttle_speed_mps", FULL_SPEED_MPS))
        self.model = Model(entity)
        self.link = Link(self.model.canonical_link(ecm))
        self.pose_type = Pose3d
        self.context = Context()
        self.context.init(args=[])
        self.node = Node("atd_gazebo_actuators", context=self.context, use_global_arguments=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.actuator_topic = drive["topic"]
        self.node.create_subscription(JointState, self.actuator_topic, self.on_command, 1)
        self.node.create_subscription(String, "/atd/gazebo/control_state", self.on_control_state,
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ground_truth = self.node.create_publisher(PoseStamped, "/atd/gazebo/ground_truth_pose",
                                                       qos_profile_sensor_data)
        self.model_speed = self.node.create_publisher(TwistStamped, "/atd/gazebo/model_speed", qos_profile_sensor_data)
        self.speed_message = TwistStamped()
        self.speed_message.header.frame_id = "base_link"
        self.hold_status = self.node.create_publisher(String, "/atd/gazebo/vehicle_hold", 1)
        self.hold_message = String()
        self.gt_pose = PoseStamped()
        self.gt_pose.header.frame_id = "gazebo_world"
        self.control_state = None
        self.active_file = scene_file.with_name("closed_loop.active")
        self.prestart_pause_file = scene_file.with_name("closed_loop.paused")
        self.trace = scene_file.with_name("rig_motion.jsonl")
        self.trace.write_text("")
        self.next_trace_s = 0.
        self.trace_interval_s = drive["trace_interval_s"]
        self.now_ns = self.next_spin_ns = 0
        imu_hz = scene["imu"]["rate_hz"]
        self.imu = ImuPublisher(scene["course"], scene["camera"], imu_hz) if imu_hz else None

    def on_command(self, msg):
        publisher_count = self.node.count_publishers(self.actuator_topic)
        if publisher_count != 1:
            if publisher_count > 1:
                self.plant.stop("multiple_actuator_publishers", before_start=True)
            return
        stamp = msg.header.stamp
        initially_active = self.plant.first_active_ns is not None
        self.plant.receive(self.now_ns, stamp.sec * 10**9 + stamp.nanosec, msg.name, msg.position)
        if self.plant.first_active_ns is not None and not initially_active:
            self.active_file.write_text(json.dumps({"sim_time_s": self.plant.first_active_ns / 1e9}) + "\n")

    def on_control_state(self, msg):
        try:
            report = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(report, dict):
            previous_state = (self.control_state or {}).get("state")
            if report.get("state") == "paused" and previous_state != "paused" and self.plant.first_active_ns is None:
                self.prestart_pause_file.touch()
            elif report.get("state") != "paused" and previous_state == "paused":
                self.prestart_pause_file.unlink(missing_ok=True)
            self.control_state = report

    def pre_update(self, info, ecm):
        if info.paused:
            return
        previous_ns = self.now_ns
        self.now_ns = round(info.sim_time.total_seconds() * 1e9)
        if self.now_ns < previous_ns:
            self.plant.stop("clock_reset")
            self.next_spin_ns = self.now_ns
        if self.now_ns >= self.next_spin_ns:
            # Process the latest actuator and controller status without blocking physics.
            self.executor.spin_once(timeout_sec=0.)
            self.executor.spin_once(timeout_sec=0.)
            self.next_spin_ns = self.now_ns + 5_000_000
        self.plant.advance(self.now_ns, max(0, self.now_ns - previous_ns) * 1e-9)
        v = self.plant.vehicle
        self.model.set_world_pose_cmd(ecm, self.pose_type(v.x, v.y, 0, 0, 0, v.yaw))

    def post_update(self, info, ecm):
        if info.paused:
            return
        seconds = info.sim_time.total_seconds()
        if self.imu is None and seconds < self.next_trace_s:
            return
        pose = self.link.world_pose(ecm)
        if self.imu is not None:
            self.imu.update(round(seconds * 1e9), pose)
        if seconds < self.next_trace_s:
            return
        self.next_trace_s += self.trace_interval_s
        if self.ground_truth.get_subscription_count():
            # The planar rig link is at floor level underneath the rear axle.
            msg = self.gt_pose
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(seconds * 1e9), 10**9)
            msg.pose.position.x, msg.pose.position.y = pose.x(), pose.y()
            msg.pose.position.z = pose.z() + REAR_AXLE_HEIGHT_M
            msg.pose.orientation.z = math.sin(pose.yaw() * .5)
            msg.pose.orientation.w = math.cos(pose.yaw() * .5)
            self.ground_truth.publish(msg)
        plant, v = self.plant, self.plant.vehicle
        if self.model_speed.get_subscription_count():
            msg = self.speed_message
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(seconds * 1e9), 10**9)
            msg.twist.linear.x = v.speed
            self.model_speed.publish(msg)
        if self.hold_status.get_subscription_count():
            self.hold_message.data = json.dumps(dict(held=plant.state == "held" and v.speed == 0.,
                                                     stamp_ns=round(seconds * 1e9), speed_mps=v.speed), allow_nan=False)
            self.hold_status.publish(self.hold_message)
        sample = dict(sim_time_s=seconds, x_m=pose.x(), y_m=pose.y(), yaw_rad=pose.yaw(),
                      speed_mps=v.speed, steering_rad=v.steering, throttle=v.throttle_setpoint,
                      requested_steering_rad=v.steering_setpoint, command_stamp_ns=plant.command_stamp_ns,
                      command_applied_ns=plant.command_applied_ns,
                      first_active_ns=plant.first_active_ns, actuator_state=plant.state,
                      actuator_reason=plant.reason, control_state=self.control_state)
        with self.trace.open("a") as stream:
            stream.write(json.dumps(sample, allow_nan=False) + "\n")


def get_system():
    return ClosedLoopMotion()
