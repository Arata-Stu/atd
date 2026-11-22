"""Collect only data received after saved-map localization is confirmed."""

import copy
import time

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data

from .capture_state import LocalizationGate, require_tf_publisher, validate_final_samples
from .collector import Collector
from .frames import FinalFrames, stamp_ns


class CaptureCollector(Collector):
    def __init__(self, hint_topic=None):
        self.vgl_hint = None
        self.tf_publisher_gid = None
        self.gate = LocalizationGate()
        self.last_update = time.monotonic()
        super().__init__()
        self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.on_diagnostics,
            qos_profile_sensor_data,
        )

        if hint_topic:
            from geometry_msgs.msg import PoseWithCovarianceStamped

            self.create_subscription(PoseWithCovarianceStamped, hint_topic, self.on_vgl_hint, 10)

    def on_vgl_hint(self, msg):
        self.vgl_hint = dict(stamp_ns=str(stamp_ns(msg.header.stamp)), frame=msg.header.frame_id)
        self.get_logger().info("VGL pose hint received; waiting for saved-map localization")

    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.hardware_id == "visual_slam" and self.gate.update(
                stamp_ns(msg.header.stamp),
                {v.key: v.value for v in status.values},
                time.monotonic(),
            ):
                self.cloud = self.path = None
                self.frames = FinalFrames()
                self.get_logger().info(
                    f"Saved-map localized: {self.gate.localized}（{self.gate.diagnostic_summary()}）"
                )

    def on_tf(self, msg, info):
        if self.tf_publisher_gid is None:
            return
        require_tf_publisher(info, self.tf_publisher_gid)
        if not self.gate.localized:
            return
        for tf in msg.transforms:
            if self.gate.accepts(stamp_ns(tf.header.stamp)):
                from .frames import transform_record

                record = transform_record(tf)
                previous = self.frames.transforms.get(record["child"])
                self.frames.update(record)
                if self.frames.transforms.get(record["child"]) != previous:
                    self.last_update = time.monotonic()

    def on_cloud(self, msg):
        if self.gate.accepts(stamp_ns(msg.header.stamp)):
            changed = (
                self.cloud is None
                or stamp_ns(msg.header.stamp) > stamp_ns(self.cloud.header.stamp)
                or msg.data != self.cloud.data
            )
            super().on_cloud(msg)
            if changed:
                self.last_update = time.monotonic()

    def on_path(self, msg):
        if self.gate.accepts(stamp_ns(msg.header.stamp)):
            msg = copy.copy(msg)
            msg.poses = [
                p for p in msg.poses if self.gate.accepts(stamp_ns(p.header.stamp))
            ]
            changed = (
                self.path is None
                or stamp_ns(msg.header.stamp) > stamp_ns(self.path.header.stamp)
                or msg.poses != self.path.poses
            )
            super().on_path(msg)
            if changed:
                self.last_update = time.monotonic()

    def check_final_inputs(self, expected, tolerance, hint_required):
        tf = self.frames.transforms.get('odom')
        return validate_final_samples(
            self.gate, expected, tolerance,
            stamp_ns(self.cloud.header.stamp) if self.cloud is not None else None,
            stamp_ns(self.path.header.stamp) if self.path is not None else None,
            int(tf['stamp_ns']) if tf else None,
            hint_required=hint_required, hint_stamp=self.vgl_hint['stamp_ns'] if self.vgl_hint else None,
        )

    def final_snapshot(self, expected, tolerance, provenance):
        hint_required = provenance.get("initial_localization") == "vgl_pose_hint"
        tail = self.check_final_inputs(expected, tolerance, hint_required)
        if hint_required:
            provenance["vgl_pose_hint"] = self.vgl_hint
        cloud_stamp = stamp_ns(self.cloud.header.stamp)
        cloud_age_s = tail['cloud_tail_age_s']
        if cloud_stamp < expected - tolerance:
            self.get_logger().warning(
                f"点群の更新時刻は入力末尾より{cloud_age_s:.3f}秒前です。"
                "最終localize成功区間の最新非空点群を採用します（全地図の網羅性は未保証）"
            )
        provenance.update(
            localized_in_existing_map=True,
            localized_since_ns=str(self.gate.since),
            final_diagnostic_stamp_ns=str(self.gate.stamp),
            expected_final_image_stamp_ns=str(expected),
            cloud_tail_age_s=cloud_age_s,
            cloud_within_tail_tolerance=tail['cloud_within_tail_tolerance'],
            point_scope="last_nonempty_visualization_cloud_not_guaranteed_full_map",
        )
        return self.snapshot(provenance)
