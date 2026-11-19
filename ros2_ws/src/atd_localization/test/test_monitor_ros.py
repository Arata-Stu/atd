"""ROS 2 を導入した環境で、隔離 domain の標準 ROS メッセージを往復させる。"""
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from atd_interfaces.msg import LocalizationStatus
from atd_localization.monitor import LocalizationMonitor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def test_ros_stamped_inputs_result_in_single_signed_observation_and_loss():
    context = Context()
    rclpy.init(context=context, domain_id=110 + os.getpid() % 50)
    params = dict(map_id='race', map_sha256='a'*64, hdmap_sha256='b'*64, left_info='/test/left',
                  right_info='/test/right', left_frame='l', right_frame='r', image_width=640, image_height=480,
                  max_age_s=5.)
    monitor = LocalizationMonitor(context=context, parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    driver = Node('visual_slam', context=context)
    executor = SingleThreadedExecutor(context=context)
    for node in (monitor, driver):
        executor.add_node(node)

    def spin_until(predicate):
        deadline = time.monotonic() + 8
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert predicate()

    try:
        cases = [(PoseWithCovarianceStamped, '/localization/pose_hint'), (Odometry, '/visual_slam/tracking/odometry'),
                 (TFMessage, '/tf'), (DiagnosticArray, '/diagnostics'), (CameraInfo, '/test/left'), (CameraInfo, '/test/right')]
        pubs = [driver.create_publisher(typ, topic, 10) for typ, topic in cases]
        received = []
        requests = []
        def trigger(_, response):
            requests.append(True)
            response.success = True
            return response
        driver.create_service(Trigger, '/localization/request_hint', trigger)
        driver.create_subscription(PoseWithCovarianceStamped, '/localization/pose_hint', lambda _: None, 10)
        sub = driver.create_subscription(LocalizationStatus, '/localization/status', received.append, 10)
        spin_until(lambda: sub.get_publisher_count() > 0 and all(p.get_subscription_count() > 0 for p in pubs))
        stamp = driver.get_clock().now().to_msg()
        for frame, pub in zip(('l','r'), pubs[-2:]):
            info = CameraInfo(width=640, height=480)
            info.header.stamp, info.header.frame_id = stamp, frame
            pub.publish(info)
        diag = DiagnosticArray(status=[DiagnosticStatus(hardware_id='visual_slam', values=[
            KeyValue(key='vo_status', value='OK'), KeyValue(key='localized_in_exist_map', value='No')])])
        diag.header.stamp = stamp
        pubs[3].publish(diag)
        # Simulate the first volatile VGL hint having been missed: the monitor
        # must request another only after both hint consumers have been discovered.
        spin_until(lambda: bool(requests))
        assert not any(m.state == 'tracking' for m in received)
        stamp = driver.get_clock().now().to_msg()
        hint = PoseWithCovarianceStamped()
        hint.header.stamp, hint.header.frame_id = stamp, 'map'
        hint.pose.pose.orientation.w = 1.
        pubs[0].publish(hint)
        spin_until(lambda: received and received[-1].state == 'localizing')
        stamp = driver.get_clock().now().to_msg()
        odom = Odometry(child_frame_id='base_link')
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x = 2.
        odom.twist.twist.linear.x = -.5
        odom.twist.twist.angular.z = .1
        tf = TransformStamped(child_frame_id='odom')
        tf.header.stamp, tf.header.frame_id = stamp, 'map'
        tf.transform.rotation.w, tf.transform.translation.y = 1., 3.
        diag.status[0].values[1].value = 'Yes'
        diag.header.stamp = stamp
        for pub, msg in zip(pubs[1:4], (odom, TFMessage(transforms=[tf]), diag)):
            pub.publish(msg)
        spin_until(lambda: any(m.state == 'tracking' for m in received))
        sample = next(m for m in received if m.state == 'tracking')
        assert (sample.map_id, sample.map_sha256, sample.hdmap_sha256) == ('race','a'*64,'b'*64)
        assert sample.header.stamp == stamp and sample.header.frame_id == 'map'
        assert (sample.pose.position.x, sample.pose.position.y, sample.speed_mps, sample.yaw_rate_rps) == (2., 3., -.5, .1)
        diag.header.stamp = driver.get_clock().now().to_msg()
        diag.status[0].values[0].value = 'Lost'
        pubs[3].publish(diag)
        spin_until(lambda: received[-1].state == 'lost')
        assert received[-1].epoch > sample.epoch
        assert requests == [True]
    finally:
        executor.shutdown()
        driver.destroy_node()
        monitor.destroy_node()
        rclpy.shutdown(context=context)
