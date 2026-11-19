"""Installed-node integration test. Only test-specific topics and a temporary bag are used."""
from pathlib import Path
import signal
import subprocess
from time import monotonic
from uuid import uuid4

import pytest
import yaml


def test_joy_start_propo_stop_writes_readable_mcap(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    import rosbag2_py
    from ament_index_python.packages import get_package_prefix
    from atd_interfaces.msg import DriveCommand, JoyDriveStatus, OperatorEvent
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    prefix = '/atd_test_' + uuid4().hex
    sample_topic = prefix + '/sample'
    command_topic = prefix + '/teleop/command'
    feedback_topic = prefix + '/vehicle/joy_drive/status'
    root = tmp_path / 'bags'
    config = tmp_path / 'recording.yaml'
    config.write_text(yaml.safe_dump({'/atd_recording_node': {'ros__parameters': {
        'profile_name': 'inputs', 'record_root': str(root), 'min_free_bytes': 1,
        'topics': [sample_topic, command_topic, feedback_topic], 'required_topics': [sample_topic],
        'required_types': ['std_msgs/msg/String'],
        'best_effort_topics': [feedback_topic],
    }}}))
    executable = Path(get_package_prefix('atd_recording')) / 'lib/atd_recording/atd_recording_node'
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(prefix[1:], context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    statuses = []
    node.create_subscription(String, prefix + '/status', lambda msg: statuses.append(yaml.safe_load(msg.data)), 10)
    joy = node.create_publisher(OperatorEvent, prefix + '/joy', 10)
    propo = node.create_publisher(OperatorEvent, prefix + '/propo', 10)
    samples = node.create_publisher(String, sample_topic, 10)

    def send(publisher, source, action):
        event = OperatorEvent(source=source, action=action)
        event.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(event)

    process = subprocess.Popen([
        str(executable), '--ros-args', '--params-file', str(config),
        '-r', '/teleop/events:=' + prefix + '/joy', '-r', '/rc/events:=' + prefix + '/propo',
        '-r', '/recording/status:=' + prefix + '/status',
    ])
    try:
        def wait_until(condition):
            deadline = monotonic() + 12
            while not condition():
                assert process.poll() is None, f'recorder exited with {process.returncode}'
                assert monotonic() < deadline, f'timed out, last status = {statuses[-1:]}'
                executor.spin_once(timeout_sec=0.05)

        wait_until(lambda: statuses and statuses[-1]['state'] == 'idle'
                   and joy.get_subscription_count() > 0 and propo.get_subscription_count() > 0)
        assert not list(root.glob('bag_*'))
        send(joy, 'joy', OperatorEvent.RECORD_START)
        wait_until(lambda: statuses[-1]['state'] == 'recording' and samples.get_subscription_count() > 0)
        # 入力機器が記録開始後に現れても rosbag2 が両方を発見できることを確認する。
        command_pub = node.create_publisher(DriveCommand, command_topic, 1)
        feedback_pub = node.create_publisher(JoyDriveStatus, feedback_topic, qos_profile_sensor_data)
        wait_until(lambda: command_pub.get_subscription_count() > 0 and feedback_pub.get_subscription_count() > 0)
        command = DriveCommand(source='joy', valid=True, enable_requested=True, steering=-0.5, throttle=0.25)
        command.header.frame_id = 'atd_gamepad'
        command.header.stamp = node.get_clock().now().to_msg()
        feedback = JoyDriveStatus(has_report=True, serial='test-pico', session_id='test-session',
                                  sequence_number=13, selector=1, rx_mode_us=1950, active_path=2,
                                  servo_us=1240, esc_us=1630, joy_active=True)
        feedback.header.frame_id = 'pico'
        feedback.header.stamp = node.get_clock().now().to_msg()
        # Use repeated samples through one status period so acquisition precedes STOP.
        acquired = monotonic()
        while monotonic() - acquired < 0.5:
            executor.spin_once(timeout_sec=0.05)
            samples.publish(String(data='vehicle calibration'))
            command_pub.publish(command)
            feedback_pub.publish(feedback)
        send(propo, 'propo', OperatorEvent.RECORD_STOP)
        wait_until(lambda: statuses[-1]['last_result'] is not None)
        status = statuses[-1]
        assert status['state'] == 'idle' and status['last_result']['success'], status
        bag = Path(status['last_result']['bag_path'])
        assert bag.parent == root
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        expected = {sample_topic: String(data='vehicle calibration'), command_topic: command, feedback_topic: feedback}
        observed = set()
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            assert deserialize_message(raw, type(expected[topic])) == expected[topic]
            observed.add(topic)
        assert observed == set(expected)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
