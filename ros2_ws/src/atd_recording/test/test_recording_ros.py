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
    from atd_interfaces.msg import OperatorEvent
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    prefix = '/atd_test_' + uuid4().hex
    sample_topic = prefix + '/sample'
    root = tmp_path / 'bags'
    config = tmp_path / 'recording.yaml'
    config.write_text(yaml.safe_dump({'/atd_recording_node': {'ros__parameters': {
        'profile_name': 'inputs', 'record_root': str(root), 'min_free_bytes': 1,
        'topics': [sample_topic], 'required_topics': [sample_topic],
        'required_types': ['std_msgs/msg/String'],
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
        samples.publish(String(data='vehicle calibration'))
        # Use repeated samples through one status period so acquisition precedes STOP.
        acquired = monotonic()
        while monotonic() - acquired < 0.5:
            executor.spin_once(timeout_sec=0.05)
            samples.publish(String(data='vehicle calibration'))
        send(propo, 'propo', OperatorEvent.RECORD_STOP)
        wait_until(lambda: statuses[-1]['last_result'] is not None)
        status = statuses[-1]
        assert status['state'] == 'idle' and status['last_result']['success'], status
        bag = Path(status['last_result']['bag_path'])
        assert bag.parent == root
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        topic, raw, _ = reader.read_next()
        assert topic == sample_topic and deserialize_message(raw, String).data == 'vehicle calibration'
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
