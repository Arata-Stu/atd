"""Jetson-local HTTP observations and bounded ROS configuration requests."""
from __future__ import annotations

import json
import math
import os
import re
import socket
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import yaml
from atd_teleop_tools import profile as teleop_profiles

from .localization import live_observations, observation
from .core import Cache, ProfileStore, TeleopProfileStore, PropoProfileStore, VehicleProfileStore, prepare_parameter_value


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def pose_value(pose, header):
    q = pose.orientation
    return {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
            'yaw': math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)),
            'frame': header.frame_id, 'stamp': stamp(header)}


def velocity_value(twist, header):
    return {'linear_mps': twist.linear.x, 'angular_rps': twist.angular.z,
            'linear': {'x': twist.linear.x, 'y': twist.linear.y, 'z': twist.linear.z},
            'frame': header.frame_id, 'stamp': stamp(header)}


def parameter_value(value):
    fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
              5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
              8: 'double_array_value', 9: 'string_array_value'}
    if value.type not in fields:
        return None
    result = getattr(value, fields[value.type])
    return list(result) if value.type >= 5 else result


class BridgeNode(Node):
    def __init__(self):
        from atd_interfaces.msg import LocalizationStatus
        from atd_esc_trials.store import TrialStore
        super().__init__('atd_web_bridge')
        fixed = ParameterDescriptor(read_only=True)
        defaults = {'port': 8790, 'freshness_s': 2.5, 'esc_trials_enabled': False,
            'localization_topic': '/localization/status',
            'odometry_topic': '/localization/odometry', 'pose_topic': '/localization/pose',
            'velocity_topic': '/vehicle/velocity', 'joy_topic': '/joy',
            'joy_raw_topic': '/joy/raw', 'joy_status_topic': '/joy/status',
            'joy_node': '/atd_joy_node', 'profile_path': '/data/results/runs/input/dualsense.yaml',
            'teleop_node': '/atd_teleop_node', 'teleop_status_topic': '/teleop/status',
            'teleop_profile_path': '/data/results/runs/input/teleop.yaml',
            'pico_status_topic': '/vehicle/pico/status', 'rc_status_topic': '/rc/status',
            'rc_node': '/atd_rc_input_node', 'rc_profile_path': '/data/results/runs/input/propo.yaml',
            'vehicle_status_topic': '/vehicle/layout/status',
            'vehicle_profile_path': '/data/results/runs/vehicle/layout.yaml',
            'recording_status_topic': '/recording/status',
            'parameter_node_prefix': '/atd_', 'parameter_allowlist_json': '{}'}
        self.settings = {key: self.declare_parameter(key, value, fixed).value for key, value in defaults.items()}
        if not 1024 <= self.settings['port'] <= 65535 or not .1 <= self.settings['freshness_s'] <= 30:
            raise ValueError('Invalid bridge port or freshness_s')
        self.allowed = json.loads(self.settings['parameter_allowlist_json'])
        if not isinstance(self.allowed, dict) or any(not isinstance(k, str) or not isinstance(v, list)
                or any(not isinstance(name, str) for name in v) for k, v in self.allowed.items()):
            raise ValueError('parameter_allowlist_json must map nodes to parameter-name lists')
        self.cache = Cache(self.settings['freshness_s'])
        self.esc_trials = TrialStore()
        self.profiles = ProfileStore(self.settings['profile_path'])
        self.teleop_profiles = TeleopProfileStore(self.settings['teleop_profile_path'])
        self.propo_profiles = PropoProfileStore(self.settings['rc_profile_path'])
        self.vehicle_profiles = VehicleProfileStore(self.settings['vehicle_profile_path'])
        self.operation_lock = threading.RLock()
        self.identity = {'schema': 'atd.web_bridge.v1', 'instance_id': uuid.uuid4().hex,
                         'hostname': socket.gethostname(), 'node': self.get_fully_qualified_name(), 'pid': os.getpid()}
        self.subscriptions_ = []
        self.subscribe(LocalizationStatus, 'localization_topic',
                       lambda m: self.cache.put('localization', observation(m)))
        self.subscribe(Odometry, 'odometry_topic', self.odometry)
        self.subscribe(PoseStamped, 'pose_topic', lambda m: self.cache.put('pose', pose_value(m.pose, m.header)))
        self.subscribe(TwistStamped, 'velocity_topic', lambda m: self.cache.put('velocity', velocity_value(m.twist, m.header)))
        self.subscribe(Joy, 'joy_topic', lambda m: self.joy('normalized', m))
        self.subscribe(Joy, 'joy_raw_topic', lambda m: self.joy('raw', m))
        self.subscribe(String, 'joy_status_topic', self.status)
        self.subscribe(String, 'teleop_status_topic', self.teleop_status)
        self.subscribe(String, 'pico_status_topic', self.pico_status)
        self.subscribe(String, 'rc_status_topic', self.rc_status)
        self.subscribe(String, 'vehicle_status_topic', self.vehicle_status)
        self.subscribe(String, 'recording_status_topic', self.recording_status)
        self.joy_clients = {
            'configure': self.create_client(SetBool, self.settings['joy_node'] + '/configure'),
            'reload': self.create_client(Trigger, self.settings['joy_node'] + '/reload_profile')}
        self.teleop_clients = {
            'configure': self.create_client(SetBool, self.settings['teleop_node'] + '/configure'),
            'reload': self.create_client(Trigger, self.settings['teleop_node'] + '/reload_profile')}
        self.propo_clients = {
            'configure': self.create_client(SetBool, self.settings['rc_node'] + '/configure'),
            'reload': self.create_client(Trigger, self.settings['rc_node'] + '/reload_profile')}
        self.parameter_clients = {}

    def subscribe(self, kind, setting, callback):
        self.subscriptions_.append(self.create_subscription(kind, self.settings[setting], callback, qos_profile_sensor_data))

    def odometry(self, message):
        self.cache.put('pose', pose_value(message.pose.pose, message.header))
        velocity = velocity_value(message.twist.twist, message.header)
        velocity['frame'] = message.child_frame_id
        self.cache.put('velocity', velocity)

    def joy(self, key, message):
        value = {'axes': list(message.axes), 'buttons': list(message.buttons),
                 'frame': message.header.frame_id, 'stamp': stamp(message.header)}
        if key == 'raw':
            try:
                value['generation'] = int(message.header.frame_id.rsplit('_', 1)[1])
            except (ValueError, IndexError):
                value['generation'] = None
        self.cache.put(key, value)

    def status(self, message):
        try:
            value = yaml.safe_load(message.data)
            if isinstance(value, dict) and type(value.get('configuring')) is bool:
                self.cache.put('status', value)
        except (ValueError, yaml.YAMLError):
            self.get_logger().warning('Invalid joy/status YAML')

    def teleop_status(self, message):
        try:
            value = yaml.safe_load(message.data)
            if isinstance(value, dict) and type(value.get('configuring')) is bool:
                self.cache.put('teleop_status', value)
        except (ValueError, yaml.YAMLError):
            self.get_logger().warning('Invalid teleop/status YAML')

    def pico_status(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError('Pico status exceeds 64 KiB')
            value = yaml.safe_load(message.data)
            if isinstance(value, dict) and type(value.get('connected')) is bool and type(value.get('streaming')) is bool:
                self.cache.put('pico_status', value)
        except (ValueError, yaml.YAMLError):
            self.get_logger().warning('Invalid vehicle/pico/status YAML')

    def rc_status(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError('RC status exceeds 64 KiB')
            value = yaml.safe_load(message.data)
            if isinstance(value, dict) and type(value.get('configuring')) is bool:
                self.cache.put('rc_status', value)
        except (ValueError, yaml.YAMLError):
            self.get_logger().warning('Invalid rc/status YAML')

    def vehicle_status(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError('Vehicle status exceeds 64 KiB')
            value = yaml.safe_load(message.data)
            if not isinstance(value, dict) or type(value.get('profile_valid')) is not bool:
                raise ValueError('Invalid vehicle status')
            stamp = value.get('stamp')
            if type(stamp) not in (int, float) or not -1e12 <= stamp <= 1e12 or not math.isfinite(stamp):
                raise ValueError('Invalid vehicle status stamp')
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            value['source_age_s'] = max(0., age) if math.isfinite(age) and age >= -.05 else None
            self.cache.put('vehicle_layout_status', value)
        except (ValueError, yaml.YAMLError):
            self.get_logger().warning('Invalid vehicle/layout/status YAML')

    def recording_status(self, message):
        try:
            if len(message.data) > 65536:
                raise ValueError('Recording status exceeds 64 KiB')
            value = yaml.safe_load(message.data)
            if not isinstance(value, dict):
                raise ValueError('Invalid recording status')
            if type(value.get('schema_version')) is not int or value['schema_version'] != 1 \
                    or type(value.get('instance_id')) is not str or not value['instance_id'] \
                    or value.get('state') not in ('idle', 'starting', 'recording', 'stopping', 'error'):
                raise ValueError('Invalid recording status schema or state')
            stamp = value.get('stamp')
            if type(stamp) not in (int, float) or not 0 <= stamp <= 1e12 or not math.isfinite(stamp):
                raise ValueError('Invalid recording status stamp')
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            value['source_age_s'] = max(0., age) if math.isfinite(age) and age >= -.05 else None
            self.cache.put('recording_status', value)
        except (ValueError, yaml.YAMLError):
            self.get_logger().warning('Invalid recording/status YAML')

    def configure_propo(self, path, body):
        state = self.cache.propo_status()
        status = state.get('input')
        if not status or status['stale'] or not self.propo_profiles.matches_path(status):
            raise ValueError('プロポ node の新鮮な状態と profile_path の一致が必要です')
        reloading = path.endswith('/reload')
        if reloading:
            self.propo_profiles.require_configuring(state)
            if not self.propo_profiles.read()['saved']:
                raise ValueError('プロポの役割を保存してから再読込してください')
            request = Trigger.Request()
        else:
            if type(body.get('enabled')) is not bool:
                raise ValueError('enabled は bool で指定してください')
            if body['enabled'] is False:
                if not self.propo_profiles.read(state)['applied']:
                    raise ValueError('保存したプロポの役割を再読込してからイベント出力を再開してください')
                if not state.get('input_current') or not state.get('board') or not status.get('input_instance_id') \
                        or status['input_instance_id'] != state['board'].get('instance_id') \
                        or status.get('serial') != state['board'].get('serial'):
                    raise ValueError('同じ Pico からの新鮮で正常なプロポ入力が必要です')
            request = SetBool.Request(data=body['enabled'])
        client = self.propo_clients['reload' if reloading else 'configure']
        if not client.wait_for_service(timeout_sec=.5):
            raise ValueError('プロポ service 未接続。成功とは扱いません')
        try:
            result = self.wait(client.call_async(request))
            if not result.success:
                raise ValueError(result.message or 'プロポ node が操作を拒否しました')
            return {'applied': True, 'message': result.message,
                    'output_resumed': not reloading and body['enabled'] is False}
        finally:
            self.cache.invalidate('rc_status')

    def configure_teleop(self, path, body):
        state = self.cache.teleop_status()
        status = state.get('status')
        if not status or status['stale'] or not self.teleop_profiles.matches_path(status):
            raise ValueError('Teleop node の新鮮な状態と profile_path の一致が必要です')
        reloading = path.endswith('/reload')
        if reloading:
            self.teleop_profiles.require_configuring(state)
            if not self.teleop_profiles.read()['saved']:
                raise ValueError('操作割り当てを保存してから再読込してください')
            request = Trigger.Request()
        else:
            if type(body.get('enabled')) is not bool:
                raise ValueError('enabled は bool で指定してください')
            if body['enabled'] is False:
                profile_state = self.teleop_profiles.read(state)
                if not profile_state['applied']:
                    raise ValueError('保存した操作割り当てを再読込してから出力を再開してください')
                if not teleop_profiles.usable(profile_state['profile']):
                    raise ValueError('操舵とスロットル、または記録開始／停止ボタンを割り当ててください')
            request = SetBool.Request(data=body['enabled'])
        client = self.teleop_clients['reload' if reloading else 'configure']
        if not client.wait_for_service(timeout_sec=.5):
            raise ValueError('Teleop service 未接続。成功とは扱いません')
        try:
            result = self.wait(client.call_async(request))
            if not result.success:
                raise ValueError(result.message or 'Teleop node が操作を拒否しました')
            return {'applied': True, 'message': result.message,
                    'output_resumed': not reloading and body['enabled'] is False}
        finally:
            # A timeout is an unknown outcome; require a new node observation too.
            self.cache.invalidate('teleop_status')

    def wait(self, future):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(2.0):
            future.cancel()
            raise ValueError('ROS service timeout。結果は未確認です。自動再送しません')
        return future.result()

    def nodes(self):
        prefix = self.settings['parameter_node_prefix']
        return sorted(set((namespace.rstrip('/') + '/' + name) for name, namespace in self.get_node_names_and_namespaces()
                          if (namespace.rstrip('/') + '/' + name).startswith(prefix)))

    def client(self, node):
        if node not in self.nodes():
            raise ValueError('許可 prefix 内の稼働中 node を選択してください')
        if node not in self.parameter_clients:
            self.parameter_clients[node] = AsyncParameterClient(self, node)
        client = self.parameter_clients[node]
        if not client.wait_for_services(timeout_sec=.5):
            raise ValueError('ROS parameter services 未接続')
        return client

    def parameters(self, node):
        client = self.client(node)
        names = self.wait(client.list_parameters([], 0)).result.names
        if len(names) > 512:
            raise ValueError('Parameter count exceeds 512')
        if not names:
            return {'node': node, 'items': []}
        descriptors = self.wait(client.describe_parameters(names)).descriptors
        values = self.wait(client.get_parameters(names)).values
        items = []
        for name, descriptor, value in zip(names, descriptors, values):
            items.append({'name': name, 'value': parameter_value(value), 'type': value.type,
                'read_only': descriptor.read_only, 'allowed': name in self.allowed.get(node, []),
                'description': descriptor.description, 'additional_constraints': descriptor.additional_constraints,
                'integer_range': [{'from_value': r.from_value, 'to_value': r.to_value, 'step': r.step} for r in descriptor.integer_range],
                'floating_point_range': [{'from_value': r.from_value, 'to_value': r.to_value, 'step': r.step} for r in descriptor.floating_point_range]})
        return {'node': node, 'items': items}

    def apply_parameters(self, node, values):
        if not isinstance(values, dict) or not 1 <= len(values) <= 64:
            raise ValueError('values は1〜64件の parameter object を指定してください')
        current = {p['name']: p for p in self.parameters(node)['items']}
        parameters = []
        for name, value in values.items():
            item = current.get(name)
            if item is None or not item['allowed'] or item['read_only']:
                raise ValueError('許可されていない、または read_only parameter: ' + name)
            value = prepare_parameter_value(value, item)
            candidate = Parameter(name, value=value)
            if candidate.type_.value != item['type']:
                raise ValueError('Parameter type mismatch: ' + name)
            parameters.append(candidate)
        result = self.wait(self.client(node).set_parameters_atomically(parameters)).result
        if not result.successful:
            raise ValueError(result.reason or 'ROS node が変更を拒否しました')
        reflected = self.parameters(node)
        actual = {p['name']: p['value'] for p in reflected['items'] if p['name'] in values}
        return {'applied': actual == values, 'requested': values, 'values': actual, **reflected}

    def dispatch(self, path, method, body):
        with self.operation_lock:
            if path.startswith(('/teleop/', '/propo/', '/vehicle/')) and method in ('PUT', 'POST'):
                if type(body.get('instance_id')) is not str or body['instance_id'] != self.identity['instance_id']:
                    raise ValueError('Jetson bridge の接続先が変更されています。状態と設定を再取得してください')
            if path == '/live/status' and method == 'GET':
                return {**live_observations(self.cache, self.count_publishers(self.settings['localization_topic']),
                                            self.get_clock().now().nanoseconds * 1e-9),
                        'parameter_nodes': self.nodes(), 'topics': [self.settings[k] for k in ('odometry_topic', 'pose_topic', 'velocity_topic', 'localization_topic')]}
            if path == '/live/parameters' and method == 'POST':
                return self.apply_parameters(body.get('node'), body['values']) if 'values' in body else self.parameters(body.get('node'))
            if path == '/esc/status' and method == 'GET':
                return {'enabled': self.settings['esc_trials_enabled'], **self.esc_trials.status()}
            if path == '/esc/prepare' and method == 'POST':
                if not self.settings['esc_trials_enabled']:
                    raise ValueError('Jetson で bringup.sh esc-trial を起動してください')
                if body.get('instance_id') != self.identity['instance_id']:
                    raise ValueError('接続先が変わりました。試験状態を再取得してください')
                if set(body) != {'steps', 'instance_id'}:
                    raise ValueError('手順と接続先を指定してください')
                return {'enabled': True, **self.esc_trials.prepare(body['steps'])}
            if path == '/esc/cancel' and method == 'POST':
                return {'enabled': self.settings['esc_trials_enabled'], **self.esc_trials.cancel(body.get('id'))}
            if path == '/input/status' and method == 'GET':
                return self.cache.input_status()
            if path == '/teleop/status' and method == 'GET':
                return self.cache.teleop_status()
            if path == '/teleop/profile' and method == 'GET':
                return self.teleop_profiles.read(self.cache.teleop_status())
            if path == '/teleop/profile' and method == 'PUT':
                return self.teleop_profiles.write(body.get('profile'), body.get('revision'), self.cache.teleop_status())
            if path in ('/teleop/reload', '/teleop/configure') and method == 'POST':
                return self.configure_teleop(path, body)
            if path == '/propo/status' and method == 'GET':
                return self.cache.propo_status()
            if path == '/propo/profile' and method == 'GET':
                return self.propo_profiles.read(self.cache.propo_status())
            if path == '/propo/profile' and method == 'PUT':
                return self.propo_profiles.write(body.get('profile'), body.get('revision'), self.cache.propo_status())
            if path in ('/propo/reload', '/propo/configure') and method == 'POST':
                return self.configure_propo(path, body)
            if path == '/vehicle/status' and method == 'GET':
                return self.cache.vehicle_status()
            if path == '/vehicle/profile' and method == 'GET':
                return self.vehicle_profiles.read(self.cache.vehicle_status())
            if path == '/vehicle/profile' and method == 'PUT':
                return self.vehicle_profiles.write(body.get('profile'), body.get('revision'), self.cache.vehicle_status())
            if path == '/recording/status' and method == 'GET':
                return self.cache.recording_status()
            if path == '/input/profile' and method == 'GET':
                return self.profiles.read()
            if path == '/input/profile' and method == 'PUT':
                return self.profiles.write(body.get('profile'), body.get('revision'), self.cache.input_status())
            if path in ('/input/reload', '/input/configure') and method == 'POST':
                client = self.joy_clients['reload' if path.endswith('reload') else 'configure']
                if not client.wait_for_service(timeout_sec=.5):
                    raise ValueError('Joy service 未接続。成功とは扱いません')
                if path.endswith('reload'):
                    state = self.cache.input_status().get('status')
                    if not state or state['stale'] or state.get('configuring') is not True:
                        raise ValueError('新鮮な Joy 設定モード状態が必要です')
                    if not self.profiles.read()['saved']:
                        raise ValueError('入力配置を保存してから再読込してください')
                    if state.get('profile_path') != str(self.profiles.path):
                        raise ValueError('Joy node と bridge の profile_path が一致しません')
                    request = Trigger.Request()
                else:
                    if type(body.get('enabled')) is not bool:
                        raise ValueError('enabled は bool で指定してください')
                    request = SetBool.Request(data=body['enabled'])
                result = self.wait(client.call_async(request))
                if not result.success:
                    raise ValueError(result.message or 'Joy node が操作を拒否しました')
                # Await the next status publication before accepting another profile write.
                self.cache.invalidate('status')
                return {'applied': result.success, 'message': result.message, 'output_resumed': path.endswith('configure') and body['enabled'] is False}
            raise ValueError('Unsupported bridge route or method')


def handler(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def do_GET(self):
            self.handle_request()

        def do_POST(self):
            self.handle_request()

        def do_PUT(self):
            self.handle_request()

        def handle_request(self):
            nonce = self.headers.get('X-ATD-Request', '')
            payload = {'identity': node.identity, 'request_id': nonce}
            status = 200
            try:
                if not re.fullmatch('[0-9a-f]{32}', nonce):
                    raise ValueError('SSH adapter request identity required')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 65536:
                    raise ValueError('Request too large')
                if self.command != 'GET' and self.headers.get('Content-Type') != 'application/json':
                    raise ValueError('JSON content-type required')
                body = json.loads(self.rfile.read(length)) if length else {}
                if not isinstance(body, dict):
                    raise ValueError('JSON object required')
                payload['data'] = node.dispatch(self.path, self.command, body)
            except Exception as exc:
                payload['error'] = str(exc)
                status = 400
            try:
                data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            except ValueError:
                status = 503
                data = json.dumps({'identity': node.identity, 'request_id': nonce, 'error': 'Non-finite ROS sample'}).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
    return Handler


def main():
    rclpy.init()
    node = BridgeNode()
    server = ThreadingHTTPServer(('127.0.0.1', node.settings['port']), handler(node))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
