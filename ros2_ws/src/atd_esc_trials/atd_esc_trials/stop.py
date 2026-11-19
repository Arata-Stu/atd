"""試験の wait_stopped ステップで使う停止根拠。"""
import math
import os
import select
import sys
import termios
import time


class VisualStop:
    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def begin(self):
        # 過去の手順中の先行入力で次の停止確認を通過させない。
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
        print('制動を維持しています。車両の停止を目視したら STOP を入力してください。', flush=True)

    def evidence(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            line = os.read(sys.stdin.fileno(), 4096)
            if not line:
                raise RuntimeError('停止確認用の端末が閉じられました')
            if line.strip() == b'STOP':
                return 'Jetson 端末で車両の停止を目視確認'
        return None


class StopEstimate:
    """同一追跡区間で、停止待ち開始後の新しい観測が連続した場合だけ使用する。"""

    def begin(self, now_ns):
        self.begin_ns = now_ns
        self.first = None
        self.last = None
        self.key = None

    def observe(self, msg, now_ns, received_s):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        key = (msg.session_id, msg.epoch, msg.map_id)
        if (msg.state != 'tracking' or not msg.session_id or not msg.map_id
                or not self.begin_ns <= stamp <= now_ns or now_ns - stamp > 250_000_000
                or not math.isfinite(msg.speed_mps) or abs(msg.speed_mps) > 0.03
                or not math.isfinite(msg.yaw_rate_rps) or abs(msg.yaw_rate_rps) > 0.05):
            self.first = self.last = None
            return None
        if self.last and self.key == key and stamp == self.last[0]:
            return None
        if (self.last is None or self.key != key or not 0 < stamp - self.last[0] <= 250_000_000
                or received_s - self.last[1] > 0.25):
            self.first = (stamp, received_s)
        self.last = (stamp, received_s)
        self.key = key
        if stamp - self.first[0] >= 500_000_000 and received_s - self.first[1] >= 0.5:
            return (f'localization で停止を推定 (vx={msg.speed_mps:.3f} m/s, '
                    f'wz={msg.yaw_rate_rps:.3f} rad/s, epoch={msg.epoch})')
        return None


class LocalizationStop:
    def __enter__(self):
        import rclpy
        from atd_interfaces.msg import LocalizationStatus
        from rclpy.signals import SignalHandlerOptions
        self.ros = rclpy
        self.ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        try:
            self.node = self.ros.create_node('atd_esc_trial_stop_observer')
            self.node.create_subscription(LocalizationStatus, '/localization/status', self.receive, 1)
        except BaseException:
            self.ros.try_shutdown()
            raise
        self.estimate = StopEstimate()
        return self

    def __exit__(self, *_):
        self.node.destroy_node()
        self.ros.try_shutdown()

    def begin(self):
        self.estimate.begin(self.node.get_clock().now().nanoseconds)
        self.confirmation = None

    def receive(self, msg):
        self.confirmation = self.estimate.observe(msg, self.node.get_clock().now().nanoseconds, time.monotonic())

    def evidence(self):
        if not self.ros.ok():
            raise RuntimeError('停止推定用の ROS context が終了しました')
        self.ros.spin_once(self.node, timeout_sec=0)
        return self.confirmation
