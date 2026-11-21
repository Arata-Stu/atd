"""Gazebo のステレオ解像度と D455 互換のシミュレーション用 IMU トピック。"""

RESOLUTIONS = {f"{w}x{h}": (w, h) for w, h in ((424, 240), (640, 480), (848, 480), (1280, 720))}
IMU_MODES = {"off": 0, "200": 200, "400": 400}

IMU_TOPIC = "/camera/camera/imu"
GYRO_TOPIC = "/camera/camera/gyro/sample"
ACCEL_TOPIC = "/camera/camera/accel/sample"
GYRO_FRAME = IMU_FRAME = "camera_gyro_optical_frame"
ACCEL_FRAME = "camera_accel_optical_frame"


def imu_streams(imu_hz):
    if type(imu_hz) is not int or imu_hz not in IMU_MODES.values():
        raise ValueError("IMU 周波数は 0、200、400 Hz から指定してください")
    if imu_hz == 0:
        return {}
    return {IMU_TOPIC: (IMU_FRAME, imu_hz), GYRO_TOPIC: (GYRO_FRAME, imu_hz),
            ACCEL_TOPIC: (ACCEL_FRAME, 200)}
