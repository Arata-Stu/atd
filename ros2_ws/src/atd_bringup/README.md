# atd_bringup

Jetson 内の実装済みlaunchを組み合わせる。`d455` は D455 と車両static TF、`manual` はそれに既存の入力一式を加える。車両への走行出力や rosbag 記録は自動開始しない。manualは記録managerを待機させ、Joy・プロポの操作イベントで録画する。

| launch | 内容 |
| --- | --- |
| `d455.launch.py` | `atd_sensors/d455.launch.py` + `atd_vehicle_description/vehicle.launch.py`。bridgeなし |
| `manual.launch.py` | 上記 + `atd_web_bridge/input.launch.py`。bridgeはinput由来の1つ |

```bash
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh d455
./scripts/bringup.sh manual
./scripts/bringup.sh --dry-run manual
```

設定TUIとカメラの要求初期値・実機照会は [atd_sensors](../atd_sensors/README.md) を参照する。`camera_config`（既定 `/data/results/runs/bringup/d455.yaml`）、`camera_serial` と各streamの引数を受け、cameraの子scopeへ渡す。`vehicle_config`、`vehicle_profile`（既定 `/data/results/runs/vehicle/layout.yaml`）は車両TFへ渡す。

manual では既存inputの `config`、`profile`、`joy_config`、`teleop`、`teleop_profile`、`teleop_config`、`propo`、`pico_config`、`pico_serial`、`pico_sensor_path`、`rc_config`、`rc_profile` も使える。監視・設定bridgeにも同じ `vehicle_profile` を渡す。`recording`（既定true）はmanager起動、`recording_profile`（manual既定mapping、車両校正はcalibration）は記録topicの選択、`recording_config`（既定空）は独自ROS parameter YAMLを指定する。managerはinput由来の1つだけで、別のrecording.launch.pyを重ねて起動しない。[記録手順](../../../docs/recording.md)を参照する。各includeを `GroupAction(scoped=True)` で分離し、`config` / `profile`等を別の子launchへ漏らさない。

vehicle の未知の寸法・取付姿勢を補完しない。camera_linkの車体取付TFは有効な設定があるときだけ車両packageが配信し、D455内部のframeは公式driverが配信する。カメラ・車両・入力の保存と再起動はそれぞれの契約に従う。

portable tests はfake ROS launch modulesで構成・scope・引数を確認する。ROS build/install・実camera・TF配信の結合検証は未実施。
