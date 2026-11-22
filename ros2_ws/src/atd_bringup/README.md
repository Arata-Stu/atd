# atd_bringup

Jetson 内でセンサ・入力・車両 I/O を起動する。`joy-drive` と `manual` は CH3 でプロポ／DualSense を切り替え、記録 manager は Joy・プロポの開始操作を待つ。

| launch | 内容 |
| --- | --- |
| `gazebo.launch.py` | Notebook の Gazebo が配信する `/clock` と左右画像で cuVSLAM ステレオ VO を起動する。[起動と確認](../../../docs/gazebo_monza.md#gazebo-の時刻で-cuvslam-を起動する) |
| `gazebo_closed_loop.launch.py` | 保存地図と走行ラインで閉ループ走行する。`atd gazebo closed-loop --rviz` で、走行中の GT と自己位置の推定をラインに重ねて見る。[起動と表示](../../../docs/gazebo_circuits.md#保存地図とステレオ画像で閉ループ走行する) |
| `bag_vslam.launch.py` | 記録済みステレオ入力から cuVSLAM で軌跡・ランドマークを生成する。Notebook では `atd replay` で RViz と bag 再生もまとめて起動できる。[使い方](../../../docs/gazebo_monza.md#記録済みのステレオ-bag-を-rviz--foxglove-で見る) |
| `d455.launch.py` | `atd_sensors/d455.launch.py` + `atd_vehicle_description/vehicle.launch.py`。bridgeなし |
| `localization.launch.py` | D455・車両 TF・入力一式に自己位置推定、planner、`atd_control` を加える。手動走行中の姿勢・前方軌道・計算舵角と、速度 FF＋PID、プロポ CH2、DualSense の正規化スロットルを監視する |
| `joy_drive.launch.py` | カメラなしの入力一式と `joy_drive`。プロポ許可時の DualSense 手動走行 |
| `autonomous.launch.py` | D455・自己位置・planner・Pure Pursuit・FF＋PID と専用 `auto_drive`。計算した指令を CH3 の許可中に Pico へ送り、追跡喪失時は停止を要求する。[校正と起動](../../../docs/bringup.md#低速前進の自動走行) |
| `manual.launch.py` | `d455` の内容と入力一式・`joy_drive`。bridge は1つ |

```bash
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh d455
./scripts/bringup.sh joy-drive
./scripts/bringup.sh manual
./scripts/bringup.sh --dry-run manual
```

`manual joy_drive:=false` は D455 と入力監視のみ。`localization` で DualSense を使う場合は `joy_drive:=true` を指定する。[初回の操作と切替](../../../docs/bringup.md#dualsense-で手動走行する)

設定TUIとカメラの要求初期値・実機照会は [atd_sensors](../atd_sensors/README.md) を参照する。`camera_config`（既定 `/data/results/runs/bringup/d455.yaml`）、`camera_serial` と各streamの引数を受け、cameraの子scopeへ渡す。`vehicle_config`、`vehicle_profile`（既定 `/data/results/runs/vehicle/layout.yaml`）は車両TFへ渡す。

manual では既存inputの `config`、`profile`、`joy_config`、`teleop`、`teleop_profile`、`teleop_config`、`propo`、`pico_config`、`pico_serial`、`pico_sensor_path`、`rc_config`、`rc_profile` も使える。監視・設定bridgeにも同じ `vehicle_profile` を渡す。`recording`（既定true）はmanager起動、`recording_profile`（manual既定mapping、車両校正はcalibration）は記録topicの選択、`recording_config`（既定空）は独自ROS parameter YAMLを指定する。managerはinput由来の1つだけで、別のrecording.launch.pyを重ねて起動しない。[記録手順](../../../docs/recording.md)を参照する。各includeを `GroupAction(scoped=True)` で分離し、`config` / `profile`等を別の子launchへ漏らさない。

vehicle の未知の寸法・取付姿勢を補完しない。camera_linkの車体取付TFは有効な設定があるときだけ車両packageが配信し、D455内部のframeは公式driverが配信する。カメラ・車両・入力の保存と再起動はそれぞれの契約に従う。

portable tests はfake ROS launch modulesで構成・scope・引数を確認する。ROS build/install・実camera・TF配信の結合検証は未実施。

`./scripts/bringup.sh esc-trial` は試験用 bridge を起動し、Jetson の端末で予約した手順を確認して実行する。[ESC 試験の手順](../../../docs/bringup.md#タミヤ-esc-の手順試験)
