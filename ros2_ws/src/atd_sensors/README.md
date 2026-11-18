# atd_sensors

D455 の起動要求を保存し、Jetson の librealsense SDK で実機対応を確認してから公式 ROS driver を起動する。SDK の Python binding は使わない。`atd_realsense_probe` は C++20 の照会 CLI で、stream の列挙と `config.can_resolve()` の検査だけを行い、pipeline を start しない。

## 起動要求と実機対応

| stream | 未保存時の要求 |
| --- | --- |
| infra1 / infra2 | 両方 ON、640×480、90 fps、Y8 |
| color | ON、640×480、30 fps、RGB8 |
| depth | OFF（設定値640×480、30 fps、Z16） |
| accel / gyro | 両方 ON、200 / 400 Hz、MOTION_XYZ32F |

この表は要求する初期値であり、接続済み個体の対応一覧ではない。SDK、firmware、接続状態によっては要求値が対応しない。特に accel 200 Hz も実機照会で検査し、未対応の場合に別の周波数へ自動変更しない。stream ごとの存在確認に加え、全ON構成の同時成立を SDK で検査する。検査成功は実際の配信速度・欠落・USB帯域の実測とは区別する。

ROS driver 4.56.3 は左右 Infra に1つの `depth_module.infra_profile` を使う。左右の ON/OFF は独立し、両方 ON の解像度とFPSは共通とする。IMU は既定 `unite_imu_method=0` で生の accel / gyro を別々に配信する。1または2で統合する場合は両方を ON にする。全stream OFF の直接launchはSDK照会もdriver起動も行わない。

## 操作

```bash
# Jetson の起動済み ATD Docker を使うホスト入口
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh d455
./scripts/bringup.sh manual
```

対話端末かつlaunch引数なしなら、実機の選択→stream ON/OFF・対応モード選択→保存して起動へ進む。fzf がなければ番号選択を使う。対応モードが分からない状態では候補を作らない。複数のD455はserialで選び、保存済みserialが接続されていない場合も別の個体を自動選択しない。

保存先は `/data/results/runs/bringup/d455.yaml`。`camera_config:=絶対パス` で変更できる。保存ファイルはJSON互換のYAMLで、通常のYAMLも `python3-yaml` で読める。C++ SDK CLI、保存用Pythonモジュール、ROS launch は別の処理であり、設定保存だけでは配信は開始しない。

```bash
# 保存設定・指定値で直接起動（非TTYでも可）
./scripts/bringup.sh d455 camera_serial:=SERIAL enable_depth:=false
# 引数があるときも設定TUIを開く
./scripts/bringup.sh manual --configure camera_config:=/data/results/runs/bringup/d455.yaml
# SDK・ROSを使わない実行予定の表示
./scripts/bringup.sh --dry-run d455
```

単独の ROS launch は `ros2 launch atd_sensors d455.launch.py`。`config` / `serial` と `enable_infra1`、`enable_infra2`、`enable_color`、`enable_depth`、`enable_accel`、`enable_gyro`、`infra_profile`、`color_profile`、`depth_profile`、`accel_fps`、`gyro_fps`、`unite_imu_method` を受ける。profile は `640x480x90` の形式。空の引数は保存設定を使う。保存ファイル未作成の既定pathだけは要求初期値を使い、明示した他の欠損pathは失敗させる。

SDKの列挙と検査は、Jetson のDocker内で次のように確認できる。

```bash
ros2 run atd_sensors atd_realsense_probe --list
ros2 run atd_sensors atd_realsense_probe --serial SERIAL --validate --stream infra1:640:480:90 --stream infra2:640:480:90
```

namespace/name は `camera/camera`、topic は `/camera/camera/infra1/...` 等、基準frameは `camera_link`。D455内部のstatic TFは公式driverの `publish_tf=true` / `tf_publish_rate=0.0` に任せ、車両への取付TFは `atd_vehicle_description` が担当する。

## 実装と検証範囲

Docker が固定する SDK v2.56.3 / ROS driver r/4.56.3 を使う。新たなSDKの取得やインストールはこのpackageから行わない。公式の [SDK config API](https://github.com/realsenseai/librealsense/blob/v2.56.3/include/librealsense2/hpp/rs_pipeline.hpp)、[stream profile API](https://github.com/realsenseai/librealsense/blob/v2.56.3/include/librealsense2/hpp/rs_sensor.hpp)、[driver launch](https://github.com/realsenseai/realsense-ros/blob/r/4.56.3/realsense2_camera/launch/rs_launch.py) と [serial handling](https://github.com/realsenseai/realsense-ros/blob/r/4.56.3/realsense2_camera/src/realsense_node_factory.cpp) を参照した。

portable tests は設定・不正値・SDK応答・TUI・launch構造をfakeの照会／ROSモジュールで検証する。ホストにはROS・SDK・カメラがないため、C++ SDK CLIの対象SDKとの実build、ROS packageのbuild/install、実USB列挙、組合せ検査、実配信と実TFの検証は別途必要。照会CLIはセンサを開始しないが、driverと同時に実行せず、停止後に設定を変更する。
