# Jetson の bringup

`scripts/bringup.sh` で ROS 起動 profile を選び、Jetson の起動済み ATD Docker 内で実行する。先に `scripts/dev.sh` でコンテナを起動し、必要な package を build する。

```bash
# Jetson ホスト、プロジェクト直下
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh
```

引数なしでは fzf（未導入時は番号）で profile を選ぶ。D455 を含む profile では、その後ストリーム設定の TUI を開く。

| profile | 起動する処理 | ROS launch |
| --- | --- | --- |
| `manual` | D455、車体static TF、Joy・Pico入力、役割処理、観測・設定bridge | `atd_bringup manual.launch.py` |
| `d455` | D455 と車体static TF | `atd_bringup d455.launch.py` |
| `input` | Joy・Pico入力、役割処理、観測・設定bridge | `atd_web_bridge input.launch.py` |
| `bridge` | 観測・設定bridgeのみ | `atd_web_bridge bridge.launch.py` |

`manual` は手動走行時のセンサ・入力監視をまとめる名称で、車両出力や rosbag 記録は開始しない。プロポの基本走行は引き続き Pico 内で完結する。全 profile は Jetson ローカル専用。Notebook は Web UI と解析を担当し、車載 DDS に参加しない。

## D455 の TUI

```bash
./scripts/bringup.sh manual          # TUIで設定してからまとめ起動
./scripts/bringup.sh d455            # TUIで設定、D455・車体TFだけ起動
./scripts/bringup.sh --configure manual camera_serial:=SERIAL
./scripts/bringup.sh --dry-run manual # 実行予定と初期設定の確認。Notebookでも可
```

初期設定は次のとおり。保存済み設定がある場合はそれを編集する。

| ストリーム | 初期値 | TUIで変更する項目 |
| --- | --- | --- |
| Infra1 / Infra2 | 各 ON、640 × 480 @ 90 Hz、Y8 | 各ON／OFF、共通の解像度・FPS |
| RGB | ON、640 × 480 @ 30 Hz、RGB8 | OFF または解像度・FPS |
| Depth | OFF | OFF または解像度・FPS、Z16 |
| 加速度 | ON、200 Hz | OFF または対応Hz |
| ジャイロ | ON、400 Hz | OFF または対応Hz |

TUI は Jetson Docker 内の librealsense SDK で接続中の D455 を列挙し、選んだ機器が報告するモードを候補にする。複数台では serial を選択する。各候補の対応だけでなく、保存前・起動前にストリームの同時構成を SDK の `config.can_resolve` で検査する。要求した初期値や保存済み値が実機・firmware・USB接続で利用できなければ理由を表示して止まり、別のHzや解像度へ黙って変更しない。カメラを他の node / Viewer で使用したまま設定しない。

設定は `/data/results/runs/bringup/d455.yaml` に保存する。各ストリームは OFF にでき、無効にしたストリームのtopicは出ない。全OFFの直接起動ではSDK照会とdriver起動を省き、カメラ未接続でも車体TFなどを起動できる。IMUは既定で加速度・ジャイロの別topicを配信し、結合は行わない（`unite_imu_method=0`）。カメラの内部 static TF は RealSense driver が担当する。

対応モードの列挙・同時構成検査の実装と、実カメラでの起動・実効FPS・画像欠落・IMU周期の確認は別である。TUI は実機のSDK応答が必要であり、`--dry-run` は TUI・SDK・ROS を実行しない。

## 引数と保存設定による直接起動

```bash
./scripts/bringup.sh --list
./scripts/bringup.sh input
./scripts/bringup.sh bridge
./scripts/bringup.sh manual camera_config:=/data/results/runs/bringup/d455.yaml
./scripts/bringup.sh d455 enable_color:=false enable_depth:=false
./scripts/bringup.sh d455 infra_profile:=640x480x90 accel_fps:=200 gyro_fps:=400
./scripts/bringup.sh manual pico_serial:=PICO_SERIAL
./scripts/bringup.sh manual vehicle_profile:=/data/results/runs/vehicle/layout.yaml
./scripts/bringup.sh --dry-run        # 従来どおりinputの実行予定
```

TTY で D455 profile を選び、launch引数を付けなかった場合に設定TUIへ進む。`name:=value` を付けた場合と非対話端末では、TUIを経由せず保存設定と上書き値で起動する。引数がある場合も `--configure` を付ければ TUI を開く。非対話では profile の明示が必要で、`--configure` は使えない。

主な D455 引数は `camera_config`、`camera_serial`、`enable_infra1` / `enable_infra2` / `enable_color` / `enable_depth` / `enable_accel` / `enable_gyro`、`infra_profile` / `color_profile` / `depth_profile`（`幅x高さxFPS`）、`accel_fps` / `gyro_fps`、`unite_imu_method`。`vehicle_config` / `vehicle_profile` は車体TF用。詳細は [atd_sensors](../ros2_ws/src/atd_sensors/README.md) と [atd_bringup](../ros2_ws/src/atd_bringup/README.md)を参照する。

`manual` は既存 `input` の引数も受ける。`profile`・`teleop_profile`・`rc_profile` が入力の永続設定、`joy_config`・`teleop_config`・`pico_config`・`rc_config` が各 node の起動設定、`config` がbridge設定。`pico_serial` / `pico_sensor_path` で Pico を選び、`teleop:=false` / `propo:=false` で各入力経路を除外する。`bridge` はbridge用の `config` と `vehicle_profile` を受ける。[bridge package](../ros2_ws/src/atd_web_bridge/README.md)

`d455` はbridgeを含まないため、UIから車体配置を編集する場合は別ターミナルで `bridge` を起動するか、`manual` を使う。`manual` はbridgeを1つだけ起動する。`input` / `bridge` と重複して起動しない。

## 車両の配置設定

`atd_vehicle_description` が `base_link`（後輪軸中央）から車軸・D455取付の static TF を発行する。Notebook UIの「車両設定」で mm / 度を入力し、Jetsonへ保存する。保存と適用は別で、TF node の再起動後に反映される。未計測値は補完しない。[測定・UI・TFの手順](vehicle_layout.md)

起動はフォアグラウンドで行い、終了は Ctrl+C。Docker の準備、ROS package の build、node 起動、設定保存、再起動による適用、実機確認はそれぞれ分けて扱う。

## profile の追加方針

日常の入口は `scripts/bringup.sh`、一覧は `scripts/lib/bringup.sh`、構成の組み合わせは `atd_bringup`、各 node の実装・launch・config は担当 package 内に置く。新しい profile は実装済みの launch・引数・対象ホストと対応させ、一覧・操作説明・起動テストを更新する。未実装の自己位置推定・自律走行 profile を先に追加しない。
