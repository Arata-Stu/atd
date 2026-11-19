# atd_recording

Joy / プロポの操作イベントで、Jetson 内の ROS topic を明示的に MCAP へ記録する。起動・再接続だけでは録画を開始しない。ファイル操作と native rosbag2 Recorder の開始・停止は、単一 worker で直列化する。

```bash
atd-build --packages-up-to atd_recording
ros2 launch atd_recording recording.launch.py profile:=mapping
# カメラ・IMU・Pico の計測用
ros2 launch atd_recording recording.launch.py profile:=calibration
# カメラ不要の入力確認用
ros2 launch atd_recording recording.launch.py profile:=inputs
```

通常は `atd_bringup` から起動する。`config:=/absolute/path.yaml` で独自の ROS parameter YAML を選べる。設定は起動時に読み込み、全 parameter を read-only とする。

| profile | 必須 topic | 任意の記録対象 |
| --- | --- | --- |
| mapping | 左右 IR Image / CameraInfo、`/tf_static` | RGB、IMU、動的 TF、Joy / プロポ / Pico / 車体設定の状態 |
| calibration | mapping の必須 topic、accel / gyro sample、Pico telemetry | TF、入力・状態 |
| inputs | カメラ不要。保存成功には記録メッセージが必要 | TF、Joy / プロポ / Pico / 車体設定の状態、起動中の D455 加速度・自己位置状態 |

`topics` に列挙した topic だけを購読する。後から出現する任意 topic は rosbag2 の discovery で取得する。自身の `/recording/status` は記録対象にできない。inputs の空 bag が自身の状態通知だけで成功することを防ぐ。

3つの標準プロファイルはいずれも、割り当て後の DualSense 操舵・スロットル（−1〜+1）を `/teleop/command` に、Pico の CH3、操作経路、サーボ・ESC の報告値（µs）と Joy の運転状態を `/vehicle/joy_drive/status` に記録する。後者は `atd_joy_drive` の best effort 配信を購読する。両方を収録するときは `joy-drive` または `manual` を起動し、Joy／プロポに割り当てたスイッチで記録を開始する。各信号には元の時刻と有効状態が残る。監視専用の `input` では、受信機の入力と基板経路を `/vehicle/pico/telemetry` に記録できる。CDC0 の出力パルス幅を収録するには手動走行 node の起動が必要になる。

## 入力と状態

各プロファイルは起動中の `/localization/status` を記録する。自己位置ノードが追跡中に報告する `speed_mps` は、そのステレオ観測時刻における車体前方を正とする速度 [m/s]。D455 の加速度には `/camera/camera/accel/sample`、統合 IMU 設定では `/camera/camera/imu` と、取付姿勢を特定する `/tf_static` を使う。地図を使わずに記録し、走行後のステレオ VO で速度を得る場合は、左右 IR 画像と CameraInfo を含む `mapping` または `calibration` を選ぶ。

- `/teleop/events`、`atd_interfaces/msg/OperatorEvent`、`source=joy`
- `/rc/events`、同型、`source=propo`
- `RECORD_START=1` / `RECORD_STOP=2`。イベントは volatile / reliable。
- `/recording/status`、`std_msgs/msg/String`、YAML、5 Hz、reliable。観測用であり、UI からの走行・録画指令 API は持たない。

状態は `idle → starting → recording → stopping → idle/error`。START の重複は録画を増やさない。STARTING 中の STOP は開始を取り消し、Writer の開始処理が返った後に閉じる。STOPPING 中の START は拒否する。停止中の flush を callback / timer で待たない。

イベントの source、action、ROS stamp の形式・年齢を検査する。zero / future / stale stamp、source ごとの逆順・重複を拒否する。同じ stamp の START / STOP は STOP を優先する。STOP と同時刻以前の遅れて届いた START は別 source でも再開させない。新しい明示 START が必要。ROS 時刻を使う実入力向けで、`use_sim_time=true` は受け付けない。

状態 YAML は `schema_version,instance_id,stamp,state,profile,bag_path,elapsed_s,started_at,message,last_request,last_result` を持つ。inactive の `bag_path` は空、開始前の時刻 / 経過時間は null。`elapsed_s` は monotonic clock で算出し、停止処理に入った時点で固定する。`last_request` は `{source,action,stamp,result,reason}`、`last_result` は `{bag_path,success,reason,elapsed_s,closed_at,size_bytes,message_count}`。サイズと件数が検証できない場合は null。既存 bag を自動削除しない。

`last_request.reason`:

```text
start_accepted stop_accepted already_starting already_recording already_idle
already_stopping busy_stopping invalid_source invalid_action invalid_stamp
zero_stamp stale_request future_request out_of_order_request duplicate_request
stop_has_priority shutting_down
```

`last_result.reason`:

```text
recording_completed start_cancelled missing_required_topics low_disk_space
storage_unavailable start_failed write_failed metadata_missing metadata_invalid
mcap_missing empty_bag required_topic_missing required_topic_empty
required_type_mismatch required_publishers_lost manager_shutdown
```

## 保存と検査

既定は `/data/record/rosbags/bag_<instance>_<session>/`。URI を再利用しない。record root の `.atd_recording.lock` を `flock` し、同じ保存先の manager 多重起動を拒否する。lock file は inode を維持するため削除しない。部分 bag も保持する。

開始前に必須 publisher / 型と空き容量を待つ。記録中も容量、必須 publisher の存在、native executor を監視し、異常時は閉じる。停止後は `metadata.yaml` を再読込し、MCAP storage、記録件数、明示 topic と型、必須 topic の非ゼロ件数、参照する MCAP の regular file / 非ゼロサイズ / 前後 magic を検査する。重複・絶対パス・親ディレクトリ参照・symlink を拒否する。失敗は部分 bag の成功扱いにしない。

この検査は停止後の保存構造確認であり、全 MCAP record の decode / CRC 検査、画像時刻差・欠落・Pico 信号の妥当性、実走行の正しさまでは証明しない。プロセス強制終了・電源断では flush を保証できない。通常 shutdown は worker が Writer を閉じるまで待つ。

| parameter | 既定 | 用途 |
| --- | --- | --- |
| profile_name | mapping | mapping / calibration / inputs の表示名 |
| record_root | /data/record/rosbags | 絶対保存先 |
| topics | 設定 YAML | 明示 topic 配列、空は不可 |
| required_topics / required_types | 設定 YAML | 同順の配列。必須 topic と完全な ROS 型名 |
| best_effort_topics | 設定 YAML | depth 10、best effort / volatile |
| transient_local_topics | /tf_static | depth 100、reliable / transient local |
| max_cache_bytes | 67108864 | rosbag2 message cache、64 MiB |
| min_free_bytes | 1073741824 | 空き容量の下限、1 GiB |
| event_timeout_s | 1.0 | 操作イベントの有効期間 |
| start_timeout_s | 10.0 | 必須 publisher を待つ上限 |
| storage_check_period_s | 0.5 | 記録中の容量・publisher 確認周期 |

画像 / CameraInfo は ROS 側で best effort / volatile、`/tf_static` は reliable / transient local を指定する。DDS の通信設定とは別である。保存容量下限は cache より十分大きくし、運用時のディスク性能・使用量に合わせる。

## Native API と検証範囲

ROS 2 Lyrical の公式 rosbag2 [commit 4a559f8](https://github.com/ros2/rosbag2/tree/4a559f8aaa867d4fd298746c21704385f73aa450) を確認した。Writer / StorageOptions / RecordOptions を渡す [Recorder ctor](https://github.com/ros2/rosbag2/blob/4a559f8aaa867d4fd298746c21704385f73aa450/rosbag2_transport/include/rosbag2_transport/recorder.hpp) を使い、コンストラクタからの自動開始を避ける。`record()` の返却と on-start callback を確認して recording に遷移する。公開 `is_recording()` API は存在しないため使わない。on-start callback 内では atomic flag を立てるだけにする。

`stop()` に加えて保持した Writer を明示 close し、開始途中の例外でも writer を閉じる。native Recorder は manager と別 executor で購読し、stop flag と期限付き `spin_once` で終了する。上流の private services はセッション固有の `/atd_recording_internal/...` に隔離する。状態機械を迂回するため、これらを外部から呼び出さない。

ROS のないホストでは以下を実行できる。

```bash
c++ -std=c++20 -Wall -Wextra -Wpedantic -Werror \
  -I ros2_ws/src/atd_recording/include \
  ros2_ws/src/atd_recording/test/test_core.cpp -o /tmp/atd-recording-test
/tmp/atd-recording-test
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s ros2_ws/src/atd_recording/test -p test_contract.py
```

portable tests は要求の鮮度・重複・STOP 優先・状態遷移、metadata 集計、MCAP envelope / パス異常、profile / launch 契約を確認する。native ROS API のビルド、実 MCAP 記録、Docker / Jetson / D455 / Pico の挙動は別に対象環境で検証する。
