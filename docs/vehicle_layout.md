# 車両配置と static TF

Notebook の Web UI の「車両設定」で車両寸法と D455 の取付位置を編集し、Jetson の `/data/results/runs/vehicle/layout.yaml` に保存する。位置は画面では mm、姿勢は度、保存値は m / rad とする。未計測の欄は空欄のまま保持する。

## 座標系と発行元

`base_link` の原点を左右後輪の回転中心の中点と定義する。x は前、y は左、z は上。原点は床面ではない。

| TF | 発行元・条件 |
| --- | --- |
| `base_link → rear_axle` | `atd_vehicle_description`。原点の定義として恒等変換 |
| `base_link → front_axle` | 同 node。ホイールベースを入力した場合だけ x 方向へ配置 |
| `base_link → camera_link` | 同 node。取付変換6値を入力し、カメラ取付TFを有効にした場合だけ発行 |
| `camera_link → camera_*_frame → camera_*_optical_frame` 等 | RealSense driver。内部の較正値・有効ストリームから配信 |

RealSense 側は `publish_tf=true` / `tf_publish_rate=0.0` で内部 static TF を発行する。車体側 node は内部カメラTFを重複発行しない。[公式 driver のTF実装](https://github.com/realsenseai/realsense-ros/blob/r/4.56.3/realsense2_camera/src/tfs.cpp)

`map` / `odom` の変換は自己位置推定が担当するため、この node では発行しない。未計測のカメラ取付位置を原点や identity で補完しない。車体寸法は配置の確認用であり、保存しただけで既存のライン生成設定・制御器の車両パラメータへ反映されるものではない。

## UI の操作

Jetson のホストで必要な package を build し、D455・車体TF・入力bridgeを起動する。

```bash
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh manual
```

Notebook の Docker 内で `atd-web` を起動し、Notebook の Chrome で `http://127.0.0.1:8766` を開く。Transfer の SSH 接続設定を済ませ、「車両設定」を開く。Notebook は SSH 経由の観測・設定 bridge を利用し、Jetson の DDS には参加しない。

1. ホイールベース・前後トレッド・車体外形・後輪軸から後端までの長さを実測して入力する。値の由来も未計測／CAD候補／実測で区別する。
2. カメラの位置 x/y/z と姿勢 roll/pitch/yaw を入力する。取付TFを発行するには6値すべてと有効化が必要。
3. 上面図・側面図で向き、原点、単位を確認し、Jetson に保存する。
4. 停車して利用中の処理を終了し、同じ bringup を再起動する。static TF は node 起動時に確定し、保存だけでは変化しない。
5. UIで「保存済み」と「適用中」が一致したことを確認する。node停止・通信断・古い状態では適用確認済みと扱わない。

`d455` profile だけで起動する場合はbridgeを含まないため、別ターミナルで `./scripts/bringup.sh bridge` も起動する。`manual` はbridgeを含み、追加起動は不要。profile ファイルを変更する場合は、TF とbridgeの `vehicle_profile` を同じ絶対パスへ揃える。

## 実測時の注意点

- ホイールベースは前後の回転軸中心間、トレッドは左右の回転中心間を測る。ボディや印刷板の幅とは区別する。
- カメラの z は後輪軸からの高さ。床から測る場合は、床から後輪軸までの高さを引く。
- 測定点は driver の `camera_link` 原点へ合わせる。筐体の中央や固定ねじをそのまま原点にしない。筐体の寸法図・光学系原点・driver の内部変換を照合する。
- 姿勢は右手系の roll（x軸）、pitch（y軸）、yaw（z軸）。車体軸とカメラの ROS 軸の取付変換を入力し、optical frame への90度回転を重ねない。

印刷フレームの CAD 外接寸法は 120 × 210.5 × 19.2 mm（CAD軸XYZ）。これは車体全体や後輪軸基準のカメラ位置を表さないため、自動で実測欄へ転記しない。[CADの測定根拠](hardware_cad.md#車両設定に使う寸法の参考)

## 実機での確認

再起動後、Jetson の Docker 内で確認する。

```bash
ros2 run tf2_ros tf2_echo base_link rear_axle
ros2 run tf2_ros tf2_echo base_link front_axle
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 topic echo /vehicle/layout/status --once
```

前輪・カメラのTFは対応する設定がないと発行しない。UI/APIの検証、設定読み取り、TFの発行、実車との寸法・姿勢一致は別の確認である。車両配置を変えた場合は、既存の地図やbagが以前の取付TFを持つことにも注意し、保存済みデータを自動で書き換えない。
