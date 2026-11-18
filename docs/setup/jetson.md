# Jetson Orin Nano のセットアップ

Jetson Orin Nano 8GB（`aarch64` / `arm64`）に JetPack 7.2 を導入し、初回ログインとネットワーク接続を済ませた状態から ATD を準備する。
共通操作は [セットアップ入口](README.md)、PC は [ノート PC 手順](pc.md)を参照する。
JetPack のフラッシュ・OS 移行はこのスクリプトの対象外である。[NVIDIA 公式 Quick Start](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html)

## 1. JetPack・SSD の確認

通常のログインユーザーでプロジェクト直下から診断する。

```bash
./scripts/setup.sh --action check
```

Jetson / Tegra の機種・BSP マーカー、Ubuntu、GPU・Docker・CLI、保存先の空き容量を確認する。
CPU が arm64 という理由だけで Jetson と扱う構成にはしない。
必要な場合だけ `--host jetson` で手動上書きする。

未設定時の記録先・結果先は実際の clone 配下の `data/record`・`data/results`。標準 clone 位置 `~/workspaces/atd` なら `~/workspaces/atd/data/{record,results}` を使い、Git 管理対象外の永続資産として保持する。1TB SSD 上で使う場合は、その filesystem がホストで mount 済みであることを確認する。
setup はディスクの初期化、パーティション変更、root filesystem の移行をしない。
記録先・解析結果先はそれぞれ存在し、通常ユーザーが読書きできることを確認する。

GPU driver や JetPack SDK の不足は、導入済み BSP に対応する NVIDIA の手順で補う。
PC 向け GPU driver の導入手順で Jetson の driver を置き換えない。

## 2. ホスト準備と Docker

```bash
./scripts/setup.sh
```

メニューから tools、Docker、NVIDIA runtime、apt 版 Isaac ROS CLI、保存先、workspace を準備する。
JetPack に Docker / Toolkit が含まれていて利用できる場合は、既存導入を使う。
CLI の apt repository は `release-5.0` / `noble-jetpack` を使用する。ホストに ROS や librealsense を重複導入しない。[Isaac ROS 公式手順](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#configure-isaac-ros-apt-repository)

docker グループへの追加は、実行中の setup や親シェルには即時反映されない。SSH の再接続や再ログイン後、必要な段階から再実行する。
runtime の設定変更で Docker の再起動が必要な場合は、既存のコンテナで作業が動いていないことを確認してから反映する。
保存先は storage の対話入力、または `--record-dir` / `--result-dir` で mount 済み SSD 等の絶対パスを指定する。

## 3. D455 と DualSense

```bash
./scripts/setup.sh --action devices
```

### D455

- カメラを外した状態で、ホスト側の udev 規則を準備する。既定の規則は librealsense `v2.56.3` を参照する。
- 規則の反映後、D455 を USB 3 対応のポート・ケーブルへ接続する。
- SDK / ROS driver はコンテナ内の `realsense455` layer を使う。firmware の書換えは setup が自動実行しない。
- 現在の SDK / driver は `v2.56.3` / `r/4.56.3`。Isaac ROS 5 の公式 firmware 基準は `5.16.0.1` であり、実機の組合せを確認する。[公式 RealSense 手順](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/sensors/realsense_setup.html)

IR・RGB・IMU・Depth は利用候補として残す。全ストリームの常時同時起動・記録を必須にはせず、必要な組合せの取得・時刻・負荷を対象機で確認する。

### DualSense

- Bluetooth daemon とペアリング情報は Jetson ホスト側で管理する。
- DualSense をペアリングモードにし、ホスト側で Bluetooth 接続する。接続済みの表示と、実際の入力取得は別に確認する。
- ROS Joy node の入力取得はコンテナ側で行う。`joy` は SDL の入力を使うため、`/dev/input/js0` の存在だけでなく input event デバイスと一般ユーザーのアクセス権も確認する。[公式 joy 資料](https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md)

独自 firmware、ESC・サーボへの出力、手動 / 自律の切替・停止仕様は後工程で検討する。この段階では入力デバイスの確認までを扱う。

## 4. ROS workspace・初回 build・通常起動

Jetson 自身の ROS workspace の場所を指定する。未設定時は実際の clone 配下の `ros2_ws`、標準配置なら `~/workspaces/atd/ros2_ws` を使う。Notebook の workspace とは別の実体で、`build/` / `install/` を共有しない。旧配置の保存済み指定は自動変更せず、workspace / storage 操作で選び直す。

| 用途 | Jetson ホスト | コンテナ内 |
| --- | --- | --- |
| ROS workspace | 指定したパス。未設定なら `<ATD checkout>/ros2_ws` | `/workspaces/isaac_ros-dev`（読書き可） |
| 自作 ROS package の編集元（別 workspace を使う場合） | `<ATD checkout>/ros2_ws/src` | `/workspaces/isaac_ros-dev/src/atd`（読書き可） |
| プロジェクト設定参照 | `<ATD checkout>` | `/workspaces/atd`（readonly） |

```bash
./scripts/setup.sh --action workspace
./scripts/setup.sh --action verify
./scripts/setup.sh --action build
```

workspace は対話入力した場所に CLI 設定を準備し、`config/workspace.env` の `ATD_JETSON_ROS_WS` に保存する。別の workspace を使う場合は `src/atd` の mount point も用意する。SSD 上へ置く場合は `--workspace /mnt/ssd/atd_ws` のように直接指定もできる。自作 package の編集元はリポジトリの `ros2_ws/src/` とする。
リポジトリの `ros2_ws` を workspace に選ぶ場合は、その `src` がコンテナの `/workspaces/isaac_ros-dev/src` に対応し、追加 source mount は行わない。
`ISAAC_ROS_WS` は `dev.sh` が設定するため、ホストの `.bashrc` へ手動 export しない。設定の詳細と既存 CLI 設定との衝突時の扱いは [共通手順](README.md#ros-workspace-のパス)を参照する。
`./scripts/tools/docker.sh` で項目別診断を保存し、`./scripts/tools/docker.sh --action logs` で Jetson の過去の結果を確認できる。
初回 build は `isaac_ros → realsense455 → atd` の順で行う。
verify は Docker 接続、CLI の Docker モード、`--gpus all` による `/bin/true` のコンテナ起動までを確認する。CUDA 演算やセンサ取得は別途検証する。
Docker 起動時には不足依存の導入と Fast DDS の loopback 設定を `.sh` で自動適用する。カメラ・録画・車両ノードは自動起動しない。
次回は `./scripts/setup.sh --action start` または `./scripts/dev.sh` を使う。

コンテナ内で、まず ROS / workspace / DDS と入力デバイスの列挙を確認できる。

```bash
printenv ROS_DISTRO ISAAC_ROS_WS RMW_IMPLEMENTATION
ros2 run joy joy_enumerate_devices
```

期待する ROS / workspace / RMW は `lyrical` / `/workspaces/isaac_ros-dev` / `rmw_fastrtps_cpp`。
列挙成功は、手動介入・実車停止の確認ではない。実機に応じて、一般ユーザーからの GPU・D455・入力アクセスを検証する。

Jetson 自身では通常 Chrome を開かない。実走行の処理は Jetson 内で完結させ、PC 向けの観測情報は bridge 経由で提供する予定。

入力・観測は起動済みコンテナに対して、Jetson ホストの別端末から明示的に起動する。

```bash
./scripts/build.sh --packages-up-to atd_web_bridge
./scripts/bringup.sh input
# 入力・役割nodeが別途起動済みならbridgeのみ
./scripts/bringup.sh bridge
```

引数なしでは起動 profile を選べる。現在は `input` / `bridge` のみで、車両出力・bag 記録は開始しない。Pico が複数ある場合は `./scripts/bringup.sh input pico_serial:=<ID>` で選択する。Notebook の UI と SSH 接続、profile の操作は [Web アプリ](../../apps/web/README.md)と [bringup](../bringup.md)を参照する。実 DDS・USB・SSH の結合と実車確認は別途行う。
