# Notebook（x86_64）のセットアップ

Ubuntu 24.04 / `x86_64` と RTX 4060 を対象とする。apt / Docker でのアーキテクチャ名は `amd64`。
共通操作は [セットアップ入口](README.md)を使い、Jetson は [Jetson 手順](jetson.md)を参照する。
通常は自動判定し、必要なら `./scripts/setup.sh --host notebook`（`--host pc` と同義）で Notebook の設定を明示する。

## 1. ホストの GPU と保存先

通常のログインユーザーでプロジェクト直下から診断する。

```bash
./scripts/setup.sh --action check
```

確認するのは Ubuntu の版、GPU / NVIDIA driver、空き容量、Docker と CLI の状態である。
Isaac ROS 5 の公式 x86 対応表は Ubuntu 24.04、Ampere 以降、GPU メモリ 8GB 以上、Driver 595 以上、CUDA 13.2 以上を掲載している。
RTX 4060 の実際の VRAM と導入済み driver を確認し、driver の変更が必要なら [Ubuntu 公式手順](https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/)で対象 GPU に対応する版を選ぶ。
セットアップスクリプトは GPU driver を自動置換しない。[Isaac ROS 対応環境](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#supported-platforms)

`nvidia-smi` の CUDA 表示は driver が対応する上限を示し、ホストに CUDA Toolkit が導入済みであることとは別である。
Docker 方式では ROS・CUDA 等のアプリ依存をコンテナに置き、ホストへ ROS / librealsense を重複導入しない。

## 2. ホスト準備

```bash
./scripts/setup.sh
```

メニューから tools、Docker、NVIDIA runtime、Isaac ROS CLI、保存先、workspace の準備を進める。
既存の Docker や runtime が使える場合はそれを維持する。
CLI は NVIDIA の `release-5.0` / `noble` apt repository から導入する。独自 CLI のソースビルドは不要。[公式 apt 導入手順](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#configure-isaac-ros-apt-repository)

docker グループ追加は実行中セッションに反映されない。案内が表示された場合は再ログインして、必要な段階から再実行する。
保存先は、たとえばホーム配下の `atd-data/pc/record` と `atd-data/pc/results`、または mount 済み SSD 配下を指定する。
storage の対話入力または `--record-dir` / `--result-dir` でホスト側の実際の絶対パスを設定し、Docker 内では共通の `/data/record` / `/data/results` を使う。

## 3. ROS workspace と初回 build

Notebook 上で ROS workspace の場所を指定し、コンテナ内では標準 workspace 名へ対応させる。既定は Notebook の `~/workspaces/atd`。

| 用途 | Notebook ホスト | コンテナ内 |
| --- | --- | --- |
| ROS workspace | 指定したパス。既定 `~/workspaces/atd` | `/workspaces/isaac_ros-dev`（読書き可） |
| 自作 ROS package の編集元（別 workspace を使う場合） | `<ATD checkout>/ros2_ws/src` | `/workspaces/isaac_ros-dev/src/atd`（読書き可） |
| プロジェクト設定参照 | `<ATD checkout>` | `/workspaces/atd`（readonly） |

```bash
./scripts/setup.sh --action workspace
./scripts/setup.sh --action verify
./scripts/setup.sh --action build
```

workspace は対話入力した場所に CLI 設定を準備し、`config/workspace.env` の `ATD_PC_ROS_WS` に保存する。別の workspace を使う場合は `src/atd` の mount point も用意する。直接指定は `--workspace /absolute/path/atd_ws` を追加する。自作 package の編集元はリポジトリの `ros2_ws/src/` とする。
リポジトリの `ros2_ws` を workspace に選ぶ場合は、その `src` がコンテナの `/workspaces/isaac_ros-dev/src` に対応し、追加 source mount は行わない。
`ISAAC_ROS_WS` は `dev.sh` が設定するため、ホストの `.bashrc` へ export を追記しない。設定の詳細と既存 CLI 設定との衝突時の扱いは [共通手順](README.md#ros-workspace-のパス)を参照する。
`./scripts/docker.sh` で項目別診断を保存し、`./scripts/docker.sh --action logs` で Notebook の過去の結果を確認できる。
初回 build は `isaac_ros → realsense455 → atd` の順で行う。librealsense SDK だけ CUDA 無効とし、Isaac ROS の GPU 処理は維持する。
verify は一般ユーザーから Docker に接続し、CLI の Docker モードと `--gpus all` による `/bin/true` のコンテナ起動を確認する。CUDA 演算成功は別に検証する。

コンテナに入った後、必要な確認を個別に行える。

```bash
printenv ROS_DISTRO ISAAC_ROS_WS RMW_IMPLEMENTATION
ros2 pkg prefix realsense2_camera
ros2 pkg prefix rosbag2_storage_mcap
```

期待する ROS / workspace / RMW は `lyrical` / `/workspaces/isaac_ros-dev` / `rmw_fastrtps_cpp`。
ROS package が見つかることは、実 D455 の動作や bag 取得の成功を意味しない。

## 4. 通常の使い方

通常は `./scripts/setup.sh --action start` または `./scripts/dev.sh` で入る。
PC では受信した bag の offline 処理、地図作成・編集、解析、可視化を行う。bag と地図の転送手段・地図形式は後工程で具体化する。
将来の UI は PC ホストの Chrome で `http://127.0.0.1:<port>` を開く予定で、UI 本体は現時点では未実装。

D455 を PC に直接接続する場合は devices のホスト準備も行い、[Jetson 手順の D455 確認](jetson.md#3-d455-と-dualsense)と同じ順に確認する。
PC の Docker と Jetson の Docker は別々に build し、PC の `build/` / `install/` を Jetson 用として転送しない。
