# ATD

RealSense D455 を初期センサとし、Jetson Orin Nano 8GB を搭載した RC カーで自動運転を行うプロジェクト。
実走行の処理・制御と車載 ROS 2 / DDS 通信は Jetson 内で完結させ、ノート PC は地図作成・編集・解析・可視化に使用する。
共通 Web アプリは React + TypeScript / Vite と FastAPI で実装し、Map・Transfer・Live・Input・車両設定を統合する。UI はノート PC の Chrome だけで `127.0.0.1:<port>` から利用する。Jetson では ROS の観測・設定 bridge と入力ノードを動かす。

想定基盤は Ubuntu 24.04 / JetPack 7.2 / ROS 2 Lyrical / Isaac ROS 5 の Docker。Notebook は x86_64・RTX 4060、Jetson は aarch64・Orin Nano を区別して準備する。イベントカメラ関連の依存は導入しない。

## 操作の入口

プロジェクト直下から通常ユーザーで実行する。日常の入口は次の5本にまとめる。

| 入口 | 用途 |
| --- | --- |
| `scripts/setup.sh` | ホスト準備、workspace・保存先の設定 |
| `scripts/dev.sh` | Docker image の build、コンテナの起動・再接続 |
| `scripts/build.sh` | 起動済みコンテナで ROS package を build |
| `scripts/vgl.sh` | VGL export・SCP 転送・Jetson engine build |
| `scripts/bringup.sh` | Jetson の実装済み起動 profile を選択・実行 |

```bash
./scripts/setup.sh
./scripts/dev.sh --build
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh joy-drive # CH3 切替による DualSense 手動走行（カメラなし）
./scripts/bringup.sh manual    # D455・車体TF と DualSense 手動走行
./scripts/bringup.sh localization  # 保存地図とラインを選び、自己位置・計算舵角・FF＋PID とプロポ／DualSense 入力を監視
./scripts/bringup.sh autonomous    # 校正済み車両で、地図・速度付きラインを選んで低速前進を開始
./scripts/bringup.sh input     # Joy・Pico入力、役割処理、観測設定bridge
./scripts/bringup.sh bridge    # bridgeだけ。入力nodeが別途起動済みの場合
```

`bringup.sh` は引数なしの対話端末では profile を fzf で選択し、未導入時は番号選択を使う。D455を含む `manual` / `d455` では、各ストリームのOFF・解像度・Hzを実機の対応モードから選択する。初期値はInfra1/2 640×480@90、RGB 640×480@30、加速度200Hz・ジャイロ400Hz、Depth OFF。`--list` は一覧、`--dry-run manual` は Notebook からも実行予定だけを確認できる。実際の起動は Jetson の既存 Docker 内で行い、車両出力や bag 記録は自動開始しない。`manual` / `input` は記録managerを待機させ、割り当てたJoy・プロポ操作で開始・停止する。[bringup の操作と拡張方針](docs/bringup.md)を参照する。

診断・Python 環境管理・Web のホスト側補助入口は `scripts/tools/{docker,python,webui}.sh` に置く。Web UI は通常 Notebook の Docker 内で `atd-web` を実行し、Notebook ホストの Chrome で `http://127.0.0.1:8766` を開く。ホストからは `./scripts/tools/webui.sh` でも同じ処理を呼べる。FastAPI・frontend の build / 配信・地図作成ジョブはコンテナ内で動く。

setup・dev・build・vgl の PC / Jetson 判定は機種・BSP 情報を使い、必要な場合だけ `--host notebook`（`pc` と同義）/ `--host jetson` で上書きする。bringup の対象ホストは profile ごとに決める。実行予定の確認には `--dry-run` を使う。
ホストのセットアップを最初から確認する場合は、[共通手順](docs/setup/README.md)、[Notebook](docs/setup/pc.md)、[Jetson](docs/setup/jetson.md)を参照する。

共通 apt 依存や SDK を更新したら Docker イメージを再ビルドする。起動時は apt 依存の導入状態を検査し、Fast DDS の UDPv4 / loopback 設定と ROS 用 uv 環境を準備する。日常コマンドはマウント済み checkout から実行するので、編集は次の実行で反映される。カメラ・bag 記録・車両ノード・bridge は明示的に起動する。[更新後の操作](docs/setup/README.md#git-pull)

## ソースと workspace

ROS package の編集元は、このリポジトリの [`ros2_ws/src/`](ros2_ws/src/README.md) に `atd_<module>` として置く。
clone の標準配置を `~/workspaces/atd` とし、未設定時の ROS workspace は実際の clone 配下の `ros2_ws`、データは `data/record` と `data/results` を使う。標準配置ならそれぞれ `~/workspaces/atd/ros2_ws`、`~/workspaces/atd/data/{record,results}` になる。
setup で Notebook / Jetson 別に設定を保存する。保存済みの指定、引数・環境変数の上書きは新しい既定値より優先する。

| ホストの対象 | コンテナ内 |
| --- | --- |
| 選択した ROS workspace。未設定なら `<ATD checkout>/ros2_ws` | `/workspaces/isaac_ros-dev`（rw） |
| `<ATD checkout>/ros2_ws/src`（同じ `ros2_ws` を workspace に選ぶ場合） | `/workspaces/isaac_ros-dev/src`（workspace mount 内） |
| `<ATD checkout>/ros2_ws/src`（別 workspace を使う場合） | `/workspaces/isaac_ros-dev/src/atd`（rw） |
| ATD の Git checkout（対話シェルの開始位置） | `/workspaces/atd`（rw） |
| 記録先。未設定なら `<ATD checkout>/data/record` | `/data/record`（毎回 rw） |
| 結果先。未設定なら `<ATD checkout>/data/results` | `/data/results`（毎回 rw） |

ROS package の編集元はホストの `ros2_ws/src/` とする。workspace に `<ATD checkout>/ros2_ws` を選ぶ場合はその `src` をそのまま使う。別の workspace を使う場合は、コンテナの `src/atd/` から同じソースを編集できる。そのホスト workspace の `src/atd` はソース mount 用の予約先であり、別の package をコピーして管理しない。
build 成果物と venv は各ホストの workspace に保持し、Notebook と Jetson の間では共有しない。bag は `data/record/rosbags/`、地図・モデル・run 結果は `data/results/{maps,models,runs}/` が標準配置。`data/` は Git 管理対象外の永続資産であり、cache として削除しない。詳細な階層は [保存構成](docs/project_structure.md#データの保存構成)を参照する。
以前の保存設定を使っている場合は、setup の workspace / storage 操作で新しい配置を明示的に選び直す。既存データの自動移動は行わない。
VGL モデルは Notebook で ONNX export し、SCP で bundle を渡して Jetson で TensorRT engine を生成する。`vgl.sh` の対話操作、既存 ONNX の取り込み、検証・ログ確認は [VGL モデルの手順](docs/vgl_models.md)を参照する。処理の入口は実装済みで、対象機の GPU / SDK・実 SCP を含む通し動作は未検証である。
ディレクトリ・ファイルの置き場所は [構成規約](docs/project_structure.md)を参照する。

## 用途別 Python 環境

Docker 内では uv `0.13.8` を使い、ROS 2 用 `ros`、AI 学習用 `train`、Notebook の VGL export 用 `vgl-export` を独立した project / lock / venv に分ける。
ROS 用は起動時に通常ユーザーで同期し、学習・VGL export 用は必要時に同期する。通常の対話シェル全体は venv へ切り替えず、用途を選んだプロセスに適用する。Python 環境の管理・実行は `scripts/tools/python.sh` / `atd-python`、ROS package のビルドは専用の `scripts/build.sh` / `atd-build` が担当する。

起動済みコンテナに対し、別端末から操作できる。

```bash
./scripts/tools/python.sh             # 用途・操作を fzf で選択
./scripts/tools/python.sh ros check
./scripts/build.sh                    # ROS package を build
./scripts/tools/python.sh train shell
```

`ros` / `train` の追加 Python 依存は現時点では空。VGL export の PyTorch 等は専用 profile に固定し、AI 学習用 framework は今後選定する。詳しくは [Python 環境](docs/python_environments.md)を参照する。

## 文書

| 文書 | 内容 |
| --- | --- |
| [プロジェクト要件](docs/project_requirements.md) | 確定要件、提案、未決事項 |
| [開発規約](AGENTS.md) | 命名、C++ / ament、component、Python、検証 |
| [構成規約](docs/project_structure.md) | ソース、設定、文書、生成物の配置 |
| [セットアップ](docs/setup/README.md) | ホスト準備、workspace・保存先、Docker・診断操作 |
| [bringup](docs/bringup.md) | Jetson の起動 profile、対話選択・引数・今後の拡張 |
| [Python 環境](docs/python_environments.md) | uv、ROS 2 用・AI 学習用環境の操作と制約 |
| [VGL モデル](docs/vgl_models.md) | Notebook の export、SCP 転送、Jetson engine build、検証・ログ |
| [ビルド方針](docs/isaac_ros_build_strategy.md) | CLI / Docker / ROS の範囲、RealSense、DDS と技術的な根拠 |
| [環境セットアップ案](docs/environment_setup_plan.md) | Kart の比較調査、未実装部分の構成案 |
| [rosbag記録](docs/recording.md) | Joy・プロポからの開始停止、記録profile、状態と保存完了 |
| [車両モデルの校正](docs/vehicle_calibration.md) | 手動走行を記録し、Notebook で VO 速度・加速度・操舵応答を解析 |
| [車両配置・TF](docs/vehicle_layout.md) | 座標系、UIの寸法編集、実測とstatic TFの適用 |
| [CAD・3Dプリント資料](docs/hardware_cad.md) | TT-02印刷フレーム、Rev6カメラ組立モデル、3MFと取り込み元 |

現在は共通基盤の初期実装段階。スクリプト・設定の自動テストと、実 Docker build・ROS / GPU / D455 / Joy・車両 I/O・実走行の検証は区別する。対象機での通し導入・実機検証は未実施。

共通 UI の使い方と検証境界は [Web アプリ](apps/web/README.md)を参照する。
