# ATD

RealSense D455 を初期センサとし、Jetson Orin Nano 8GB を搭載した RC カーで自動運転を行うプロジェクト。
実走行の処理・制御と車載 ROS 2 / DDS 通信は Jetson 内で完結させ、ノート PC は地図作成・編集・解析・可視化に使用する。
共通 Web アプリは React + TypeScript / Vite と FastAPI で実装し、Map・Transfer・Live・Input を統合する。UI はノート PC の Chrome だけで `127.0.0.1:<port>` から利用する。Jetson では ROS の観測・設定 bridge と入力ノードを動かす。

想定基盤は Ubuntu 24.04 / JetPack 7.2 / ROS 2 Lyrical / Isaac ROS 5 の Docker。Notebook は x86_64・RTX 4060、Jetson は aarch64・Orin Nano を区別して準備する。イベントカメラ関連の依存は導入しない。

## 操作の入口

プロジェクト直下から通常ユーザーで実行する。対話操作には fzf を使い、setup の初回は番号選択も利用できる。

```bash
./scripts/setup.sh            # ホスト準備、workspace・保存先の対話設定
./scripts/dev.sh --build      # 初回イメージを build して入室
./scripts/dev.sh              # 通常起動・稼働中コンテナへの再接続
./scripts/build.sh            # 起動済みコンテナで ROS package を build
./scripts/docker.sh           # OK / FAIL / SKIP 診断とログ保存
./scripts/docker.sh --logs    # 過去ログを選択
./scripts/vgl.sh              # VGL export・SCP 転送・Jetson engine build を選択
./scripts/webui.sh            # ホストからの補助入口。通常は Docker 内で atd-web
./scripts/input.sh            # Jetson の Joy 入力・teleop 操作要求・観測設定 bridge を明示起動
```

Web UI は Notebook の Docker 内で `atd-web` を実行し、Notebook ホストの Chrome で `http://127.0.0.1:8766` を開く。FastAPI・frontend の build / 配信・地図作成ジョブはコンテナ内で動く。

PC / Jetson は機種・BSP 情報から自動判定する。必要な場合だけ `--host notebook`（`pc` と同義）/ `--host jetson` で上書きする。実行予定の確認には `--dry-run` を使う。
ホストのセットアップを最初から確認する場合は、[共通手順](docs/setup/README.md)、[Notebook](docs/setup/pc.md)、[Jetson](docs/setup/jetson.md)を参照する。

Docker 起動時に共通 apt 依存、Fast DDS の UDPv4 / loopback 設定、ROS 用 uv 環境を準備する。カメラ・bag 記録・車両ノード・bridge は自動起動しない。構成更新時は利用中の処理を終了してコンテナを再作成する。

## ソースと workspace

ROS package の編集元は、このリポジトリの [`ros2_ws/src/`](ros2_ws/src/README.md) に `atd_<module>` として置く。
clone の標準配置を `~/workspaces/atd` とし、未設定時の ROS workspace は実際の clone 配下の `ros2_ws`、データは `data/record` と `data/results` を使う。標準配置ならそれぞれ `~/workspaces/atd/ros2_ws`、`~/workspaces/atd/data/{record,results}` になる。
setup で Notebook / Jetson 別に設定を保存する。保存済みの指定、引数・環境変数の上書きは新しい既定値より優先する。

| ホストの対象 | コンテナ内 |
| --- | --- |
| 選択した ROS workspace。未設定なら `<ATD checkout>/ros2_ws` | `/workspaces/isaac_ros-dev`（rw） |
| `<ATD checkout>/ros2_ws/src`（同じ `ros2_ws` を workspace に選ぶ場合） | `/workspaces/isaac_ros-dev/src`（workspace mount 内） |
| `<ATD checkout>/ros2_ws/src`（別 workspace を使う場合） | `/workspaces/isaac_ros-dev/src/atd`（rw） |
| ATD checkout の設定参照 | `/workspaces/atd`（readonly） |
| 記録先。未設定なら `<ATD checkout>/data/record` | `/data/record`（毎回 rw） |
| 結果先。未設定なら `<ATD checkout>/data/results` | `/data/results`（毎回 rw） |

ROS package の編集元はホストの `ros2_ws/src/` とする。workspace に `<ATD checkout>/ros2_ws` を選ぶ場合はその `src` をそのまま使う。別の workspace を使う場合は、コンテナの `src/atd/` から同じソースを編集できる。そのホスト workspace の `src/atd` はソース mount 用の予約先であり、別の package をコピーして管理しない。
build 成果物と venv は各ホストの workspace に保持し、Notebook と Jetson の間では共有しない。bag は `data/record/rosbags/`、地図・モデル・run 結果は `data/results/{maps,models,runs}/` が標準配置。`data/` は Git 管理対象外の永続資産であり、cache として削除しない。詳細な階層は [保存構成](docs/project_structure.md#データの保存構成)を参照する。
以前の保存設定を使っている場合は、setup の workspace / storage 操作で新しい配置を明示的に選び直す。既存データの自動移動は行わない。
VGL モデルは Notebook で ONNX export し、SCP で bundle を渡して Jetson で TensorRT engine を生成する。`vgl.sh` の対話操作、既存 ONNX の取り込み、検証・ログ確認は [VGL モデルの手順](docs/vgl_models.md)を参照する。処理の入口は実装済みで、対象機の GPU / SDK・実 SCP を含む通し動作は未検証である。
ディレクトリ・ファイルの置き場所は [構成規約](docs/project_structure.md)を参照する。

## 用途別 Python 環境

Docker 内では uv `0.13.8` を使い、ROS 2 用 `ros`、AI 学習用 `train`、Notebook の VGL export 用 `vgl-export` を独立した project / lock / venv に分ける。
ROS 用は起動時に通常ユーザーで同期し、学習・VGL export 用は必要時に同期する。通常の対話シェル全体は venv へ切り替えず、用途を選んだプロセスに適用する。Python 環境の管理・実行は `python.sh` / `atd-python`、ROS package のビルドは専用の `build.sh` / `atd-build` が担当する。

起動済みコンテナに対し、別端末から操作できる。

```bash
./scripts/python.sh                   # 用途・操作を fzf で選択
./scripts/python.sh ros check
./scripts/build.sh                    # ROS package を build
./scripts/python.sh train shell
```

`ros` / `train` の追加 Python 依存は現時点では空。VGL export の PyTorch 等は専用 profile に固定し、AI 学習用 framework は今後選定する。詳しくは [Python 環境](docs/python_environments.md)を参照する。

## 文書

| 文書 | 内容 |
| --- | --- |
| [プロジェクト要件](docs/project_requirements.md) | 確定要件、提案、未決事項 |
| [開発規約](AGENTS.md) | 命名、C++ / ament、component、Python、検証 |
| [構成規約](docs/project_structure.md) | ソース、設定、文書、生成物の配置 |
| [セットアップ](docs/setup/README.md) | ホスト準備、workspace・保存先、Docker・診断操作 |
| [Python 環境](docs/python_environments.md) | uv、ROS 2 用・AI 学習用環境の操作と制約 |
| [VGL モデル](docs/vgl_models.md) | Notebook の export、SCP 転送、Jetson engine build、検証・ログ |
| [ビルド方針](docs/isaac_ros_build_strategy.md) | CLI / Docker / ROS の範囲、RealSense、DDS と技術的な根拠 |
| [環境セットアップ案](docs/environment_setup_plan.md) | Kart の比較調査、未実装部分の構成案 |
| [車体 CAD](hardware/cad/tt02-v1.FCStd) | 保存する編集元データ。実物との一致は未検証 |

現在は共通基盤の初期実装段階。スクリプト・設定の自動テストと、実 Docker build・ROS / GPU / D455 / Joy・車両 I/O・実走行の検証は区別する。対象機での通し導入・実機検証は未実施。

共通 UI の使い方と検証境界は [Web アプリ](apps/web/README.md)を参照する。
