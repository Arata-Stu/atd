# ディレクトリ・ファイルの配置規約

clone 内でソース・設定、ROS workspace、永続データ、生成物の役割を分ける。clone の標準配置は `~/workspaces/atd`。未設定時の workspace と保存先は実際の clone 位置を基準に決める。
配置はこの文書を基準とし、必要な実装を追加する時点でディレクトリを作る。将来用の空ディレクトリや空の文書を先に増やさない。

## リポジトリの構成

```text
~/workspaces/atd/              # clone の標準配置
├── AGENTS.md                  # 開発規約
├── README.md                  # 目的・操作入口・文書一覧
├── config/                    # ホスト固有設定の例とローカル設定
├── data/                      # Git 管理対象外の永続資産。storage 操作で root を準備
│   ├── record/                # bag 等の記録データ
│   └── results/               # 地図・モデル・run 結果
├── docker/
│   ├── Dockerfile.*           # Isaac ROS の追加 image layer
│   ├── isaac_ros/             # 公式 CLI 用の設定テンプレート
│   ├── middleware_profiles/   # DDS XML
│   ├── python/ros/            # ROS 2 用 pyproject.toml / uv.lock
│   ├── python/train/          # 学習用 pyproject.toml / uv.lock
│   ├── python/vgl-export/     # Notebook の VGL ONNX export 用依存
│   ├── python/web/            # Notebook の Web API 用 uv 環境
│   └── scripts/               # コンテナの起動処理・環境操作
├── docs/                      # 要件・構成・操作手順・技術的な判断
│   └── setup/                 # 共通・Notebook・Jetson の導入手順
├── hardware/
│   └── cad/                   # 車体・部品CADと3Dプリント資料
├── ros2_ws/                   # リポジトリ内の ROS 2 workspace
│   └── src/                   # 自作 package の編集元。atd_<module>/ を追加する
│       ├── atd_vehicle_interface/
│       │   └── firmware/pico_rev6/ # Pico用ソース・テスト。SDK・生成物はGit管理外
│       └── README.md          # package の配置ガイド
├── scripts/                   # 日常入口は次の5本
│   ├── setup.sh               # ホスト準備・workspace・保存先
│   ├── dev.sh                 # Dockerのbuild・起動・再接続
│   ├── build.sh               # ROS packageのbuild
│   ├── vgl.sh                 # VGLモデルの準備・転送
│   ├── bringup.sh             # 実装済みROS起動profileの選択・実行
│   ├── tools/                 # docker.sh・python.sh・webui.shの補助入口
│   ├── lib/                   # 共通関数・detect_host.sh・bringup.shのprofile一覧
│   └── setup/                 # ホスト準備の内部処理
├── tools/
│   └── vgl/                   # VGL export bundle・転送・engine build の処理
└── tests/                     # setup・Docker 等の共通基盤のテスト
```

`apps/web/` と `atd_joy` / `atd_web_bridge`、運転要求型の `atd_interfaces` と操作割り当ての `atd_teleop`、Pico USB 受信の `atd_vehicle_interface` とプロポ役割の `atd_rc_input`、保存地図の点群取得用 `atd_mapping`、D455の `atd_sensors`、車両配置・static TFの `atd_vehicle_description` と統合起動の `atd_bringup`、操作イベントから記録する `atd_recording`、Notebookで車両応答を解析する `atd_vehicle_calibration` を実装した。Pico firmware の編集元も関連する `atd_vehicle_interface` 内へまとめる。ソース実装と対象機での build・実機検証は区別する。

| 配置 | 内容 |
| --- | --- |
| `apps/web/` | 地図・転送・ライブ表示等をまとめる共通 Web アプリ。frontend / backend はこの配下で分ける |
| `ros2_ws/src/atd_vehicle_interface/firmware/pico_rev6/` | Pico Rev6 firmware の編集元・設定・ホストテスト。必要な JPB1 共通通信コードもこの中で管理する |
| `tools/<purpose>/` | ROS node ではないデータ抽出・解析・学習等の実装 |
| `hardware/cad/` | 車体・部品のFCStdと印刷用3MF。TT-02-v3とRev6カメラ組立モデルはディレクトリを分ける |
| `hardware/<kind>/` | CAD 以外の回路・配線等の編集元資料 |

共通 Web アプリは `apps/web/{frontend,backend,processing}/` に分ける。frontend は React / TypeScript / Vite、backend は FastAPI とジョブ・転送・公式処理 adapter、processing は Web 非依存の HDMap 幾何処理・ライン生成。UI を使うのは Notebook のみで、Jetson 側の入力・役割 node と `atd_web_bridge` は `scripts/bringup.sh input` で明示起動する。プロポの役割設定は `/data/results/runs/input/propo.yaml` へ保存し、Pico の firmware と走行用 CH 設定は変更しない。操作は [Web アプリ](../apps/web/README.md)を参照する。

Pico firmware はユーザー提供の `/Users/at/project/tools/pcb/kart_bridge_board/firmware/pico_rev6/` と必要な共通通信コードを取り込み、以後の ATD 向け編集元を `atd_vehicle_interface/firmware/pico_rev6/` とする。元ディレクトリは保持し、自動同期・削除は行わない。SDK は同ディレクトリの `.cache/pico-sdk/`、生成物は `build/` に置き、Git 管理対象から外す。`firmware/COLCON_IGNORE` で独立した Pico CMake project を ROS workspace の探索から除外し、colcon から SDK の取得・ARM build・書き込みを実行しない。実基板へ書き込まれた版との一致は別途確認する。

## ROS 2 package

自作 package はリポジトリの `ros2_ws/src/` 直下に置き、ディレクトリ名・`package.xml` の名前を `atd_<module>` に合わせる。`ros2_ws/` は workspace、その下の `src/` は package ソースとして役割を分ける。責務の境界で分け、PC 用・Jetson 用という理由だけで同じ実装を複製しない。構成の違いは launch・設定・build 条件で表す。

```text
ros2_ws/src/atd_example/
├── CMakeLists.txt             # ament_cmake_auto を基本にする
├── package.xml
├── include/atd_example/       # 公開する C++ header
├── src/                      # C++ 実装・component
├── launch/                   # 必要な launch
├── config/                   # package に属する parameter 等
├── test/                     # package のテスト
└── README.md                 # 役割・入出力・使い方
```

使わないディレクトリは作らない。C++ のファイル名は `snake_case.cpp` / `snake_case.hpp` を基本とし、公開 header は `include/<package>/` に置く。大容量データを扱う node は component として登録する前提で実装する。
独自 message・service・action が必要な package だけ、`msg/`・`srv/`・`action/` を追加する。車体記述等のリソースも、それを管理する package 内に配置する。

Python node も CMake / `ament_cmake_auto` で build する。実行入口は package 内の `scripts/`、import する Python module は package 名のディレクトリに置き、必要に応じて `ament_cmake_python` で install する。直接実行するファイルは shebang・実行権限・`install(PROGRAMS ...)` を整える。

package 固有の parameter、較正設定、launch はその package に置く。現在の入力系の統合 launch は `atd_web_bridge/launch/input.launch.py` にあり、`scripts/bringup.sh input` から起動する。日常操作の profile 選択と ROS launch の責務を分け、profile 一覧は `scripts/lib/bringup.sh` に置く。センサ・自己位置推定・車両I/O等をまとめる実装が必要になった時点で、`atd_bringup` 等の専用 package の責務を決める。未実装の profile や空 package は用意しない。root の `config/` はホストのパス・保存先等に使い、ROS parameter を集める場所にはしない。

ホストでの編集元は `<ATD checkout>/ros2_ws/src/` とする。

| 選択したホスト workspace | コンテナ内の自作 package ソース |
| --- | --- |
| `<ATD checkout>/ros2_ws` | `/workspaces/isaac_ros-dev/src/`。workspace mount をそのまま使い、追加の source mount は行わない |
| その他のパス | `/workspaces/isaac_ros-dev/src/atd/`。編集元を rw で追加 mount する |

別のホスト workspace を選択した場合、その `src/atd` は mount point として予約し、そこへ別のソースをコピーしない。既存データがある場合は setup を停止して保護する。
コンテナの `/workspaces/atd` はホストの Git checkout を読書き共有する。対話シェルで `git pull` すると、workspace 内の package ソースにも更新が反映される。別の workspace を使う場合も、編集元の `ros2_ws/src/` と workspace の `src/` を区別する。
標準の build 入口は `./scripts/build.sh`、コンテナ内は `atd-build` とし、workspace の `src` を探索対象にする。`docker/scripts/atd-build.sh` が colcon の実行と CMake の interpreter 指定を担当する。直接 `colcon build` を使う場合も `--base-paths src` を明示し、checkout と workspace が同じ場所にあるときの重複探索を避ける。

## 設定・依存・操作の入口

| 対象 | 配置・管理 |
| --- | --- |
| workspace・保存先の設定例 | `config/*.env.example`。変数名・実パスの記入例・説明を記録する |
| ホスト固有の値 | `config/workspace.env`、`config/storage.env`。ホストごとに保存し、Git 管理対象外 |
| 共通 apt 依存 | `docker/dependencies.apt` |
| 追加 Python 依存 | `docker/python/<purpose>/{pyproject.toml,uv.lock}`。用途別に管理する |
| 初回 setup | `scripts/setup.sh` |
| Docker 起動・build | `scripts/dev.sh` |
| 診断と履歴 | `scripts/tools/docker.sh` |
| 用途別 Python 環境の管理・実行 | `scripts/tools/python.sh` → コンテナ内の `atd-python`。sync / check / shell / run |
| ROS package の build | `scripts/build.sh` → コンテナ内の `atd-build` |
| VGL モデルの export・転送・build | `scripts/vgl.sh` |
| Notebook 共通 Web UI | Docker 内の `atd-web`。ホストからの補助入口は `scripts/tools/webui.sh` |
| Jetson の ROS 起動 | `scripts/bringup.sh`。`manual` / `d455` / `input` / `bridge` の実在 profile を選択 |

日常操作の入口を `scripts/` 直下に置き、現在は setup・dev・build・vgl・bringup の5本にまとめる。診断・環境管理等の補助入口は `scripts/tools/` に置く。コンテナ内の処理は `docker/scripts/`、ホスト準備の内部処理は `scripts/setup/`、ホスト側の共通関数は `scripts/lib/` に置く。ホスト判定は `scripts/lib/detect_host.sh` を共用する。入口ごとに同じ設定読込みやホスト判定を実装し直さない。コンテナ内のビルド処理と Python 環境管理は独立したコマンドにし、共通の環境準備は `docker/scripts/lib/` から読み込む。

新規のディレクトリ・文書・設定ファイルは、内容が分かる英語の `snake_case` を基本とする。`Dockerfile.<image_key>`、`CMakeLists.txt`、`package.xml` 等のツールが要求する名前と、既存の公開コマンド名はその規則を優先する。

## 文書とハードウェア資料

| 文書 | 担当する内容 |
| --- | --- |
| [README](../README.md) | プロジェクトの入口。詳細手順を重複掲載しない |
| [開発規約](../AGENTS.md) | 開発時に守るルール |
| [要件](project_requirements.md) | 確定要件・提案・未決事項 |
| この文書 | ディレクトリとファイルの置き場所 |
| [セットアップ手順](setup/README.md) | 実際の導入・起動・診断操作 |
| [bringup](bringup.md) | Jetson の起動 profile、選択・引数・拡張方針 |
| [rosbag記録](recording.md) | 入力イベントによる記録開始停止、profile、状態と実機確認 |
| [車両モデルの校正](vehicle_calibration.md) | 手動走行で取得する入力とオフライン VSLAM 速度を使った同定手順 |
| [車両配置・TF](vehicle_layout.md) | 座標系、UI編集、実測、static TFの適用 |
| [Python 環境](python_environments.md) | uv、用途別環境、ROS 2 Python との関係 |
| [VGL モデル](vgl_models.md) | Notebook export、SCP 転送、Jetson engine build、検証・ログ |
| [ビルド方針](isaac_ros_build_strategy.md) | CLI・Docker・ROS package の build 方針と技術的な根拠 |
| [環境セットアップ案](environment_setup_plan.md) | Kart の比較調査、初期実装の状況と対象機での残りの検証 |
| [CAD・3Dプリント資料](hardware_cad.md) | CAD・3MFの一覧、取り込み元、版と形式、確認範囲 |

CAD 等の編集元データと提供された印刷用3MFは `hardware/cad/` に置き、一覧と由来を [CAD・3Dプリント資料](hardware_cad.md) から参照する。FCStdと3MFは元の形式で保持し、FreeCADの自動バックアップはGit対象外にする。`docs/` は説明文書を置く場所とし、編集元 CAD と空の仕様書を混在させない。実測・確定した仕様が増えた時点で、対応する文書を追加する。

## 生成物・データ

| 対象 | 配置・扱い |
| --- | --- |
| CLI 用の生成 Dockerfile | `<ATD checkout>/.cache/docker/<revision>/`。再生成可能、Git 管理対象外 |
| 診断ログ | `<ATD checkout>/logs/docker/<host>/`。履歴を保持し、Git 管理対象外 |
| ROS build 成果物 | `<選択した workspace>/{build,install,log}/`。未設定時の workspace は `<ATD checkout>/ros2_ws` |
| Python venv / cache | `<選択した workspace>/.atd-python/`。各ホスト・アーキテクチャで生成する |
| Web の node_modules / frontend build | `<選択した workspace>/.atd-web/frontend/`。Git checkout のソースと分け、Notebook 上で生成する |
| Joy 物理配置・入力の役割設定 | Jetson の `/data/results/runs/input/{dualsense,teleop,propo}.yaml`。再生成用 cache ではなく永続設定として保持 |
| ESC 試験 | Jetson の `/data/results/runs/esc_trials/` に予約手順・最終状態と試験ごとの時系列ログを保持 |
| D455起動・車両配置設定 | Jetsonの `/data/results/runs/bringup/d455.yaml` と `/data/results/runs/vehicle/layout.yaml`。再起動後も保持 |
| bag | ホスト別設定の記録先。コンテナでは `/data/record` |
| 地図・モデル・解析・学習結果 | ホスト別設定の結果先。コンテナでは `/data/results` |

bag・地図・モデル重み・TensorRT engine・大きな学習結果は、clone 配下の `data/` を既定の保存先とし、Git の管理対象には含めない。実装コードと永続資産の役割を分け、参照する入力・モデル・成果物の情報を管理する。venv や build 成果物を Notebook と Jetson の間でコピーして利用しない。

自動生成できる cache と bytecode は必要時に再生成する。記録データ・地図・診断ログ・CAD 等の編集元は、cache と同じ扱いで削除しない。

## データの保存構成

保存先の設定と mount は既存の記録先・結果先の二つを使い、その中の用途別配置を次のように定める。新しい環境変数や mount は追加しない。
Notebook では `ATD_PC_RECORD_DIR` / `ATD_PC_RESULT_DIR`、Jetson では `ATD_JETSON_RECORD_DIR` / `ATD_JETSON_RESULT_DIR` がホスト側の root に対応する。未設定時は実際の clone 配下の `data/record`・`data/results` を使う。clone が `~/workspaces/atd` なら `~/workspaces/atd/data/record`・`~/workspaces/atd/data/results` となる。
両ホストの設定は別々に保持し、保存済み指定や引数・環境変数の上書きを優先する。設定例のコメントはホストの実パスを記入して使うためのものであり、他ホストの未設定値を自動保存するものではない。

```text
/data/record/
└── rosbags/<bag_id>/                 # bag ディレクトリ全体
    ├── metadata.yaml
    └── *.mcap 等                    # recorder が作成した内部構成を維持

/data/results/
├── maps/<map_id>/
│   ├── nvidia/                      # NVIDIA 公式処理の map_dir。以下は公式出力例
│   │   ├── cuvslam_map/
│   │   ├── cuvgl_map/
│   │   ├── occupancy_map.yaml
│   │   └── ground_plane.yaml
│   ├── hdmap/
│   │   ├── map.json                 # HDMap 編集正本・採用captureへの参照
│   │   └── captures/<capture_id>/   # snapshot.json・表示cloud.json・manifest.json
│   └── lines/                       # centerline / raceline / custom line
├── models/
│   ├── vgl/<asset_id>/
│   │   ├── source/                  # Notebook で取得・準備する元モデル・設定
│   │   ├── exported/<export_id>/    # Notebook で作る可搬 export bundle。SCP 転送単位
│   │   └── engines/<target_id>/<build_id>/
│   │                                # Notebook・Jetson のGPU別に生成。engineを相互流用しない
│   └── e2e/<model_id>/<version>/     # 将来の E2E 用。必要になった時点で作成
│       ├── weights/
│       ├── onnx/
│       └── engines/<target_id>/<build_id>/
├── runs/<run_id>/                    # 処理・学習ごとの結果
│   ├── logs/
│   ├── checkpoints/
│   └── ...                          # metrics・可視化結果等
├── datasets/<dataset_id>/            # 必要時の抽出・学習データ
└── cache/<purpose>/                  # 再生成可能な一時データに限定
```

setup の storage 操作は記録先・結果先の root を準備する。上記の用途別・ID 別ディレクトリは、その機能を初めて使用する時点で作成する。現段階で空のモデル置場やダミーの管理ファイルは作らず、既存データも自動移動しない。
新しい bag は `rosbags/<bag_id>/` を保存・SCP 転送の単位とし、`metadata.yaml` と MCAP 等を含む内部構成を保つ。既存 bag のディレクトリ構成をこの整理のために書き換えない。

### NVIDIA 地図と VGL モデル

NVIDIA 公式の地図生成には `maps/<map_id>/nvidia/` を `map_dir` として渡す。公式チュートリアルにある出力例は `cuvslam_map/`、`cuvgl_map/`、`occupancy_map.yaml`、`ground_plane.yaml` であり、ATD 独自の `cuvslam/`・`vgl/` 等へ改名しない。`hdmap/` と `lines/` はその外側に分け、地図 ID で対応付ける。[公式地図作成](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/visual_global_localization/tutorials/tutorial_map_creation.html)

地図側の `cuvgl_map/` と、VGL が使用するモデル・engine は別の資産として扱う。モデル処理は次の分担とする。

| 段階 | 実行場所と保存先 |
| --- | --- |
| 元モデル・設定の取得、準備 | Notebook の `models/vgl/<asset_id>/source/`。入力側の構成を維持する |
| 可搬な中間表現への export | Notebook の `exported/<export_id>/`。ALIKED-n16 を FP32 / opset 17 の ONNX に export。既存 ONNX の package も対応 |
| 転送 | `exported/<export_id>/` の bundle 全体を SCP で Jetson へ渡す |
| TensorRT engine build | Jetson が受信 bundle から `engines/<target_id>/<build_id>/` を生成する |

転送 bundle は `models/`・`configs/` と `manifest.json` を持ち、graph、external data、必要な設定、元モデル・export の由来と hash をまとめて内部の参照関係を保つ。SCP 後にも検証してから正式な保存先へ反映し、既存 ID の資産は上書きしない。Jetson は受信済み export を engine 化する役割とし、必要な場合の学習や元 checkpoint からの中間 export は Notebook 側で行う。

操作は `scripts/vgl.sh` と [VGL モデルの手順](vgl_models.md)を使う。Notebook の export は専用の Python 処理、Jetson の engine build は公式の `export_lightglue_engine`・`export_extractor_engine` を呼び出す。これらの公式コマンドは TensorRT engine 生成の段階で使い、Notebook の ONNX export と混同しない。`atd_localization` は対象機の GPU・TensorRT・export に対応する engine を model 資産から解決する。候補が複数ある場合は `engine_dir` で build を選ぶ。[公式 VGL localization](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/visual_global_localization/tutorials/tutorial_cuvgl_localization.html)

engine は対象 GPU・TensorRT 等の環境別に生成・管理する。実装では model / export ID と hash、GPU の識別情報、TensorRT の版、CUDA driver version、関連 package の版、起動構成 hash `atd_runtime_revision` と build ログを対応付ける。Docker image digest 自体は現実装では記録していない。転送する中間表現も Jetson 側の対応演算・依存を確認し、CPU / GPU 間での利用を検証済みとは扱わない。[TensorRT の engine 互換性](https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/support-matrix.html#software-and-system-requirements)
地図には元 bag と使用 model の ID、座標系・較正等の対応を持たせる方針とし、地図用の管理ファイルの名前・schema は実装時に決める。
`source/` の入力モデル、`exported/` の転送 bundle、`engines/` の生成成果物はいずれも保持する資産であり、通常の cache 削除には含めない。`runs/` のログ・checkpoint も同様に保持する。

E2E 用の配置は将来の保存先の予約であり、framework、重み・ONNX の形式、学習・export 手順を採用済みとするものではない。VGL の処理入口と資産・hash 検査は実装しているが、対象機でのモデル取得・ONNX export・実 SCP・engine build・localization は未検証である。地図作成も実機検証は未実施である。
