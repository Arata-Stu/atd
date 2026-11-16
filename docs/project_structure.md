# ディレクトリ・ファイルの配置規約

ATD のソース・設定と、ホストごとの ROS workspace・生成物を分ける。配置はこの文書を基準とし、必要な実装を追加する時点でディレクトリを作る。将来用の空ディレクトリや空の文書を先に増やさない。

## リポジトリの構成

```text
atd/
├── AGENTS.md                  # 開発規約
├── README.md                  # 目的・操作入口・文書一覧
├── config/                    # ホスト固有設定の例とローカル設定
├── docker/
│   ├── Dockerfile.*           # Isaac ROS の追加 image layer
│   ├── isaac_ros/             # 公式 CLI 用の設定テンプレート
│   ├── middleware_profiles/   # DDS XML
│   ├── python/ros/            # ROS 2 用 pyproject.toml / uv.lock
│   ├── python/train/          # 学習用 pyproject.toml / uv.lock
│   ├── python/vgl-export/     # Notebook の VGL ONNX export 用依存
│   └── scripts/               # コンテナの起動処理・環境操作
├── docs/                      # 要件・構成・操作手順・技術的な判断
│   └── setup/                 # 共通・Notebook・Jetson の導入手順
├── hardware/
│   └── cad/                   # 編集元の車体 CAD
├── ros2_ws/                   # リポジトリ内の ROS 2 workspace
│   └── src/                   # 自作 package の編集元。atd_<module>/ を追加する
│       └── README.md          # package の配置ガイド
├── scripts/                   # ユーザーが実行する短い .sh 入口
│   ├── lib/                   # 複数の入口から使う関数
│   └── setup/                 # ホスト準備の内部処理
├── tools/
│   └── vgl/                   # VGL export bundle・転送・engine build の処理
└── tests/                     # setup・Docker 等の共通基盤のテスト
```

次の配置は、対応するコードを実装する時点で使用する。現段階で UI・firmware・ROS node の実装済みを示すものではない。

| 配置 | 内容 |
| --- | --- |
| `apps/web/` | 地図・転送・ライブ表示等をまとめる共通 Web アプリ。frontend / backend はこの配下で分ける |
| `firmware/<target>/` | ESC・サーボ制御用の独自 firmware。対象マイコン確定後に追加する |
| `tools/<purpose>/` | ROS node ではないデータ抽出・解析・学習等の実装 |
| `hardware/<kind>/` | CAD 以外の回路・配線等の編集元資料 |

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

package 固有の parameter、較正設定、launch はその package に置く。複数 package の起動をまとめる設定は、実装時に追加する `atd_bringup` 等の責務として扱う。root の `config/` はホストのパス・保存先等に使い、ROS parameter を集める場所にはしない。

ホストでの編集元は `<ATD checkout>/ros2_ws/src/` とする。

| 選択したホスト workspace | コンテナ内の自作 package ソース |
| --- | --- |
| `<ATD checkout>/ros2_ws` | `/workspaces/isaac_ros-dev/src/`。workspace mount をそのまま使い、追加の source mount は行わない |
| その他のパス | `/workspaces/isaac_ros-dev/src/atd/`。編集元を rw で追加 mount する |

別のホスト workspace を選択した場合、その `src/atd` は mount point として予約し、そこへ別のソースをコピーしない。既存データがある場合は setup を停止して保護する。
コンテナの `/workspaces/atd` はプロジェクト設定の readonly 参照用であり、ソースの編集には workspace 内の package ソースを使う。別の workspace を使う場合も、編集元の `ros2_ws/src/` と workspace の `src/` を区別する。
標準の build 入口は `./scripts/python.sh ros build` とし、workspace の `src` を探索対象にする。直接 `colcon build` を使う場合も `--base-paths src` を明示し、checkout と workspace が同じ場所にあるときの重複探索を避ける。

## 設定・依存・操作の入口

| 対象 | 配置・管理 |
| --- | --- |
| workspace・保存先の設定例 | `config/*.env.example`。共有する既定値・説明を記録する |
| ホスト固有の値 | `config/workspace.env`、`config/storage.env`。ホストごとに保存し、Git 管理対象外 |
| 共通 apt 依存 | `docker/dependencies.apt` |
| 追加 Python 依存 | `docker/python/<purpose>/{pyproject.toml,uv.lock}`。用途別に管理する |
| 初回 setup | `scripts/setup.sh` |
| Docker 起動・build | `scripts/dev.sh` |
| 診断と履歴 | `scripts/docker.sh` |
| 用途別 Python 操作 | `scripts/python.sh` |
| VGL モデルの export・転送・build | `scripts/vgl.sh` |

ユーザー向けの操作は `scripts/` の短い入口から提供する。コンテナ内の処理は `docker/scripts/`、ホスト準備の内部処理は `scripts/setup/`、共通関数は `scripts/lib/` に置く。入口ごとに同じ設定読込みやホスト判定を実装し直さない。

新規のディレクトリ・文書・設定ファイルは、内容が分かる英語の `snake_case` を基本とする。`Dockerfile.<image_key>`、`CMakeLists.txt`、`package.xml` 等のツールが要求する名前と、既存の公開コマンド名はその規則を優先する。

## 文書とハードウェア資料

| 文書 | 担当する内容 |
| --- | --- |
| [README](../README.md) | プロジェクトの入口。詳細手順を重複掲載しない |
| [開発規約](../AGENTS.md) | 開発時に守るルール |
| [要件](project_requirements.md) | 確定要件・提案・未決事項 |
| この文書 | ディレクトリとファイルの置き場所 |
| [セットアップ手順](setup/README.md) | 実際の導入・起動・診断操作 |
| [Python 環境](python_environments.md) | uv、用途別環境、ROS 2 Python との関係 |
| [VGL モデル](vgl_models.md) | Notebook export、SCP 転送、Jetson engine build、検証・ログ |
| [ビルド方針](isaac_ros_build_strategy.md) | CLI・Docker・ROS package の build 方針と技術的な根拠 |
| [環境セットアップ案](environment_setup_plan.md) | Kart の比較調査と、未実装の UI・デバイス等の構成案 |

CAD 等の編集元データは `hardware/` に置き、文書から参照する。`docs/` は説明文書を置く場所とし、編集元 CAD と空の仕様書を混在させない。実測・確定した仕様が増えた時点で、対応する文書を追加する。

## 生成物・データ

| 対象 | 配置・扱い |
| --- | --- |
| CLI 用の生成 Dockerfile | `<ATD checkout>/.cache/docker/<revision>/`。再生成可能、Git 管理対象外 |
| 診断ログ | `<ATD checkout>/logs/docker/<host>/`。履歴を保持し、Git 管理対象外 |
| ROS build 成果物 | `<選択した workspace>/{build,install,log}/`。各ホストで生成する |
| Python venv / cache | `<選択した workspace>/.atd-python/`。各ホスト・アーキテクチャで生成する |
| bag | ホスト別設定の記録先。コンテナでは `/data/record` |
| 地図・モデル・解析・学習結果 | ホスト別設定の結果先。コンテナでは `/data/results` |

bag・地図・モデル重み・TensorRT engine・大きな学習結果をコードのリポジトリへ直接置かない。実装コードと、参照する入力・モデル・成果物の情報を管理する。venv や build 成果物を Notebook と Jetson の間でコピーして利用しない。

自動生成できる cache と bytecode は必要時に再生成する。記録データ・地図・診断ログ・CAD 等の編集元は、cache と同じ扱いで削除しない。

## データの保存構成

保存先の設定と mount は既存の記録先・結果先の二つを使い、その中の用途別配置を次のように定める。新しい環境変数や mount は追加しない。
Notebook では `ATD_PC_RECORD_DIR` / `ATD_PC_RESULT_DIR`、Jetson では `ATD_JETSON_RECORD_DIR` / `ATD_JETSON_RESULT_DIR` がホスト側の root に対応する。設定例の既定値はそれぞれ `~/atd-data/<pc|jetson>/record` と `~/atd-data/<pc|jetson>/results` であり、SSD を使う場合はホストごとの実際のパスを設定する。

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
│   ├── hdmap/                       # HDMap の編集・保存データ
│   └── lines/                       # centerline / raceline / custom line
├── models/
│   ├── vgl/<asset_id>/
│   │   ├── source/                  # Notebook で取得・準備する元モデル・設定
│   │   ├── exported/<export_id>/    # Notebook で作る可搬 export bundle。SCP 転送単位
│   │   └── engines/<target_id>/<build_id>/
│   │                                # Jetson で生成する TensorRT engine の出力構成を維持
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

操作は `scripts/vgl.sh` と [VGL モデルの手順](vgl_models.md)を使う。Notebook の export は専用の Python 処理、Jetson の engine build は公式の `export_lightglue_engine`・`export_extractor_engine` を呼び出す。これらの公式コマンドは TensorRT engine 生成の段階で使い、Notebook の ONNX export と混同しない。localization の `vgl_model_dir` には Jetson で生成した model 資産側の出力先を渡す。[公式 VGL localization](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/visual_global_localization/tutorials/tutorial_cuvgl_localization.html)

engine は対象 GPU・TensorRT 等の環境別に生成・管理する。実装では model / export ID と hash、GPU の識別情報、TensorRT の版、CUDA driver version、関連 package の版、起動構成 hash `atd_runtime_revision` と build ログを対応付ける。Docker image digest 自体は現実装では記録していない。転送する中間表現も Jetson 側の対応演算・依存を確認し、CPU / GPU 間での利用を検証済みとは扱わない。[TensorRT の engine 互換性](https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/support-matrix.html#software-and-system-requirements)
地図には元 bag と使用 model の ID、座標系・較正等の対応を持たせる方針とし、地図用の管理ファイルの名前・schema は実装時に決める。
`source/` の入力モデル、`exported/` の転送 bundle、`engines/` の生成成果物はいずれも保持する資産であり、通常の cache 削除には含めない。`runs/` のログ・checkpoint も同様に保持する。

E2E 用の配置は将来の保存先の予約であり、framework、重み・ONNX の形式、学習・export 手順を採用済みとするものではない。VGL の処理入口と資産・hash 検査は実装しているが、対象機でのモデル取得・ONNX export・実 SCP・engine build・localization は未検証である。地図作成も実機検証は未実施である。
