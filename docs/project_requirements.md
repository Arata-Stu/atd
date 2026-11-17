# ATD プロジェクト要件・決定事項

最終更新: 2026-11-17

この文書は、要件・規約と Docker 開発環境の初期実装方針をまとめたものである。
「確定要件」はユーザーが明示した内容、「技術調査メモ」は会話中に確認した一次情報、「提案・候補」は採用未確定の内容を示す。
設定・スクリプトの作成状況は記録するが、ホストへの導入、Docker build、実機での動作確認の完了とは区別する。

## 1. 目的と初期構成

- RC カーを用いた自動運転を実現する。
- 第一想定のセンサ構成は RealSense D455 のみとする。
- イベントカメラは今後使用しない。関連依存は環境構築・将来の追加候補から除外する。
- 実走行は Jetson を搭載した RC カーで行う。
- 地図作成・編集・解析・走行情報の可視化は作業用ノート PC で行う。
- プロジェクトの規約と要件を記録し、初期の Docker 開発環境を整備する。
- 自動運転ソフトウェアの詳細は後から検討する。走行目標やアルゴリズムの確定を、現在の環境整備・共通規約の定義の前提条件にしない。

## 2. 確定要件

### 2.1. ハードウェアと想定環境

| 項目 | 内容 | 補足 |
| --- | --- | --- |
| 車載計算機 | Jetson Orin Nano 8GB | 実走行の処理と制御を担当 |
| 初期センサ | RealSense D455 のみ | IR（infra）、RGB、IMU、Depth を利用候補として残す |
| 作業用ノート PC | Ubuntu 24 系、NVIDIA RTX 4060 | 共通の OS 基準は Ubuntu 24.04。地図作成・編集・解析・可視化を担当 |
| ストレージ | 1TB SSD を利用可能 | 搭載先、マウント先、空き容量は環境構築時に記録する |
| OS 基準 | Ubuntu 24.04 | 2026-11-16 時点の構成想定 |
| JetPack | 7.2 | 車載 Jetson の想定基盤 |
| ROS 2 | Lyrical | 詳細バージョンは環境構築時に記録する |
| Isaac ROS | v5 | 詳細バージョンは環境構築時に記録する |
| 実行環境 | Isaac ROS v5 の Docker を前提とする | ホストへの直接導入を標準構成にしない |

### 2.2. 処理の配置と通信範囲

- 認識、自己位置推定、計画、制御、車両 I/O など、実走行に必要な処理は Jetson 内で完結させる。
- PC から遠隔で走行制御を与えない。走行制御に PC との通信を必須としない。
- 車載系の ROS 2 / DDS の直接探索・topic・service・action 通信を Jetson 内部に限定し、外部ネットワークへの意図しない流出を防ぐ。
- ノート PC から自己位置等の情報を観測するため、Foxglove Bridge 等を利用する予定である。bridge を介して明示的に提供する情報の外部通信は許容する。
- PC での可視化に、PC を車載 DDS の参加者にする構成は前提としない。走行処理は PC との接続なしでも Jetson 内で完結させる。
- Docker の外部通信を一律に遮断する要件ではない。必要なら `--network host` を維持する。
- DDS 実装、UDP 等の転送方式、QoS、DDS の通信範囲、bridge の公開範囲は別の項目として定義する。

### 2.3. パッケージ・ビルド・実装方針

- ROS 2 パッケージはモジュールごとに分割する。
- 自作パッケージは `atd_<pkg_name>` と命名する。
- C++ を基本とし、効率を重視して実装する。
- ビルドは `CMakeLists.txt` と `ament_cmake_auto` を原則として用いる。
- composable node / component container の利用を推奨する。
- 特に大容量データを交換する処理は、composable な構成を想定して実装する。
- Python node を作成する場合も、CMake と `ament_cmake_auto` を基本とする。
- Python の直接起動用スクリプトは実行権限に注意し、`colcon build` 後に実行できる形で配置・確認する。

### 2.4. センサ・手動介入・車両 I/O の方針

- D455 の IR（infra）、RGB、IMU、Depth はいずれも使用する可能性を残し、RealSense driver で起動する方針とする。
- 実際に使用するストリームの組み合わせ、解像度、周期、bag の記録対象は後から決める。全ストリームの常時同時起動・記録を必須とはしない。
- Joy による手動介入を想定し、DualSense を Bluetooth で Jetson に接続する予定とする。
- Joy の操作割り当て、手動・自律の切替条件、停止・再開条件は後から決める。
- Joy の入力監視と軸・ボタン配置の調整は、地図・転送・ライブ表示と同じ Web アプリから行えるようにする。プロポへの対応も将来の検討対象とし、機種・入力経路・切替条件は固定しない。
- ESC・ステアリングサーボの制御に PCA は使わず、独自 firmware で対応する予定とする。
- firmware を実行するマイコン、Jetson との通信方式、出力経路、異常時の動作等の詳細は未決定である。

### 2.5. Isaac ROS CLI と Docker の導入方針

- 初回準備の入口は `scripts/setup.sh` とする。fzf のメニューを優先し、未導入の初回は番号選択を利用する。
- Notebook（Ubuntu 24.04 / x86_64・amd64）と Jetson（JetPack 7.2 / aarch64・arm64、BSP マーカー）を区別してホスト準備を行う。`--host notebook` は内部の `pc` 設定へ対応する。
- clone の標準配置は `~/workspaces/atd`、未設定時の ROS workspace は実際の clone 配下の `ros2_ws` とする。標準配置なら `~/workspaces/atd/ros2_ws` となる。workspace 操作で対話的に指定し、`config/workspace.env` の `ATD_PC_ROS_WS` / `ATD_JETSON_ROS_WS` に別々に保存する。`--workspace` での直接指定にも対応する。Docker 内は `/workspaces/isaac_ros-dev` とし、両ホストの build / install は別々に扱う。
- bag・地図・モデル等は clone 配下の `data/record`・`data/results` を未設定時の保存先とする。`data/` は Git 管理対象外の永続資産として保持する。既存の保存済み指定、引数・環境変数の上書きを優先し、旧設定の変更と既存データの移動は自動実行しない。新しい配置は setup の workspace / storage 操作で明示的に選び直す。
- 自作 ROS package の編集元はリポジトリの `ros2_ws/src/atd_<module>/` とする。別の workspace を使う場合はコンテナの workspace `src/atd/` に rw で mount する。CLI 設定テンプレートは `docker/isaac_ros/`、生成 Dockerfile は Git 管理対象外の `.cache/docker/` に置く。配置の詳細は [構成規約](project_structure.md)に従う。
- `scripts/docker.sh` でセットアップ状態を項目ごとの `OK` / `FAIL` / `SKIP` として表示する。結果は `logs/docker/<host>/` に保存し、後から対話的に確認できるようにする。診断だけではホストへの導入や Docker 起動を行わない。
- Isaac ROS CLI は apt 版を採用する。
- Docker のカスタムは Isaac ROS CLI の YAML と追加 Dockerfile で対応する。
- RealSense は標準の `realsense` layer をローカルの `realsense455` layer に置き換え、`isaac_ros → realsense455 → atd` の順で構築する。librealsense SDK のみ CUDA 無効とし、ベースの CUDA や他の Isaac ROS GPU 処理は無効化しない。
- CLI 自体の source build を必要とする具体的な改修は、現時点では特定されていない。以前の `kart` の mount・起動処理の変更を ATD に移植することも未決定である。
- source build の追加コストは必要に応じて許容するが、用途を特定せず CLI 全体の source 運用を必須にしない。
- CLI の source build、Docker イメージの build、個別 ROS パッケージの source build を区別する。Isaac ROS 全パッケージの source build は決定していない。
- 起動時に PC / Jetson を自動判定し、記録先と解析結果の保存先をホストごとに選択する。必要な場合だけ `--host pc|jetson` で上書きできるようにする。
- 記録先と解析結果は毎回読書き可能で mount する。記録・解析による起動時の切替は不要とする。ホスト上の実際の保存先は環境ごとに設定する。
- データ整理のため記録先と解析結果の保存先を別々にすることを推奨するが、ディレクトリの同一・親子関係を起動条件として制限しない。
- 共通依存と DDS 設定は、Docker 起動時に root の `.sh` から自動適用する。
- 依存一覧 `docker/dependencies.apt` を基準に不足パッケージだけを apt で導入し、揃っている場合は apt 操作を行わない。毎回の apt upgrade は行わない。
- Docker build 時にも同じ依存導入スクリプトを実行し、既知の依存をキャッシュする。D455 SDK / driver は既存の `realsense455` build 層で準備する。
- 稼働中コンテナへの再接続では依存や DDS を更新せず、起動時構成との整合を確認する。構成が変わった場合や旧コンテナは、処理終了後に再作成する。
- 対話シェルは ATD の `atd-shell` を使用し、ホストの旧 DDS 設定を含む `.bashrc` に依存しない。

比較根拠、実装内容と起動方法: [Isaac ROS CLI・Docker のビルド方針](isaac_ros_build_strategy.md)

### 2.6. 記録・地図作成・転送の流れ

1. RC カーを手動走行させ、Jetson で bag を記録する。
2. 記録した bag をノート PC へ転送する。
3. ノート PC で offline の地図作成を行う。
4. 作成した地図を Jetson へ転送する。

地図作成は NVIDIA の公式 offline 処理を利用し、cuVSLAM / VGL で使う地図を準備する。HDMap の作成・編集と centerline / raceline / 必要に応じた custom line の生成も共通 Web アプリで扱う。
bag の Jetson → ノート PC、作成地図のノート PC → Jetson の転送は、SCP を簡単に実行できる機能として提供する。

VGL モデルの準備・配備は、次の分担とする。

1. Notebook でモデルを準備・export し、Jetson の builder 入力に必要なモデル・設定・出自と hash 等の管理情報をまとめる。
2. export 成果物の bundle を SCP で Jetson へ転送する。
3. Jetson では、その資産から対象 GPU 用の TensorRT engine を build する。VGL モデルの準備・export は Notebook 側で完結させる。

保存先は [構成規約](project_structure.md#データの保存構成)に従い、元の入力、export 成果物、対象環境別 engine を分ける。操作は `scripts/vgl.sh` の fzf メニューから export・検証・SCP 転送・Jetson build へ進める。初期実装の対応モデル・形式とコマンドは [VGL モデルの手順](vgl_models.md)に記録し、対象 GPU / SDK と実 SCP を含む通し動作は未検証として扱う。

bag の記録対象や各 bag の topic / frame は実データに合わせて確認する。HDMap の初期 schema とライン生成方式は Web アプリの README に記録し、走行開始時の自己位置推定や実地図・実車に応じた方式の具体化は後工程で検討する。

### 2.7. シェルスクリプト・TUI・ブラウザ UI の操作方針

- ノート PC と Jetson の日常操作では、シェルスクリプト（`.sh`）や UI を基本とし、ターミナルコマンドを繰り返し手入力する負担を減らす。
- ユーザーは CUI であっても対話形式や TUI を好む。ユーザー向けの CLI・補助スクリプトでは、この操作方針を優先する。
- `fzf` を用いた TUI を積極的に活用し、候補を検索・選択して対話的に実行できるようにする。
- 対象となる操作の例は、実行する処理、bag、地図、設定の選択である。具体的な画面や操作は各機能の実装時に設計する。
- ブラウザ UI の作成も積極的に行う。主な用途は地図の編集、自己位置の確認、PC / Jetson の操作・状態確認とする。
- UI はノート PC の Chrome のみ で `http://127.0.0.1:<port>` を開いて利用する。ここでの `127.0.0.1` はブラウザを実行するノート PC の loopback を指す。
- Jetson では UI のブラウザ・frontend を起動しない。これは Jetson 側で必要なバックエンドや bridge を動かさないという意味ではない。
- UI の追加でも、走行処理の Jetson 内完結、PC からの遠隔走行制御を行わないこと、車載 DDS の直接通信範囲の要件を維持する。
- 基本機能は 2.9 節の地図・転送・ライブ表示・入力設定とする。UI サーバーの配置、ライブ観測・設定用の Jetson 接続方法、ポート、画面の詳細は実装時に決める。
- setup・Docker・Python・VGL に加えて Web UI と Jetson 入力の起動入口を実装した。UI のローカル検証と Docker / ROS / GPU / 実機での結合確認は分けて扱う。

### 2.8. Docker 内の Python 環境

- Docker 内の Python 環境構築は uv を基盤とする。
- 用途ごとに Python 環境を分離し、ROS 2 用の追加依存と AI 学習用の依存を別の環境で管理する。
- 具体的な学習 framework、Python / CUDA wheel の組み合わせや、その他の用途別環境は必要になった段階で決める。

Notebook の VGL export は `vgl-export` の独立した固定依存を持つ。これは初期実装の用途別環境であり、AI 学習用 framework の採用とは分ける。

用途別環境の構成・操作・実装上の制約は [Python 環境](python_environments.md) を参照する。ROS 2 package の CMake / `ament_cmake_auto` による build 方針は維持する。

### 2.9. 共通 Web アプリの基本機能

以前の `kart` のシステムと作業の流れを基本として、次の 4 機能を一つのアプリで扱う。機能の採用と、旧コード・framework・設定値の一括移植は区別する。React + TypeScript / Vite と FastAPI の採用を確定し、`apps/web/` に初期実装した。UI はノート PC のみで使用する。動作・制約と検証境界は [Web アプリ](../apps/web/README.md)を参照する。

| 機能 | ユーザーが明示した範囲 |
| --- | --- |
| 地図 | rosbag から NVIDIA 公式処理による cuVSLAM / VGL 用地図の準備、HDMap の作成・編集、centerline / raceline / 必要に応じた custom line の生成 |
| Jetson との転送 | SCP を簡単に実行する UI。Jetson → ノート PC の rosbag 受信、ノート PC → Jetson の作成地図送信 |
| ライブ表示・調整 | bringup 実行中の観測。地図走行時の地図と自己位置推定の表示、ROS 2 parameter の動的調整、速度等の時系列プロット |
| 入力監視・設定 | Joy の入力監視と軸・ボタン配置調整を共通アプリへ統合。プロポ対応は将来検討 |

Joy は Jetson に接続し、ノート PC のブラウザでその入力を監視・設定する。車載 ROS 2 / DDS への直接参加を PC に要求しない。
parameter の調整や入力配置の設定は、PC から操舵・スロットルの連続指令を送る遠隔走行制御とは区別する。走行処理は引き続き Jetson 内で完結させる。
動的変更の対象 parameter、値の範囲、変更可能な車両状態、保存・再起動後の扱い、Joy 設定の反映・出力再開条件は未決定である。
bringup 実行中の可視化は今回の要件に含むが、アプリからの bringup 起動・終了操作の範囲は別途具体化する。

## 3. 技術調査メモ

以下は 2026-11-16 の会話中に参照した一次情報と、ローカルの実装・調査メモである。ユーザーの過去の報告、設定の実装、実機での動作確認は区別する。

### 3.1. 環境と C++ 規格

- Isaac ROS 5 の公式対応表には Jetson Orin / JetPack 7.2、x86_64 / Ubuntu 24.04、ROS 2 Lyrical が記載されている。
- Ubuntu 24.04 向け Lyrical のパッケージは NVIDIA の Isaac ROS Buildfarm を利用する構成である。
- Lyrical の最低 C++ 要件は C++20。環境構築時に実際のコンパイラ・依存パッケージとの整合を確認する。

参照: [Isaac ROS 対応環境](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#supported-platforms)、[Buildfarm](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/isaac_ros_buildfarm_cdn.html)、[Lyrical の言語要件](https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Releases/lyrical/supported-platforms.rst)

### 3.2. D455 と地図生成

- Isaac ROS 5 のリリースノートでは RealSense のサポートが Docker モードに限定されている。
- 同リリースの Nvblox の D455 サンプルには、深度・カラー画像が統合されず地図が空になる既知問題が記載されている。
- この問題を D455 を使う全処理の非対応と解釈しない。Nvblox 自体の採用も未決定である。
- カメラ firmware、librealsense、realsense-ros は互換性のある組み合わせを確認し、環境構築時に版を記録する。
- ATD の `realsense455` layer は SDK `v2.56.3` / driver `r/4.56.3` と RSUSB、Lyrical パッチ、driver の Debian パッケージ構築を維持し、SDK build に `--no_cuda` を指定する。設定の実装と実機検証は区別する。
- ユーザーが報告した過去の CUDA 有効構成での起動失敗について、CUDA が原因であったことや、この変更で解消することは未確認である。

参照: [Isaac ROS 5 リリースノート](https://nvidia-isaac-ros.github.io/v/release-5.0/releases/index.html)、[RealSense セットアップ](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/sensors/realsense_setup.html)、[ATD の RealSense layer](../docker/Dockerfile.realsense455)

### 3.3. DDS と CUDA バッファ

- ユーザーの当初候補は Cyclone DDS と UDP によるローカル通信であった。DDS の直接通信を Jetson 内部に限定することは必須要件である。bridge 経由の情報提供は許容する。
- Isaac ROS 5 は `rosidl::Buffer` と CUDA buffer backend を利用する。従来の NITROS API をそのまま前提にしない。
- 会話中に確認した `rmw_cyclonedds_cpp` の公式リポジトリでは、非 CPU バッファの CPU fallback 対応 PR が未マージであった。このため Fast DDS を初期標準の候補として提案した。
- その後の実装指示に基づき、初期構成として Fast DDS の loopback UDP 設定を実装した。実機通信の確認は未実施である。
- CUDA buffer backend は条件を満たす同一ホスト内の通信で GPU メモリを共有する。DDS の UDP 設定と CUDA バッファの共有は別の層として扱う。

参照: [NITROS からの移行](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/rosidl_buffer/nitros_migration.html)、[CUDA buffer backend](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/rosidl_buffer/cuda_buffer_backend.html)、[Cyclone DDS の対応 PR #603](https://github.com/ros2/rmw_cyclonedds/pull/603)

### 3.4. Fast DDS の XML 設定

- Fast DDS でも XML で通信インターフェース、transport、探索、バッファ、QoS 等を設定できる。
- Lyrical の構成例では `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` と `FASTDDS_DEFAULT_PROFILES_FILE` で設定ファイルを指定する。
- UDP transport の `interfaceWhiteList` により loopback に限定できる。探索と既定 transport も含めて設定する必要がある。
- ROS 側で明示した QoS は XML より優先される。transport の XML 設定を読むだけなら `RMW_FASTRTPS_USE_QOS_FROM_XML=1` は不要である。
- 探索を XML で管理する場合は、ROS の discovery 環境変数との干渉にも注意する。

実装した構成:

- Fast DDS / UDPv4 を標準とし、XML の通信インターフェース、探索 peer、unicast locator を `127.0.0.1` に限定する。
- `10-atd-runtime.sh` が依存導入後に `configure-dds.sh` を実行し、`/etc/atd/fastdds.xml` と `/etc/atd/dds.env.sh` を配置する。
- Docker ENV と `atd-shell` で設定を利用し、旧 Cyclone DDS 等の設定が対話シェルから混入することを防ぐ。
- topic ごとの QoS は別途設計する。XML の実装と、実機での通信範囲の検証完了は区別する。

参照: [Lyrical の rmw_fastrtps](https://github.com/ros2/rmw_fastrtps/blob/lyrical/README.md)、[インターフェースの制限](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/whitelist.html)

### 3.5. CLI の拡張と以前の構成

- 公式の apt 版 CLI でも YAML 設定と追加 Dockerfile によるローカル build が可能である。Docker のカスタムだけを理由に CLI source build が必須とはしない。
- 以前の `kart` ブランチは Isaac ROS 5 / Lyrical 向けで、主な独自変更はワークスペースの mount、作業ディレクトリ、ホスト設定の共有方法である。
- 同ブランチの起動処理は `--network host` を使用する。host network の使用自体は ATD の要件違反ではなく、DDS の通信範囲と bridge の外部通信を別に設定・検証する。

参照: [公式 Development Environment](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/dev_env/index.html)、[比較・ソース確認の詳細](isaac_ros_build_strategy.md)

### 3.6. Foxglove Bridge と DDS の通信範囲

- Foxglove Bridge は Jetson 側の ROS 2 topic を購読し、PC へ WebSocket で情報を提供できる。PC 側に車載 DDS との直接通信を要求しない。
- host network と、DDS の loopback 制限、bridge の外部向け WebSocket 待受は両立する設計が可能である。ATD での設定・検証は未実施である。
- localhost 向けの自動探索設定だけを、全 DDS データの外部流出防止の証拠にはしない。使用する DDS の transport・interface・locator 等の設定と、実際の外部インターフェースの通信を確認する。

参照: [Foxglove Bridge](https://docs.foxglove.dev/docs/fleet/bridge)、[Docker host network](https://docs.docker.com/engine/network/drivers/host/)、[ROS 2 Lyrical の探索設定](https://github.com/ros2/ros2_documentation/blob/lyrical/source/Developer-Tools/Launch/Improved-Dynamic-Discovery.rst)

### 3.7. kart の地図処理と Joy UI の確認

- kart の地図作成は `ros2 run isaac_mapping_ros create_map_offline.py` を呼び、`edex / compute_poses` を実行する。NVIDIA の公式リポジトリにも同名スクリプトがある。
- kart では cuVSLAM 保存地図、VGL bundle、HDMap 背景用の点群取得、HDMap 編集・ライン生成を別工程として実装している。ATD での具体的な工程・引数・モデル・資産形式は対象環境で確認する。
- kart の Joy GUI は Studio とは別アプリで、Jetson の生入力・状態を表示し、プロファイルを保存・再読込する。入力取得・正規化と、車両動作への割り当ては別の責務である。
- kart の設定モードや再開条件、地図 schema、ライン生成方式・既定値は参考実装として扱い、ATD で採用済みとはしない。今回の確認はソースの読み取りであり、ROS / GPU / 実 bag / 実機での実行検証は行っていない。

参照: [NVIDIA create_map_offline.py](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mapping_and_localization/blob/main/isaac_mapping_ros/scripts/create_map_offline.py)、[kart 地図作成](https://github.com/Arata-Stu/kart/blob/e5224221733dd4271d7132d8febcb80e91916063/ros2_ws/src/kart_mapping/kart_mapping/build_map.py)、[kart VGL 準備](https://github.com/Arata-Stu/kart/blob/e5224221733dd4271d7132d8febcb80e91916063/ros2_ws/src/kart_bringup/kart_bringup/prepare_vgl_map.py)、[kart Joy](https://github.com/Arata-Stu/kart/blob/e5224221733dd4271d7132d8febcb80e91916063/ros2_ws/src/kart_joy/README.md)

### 3.8. VGL モデル処理の初期実装

- Notebook の標準 export は ALIKED-n16 を FP32 / opset 17 の ONNX に変換し、専用 uv 環境 `vgl-export` を使う。既存 NVIDIA ONNX の package モードは再export と区別する。
- export bundle は `models/`・`configs/`・`manifest.json` を含み、ID と全ファイルの hash / size を検証する。SCP では bundle 全体を送り、転送先で検証後に正式な保存先へ反映する。
- Jetson は公式 `export_extractor_engine`・`export_lightglue_engine` を engine 生成に使う。Notebook の可搬 export と、この engine build を別工程として実装している。
- `scripts/vgl.sh` に診断・資産一覧・検証・ログ確認を含む対話操作を用意した。接続先はホストの `config/vgl.env` に保存する。具体的な操作と検証範囲は [VGL モデルの手順](vgl_models.md)を参照する。
- これらは処理・操作契約の実装であり、実 GPU / TensorRT / Isaac ROS 5 SDK での数値互換性、実 SCP、localization の動作確認は未実施である。

### 3.9. 共通 Web アプリの初期実装

- React + TypeScript / Vite の frontend、FastAPI の backend、Web 非依存の geometry / ライン生成を `apps/web/` に実装した。framework と UI の Notebook 専用配置は採用済み。
- Map / Transfer / Live / Input を統合する。地図・転送処理は独立 worker で実行し、結果先の `runs/web/` に設定・ジョブ状態・ログを保持する。
- 地図作成は公式 `create_map_offline.py`、VGL 地図は公式 builder / `create_cuvgl_map.py` を呼ぶ。apt 導入、モデル資産、GPU engine、地図成果物を別に扱う。表示用点群の自動 snapshot 採取は未実装で、map 座標 JSON の明示インポートを提供する。
- Jetson 側に C++ component の `atd_joy` と、観測・設定用の `atd_web_bridge` を追加した。Notebook API は SSH から Jetson loopback HTTP へアクセスし、車載 DDS に直接参加しない。
- Web UI は Notebook の Docker 内で `atd-web` を実行して起動し、ブラウザだけを Notebook ホストで開く。ホスト用の `scripts/webui.sh` は `docker exec` による補助入口とする。Jetson の入力起動は `scripts/input.sh`、手順・初期 schema・生成方式・制約は [Web アプリ](../apps/web/README.md)と package の README に記録する。
- Docker build、ROS build / install、実 bag / GPU での公式処理、実 SCP、Bluetooth / evdev、実走行は対象機で別に確認する。ローカルのコード検証だけでこれらを検証済みとはしない。

## 4. 提案・候補（未確定）

| 対象 | 提案内容 | 状態 |
| --- | --- | --- |
| ROS 依存の取得 | 変更しないパッケージ・システム依存は apt、自作 `atd_*` とソース・コンパイル条件を変更する対象は source build | 共通 apt 依存は manifest と起動スクリプトを実装済み。追加の個別パッケージは未選定 |
| 高速データ経路 | component 間の intra-process 通信や CUDA バッファ共有を利用する | 個別処理の設計・実測は後工程。DDS の UDP / loopback 設定は実装済み |
| Docker と通信範囲 | host network と DDS の loopback 制限を併用し、Foxglove Bridge は PC 向けに待受する | DDS の XML は実装済み・実通信は未検証。bridge の個別設定は後工程。`network_mode: none` は標準としない |
| bridge の公開範囲 | 観測する topic と接続範囲を明示する。PC から走行制御を与えない要件に合わせて書込み機能の扱いを決める | Foxglove Bridge 等の利用は予定済み。個別設定は未決定 |
| XML と QoS の管理 | DDS ネットワーク設定は XML、topic ごとの QoS はコード・パラメータを基本とする | ネットワーク XML は実装済み。topic ごとの QoS は後工程 |
| 指令の優先順位 | 非常停止・駆動系の重大異常 ＞ 手動 ＞ 自律 | 提案。異常の種類ごとの許可動作は別途定義 |
| Joy 途絶 | 手動操作中の切断・入力期限切れで停止し、自律へ自動復帰させない | 提案。停止・再開条件未決定 |
| プロセスの配置 | 車両 I/O・watchdog を高負荷の画像処理と別プロセスにする | 提案 |
| 通信契約 | 単位、座標系、TF 発行元、時刻、topic 型、QoS、周期、許容遅延を文書化する | 提案。個別仕様未決定 |
| 再現性 | Docker digest、依存 commit、較正、設定、地図、bag の対応を記録する | 提案 |
| bag 再生 | 実機デバイスを渡さず、車両 I/O を起動しない解析専用構成を用意する | 提案 |

ネットワーク方式の参照: [Docker の host network](https://docs.docker.com/engine/network/drivers/host/)

## 5. 今後の検討事項

### 5.1. 環境・共通基盤の具体化に合わせて決める項目

- Fast DDS の loopback UDP 設定について、探索・データ通信と外部インターフェースでの実通信を検証する。
- Foxglove Bridge 等で PC に公開する観測情報と接続設定。
- ノート PC の Chrome で利用するブラウザ UI の提供方法、サーバー配置、Jetson 側バックエンドとの接続方法。

### 5.2. 後工程で検討する項目

以下は自動運転ソフトウェアや車両制御を具体化する段階で決める。現時点の環境整備を進めるための回答必須事項にはしない。

1. 独自 firmware 用のマイコン、Jetson との通信方式、ESC・サーボへの出力経路、Jetson 停止・通信断時の動作。
2. 初期の走行場所・路面・目標速度・到達目標、障害物に対する停止 / 回避の範囲。
3. D455 の各ストリームの利用・記録設定、撮影モード、取付姿勢と較正。
4. DualSense の操作割り当て、手動 / 自律の切替と停止・再開条件、UI からの配置変更の反映条件。将来のプロポ対応機器・入力経路。
5. cuVSLAM / VGL の公式地図作成の入力・手順、HDMap schema、ライン生成方式、走行開始時の自己位置推定。VGL モデル処理の対象機での互換性、SCP の実接続と転送後の利用、成果物の対応管理。
6. 個別パッケージの責務、通信契約、性能目標と検証手順。
7. ライブ表示の項目・周期・時刻と座標系、動的 parameter の対象・範囲・適用条件・永続化。

## 6. 文書の配置

| 文書・場所 | 役割 |
| --- | --- |
| [AGENTS.md](../AGENTS.md) | 開発・調査・検証時に守る規約 |
| この文書 | 要件、機材、想定環境、提案と未決事項の参照元 |
| [project_structure.md](project_structure.md) | ROS package、UI、firmware、設定、文書、生成物の配置規約 |
| [setup/README.md](setup/README.md) | Notebook / Jetson の初回準備、対話メニュー、workspace と保存先設定 |
| [isaac_ros_build_strategy.md](isaac_ros_build_strategy.md) | CLI の apt / source 比較、以前の fork の確認結果、Docker・ROS のビルド範囲 |
| [environment_setup_plan.md](environment_setup_plan.md) | `kart/docker` の比較、ホスト / Docker の分担、依存追加、TUI / UI 配置、段階的なセットアップの提案 |
| [python_environments.md](python_environments.md) | uv による用途別 Python 環境、ROS 2 の ABI・依存管理、AI 学習環境との分離 |
| [vgl_models.md](vgl_models.md) | Notebook の ONNX export、SCP 転送、Jetson の engine build と検証・ログ |
| [hardware/cad/tt02-v1.FCStd](../hardware/cad/tt02-v1.FCStd) | 保存する車体 CAD の編集元。実物との一致は未検証 |
| `../docker/` | 追加 Dockerfile とデータを毎回 mount する起動引数 |
| `../docker/isaac_ros/` | 選択した workspace から参照する CLI 設定テンプレート |
| `../ros2_ws/src/` | 自作 `atd_*` package の編集元。build 成果物はホスト別 workspace に作成する |
| `../scripts/dev.sh` | PC / Jetson を自動判定する開発環境の起動スクリプト。必要に応じて手動上書き可能 |
| `../scripts/docker.sh` | Docker 開発環境の診断、結果の保存・閲覧、起動・build の入口 |
| `../config/workspace.env.example` | PC / Jetson ごとの ROS workspace 設定例 |
| `../config/storage.env.example` | PC / Jetson ごとの記録先・解析結果保存先の設定例 |

## 7. 更新履歴

- 2026-11-17: React + TypeScript / Vite と FastAPI の採用、UI の Notebook 専用利用を確定。共通 Web アプリ、Jetson の入力・観測設定 bridge、Docker 追加依存と起動入口を初期実装し、未検証の実機項目を区別した。

- 2026-11-17: clone を `~/workspaces/atd`、ROS workspace をその `ros2_ws`、bag・地図・モデル等を `data/` 以下へまとめる配置を選択した。未設定時の値は実際の clone 位置を基準にし、PC / Jetson 別の保存済み指定と上書き指定を優先する。`data/` は Git 管理外の永続資産で、旧設定・データは自動移行しない。
- 2026-11-17: VGL の二段階処理を `scripts/vgl.sh` として実装した。ALIKED-n16 ONNX export と既存 bundle の package を分け、manifest 検証、SCP、Jetson の公式 engine builder、診断・ログの入口を追加した。対象機の GPU / SDK・実 SCP・localization の通し検証は未実施。
- 2026-11-17: VGL モデルは Notebook で準備・export し、export 成果物を SCP 転送して Jetson では TensorRT engine build のみを行う分担を先に確定した。保存先へ `exported/<export_id>/` を追加し、方針定義・処理の実装・実機検証を分離した。
- 2026-11-16: 構成規約を定義し、ROS 編集元を `ros2_ws/src/`、CLI テンプレートを `docker/isaac_ros/`、生成物を `.cache/docker/`、CAD 編集元を `hardware/cad/` へ整理した。README の操作説明は詳細手順へ集約し、空の仕様書・仮置きと再生成可能な cache を整理した。
- 2026-11-16: kart を基本にした共通 Web アプリの 4 機能を要件へ追加した。公式 cuVSLAM / VGL 地図の準備、HDMap とライン、双方向 SCP、ライブ表示・parameter 調整、Joy UI 統合を対象とし、プロポは将来検討とした。React / TypeScript / FastAPI 等の構成は提案として分離した。
- 2026-11-16: Docker 内の Python 環境構築に uv を使い、ROS 2 用と AI 学習用を用途ごとに分ける要件を追加した。

- 2026-11-16: 会話から初版を作成。ユーザーが明示した要件、技術調査メモ、未確定の提案を分離した。
- 2026-11-16: CLI の source build と Docker のローカル build の採用方針、apt との使い分け案、以前の `kart` ブランチの調査結果を追記した。
- 2026-11-16: ユーザー補足を反映。bridge による PC への観測情報提供を許容し、DDS の直接通信制限と区別した。CLI source 運用を必須とした整理を修正し、YAML・追加 Dockerfile と apt 版 CLI を基本とする提案へ見直した。
- 2026-11-16: apt 版 CLI の採用を確定し、PC / Jetson と record / analysis を切り替える初期 Docker 構成・保存先設定・起動スクリプトを追加した。Docker build と実機検証は未実施。
- 2026-11-16: ユーザー指定により記録・解析の起動切替を廃止し、記録先・解析結果を毎回読書き可能で mount する構成へ統一した。保存先の同一・親子関係の制限も撤去した。
- 2026-11-16: ユーザー指定により PC / Jetson の指定を任意とし、機種情報・BSP のマーカーによる自動判定を標準にした。`--host` は手動上書き用として残す。実 Jetson 上での判定は未検証。
- 2026-11-16: 自動運転ソフトウェアの詳細を後工程の検討事項へ移した。PCA を使わない独自 firmware 方針、DualSense / Bluetooth、D455 の IR・RGB・IMU・Depth を利用候補に残す方針、手動走行での bag 記録から PC での offline 地図作成・Jetson への地図転送までの流れを反映した。
- 2026-11-16: apt 版 CLI を維持し、標準 RealSense layer を `realsense455` に置き換えた。librealsense SDK の CUDA のみ無効化し、補助スクリプト・driver 構築は公式構成から再利用する。過去の起動失敗の原因・解消、実 Docker build・D455 性能は未確認。
- 2026-11-16: ユーザーの操作上の好みを反映し、CLI・補助スクリプトで対話形式・TUI を優先し、`fzf` を積極的に活用する方針を追加した。
- 2026-11-16: 日常操作に `.sh`・TUI・ブラウザ UI を積極的に用意する方針を追記した。UI はノート PC の Chrome から `127.0.0.1` で利用し、地図編集・自己位置確認等を行う想定。Jetson 自身では通常ブラウザを開かず、サーバー配置や接続方法は実装時に決める。
- 2026-11-16: イベントカメラを今後の対象から完全に除外した。Docker 起動時の root スクリプトによる不足依存の導入と Fast DDS の loopback UDP 設定を実装した。依存の build 時キャッシュ、`atd-shell` と再接続時の整合確認を追加し、実機導入・実 Docker build・実通信検証とは区別した。
- 2026-11-16: `setup.sh` とホスト別セットアップ手順を追加した。Notebook / Jetson の区別、fzf 対話メニュー、ホスト準備、保存先と ROS workspace の確認を入口へまとめた。ホストへの実導入は未実施。
- 2026-11-16: ROS workspace のパスを対話入力し、Notebook / Jetson 別に保存する方針を追加した。当時の workspace 既定は `~/workspaces/atd` 自体で、2026-11-17 に clone 配下の `ros2_ws` へ見直した。`docker.sh` による項目別診断とログの保存・再表示を追加した。対象ホストへの通し導入・実 Docker build は未実施。
