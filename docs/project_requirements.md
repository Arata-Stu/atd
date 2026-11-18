# ATD プロジェクト要件・決定事項

最終更新: 2026-11-18

この文書は、要件・規約と Docker 開発環境の初期実装方針をまとめたものである。
「確定要件」はユーザーが明示した内容、「技術調査メモ」は会話中に確認した一次情報、「提案・候補」は採用未確定の内容を示す。
設定・スクリプトの作成状況は記録するが、ホストへの導入、Docker build、実機での動作確認の完了とは区別する。

## 1. 目的と初期構成

- RC カーを用いた自動運転を実現する。
- Tamiya 車両の Bluetooth コントローラによる手動運転から開発を進め、まず Jetson で求めたパルス幅指令を基板へ送信することを目標にする。パルス波形の生成は基板側で行う。
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
| 車両 I/O 基板 | Raspberry Pi Pico ベース。受信機の S.BUS を入力し、Jetson とは USB 接続 | 参照 firmware は JPBB Rev6 / RP2040 版 Pico 用。実基板での確認は別途必要 |
| プロポ | 6 チャンネルを基本とし、将来の10ch導入も可能性あり | Pico の CH1＝操舵・CH2＝スロットル・CH3＝操作権は固定。全16chを受信・保持し、CH4〜CH16 は Jetson 側の記録・予備役割を変更可能。予備2枠の用途は未定 |
| 初期センサ | RealSense D455 のみ | IR（infra）、RGB、IMU、Depth を利用候補として残す |
| 作業用ノート PC | Ubuntu 24 系、NVIDIA RTX 4060 | 共通の OS 基準は Ubuntu 24.04。地図作成・編集・解析・可視化を担当 |
| ストレージ | 1TB SSD を利用可能 | 搭載先、マウント先、空き容量は環境構築時に記録する |
| OS 基準 | Ubuntu 24.04 | 2026-11-16 時点の構成想定 |
| JetPack | 7.2 | 車載 Jetson の想定基盤 |
| ROS 2 | Lyrical | 詳細バージョンは環境構築時に記録する |
| Isaac ROS | v5 | 詳細バージョンは環境構築時に記録する |
| 実行環境 | Isaac ROS v5 の Docker を前提とする | ホストへの直接導入を標準構成にしない |

### 2.2. 処理の配置と通信範囲

- 自動運転・DualSense 操作側の認識、自己位置推定、計画、制御と ROS 車両 I/O は Jetson 内で完結させる。プロポの基本的な手動操作は Pico 内で ESC・サーボ出力まで完結し、Jetson の稼働を必要としない。
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
- ROS package のビルドは `scripts/build.sh` / コンテナ内の `atd-build` に分離する。`scripts/python.sh` / `atd-python` は用途別 Python 環境の sync・check・shell・run を扱い、colcon のビルド処理を持たせない。
- composable node / component container の利用を推奨する。
- 特に大容量データを交換する処理は、composable な構成を想定して実装する。
- Python node を作成する場合も、CMake と `ament_cmake_auto` を基本とする。
- Python の直接起動用スクリプトは実行権限に注意し、`colcon build` 後に実行できる形で配置・確認する。

### 2.4. センサ・手動介入・車両 I/O の方針

- D455 の IR（infra）、RGB、IMU、Depth はいずれも使用する可能性を残し、RealSense driver で起動する方針とする。
- 実際に使用するストリームの組み合わせ、解像度、周期、bag の記録対象は後から決める。全ストリームの常時同時起動・記録を必須とはしない。
- Joy による手動介入を想定し、DualSense を Bluetooth で Jetson に接続する予定とする。
- Joy の操作割り当て、手動・自律の切替条件、停止・再開条件は後から決める。
- Joy の入力監視と軸・ボタン配置の調整は、地図・転送・ライブ表示と同じ Web アプリから行えるようにする。プロポは受信機 → S.BUS → Pico → USB → Jetson の経路とし、Jetson では値の監視と rosbag の記録開始・停止等に利用する。Pico の走行用 CH1〜CH3 は固定し、Jetson 側の CH4〜CH16 の役割を設定できる。UIは既定でCH1〜CH6のrawとPWM相当µsを表示し、10／16ch表示へ切り替えられる構成とする。実際のPWM出力・波形測定値と入力換算値を区別する。送受信機種・スイッチ位置・校正値は実機で確認する。
- プロポの基本操作は Pico 内で完結させ、プロポ／Jetson の最終的な操作権切替も Pico 側で行う。Jetson 内での DualSense／将来自律入力の許可判断とは別の層として扱う。6ch の用途は操舵、スロットル、操作権切替、rosbag 開始／停止、未定2枠とし、未定枠に役割を割り当てない。
- 入力取得・正規化と、操舵・スロットル・操作要求への役割割り当てを分ける。役割は設定で変更し、Jetson で入力・teleop を起動した後、Notebook の同じ入力画面から受信情報の監視と設定を行う。具体的な割り当ては初期値で固定しない。
- 初回実装は `atd_interfaces` の運転要求・操作イベント型と `atd_teleop` を対象とする。正規化した要求は最終的な走行許可・PWM・実速度ではなく、出力許可判断、パルス幅変換、基板通信、rosbag 記録処理は後続実装で接続する。
- ESC・ステアリングサーボの制御に PCA は使わず、独自 firmware で対応する予定とする。
- マイコンは Pico ベース、Jetson との接続は USB とする。ユーザー提供の Rev6 firmware を `atd_vehicle_interface/firmware/pico_rev6/` で Git 管理する。USB 制御指令形式の採用、実際の CH 配置・端点、ESC／サーボへの適合、停止・再開条件の ATD への反映と実機確認は別途行う。実装と取り込み元は 3.10 節に記録する。

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
3. ノート PC の同じ Web UI で cuVSLAM / VGL 地図を準備し、保存地図と元 bag から表示用点群を取得する。
4. 取得した点群を背景に HDMap を編集し、走行ラインを生成する。
5. 作成した地図を Jetson へ転送する。

地図作成は NVIDIA の公式 offline 処理を利用し、cuVSLAM / VGL で使う地図を準備する。保存地図への localize と点群 snapshot の取得、HDMap の作成・編集、centerline / raceline / 必要に応じた custom line の生成まで共通 Web アプリで扱う。駐車機能は対象に含めない。
bag の Jetson → ノート PC、作成地図のノート PC → Jetson の転送は、SCP を簡単に実行できる機能として提供する。

VGL モデルの準備・配備は、次の分担とする。

1. Notebook でモデルを準備・export し、Jetson の builder 入力に必要なモデル・設定・出自と hash 等の管理情報をまとめる。
2. export 成果物の bundle を SCP で Jetson へ転送する。
3. Jetson では、その資産から対象 GPU 用の TensorRT engine を build する。VGL モデルの準備・export は Notebook 側で完結させる。

保存先は [構成規約](project_structure.md#データの保存構成)に従い、元の入力、export 成果物、対象環境別 engine を分ける。操作は `scripts/vgl.sh` の fzf メニューから export・検証・SCP 転送・Jetson build へ進める。初期実装の対応モデル・形式とコマンドは [VGL モデルの手順](vgl_models.md)に記録し、対象 GPU / SDK と実 SCP を含む通し動作は未検証として扱う。

bag の記録対象や各 bag の topic / frame は実データに合わせて確認する。HDMap の初期 schema と採用済みライン生成方式は 2.9 節と Web アプリの README に記録する。走行開始時の自己位置推定、実地図・実車での動作・性能確認は後工程で行う。

### 2.7. シェルスクリプト・TUI・ブラウザ UI の操作方針

- ノート PC と Jetson の日常操作では、シェルスクリプト（`.sh`）や UI を基本とし、ターミナルコマンドを繰り返し手入力する負担を減らす。
- ユーザーは CUI であっても対話形式や TUI を好む。ユーザー向けの CLI・補助スクリプトでは、この操作方針を優先する。
- `fzf` を用いた TUI を積極的に活用し、候補を検索・選択して対話的に実行できるようにする。
- 対象となる操作の例は、実行する処理、bag、地図、設定の選択である。具体的な画面や操作は各機能の実装時に設計する。
- ブラウザ UI の作成も積極的に行う。主な用途は地図の編集、自己位置の確認、PC / Jetson の操作・状態確認とする。
- UI はノート PC の Chrome のみ で `http://127.0.0.1:<port>` を開いて利用する。ここでの `127.0.0.1` はブラウザを実行するノート PC の loopback を指す。
- Jetson では UI のブラウザ・frontend を起動しない。これは Jetson 側で必要なバックエンドや bridge を動かさないという意味ではない。
- UI の追加でも、走行処理の Jetson 内完結、PC からの遠隔走行制御を行わないこと、車載 DDS の直接通信範囲の要件を維持する。
- 基本機能は 2.9 節の地図・転送・ライブ表示・入力設定とする。UI サーバーは Notebook の Docker 内、Jetson との観測・設定接続は SSH 経由の loopback HTTP とし、操作・既定ポートは Web アプリの README に記録する。個別の観測 topic・周期等は実 bringup に合わせて設定する。
- setup・Docker・Python・VGL に加えて Web UI と Jetson 入力の起動入口を実装した。UI のローカル検証と Docker / ROS / GPU / 実機での結合確認は分けて扱う。

### 2.8. Docker 内の Python 環境

- Docker 内の Python 環境構築は uv を基盤とする。
- 用途ごとに Python 環境を分離し、ROS 2 用の追加依存と AI 学習用の依存を別の環境で管理する。
- 具体的な学習 framework、Python / CUDA wheel の組み合わせや、その他の用途別環境は必要になった段階で決める。

Notebook の VGL export は `vgl-export`、Web API は `web`、Raceline の最小曲率・反復最小曲率 helper は Python 3.12 の `raceline` という独立した uv project / lock / venv を持つ。通常の `atd-web` 起動で `web` / `raceline` を同期し、QP / IQP 計算は別プロセスへ分離する。これらは AI 学習用 framework の採用とは分ける。

用途別環境の構成・操作・実装上の制約は [Python 環境](python_environments.md) を参照する。ROS 2 package の CMake / `ament_cmake_auto` による build 方針は維持する。

### 2.9. 共通 Web アプリの基本機能

以前の `kart` のシステムと作業の流れを基本として、次の 4 機能を一つのアプリで扱う。機能の採用と、旧コード・framework・設定値の一括移植は区別する。React + TypeScript / Vite と FastAPI の採用を確定し、`apps/web/` に初期実装した。UI はノート PC のみで使用する。動作・制約と検証境界は [Web アプリ](../apps/web/README.md)を参照する。

| 機能 | ユーザーが明示した範囲 |
| --- | --- |
| 地図 | rosbag から NVIDIA 公式処理で cuVSLAM / VGL 地図を準備し、保存地図の localize と点群取得後に HDMap を編集。centerline / raceline / 必要に応じた custom line を生成。駐車機能は不要 |
| Jetson との転送 | SCP を簡単に実行する UI。Jetson → ノート PC の rosbag 受信、ノート PC → Jetson の作成地図送信 |
| ライブ表示・調整 | bringup 実行中の観測。地図走行時の地図と自己位置推定の表示、ROS 2 parameter の動的調整、速度等の時系列プロット |
| 入力監視・設定 | Jetson の Joy 入力監視、物理配置の調整、操舵・スロットル・操作要求への役割設定を共通アプリへ統合。Pico USBのraw・PWM相当µs・基板状態を監視し、6／10／16ch表示とCH4〜CH16のJetson側役割設定を扱う |

地図画面は cuVSLAM 地図作成、VGL 地図準備、保存地図からの点群取得、HDMap 編集を一つの流れとして提供する。元 bag と VGL 準備時の Notebook 用モデルを原則として自動選択し、前工程や資産対応に不備があれば理由と必要な操作を示す。NVIDIA 地図は点群取得前に編集・生成を開始せず、空の手動 HDMap は点群なしで編集できる。成功したジョブの表示は自動更新し、未保存編集は破棄しない。

Centerline は **領域・等距離線（region）のみ** とし、方式選択は設けない。格子間隔の初期値は 0.02 m、点数上限は 800。Raceline は **mincurv（既定）/ mincurv_iqp のみ** とし、旧 paired Centerline・local Raceline は生成方式から除外する。Centerline / Raceline は閉路向けで、Raceline は保存済みの領域方式 Centerline を参照する。Customline は編集点列から生成する。駐車は含めず、これらの初期寸法・速度値を実車検証済みとは扱わない。

Joy は Jetson に接続し、ノート PC のブラウザでその入力を監視・設定する。車載 ROS 2 / DDS への直接参加を PC に要求しない。
parameter の調整や入力配置の設定は、PC から操舵・スロットルの連続指令を送る遠隔走行制御とは区別する。走行処理は引き続き Jetson 内で完結させる。
Joy / teleop の初期設定は永続 profile の保存、設定モード中の再読込、全入力解放後の要求出力再開を別操作にする。この入力利用条件と、最終的な走行許可・実車の停止再開条件は区別する。その他の動的 parameter の対象・範囲・変更可能な車両状態・永続化は未決定である。
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
- kart の設定モードや再開条件、全設定の一括移植は採用していない。ATD で採用する地図工程・ライン方式は 2.9 節に明記する。この調査時の確認はソースの読み取りであり、ROS / GPU / 実 bag / 実機での実行検証とは区別する。

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
- 地図作成は公式 `create_map_offline.py`、VGL 地図は公式 builder / `create_cuvgl_map.py` を呼ぶ。自作 `atd_mapping` が保存地図の作業コピーと元 bag を使い、VGL hint・localize 成功後の非空点群・軌跡・TF を採取する。完了時だけ snapshot を公開して編集可能にする。apt 導入、モデル資産、GPU engine、地図成果物、点群取得成功を別に扱う。
- map identity・VGL との対応 hash・地図生成時の平面拘束設定がない旧資産は再生成を案内し、推測で補完・自動移行しない。新規地図は平面拘束を無効と明示して記録し、取得側にも同じ値を適用する。
- Notebook の初回は Docker 内で `atd-build --packages-up-to atd_mapping` が必要である。公式 apt や `web` / `raceline` の同期は自作 ROS package の build を代替しない。`atd-web check` で `atd_mapping` を含む導入状態を診断する。
- Centerline の region と、独立した `raceline` 環境による mincurv / mincurv_iqp を実装した。旧 paired / local への fallback は設けない。単体テスト、小規模閉路で実 QP / IQP solver を使うテスト、frontend の型検査・build はローカルで確認した。
- Jetson 側に C++ component の `atd_joy` と、観測・設定用の `atd_web_bridge` を追加した。Notebook API は SSH から Jetson loopback HTTP へアクセスし、車載 DDS に直接参加しない。
- `atd_interfaces` に正規化運転要求と操作イベント、`atd_teleop` に設定可能な役割割り当てを追加した。Notebook の入力画面に物理配置と役割設定を統合し、保存済みと反映済みを区別する。操作要求の出力までで、最終走行許可・基板通信・rosbag の記録実行は未実装。仕様と検証境界は [atd_teleop](../ros2_ws/src/atd_teleop/README.md)を参照する。
- Web UI は Notebook の Docker 内で `atd-web` を実行して起動し、ブラウザだけを Notebook ホストで開く。ホスト用の `scripts/webui.sh` は `docker exec` による補助入口とする。Jetson の入力起動は `scripts/input.sh`、手順・初期 schema・生成方式・制約は [Web アプリ](../apps/web/README.md)と package の README に記録する。
- Docker build、ROS build / install、実 bag / GPU での公式処理、実 SCP、Bluetooth / evdev、実走行は対象機で別に確認する。ローカルのコード検証だけでこれらを検証済みとはしない。

### 3.10. Rev6 Pico firmware の確認（2026-11-18）

現在の ATD 向け編集元は `ros2_ws/src/atd_vehicle_interface/firmware/pico_rev6/`。ユーザーの指示により `/Users/at/project/tools/pcb/kart_bridge_board/firmware/pico_rev6/` と必要な JPB1 共通通信コードを取り込んだ。実行用 C/H は取り込み元と同一に保ち、ビルド・テストの相対パスを package 内で完結させる。取り込み元のディレクトリは未追跡のローカル試作ソースであり、元のファイルは保持して自動同期・削除はしない。実基板へ書き込まれた版との一致は別途確認する。

小さいソース・テスト・説明を Git 管理し、SDK・toolchain・bytecode・生成バイナリは含めない。Pico SDK 2.3.1／TinyUSB の固定commitは維持し、SDK取得とARM buildはfirmwareの明示的なスクリプト操作に分ける。`firmware/COLCON_IGNORE` により通常のROS workspace探索でPico projectを別packageとして扱わず、colconからSDK取得・ARM build・書き込みを実行しない。ROS packageのinstallには小さい編集元だけを同梱する。

| 対象 | ソースで確認した実装 |
| --- | --- |
| 対象ボード | JPBB Rev6 の RP2040 版 Raspberry Pi Pico。Pico 2 等への対応を意味しない |
| 手動操作 | Pico が S.BUS から操舵・ESC のパルスを生成し、プロポ／HOST の最終選択も実行。Jetson／USB がなくても手動処理を継続できる設計 |
| チャンネル既定値 | CH1＝操舵、CH2＝ESC、CH3＝2位置の操作権。今回の ATD 実装もこの走行用 CH を固定して扱う。USB／ATD UI から Pico の配置を動的変更する処理は対象外 |
| USB CDC0 / if00 | JPB1 制御・状態。状態は20 Hz、選択済み3chの µs 相当値と出力・操作権・異常を配信。制御 C フレームを受けてから状態配信を開始 |
| USB CDC1 / if02 | `STREAM ON` で JPX1 の50 Hz監視を開始。全16chの raw 値・受信状態・時刻・基板状態等を含む。記録用を含む4つ目以降の CH の取得に利用可能 |
| 入力品質 | 監視 raw は frame_lost 付きのフレームでも更新され得る。`sbus_fresh` だけでなく frame_lost / failsafe と受信の進行を検査し、同じ値の再配信を新しい押下として扱わない |
| 制御値 | `JPB1,C` は操舵 −1000〜1000、前進／後退／brake 0〜1000 の正規化値を受け、Pico がパルス幅へ変換。µs を直接送る入口は別の `SET` / `STREAM` CLI。従来の「Jetson でパルス幅計算」との整合は、制御形式採用時に決める |
| 応答の意味 | C ごとの ACK はない。S の seq は基板の状態送信カウンタであり、送った指令の seq の echo ではない。状態受信を各指令の受理確認と混同しない |
| 異常・復帰 | 正常 S.BUS 値の期限50 ms、HOST 指令200 ms、heartbeat500 ms。HOST 再開には現在の許可条件と新しい厳密中立要求が必要。JPB1 C は heartbeat も更新する。これらは参照実装の値であり、実機の停止・制動を検証したものではない |

原典は `include/rev6_core.h`、`src/rev6_core.c`、`src/main.c`、`README.md`、`BENCH_CHECK.md`。取り込み前の確認では、既存の C ホストテストを一時ディレクトリへ build し、制御・S.BUS・行処理／CRC・CH 入替／反転の4条件を確認した。元ディレクトリのファイルは変更していない。packageへの取り込み後の検証結果は以下のとおり。

packageへの取り込み後もCホストテスト（core・S.BUS・transport・protocol・CH入替／反転）、UF2の4テスト、明示指定した元基板netlistとのGPIO 11信号照合が通過した。基板データを指定しない通常のホストテストではGPIO照合をSKIPと明示する。既存の固定版SDK・toolchainを使用した移設先でのRP2040向けRelease build、UF2生成とpayload照合も通過した。SDKの新規取得・実基板への書き込み・USB／S.BUS／PWM波形・実車の検証は行っていない。

ATD 側に `atd_vehicle_interface` の CDC1 受信・状態配信と、`atd_rc_input` の Jetson 側役割処理、Notebook のプロポ監視・設定 UI を追加した。受信 node は USB 属性・固有 serial・if02 を照合し、`INFO` / `STREAM ON` のみを送信する。JPX1 の CRC・範囲・seq・時刻と S.BUS の進行を検査し、受信時刻を保った typed telemetry と監視状態を配信する。Pico が複数ある場合は明示選択が必要。CDC0 と走行制御指令は使わず、firmware は変更しない。

UI はraw値（0〜2047）とPWM相当値（µs）、USB・S.BUS・操作経路・電源・異常・カウンタを表示する。既定はCH1〜CH6で、6／10／16ch表示を選択・Notebookのブラウザに保持する。役割が高いCHにあれば表示数を自動で広げる。受信・ROS・APIは全16chを保持し、10chへの変更で型定義を変えない。表示数は利用中の送受信機のch数を自動検出した結果ではない。PWM相当値は参照firmwareの既定値raw172／992／1811→1000／1500／2000µsを区間補間・整数丸め・端点制限した参考表示であり、ESC・サーボの実出力や波形の実測値ではない。役割閾値はraw単位を維持し、欠落・古い入力を中立値で埋めない。

Jetson 側の記録開始／停止・予備1・予備2は CH4〜CH16 または未割り当てへ設定でき、閾値・反転・debounce を `/data/results/runs/input/propo.yaml` へ保存する。version 1の既存設定を維持する。初期値は全て未割り当てで、予備2枠は状態表示のみ。設定開始・保存・再読込・イベント再開を分け、設定中も Pico の手動走行は継続する。再接続・反映・再開時は現在位置を基準に取り直し、スイッチを保持しているだけでは要求を出さない。正常で新鮮な入力の安定した低→高で開始要求、高→低で停止要求を `/rc/events` に出す。実際の rosbag 記録処理は未実装。

ホスト上でプロトコル、sysfs/PTY による USB 選択・排他・切断、役割判定、Web API・画面ロジックを検証した。ROS build/install・DDS/service 結合・Jetson Docker・実 USB・S.BUS・実基板の動作は未検証。Pico の選択状態と Jetson 内の HOST 要求の gate を分ける方針を維持する。

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
- Notebook Docker の Web API・地図処理と、SSH 経由の Jetson bridge 接続について、対象機での導入・実通信・操作を確認する。

### 5.2. 後工程で検討する項目

以下は自動運転ソフトウェアや車両制御を具体化する段階で決める。現時点の環境整備を進めるための回答必須事項にはしない。

1. Pico firmware と ATD 間の USB 制御形式、実 CH 配置・校正、ESC・サーボへの適合、操作権・通信断時の停止と再開条件の結合確認。Pico／USB／S.BUS の方針は決定済みである。
2. 初期の走行場所・路面・目標速度・到達目標、障害物に対する停止 / 回避の範囲。
3. D455 の各ストリームの利用・記録設定、撮影モード、取付姿勢と較正。
4. DualSense の操作割り当て、手動 / 自律の切替と停止・再開条件。プロポの実機種・スイッチ位置・校正、利用可能なCH4〜CH16の役割選択と実際の記録処理の結合。Pico の CH1〜CH3 を固定し、予備2枠の用途は未定。10chプロポの採用は候補であり、購入・実機対応確認済みとは扱わない。
5. 採用済み cuVSLAM / VGL・点群取得・region / mincurv / mincurv_iqp の対象機・実 bag での結合確認、HDMap schema の拡張、実車の性能、走行開始時の自己位置推定。VGL モデルの GPU / SDK 互換性、SCP の実接続と転送後の利用、成果物の対応管理。
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
| [hardware_cad.md](hardware_cad.md) | 車体CAD・印刷用3MF・Rev6カメラ組立モデルの一覧と取り込み元。実物との一致は未検証 |
| `../docker/` | 追加 Dockerfile とデータを毎回 mount する起動引数 |
| `../docker/isaac_ros/` | 選択した workspace から参照する CLI 設定テンプレート |
| `../ros2_ws/src/` | 自作 `atd_*` package の編集元。build 成果物はホスト別 workspace に作成する |
| `../scripts/dev.sh` | PC / Jetson を自動判定する開発環境の起動スクリプト。必要に応じて手動上書き可能 |
| `../scripts/docker.sh` | Docker 開発環境の診断、結果の保存・閲覧、起動・build の入口 |
| `../config/workspace.env.example` | PC / Jetson ごとの ROS workspace 設定例 |
| `../config/storage.env.example` | PC / Jetson ごとの記録先・解析結果保存先の設定例 |

## 7. 更新履歴

- 2026-11-18: ユーザー指定のTT-02-v3の3MF 4件とD455／D435iのFCStd組立モデル2件を `hardware/cad/` に保管。元ファイルと既存v1モデルを保持し、SHA-256一致・ZIP CRC・XML／内部参照を確認した。寸法・組付け・機材採用の確定とは区別する。
- 2026-11-18: ユーザーの指示でPico Rev6 firmwareと必要な共通通信コードを `atd_vehicle_interface/firmware/pico_rev6/` に同梱。制御・通信のC/Hを保持し、SDK／成果物をGitから除外、ROS buildとARM buildを分離し、移設先でのホストテストを確認した。
- 2026-11-18: プロポ監視をrawとPWM相当µsの併記にし、既定6chから10／16chへ表示を切り替えられる構成に拡張。Jetsonの役割候補もCH4〜CH16へ広げ、Picoの固定走行CH・全16chの受信型・既存profile v1を維持した。µs表示は参照firmwareの既定校正による換算値で、実PWM出力の測定とは区別する。
- 2026-11-18: Pico CDC1 の受信、6ch・基板状態の監視、CH4〜CH6 の Jetson 側役割設定 UI と記録要求イベントを実装。Pico の走行用 CH1〜CH3 と firmware は変更しないことを確定。記録実行・走行指令送信・実機確認は後続工程とした。
- 2026-11-18: 6ch プロポの S.BUS → Pico → USB → Jetson 経路、Pico 単独での基本手動操作、Jetson による入力監視・rosbag 開始／停止を確定。提供された Rev6 firmware の2 CDC・既定 CH・操作権・正規化指令形式を調査し、ユーザー要件・参照実装・未実装の ATD 接続・実機未検証を区別した。

- 2026-11-17: ROS package のビルドを専用の `build.sh` / `atd-build` へ分離し、`python.sh` / `atd-python` は環境管理・実行だけを担当する構成にした。ビルドの旧 `ros build` 操作は廃止し、環境準備の共通処理は共有する。対象 Docker 内のビルド検証とは区別する。

- 2026-11-17: 地図画面を cuVSLAM / VGL 準備、保存地図の localize と自動点群取得、HDMap 編集へ接続した。駐車を除外し、Centerline は region、Raceline は mincurv（既定）/ mincurv_iqp に限定。独立した raceline 環境、atd_mapping の初回 build、旧資産の再生成条件と実機検証境界を記録した。

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
