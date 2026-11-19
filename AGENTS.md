# ATD 開発規約

## 基本方針

- 原則として日本語で回答・記述する。
- 目的達成を優先し、必要な調査、ファイル確認、修正、検証を自律的に進める。
- 結果に大きく影響しない不明点は合理的に補い、重要な不明点だけ確認する。
- 事実、推測、提案、検証結果を区別する。調査では原論文、公式ドキュメント、公式リポジトリを優先する。
- 既存構成と周辺内容を確認してから必要最小限の変更を行い、変更に応じた実行・検証を行う。

## 要件の参照と扱い

- プロジェクトの要件、機材、想定環境、提案、未決事項は [docs/project_requirements.md](docs/project_requirements.md) を参照する。
- ユーザーが明示した要件と、調査に基づく提案を混同しない。提案や候補を採用済みとして扱わない。
- 設計に影響する要件が決まった場合は、関連する文書も整合させる。
- 現在は Docker 開発環境と共通 Web アプリ・入力監視の初期実装段階である。設定・スクリプトの作成、ホスト上の検証、Docker 内での動作、実機検証を区別して報告する。
- 自動運転ソフトウェアの詳細は後工程で検討する。走行目標・アルゴリズム等の確定を、現在の環境整備や共通規約の定義を進める前提条件にしない。

## ディレクトリ・ファイルの配置

- 配置は [構成規約](docs/project_structure.md) を基準とする。自作 ROS package は `ros2_ws/src/atd_<module>/`、共通 Web アプリは `apps/web/` に置く。Pico Rev6 firmware は関連する ROS package の `ros2_ws/src/atd_vehicle_interface/firmware/pico_rev6/` で Git 管理する。SDK・toolchain・生成バイナリをソースへ同梱せず、firmware の cross build は通常の colcon build と分ける。
- ROS package の `launch/`・`config/`・`test/` は package 内に置く。root の `config/` はホストの workspace・保存先等、root の `tests/` は setup・Docker 等の共通基盤に使う。
- よく使う日常操作の入口を `scripts/` 直下に置く。現在は `setup.sh`・`dev.sh`・`build.sh`・`vgl.sh`・`bringup.sh` の5本とする。診断・Python 環境管理・Web の補助入口は `scripts/tools/`、共通関数とホスト判定は `scripts/lib/`、ホスト準備の内部処理は `scripts/setup/`、コンテナ内処理は `docker/scripts/` に置く。
- CLI 設定テンプレートは `docker/isaac_ros/`、生成 Dockerfile は Git 管理対象外の `.cache/docker/` に置く。ROS package の編集元は `ros2_ws/src/` とする。リポジトリの `ros2_ws` を workspace に選ぶ場合はその `src` を使い、別の workspace の場合はコンテナの `src/atd` に rw で mount する。
- CAD 等の編集元は `hardware/`、説明文書は `docs/` に置く。clone の標準配置は `~/workspaces/atd` とし、bag・地図・モデル重み・engine・学習結果は clone 配下の `data/record`・`data/results` を既定とする。`data/` は Git 管理対象外の永続資産として保持する。
- データは既存の記録先・結果先を使い、`/data/record/rosbags/` と `/data/results/{maps,models,runs}/` に整理する。NVIDIA の地図・モデル・exporter の内部構成は維持し、地図とモデルを混同しない。対象環境別 engine・run の checkpoint / log は保持する資産とし、cache として自動削除しない。詳細は [保存構成](docs/project_structure.md#データの保存構成)に従う。
- VGL モデルの準備・export は Notebook で行い、export 済みモデルと必要な設定・管理情報を SCP で Jetson へ転送する。VGL モデル準備に関して Jetson では転送済み資産からの TensorRT engine build のみを行う。入力、export 成果物、対象環境別 engine を分けて保存し、処理の実装と対象機での動作検証を区別する。
- VGL 操作の入口は `scripts/vgl.sh` と [手順](docs/vgl_models.md) とする。Notebook のモデルからの ONNX export と、既存 ONNX bundle の package 操作を区別する。転送・build では manifest の ID・hash を検証し、既存の資産を上書きしない。実装のテストと、対象 SDK / GPU・実 SCP での検証を分けて報告する。
- 未実装のための空ディレクトリ・空文書を先に作らない。cache・bytecode と、記録データ・ログ・編集元資料を区別する。

## 実行環境と処理の配置

- 基盤は Ubuntu 24.04、JetPack 7.2、ROS 2 Lyrical、Isaac ROS 5 を想定し、Docker を前提とする。
- Isaac ROS CLI は apt 版を採用し、Docker のカスタムは YAML と追加 Dockerfile で行う。CLI 本体の改修が必要になった場合に source build を検討する。詳細は [ビルド方針](docs/isaac_ros_build_strategy.md) を参照する。
- RealSense はローカルの `realsense455` layer を使い、librealsense SDK のみ CUDA 無効で構築する。ベースの CUDA と他の Isaac ROS GPU 処理は無効化しない。
- イベントカメラは今後使用しない。関連 SDK・driver・ROS package・Python 依存を導入せず、将来の追加候補にも含めない。
- OS・配布済み ROS パッケージ、SDK、uv 本体、Notebook の Node / pnpm などコンテナのシステムに導入する依存は Docker build で準備する。共通 apt 依存は `docker/dependencies.apt` で管理し、起動時は導入状態を検査する。不足やイメージとの不一致があれば起動を止め、再ビルドを案内する。
- 日常コマンドは `/workspaces/atd` にマウントした checkout から実行し、編集は次の実行で反映する。DDS・USB serial・ROS 用 Python 同期など起動時に適用する設定の変更はコンテナを再作成して反映する。uv の venv、Web の node_modules、自作 ROS package の colcon 成果物は各ホストのマウント済み workspace に置く。
- DDS は Fast DDS の UDPv4 / loopback 設定を使う。生成先は `/etc/atd/fastdds.xml` と `/etc/atd/dds.env.sh` とし、起動時の root スクリプトで配置する。
- 初回準備は `scripts/setup.sh` と [セットアップ手順](docs/setup/README.md) を入口とする。Notebook（x86_64 / amd64）と Jetson（aarch64 / arm64 と BSP）を区別し、既存 Docker・CLI 設定を不用意に置換しない。
- ROS workspace は setup の workspace 操作で対話的に指定し、未設定時は実際の clone 配下の `ros2_ws` を使う。標準 clone 位置なら `~/workspaces/atd/ros2_ws` となる。`config/workspace.env` の `ATD_PC_ROS_WS` / `ATD_JETSON_ROS_WS` に別々に保存する。保存済み指定と引数・環境変数の上書きを優先し、旧設定や既存データを自動移行しない。コンテナでは `/workspaces/isaac_ros-dev` を使い、`ISAAC_ROS_WS` は `dev.sh` が設定する。Notebook / Jetson のホストパスやビルド成果物を同一視しない。別の workspace を使う場合、その `src/atd` はリポジトリの `ros2_ws/src/` を mount する予約先とし、編集元を別々に管理しない。
- 環境の診断は `scripts/tools/docker.sh` から項目ごとの `OK` / `FAIL` / `SKIP` を表示し、結果を `logs/docker/<host>/` に保存して後から確認できるようにする。導入状態の診断と、Docker 起動・GPU 演算・センサ・実走行の検証を区別する。
- 開発環境は `scripts/dev.sh` で起動し、PC / Jetson は機種情報・BSP のマーカーから自動判定する。必要な場合だけ `--host pc|jetson` で上書きする。保存先はホストごとに設定し、記録データと解析結果を毎回読書き可能で mount する。rosbag 記録や車両起動は自動実行しない。
- 新規起動と稼働中コンテナへの再接続を区別する。再接続は `atd-shell` を使い、稼働中の依存・DDS 設定を更新しない。構成が変わった場合や旧コンテナは、処理を終了して再作成する。
- コンテナ内の対話シェルは `atd-shell` を標準とし、ホストから共有された旧 DDS 設定を含む `.bashrc` に依存しない。
- 初期センサ構成は RealSense D455 のみとする。IR（infra）、RGB、IMU、Depth はいずれも利用候補として残し、RealSense driver で起動する方針とする。全ストリームの常時同時起動・記録を必須と解釈しない。起動初期値はInfra1/2 640×480@90、RGB 640×480@30、加速度200Hz・ジャイロ400Hz、Depth OFF。TUIで各OFF・実機対応モードを選び、同時構成を検査する。
- 車体static TFと車両配置は `atd_vehicle_description` が担当する。`base_link` は後輪回転軸中央、x前・y左・z上。カメラ内部TFはRealSense driverに任せる。車両UIではmm／度で実測し、未計測値を原点・identityで補完しない。CAD参考寸法と実車寸法を区別し、保存とTF node再起動による適用を分けて示す。
- 自動運転・DualSense 操作側に必要な処理と制御は Jetson Orin Nano 8GB 内で完結させる。プロポの基本手動操作は Pico 内で ESC・サーボ出力まで完結させ、Jetson の稼働を必要としない。
- ノート PC は地図作成・編集・解析・走行情報の可視化を担当する。Foxglove Bridge 等による自己位置等の観測を想定する。ノート PC から遠隔の走行制御を与える構成にしない。
- 地図作成は、手動走行中に Jetson で記録した bag を SCP で Notebook へ転送し、同じ Web UI で NVIDIA 公式 cuVSLAM / VGL 地図準備、保存地図への localize と点群 snapshot 取得、HDMap 編集・ライン生成まで進め、作成地図を Jetson へ SCP 転送する。NVIDIA 地図は点群取得後に編集可能とし、空の手動 HDMap は点群なしでも編集できる。駐車は対象に含めない。
- Centerline は領域・等距離線の `region` のみ（初期格子 0.02 m、点数上限 800）。Raceline は `mincurv`（既定）と `mincurv_iqp` のみとし、保存済みの region Centerline を参照する。旧 `paired` / `local` の生成・fallback を追加しない。Customline は編集点列から生成する。生成成功と実車の性能・走行検証を区別する。
- 点群取得は地図の作業コピーと元 bag、対応する VGL 地図・Notebook 用モデルを使う。地図内容・保存画像・モデルの対応や地図生成時の平面拘束設定を検査し、必要な metadata がない旧資産は再生成を案内する。原点・identity TF・推測した設定で補完せず、既存資産を自動移行しない。ROS package の build は `scripts/build.sh` を入口とし、Notebook の初回は Docker 内で `/workspaces/atd/scripts/build.sh --packages-up-to atd_mapping` を実行する。apt 導入や Web の起動だけで自作 package を build 済みと扱わない。
- 車載系の ROS 2 / DDS の直接探索・データ通信は Jetson 内部に限定する。Foxglove Bridge 等を介した明示的な情報提供は許容する。必要なら `--network host` を維持し、DDS と bridge の通信設定を区別する。詳細は要件文書を参照する。
- Joy は DualSense を Bluetooth で接続する予定とする。操作割り当て、切替条件、停止・再開条件は、決定前に固定しない。
- ESC・ステアリングサーボの制御に PCA は使わず、Pico ベースの独自 firmware で対応する。プロポは6chを基本とし、将来の10chにも対応できる構成にする。入力は受信機 → S.BUS → Pico、Jetson との接続は USB とし、全16chを保持する。Pico でプロポ／Jetson の操作権を切り替え、Jetson はプロポ値の監視と rosbag 開始／停止等に使う。Pico の走行用 CH1＝操舵・CH2＝スロットル・CH3＝操作権は固定し、CH4〜CH16 の Jetson 側役割を変更できる。校正値・USB 制御指令の採用と実機確認は別途行う。

## シェルスクリプト・TUI・ブラウザ UI の操作方針

- ノート PC と Jetson の日常操作では、シェルスクリプト（`.sh`）・TUI・ブラウザ UI を積極的に用意し、ターミナルコマンドを繰り返し手入力する負担を減らす。
- ユーザーが操作する CLI・補助スクリプトは、対話形式や TUI を優先して設計する。
- 処理、bag、地図、設定等の候補を検索・選択する場面では、`fzf` を積極的に活用し、対話的に実行できるようにする。
- 対話操作と、引数・環境変数による直接指定を併用できる構成にする。既存の PC / Jetson の自動判定など、確定済みの起動方針との整合を保つ。
- ROS のまとめ起動は `scripts/bringup.sh` を入口とし、実在する `joy-drive`（カメラなしの DualSense 手動走行）、`manual`（手動走行・D455・車体TF・入力監視・bridge）、`d455`（D455・車体TF）、`input`（Joy・Pico入力・役割処理・bridge）、`bridge`（bridgeのみ）を扱う。引数なしの対話端末では fzf、未導入なら番号で選択し、非対話では profile の明示を求める。実行は Jetson の既存 Docker 内に限定し、Notebook では `--list` と `--dry-run` による確認を使う。profile 一覧は `scripts/lib/bringup.sh` で管理し、追加時は実装済み launch・必要な引数・対象ホストを対応させる。構成の組合せは `atd_bringup`、D455の設定・起動は `atd_sensors` が担当する。未実装の自律走行 profile は先に作らない。操作は [bringup](docs/bringup.md)を参照する。
- 操作や可視化に役立つブラウザ UI も積極的に作成する。主な用途は地図の編集、自己位置の確認、PC / Jetson の操作・状態確認である。
- ブラウザ UI はノート PC のみで使い、Chrome で `http://127.0.0.1:<port>` を開く。Jetson で UI のブラウザ・frontend を起動しない。観測・設定用の Jetson 側 ROS bridge は別に配置する。
- 共通 Web アプリは React + TypeScript / Vite と FastAPI を採用する。`apps/web/` に frontend・backend・Web に依存しない processing を分け、通常は Notebook の Docker 内で `atd-web` を実行する。`scripts/tools/webui.sh` はホストから同じコンテナ内コマンドへ委譲する補助入口。ブラウザだけを Notebook ホストで開く。Node / pnpm と `web` 用 uv 環境は Notebook 専用。実装・ホスト上の検証・Docker / ROS / 実機検証を区別する。
- 共通 Web アプリは地図、Jetson との SCP 転送、bringup 中のライブ表示・ROS 2 parameter 動的調整、Joy 入力監視・軸ボタン配置調整を扱う。プロポは Pico の USB 入力と基板状態を監視し、CH1〜CH6を既定表示、6／10／16chで切り替えられるようにする。raw 値と PWM 相当の µs 換算値を併記し、実際の ESC・サーボ出力や波形の測定値とは区別する。CH4〜CH16 の記録・予備役割は初期設定を未割り当てとする。設定中は Jetson の役割イベントだけを止め、Pico の手動走行には介入しない。再接続・再開時のスイッチ位置は基準として取り直し、記録開始イベントを合成しない。記録の実行処理は `atd_recording` に集約し、Joy・プロポの要求からMCAPを作成する。bringupでmanagerを起動しても録画は自動開始しない。保存完了はstop後のmetadata・MCAP検査で判定し、UI・SSH断を停止扱いにしない。DualSense の走行指令は Jetson の専用 node が USB CDC0 で送る。CH3 の HOST 許可・中立での開始を確認し、プロポ側の直接走行は Pico 内で完結させる。機能要件と実装・検証状態は [要件文書](docs/project_requirements.md) で区別する。
- UI の追加でも、走行処理の Jetson 内完結と、PC から遠隔の走行制御を与えない要件を維持する。ブラウザの実行場所と、サーバー・Jetson 側バックエンドの配置は区別する。

## ROS 2 パッケージと実装

- 自作 ROS 2 パッケージはモジュールごとに分割し、`atd_<pkg_name>` と命名する。
- C++ を基本とし、計算量、メモリ使用量、コピー、遅延を意識して効率よく実装する。
- `CMakeLists.txt` と `ament_cmake_auto` を原則として用いる。
- composable node / component container の利用を推奨する。特に画像・深度など大容量データを交換する経路は、component として利用する前提で設計・実装する。
- component 化だけでコピー削減や性能達成を検証済みとは扱わず、必要に応じて実際のデータ経路と性能を確認する。
- Python node も CMake と `ament_cmake_auto` を基本とし、必要に応じて `ament_cmake_python` を併用する。
- 直接起動する Python スクリプトには適切な shebang と実行権限を設定し、CMake の `install(PROGRAMS ...)` などで配置する。`colcon build` 後の起動も確認する。import 用モジュールへの実行権限は必須ではない。
- DDS のネットワーク設定と、topic ごとの QoS を区別する。ROS 側と XML のどちらで設定するかを明確にする。

## Docker 内の Python 環境

- Python の追加依存と仮想環境は uv を基盤として管理する。ROS 2 用と AI 学習用を分け、用途ごとに独立した `pyproject.toml`・`uv.lock`・venv を持つ。
- Docker には uv `0.13.8` を固定導入する。root 起動 hook から `gosu admin` を通して ROS 2 用環境を自動同期し、venv・cache は通常ユーザー所有にする。AI 学習用環境は明示的な `sync`・`run`・`shell` で準備する。
- ROS 2 の apt パッケージと OS の Python はシステム側で管理する。ROS 2 用 venv はコンテナの `/usr/bin/python3` と `--system-site-packages` を使い、配布済み ROS 2 Python 拡張との ABI を合わせる。
- AI 学習用 venv には ROS 2 の system site-packages や `PYTHONPATH` を引き継がせない。ROS 2 の追加依存と学習依存を同じ venv に混在させない。
- Notebook の VGL export は `docker/python/vgl-export/` の独立した project / lock / venv を使う。重い export 依存を ROS 用・学習用へ混在させず、Jetson の標準起動時には同期しない。
- Notebook の Web API と最小曲率計算は `docker/python/web/` / `docker/python/raceline/` の独立した uv project / lock / venv を使う。`raceline` は Python 3.12 の QP / IQP helper 用で、通常の `atd-web` 起動から同期し別プロセスで実行する。Web・ROS・学習依存へ混在させず、Jetson の標準環境へ追加しない。
- OS Python への `pip install`、`uv pip install --system`、`--break-system-packages` による追加依存の導入を標準手順にしない。
- `uv.lock` を管理対象とし、venv・cache・ダウンロード済み Python は各ホスト・コンテナで作成する。Notebook と Jetson の venv をコピーして共有しない。
- 学習 framework、CUDA wheel、学習用 Python のバージョンと Jetson での学習対応は用途が決まった段階で選定する。Notebook と Jetson の GPU 依存を同一と仮定しない。
- uv による依存管理と、ROS 2 package の CMake / `ament_cmake_auto` による build・install を区別する。Python node の interpreter・shebang・CMake の Python 検出先を確認する。
- ROS package の build は `scripts/build.sh` / コンテナ内の `atd-build` に実装し、`scripts/tools/python.sh` / `atd-python` は `sync`・`check`・`shell`・`run` に限定する。環境準備は共通関数を使い、ビルド処理を Python 環境管理のコマンドへ戻さない。
- 操作と制約は [Python 環境](docs/python_environments.md) を参照する。

## 検証と記録

- 想定環境、導入済み環境、検証済み環境を区別して記録する。
- ホスト・コンテナのアーキテクチャやハードウェアに依存する設定を、根拠なく同一視しない。
- ビルド・単体検証、bag 再生、実機 I/O 確認、実走行の結果を分けて報告する。
- 停止指令の出力、ESC のニュートラル、実際の制動・停止を区別する。
