# ATD 共通 Web アプリ

Notebook の Chrome から使う React + TypeScript / Vite と FastAPI のローカルアプリ。地図・転送・車両校正・ライブ・コントローラ・車両設定を扱う。Jetson では観測・設定用の ROS bridge と入力ノードを実行する。

## 起動

Notebook のホストで Docker を起動・入室し、コンテナ内から `atd-web` を実行する。
Docker 設定を更新した場合は、利用中の処理を終了してイメージを再構築する。

```bash
# Notebook のホスト: Docker を起動・入室（初回・設定変更時は --build）
./scripts/dev.sh --build
# ここから Notebook の Docker 内。初回・自作package更新時に実行
atd-build --packages-up-to atd_mapping atd_vehicle_calibration
atd-web
```

ブラウザは Notebook ホストの Chrome で `http://127.0.0.1:8766` を開く。FastAPI・frontend の build / 配信・地図作成等のジョブは Notebook の Docker 内で実行する。`atd-web` は専用の `web` 用 uv 環境を同期・選択して API を起動する。Raceline の最小曲率計算用には、独立した Python 3.12 の `raceline` 環境も同期する。初回は frontend 依存の同期と build を行い、以後はソースの変更時だけ再buildする。ソースをホストと共有し、`node_modules` と `dist` は選択した workspace の `.atd-web/frontend/` へ生成する。

```bash
# Notebook の Docker 内
atd-web check                     # Web / Raceline 環境と、地図作成・車両校正用の ROS package を確認
atd-web build                     # frontend を明示的に再build
atd-web dev                       # 開発用 UI: http://127.0.0.1:5173
atd-web --port 8769                # 通常 UI / API の port を変更
atd-web --dry-run                 # 起動予定を表示
```

ホストから起動したい場合の補助入口として `./scripts/tools/webui.sh` も用意している。これは既存コンテナへ `docker exec` で入って同じ `atd-web` を実行する。ホストに Web サーバーや Node / FastAPI を導入する経路ではない。

コンテナ・センサ・車両・記録はこの入口では自動起動しない。API は loopback 待受。通常利用はビルド済み frontend と API を同じ origin で配信する。

初回は上記の ROS build で Notebook の workspace に `atd_mapping` と `atd_vehicle_calibration` を準備する。`atd-web check` は ROS と workspace の setup を読み込んで package の導入状態を確認する。環境や package が未準備なら `FAIL` と表示する。

## 車両校正

「データ転送」で手動走行の bag を受信したら、「この bag を車両校正へ」→「解析を開始」と進む。Notebook が連続した VO 軌跡を準備し、プロポ入力・IMU と同期して速度、加速度、操舵応答を求める。同じ走行から作成した軌跡を再利用できる。進行中のログ、過去の解析、応答グラフと CSV／JSON をこの画面から確認できる。[記録から解析までの操作](../../docs/vehicle_calibration.md)

## Map

「車両校正」で走行結果を開くと、Raceline の曲率・横加速度・加減速の候補と前進スロットルのフィードフォワード表を確認できる。採用する制限と地図を選び、その地図で Raceline を生成すると反映される。速度フィードフォワード表は JSON で保存できる。[候補の算出条件と採用手順](../../docs/vehicle_calibration.md)

同じ Map 画面で、次の順に進める。駐車機能は対象に含めない。

1. **cuVSLAM 地図を作成**: `/data/record/rosbags/` の bag、地図 ID、左右画像・CameraInfo の topic、base / optical frame を指定する。NVIDIA の `isaac_mapping_ros/create_map_offline.py` を独立 worker で呼び、`maps/<id>/nvidia/` に公式の `cuvslam_map/`・`map_frames/` 等の構成を維持して保存する。
2. **VGL 地図を準備**: Notebook の export bundle を指定し、この PC の GPU 用 engine を準備してから公式 `create_cuvgl_map.py` を実行する。model 資産側の engine と地図の `cuvgl_map/` は別に保持する。Notebook engine を Jetson 用として転送しない。
3. **保存地図から点群を取得**: 「保存地図から点群を取得」で、元 bag と VGL 準備時の Notebook 用モデルを自動選択する。必要なら詳細で選び直す。地図・保存画像・モデル・入力画像サイズ等の対応、ROS と `atd_mapping` の導入を開始前に検査し、不備は理由と前工程への案内を表示する。
4. **HDMap を編集**: 取得後の点群を背景に、左右境界と Custom 点列を Canvas で編集・保存し、走行ラインを生成する。処理成功後は選択中の地図と背景・ラインを自動更新する。未保存編集や別地図への選択変更を、ジョブ完了で破棄しない。

点群取得では元の保存地図を作業用に複製し、元 bag の画像を再生する。[atd_mapping](../../ros2_ws/src/atd_mapping/README.md) が VGL の姿勢 hint と保存地図への localize 成功を確認し、その成功区間の非空点群・軌跡と TF を採取する。点群が既に `map` frame なら再変換せず、それ以外では確認した TF を一度だけ適用する。TF 不足や自己位置未確定を原点・identity 変換で補わない。点群は全地図の landmark 網羅性を保証せず、採用時刻・入力末尾との差等は provenance に残す。

成功した snapshot と manifest を `hdmap/captures/<capture_id>/` に保存し、revision と `source.pointcloud_status=ready` を更新してから編集可能にする。公式地図の生成成功と、表示用点群の取得成功は別の状態である。NVIDIA 地図は点群取得前の編集・ライン生成を禁止する。空の HDMap は手動編集を開始でき、任意の背景として `map` 座標・メートル単位の JSON 点群を読み込める。

cuVSLAM 地図の内容 hash、VGL 地図・保存画像・モデルの対応情報、地図生成時の平面拘束設定がない旧資産は、開始前の検査で再生成を案内する。未記録値の推測や既存資産の自動移行は行わない。新規地図は平面拘束を無効と明示して記録し、点群取得もその記録と同じ設定を使う。

### ライン生成

| ライン | 採用する方式 | 初期値・条件 |
| --- | --- | --- |
| Centerline | 領域・等距離線（`region`）のみ | 格子間隔 0.02 m、点数上限 800、閉路のみ。方式の選択欄は設けない |
| Raceline | 最小曲率（`mincurv`）、反復最小曲率（`mincurv_iqp`）のみ | 既定は `mincurv`。保存済みの領域方式 Centerline を参照し、閉路で生成する |
| Customline | 編集点列から生成 | 開路・閉路に対応。指定した車体・余裕を検査し、曲率から速度を制限する |

左右境界の弧長対応を使う旧 Centerline（`paired`）と、簡易平滑化の旧 Raceline（`local`）は生成方式から除外した。依存不足や最適化失敗で別方式へ切り替えない。旧生成線の表示は可能だが、旧 Centerline を Raceline の参照に使う場合は領域方式で再生成する。

自己交差・領域外・車幅と余裕・長方形車体を検査し、Raceline には曲率上限も適用する。各線の曲率と加減速制約から目標速度を付ける。点間隔の初期値は 0.10 m。車幅・全長・余裕・速度・加速度の初期値は実測値として確定したものではなく、車両・走行条件に合わせて設定する。最短ラップや実車の操舵・制動性能の検証を意味しない。

補正 `tph_full_curvature_jacobian_v1` は、[TPH 原実装](https://github.com/TUMFTM/trajectory_planning_helpers/blob/master/trajectory_planning_helpers/opt_min_curv.py)の接線・分母を固定する曲率近似が円環を内側へ縮める場合を修正する。TPH の spline・quadprog・[IQP 反復](https://github.com/TUMFTM/trajectory_planning_helpers/blob/master/trajectory_planning_helpers/iqp_handler.py)と幅・曲率制約を維持し、曲率の完全な一次微分を用いた基準弧長重みの曲率二乗 QP とする。出力は点数に依存する単純和ではなく3点曲率と弧長の台形和による近似 `∫κ² ds` [1/m] で保存済み Centerline と比較し、増加する候補を拒否する。単純閉路の符号付き `∫κ ds ≈ ±2π` はこの最小化目的とは異なる。補正 ID・目的・比較指標・検査による縮小倍率を metadata に記録し、IQP の検査縮小は最終反復だけでなく初期 spline からの累積移動全体へ適用する。いずれも局所近似と保守的な検査を経た候補であり、大域最適解は保証しない。

編集正本は `hdmap/map.json`、生成物は `lines/`。revision で保存競合を検出し、境界変更で既存ラインを失効させる。QP / IQP は `docker/python/raceline/` の独立した uv 環境の別プロセスで計算し、Web API・ROS の依存へ混在させない。

## Transfer

Jetson の SSH host・user・port、bag root と map root を画面で設定する。Jetson の bag ディレクトリを階層から選んで Notebook へ受信し、作成した地図ディレクトリを Jetson へ送信する。map root の例は Jetson ホストの `<result_dir>/maps`、bag root は `<record_dir>/rosbags`。コンテナ内の `/data/...` とホストのパスを混同しない。

鍵認証と既存の `known_hosts` を使い、password・秘密鍵をアプリ設定に保存しない。SCP は一時領域へ転送し、転送前後のファイル内容 SHA256 を確認して新規ディレクトリへ公開する。既存 ID は上書きしない。転送完了と Jetson が地図を読み込み走行に適用したことは別である。

ジョブの状態・入力・ログ・中止要求は `/data/results/runs/web/` に保持する。UI や API サーバーの終了だけでは独立 worker を停止せず、再接続で状態を復元する。Docker 停止・OS 再起動からの処理の自動再実行は行わない。

## Live / Input

Jetson で自作 ROS package を build し、入力・teleop 操作要求・bridge を明示的に起動する。

Pico 対応を初めて取り込むときは、Jetson の利用中の処理を終了して `./scripts/dev.sh --build` で image とコンテナを更新する。USB の後付け・再接続用 mount と通常ユーザーの serial group を起動時に準備するため、旧コンテナへの再接続だけでは反映されない。

```bash
# Jetson の起動済み ATD コンテナを利用
./scripts/build.sh --packages-up-to atd_web_bridge
./scripts/bringup.sh input
```

入力 node が既に起動済みなら `./scripts/bringup.sh bridge`。bridge の設定は [atd_web_bridge](../../ros2_ws/src/atd_web_bridge/README.md) を参照する。既定 topic は実際の bringup の topic・型へ合わせる必要がある。bridge の設定ファイルを用意して `./scripts/bringup.sh input config:=/absolute/path/bridge.yaml` で指定できる。
`input` profile は Joy・teleop・Pico 受信・プロポ役割処理・bridge、`bridge` profile は bridge だけを起動する。`input` では `teleop:=false` で Joy の役割処理、`propo:=false` で Pico とプロポ役割処理を除外できる。起動 profile は `./scripts/bringup.sh --list` で確認し、引数なしの対話端末でも選べる。Notebook では `./scripts/bringup.sh --dry-run input` による確認だけを行い、実際の起動は Jetson の既存 Docker 内で行う。いずれも基板出力・bag 記録は実行しない。[bringup の操作](../../docs/bringup.md)も参照する。

Notebook backend は SSH 経由で Jetson の `127.0.0.1:8790` へ接続する。Notebook は車載 DDS に参加しない。Live では地図・pose・軌跡・速度の観測と ROS parameter の読書きを扱い、欠落・古いデータは表示で区別する。

parameter の閲覧は node prefix、変更は node と parameter 名の allowlist で指定する。既定では変更許可なし。descriptor の型・範囲・read_only を検査し、atomic 変更後の値を再取得する。変更を永続 YAML へ自動保存しない。

Input は Jetson に Bluetooth 接続した DualSense の生入力・正規化入力・接続状態と、物理配置・操作の役割を同じ画面で扱う。Bluetooth のペアリングは Jetson ホスト側で行う。ブラウザの Gamepad API や Notebook 接続のコントローラは使わない。

1. Jetson のホストでコントローラを Bluetooth 接続し、Docker 起動後に上記の build と `scripts/bringup.sh input` を実行する。
2. Notebook の Docker で `atd-web` を実行し、Notebook ホストの Chrome で `http://127.0.0.1:8766` を開く。「データ転送」で Jetson の SSH 接続先を設定する。
3. 「コントローラ」のモニターで Jetson が受け取った信号を確認する。「軸・ボタンの配置」は evdev 入力から標準軸・ボタンへの物理配置を設定する。設定開始 → 保存 → 再読込 → 全入力を離して Joy 出力を再開する。
4. 同じ画面の操作割り当てで操舵・スロットル・操作要求の役割を設定する。こちらも設定開始 → 保存 → 再読込 → 全入力を離して操作要求の出力を再開する。二つの設定モードは独立しており、Joy 側が設定中なら teleop の要求も無効になる。

物理配置は `/data/results/runs/input/dualsense.yaml`、役割設定は `/data/results/runs/input/teleop.yaml` に永続保存する。役割の初期値はすべて未割り当て。操舵・スロットルは標準軸と差し引く軸、倍率（負値で反転）を選択でき、ボタンは出力許可の要求・記録開始要求・記録終了要求へ割り当てる。保存済みと node への反映済みを区別し、未保存の編集中に監視更新で内容を置き換えない。接続先や bridge が変わった場合は設定を再取得してから操作する。

`atd_teleop` が生成するのは正規化した運転要求と操作イベント。`valid` や `enable_requested` は最終的な走行許可ではない。記録ボタンのイベントは `atd_recording` が実際のrosbag開始・停止へ接続する。記録ボタンだけの役割設定でも操作要求を再開でき、運転用軸が未設定なら運転要求は無効のままになる。出力許可判断・ATD の基板通信は後続実装。詳細は [atd_joy](../../ros2_ws/src/atd_joy/README.md)、[atd_teleop](../../ros2_ws/src/atd_teleop/README.md)、[atd_interfaces](../../ros2_ws/src/atd_interfaces/README.md)を参照する。

「コントローラ」→「プロポ・Pico」では、受信機 → S.BUS → Pico → USB → Jetson で受け取った入力の PWM 相当値（µs）と raw 値（0〜2047）、USB・S.BUS・操作経路・電源・異常・受信カウンタを監視する。CH1〜CH6を既定表示し、表示数を6／10／16chに切り替えられる。表示数は Notebook のブラウザに保持し、高いCHに役割があれば表示を自動で広げる。受信・ROS・APIでは常に全16chを保持するため、10chプロポへの変更で通信の型を作り直す必要はない。現在のUSB監視データから使用中のch数は判別できないため、表示数を送受信機の自動検出結果とは扱わない。Pico の CH1＝操舵・CH2＝スロットル・CH3＝操作権は固定で、この画面では変更しない。

µs は参照 Rev6 firmware の既定換算に合わせた参考値で、raw 172／992／1811 を1000／1500／2000 µsに対応させ、区間ごとに補間・整数丸めし端点で制限する。受信入力を見やすくする換算表示であり、ESC・サーボへの実出力や実測波形ではない。役割の判定閾値は引き続き raw 単位を使う。未受信・期限切れの値を1500 µs等で補完しない。

Jetson 側の「記録の開始・停止」「予備1」「予備2」は CH4〜CH16 へ割り当てられ、初期値は全て未割り当て。低側・高側の raw 閾値、反転、位置の確認時間を設定できる。使用するプロポが送信できるCHを選ぶ。設定開始 → 保存 → 再読込 → 操作要求を再開の順で反映し、保存先は `/data/results/runs/input/propo.yaml`。既存の version 1 の設定も利用できる。設定開始は役割イベントだけを停止し、Pico の基本手動操作は継続する。再開には正常で新鮮な RC 入力が必要で、現在のスイッチ位置を基準に取り直す。通常は安定した低側→高側で記録開始要求、高側→低側で停止要求となる。中間位置とチャタリングでは要求を出さない。予備2枠は状態表示のみ。記録要求は `/rc/events` から `atd_recording` へ渡し、Joyと同じ記録セッションを開始・停止する。

`atd_vehicle_interface` は Rev6 の監視用 CDC1 / if02 を USB 属性と Pico 固有 serial で識別し、`INFO` と `STREAM ON` だけを送信する。複数 Pico があれば自動選択せず、`./scripts/bringup.sh input pico_serial:=<ID>` で選ぶ。必要なら `pico_sensor_path:=/dev/serial/by-id/<CDC1>`・`pico_config:=<YAML>`・`rc_profile:=<YAML>` を指定できる。CDC0 と走行指令は使わず、firmware の制御処理は変更していない。firmware ソースは [Pico firmware](../../ros2_ws/src/atd_vehicle_interface/firmware/pico_rev6/README.md) として同じ ROS package 内で Git 管理する。詳細は [Pico 受信](../../ros2_ws/src/atd_vehicle_interface/README.md)、[プロポ役割](../../ros2_ws/src/atd_rc_input/README.md)、[要件3.10節](../../docs/project_requirements.md#310-rev6-pico-firmware-の確認2026-11-18)を参照する。ホスト検証と Jetson Docker・実 USB・実機の確認は別である。

## rosbagの記録状態

コントローラ画面のJoy／プロポに共通のカードで、記録待機・開始処理・記録中・終了処理・エラーを観測する。開始停止はJetsonに接続したJoy・プロポの割当操作で行い、Webの記録操作endpointは追加しない。直近の要求受付、保存先、状態時点の経過、前回の保存確定と件数・サイズを区別して表示する。

`GET /api/recording/status` はSSH経由でmanagerの状態を観測する。元stampからの経過時間にbridgeとbrowserの経過を加え、受信が止まった状態を現在の「記録中」として保持しない。未受信・古い状態・Jetson接続断は停止済みと補完しない。UI／SSH接続が切れても記録は継続し、Jetson側の入力操作で停止できる。[ボタン割当と記録手順](../../docs/recording.md)

## 車両設定

Jetson で `./scripts/build.sh --packages-up-to atd_bringup` 後、`./scripts/bringup.sh manual` を起動する。D455・車体static TF・入力・bridgeが起動する。センサだけの `d455` profile を使う場合は `bridge` を別ターミナルで起動する。

「車両設定」では上面図・側面図に寸法線を表示し、入力欄と図の対応箇所を強調する。ホイールベース、前後トレッド、車体外形、後輪軸から後端までの長さと、D455の取付位置・姿勢を mm／度で編集する。寸法を説明する模式図と、入力値の同一縮尺プレビューを区別する。原点は後輪車軸中央（+X前／+Y左／+Z上）で、床面ではない。

空欄は未測定として保持し、印刷フレームのCAD参考寸法を車体外形や取付TFへ自動転記しない。カメラ取付TFを有効にするには位置・姿勢の全6値が必要。RealSense内部TFはdriverが管理する。

設定はSSH経由でJetsonの `/data/results/runs/vehicle/layout.yaml` に保存する。保存と適用は別で、static TF nodeを再起動するまで配信値は変わらない。画面では未保存・保存済み・再起動待ち・適用未確認・適用済みを区別し、通信断や古いnode状態を適用済みとして表示しない。接続先変更時も編集中の値を保持し、再取得してから保存する。走行操作やnode再起動はこの画面から行わない。[実測・TF確認の手順](../../docs/vehicle_layout.md)

## 構成と依存

| 配置 | 責務 |
| --- | --- |
| `frontend/` | React / TypeScript / Vite。依存は pnpm lock で固定 |
| `backend/` | FastAPI、資産・設定・ジョブ管理、SCP、公式 ROS CLI、Jetson 接続 |
| `processing/` | Web に依存しない HDMap 幾何処理・ライン生成 |
| `../../ros2_ws/src/atd_mapping/` | 保存地図の localize と点群 snapshot の取得 |
| `../../ros2_ws/src/atd_web_bridge/` | Jetson の ROS 観測・設定と loopback HTTP |
| `../../ros2_ws/src/atd_joy/` | evdev の入力取得・正規化を行う C++ component |
| `../../ros2_ws/src/atd_teleop/` | 軸・ボタンの役割を設定し、運転要求と操作イベントを生成する C++ component |
| `../../ros2_ws/src/atd_vehicle_interface/` | Pico の監視用 USB CDC を受信する C++ component |
| `../../ros2_ws/src/atd_rc_input/` | CH4〜CH16 の Jetson 側役割と記録要求を処理する C++ component |
| `../../ros2_ws/src/atd_vehicle_description/` | 車両配置profileと車体・カメラ取付のstatic TF |
| `../../ros2_ws/src/atd_sensors/` | D455の実機対応モード照会・TUI・検査付き起動 |
| `../../ros2_ws/src/atd_bringup/` | D455・車体TF・入力監視の統合起動 |
| `../../ros2_ws/src/atd_recording/` | Joy・プロポのイベントから共通のrosbag2 recorderを管理し、MCAPと終了結果を確認 |
| `../../ros2_ws/src/atd_vehicle_calibration/` | Notebook の bag・VO 軌跡から速度・加速度・操舵応答を解析 |
| `../../ros2_ws/src/atd_interfaces/` | 正規化した運転要求・操作イベントの ROS message 定義 |

Docker の追加 apt 依存は [dependencies.apt](../../docker/dependencies.apt)、追加処理は [Dockerfile.atd](../../docker/Dockerfile.atd)。cuVSLAM・公式 mapping・visual mapping・VGL の apt package を使う。Node / pnpm と Web 用 Python 環境は Notebook 専用で、Jetson の通常起動では Web 依存を同期しない。

Docker の Web 用 Python は `docker/python/web/` の独立した uv project / lock と、workspace の `.atd-python/<architecture>/web` を使う。最小曲率 helper は `docker/python/raceline/` と `.atd-python/<architecture>/raceline` の Python 3.12 環境を使い、通常の `atd-web` 起動で同期する。API は ROS の `PYTHONPATH` を引き継がず、公式 CLI を実行する子プロセスでだけ ROS 環境を読み込む。ローカル開発用には `backend/pyproject.toml` と `uv.lock` があり、`processing` をローカル依存として参照する。

モデルのダウンロードや対象 GPU の engine build は、apt インストール成功とは別の操作・検証である。公式手順: [cuVSLAM](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)、[地図作成](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/visual_global_localization/tutorials/tutorial_map_creation.html)。

## 検証の境界

ローカルの API・保存競合・capture の資産整合と成功条件・geometry・worker・SSH 引数・入力 profile の単体テスト、実際の QP / IQP solver を使う小規模閉路テスト、frontend の型検査・build を確認した。teleop の C++ 単体検証、役割設定・bridge/API の単体テスト、疑似 Jetson データを使ったブラウザの保存・再読込・要求出力再開も確認した。車両配置のnull・単位換算・保存競合・適用判定、純粋C++のTF計算と、疑似APIによる上面／側面の寸法選択・保存・通信断時の編集保持も確認した。実 GPU と実 bag による地図作成・localize・点群取得、ROS package の build / install・型生成・service 結合、Docker image build、実 SCP、Bluetooth / evdev、実走行は未検証であり、対象機で別に確認する。
