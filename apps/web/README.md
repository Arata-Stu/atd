# ATD 共通 Web アプリ

Notebook の Chrome から使う React + TypeScript / Vite と FastAPI のローカルアプリ。地図・転送・車両校正・ライブ・コントローラ・車両設定を扱う。Jetson では観測・設定用の ROS bridge と入力ノードを実行する。

## 起動

オンラインで Notebook の Docker を起動し、初回はコンテナ内で Web UI を準備する。

```bash
# Notebook ホストから初回のイメージをビルドして入室
./scripts/dev.sh --build
# ここから Notebook の Docker 内
atd-web prepare
# bag 解析・地図作成に使う自作 ROS package は初回・更新時にビルド
atd-build --packages-up-to atd_control atd_vehicle_calibration atd_web_bridge
```

`atd-web prepare` は Web API と Raceline 用の Python 依存を同期し、画面をビルドする。Python 環境は選択した workspace の `.atd-python/`、フロントエンドの `node_modules` と `dist` は `.atd-web/frontend/` に残る。以前の自動インストール版から更新した場合も、このコマンドを一度実行する。その後は Web・Raceline の依存やフロントエンドを更新したときに、現地へ行く前に同じ workspace で再実行する。

現地では Notebook の Docker 内で次を実行し、Notebook ホストの Chrome で [Web UI](http://127.0.0.1:8766) を開く。

```bash
atd-web
```

通常起動は準備済みの環境・画面を使い、依存の取得やビルドを行わない。インターネット接続は不要。実車のライブ監視とデータ転送には Notebook と Jetson の間の接続が必要になる。

Gazebo の閉ループ走行は Notebook だけでライブ観測できる。`atd gazebo closed-loop --web` をホストで起動し、「ライブ」の観測先を「Gazebo（Notebook）」に切り替える。Notebook API は同じマシンの `127.0.0.1:8791` に接続する。走行中の速度・操舵・スロットルをグラフで確認できる。「車両を一時停止」してから「走行設定」で調整し、「走行を再開」で同じ試行を続けられる。[耐久走行の手順](../../docs/gazebo_circuits.md#耐久走行を-web-ui-で観測調整する)

```bash
# Notebook の Docker 内
atd-web check                     # Web / Raceline 環境と、地図作成・自己位置検証・車両校正用の ROS package を確認
atd-web build                     # frontend を明示的に再build
atd-web dev                       # 開発用 UI: http://127.0.0.1:5173
atd-web --port 8769                # 通常 UI / API の port を変更
atd-web --dry-run                 # 起動予定を表示
```

Notebook ホストから準備する場合は、コンテナを起動してから `./scripts/setup.sh --action web` または `./scripts/webui.sh prepare` を実行する。ホストからの通常起動は `./scripts/webui.sh` を使う。いずれも同じコンテナ内の `atd-web` に委譲する。

コンテナ・センサ・車両・記録はこの入口では自動起動しない。API は loopback 待受。通常利用はビルド済み frontend と API を同じ origin で配信する。

初回は上記の ROS build で Notebook の workspace に `atd_control`（`atd_planning`・`atd_localization`・`atd_mapping` を含む）、`atd_vehicle_calibration`、手動走行 bag の読取に使う `atd_web_bridge` を準備する。`atd-web check` は ROS と workspace の setup を読み込んで package の導入状態を確認する。環境や package が未準備なら `FAIL` と表示する。

## 保存済み bag のステレオ欠落診断

「コントローラ → 記録した走行」で Notebook に保存した bag を選び、「ステレオ欠落診断」で録画時の IR fps を指定する。左右の D455 IR 画像の `header.stamp` から推定欠落数、ギャップの位置、時刻の重複・逆行、左右ペアの不一致を表示する。推定は画像の時刻差に基づく。更新後は Notebook のコンテナで `atd-build --packages-up-to atd_web_bridge` と `atd-web prepare` を実行する。

## 履歴・地図・解析結果の整理

| 画面 | 操作 |
| --- | --- |
| ジョブ → 履歴を整理 | 終了したジョブを複数選択するか、開始日時の範囲または全件を選ぶ。日時はブラウザーの現地時刻で、日付だけでも指定できる。表示された対象と件数を確認して、履歴とログを削除する。 |
| 地図とライン | 選んだ地図を別 ID に複製するか、削除する。地図全体を選ぶと、その Notebook 上の cuVSLAM・VGL・点群・HDMap・走行ラインをまとめて扱う。HDMap 編集内容の削除では境界・ライン等を消去し、同じ地図と背景点群で編集をやり直せる。HDMap だけの複製は独立した編集用地図になる。 |
| VSLAM 軌跡誤差 | bag 名、VO／VIO、実行日時を照合して、その評価の軌跡と誤差を削除する。元の bag を消した後も操作できる。 |
| 自己位置検証・車両校正・2D シミュレーター | 各画面の保存履歴から実行結果を選んで削除する。自己位置検証では、前方軌道や速度比較の計算結果だけを選んで削除することもできる。 |
| 手動走行の記録 | 選んだ bag から抽出した操作・PWM・IMU の時系列を削除する。必要になれば元の bag から再度読み出せる。 |

ジョブ履歴と成果物は別々に管理する。これらの削除は Notebook に保存した対象へ適用され、元の rosbag や Jetson に転送済みの地図は引き続き使用できる。

新しい前方軌道・速度比較の SQLite は自己位置検証の実行フォルダー内に保存される。旧版の共有フォルダーにあるものは、ジョブ履歴から対象の走行を確認できた分を一緒に削除する。先にその履歴を消した場合、所属を特定できない旧版の内部インデックスが残ることがある。

## Gazebo 閉ループの走行結果

「走行評価」から Gazebo 閉ループ、2D シミュレーター、VSLAM 軌跡誤差、自己位置検証を選べる。Gazebo で接触した走行を見返すには、[Gazebo 閉ループ](http://127.0.0.1:8766/#closed-loop)でコース・地図・ラインを確認して走行結果を選ぶ。コース図とグラフは共通のシミュレーション時刻を使う。記録済みの結果は Gazebo や ROS を再起動せずに閲覧できる。[指標と走行ログ](../../docs/gazebo_circuits.md#走行後に接触直前を確認する)

## 2D 走行シミュレーター

Notebook で[シミュレーター](http://127.0.0.1:8766/#simulation)を開き、同じコースを固定先読みと速度連動先読みで走らせる。軌跡、横ずれ、目標・模擬速度、操舵と、仮定した遅延による違いを確認できる。[モデルと比較手順](../../docs/simulation.md)

「コース別評価」と「走行比較」では、同梱のお台場（1/5）と「地図とライン」で保存した閉路の HDMap を選べる。お台場サンプルは Centerline と Raceline を生成済みで、初回から評価できる。編集する場合は「地図とライン」→「HDMapを作成」で作成元にお台場を選び、新しい地図 ID にコピーする。選択したラインを持つ全地図をまとめて評価するか、1つの地図を選べる。保存した評価結果はコース・試行ごとに閲覧できる。[操作と指標](../../docs/simulation.md)

## 自己位置検証

「再生設定」で `map → odom` の地図補正元を選ぶ。既定の cuVSLAM は初期位置候補を cuVGL から受け取り、保存地図で追跡する。cuVGL を選ぶと cuVSLAM は局所 VO を担当し、cuVGL が照合した地図姿勢で補正する。照合頻度の既定は 1 Hz。0 Hz では初回成功後、次の手動トリガーまで照合を待つ。

Notebook の [自己位置検証](http://127.0.0.1:8766/#localization) を開き、cuVSLAM・VGL 準備済みの地図と、地図作成に使った走行とは別の bag を選んで検証を開始する。前方軌道を試したい Raceline・Centerline・Customline は、検証前に地図画面で生成・保存しておく。同じ名前の bag は親ディレクトリを含むパスで選択できる。ジョブのログと停止操作は同じ画面から開ける。

結果の地図には、選んだラインと追跡できた区間ごとの推定軌跡を重ねる。中断を選ぶと、最後に有効だった位置と、復帰した場合は最初に再取得した位置を確認できる。区間間の空白は位置が得られなかった範囲を表し、経路の補間は行わない。初期位置が一度も確定しなかった場合は、状態遷移と追跡率で確認する。

検証結果は `/data/results/runs/localization/<run>/` に保存される。既存の CLI 結果も履歴から開ける。新規検証では当時の全保存ラインを含む `hdmap.json` を一緒に保存する。過去にこのファイルを保存していなかった検証では、現在の地図の識別情報が結果に一致するときに重ねる。[検証用 ROS パッケージ](../../ros2_ws/src/atd_localization/README.md)

検証結果の「基準ライン」を選んで前方軌道を計算すると、時刻を選択・再生して、投影点から先の軌道・横ずれ・目標速度・推定速度を確認できる。同じ走行のライン比較には保存済みの姿勢・速度を再利用する。横ずれはラインの進行方向を向いて左側が正。Customline は編集後に速度付きのラインを生成しておく。[前方軌道の計算](../../ros2_ws/src/atd_planning/README.md)

「この bag で先読み設定を試す」で最小距離・先読み時間・最大距離を変更すると、同じ走行時刻の Pure Pursuit の追従点と曲率を比較できる。実測ホイールベースを mm で入力すると等価操舵角 [°] も表示する。bag の自己位置推定は一度でよく、設定はこの画面での再計算に使う。ライブ画面の「Jetson の追従設定」では、Jetson 起動時に適用した値を確認できる。[先読みと舵角の定義](../../ros2_ws/src/atd_control/README.md)

## 車両校正

「データ転送」で手動走行の bag を受信したら、「この bag を車両校正へ」→「解析を開始」と進む。Notebook が連続した VO 軌跡を準備し、プロポ入力・IMU と同期して速度、加速度、操舵応答を求める。同じ走行から作成した軌跡を再利用できる。進行中のログ、過去の解析、応答グラフと CSV／JSON をこの画面から確認できる。[記録から解析までの操作](../../docs/vehicle_calibration.md)

「車両校正」で走行結果を開くと、Raceline の曲率・横加速度・加減速の候補と前進スロットルのフィードフォワード表を確認できる。採用する制限と地図を選び、その地図で Raceline を生成すると反映される。速度フィードフォワード表は JSON で保存できる。[候補の算出条件と採用手順](../../docs/vehicle_calibration.md)

### 速度制御を手動入力と比較する

1. 「車両校正」で校正結果のスロットル表と速度域を確認し、「速度比較に採用」を押す。複数の走行から採用した表は出典と版で選び分けられる。
2. **別 bag** は「自己位置検証」で地図作成と別の走行を選び、Notebook の GPU で自己位置を推定する。保存済みの結果があればそのまま開く。Raceline・Centerline・Customline から試したいラインの前方軌道を計算し、校正表と Kp・Ki・Kd を選んで「この設定で速度制御を計算」を押す。スライダー・再生に連動して、その時刻までの直近 5 秒間を表示する。速度比較を再計算するときは保存済みの姿勢・速度と元 bag の手動入力を使う。
3. **ライブ** はプロポ走行なら Jetson で `./scripts/bringup.sh localization`、DualSense で運転しながら比較するなら `./scripts/bringup.sh localization joy_drive:=true` を起動する。DualSense は Jetson に Bluetooth 接続し、「コントローラ → 操作の役割」で操舵・スロットルの割り当てと入力状態を確認する。Notebook の「ライブ → Jetson の FF 表と PID 設定」で同じ校正表を選び、「Jetson の観測に適用」を押す。

「スロットル表示」の初期値では、計算スロットルと実プロポ入力を CH2 raw、DualSense の割り当て後の入力を正規化値で表示する。「共通正規化」または「PWM 相当」を選ぶと、3 系列を同じグラフで比較できる。目標・推定速度 [m/s] と操作経路も同じ時間軸で読める。

換算は Rev6 の既定値を使う。CH2 raw の 172／992／1811 は、順に正規化 +1／0／−1、PWM パルス幅 1000／1500／2000 µs に相当する。Pico の S.BUS 校正値や CH2 反転を変更した場合、プロポ入力の表示用換算と基板の出力値に差が生じうる。制動・後退の判定には ESC の状態も必要になる。

操作元は Pico が報告した経路と `/teleop/command` の入力を照合する。プロポ経路でも DualSense を操作した時刻には両方の入力が見える。HOST 経路で Joy を観測した場合、Pico のテレメトリから確認できるのは HOST 経路が有効だったところまで。Joy だけを記録した bag では正規化スロットルを表示し、操作経路は不明とする。[速度制御の計算条件](../../ros2_ws/src/atd_control/README.md)

## Map

「地図とライン」の右欄「地図を編集する準備」から各工程を開ける。完了した工程にはチェック、次に進める工程には濃い緑のボタンが付く。前工程待ちはグレー、実行中は青で表示する。

Notebook ホストで `atd gazebo bag --course odaiba --speed 3 --laps 2` を実行して作成した2周分の bag も、実車の記録と同じ「rosbag から地図を作成」から選べる。[Gazebo の記録手順と所要容量](../../docs/gazebo_circuits.md)

1. **cuVSLAM 地図を作成**: `/data/record/rosbags/` の bag、地図 ID、左右画像・CameraInfo の topic、base / optical frame を指定する。NVIDIA の `isaac_mapping_ros/create_map_offline.py` を独立 worker で呼び、`maps/<id>/nvidia/` に公式の `cuvslam_map/`・`map_frames/` 等の構成を維持して保存する。
2. **VGL 地図を準備する**。「VGL地図を作成」で Notebook に export 済みのモデルを選ぶ。入力解像度と export ID を確認して「VGL地図の準備を開始」を押すと、この PC の GPU 用 engine を生成し、公式 `create_cuvgl_map.py` で地図の `cuvgl_map/` を作成する。地図を選ぶ前でも「VGLモデル一覧」で export 済みのモデルを確認できる。[モデルの準備手順](../../docs/vgl_models.md)
3. **保存地図から点群を取得**。「保存地図から点群を取得」で、元 bag と VGL 準備時の Notebook 用モデルを自動選択する。必要なら詳細で選び直す。Notebook 内の画像転送には `atd replay` と同じ Fast DDS の共有メモリ対応設定を使う。取得に失敗したら「ジョブとログ」で対象のジョブを選び、「エラーをコピー」を押す。bag に記録された左右画像の最大間隔、cuVSLAM が処理したフレームの最大間隔の警告、自己位置合わせの成否を同じ抜粋で確認できる。
4. **HDMap を編集**: 取得後の点群を背景に、左右境界と Custom 点列を Canvas で編集・保存し、走行ラインを生成する。処理成功後は選択中の地図と背景・ラインを自動更新する。未保存編集や別地図への選択変更を、ジョブ完了で破棄しない。

地図を開いたときと「全体を表示」を押したときは、編集済みの境界・ラインに倍率を合わせる。未編集の地図では保存した走行軌跡と点群の集中範囲を使い、遠方の孤立点を倍率の計算から除く。元の点群は保持され、ホイールで縮小すると周囲も確認できる。

点群取得では元の保存地図を作業用に複製し、元 bag の画像を再生する。[atd_mapping](../../ros2_ws/src/atd_mapping/README.md) が VGL の姿勢 hint と保存地図への localize 成功を確認し、その成功区間の非空点群・軌跡と TF を採取する。点群が既に `map` frame なら再変換せず、それ以外では確認した TF を一度だけ適用する。TF 不足や自己位置未確定を原点・identity 変換で補わない。点群は全地図の landmark 網羅性を保証せず、採用時刻・入力末尾との差等は provenance に残す。

成功した snapshot と manifest を `hdmap/captures/<capture_id>/` に保存し、revision と `source.pointcloud_status=ready` を更新してから編集可能にする。公式地図の生成成功と、表示用点群の取得成功は別の状態である。NVIDIA 地図は点群取得前の編集・ライン生成を禁止する。空の HDMap は手動編集を開始でき、任意の背景として `map` 座標・メートル単位の JSON 点群を読み込める。

cuVSLAM 地図の内容 hash、VGL 地図・保存画像・モデルの対応情報、地図生成時の平面拘束設定がない旧資産は、開始前の検査で再生成を案内する。未記録値の推測や既存資産の自動移行は行わない。新規地図は平面拘束を無効と明示して記録し、点群取得もその記録と同じ設定を使う。

Notebook の地図作成では `cuvslam.cfg_slam_max_map_size=0`、点群取得では `slam_max_map_size=0` を指定する。どちらも姿勢グラフの保持数が無制限になる。新規地図の作成値は `mapping.json` の `source.mapping_options.slam_max_map_size`、点群取得時の値は `snapshot.json` の `provenance.parameters.cuvslam.slam_max_map_size` で確認できる。すでに縮約された旧地図を全入力から比較し直すには、同じ bag を使って新しい地図 ID で作成する。[NVIDIA の release-5.0 地図作成設定](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mapping_and_localization/blob/release-5.0/isaac_mapping_ros/configs/map_creation_config.yaml#L42-L46)

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

Raceline の色は、保存された目標速度の最小値が青、中間値が緑、最大値が赤。凡例に各速度を m/s で表示する。全周が等速なら緑一色で「一定」と表示する。

## Transfer

「データ転送」で USB（`192.168.55.1`）または Hotspot（`10.42.0.1`）を選ぶ。普段使う Wi-Fi の IP も名前を付けて登録できる。追加したプリセットは Notebook の同じブラウザで次回も選べる。「接続設定を保存」で選んだ宛先を転送とライブ監視に適用する。

SSH ユーザーの初期値は `atd`。Jetson ホスト上の bag ディレクトリは `/home/atd/workspaces/atd/data/record/rosbags`、地図ディレクトリは `/home/atd/workspaces/atd/data/results/maps` が初期値で、実際の配置に合わせて変更できる。

Jetson からは一覧で選んだ停止済み bag を受信し、Notebook からは保存済みの地図を送信する。USB 用 IP 以外の宛先では、いずれも開始前に接続先を確認する。判定には転送先のホスト名・IP を使う。Notebook のネットワークインターフェースや実際の経路は検出していない。

鍵認証と既存の `known_hosts` を使い、password・秘密鍵をアプリ設定に保存しない。SCP は一時領域へ転送し、転送前後のファイル内容 SHA256 を確認して新規ディレクトリへ公開する。既存 ID は上書きしない。転送完了と Jetson が地図を読み込み走行に適用したことは別である。

ジョブの状態・入力・ログ・中止要求は `/data/results/runs/web/` に保持する。UI や API サーバーの終了だけでは独立 worker を停止せず、再接続で状態を復元する。Docker 停止・OS 再起動からの処理の自動再実行は行わない。

ジョブを開始すると右下の「ジョブ」に実行件数が反映される。「ジョブ」または対象の「実行ログ」を押すとログを開ける。

## Live / Input

### 手動走行の操作元と PWM を確認する

Jetson で `./scripts/bringup.sh joy-drive` を起動し、Notebook の [コントローラ](http://127.0.0.1:8766/#input) を開く。Joy とプロポのどちらの設定タブでも、CH3 の位置、Pico が選んだ経路、割り当て後の DualSense の操舵・スロットル、サーボ・ESC の基板報告値を同じ画面で確認できる。

CH3 を Jetson 側にした直後は、Joy の中立確認が済むまで待機する。運転元「DualSense」は、Pico が HOST 経路を報告し、Jetson の手動走行 node もその経路で運転中のときに表示する。PWM の数値は Pico の制御ポートが返した出力パルス幅 [µs]。個々の Joy 入力への応答を特定したり、物理端子の波形を計測したりするものではない。制御ポートの報告が途切れた区間は欠測になる。[Jetson の起動と操作](../../docs/bringup.md#dualsense-で手動走行する)

走行後は「コントローラ → 記録した走行」で転送済みの bag を選び、「時系列を読み出す」を押す。CH3・操作元・DualSense とプロポのスロットル・ESC の PWM に、VSLAM の前後速度と IMU の前後加速度を同じ時刻で照合できる。「次の切替」で後退に移った時刻へジャンプできる。VSLAM 速度には走行中の記録、同じ bag の解析結果、Notebook GPU で新たに生成した連続 VO 軌跡を使える。[減速・停止付近・後退の確認手順](../../docs/recording.md#減速停止付近後退を確認する)

### 自動走行の指令と基板出力を比較する

Jetson で `./scripts/bringup.sh autonomous`、Notebook の Docker 内で `atd-web` を起動する。「データ転送」に Jetson の SSH 接続先を設定し、[ライブ画面](http://127.0.0.1:8766/#live)を開く。CH3 には Pico が受信したスイッチの位置、操作元「自動」は Pico が HOST 経路を報告し、自動走行ノードの状態と新しい USB 送信指令が一致したときに表示する。発進待ち・制動・停止要求・情報不足は区別して表示する。

「ライブ」の観測先を選び「走行設定」を開く。速度付きで保存済みの Centerline・Raceline・Customline から参照ラインを選び、速度上限 [m/s]、先読みの最小・最大距離 [m] と時間 [s]、速度 PID の Kp・Ki・Kd を調整できる。「変更を適用」で編集項目を一括送信し、接続先から値を読み戻す。「計画中のライン」には planner で観測したラインが表示される。再起動後は起動設定から始まる。

直近 15 秒の折れ線で目標速度と VSLAM 推定速度 [m/s] を比較する。操舵・スロットルは Jetson が USB に送った正規化値で、スロットルの負側は自動走行中のブレーキを表す。PWM [µs] は送信指令に対応するパルス幅と Pico が報告した値を照合する。横軸はそれぞれ速度の元観測、指令の送信、基板報告の受信に対応する Jetson 上の時刻。基板報告は車輪・サーボ角の実測ではない。追跡喪失による停止要求は縦線で示し、プロポに戻した後も理由と時刻が残る。未受信区間は折れ線が途切れ、前回取得した値には時刻を付けて表示する。

### Notebook でライブプロットを画面収録する

「ライブ」の「走行モニター」では、自動走行中の車両と Pure Pursuit の先読み点を、速度・操舵・スロットルのグラフと同時に確認できる。紫の丸が先読み点、破線の起点がその制御計算に使った後輪軸の位置。凡例の距離はこの2点間の直線距離 [m]。選択した Centerline・Raceline・Customline と有効な制御観測に従って表示し、追跡喪失や観測の失効時は先読み点を消す。Jetson の手動走行中に追従計算を起動した場合は、自動操舵のプレビューとして読める。

観測先を「Gazebo（Notebook）」にすると、緑の自己位置推定の矢印に加えて、接触判定に使う車体外形を青枠で表示する。青枠は Gazebo の正解姿勢（GT）で動き、前端の印が進行方向を示す。「GT 追従」を押すと車体周辺が拡大され、Raceline を走るときのコーナー内側の余裕を確認しやすい。最初に同時刻の GT と推定姿勢で座標系を合わせるため、初期位置合わせが済むと表示される。その後は推定を失っても、新しい GT が届いていれば青枠が動く。

「ライブ」→「録画用表示」を開き、Notebook の画面収録ツールで Chrome のウィンドウを選ぶ。地図上の車両の位置・向きと、速度・操舵・スロットルの直近 15 秒のグラフを一画面に表示する。CH3、操作元、停止要求の理由も同じ画面で確認できる。[録画用表示を直接開く](http://127.0.0.1:8766/#live?view=capture)

地図は観測先で走行に使っている地図を自動選択する。Notebook に保存した HDMap のコース境界と選択ラインを背景に使い、地図を照合できたら追跡中の自己位置を矢印で重ねる。矢印の先が車両の前方。追跡喪失や地図の版違いは地図欄に表示し、現在位置の矢印を消す。「通常表示に戻る」→「詳細・設定」で点群を含む地図やパラメータも確認できる。

- **取得往復**は、Notebook のブラウザが最新状態を要求してから JSON を読み終えるまでの時間。SSH、Jetson の応答処理、Notebook 側の処理を含む。
- **基板報告の経過（受信時）**は、Jetson が Pico の PWM 報告を読み取ってから、その報告を含む JSON を Notebook が読み終えるまでの推定範囲。Jetson で読み取ってから応答用に採取するまでが 20 ms、取得往復が 80 ms の場合は 20〜100 ms になる。両端末の時計は独立でよい。速度や PWM の折れ線には引き続き信号ごとの元時刻を使う。

画面収録は、運転中に見えた状態や停止までの経緯を映像で確認する用途に使える。通信が途切れると画面の更新も止まる。数値を残したい走行では、Jetson の起動時に `./scripts/bringup.sh autonomous recording_profile:=inputs` を選ぶと、カメラ画像を除いた操作・制御・推定状態などをボタンで記録できる。後で画像から VSLAM を再実行する走行には、左右 IR・CameraInfo・TF などを含む `mapping` を使う。[記録プロファイル](../../docs/recording.md)

### 入力監視と設定

Jetson で自作 ROS package を build し、入力・teleop 操作要求・bridge を明示的に起動する。

Pico 対応を初めて取り込むときは、Jetson の利用中の処理を終了して `./scripts/dev.sh --build` で image とコンテナを更新する。USB の後付け・再接続用 mount と通常ユーザーの serial group を起動時に準備するため、旧コンテナへの再接続だけでは反映されない。

```bash
# Jetson の起動済み ATD コンテナを利用
./scripts/build.sh --packages-up-to atd_web_bridge
./scripts/bringup.sh input
```

入力 node が既に起動済みなら `./scripts/bringup.sh bridge`。bridge の設定は [atd_web_bridge](../../ros2_ws/src/atd_web_bridge/README.md) を参照する。既定 topic は実際の bringup の topic・型へ合わせる必要がある。bridge の設定ファイルを用意して `./scripts/bringup.sh input config:=/absolute/path/bridge.yaml` で指定できる。
`input` profile は Joy・teleop・Pico 受信・プロポ役割処理・bridge、`bridge` profile は bridge だけを起動する。`input` では `teleop:=false` で Joy の役割処理、`propo:=false` で Pico とプロポ役割処理を除外できる。起動 profile は `./scripts/bringup.sh --list` で確認し、引数なしの対話端末でも選べる。Notebook では `./scripts/bringup.sh --dry-run input` による確認だけを行い、実際の起動は Jetson の既存 Docker 内で行う。いずれも基板出力・bag 記録は実行しない。[bringup の操作](../../docs/bringup.md)も参照する。

ライブ画面では Jetson が使用中の地図を自動選択する。地図 ID、cuVSLAM 地図と HDMap の SHA256 を照合し、追跡中に同じ時刻で得た位置・向き・車体前後速度と実走軌跡を Jetson で選択したラインに重ねる。前方軌道と横ずれ、同じ観測時刻の目標速度と推定速度を表示する。追跡喪失、通信途絶、地図不一致の理由も確認できる。[別 bag の検証と Jetson の起動](../../docs/bringup.md#プロポ手動走行中に自己位置を確認する)

Notebook backend は SSH 経由で Jetson の `127.0.0.1:8790` へ接続する。Notebook は車載 DDS に参加しない。Live では地図・pose・軌跡・速度の観測と ROS parameter の読書きを扱い、欠落・古いデータは表示で区別する。

parameter の閲覧は node prefix、変更は node と parameter 名の allowlist で指定する。既定では変更許可なし。descriptor の型・範囲・read_only を検査し、atomic 変更後の値を再取得する。変更を永続 YAML へ自動保存しない。

Input は Jetson に Bluetooth 接続した DualSense の生入力・正規化入力・接続状態と、物理配置・操作の役割を同じ画面で扱う。Bluetooth のペアリングは Jetson ホスト側で行う。ブラウザの Gamepad API や Notebook 接続のコントローラは使わない。

1. Jetson のホストでコントローラを Bluetooth 接続し、Docker 起動後に上記の build と `scripts/bringup.sh input` を実行する。
2. Notebook の Docker で `atd-web` を実行し、Notebook ホストの Chrome で `http://127.0.0.1:8766` を開く。「データ転送」で Jetson の SSH 接続先を設定する。
3. 「コントローラ」のモニターで Jetson が受け取った信号を確認する。「軸・ボタンの配置」は evdev 入力から標準軸・ボタンへの物理配置を設定する。設定開始 → 保存 → 再読込 → 全入力を離して Joy 出力を再開する。
4. 同じ画面の操作割り当てで操舵・スロットル・操作要求の役割を設定する。こちらも設定開始 → 保存 → 再読込 → 全入力を離して操作要求の出力を再開する。二つの設定モードは独立しており、Joy 側が設定中なら teleop の要求も無効になる。

物理配置は `/data/results/runs/input/dualsense.yaml`、役割設定は `/data/results/runs/input/teleop.yaml` に永続保存する。役割の初期値はすべて未割り当て。操舵・スロットルは標準軸と差し引く軸、倍率（負値で反転）を選択でき、ボタンは出力許可の要求・記録開始要求・記録終了要求へ割り当てる。保存済みと node への反映済みを区別し、未保存の編集中に監視更新で内容を置き換えない。接続先や bridge が変わった場合は設定を再取得してから操作する。

`atd_teleop` は正規化した運転要求と操作イベントを配信する。DualSense 走行時は `atd_joy_drive` が CH3、Joy の入力状態、Pico の操作経路を確認して CDC0 に運転指令を送る。記録ボタンのイベントは `atd_recording` が rosbag の開始・停止へ接続する。記録ボタンだけを割り当てた場合も記録操作は利用でき、運転用軸が未設定の間は運転要求が無効になる。詳細は [atd_joy](../../ros2_ws/src/atd_joy/README.md)、[atd_teleop](../../ros2_ws/src/atd_teleop/README.md)、[atd_interfaces](../../ros2_ws/src/atd_interfaces/README.md)を参照する。

「コントローラ」→「プロポ・Pico」では、受信機 → S.BUS → Pico → USB → Jetson で受け取った入力の PWM 相当値（µs）と raw 値（0〜2047）、USB・S.BUS・操作経路・電源・異常・受信カウンタを監視する。CH1〜CH6を既定表示し、表示数を6／10／16chに切り替えられる。表示数は Notebook のブラウザに保持し、高いCHに役割があれば表示を自動で広げる。受信・ROS・APIでは常に全16chを保持するため、10chプロポへの変更で通信の型を作り直す必要はない。現在のUSB監視データから使用中のch数は判別できないため、表示数を送受信機の自動検出結果とは扱わない。Pico の CH1＝操舵・CH2＝スロットル・CH3＝操作権は固定で、この画面では変更しない。

µs は参照 Rev6 firmware の既定換算に合わせた参考値で、raw 172／992／1811 を1000／1500／2000 µsに対応させ、区間ごとに補間・整数丸めし端点で制限する。受信入力を見やすくする換算表示であり、ESC・サーボへの実出力や実測波形ではない。役割の判定閾値は引き続き raw 単位を使う。未受信・期限切れの値を1500 µs等で補完しない。

Jetson 側の「記録の開始・停止」「予備1」「予備2」は CH4〜CH16 へ割り当てられ、初期値は全て未割り当て。低側・高側の raw 閾値、反転、位置の確認時間を設定できる。使用するプロポが送信できるCHを選ぶ。設定開始 → 保存 → 再読込 → 操作要求を再開の順で反映し、保存先は `/data/results/runs/input/propo.yaml`。既存の version 1 の設定も利用できる。設定開始は役割イベントだけを停止し、Pico の基本手動操作は継続する。再開には正常で新鮮な RC 入力が必要で、現在のスイッチ位置を基準に取り直す。通常は安定した低側→高側で記録開始要求、高側→低側で停止要求となる。中間位置とチャタリングでは要求を出さない。予備2枠は状態表示のみ。記録要求は `/rc/events` から `atd_recording` へ渡し、Joyと同じ記録セッションを開始・停止する。

`atd_vehicle_interface` は Rev6 の監視用 CDC1 / if02 を USB 属性と Pico 固有 serial で識別し、`INFO` と `STREAM ON` だけを送信する。複数 Pico があれば自動選択せず、`./scripts/bringup.sh input pico_serial:=<ID>` で選ぶ。必要なら `pico_sensor_path:=/dev/serial/by-id/<CDC1>`・`pico_config:=<YAML>`・`rc_profile:=<YAML>` を指定できる。DualSense で実際に運転する場合は Jetson の `joy-drive` または `manual` を起動し、CH3 が HOST のときに別 node から CDC0 / if00 へ操作値を送る。firmware ソースは [Pico firmware](../../ros2_ws/src/atd_vehicle_interface/firmware/pico_rev6/README.md) として同じ ROS package 内で Git 管理する。詳細は [Pico 受信](../../ros2_ws/src/atd_vehicle_interface/README.md)、[プロポ役割](../../ros2_ws/src/atd_rc_input/README.md)、[要件3.10節](../../docs/project_requirements.md#310-rev6-pico-firmware-の確認2026-11-18)を参照する。ホスト検証と Jetson Docker・実 USB・実機の確認は別である。

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
| `../../ros2_ws/src/atd_planning/` | 手動走行・別 bag 共通のライン投影と前方軌道・速度計算 |
| `../../ros2_ws/src/atd_control/` | 速度適応 Pure Pursuit、速度 FF＋PID、プロポと DualSense の入力・操作経路との時刻照合 |
| `../../ros2_ws/src/atd_localization/` | 別走行 bag による保存地図での自己位置推定と追跡区間の検証 |
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

Docker の Web 用 Python は `docker/python/web/` の独立した uv project / lock と、workspace の `.atd-python/<architecture>/web` を使う。最小曲率 helper は `docker/python/raceline/` と `.atd-python/<architecture>/raceline` の Python 3.12 環境を使う。API は ROS の `PYTHONPATH` を引き継がず、公式 CLI を実行する子プロセスでだけ ROS 環境を読み込む。ローカル開発用には `backend/pyproject.toml` と `uv.lock` があり、`processing` をローカル依存として参照する。

モデルのダウンロードや対象 GPU の engine build は、apt インストール成功とは別の操作・検証である。公式手順: [cuVSLAM](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)、[地図作成](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/visual_global_localization/tutorials/tutorial_map_creation.html)。

## 検証の境界

ローカルの API・保存競合・capture の資産整合と成功条件・geometry・worker・SSH 引数・入力 profile の単体テスト、実際の QP / IQP solver を使う小規模閉路テスト、frontend の型検査・build を確認した。teleop の C++ 単体検証、役割設定・bridge/API の単体テスト、疑似 Jetson データを使ったブラウザの保存・再読込・要求出力再開も確認した。車両配置のnull・単位換算・保存競合・適用判定、純粋C++のTF計算と、疑似APIによる上面／側面の寸法選択・保存・通信断時の編集保持も確認した。実 GPU と実 bag による地図作成・localize・点群取得、ROS package の build / install・型生成・service 結合、Docker image build、実 SCP、Bluetooth / evdev、実走行は未検証であり、対象機で別に確認する。

## ESC 試験

Studio の [ESC 試験](http://127.0.0.1:8766/#esc) で前進・制動・停止確認・中立・後退を順番に配置し、出力と時間を設定する。Jetson の試験用 bringup が起動すると予約できる。Jetson 端末で対象と手順を確認して開始し、Notebook から基板報告 PWM、指令、推定 ESC 状態と履歴を確認する。[準備・操作手順](../../docs/bringup.md#タミヤ-esc-の手順試験)
