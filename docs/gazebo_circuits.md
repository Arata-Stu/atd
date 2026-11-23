# お台場の Gazebo コースと地図作成

## お台場を2周して再訪時の追跡を確認する

`odaiba` はシティサーキット東京ベイの屋外 SKY TRACK。施設の走行ガイドから計時線・走行方向・中心線を読み取り、公称400 mを基準に1/5とした。代表幅は図から推定した1.4 m、平滑化したカメラ台の経路は1周79.66 m。3 m/sでは2周53.11シミュレーション秒、1280×720・30 Hzの左右生画像は走行区間で約2.94 GBになる。MCAPにはこれに発着前後のデータやメッセージ管理情報が加わる。[公式走行ガイド](https://city-circuit.com/beginners/)・[屋外コースの公称距離](https://city-circuit.com/private/)

ピット・テラス・ブリーフィングルームの位置は2025年12月の大会施設図を走行ガイドに合わせた。屋根の開口と柱、デッキの手すり、青白の低いバリアは施設写真を参考にした。建物の高さ・奥行き・支柱間隔は描画用の概算で、カメラは既存のRC車載設定（光学中心高12 cm、左右基線9.5 cm）を使う。[大会施設図](https://kart-ev-div.city-kart.jp/wp-content/uploads/2025/12/b93fd2f6f6482e8449159fbfb90ace03.pdf)・[屋外コースの写真](https://city-circuit.com/wp-content/themes/city-circuit/assets/img/home/facility01-1.jpg)・[テラスの写真](https://city-circuit.com/wp-content/themes/city-circuit/assets/img/home/facility05.jpg)

![左・右の光学中心から見たピットとテラス。生成したSDFをVTKで描画した形状プレビュー](images/gazebo-odaiba.jpg)

Notebook での再生試験では、2周走行時に3 m/sでループ閉じ込みを確認し、5 m/sでは確認できなかった（ユーザー報告）。3 m/s・2周を1280×720、30 Hzのステレオで記録するには、準備済みの Ubuntu Notebook ホストで次を実行する。メニューからは `atd gazebo` → `bag` → `odaiba` を選ぶ。

```bash
atd gazebo bag --course odaiba --speed 3 --laps 2 --resolution 1280x720 --camera-hz 30 --imu off
```

Notebook の Isaac ROS 開発コンテナで、次を実行する。`atd build` は初回、ワークスペースに `atd_bringup` を準備するときだけ実行する。bag の選択画面では `gazebo-odaiba` で絞り込み、先ほどの記録を選ぶ。

```bash
atd build --packages-up-to atd_bringup
atd replay --tracking vo
```

2周目のピット前で、RViz の橙の矢印が青い Gazebo 正解軌跡のどこにあり、どちらを向いているかを見る。矢印は直近の推定姿勢を表し、表示数は **SLAM odometry (recent) → Keep** で調整できる。ピンクの `Loop closure landmarks` は cuVSLAM が閉じ込み処理に使う点群。各周で最初に非空点群を観測した地点が端末に出る。[cuVSLAM の ROS 出力](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)

bag の末尾で、同時刻の姿勢グラフの頂点・辺を、cuVSLAM の時刻付き姿勢と照合する。半周以上離れた別の周の姿勢を直接結ぶ辺があり、短い時間間隔の辺をたどっても両端がつながると、端末に「周回間の閉路を姿勢グラフで確認」と出る。表示された周回内距離と GT 上の隔たりから、コースのどの場所同士が結ばれたかを確認できる。よく似た隣のレーンを誤って結ぶ場合もあるため、RViz の正解軌跡と推定姿勢、または Web UI の周回別軌跡と合わせて確認する。

端末に出る結果ディレクトリの `summary.json` に判定を保存する。`loop_closure_graph.status` は、閉路を確認した場合が `cross_lap_cycle`、照合できた範囲で見つからなかった場合が `not_observed`、データ不足などで照合できなかった場合が `unavailable`。点群の更新数と初出地点は `loop_closure_cloud` に保存する。[姿勢グラフの配信実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-5.0/isaac_ros_cuvslam/src/impl/posegraph_vis_helper.cpp)・[時刻付き姿勢の取得実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-5.0/isaac_ros_cuvslam/src/visual_slam_node.cpp)

## 実行方法

### 計時線から最初の折返しまで

計時線から約4 m進むと、最初の折返しに入る。進入側の外周に斜材のある柵、金属トラス、機材箱、奥行きの異なる樹木を配置し、折返しの島には長さ42 cmの低い柵を置いた。北ゲートとプレス区画を配置の目安にし、個々の物体の寸法・位置は推定した。[大会施設図](https://kart-ev-div.city-kart.jp/wp-content/uploads/2025/12/b93fd2f6f6482e8449159fbfb90ace03.pdf)・[屋外コースの写真](https://city-circuit.com/wp-content/themes/city-circuit/assets/img/home/facility01-1.jpg)

![同じGazebo境界から生成したHDMapとスタート方向の左眼像。計時線と4 m地点を同じ姿勢で変更前後比較](images/odaiba-start-hdmap.jpg)

VTKで左右を描画した1280×720のモノクロ画像では、ORBの左右対応数が計時線で160→398組、4 m地点で106→390組、5 m地点で95→324組となった。照合には左右相互の最近傍比0.75、垂直ずれ1.5 px以内、水平視差1〜257 pxを用いた。cuVSLAMの追跡と閉路は、この景観で新しく収録したbagを再生して評価する。[同じ境界で2Dベンチマークを実行](simulation.md#お台場の同梱-hdmap-で走る)

### Gazebo の起動設定

準備済みの Linux Notebook では `atd gazebo` から操作・速度・周回数・解像度・撮影レート・IMU を選べる。コースIDは `odaiba`、ルートは全周の `lap`。発着の計時線を毎周同じ向きで通過する。[イメージの準備・動画・配信・診断](gazebo_monza.md)

| 設定 | 選択肢・単位 |
|---|---|
| `--speed` | RC車の m/s。お台場で省略すると1周33シミュレーション秒（約2.41 m/s） |
| `--laps` | 正の整数。動画・bag・地図の既定は1周。連続配信は省略すると Ctrl+C まで周回 |
| `--resolution` | 片眼の幅×高さ。424x240、640x480（既定）、848x480、1280x720 |
| `--camera-hz` | シミュレーション時刻で30、60、90 Hz（既定） |
| `--imu` | off（既定）、200、400。加速度は200 Hz、ジャイロと統合 IMU は指定した Hz |
| `--duration` | 動画・bag・地図では1周のシミュレーション秒数で、速度とどちらかを指定する。benchmark・diagnose では実時間の計測窓 |

## 1周でcuVSLAM地図を作る

### 記録した bag からオフラインで作成する

上記の `atd gazebo bag` で記録した2周のbagを使える。1周だけ記録する場合は `--laps 1` を指定する。Gazebo は左右の画像と CameraInfo、固定 TF をMCAPに保存し、走行後に各周の発着と時刻を検査する。IMUを選択した場合は生の加速度・ジャイロと統合 IMU も検査する。

確定した bag は Notebook の記録先の `rosbags/gazebo-odaiba-…`、走行ログは結果先の `runs/gazebo/odaiba/<run-id>/` に保存される。MCAP検査の詳細は `run.json` にある。bag 名には開始日時（日本時間）、解像度・撮影レート、IMU 設定、周回数、速度が入る。

Notebook の Isaac ROS 開発コンテナで `atd-web` を起動する。Chrome の「地図とライン」→「rosbag から地図を作成」で先ほどの bag を選び、「新しい地図ID」に例えば `odaiba_01` を入力して「cuVSLAM地図作成を開始」を押す。成功すると `/data/results/maps/odaiba_01/nvidia/cuvslam_map/` に保存される。後から VGL・点群・HDMap の処理にも元の bag を使える。[Notebook の地図画面と準備](../apps/web/README.md)

### 走行と同時に地図を作る場合

Gazebo 専用イメージを準備し、Notebook ホストで `atd dev` を実行して Isaac ROS 開発コンテナを起動する。コンテナ内で初回に `atd build --packages-up-to atd_bringup` を実行する。別端末の Notebook ホストで次を実行する。

```bash
atd gazebo map --course odaiba --speed 3 --laps 2 --resolution 1280x720 --camera-hz 30
```

Gazebo と cuVSLAM がステレオ通信・初期追跡を確認してから発進し、指定周回の最終ゴールで地図を保存する。結果先の `maps/gazebo-odaiba-<run-id>/` に地図本体と `mapping.json` ができる。途中の追跡に途切れがある場合は `save.tracking.coverage` を確認する。次に VGL 地図や点群・HDMap まで作る場合は、元の bag と保存画像を利用できるオフラインの手順を使う。[Notebook の地図作成手順](../apps/web/README.md)

## 保存地図とステレオ画像で閉ループ走行する

Notebook の Gazebo が計時線で静止した車両から左右の画像を配信する。Isaac ROS は VGL で初期位置を求め、cuVSLAM で保存地図上の位置を追跡する。Planner がその位置から前方の軌道と速度を出し、Pure Pursuit と FF＋PID の指令で車両が動く。次の画像はその車両位置から描画される。Gazebo の正解姿勢は周回・境界の採点に使う。

1. お台場の走行 bag を Notebook の Web UI「地図とライン」で選び、cuVSLAM 地図を作る。以下では地図 ID を `odaiba_1p5` とする。
2. 同じ地図で VGL 地図と対応する Notebook 用 engine を準備し、点群を取得する。点群に合わせてコース境界を編集し、閉路の速度付き Centerline、Raceline または Customline を保存する。同梱の2Dサンプルの座標は Gazebo 基準なので、保存地図と重ねるには位置合わせが必要になる。[地図とラインの操作](../apps/web/README.md)
3. Notebook ホストで `atd dev` を起動し、開いたコンテナ内で変更をビルドする。

```bash
atd build --packages-up-to atd_bringup
```

別の端末で、**Ubuntu Notebook ホスト**から起動する。`--speed` はラインに設定した目標速度の上限（m/s）。解像度は保存した VGL 地図から選び、撮影レートは既定で30 Hz。地図とラインを対話的に選ぶ場合は `atd gazebo closed-loop` を使う。

```bash
atd gazebo closed-loop --course odaiba --map odaiba_1p5 --line centerline --speed 1.5 --laps 2
```

Raceline の速度上限を 10 m/s で試す場合は、Web UI「地図とライン」で「速度上限」を 10 にして Raceline を再生成し、Notebook ホストから次を実行する。

```bash
atd gazebo closed-loop --course odaiba --map gazebo --line raceline --speed 10 --laps 2 --rviz
```

各地点の制御目標は、保存したラインの目標速度と `--speed` の小さい方になる。カーブの横加速度や前後の加減速制限によって、保存した目標速度は 10 m/s より低くなることがある。

模擬車両の全開時の速度は既定で 3.2 m/s。`--speed` がこれを超えると、その値に合わせて全開時の速度と模擬スロットル FF 表を設定する。走行上限だけを変えて同じ車両で比較するときは、両方の試行に同じ `--full-throttle-speed`（例 `--full-throttle-speed 10`）を付ける。全開時の速度は各試行の `closed_loop.host.json` に `full_throttle_speed_mps` として保存する。

同じモデルの VGL engine が複数ある場合、現在の GPU・TensorRT に適合すれば、地図の `vgl.json` の `runtime_model_dir` に記録された build を使う。別の適合する build で比較するときは、`--engine-dir` に結果ディレクトリからの相対パスを渡す。

走行中に RViz で追従を確認するには、Notebook のデスクトップ端末で `atd dev` を起動しておく。地図 ID が `gazebo` の場合は、別のホスト端末で次を実行する。

```bash
atd gazebo closed-loop --course odaiba --map gazebo --line raceline --speed 1.5 --laps 2 --rviz
```

今の 30 Hz 条件から本番想定の 90 Hz に切り替えるには、同じコマンドに `--camera-hz 90` を加える。追跡中は新しいステレオ観測の時刻で自己位置を更新し、その姿勢を受けて計画、Pure Pursuit、速度 FF＋PID を順に計算する。推定が欠けたフレームでは新しい指令は生成されない。ここでの Hz はシミュレーション時間を基準とする。

上面図の緑が選択した走行ライン、青が Gazebo の後輪軸中心の GT、オレンジが保存地図上の自己位置推定。緑と青を見比べると、模擬車両がラインを外れた区間を追える。GT は最初の有効な推定姿勢と同じ観測時刻で一度だけ地図に合わせるため、青とオレンジで比較できるのは初期位置合わせ後のずれになる。

矢印は GT と推定それぞれの最新の位置と向きを表す。更新時刻が異なる場合、その時間差も矢印の間隔に含まれる。追跡を失うと推定矢印が消える。RViz は試行終了時に閉じる。画面なしで評価するときは `--rviz` を省略する。

`--rviz` では「SLAM landmarks (live)」が走行中の SLAM の特徴点（青灰色）、「Map used for localization」が保存地図との位置合わせに使われた点群（紫）を表示する。紫の点は cuVSLAM が `/visual_slam/vis/localizer_map_cloud` に配信したときに見える。照合の成否は試行のログで確認できる。[NVIDIA の可視化トピック一覧](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html#ros-topics-published)

同じ計時線から Raceline を試すときは `--line raceline` に切り替える。シミュレーションでの Pure Pursuit の最短先読み距離は Centerline・Customline が0.35 m、Raceline が0.65 m。同梱のお台場の Raceline は計時線中央から約0.49 mずれており、停止時にも先読み円とラインが交わるようにした。編集したラインに合わせて `--min-lookahead` で上書きできる。

車両は保存地図上で追跡が成立し、有効な計画・制御指令を受けると発進する。Gazebo の閉ループでは、画像ごとの `map → odom` 発行は不要。位置合わせ後は、観測時刻までに得た最新の地図補正と、その時刻の VO オドメトリ・追跡診断から車体姿勢を求める。補正が届く間の移動量は VO の推定に依存する。

自己位置と計画の更新が一時的に途切れると模擬車両はニュートラルで減速し、同じ追跡区間で新しい有効な組がそろうと走行を再開する。明示的な追跡喪失は試行の停止理由になる。車両は前進用の運動学的自転車モデルで、既定のスロットル FF 表と操舵・速度応答はシミュレーション用の設定値。

端末に表示される `runs/gazebo/odaiba/<run-id>/` の `closed_loop.result.json` で完走周回数、周回タイム、境界への接触や停止理由を確認できる。`scene/rig_motion.jsonl` は Gazebo の車体姿勢と制御状態の時系列、`closed_loop.ros.log` は初期位置推定・追跡・制御のログ。初期化中に終了した場合は `closed_loop.host.json` にも理由が残る。

発進時の地図照合を確認するには、該当する試行の保存先で次を実行する。

```bash
grep -nE 'Trying to localize in map|Successfully localized at|Failed to localize in map|保存地図の照合成功を診断で確認|atd_gazebo_control.*driving' closed_loop.ros.log
```

`Trying to localize in map` に照合先のパス、`Successfully localized at` に cuVSLAM が求めた初期位置が出る。モニターが成功診断を受け取った行と、制御が最初に `driving` を出した行には、同じ追跡区間を特定する `session`・`epoch` とそれぞれの画像時刻（ns）が残る。

### 耐久走行を Web UI で観測・調整する

地図 ID が `gazebo` の場合、Notebook ホストで次を実行する。TUI では `closed-loop` を選び、周回数に `0`、走行中の表示に `web` を指定する。`--web --rviz` の併用もできる。

```bash
atd gazebo closed-loop --course odaiba --map gazebo --line centerline --speed 1.5 --laps 0 --web
```

計時線を通過するたびに完了周回数とラップタイムを記録する。手動で終了するには Gazebo を起動した端末で Ctrl+C を押す。結果に完了周回数、ラップタイム、最後の観測までの走行距離が残る。境界接触・追跡喪失などでも試行は終了する。走行ログの容量は走行時間とともに増える。センサー連続配信だけを試すときは `atd gazebo serve --course odaiba --laps 0` を使う。

Web UI 更新時は Notebook の Isaac ROS コンテナで `atd-web prepare` を一度実行し、`atd-web` を起動する。「ライブ」の観測先で **Gazebo（Notebook）** を選ぶと、同じ Notebook 上の bridge から自己位置と走行ライン、目標・推定速度、操舵角指令、スロットル指令を確認できる。折れ線の時間軸はシミュレーション秒。走行中にブラウザを閉じても計測は続く。[ライブ画面](http://127.0.0.1:8766/#live)

「パラメータ調整」で `/atd_gazebo_control` を選び、「現在値を取得」して変更した値を適用する。実行中の速度上限は起動時の `--speed` まで変更できる。Pure Pursuit の先読み距離・先読み時間や速度 PID も試せる。変更はその試行に適用され、`closed_loop.ros.log` に変更値とシミュレーション時刻が残る。

### 走行後に接触直前を確認する

Notebook の Web UI で「走行評価 → Gazebo 閉ループ」を開き、該当する Raceline の走行結果を選ぶ。境界に達した走行では接触直前のグラフとコース図を同じ時刻で確認できる。[Gazebo 閉ループの結果画面](http://127.0.0.1:8766/#closed-loop)

- 推定の横ずれ・向きの誤差は、最初の対応する観測で座標系を合わせ、その後の画像時刻で GT と照合する。向きが先にずれたか、GT と推定が一緒にラインを外れたかを見比べる。発進時からある地図上の絶対位置誤差はこの照合には表れない。
- Raceline の色は地図に保存された各地点の目標速度（m/s）を示す。コース内の最低速が青、中間が緑、最高速が赤。全点が同速なら緑一色で表示する。走行時の速度上限は試行の概要、実際の車速は時刻スライダー下の「GT 速度」で確認できる。
- 境界までの距離は GT に載せた車体外形から実際のコース境界まで測る。グラフは各観測時点の距離で、試行の接触判定には観測と観測の間を車体が通過した範囲も含む。操舵角は Gazebo の車両モデルが応答した角度を表示する。
- 画像から制御指令までの経過はシミュレーション時計で測る。cuVSLAM のトラッキング処理時間は実時間であり、双方の差をそのまま通信遅延には換算できない。
- 過去の走行で走行ラインのスナップショットがない場合は、参照元を確認して現在の地図のラインを参考表示できる。地図を編集した場合、当時のラインと異なる可能性がある。

更新後の `atd gazebo closed-loop` は、`hdmap.json` に検証済みの走行地図、`closed_loop.reference.json` に選択ラインと地図の SHA-256、`scene/scene.json` の `course_evaluation` に採点に使った舗装境界と車体寸法（m）を保存する。
`closed_loop.localization.jsonl` は初期位置探索・追跡・喪失などの状態を記録し、追跡中だけ画像時刻と地図上の後輪軸の姿勢を含む。`closed_loop.vslam.jsonl` は各ステレオ観測の VO 追跡とノードのコールバックの実時間（秒）。[NVIDIA のメッセージ定義](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-5.0/isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus.msg)
`scene/rig_motion.jsonl` の `command_stamp_ns` は指令の元画像時刻、`command_applied_ns` は Gazebo がその指令を採用したシミュレーション時刻（ns）。初回採用前は両方とも `null`。
