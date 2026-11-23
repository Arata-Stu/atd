# atd_control

プロポで手動走行中の後輪軸姿勢と planner の前方軌道から、Pure Pursuit が選ぶ目標・曲率・等価前輪舵角を観測する。購読は `/planning/status`、配信は `/control/status`。Notebook の別 bag 検証でも同じ Python 関数を使う。走行指令の配信や Pico への書き込みは行わない。

`bringup.sh autonomous` では、Jetson の `atd_auto_drive` が `/vehicle/drive_tuning` で地図・ライン・先読み・速度上限・PID の設定を制御器へ渡す。各制御器は採用時刻を付けて操舵を `/control/tuned_steering`、速度制御を `/control/tuned_speed` に配信する。Jetson は同じ設定と元観測に対応する両方の値を照合して Pico へ前進指令を送る。Web UI と rosbag は従来の観測 topic `/control/status`・`/control/speed_status` を使える。[低速前進の自動走行](../../../docs/bringup.md#低速前進の自動走行)

## 先読みと舵角

軌道は移動方向に並んだ map 座標の点列、車体原点は後輪軸中央。速度は元の自己位置観測の前後速度 [m/s]。前進用の計算は停車・前進のサンプルを扱う。

```text
L = min(max_lookahead_m, min_lookahead_m + |推定速度| × lookahead_time_s)
```

初期値は最小 0.35 m、時間 0.4 s、最大 1.5 m。推定速度 0、1、3 m/s の場合はそれぞれ 0.35、0.75、1.5 m になる。時間を 0 s にすると最小距離に固定する。初期値は実車での調整前の値。

後輪軸を中心とする半径 L の円と軌道線分の交点を線分上で補間し、車体前方の最初の交点を選ぶ。プレビュー全体が円内なら前方の終端を採用し、`preview_limited` を立てる。要求距離と実際の目標までの直線距離を両方返す。前方目標が存在しないときは理由付きで無効化する。

車体座標の目標を `(x, y)`、実際の距離を `d` とすると、曲率 [1/m] は `2y/d²`。車両配置 `dimensions.wheelbase` の値が実測済み（`source: manual`）なら、自転車モデルの等価前輪舵角 `atan(wheelbase_m × curvature_per_m)` を rad で配信する。左旋回が正。未測定・CAD 候補の場合は目標と曲率を表示し、舵角を欠測とする。舵角とサーボ PWM の対応は実車校正を要する。[Coulter, CMU-RI-TR-92-01](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm)

## ROS での起動

通常は Jetson の bringup から planner と一緒に起動する。planner が稼働していれば同じ Docker 内で個別に起動できる。

```bash
ros2 launch atd_control control.launch.py min_lookahead_m:=0.35 lookahead_time_s:=0.4 max_lookahead_m:=1.5
```

`profile` の既定は `/data/results/runs/vehicle/layout.yaml`。node 起動時に読み込む。目標・姿勢・舵角は同じ planner 元観測の時刻と地図・セッション・追跡世代を持つ。有効期限切れ、元時刻逆行、planner 無効、後退、または複数 planner を検出すると数値を失効させる。`valid` と `has_steering` を確認して表示する。

`control.launch.py` 単体では `drive_tuning` の既定は `false`。実車自動走行では `true` と正の `max_target_speed_mps` を渡す。変更を受けると古い計画と PID 履歴を破棄し、同じ地図・ラインで設定時刻より新しい計画から計算を再開する。設定が同じでも追跡を失えば無効状態を配信する。

将来 MPC / MPPI を追加する場合も `PlanningStatus` の姿勢・推定速度・軌道・速度列を利用できる。`ControlStatus.controller` は計算法を識別する。先読み距離・終端フラグは Pure Pursuit 固有の診断情報である。

## 速度制御のプレビュー

`speed_monitor` は同じ `/planning/status` の目標速度・車体前後速度から、前進を維持する CH2 入力を計算して `/control/speed_status` に配信する。校正で観測した速度範囲内だけで FF 表を線形補間する。目標速度が範囲外のとき、停止目標、後退中は計算値を欠測とし、速度・照合できた手動入力・欠測理由を表示する。

低速自動走行の `autonomous` profile では `max_target_speed_mps` を両制御器に渡す。速度制御は目標をこの値までに抑えてから FF・PID を計算し、上限での定常 FF 値を前進側の出力・積分の限界にする。停止周辺の VO の揺らぎ（±0.05 m/s）は先読みと PID の計算で 0 として扱い、速度 status には元の符号付き観測と目標からの実際の誤差を残す。新しい Pico 出力状態で自動走行中と確認できる間だけ PID 履歴を引き継ぐ。手動・待機・制動中、出力状態の途絶、地図・ライン・送信セッション変更時は FF＋P から始める。採用する FF 表は起動時に固定される。[実車出力の校正](../../../docs/vehicle_calibration.md#7-低速自動走行用の出力を校正する)

FF 値に P（目標−推定速度）、I（誤差の台形積分）、D（推定速度の変化率の負値）を加算する。前進速度が上がるほど CH2 raw が小さくなるプロポ設定では各補正の符号を反転する。出力と積分は校正で観測した CH2 範囲を使って制限する。ゲインの初期値は `speed_kp:=40.0`、`speed_ki:=8.0`、`speed_kd:=1.0`。単位はそれぞれ raw/(m/s)、raw/m、raw/(m/s²)。これは比較用の初期値で、実車の同定結果から得たゲインではない。0.5 秒を超す観測間隔、追跡の途切れ・地図／ライン／追跡世代の変化、表更新で積分・微分履歴を初期化する。

プロポ CH2 は `/vehicle/pico/telemetry` の 16ch 受信値から取り、速度観測と同時刻か最大 50 ms 前の最新フレームを使う。プロポ手動経路、基板電源、S.BUS 鮮度・状態、fault を確認する。比較する2つの値はどちらも S.BUS raw。プロポの操作を実際に ESC がどう解釈したかを示すには、別途 ESC 状態や実車の観測が必要になる。

DualSense には Jetson の `/teleop/command`（`source=joy`）の割り当て後の入力を使う。正規化スロットル [-1, 1] と enable ボタンの要求状態を、元の Joy 入力時刻から最大 250 ms まで照合する。同じ元時刻の再配信では鮮度を延ばさない。Notebook では元の入力単位と、3 系列を重ねる正規化・PWM 相当表示を選べる。表示用換算は Rev6 既定の CH2 の端点・中立と HOST 指令の極性を使う。校正表と PID は引き続き CH2 raw で計算する。[速度比較の操作と換算基準](../../../apps/web/README.md#速度制御を手動入力と比較する)

`manual_source` は Pico の報告と Joy の観測をまとめる。プロポ、HOST、HOST で Joy 入力あり、待機、異常、経路不明を区別する。HOST パケットの生成元を特定する情報は Pico のテレメトリには含まれない。Pico が未記録の bag でも有効な Joy 入力を確認できる。速度フィードバックには VSLAM の推定前後速度を使う。モータ印加電圧由来の RPM 信号は搭載後の応答評価に使う補助観測候補。

Jetson では Notebook の校正結果から選択した表を、速度 node の `feedforward_path`（既定 `/data/results/runs/vehicle/throttle_feedforward.json`）へ保存する。`/control/set_speed_table` はこの観測用の表と PID の履歴を更新する。ファイルとして配置または置換した場合も約1秒で反映される。別 bag の比較では同じ計算を元観測の時刻順に適用する。[校正表の作成](../../../docs/vehicle_calibration.md)
