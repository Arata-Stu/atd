# atd_rc_input

Pico の検査済み USB telemetry を受け、プロポの補助CHをJetson側の役割へ割り当てる C++20 component / standalone node。Picoが直接扱う CH1＝操舵・CH2＝スロットル・CH3＝操作権は変更しない。手動走行やPico設定を書き換える処理は持たない。

`vehicle/pico/telemetry`（`atd_interfaces/msg/PicoTelemetry`）を購読し、`rc/status`（YAML String、10Hz）と `rc/events`（`OperatorEvent`）を配信する。記録役割の安定した Low→High で RECORD_START、High→Low で RECORD_STOP。反転で開始／停止の向きを交換できる。これは要求イベントであり、rosbag実行・記録成功通知ではない。予備役割は状態のみを配信する。

設定は `/data/results/runs/input/propo.yaml` に保存する。`roles.recording / auxiliary_1 / auxiliary_2` は未割当(0)またはCH4〜16、重複不可。6chを基本としつつ、10chプロポの入力も同じ型で扱える。全て未割当から始める。各役割の `low_threshold / high_threshold` はrawの0〜2047で低側 < 高側、`inverted` はbool。閾値700／1300と `debounce_ms=100` は初期候補で、実入力に合わせてUIで設定する。UIのPWM相当値（µs）は監視用の換算表示で、役割の閾値やPicoの校正・出力設定は変更しない。profile schemaはversion 1を維持する。

`~/configure`（SetBool）で役割イベントの生成を中断し、`~/reload_profile`（Trigger）で検証済みファイルを反映する。保存・再読込・再開は個別操作。再開には有効なプロポ信号が必要。設定モードはPicoの手動操作を止めない。起動・再接続・設定変更・入力途絶の後は現在のスイッチ位置を取り直し、保持されている位置から記録イベントを生成しない。

元の入力stampを保持し、USB stream/session・sequence・board時刻・S.BUSフレーム数の進行・flags・50msの受信期限を検査する。telemetryだけが再配信されても古いS.BUSを新しい操作にしない。USB受信は最大250msで失効する。steady clockの監視とROS時刻の元stampを区別する。スイッチの中間帯ではイベントを生成せず、最後の安定位置を保つ。

通常は Jetson で `./scripts/build.sh --packages-up-to atd_web_bridge` 後、`./scripts/input.sh` でPico受信・Joy・役割設定・監視bridgeを起動する。単体は `ros2 launch atd_rc_input rc.launch.py`。component は `atd_rc_input::RcInputNode`、node/executable は `atd_rc_input_node`。

`test/test_core.cpp` はROS非依存で変化検出・debounce・再接続・入力失効・CH変更を検証し、`test/test_profile.py` はschemaと永続保存を検証する。ROS build・DDS/service結合・USB実機・プロポ実信号・実車は別途検証する。
