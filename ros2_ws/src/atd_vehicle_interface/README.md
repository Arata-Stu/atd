# atd_vehicle_interface

JPBB Rev6 Pico の USB CDC1（センサ側、interface `02`）から `JPX1,T` を受け取る C++20 component / standalone node。常に全16ch・電圧・受信品質・基板が報告した操作経路・fault・RPM生情報を保持し、6ch／10chプロポで同じ受信経路を使う。UIは6／10／16ch表示を選択し、rawとPWM相当µsを併記する。µsは参照firmwareの既定校正による表示用換算で、Picoから取得した実出力値や波形測定値ではない。Pico の基本的な手動操作はこの node や Jetson に依存しない。

テレメトリ node が CDC1 に送るのは `INFO` と `STREAM ON`。DualSense 手動走行には別の `joy_drive` node が制御用 CDC0 を使う。firmware の編集元はこの package の [firmware/pico_rev6/](firmware/pico_rev6/README.md) に同梱する。

firmware は独立した CMake project で、通常の `colcon build` は ROS node だけを build する。小さい firmware のソース・テスト・説明は `share/atd_vehicle_interface/firmware/pico_rev6/` にも install し、SDK checkout と生成バイナリは含めない。Pico 用SDK・toolchainは別途準備する。firmware のホストテスト・ARM build・書き込み手順は上記READMEを参照する。ソースの同梱で、実基板へ同じ版が書き込まれたことを検証したとは扱わない。

## DualSense 手動走行

`atd_joy_drive` は Jetson の役割設定済み `/teleop/drive_command` を受け、USB CDC0 (if00) の JPB1 で Rev6 に操舵・スロットルを送る。入力の enable ボタンを設定した場合は保持を要する。現在の CH3 は同じ制御ポートの基板報告から判定する。接続時の中立・HOST 解除フレームで状態配信を開始し、有効な受信機と CH3 の HOST 許可、新しい中立の Joy 要求、中立での HOST 経路確立を順に確認してから操作値を送る。受信機なしの USB 接続では走行しない。

プロポ側では Pico が CH1・CH2 の値を直接使う。Joy の入力失効・設定変更・運転許可ボタン解放時は HOST を解除する。USB 断や送信停止時は Pico の watchdog が働く。負スロットルは Rev6 の逆側パルスを要求し、停止や後退開始の判定と中立操作は運転者が行う。基板上のパルス幅・機械的な応答は実機で確認する。

通常は `./scripts/bringup.sh joy-drive` を使う。CDC0 は排他で開き、ESC 手順試験と同時には使用できない。[CH3 切替と起動](../../../docs/bringup.md#dualsense-で手動走行する)

この node は CDC0 の JPB1 状態フレームを `/vehicle/joy_drive/status` に公開する。CH3 の受信パルス幅、操作経路、電源などの異常、サーボ・ESC の出力値、Joy の運転状態を Notebook から確認できる。出力値の基準時刻は Jetson が基板報告を受けた時刻で、元の操作時刻は `/teleop/command` が保持する。

## 機器の識別と受信

Linux の `/sys/class/tty` から VID/PID `CAFE:4002`、product `JPBB Rev6 Pico`、interface `02`、Pico 固有 serial を照合する。`ttyACM0/1` の順番では選ばない。候補が複数なら待機し、起動引数の `serial` または `sensor_path`（`/dev/serial/by-id/...if02...` 等）で絞る。指定パスでも USB の属性検査を省略しない。serial は Pico SDK が生成する16桁hexの固有ID。

非ブロッキング raw TTY と排他制御を使う。切断・入力途絶時は閉じて再探索し、再接続ごとに INFO / STREAM ON の応答を待つ。古いFIFOの残りを新しい操作として渡さないため、stream応答以前の telemetry は採用しない。USB 接続、JPX1 stream、S.BUS 入力の有効性は別の状態として表示する。

CSV の40フィールド（CRCを除く）、CRC16/CCITT-FALSE、数値範囲・bool値を検証する。過長行・NUL・非ASCII制御文字は区切りまで破棄し、切り詰めた文字列を受理しない。10ms周期の受信 callback が50msを超えて途切れた場合は、読み出す前に接続を閉じる。これは `telemetry_timeout_s`（既定0.3秒）とは独立したRC鮮度の上限であり、100ms等の停止後にOSへ滞留したボタン変化を新しい操作として渡さないための条件。再接続でsessionを更新し、INFO / STREAM応答後の新しい入力を下流の初期状態として受け取る。CRC不正・重複／逆順seqで鮮度を更新しない。32bit seq のwrapは許容し、64bit `board_us` の逆行は基板再起動として session を更新する。

## ROS の契約

| 方向 | topic / 型 | 内容 |
| --- | --- | --- |
| 配信 | `/vehicle/pico/telemetry` / `atd_interfaces/msg/PicoTelemetry` | 新しい受理済み telemetry ごと。入力時刻、接続session、全16chと基板状態 |
| 配信 | `/vehicle/pico/status` / `std_msgs/msg/String` | 10HzのYAML。接続・stream状態・理由・serial/path・各カウンタ・最新 telemetry object または null |

node / executable は `atd_vehicle_interface` / `atd_vehicle_interface_node`、component は `atd_vehicle_interface::PicoNode`。topic は namespace 相対。QoS は ROS コード側で reliable / volatile / keep-last depth1。service・制御topic購読は持たない。

`instance_id` は node 起動と接続 epoch の識別子。接続・基板再起動で変わる。`header.stamp` は受理した新しいフレームのホスト受信時刻で、状態再配信では更新しない。status.telemetry は `stamp`、`age_s`、`stale`、`rc_valid`、`rc_age_s` と元の全フィールドを持つ。status が更新されていても telemetry が新しくなったとは扱わない。

`rc_valid` は受信時点の観測条件であり、走行許可ではない。JPX1の受信期限に加え、`sbus_fresh`、`sbus_age_ms<=50`、frame_lost / failsafe のないこと、ホストで確認したS.BUSカウンタの進行を要求する。telemetryだけ再配信されてもRCの期限を更新しない。下流も受信期限・元stamp・sessionを確認し、再接続で保持ボタンを新しい押下として扱わない。`sbus_age_ms` は最後の構造正常フレームからの時間で、最後の正常制御値の年齢とは異なる。

`active_path` は0待機・1プロポ・2HOST・3異常。`fault_bits`、`sbus_flags` は元の値を保持する。`rpm_*` は基板入力で観測した累積エッジ数・周期・鮮度を表す。搭載予定のセンサの信号源はモータに加わる電圧で、機械的な回転との関係と応答の遅れは未測定。BEC 電圧は別の `vbec_mv` で取得する。

## 起動

低速自動走行では専用の `auto_drive` が制御ポート CDC0 を開き、CH3 の HOST 許可中に同時刻の Pure Pursuit・速度 FF＋PID を実測校正に従って送る。Jetson ホストの `./scripts/bringup.sh autonomous` から使用する。[校正、手動への復帰、追跡喪失時の停止](../../../docs/bringup.md#低速前進の自動走行)

自動走行の `/vehicle/auto_drive/status` は JSON を載せた `std_msgs/msg/String`。`command.stamp_ns` は JPB1 フレーム全体を USB に書き込んだ Jetson 時刻、`pico.stamp_ns` は Pico の状態フレームを受信した Jetson 時刻。両フレームの連番は独立している。`command.steering` と `throttle` は実送信した千分率を正規化した値で、制動時のスロットルは負。指令から換算した `servo_us`・`esc_us` は HOST 解除で null になり、基板の報告値は引き続き `pico` に入る。

`speed` は同じ地図・追跡区間・元観測時刻の目標速度と推定速度 [m/s]。CH3 がプロポ側でも表示でき、追跡喪失や期限切れで null になる。`last_stop` は最後に自動走行を停止または開始を抑止した時刻と理由で、プロポへの切替後も同じノードのセッション内で参照できる。

```bash
./scripts/build.sh --packages-up-to atd_web_bridge
```

単体起動は `ros2 launch atd_vehicle_interface pico.launch.py`。`config`、`serial`、`sensor_path`、`namespace`、`composed` を指定できる。`composed=false` で standalone。parameter は起動時固定で、既定値は `config/pico.yaml`。`telemetry_timeout_s=0.3`（.05〜5秒）、`scan_period_ms=1000`（100〜10000 ms）。複数台の場合は `serial:=<Pico固有ID>` で指定する。未接続・権限不足・別プロセス使用中でも異常終了せず待機し、UIの理由表示に使う。

Jetson の `scripts/dev.sh` は既定の privileged コンテナで `/dev` を共有し、起動後に追加・再接続された ttyACM を見せる。root 起動処理でホストの dialout と接続済み ttyACM の実 GID を通常ユーザーへ追加し、デバイスの所有者・mode は変更しない。この変更を初めて取り込む際は、利用中の処理を終了し `./scripts/dev.sh --build` で image を更新・コンテナを再作成する。通常の dialout 以外に属するデバイスを後付けする独自 udev 構成では、接続した状態で再作成して実 GID を反映する。実 Docker・USB の抜き差しは対象機で確認する。

## 検証

ROS 非依存の `test/test_protocol.cpp` はCRC、実frame配置、欠落・過長・範囲・分割受信、重複・逆順・wrap・reset・入力期限と受信callback間隔の50ms境界を検証する。`test/test_serial.cpp` は一時的なsysfs fixtureとPTYを使い、USB属性・CDC0除外・serial/path指定・複数候補・送信コマンド・排他・切断を確認する。実機デバイスは開かない。ROS環境では `colcon test --packages-select atd_vehicle_interface`。

同じ `colcon test` の `pico_firmware_host` で、同梱 firmware の C ホストテストと UF2 検証も実行する。これはSDK不要の検証で、ARM build や実基板への書き込みは行わない。GPIOの基板設計との照合は別途netlistを明示指定して行う。

これらのホスト検証と、ROS 型生成・build/install・JetsonのDocker／デバイス権限・実USB列挙・S.BUS・基板I/O・実車は別であり、後者は今回の実装だけでは検証済みとしない。
