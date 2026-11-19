# Joy・プロポからの rosbag 記録

Jetson の `atd_recording` が Joy・プロポの操作イベントを受け取り、共通の rosbag2 recorder を開始・停止する。起動しただけでは録画しない。保存形式は MCAP、保存先はコンテナの `/data/record/rosbags/<bag_id>/`。ホストでは setup で指定した record 保存先の `rosbags/` に対応する。

## 起動

Jetson ホストのプロジェクト直下で実行する。

```bash
./scripts/build.sh --packages-up-to atd_bringup
./scripts/bringup.sh manual
```

`manual` は D455・車両TF・Joy・Pico受信・役割処理・記録待機・bridge を起動する。D455 の設定 TUI は従来どおり利用できる。Pico 内で完結するプロポの手動走行と、Jetson の記録処理は独立している。

| bringup | 記録の既定 | 用途 |
| --- | --- | --- |
| `joy-drive` | `inputs` | D455 を使わず、プロポと DualSense の切替・操作・Pico の出力を記録 |
| `manual` | `mapping` | D455 の左右IRを使う地図用記録 |
| `input` | `inputs` | カメラを必要としない入力・基板状態の記録 |
| `d455` / `bridge` | managerなし | センサのみ／観測bridgeのみ |

```bash
./scripts/bringup.sh input
./scripts/bringup.sh manual recording_profile:=calibration
./scripts/bringup.sh manual recording_profile:=inputs
./scripts/bringup.sh manual recording:=false
./scripts/bringup.sh manual recording_config:=/absolute/path/recording.yaml
./scripts/bringup.sh --dry-run manual recording_profile:=mapping
```

`calibration` はプロポで手動走行して車両モデルを校正するための記録設定。左右の IR 画像と CameraInfo、静的 TF、分離した加速度・ジャイロ、Pico のテレメトリを必須にする。画像の記録は左右 IR に絞る。D455 の起動設定は別に選択できる。[車両校正用の走行と解析手順](vehicle_calibration.md)

`recording:=false` は manager の起動を除外する。`recording_config` はコンテナ内の独自 ROS parameter YAML の絶対パス。設定ファイルは [atd_recording/config](../ros2_ws/src/atd_recording/config/) を基に作り、対象topic・保存先・空き容量等を起動時に指定する。記録は明示したtopicを対象にする。

## ボタンの割当て

Notebook の Docker 内で `atd-web` を起動し、Notebook の Chrome で `http://127.0.0.1:8766` を開く。Jetson の SSH 接続先を設定し、「コントローラ」画面を使う。

**Joy** は物理配置を保存・反映した後、「操作の役割」で「記録開始」「記録停止」に別々のボタンを割り当てる。設定開始 → 保存 → 再読込 → 入力をすべて離して操作要求を再開、の順に行う。記録ボタンだけの役割設定も利用でき、運転用の操舵・スロットル軸が未設定なら運転要求は無効のままになる。初期のボタン割当ては未設定で、R1等を固定しない。

**プロポ** は「プロポ」の記録役割を利用できる CH4〜CH16 に割り当てる。設定開始 → 保存 → 再読込 → イベント出力再開、を完了する。通常は安定した低側→高側で開始、高側→低側で停止となり、反転設定で入れ替えられる。開始と停止は同じCHを使う。Pico の走行用 CH1〜CH3 は変更しない。

再接続時や設定反映時に押されたままのボタン・保持中のスイッチ位置から、記録開始を合成しない。設定を再開した後、改めて操作する。

Joy とプロポは同じ記録セッションを操作する。Joy で開始した記録をプロポで停止することもできる。開始要求の重複で別のbagを同時に作らず、停止処理中の開始要求は保留せず拒否する。

## 状態と保存完了

UI の「rosbag 記録」は両入力タブに共通の観測表示で、開始・停止の操作受付と、recorder の実行状態・保存結果を分けて表示する。

| 状態 | 意味 |
| --- | --- |
| 待機中 | manager が待機している |
| 記録を開始中 | 必要なtopicや保存先を確認し、recorderを開いている |
| 記録中 | recorderの開始処理が完了した |
| 記録を終了処理中 | 停止・flush・metadataの確定を待っている |
| 記録エラー | 開始・継続・終了確認に失敗した。説明と前回結果を確認する |

「保存確定」は停止処理だけでなく、metadataと参照するMCAP・必要なtopicの記録結果を確認した後に表示する。元のbagは失敗時も保持し、自動削除・上書きしない。記録中のボタン要求は、停止後も「前回の終了結果」と区別して表示する。

`mapping` は左右IR画像・左右CameraInfo等の必須入力を検査する。InfraをOFFにした場合や必要なpublisherがない場合は、その理由で開始に失敗する。カメラ不要の入力診断には `inputs` を選ぶ。実際の一覧・QoS・制限値は [packageのREADME](../ros2_ws/src/atd_recording/README.md) と設定ファイルを参照する。

UI・SSHが切れても記録の停止操作は送らない。通信断・状態未受信・古い状態は「停止済み」と補完せず、現在の状態を未確認として表示する。Jetson上の入力操作で停止し、再接続して結果を確認する。

## 手動走行を Notebook で振り返る

1. Jetson の ATD コンテナを起動し、ホストで `./scripts/build.sh --packages-up-to atd_bringup` を実行する。カメラを使わない走行は `./scripts/bringup.sh joy-drive`、D455 も使う走行は `./scripts/bringup.sh manual` で起動する。割り当てたボタンで記録を開始・停止し、保存確定を確認する。
2. Notebook の Docker 内で初回・更新時に `atd-build --packages-up-to atd_web_bridge` を実行し、`atd-web` を起動する。「データ転送」で停止済みの bag を受信する。すでに Notebook の `/data/record/rosbags/` にある bag も選択できる。
3. 「コントローラ → 記録した走行」で bag を選び、「時系列を読み出す」を押す。グラフをクリックすると、同じ時刻の CH3 の位置、実際の操作元、正規化した操舵・スロットル（−1〜+1）、サーボ・ESC のパルス幅（µs）が並ぶ。「再生」で切替を追ったり、秒数を指定して移動したりできる。読み込みは Notebook 内で完結し、次回は抽出済みの結果を開ける。[記録した走行を開く](http://127.0.0.1:8766/#input?view=bag)

全標準記録プロファイルが次の topic を対象にする。追加した基板報告を受け取るには、走行中に `atd_joy_drive` を含む bringup を起動する。

| 記録する topic | 時系列で確認できること |
| --- | --- |
| `/teleop/command` | 割り当て後の DualSense の操舵・スロットル。プロポ走行中も Joy 側の有効な操作入力を表示する |
| `/vehicle/joy_drive/status` | Pico が制御ポート CDC0 で報告した CH3 のパルス幅・出力経路・サーボ／ESC の PWM、および Jetson の Joy 送信状態 |
| `/vehicle/pico/telemetry` | プロポが操作元だった区間の CH2 スロットル（S.BUS raw）。CDC1 だけの区間でも受信できた CH3 raw と基板の操作経路を表示 |
| `/localization/status` | 走行中に VSLAM を起動した場合の、追跡中の車体前後速度（m/s） |
| `/camera/camera/accel/sample`、`/camera/camera/imu`、`/tf_static` | センサ位置での前後加速度（m/s²）。記録済みの取付姿勢で加速度を車体の前方へ投影する |

共通軸の 0 秒は bag の記録開始時刻。Joy は元入力の時刻、Pico は Jetson が基板報告を受けた時刻を使う。同じ基板報告に対して後から Joy 送信の解除が通知されたときは、その通知を記録した時点から操作元を更新する。

CH3 の位置と実際の操作元には別の行を使う。CH3 が Jetson 側であっても、基板の HOST 経路と Joy 送信がそろった区間だけが DualSense になる。電源異常でパルスが無効の区間や報告が途切れた区間は PWM を欠測にする。制御ポートの報告を含まない旧 bag では保存されている系列だけを表示する。

### 減速・停止付近・後退を確認する

1. 後処理する走行は Jetson で `./scripts/bringup.sh manual` を起動する。既定の `mapping` 記録を使い、D455 の左右 IR と加速度を有効にする。車両配置にカメラの取付姿勢を設定し、`base_link` から両光学フレーム・加速度フレームまでの静的 TF を一緒に記録する。車両校正も行う日は `manual recording_profile:=calibration` で同じ確認ができる。
2. Notebook の「記録した走行」で bag を読み出し、「VSLAM 速度の出典」を確認する。記録済み速度がある場合はそれを既定とする。同じ bag の車両校正、自己位置検証、連続 VO 軌跡も選べる。ステレオ記録から新しく求める場合は「ステレオから VSLAM 速度を生成」を押す。Notebook の GPU で連続姿勢を生成し、車体の前後速度を計算する。処理状態とログは同じ画面で確認できる。生成した軌跡と cuVSLAM 地図は Notebook の `/data/results/maps/drive-vo-<ID>/` に保存され、次回から再利用できる。
3. 「速度の切替へ移動」で「後退に移行」→「次の切替」を選ぶ。直前に停止付近を確認できた場合は「直前の停止付近へ」で戻る。速度の絶対値が下がる区間をグラフでたどり、同じカーソルで操作元のスロットル、ESC の PWM、IMU の値を比較する。

速度は後輪軸中心の前方を正とする。表示上の「停止付近」は `|v| ≤ 0.05 m/s`。切替一覧は同じ推定区間で次の状態が 0.15 秒続いたことを確認し、その区間の始まりを示す。追跡喪失を挟む前進と後退は別の観測区間になる。この目安で機械的な完全停止や ESC の内部モードを断定することはできない。

IMU の表示は、センサ位置で測った比力を車体前方へ投影した値。重力と取付位置による回転成分を含み、車体が傾くと停車中にも値が出る。速度の符号と変化を合わせて読む。古い bag も、保存済みの各信号と出典が一致する後処理結果から表示できる。速度または IMU が欠けた区間はグラフを空白にする。

## 終了と実機での確認

録画を止め、「保存確定」とbag保存先を確認してから bringup を終了する。managerの通常終了時も記録の終了処理を行うが、強制終了・電源断まで正常保存を保証するものではない。終了処理を確認できなかったbagは保持し、後から中身を調べる。

Jetson の Docker 内で、実際に表示された保存先を指定する。

```bash
ros2 topic echo /recording/status --once
ros2 bag info /data/record/rosbags/<bag_id>
```

最初は短い記録で、Joy開始→プロポ停止、その逆、同じボタンの再押下、必須ストリーム不足、通常終了を確認する。地図用bagでは左右画像の件数・stampの対応・欠落、CameraInfoと `/tf_static` の内容を別に検査する。「保存確定」はカメラの実効FPSや地図作成の成功を保証しない。

ホスト上の状態機械・設定・API・UIの検証と、ROS build/install・実デバイスからのイベント・MCAP書込み・実カメラの欠落検証は分けて扱う。
