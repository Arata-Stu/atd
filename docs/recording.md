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

## 終了と実機での確認

録画を止め、「保存確定」とbag保存先を確認してから bringup を終了する。managerの通常終了時も記録の終了処理を行うが、強制終了・電源断まで正常保存を保証するものではない。終了処理を確認できなかったbagは保持し、後から中身を調べる。

Jetson の Docker 内で、実際に表示された保存先を指定する。

```bash
ros2 topic echo /recording/status --once
ros2 bag info /data/record/rosbags/<bag_id>
```

最初は短い記録で、Joy開始→プロポ停止、その逆、同じボタンの再押下、必須ストリーム不足、通常終了を確認する。地図用bagでは左右画像の件数・stampの対応・欠落、CameraInfoと `/tf_static` の内容を別に検査する。「保存確定」はカメラの実効FPSや地図作成の成功を保証しない。

ホスト上の状態機械・設定・API・UIの検証と、ROS build/install・実デバイスからのイベント・MCAP書込み・実カメラの欠落検証は分けて扱う。
