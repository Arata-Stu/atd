# Jetson の bringup

`scripts/bringup.sh` で実装済みの ROS 起動 profile を選び、Jetson の起動済み ATD Docker 内で実行する。先に `scripts/dev.sh` でコンテナを起動し、必要な package を各ホストで build する。

```bash
# Jetson ホスト、プロジェクト直下
./scripts/build.sh --packages-up-to atd_web_bridge
./scripts/bringup.sh --list
./scripts/bringup.sh input
```

| profile | 起動する処理 | ROS launch |
| --- | --- | --- |
| `input` | Joy入力、teleop、Pico USB受信、プロポ役割処理、観測・設定bridge | `atd_web_bridge input.launch.py` |
| `bridge` | 観測・設定bridgeのみ。入力や役割nodeは別途起動する | `atd_web_bridge bridge.launch.py` |

現在の両 profile は Jetson ローカル専用で、車両への出力や rosbag 記録は開始しない。Notebook は Web UI と解析を担当し、車載 DDS に参加しない。Docker の準備・ROS package の build・ROS node の起動は別の操作である。

## 選択と引数

```bash
./scripts/bringup.sh                  # 対話端末でprofileを選択
./scripts/bringup.sh bridge           # bridgeのみ
./scripts/bringup.sh --dry-run input  # 実行予定だけ表示。Notebookでも可
./scripts/bringup.sh --dry-run        # inputの実行予定を表示
./scripts/bringup.sh input pico_serial:=PICO_SERIAL
./scripts/bringup.sh input rc_profile:=/data/results/runs/input/propo.yaml
./scripts/bringup.sh bridge config:=/absolute/path/bridge.yaml
```

引数なしの対話端末では fzf を使い、未導入時は番号で選択する。非対話では `input` / `bridge` を明示する。`--dry-run` は起動せず予定するコマンドだけを表示する。

profile の後ろの `name:=value` は対応する ROS launch へ渡す。`input` では `profile`・`teleop_profile`・`rc_profile` が永続設定、`joy_config`・`teleop_config`・`pico_config`・`rc_config` が各 node の起動設定、`config` が bridge の設定を指す。`pico_serial` / `pico_sensor_path` で Pico を選び、`teleop:=false` / `propo:=false` で各入力経路を除外できる。`bridge` は bridge 用の `config` を受ける。詳細は [bridge package](../ros2_ws/src/atd_web_bridge/README.md) と [Web アプリ](../apps/web/README.md)を参照する。

起動はフォアグラウンドで行う。終了は起動したターミナルの Ctrl+C を使い、同じ node を重複起動しない。設定・コード変更後の再起動と、profile の保存・再読込・イベント出力再開を区別する。

## profile の追加方針

日常操作の入口は `scripts/bringup.sh`、profile 一覧は `scripts/lib/bringup.sh`、実際の node・parameter・起動構成は各 ROS package の `launch/`・`config/` に置く。新しい profile は実装済みの launch と必要な引数・対象ホストを対応させ、一覧・操作説明・起動テストも更新する。

現段階では入力・監視の2 profile を提供する。センサ・自己位置推定・車両I/O・自律走行をまとめる必要が生じた時点で、専用の `atd_bringup` package と起動単位を検討する。未実装の profile や空 package を先に作らない。起動スクリプトのホスト検証と、Jetson Docker 内の ROS 結合・実機の検証は分けて扱う。
