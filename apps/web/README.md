# ATD 共通 Web アプリ

Notebook の Chrome から使う React + TypeScript / Vite と FastAPI のローカルアプリ。Map・Transfer・Live・Input を同じ画面構成で扱う。Jetson ではブラウザ・Web frontend を起動せず、観測・設定用の ROS bridge と入力ノードを実行する。

## 起動

Notebook のホストで Docker を起動・入室し、コンテナ内から `atd-web` を実行する。
Docker 設定を更新した場合は、利用中の処理を終了してイメージを再構築する。

```bash
# Notebook のホスト: Docker を起動・入室（初回・設定変更時は --build）
./scripts/dev.sh --build
# ここから Notebook の Docker 内
atd-web
```

ブラウザは Notebook ホストの Chrome で `http://127.0.0.1:8766` を開く。FastAPI・frontend の build / 配信・地図作成等のジョブは Notebook の Docker 内で実行する。`atd-web` は専用の `web` 用 uv 環境を同期・選択して API を起動する。初回は frontend 依存の同期と build を行い、以後はソースの変更時だけ再buildする。source は readonly mount のまま、`node_modules` と `dist` は選択した workspace の `.atd-web/frontend/` へ生成する。

```bash
# Notebook の Docker 内
atd-web check                     # Web 環境と公式 ROS package を確認
atd-web build                     # frontend を明示的に再build
atd-web dev                       # 開発用 UI: http://127.0.0.1:5173
atd-web --port 8769                # 通常 UI / API の port を変更
atd-web --dry-run                 # 起動予定を表示
```

ホストから起動したい場合の補助入口として `./scripts/webui.sh` も用意している。これは既存コンテナへ `docker exec` で入って同じ `atd-web` を実行する。ホストに Web サーバーや Node / FastAPI を導入する経路ではない。

コンテナ・センサ・車両・記録はこの入口では自動起動しない。API は loopback 待受。通常利用はビルド済み frontend と API を同じ origin で配信する。

## Map

- `/data/record/rosbags/` の bag を一覧から選び、地図 ID、左右画像・CameraInfo の topic、base / optical frame を指定する。NVIDIA の `isaac_mapping_ros/create_map_offline.py` を別 worker で呼ぶ。
- 結果は `/data/results/maps/<id>/nvidia/`。公式の `cuvslam_map/`、`map_frames/` 等の内部名称を維持する。
- VGL 用地図は、Notebook で用意した export bundle を選択し、Notebook の GPU 用 engine を準備した上で公式 `create_cuvgl_map.py` を実行する。model 資産側の engine と地図の `cuvgl_map/` は別に管理する。Notebook の engine を Jetson 用として転送しない。
- 空の HDMap を作成し、左右境界と Custom 点列を Canvas で編集・保存できる。map frame の JSON 点群をインポートして背景表示する API も提供する。
- Centerline は左右境界の弧長対応、Raceline は領域・余裕を守る簡易平滑化、Custom は編集点の弧長再サンプリング。曲率と加減速の制約から目標速度を付ける。自己交差・領域外・余裕不足を拒否し、生成結果には方式と設定を記録する。最短ラップ最適化や実車の操舵・制動性能の検証を意味しない。
- 車幅・余裕・速度・加速度の初期値は車両の実測値として確定したものではない。使用する車両・走行条件に合わせて画面で設定する。
- 編集正本は `hdmap/map.json`、生成物は `lines/`。revision で保存競合を検出し、境界変更で既存ラインを失効させる。

公式地図生成が成功しても、HDMap 表示用点群の取得済みとは扱わない。現実装では保存地図への bag 再生・localize 成功確認・点群 snapshot 採取の自動工程は含まず、点群の明示インポートを用いる。

## Transfer

Jetson の SSH host・user・port、bag root と map root を画面で設定する。Jetson の bag ディレクトリを階層から選んで Notebook へ受信し、作成した地図ディレクトリを Jetson へ送信する。map root の例は Jetson ホストの `<result_dir>/maps`、bag root は `<record_dir>/rosbags`。コンテナ内の `/data/...` とホストのパスを混同しない。

鍵認証と既存の `known_hosts` を使い、password・秘密鍵をアプリ設定に保存しない。SCP は一時領域へ転送し、転送前後のファイル内容 SHA256 を確認して新規ディレクトリへ公開する。既存 ID は上書きしない。転送完了と Jetson が地図を読み込み走行に適用したことは別である。

ジョブの状態・入力・ログ・中止要求は `/data/results/runs/web/` に保持する。UI や API サーバーの終了だけでは独立 worker を停止せず、再接続で状態を復元する。Docker 停止・OS 再起動からの処理の自動再実行は行わない。

## Live / Input

Jetson で自作 ROS package を build し、入力と bridge を明示的に起動する。

```bash
# Jetson の起動済み ATD コンテナを利用
./scripts/python.sh ros build -- --packages-up-to atd_web_bridge
./scripts/input.sh
```

Joy が既に起動済みなら `./scripts/input.sh --bridge-only`。bridge の設定は [atd_web_bridge](../../ros2_ws/src/atd_web_bridge/README.md) を参照する。既定 topic は実際の bringup の topic・型へ合わせる必要がある。bridge の設定ファイルを用意して `./scripts/input.sh config:=/absolute/path/bridge.yaml` で指定できる。

Notebook backend は SSH 経由で Jetson の `127.0.0.1:8790` へ接続する。Notebook は車載 DDS に参加しない。Live では地図・pose・軌跡・速度の観測と ROS parameter の読書きを扱い、欠落・古いデータは表示で区別する。

parameter の閲覧は node prefix、変更は node と parameter 名の allowlist で指定する。既定では変更許可なし。descriptor の型・範囲・read_only を検査し、atomic 変更後の値を再取得する。変更を永続 YAML へ自動保存しない。

Input は Jetson に Bluetooth 接続した DualSense の生入力・正規化入力・接続状態と物理的な軸・ボタン配置を扱う。設定開始、profile 保存、再読込、入力出力再開を別操作とする。車両の動作割り当て・走行許可とは別の入力層である。詳細は [atd_joy](../../ros2_ws/src/atd_joy/README.md)。プロポの機器・入力経路は未決定で、対応済みとは表示しない。

## 構成と依存

| 配置 | 責務 |
| --- | --- |
| `frontend/` | React / TypeScript / Vite。依存は pnpm lock で固定 |
| `backend/` | FastAPI、資産・設定・ジョブ管理、SCP、公式 ROS CLI、Jetson 接続 |
| `processing/` | Web に依存しない HDMap 幾何処理・ライン生成 |
| `../../ros2_ws/src/atd_web_bridge/` | Jetson の ROS 観測・設定と loopback HTTP |
| `../../ros2_ws/src/atd_joy/` | evdev の入力取得・正規化を行う C++ component |

Docker の追加 apt 依存は [dependencies.apt](../../docker/dependencies.apt)、追加処理は [Dockerfile.atd](../../docker/Dockerfile.atd)。cuVSLAM・公式 mapping・visual mapping・VGL の apt package を使う。Node / pnpm と Web 用 Python 環境は Notebook 専用で、Jetson の通常起動では Web 依存を同期しない。

Docker の Web 用 Python は `docker/python/web/` の独立した uv project / lock と、workspace の `.atd-python/<architecture>/web` を使う。API は ROS の `PYTHONPATH` を引き継がず、公式 CLI を実行する子プロセスでだけ ROS 環境を読み込む。ローカル開発用には `backend/pyproject.toml` と `uv.lock` があり、`processing` をローカル依存として参照する。

モデルのダウンロードや対象 GPU の engine build は、apt インストール成功とは別の操作・検証である。公式手順: [cuVSLAM](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)、[地図作成](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/visual_global_localization/tutorials/tutorial_map_creation.html)。

## 検証の境界

ローカルの API・ファイル保存・geometry・worker・SSH 引数・入力 profile・キャッシュのテストと、frontend の型検査・build を実行する。Docker image build、NVIDIA GPU と実 bag の公式処理、実 SCP、ROS build / service、Bluetooth / evdev、車両の実走行は対象機で別に確認する。
