# ATD セットアップ

Ubuntu ホストに必要な準備を行い、ATD の Docker 開発環境へ入るための手順。
通常はプロジェクト直下で次を実行し、対話メニューから進める。

```bash
./scripts/setup.sh
```

Notebook / Jetson は自動判定する。必要な場合だけ `--host notebook`（`--host pc` と同義）または `--host jetson` で上書きする。保存先設定等の内部名は従来どおり `pc` / `jetson` を使う。
通常のログインユーザーで実行する。`sudo ./scripts/setup.sh` とはせず、ホスト変更に必要な処理だけスクリプト内で `sudo` を使用する。

## 対象

| ホスト | アーキテクチャ・判定 | 前提・固有の手順 |
| --- | --- | --- |
| Notebook / ノート PC | `x86_64`（apt / Docker では `amd64`） | Ubuntu 24.04、RTX 4060。[Notebook の準備と確認](pc.md) |
| Jetson Orin Nano 8GB | `aarch64`（`arm64`）と Jetson / Tegra・BSP マーカー | JetPack 7.2 を導入し初回ログイン済み。[Jetson の準備と確認](jetson.md) |

両方とも Isaac ROS 5 / ROS 2 Lyrical の Docker を使う。
ホストには GPU driver / JetPack、Docker、NVIDIA Container Toolkit、apt 版 Isaac ROS CLI、入力・USB の準備を置く。
ROS、RealSense SDK / driver、ATD の共通依存、DDS 設定はコンテナ側で管理する。
イベントカメラ関連の依存は導入しない。
Notebook と Jetson は driver / BSP と CLI の apt suite が異なるため、それぞれの手順を使う。`aarch64` の一般 PC を Jetson として扱わない。

## 初回の進め方

1. メニューで現状を診断し、不足するホスト準備を順に実行する。
2. 記録先と解析結果の保存先を設定する。SSD を使う場合は、先にホストで mount 済みのパスを選ぶ。
3. workspace で ROS workspace のパスを入力して保存し、CLI 設定とホスト / コンテナの対応を確認する。
4. Docker と NVIDIA runtime、CLI の状態を確認する。
5. 初回イメージを build してコンテナへ入る。

直接指定する場合も、同じ入口を使える。

```bash
# 変更せずに診断する
./scripts/setup.sh --action check
# 項目ごとの OK / FAIL と、後から読める診断ログ
./scripts/docker.sh
# 初回セットアップで実行する操作を確認する
./scripts/setup.sh --action all --dry-run
# 初回イメージの build と起動
./scripts/setup.sh --action build
```

`all` は tools → docker → runtime → isaac → devices → storage → workspace → verify → build の順に実行する。
docker グループへの追加は、実行中の setup や親シェルには即時反映されない。途中で権限エラーになった場合は再ログインまたは SSH 再接続を行い、メニューから失敗した段階以降を再実行する。
Docker サービスの反映が必要な場合も、表示された案内に従ってから再実行する。
OS のフラッシュ、GPU driver の自動置換、ディスクの初期化、カメラ firmware の自動更新は行わない。

## 対話メニューと直接実行

選択には `fzf` を使用する。未導入時は番号メニューを表示し、tools で導入した後は同じ入口で `fzf` を使う。

| action | 用途 |
| --- | --- |
| `check` | OS、機種、GPU、Docker、CLI 等の現状確認 |
| `tools` | ホスト用の基本ツール・fzf 等の準備 |
| `docker` | Docker の準備。既存導入の状態を確認して扱う |
| `runtime` | NVIDIA Container Toolkit と Docker runtime の準備 |
| `isaac` | NVIDIA の apt repository と apt 版 Isaac ROS CLI の準備 |
| `storage` | 記録・解析結果の保存先を設定・作成 |
| `devices` | D455 のホスト側 udev、Jetson の Bluetooth / 入力準備 |
| `workspace` | workspace のパスを対話入力してホスト別に保存し、`src` と CLI 設定を準備 |
| `verify` | Docker、CLI の Docker モード、NVIDIA runtime の起動確認 |
| `build` | `dev.sh --build` で初回イメージを build・起動 |
| `start` | `dev.sh` で通常起動・稼働中コンテナへ再接続 |
| `all` | ホスト準備、保存先・workspace 設定、verify、初回 build を順に実行 |

`--dry-run` は予定する操作の表示だけを行い、`sudo`、通信、ファイル作成を実行しない。macOS でも操作の確認に使えるが、実際のホスト導入は Ubuntu 24.04 で行う。
`verify` や `build` の実行はコンテナやイメージ取得を伴うため、読取りだけの `check` とは区別する。

## ROS workspace のパス

clone の標準配置は `~/workspaces/atd`、未設定時の ROS workspace は実際の clone 配下の `ros2_ws` とする。標準配置なら `~/workspaces/atd/ros2_ws` が workspace で、その下に `src`・`build`・`install`・`log` を持つ。
Notebook と Jetson の設定は各ホストで保存する。保存済みのパス、引数・環境変数の指定はこの既定値より優先される。別の workspace も指定できるが、checkout と workspace のどちらも CLI の制約により空白や shell 特殊文字を含まないパスを使う。

| 用途 | ホスト | コンテナ内 | アクセス |
| --- | --- | --- | --- |
| ROS workspace | 指定したパス。未設定なら `<ATD checkout>/ros2_ws` | `/workspaces/isaac_ros-dev` | 読書き可 |
| 自作 ROS package の編集元（リポジトリの `ros2_ws` を workspace に選ぶ場合） | `<ATD checkout>/ros2_ws/src` | `/workspaces/isaac_ros-dev/src` | workspace mount 内 |
| 自作 ROS package の編集元（別 workspace を使う場合） | `<ATD checkout>/ros2_ws/src` | `/workspaces/isaac_ros-dev/src/atd` | 読書き可 |
| プロジェクト設定の参照元 | `<ATD checkout>` | `/workspaces/atd` | readonly |

```bash
./scripts/setup.sh --action workspace
# clone 配下を明示する場合（プロジェクト直下から実行）
./scripts/setup.sh --host notebook --action workspace --workspace "$PWD/ros2_ws"
```

対話入力は空欄で表示した既存値・既定値を使い、`~/` も指定できる。設定は [設定例](../../config/workspace.env.example)を基に `config/workspace.env` へ保存する。以前の `~/workspaces/atd` 自体を workspace として保存していた場合は、この操作で `~/workspaces/atd/ros2_ws` を明示的に選び直す。保存済み設定は自動変更しない。

| ホスト | 保存する設定 |
| --- | --- |
| Notebook（`pc` / `notebook`） | `ATD_PC_ROS_WS` |
| Jetson | `ATD_JETSON_ROS_WS` |

選択したホストの設定だけを更新し、もう一方は保持する。別の設定ファイルを使う場合は `ATD_WORKSPACE_CONFIG=/absolute/path/workspace.env` を setup / dev / docker の各入口へ渡す。起動時だけ上書きする場合は `ATD_ROS_WS` を使える。
workspace 操作では CLI 設定を準備する。別の workspace を選択した場合は、その `src/atd` をコンテナ内でリポジトリの `ros2_ws/src/` を mount する予約先として用意し、ホストでは空の mount point とする。既存ファイル・別の symlink 等がある場合は内容を保護して停止する。編集はホストの `<ATD checkout>/ros2_ws/src`、またはコンテナの `/workspaces/isaac_ros-dev/src/atd` で行う。
リポジトリの `ros2_ws` 自体を workspace に選ぶ場合は、その `src` がコンテナの `/workspaces/isaac_ros-dev/src` にそのまま対応する。追加の `src/atd` mount point は作らない。直接指定する場合は、プロジェクト直下で `./scripts/setup.sh --action workspace --workspace "$PWD/ros2_ws"` を使える。既定パスとホスト別設定の保存方法は変わらない。
CLI の探索規則に合わせ、`<workspace>/.isaac-ros-cli/config.yaml`、`<workspace>/scripts/.isaac_ros_common-config`、`<workspace>/../scripts/.build_image_layers.yaml` から `docker/isaac_ros/` のテンプレートを参照する。親の `scripts` は兄弟 workspace と共通になることがある。ATD の旧配置への既知の参照は新配置へ移行し、異なる既存設定は上書きせず停止する。
`ISAAC_ROS_WS` は `dev.sh` がホストでは選択先へ設定し、コンテナ内では `/workspaces/isaac_ros-dev` を使う。ホストの `.bashrc` へ手動で export する必要はない。
ROS の build は workspace 内で行い、コンテナの `/workspaces/atd` は設定参照用として扱う。標準の入口は `./scripts/build.sh`。直接 `colcon build` を使う場合は workspace の `src` を対象に `--base-paths src` を明示する。
Notebook の `build/` / `install/` を Jetson へコピーせず、各ホスト用に build する。
CLI 設定テンプレートは `docker/isaac_ros/`、Docker の生成物は `<ATD checkout>/.cache/docker/` に置く。テンプレート・ソースと、各ホストの workspace の build 成果物を分ける。詳しくは [構成規約](../project_structure.md)を参照する。

## 状態の診断と履歴

```bash
./scripts/docker.sh
./scripts/docker.sh --host jetson --action check
./scripts/docker.sh --action logs
# 選択せず、最新の結果だけを表示
./scripts/docker.sh --latest
```

`docker.sh` の既定操作は check。項目ごとに `OK` / `FAIL` / `SKIP` を表示し、最後に件数をまとめる。workspace、保存先、OS・アーキテクチャ、Docker 接続、NVIDIA runtime、CLI の Docker モード、DDS 設定等を確認する。
実行中のコンテナは、現在の起動スクリプト・依存の revision と初期化完了マーカー、workspace・データの mount、DDS 環境設定の一致も確認する。
コンテナがまだない場合など、起動して初めて確認できる項目は `SKIP` として区別する。`FAIL` があれば終了コードは失敗になるが、後続の独立した項目も確認して結果を残す。

診断は Docker 起動・イメージ取得・インストールを行わない。結果を `logs/docker/pc/` または `logs/docker/jetson/` に日時付きで保存する。ログは Git 管理対象外で、権限は本人だけが読書きできる `0600` とする。保存先ベースは `ATD_LOG_DIR` で変更できる。
logs 操作は対話端末で `fzf` により過去ログを選び、非対話実行では最新ログを表示する。`--dry-run` は実行予定だけを表示し、ログも作成しない。

```text
[OK]   ROS workspace ...
[OK]   Docker CLI ...
[FAIL] Docker daemon ...
[SKIP] Container ...
```

表示例であり、実機の成功結果ではない。起動する場合は `./scripts/docker.sh --action start`、build する場合は `--action build` を使える。事前診断が通れば `dev.sh` へ渡し、その終了コードも記録する。CLI の終了コードだけをコンテナ起動成功の証拠にはせず、起動後は改めて check で初期化状態を確認する。

## 保存先

記録先は `<ATD checkout>/data/record`、地図・モデル等の結果先は `<ATD checkout>/data/results` を未設定時の既定とする。clone が `~/workspaces/atd` なら、その下の `data/` を使う。`data/` は Git 管理対象外の永続資産であり、cache ではない。
設定は storage の対話操作で、選択したホストの値だけを `config/storage.env` に保存する。[設定例](../../config/storage.env.example)を手動コピーして使う場合は、使用するホストの行へ実パスを記入してコメントを解除する。他ホストの値を推測して入力する必要はない。
storage は対話端末で記録先と結果先を順に尋ね、空欄なら表示した既存値・既定値を使う。引数で絶対パスを直接指定することもできる。

```bash
./scripts/setup.sh --action storage \
  --record-dir "$PWD/data/record" \
  --result-dir "$PWD/data/results"
```

clone が `/home/atd/workspaces/atd` の場合、対話入力の記録先には `/home/atd/workspaces/atd/data/record`、続く結果先には `/home/atd/workspaces/atd/data/results` を指定する。
保存済みの設定や引数・`ATD_RECORD_DIR` / `ATD_RESULT_DIR` の指定を優先するため、以前の保存先から切り替える場合は storage 操作で明示的に選び直す。既存データは自動移動しない。

設定ファイルを分ける場合は `ATD_STORAGE_CONFIG=/absolute/path/storage.env` を指定し、その後の setup / dev 起動でも同じ環境変数を使う。
コンテナ内は PC / Jetson とも次のパスを使う。

| データ | コンテナ内 | アクセス |
| --- | --- | --- |
| bag の記録・解析元 | `/data/record/rosbags/<bag_id>/` | 記録先 root を毎回読書き可で mount |
| NVIDIA 公式の地図出力 | `/data/results/maps/<map_id>/nvidia/` | 結果先 root を毎回読書き可で mount |
| HDMap・ライン | `/data/results/maps/<map_id>/{hdmap,lines}/` | 同上 |
| VGL・将来の E2E モデル | `/data/results/models/` | 同上 |
| 処理・学習ごとのログ・checkpoint 等 | `/data/results/runs/<run_id>/` | 同上 |

設定するのは記録先と結果先の二つだけで、モデル専用の環境変数や mount は増やさない。storage 操作は両 root を準備し、用途別の下位ディレクトリは初利用時に作成する。既存の bag・地図・モデルを自動で移動しない。
公式の出力構成、VGL の入力モデルと対象環境別 engine、将来の E2E 資産の置き場所は [保存構成](../project_structure.md#データの保存構成)を参照する。
VGL は `./scripts/vgl.sh` の fzf メニューから、Notebook の ONNX export、bundle の SCP 転送、Jetson の TensorRT engine build を実行する。既存 ONNX の package 操作も選べる。最初に両ホストの Docker イメージを `dev.sh --build` で更新し、コンテナを起動しておく。SSH 設定、処理の分担、検証・ログ確認は [VGL モデルの手順](../vgl_models.md)を参照する。対象機の GPU / SDK と実 SCP の通し動作は未検証である。

記録 / 解析の起動モード切替はない。データ整理のため保存先を分けることは推奨するが、同一・親子ディレクトリを禁止しない。
PC と Jetson のディレクトリは別々の実体であり、bag や地図の転送は別の処理になる。
設定変更後はコンテナを再作成して mount を反映する。実行中の処理は自動停止しない。

## 通常の起動と更新

```bash
./scripts/setup.sh --action start
```

短い入口として、従来の `./scripts/dev.sh` も使える。
Dockerfile・起動用スクリプト等を更新した場合は、利用中の処理を終了してからメニューの build を実行する。

Docker 起動時には root の `.sh` が追加依存の不足分を確認し、Fast DDS の UDPv4 / loopback 設定を適用する。
依存が揃っていれば apt 操作を省略する。再接続時は初期化を繰り返さず、設定の整合を確認して `atd-shell` を開く。
詳細は [CLI・Docker のビルド方針](../isaac_ros_build_strategy.md)を参照する。

## Docker 内の用途別 Python 環境

追加 Python 依存の管理は uv `0.13.8` を基盤とし、ROS 2 用の `ros` と AI 学習用の `train` を分ける。初回は変更を反映した Docker イメージを build し、`dev.sh` でコンテナを起動しておく。ホスト側に uv や同じ venv を導入する必要はない。

```bash
# 起動済みコンテナの用途別操作を fzf で選択する
./scripts/python.sh
# ROS 2 用環境を再同期し、環境の状態を確認する
./scripts/python.sh ros sync
./scripts/python.sh ros check
# 学習用環境で Python を実行する
./scripts/python.sh train run -- python -c 'import sys; print(sys.executable)'
# ROS package を build する
./scripts/build.sh
```

Notebook / Jetson の選択には通常の自動判定と `--host notebook|pc|jetson` を使う。`python.sh` は起動済みコンテナへ接続する入口であり、コンテナを起動していない場合は先に `dev.sh` を実行する。
コンテナ内の環境操作には `atd-python ros ...` / `atd-python train ...` を使用し、操作は sync / check / shell / run とする。ROS package のビルドは専用の `atd-build` で実行する。通常の対話シェル全体を自動で venv へ切り替えず、選んだ用途のプロセスへ環境を適用する。
`atd-build` と共通の環境準備処理を追加した構成へ更新する場合は、利用中の処理を終了して `./scripts/dev.sh --build` で image を更新・コンテナを再作成する。旧 `atd-python ros build` は使用しない。

用途ごとの `pyproject.toml`・`uv.lock` は `docker/python/ros/` と `docker/python/train/` に置く。venv は選択した workspace の `.atd-python/<architecture>/{ros,train}`、cache は `.atd-python/cache` へ作成する。workspace の bind mount に保存するためコンテナの再作成後も使えるが、別ホスト・別アーキテクチャへコピーしない。
`sync` は lockfile の更新を許さない同期を行い、`run`・`shell` と `build.sh` は実行前に同期する。依存の更新は別の開発作業として manifest と lockfile に記録する。

Docker 起動時は固定版 uv の利用可能性を確認し、root hook から `gosu admin` 経由で ROS 2 用環境を自動同期する。venv・cache は通常ユーザー所有にする。学習用環境は自動同期せず、上記の `sync`・`run`・`shell` の明示操作で準備する。
`ros` / `train` の追加 Python 依存は空で、学習 framework は未選定。Notebook の VGL export は独立した `vgl-export` profile の固定依存を操作時に同期する。Jetson の標準起動には追加せず、実 GPU / SDK での動作検証は別途行う。
ROS 2 の apt 依存との関係、Python node の interpreter と CMake の注意点は [Python 環境](../python_environments.md)を参照する。

## つまずいた場合

| 状態 | 対応 |
| --- | --- |
| Docker socket が permission denied | docker グループ追加は実行中セッションに反映されない。再ログインまたは SSH 再接続後、check / verify を再実行する |
| Docker デーモンへ接続できない | ホストの Docker サービスを確認する。既存コンテナを動かしたまま不要な再インストールをしない |
| NVIDIA runtime の設定反映が必要 | 実行中コンテナを終了してから案内された設定・サービス反映を行う |
| apt がロックされている | 実行中の apt / 自動更新の完了を待つ。ロックファイルの削除やプロセス強制終了で回避しない |
| 保存先がない・書込みできない | mount 状態と保存先設定を確認し、storage を実行する |
| ROS workspace の場所が分からない | workspace で保存済みパスを確認する。指定したホストの workspace とコンテナの `/workspaces/isaac_ros-dev` は同じソースを指す |
| workspace の CLI 設定が既存設定と衝突する | 表示されたファイルを確認する。setup は別用途の設定を自動上書きしない |
| 以前の診断結果を確認したい | `./scripts/docker.sh --action logs` でホスト別のログを選ぶ |
| 起動中コンテナの設定が古い | 作業を終了してコンテナを再作成する。必要なら build を実行する |
| Python 用途別環境を使えない | `dev.sh` でコンテナを起動し、`python.sh <ros|train> sync` と `check` を実行する。uv 未導入の旧 image は build して再作成する |
| D455 / DualSense が見えない | まずホスト認識、その後コンテナの device と非 root 権限を確認する。詳細は Jetson 手順を参照 |

## 検証の区別

- セットアップ入口とホスト準備は、一時ディレクトリ・偽の sudo / apt / Docker / CLI を使う自動テストで検証する。これらのテストは対象ホストへの実導入とは区別する。
- `check` は導入状態の診断であり、GPU 演算・センサ取得・実走行の成功を保証しない。
- `verify` は `docker info`、CLI の JSON 状態による Docker モード確認、`docker run --rm --gpus all ubuntu:24.04 /bin/true` を実行する。GPU runtime 指定での起動確認であり、CUDA 演算は別に検証する。
- D455 の IR・RGB・IMU・Depth、DualSense 入力、DDS の外部インターフェースへの流出、bridge の接続は対象機で検証する。
- Chrome で開くブラウザ UI、地図編集、自己位置の表示は今後実装する機能であり、このセットアップで UI を起動できる状態とは扱わない。
- この作成作業ではホスト環境を変更していない。Ubuntu PC / Jetson への通し導入・実 Docker build・実機検証は未実施である。

## 参照

- [Isaac ROS 5 Getting Started](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html)
- [Docker Engine の Ubuntu 手順](https://docs.docker.com/engine/install/ubuntu/)
- [NVIDIA Container Toolkit の導入](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [Isaac ROS RealSense Setup](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/sensors/realsense_setup.html)
- [参考にした Kart セットアップ](https://github.com/Arata-Stu/kart/tree/e5224221733dd4271d7132d8febcb80e91916063/docs/setup)

Kart からはホスト別の手順と確認順序を参考にした。ATD は apt 版 CLI、標準 workspace、Fast DDS を使うため、Kart 独自 CLI の構築やパッケージ一覧をそのまま実行する手順にはしない。
