# Isaac ROS CLI・Docker のビルド方針

調査日: 2026-11-16

初回のホスト準備は [セットアップ手順](setup/README.md) と `scripts/setup.sh` を参照する。ここでのホスト準備と、Docker 内の追加依存・DDS の起動時処理は別の段階である。

`kart/docker` の比較から整理した今後のホスト準備・依存追加・TUI / UI 配置の提案は、[環境セットアップ案](environment_setup_plan.md)を参照する。現行の実装状況と、未実施の提案は区別する。

## 結論と採用範囲

ユーザーの指定に従い、Isaac ROS CLI は apt 版を採用し、Docker のカスタムは YAML と追加 Dockerfile で行う。
PC / Jetson を自動判定する起動スクリプト、保存先設定の例、Docker 構成を作成した。記録先・解析結果は毎回読書き可能で mount する。
RealSense はローカルの `realsense455` layer を使い、librealsense SDK のみ CUDA 無効で構築する。
共通依存と Fast DDS の loopback UDP 設定は、Docker の root 起動処理からスクリプトで自動適用する。
イベントカメラは今後使用せず、関連依存を将来の候補からも除外する。
CLI 本体は変更せず、以前の `kart` の独自 mount・起動処理も移植していない。CLI 本体の改修が必要になった場合に source build を検討する。
これらは初期実装であり、対象ホストへの CLI 導入、Docker build、実機検証は未実施である。

変更しない ROS パッケージ・システム依存は apt を基本とし、自作 `atd_*` とソース・コンパイル条件を変更するパッケージだけ source build する構成を提案する。
個別の使用パッケージは今後選定する。`realsense455` layer 内の SDK・driver の source build は、CLI 自体の source build とは区別する。

## source build を使う具体的な用途

| 対象 | source build が必要になる用途 | 現時点の ATD |
| --- | --- | --- |
| CLI 本体 | 設定・追加引数では対応できない mount 規則や起動処理、CLI 機能を変更する | 必要な改修は未特定。旧 fork と同じ変更を必要とはしていない |
| librealsense SDK | 配布済み構成と異なる SDK のビルド条件を適用する | `realsense455` layer で CUDA 無効にして構築する |
| 既製 ROS パッケージ | C++ 実装の修正、パラメータの追加、配布版にない修正や CMake 条件を適用する | RealSense driver は layer 内で公式相当の Debian パッケージを構築する。その他の変更対象は必要に応じて検討する |
| 自作 `atd_*` | ATD のノード・component を実装する | 自作コードは `colcon build` する |

追加 apt パッケージ、DDS XML、ROS パラメータ、Foxglove Bridge の接続設定、追加 Dockerfile のために CLI 本体を source build する予定はない。

## 区別する三つの対象

| 対象 | apt / 配布済み成果物を使う場合 | source / ローカル build を使う場合 |
| --- | --- | --- |
| Isaac ROS CLI | NVIDIA の `isaac-ros-cli` パッケージを導入 | CLI ソースを変更・固定し、`make build` で独自 `.deb` を作って導入 |
| Docker イメージ | 配布済みのベースイメージを利用 | ベースと追加 Dockerfile からイメージを構築。依存の導入には apt を利用できる |
| コンテナ内の ROS パッケージ | `ros-lyrical-*` を apt で導入 | 対象リポジトリを取得し、依存を解決して `colcon build` |

CLI を apt で入れても Docker をローカルビルドできる。CLI を source build しても ROS パッケージを apt で入れられる。
CLI は Python を中心としたツールの Debian パッケージ化であり、CUDA を含む ROS パッケージ群のコンパイルとは別の作業である。
所要時間・メモリ使用量は今回測定していない。

参照: [公式 CLI README](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/blob/release-5.0/README.md)、[既存 fork の Makefile](https://github.com/carbuncle01/isaac-ros-cli/blob/dff97a3af51ec40f8d0a92e65220391ec735a041/Makefile)、[公式 cuVSLAM 導入手順](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)

## CLI の apt 導入と source build の比較

| 観点 | NVIDIA の apt 版 CLI | ソースからビルドする CLI |
| --- | --- | --- |
| 導入の手軽さ | 公式標準手順。依存解決と更新が容易 | ソース取得、テスト、`.deb` 作成・導入が増える |
| 追加 apt パッケージ・D455 依存 | YAML と追加 Dockerfile で対応可能 | 同じ拡張機構を使える。CLI 改修は通常不要 |
| ROS パラメータ・launch・DDS XML | プロジェクト側で設定可能 | 同様。これらの変更だけなら CLI source 化は不要 |
| mount・起動コマンドの固定処理を変える | 提供された設定・追加引数の範囲で対応 | 起動処理そのものを変更し、テストできる |
| 更新時の作業 | 配布版への追従が容易 | 上流差分の確認と独自変更の再検証が必要 |
| 再現性 | apt バージョンと Docker digest 等を記録する | 上記に加え CLI commit と独自 `.deb` を記録する |

NVIDIA は CLI の apt 導入を標準手順とし、プロジェクト固有の依存を追加 Dockerfile に記録する方法を推奨している。
workspace の `.isaac-ros-cli/config.yaml` で `additional_image_keys` を追加し、`isaac-ros activate --build-local` で構築できる。
したがって「Docker を細かくカスタムするには常に CLI の source build が公式推奨」とは解釈しない。
CLI の起動処理を管理できることは source 運用の利点だが、現在の ATD ではその改修用途がないため、採用を必須とする理由にはしない。

参照: [Getting Started](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#initialize-isaac-ros-cli)、[Development Environment](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/dev_env/index.html#custom-docker-image-layers)

## 以前の `kart` ブランチで確認した事実

確認対象: [carbuncle01/isaac-ros-cli の kart ブランチ](https://github.com/carbuncle01/isaac-ros-cli/tree/kart)
調査時の HEAD は `dff97a3af51ec40f8d0a92e65220391ec735a041`、CLI のパッケージ版は `2.6.0-1+kart1`。
Isaac ROS 5.0 / ROS 2 Lyrical / JetPack 7.2 を対象とし、上流に対して一つのカスタム commit を持つ。

| 確認した変更・挙動 | ATD での扱い |
| --- | --- |
| `<project>/ros2_ws` の親を `/workspaces` に一度だけ mount し、`/workspaces/ros2_ws` を使用 | 移植しない。ATD は標準の `/workspaces/isaac_ros-dev` を使い、Git 操作用に checkout 全体を `/workspaces/atd` へ rw mount する |
| ホストの bash 設定や SSH / AWS ディレクトリの自動 mount を削減 | 必要な共有範囲を明示する設計を引き継ぐ候補。Jetson の SSH agent forwarding は残っている |
| `DOCKER_ARGS_FILE` のコメント行を読み飛ばす | 設定ファイルをプロジェクトで管理する際に再利用可能 |
| Dockerfile・画像 hash・build cache の処理はカスタム commit で変更していない | CLI source 化自体が毎回の全ソース再ビルドを要求するわけではない |
| `--network host`、`--privileged`、`--ipc=host` が起動処理に残る | host network は要件違反ではない。DDS の直接通信を Jetson 内部に制限し、bridge の外部向け通信と区別する。デバイス権限は別途具体化する |

参照: [変更 commit](https://github.com/carbuncle01/isaac-ros-cli/commit/dff97a3af51ec40f8d0a92e65220391ec735a041)、[起動処理](https://github.com/carbuncle01/isaac-ros-cli/blob/dff97a3af51ec40f8d0a92e65220391ec735a041/scripts/run_dev/run_dev.py#L216-L228)、[Docker 起動引数](https://github.com/carbuncle01/isaac-ros-cli/blob/dff97a3af51ec40f8d0a92e65220391ec735a041/scripts/run_dev/run_dev.py#L372-L415)

この fork は親ディレクトリの `packages.repos` を必須とする。調査時点の ATD には同ファイルも `ros2_ws/` もなく、そのまま再利用する構成にはなっていない。
また、この版では YAML schema に `workdir` / `entrypoint` があっても、CLI から起動スクリプトへの引数には渡されず、起動処理に固定値がある。
設定項目が存在することと、設定だけで mount・作業ディレクトリを置換できることを同一視しない。ただし、ATD がこの変更を必要とするかは別の判断である。

参照: [CLI から起動スクリプトへの引数](https://github.com/carbuncle01/isaac-ros-cli/blob/dff97a3af51ec40f8d0a92e65220391ec735a041/src/isaac_ros_cli/commands/activate/docker.py#L43-L98)

## 実装した開発環境

| ファイル | 役割 |
| --- | --- |
| [`../scripts/dev.sh`](../scripts/dev.sh) | ホストを自動判定し、保存先・CLI の設定と起動引数を選択する。ホストの手動上書きも可能 |
| [`../scripts/setup.sh`](../scripts/setup.sh) | workspace を対話的に指定し、ホスト別の設定と CLI テンプレートへの参照を準備する |
| [`../scripts/bringup.sh`](../scripts/bringup.sh) と [`../scripts/lib/bringup.sh`](../scripts/lib/bringup.sh) | 起動 profile の選択と一覧。現在は Jetson の `input` / `bridge` を既存 ROS launch へ渡す。Docker起動・ROS buildとは分離する |
| [`../scripts/tools/docker.sh`](../scripts/tools/docker.sh) | 項目別の診断結果をホストごとのログへ保存・再表示し、起動・build に進める |
| [`../scripts/lib/detect_host.sh`](../scripts/lib/detect_host.sh) | Jetson / Tegra の機種情報・BSP マーカーを参照する判定関数 |
| [`../config/storage.env.example`](../config/storage.env.example) | PC / Jetson ごとの記録先・解析結果保存先の設定例 |
| [`../config/workspace.env.example`](../config/workspace.env.example) | PC / Jetson ごとの ROS workspace の設定例。未設定時は実 clone 配下の `ros2_ws` |
| [`../docker/Dockerfile.realsense455`](../docker/Dockerfile.realsense455) | 標準 RealSense layer の置換。librealsense SDK のみ CUDA 無効で構築する |
| [`../docker/Dockerfile.atd`](../docker/Dockerfile.atd) | 共通依存・DDS の build 時準備、固定版 uv、root 起動 hook、専用シェル、Docker ENV を配置する |
| [`../docker/dependencies.apt`](../docker/dependencies.apt) | 起動時に不足分を確認する共通 apt 依存一覧。イベントカメラ関連は含めない |
| [`../docker/scripts/install-dependencies.sh`](../docker/scripts/install-dependencies.sh) | root / Lyrical と依存一覧を確認し、不足パッケージだけ apt 導入する |
| [`../docker/scripts/configure-dds.sh`](../docker/scripts/configure-dds.sh) | Fast DDS XML と環境ファイルを `/etc/atd` へ配置する |
| [`../docker/scripts/10-atd-runtime.sh`](../docker/scripts/10-atd-runtime.sh) | 依存・DDS を順に準備し、成功後に `runtime.ready` を記録する root hook |
| [`../docker/scripts/atd-shell.sh`](../docker/scripts/atd-shell.sh) | 専用 `atd-shell` の入口。ホストの旧 bashrc を読み込まず ATD 環境で対話シェルを開く |
| `../docker/scripts/atd-python.sh` と [`../scripts/tools/python.sh`](../scripts/tools/python.sh) | 起動済みコンテナで、ROS 2 用・AI 学習用の独立した uv project / venv を選択・同期・実行する |
| `../docker/python/{ros,train,vgl-export}/` | 用途別の `pyproject.toml` と `uv.lock`。VGL export は Notebook 専用、個別の学習 framework は未選定 |
| [`../scripts/vgl.sh`](../scripts/vgl.sh) と [`../tools/vgl/cli.py`](../tools/vgl/cli.py) | VGL の Notebook export、SCP 転送、Jetson engine build、資産・hash 検証。[操作手順](vgl_models.md)を参照 |
| [`../docker/middleware_profiles/fastdds.xml`](../docker/middleware_profiles/fastdds.xml) | UDPv4 のインターフェース・探索・unicast locator を loopback に限定するプロファイル |
| [`../docker/dev.dockerargs`](../docker/dev.dockerargs) | 記録先と解析結果を毎回読書き可で mount する |
| [`../ros2_ws/src/README.md`](../ros2_ws/src/README.md) | 自作 ROS package の編集元。別 workspace の場合は `src/atd` に mount する |
| [`../docker/isaac_ros/config.yaml`](../docker/isaac_ros/config.yaml) | 選択した workspace から参照するテンプレート。`realsense455`、`atd` の追加 image key を指定する |
| [`../docker/isaac_ros/common-config.sh`](../docker/isaac_ros/common-config.sh) | workspace から参照するテンプレート。ATD と CLI 標準の Dockerfile 検索先を指定する |
| [`../docker/isaac_ros/build_image_layers.yaml`](../docker/isaac_ros/build_image_layers.yaml) | `isaac_ros → realsense455 → atd` の構築順を指定する |

追加層の名前は CLI の `Dockerfile.<image_key>` 規則に合わせて `Dockerfile.atd` とした。image key は `atd` であり、`.additional_setting` 等の独自名は使わない。

`scripts/dev.sh` は apt 版の `isaac-ros activate` を利用する。`--build` を付けると `--build-local` を渡す。
ホストの workspace は `config/workspace.env` の保存済み指定と上書き指定を使い、未設定時は実際の clone 配下の `ros2_ws` とする。clone の標準配置 `~/workspaces/atd` なら `~/workspaces/atd/ros2_ws`。記録先・結果先は clone の `data/record`・`data/results` を未設定時の値とし、コンテナ内の workspace とデータのパスは維持する。CLI の探索規則に合わせ、選択先とその親の `scripts` から `docker/isaac_ros/` のテンプレートを参照する。ATD の旧配置への既知の CLI 参照を移行し、異なる既存設定や保存先は自動変更しない。具体的な配置と設定変更は [セットアップ手順](setup/README.md#ros-workspace-のパス)を参照する。
公式の `isaac_ros` をベースに、ローカルの `realsense455`、`atd` layer を構築し、コンテナ内での手動インストールだけに依存しない構成とした。
Foxglove Bridge のパッケージ導入と、bridge の起動・公開設定・動作確認は区別する。現時点では bridge は自動起動しない。

### RealSense layer の変更範囲

標準 `realsense` layer の後に別の SDK を追加する構成にはせず、追加 image key を `realsense455` へ置き換える。
`docker/isaac_ros/build_image_layers.yaml` で `image_key_order: [isaac_ros.realsense455.atd]` を指定する。
上流で順序未定義の custom key は名前順に並ぶため、YAML の追加 key の記載順だけに依存しない。ベースの build context と registry の既定設定は維持する。

`Dockerfile.realsense455` は、`isaac_ros` ベース内の `/etc/isaac-ros-cli/docker/` にある build helper 2 本と `realsense-ros-lyrical.patch` を `RUN cp` で `/opt/realsense` に再利用する。
ホスト側のファイルを `COPY` する方式や、CLI ソース一式の vendor 化は行わない。ホストの apt 版 CLI と、ベース内の apt 版 CLI のバージョンが同一とは仮定せず、実際のビルド時に補助ファイルの存在と挙動を確認する。

SDK `v2.56.3` / driver `r/4.56.3`、RSUSB、Lyrical パッチ、driver の Debian パッケージ構築は維持する。
標準 RealSense layer の CUDA Toolkit を追加 apt 導入する処理を省き、SDK の build helper に `--no_cuda` を固定で渡す。
SDK build 後には `CMakeCache.txt` の `BUILD_WITH_CUDA` が無効であることを検査する処理を追加した。実 build での検査通過は未確認である。
ベースに含まれる CUDA を除去する変更ではなく、他の Isaac ROS GPU 処理を無効化する構成でもない。

ユーザーが報告した過去の CUDA 有効構成での起動失敗について、原因を CUDA と断定しない。今回の変更による解消も未確認である。
D455 の IR・RGB・IMU・Depth の利用候補は維持し、実際の起動、CPU 負荷、FPS、フレーム欠落、他処理との同時実行性能は実機で検証する。

変更を反映するには、利用中の処理を終了し、同名の実行中コンテナを停止してから `./scripts/dev.sh --build` で再構築・起動する。
既存コンテナへの再接続だけでは新しい SDK 構成は反映されない。

### 起動時の依存・DDS 設定

Dockerfile は既知の共通依存を同じ `install-dependencies.sh` で事前導入し、Fast DDS の設定と root hook を配置する。
上流 CLI の `--rm` により終了後に新規コンテナを作成する場合でも、既知の依存を毎回インストールし直さないためである。
root 起動 hook `10-atd-runtime.sh` は、mount された `/workspaces/atd/docker` を優先する。イメージ内の起動スクリプトと固定版 uv `0.13.8` を確認し、次の処理を実行する。

1. `install-dependencies.sh` が依存一覧を検査し、`dpkg-query` で不足を調べる。不足時だけ `apt-get update` と `apt-get install` を実行する。
2. `configure-dds.sh` が `/etc/atd/fastdds.xml` と `/etc/atd/dds.env.sh` を配置する。
3. `gosu admin` 経由で ROS 2 用の追加 Python 依存を同期する。学習用環境は明示操作で準備する。
4. 環境を読み込み、すべての処理の成功後に `/etc/atd/runtime.ready` へ起動時の構成識別子を記録する。

依存導入や DDS 設定に失敗した場合は起動を失敗させる。毎回の apt upgrade は実行しない。
依存一覧のイベントカメラ関連名は拒否する。現行 ATD にはこれらの導入処理はなく、参照先 kart の関連 SDK・driver を移植しない。

上流 CLI で Dockerfile の `COPY` 元の変更がイメージ識別へ反映されない経路にも対応する。
`dev.sh` は Dockerfile、依存一覧、`docker/scripts/`、`docker/middleware_profiles/`、`docker/python/` の入力を hash し、`.cache/docker/<revision>/Dockerfile.atd` に revision コメントを加えた生成版を配置する。
このディレクトリを検索先の先頭に置き、`context_overrides.atd: ../../../docker` で元の `docker/` を build context として維持する。生成物は Git 管理対象外とする。
起動 hook は Dockerfile・hook・専用シェル・bashrc のプロジェクト側とイメージ内実体も比較し、不一致なら `--build` での再構築を要求する。

Fast DDS は `rmw_fastrtps_cpp` とし、独自 UDPv4 transport の interface、探索 peer、unicast locator を `127.0.0.1` に限定する。
既定 transport を無効にし、XML を使うため `ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT` とする。
Docker ENV は `docker exec` にも既定値を渡し、`BASH_ENV=/etc/atd/dds.env.sh` と専用シェルで ATD 環境を読み込む。
対話用の `atd-shell` はホストから共有される旧 `.bashrc` を読み込まない。追加ターミナルにも通常の `/bin/bash` ではなく `atd-shell` を使用する。
Ubuntu の対話 Bash が先に読む `/etc/bash.bashrc` は、ATD の専用 rc から重ねて source しない。sudo 案内などのシステム初期化の二重実行を防ぐ。[Ubuntu Bash の起動処理](https://git.launchpad.net/ubuntu/+source/bash/plain/shell.c?h=ubuntu/noble)と [SYS_BASHRC の設定](https://git.launchpad.net/ubuntu/+source/bash/plain/debian/patches/deb-bash-config.diff?h=ubuntu/noble)を参照する。

上流 CLI の host network は維持する。XML による制限の実装と、実機での DDS 通信範囲の検証完了は区別する。
Foxglove Bridge は導入のみで、公開 topic・待受・書込み機能の設定と接続検証は後工程で行う。
VSLAM 等の採用と topic ごとの QoS は今回の環境実装では固定しない。

OS・配布済み ROS 2 依存は apt を基本とし、C++ 実装やコンパイル条件を変えるパッケージは個別に source build する。追加の Python 依存は [用途別の uv project / venv](python_environments.md)で管理し、ROS 2 用と AI 学習用を分ける。

source build の具体例として、Isaac ROS 5 の `isaac_ros_cuvslam` は cuVSLAM 本体も submodule からビルドし、CMake で `USE_NVTX` などの条件を指定する。
この種の条件を変える場合は ROS パッケージ側の source build が必要であり、CLI source 化とは別の判断になる。cuVSLAM の採用自体は未決定である。

参照: [cuVSLAM の CMakeLists.txt](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-5.0/isaac_ros_cuvslam/CMakeLists.txt)、[submodule 定義](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-5.0/.gitmodules)

通信構成の参照: [Foxglove Bridge](https://docs.foxglove.dev/docs/fleet/bridge)、[Docker host network](https://docs.docker.com/engine/network/drivers/host/)

## workspace・mount・再接続

日常の導入・設定・起動手順は [セットアップ手順](setup/README.md)、ディレクトリの役割は [構成規約](project_structure.md)を参照する。ホストの workspace・保存先は setup の対話入力でホスト別に保持し、通常起動は `scripts/dev.sh` を使う。

| 内容 | コンテナ内のパス | アクセス |
| --- | --- | --- |
| 選択した ROS workspace | `/workspaces/isaac_ros-dev` | CLI 標準の rw mount |
| `<ATD checkout>/ros2_ws/src`（同じ `ros2_ws` を workspace に選ぶ場合） | `/workspaces/isaac_ros-dev/src` | workspace mount 内。追加 source mount は行わない |
| `<ATD checkout>/ros2_ws/src`（別 workspace を使う場合） | `/workspaces/isaac_ros-dev/src/atd` | 自作 package の編集用 rw mount |
| ホストの ATD checkout | `/workspaces/atd` | read-write、Git 操作と対話シェルの開始位置 |
| rosbag の記録・解析元 | `/data/record` | 毎回 rw |
| 地図・解析等の結果 | `/data/results` | 毎回 rw |

自作 package は `ros2_ws/src/atd_<module>/` で管理する。別の workspace を使う場合はその `src/atd` をコンテナの mount point として予約し、ホスト側の予約先に別データがある場合は停止して保護する。編集元と build 成果物を区別し、標準 build では workspace の `src` だけを探索する。
選択先がリポジトリの `ros2_ws` なら `src` をそのまま使い、source の追加 mount を省く。必要な bind mount はこの場合四つ、別 workspace の場合は五つとなる。

追加 bind mount は公式 CLI の `DOCKER_ARGS_FILE` で指定する。PC / Jetson の実際の SSD パスはホスト別設定から渡し、コンテナ内のパスを揃える。bind mount が参照するのは各 Docker ホストのファイルであり、PC と Jetson 間の転送は別処理とする。
記録・解析に応じた起動切替は行わず、両保存先を毎回 mount する。rosbag 記録・再生、車両 node は自動実行しない。

`scripts/dev.sh` は CLI の Docker モードを確認し、ホスト別の `atd_pc_dev` / `atd_jetson_dev` を使う。稼働中コンテナへの再接続では必要な bind mount、runtime API、Docker ENV、`ATD_RUNTIME_REV` と `runtime.ready` を確認する。不一致・未準備なら自動停止せずエラーとし、処理を終了してから再作成する。
再接続は `docker exec ... atd-shell` を使い、稼働中の apt 導入・DDS 設定更新は行わない。`--build` 指定時に同名コンテナが稼働中の場合も停止せずエラーにする。

確認した上流 loader は起動引数ファイルのコメント行を明示的に除外しないため、`docker/dev.dockerargs` には引数行だけを置く。
`DOCKER_ARGS_FILE` 以外の既存ユーザー・標準引数ファイルも加算されるので、同じ宛先の mount を重複させない。上流の実機 device mount 等も含めた実効構成は対象機で確認する。

参照: [公式 CLI の追加起動引数 loader](https://github.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/blob/release-5.0/scripts/run_dev/run_dev.py)、[Docker bind mount](https://docs.docker.com/engine/storage/bind-mounts/)

## 検証状況と次の確認

- 作成済み: CLI の workspace 設定、`realsense455` / `atd` の Dockerfile、構築順の設定、ホスト保存先の設定例、共通の起動引数、起動スクリプト。
- 追加実装済み: 共通依存一覧、不足分だけ導入するスクリプト、Fast DDS の loopback XML と配置スクリプト、root 起動 hook、`atd-shell`、構成識別子による再接続の整合確認。実 Docker 内での動作確認とは区別する。
- 調査・確認済み: 公式文書、CLI 公開ソース、`kart` の差分、従来の mount 引数の mock 検証。
- 構成ファイルの静的検証済み: shell 構文の `bash -n`、YAML の読込み、上流の build 設定 loader による Dockerfile 探索順の確認。
- RealSense 層の模擬検証済み: 上流 CLI の設定 loader・image key resolver・bake 設定生成を使い、amd64 / arm64-jetpack とも `isaac_ros → realsense455 → atd` の順序、build context、`BASE_IMAGE` の依存関係を確認した。表示と YAML 読込みの依存を代替しており、Docker build は実行していない。
- CUDA 無効指定の模擬検証済み: 新 Dockerfile の引数で公式 helper のオプション処理を実行し、CMake 呼出しに `BUILD_WITH_CUDA=false`、`FORCE_RSUSB_BACKEND=true` が渡ることを確認した。7 個の `RUN` の Bash 構文、helper の Debian 配置定義も確認した。実 CMake・コンパイル・apt 導入・D455 I/O は実行していない。
- 起動・依存・DDS の検証済み: 自動テストで CLI・Docker・apt を偽物に置き換え、初期化の順序、失敗時停止、不足分導入、再接続・revision / ready の検査、生成 Dockerfile、ホスト rc の回避、ROS 環境と DDS の読込みを確認した。テストは [tests/](../tests/) を参照する。
- XML の検証済み: `ATD_FASTDDS_XSD` に Fast DDS v3.6.2 の公式 `fastdds_profiles.xsd` を指定し、公式 XSD による受理を確認した。未指定の場合は XSD 検証を skip する。XML の受理確認と、実 ROS の探索・データ通信・外部パケット確認は区別する。
- 生成 Dockerfile の模擬検証済み: 上流の `Config.load_shell_common_config` / `load_yaml`、`resolve_dockerfiles`、`generate_bake_dict` を使用し、生成版の優先探索、元 `docker/` への context 解決、DDS helper 更新による ATD 層・最終イメージ hash の変化、Isaac ROS / RealSense 層の選択・hash の維持を確認した。表示と YAML parser、Linux の CLI 探索先をローカルで代替した検証であり、実 Docker build ではない。
- 現在の Mac ホストでは、保存先を一時ディレクトリに指定した `--dry-run` で `host: pc (auto)` と設定選択を確認した。Ubuntu PC / Jetson の実起動検証とは区別する。
- 未実施: 対象ホストへの CLI 導入、実際の Docker build、ROS build / bag 再生、D455・Joy・車両 I/O、bridge 接続、実走行。
- 現在の Mac では Docker デーモンへ接続できず、今回の Docker build・実コンテナ検証は実行できなかった。
- 初回 setup の検証: 偽コマンドと一時プロジェクトによる自動テストを用意している。Notebook / Jetson の区別、workspace のパス・設定、保存先、fzf の取消、失敗時停止、既存設定の維持と権限不足時の読取 fallback を対象にする。Ubuntu 実ホストの apt 導入・サービス変更は実行していない。
- 構築時に確認: 対応版の組合せ、CLI のシステム全体への導入と他プロジェクトへの影響、イメージの再作成、実際の起動引数・mount・デバイス、DDS の直接通信が外部に出ないことと bridge 経由の観測ができること。
- Orin Nano 8GB での source build の時間・RAM と、全処理同時実行時の性能は測定する。CLI のパッケージ化と画像処理パッケージ群のビルドコストを分けて記録する。

D455 の対応版・既知問題は [要件文書 3.2](project_requirements.md#32-d455-と地図生成) を参照する。導入方式を source build に変更しただけで既知問題が解消したとは扱わない。
公式 5.0 の custom Dockerfile 説明には `jazzy` の例が残っているため、サンプルを使う際は ATD の `lyrical` 基準と実際のパッケージ名を確認する。
