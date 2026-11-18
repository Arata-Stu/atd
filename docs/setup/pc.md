# Notebook（x86_64）のセットアップ

Ubuntu 24.04 / `x86_64` と RTX 4060 を対象とする。apt / Docker でのアーキテクチャ名は `amd64`。
共通操作は [セットアップ入口](README.md)を使い、Jetson は [Jetson 手順](jetson.md)を参照する。
通常は自動判定し、必要なら `./scripts/setup.sh --host notebook`（`--host pc` と同義）で Notebook の設定を明示する。

## 1. ホストの GPU と保存先

通常のログインユーザーでプロジェクト直下から診断する。

```bash
./scripts/setup.sh --action check
```

確認するのは Ubuntu の版、GPU / NVIDIA driver、空き容量、Docker と CLI の状態である。
Isaac ROS 5 の公式 x86 対応表は Ubuntu 24.04、Ampere 以降、GPU メモリ 8GB 以上、Driver 595 以上、CUDA 13.2 以上を掲載している。
RTX 4060 の実際の VRAM と導入済み driver を確認し、driver の変更が必要なら [Ubuntu 公式手順](https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/)で対象 GPU に対応する版を選ぶ。
セットアップスクリプトは GPU driver を自動置換しない。[Isaac ROS 対応環境](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#supported-platforms)

`nvidia-smi` の CUDA 表示は driver が対応する上限を示し、ホストに CUDA Toolkit が導入済みであることとは別である。
Docker 方式では ROS・CUDA 等のアプリ依存をコンテナに置き、ホストへ ROS / librealsense を重複導入しない。

## 2. ホスト準備

```bash
./scripts/setup.sh
```

メニューから tools、Docker、NVIDIA runtime、Isaac ROS CLI、保存先、workspace の準備を進める。
既存の Docker や runtime が使える場合はそれを維持する。
CLI は NVIDIA の `release-5.0` / `noble` apt repository から導入する。独自 CLI のソースビルドは不要。[公式 apt 導入手順](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#configure-isaac-ros-apt-repository)

docker グループ追加は実行中セッションに反映されない。案内が表示された場合は再ログインして、必要な段階から再実行する。
未設定時の保存先は実際の clone 配下の `data/record` と `data/results`。標準 clone 位置 `~/workspaces/atd` なら `~/workspaces/atd/data/{record,results}` を使い、データは Git 管理対象外の永続資産として保持する。
storage の対話入力または `--record-dir` / `--result-dir` でホスト側の実際の絶対パスを設定し、Docker 内では共通の `/data/record` / `/data/results` を使う。

## 3. ROS workspace と初回 build

Notebook 上で ROS workspace の場所を指定し、コンテナ内では標準 workspace 名へ対応させる。未設定時は実際の clone 配下の `ros2_ws`、標準配置なら `~/workspaces/atd/ros2_ws` を使う。保存済み指定は維持するため、旧配置から変える場合は workspace / storage を選び直す。

| 用途 | Notebook ホスト | コンテナ内 |
| --- | --- | --- |
| ROS workspace | 指定したパス。未設定なら `<ATD checkout>/ros2_ws` | `/workspaces/isaac_ros-dev`（読書き可） |
| 自作 ROS package の編集元（別 workspace を使う場合） | `<ATD checkout>/ros2_ws/src` | `/workspaces/isaac_ros-dev/src/atd`（読書き可） |
| Git checkout・対話シェルの開始位置 | `<ATD checkout>` | `/workspaces/atd`（読書き可） |

```bash
./scripts/setup.sh --action workspace
./scripts/setup.sh --action verify
./scripts/setup.sh --action build
```

workspace は対話入力した場所に CLI 設定を準備し、`config/workspace.env` の `ATD_PC_ROS_WS` に保存する。別の workspace を使う場合は `src/atd` の mount point も用意する。直接指定は `--workspace /absolute/path/atd_ws` を追加する。自作 package の編集元はリポジトリの `ros2_ws/src/` とする。
リポジトリの `ros2_ws` を workspace に選ぶ場合は、その `src` がコンテナの `/workspaces/isaac_ros-dev/src` に対応し、追加 source mount は行わない。
`ISAAC_ROS_WS` は `dev.sh` が設定するため、ホストの `.bashrc` へ export を追記しない。設定の詳細と既存 CLI 設定との衝突時の扱いは [共通手順](README.md#ros-workspace-のパス)を参照する。
`./scripts/tools/docker.sh` で項目別診断を保存し、`./scripts/tools/docker.sh --action logs` で Notebook の過去の結果を確認できる。
初回 build は `isaac_ros → realsense455 → atd` の順で行う。librealsense SDK だけ CUDA 無効とし、Isaac ROS の GPU 処理は維持する。
Notebook の `./scripts/dev.sh --build` は、ホストの Isaac ROS CLI と Buildx の一時ファイルを既定で `$HOME/.cache/atd/build-tmp` に置く。初回に所有者専用（`0700`）で作成し、呼び出し元で `TMPDIR` を指定した場合はその値を優先する。`./scripts/dev.sh --build --dry-run` で使用予定のパスを確認できる。`setup.sh --action build` と `scripts/tools/docker.sh --build` も同じ設定を使う。通常起動や Jetson では、呼び出し元の一時ディレクトリ設定を使う。
verify は一般ユーザーから Docker に接続し、CLI の Docker モードと `--gpus all` による `/bin/true` のコンテナ起動を確認する。CUDA 演算成功は別に検証する。

コンテナに入った後、必要な確認を個別に行える。

```bash
printenv ROS_DISTRO ISAAC_ROS_WS RMW_IMPLEMENTATION
ros2 pkg prefix realsense2_camera
ros2 pkg prefix rosbag2_storage_mcap
```

期待する ROS / workspace / RMW は `lyrical` / `/workspaces/isaac_ros-dev` / `rmw_fastrtps_cpp`。
ROS package が見つかることは、実 D455 の動作や bag 取得の成功を意味しない。

### build 中に `no space left on device` が出た場合

`docker buildx bake final_target` は最終イメージ用の Dockerfile をホストの一時ディレクトリに書く。エラーが出たパスのファイルシステムで、空き容量（`Avail`）と空き inode（`IFree`）を確認する。次のコマンドで、Notebook の build 用一時ディレクトリ、`/tmp`、ローカルの Docker データの保存先を比較できる。[Buildx の実装](https://github.com/docker/buildx/blob/master/build/opt.go#L1171-L1175)

```bash
build_tmp="${TMPDIR:-$HOME/.cache/atd/build-tmp}"
docker_root="$(docker info --format '{{.DockerRootDir}}')"
df -hT "$build_tmp" /tmp "$docker_root"
df -i  "$build_tmp" /tmp "$docker_root"
```

ホームが満杯の場合は、空きのある別のファイルシステムに書込み可能なディレクトリを用意し、その絶対パスを `TMPDIR` に指定して build を再実行する。上流 CLI が一時ファイルのパスをシェルに渡すため、空白やシェルの特殊文字を含まないパスを使う。イメージと再ビルド用キャッシュは Docker が管理する。`docker system df` と `docker buildx du --builder default` で使用量を確認できる。[Docker のキャッシュ使用量](https://docs.docker.com/reference/cli/docker/buildx/du/)

### ローカル Docker を空にして作り直す

Notebook ホストの ATD checkout で実行する。削除の範囲はその Docker Engine 上の**全プロジェクト**。起動中のコンテナも停止するので、記録やビルドを終了し、残したいコンテナ内のファイルやボリュームのデータを先に退避する。

```bash
./scripts/tools/clean-docker-cache.sh --reset-all --dry-run
./scripts/tools/clean-docker-cache.sh --reset-all
```

2本目は対象を表示して入力を待つ。`RESET` と入力して Enter を押すと、コンテナ、Ubuntu・Isaac ROS を含む全イメージ、追加ネットワーク、ビルドキャッシュ、通常の名前付き・匿名ローカルボリュームを削除する。ボリュームに保存したデータベースなども消える。Docker は終了時の残量を表示し、使用中のデータや外部ドライバのボリュームなどが残った場合はスクリプトが報告する。[Docker の system prune](https://docs.docker.com/reference/cli/docker/system/prune/)・[名前付きボリュームの削除](https://docs.docker.com/reference/cli/docker/volume/prune/)

Docker Engine と CLI、認証・context 設定、標準ネットワークは次のビルドにも使える。ATD の checkout、workspace、bag・地図・モデルの保存先はホストからの bind mount なので保持される。同じ checkout で `./scripts/dev.sh --build` を実行すると、ベースイメージの取得から開発環境を作り直せる。

### ビルドキャッシュと過去の ATD イメージだけを整理する

Notebook ホストの ATD checkout で、まず使用量と削除候補を確認する。

```bash
./scripts/tools/clean-docker-cache.sh --dry-run
```

`docker system df` の `Images` は保存済みイメージ、Buildx の `Reclaimable` はビルドキャッシュの回収対象を示す。同じ ID の複数のタグは一つのイメージを指す。異なる ID でもレイヤーを共有するため、表示サイズは単純に合算できない。各イメージの共有サイズと固有サイズは `docker system df -v` の `SHARED SIZE`・`UNIQUE SIZE` で確認する。[Docker の容量表示](https://docs.docker.com/reference/cli/docker/system/df/)

| 整理する対象 | ホストでの操作 |
| --- | --- |
| ビルドキャッシュ | `./scripts/tools/clean-docker-cache.sh` を実行し、Docker の確認で `y` と Enter を入力する。完了時に `Total` が表示される。対象は `default` builder を使う全プロジェクトの未使用キャッシュ。 |
| 過去の ATD 完成イメージ | ビルド完了後に `./scripts/tools/clean-docker-cache.sh --old-images` を実行し、削除したい ID の先頭を空白区切りで入力する。`all` は表示中の候補をまとめて選ぶ。対象機種ごとに CLI のキャッシュ別名が指す ATD イメージを確認できない場合は、個別に ID を指定する。空のまま Enter で中止する。 |

イメージの候補は `isaac_ros-realsense455-atd_<hash>` のタグを ID ごとにまとめて表示する。コンテナ（停止中を含む）が参照するイメージ、Isaac ROS CLI の `cached_isaac_run_dev_image_local:latest` が指すイメージ、別用途のタグが付いたイメージを保持する。Isaac ROS・RealSense の共通中間イメージ、Ubuntu、ボリューム、bag・地図・モデルはこの操作の対象外。整理後の使用量は再度 `--dry-run` で確認できる。[Docker のイメージ削除](https://docs.docker.com/reference/cli/docker/image/rm/)・[Docker のキャッシュ削除](https://docs.docker.com/reference/cli/docker/buildx/prune/)

<a id="kart-cli"></a>

## `ISAAC_ROS_WS must be <kart project>/ros2_ws beside packages.repos` で止まる場合

この例外は旧 kart 版 CLI がコンテナを起動する直前に出す。ATD の `packages.repos` や workspace パスを変更する必要はない。復旧では Notebook ホストの CLI を NVIDIA 配布版に切り替える。同じホストで旧 kart を使う場合は、次回の kart 起動にもこの切り替えが影響する。[kart 版の起動処理](https://github.com/carbuncle01/isaac-ros-cli/blob/dff97a3af51ec40f8d0a92e65220391ec735a041/scripts/run_dev/run_dev.py#L216-L224)

### 旧版を purge 済みの場合

Ubuntu 24.04 / amd64 の Notebook で `Unable to locate package isaac-ros-cli` が出たときは、apt の索引にパッケージが見えていない。NVIDIA 公式の署名鍵と `release-5 noble main` を登録し、索引を更新して 5.x 系の最新 CLI をインストールする。ATD の標準セットアップが登録する `release-5.0` は 5.0 系を対象とする。[NVIDIA の apt 配布元登録手順](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#configure-isaac-ros-apt-repository)

```bash
(
  set -e -o pipefail
  sudo apt-get install -y ca-certificates curl gnupg

  repo=https://isaac.download.nvidia.com/isaac-ros
  keyring=/usr/share/keyrings/nvidia-isaac-ros.gpg
  tmp_key=$(mktemp)
  trap 'rm -f -- "$tmp_key"' EXIT
  curl -fsSL "$repo/repos.key" | gpg --batch --dearmor > "$tmp_key"
  sudo install -m 0644 "$tmp_key" "$keyring"

  sources=/etc/apt/sources.list.d/nvidia-isaac-ros.list
  entry="deb [signed-by=$keyring] $repo/release-5 noble main"
  grep -Fxq "$entry" "$sources" 2>/dev/null || printf '%s\n' "$entry" | sudo tee -a "$sources"

  sudo apt-get --error-on=any update
  apt-cache policy isaac-ros-cli
  sudo apt-get install isaac-ros-cli
)
```

`apt-cache policy` で NVIDIA の `release-5 noble/main` と候補版を確認できる。インストール成功後、Notebook の ATD の clone 直下で実行する。

```bash
./scripts/setup.sh --action isaac && ./scripts/dev.sh --build
```

### 旧版がインストールされている場合

Notebook ホストで、導入版と apt の配布元を確認する。`+kart1` が付いている版のほか、公式版と同じバージョンで `run_dev.py` だけを書き換えた場合も対象になる。

```bash
dpkg-query -W isaac-ros-cli
grep -nF '<kart project>/ros2_ws beside packages.repos' /usr/lib/isaac-ros-cli/run_dev.py
sudo apt-get update
apt-cache madison isaac-ros-cli
```

`https://isaac.download.nvidia.com/isaac-ros/release-5.0` のうち、Notebook は `noble/main`（amd64）、Jetson で同じ例外が出た場合は `noble-jetpack/main`（arm64）の行にある版を選び、次の入力欄に貼り付ける。該当行がない場合は NVIDIA 公式手順で **x86 (Ubuntu 24.04 Noble)** または **Jetson (JetPack 7.2)** を選び、**US CDN / Specific Version (release-5.0)** の配布元を登録する。その後 `sudo apt-get update` と `apt-cache madison isaac-ros-cli` を再実行する。[NVIDIA の apt 配布元登録手順](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html#configure-isaac-ros-apt-repository)

```bash
read -r -p 'NVIDIA の行に表示された版番号: ' atd_cli_version &&
  test -n "$atd_cli_version" &&
  sudo apt-get install --reinstall --allow-downgrades "isaac-ros-cli=$atd_cli_version" &&
  dpkg-query -W isaac-ros-cli &&
  dpkg --verify isaac-ros-cli
```

導入版が選んだ版と一致し、`dpkg --verify` で `/usr/lib/isaac-ros-cli/run_dev.py` に差異が出ないことを確認する。ATD の clone 直下で CLI の Docker モードを確認して、build を再開する。

```bash
./scripts/setup.sh --action isaac
./scripts/dev.sh --build
```

## 4. 通常の使い方

通常は `./scripts/setup.sh --action start` または `./scripts/dev.sh` で入る。
PC では受信した bag の offline 処理、地図作成・編集、解析、可視化を行う。共通 Web アプリは `./scripts/tools/webui.sh` で起動し、Notebook ホストの Chrome から `http://127.0.0.1:8766` を開く。[Web アプリ](../../apps/web/README.md)を参照する。Jetson では UI を起動せず、入力・観測用の `./scripts/bringup.sh input` を使用する。

D455 を PC に直接接続する場合は devices のホスト準備も行い、[Jetson 手順の D455 確認](jetson.md#3-d455-と-dualsense)と同じ順に確認する。
PC の Docker と Jetson の Docker は別々に build し、PC の `build/` / `install/` を Jetson 用として転送しない。

## 5. Web アプリから Jetson へ接続する SSH

転送とライブ・入力監視は Notebook のコンテナ内で SSH を実行する。鍵と `known_hosts` の正本は Notebook ホストの `~/.ssh` とし、アプリにパスワードや秘密鍵を保存しない。初回はホストのターミナルで、実際のユーザー名・接続先・portへ置き換えて接続する。

```bash
# Notebook ホスト。表示されるホスト鍵を Jetson 側の情報と照合して登録する。
ssh -p 22 USER@JETSON_HOST true
# パスフレーズ付き鍵は既存の SSH agent へ読み込んでおく。
ssh-add ~/.ssh/id_ed25519
./scripts/dev.sh
./scripts/tools/webui.sh
```

`ssh-add` は agent を利用する場合だけ実行し、鍵ファイル名は利用中のものに合わせる。Web の「転送」で同じ接続先・ユーザー名・SSH portを保存する。ライブ・入力画面もこの接続設定を共用する。アプリは `BatchMode=yes` / `StrictHostKeyChecking=yes` を使うため、未登録のホスト鍵や未解除の鍵を接続成功扱いしない。

`scripts/dev.sh` は `~/.ssh` が存在するときだけ `/home/admin/.ssh` へ readonly mount する。ユーザー・workspace・apt CLI の追加引数は公式loaderと同じ優先順で調べ、公式テンプレートの realpath 形式を含め同じ source・readonly の mount があれば再利用する。異なる source・書込可能・重複した指定は起動前に停止する。既存のファイルをコピー・上書きせず、鍵の権限・所有者も変更しない。コンテナの admin が秘密鍵を読めるにはホストとの UID・ファイル権限の整合が必要であり、アクセス失敗時はホストの権限を緩めず、UIDとmountを確認する。

Notebook では既存の `SSH_AUTH_SOCK` が Unix socket の場合、`/run/atd/ssh-agent` へ共有してコンテナの同環境変数を設定する。これにより agent 内の鍵を使用できる。agent がなければ readonly の鍵ファイルを通常の OpenSSH 手順で使用する。Jetson 側の agent 共有は上流 CLI の既存経路を維持する。

SSH directory・agent の共有条件が変わった場合、`dev.sh` は旧コンテナへそのまま再接続しない。実行中の処理を終了してコンテナを再作成する。agent を再起動した場合も再作成する。ホスト鍵の登録・更新は readonly のコンテナ内から行わず、Notebook ホストで行う。設定・引数の単体検証と、実際の鍵・SSH/SCP・Jetson ROS の動作確認は区別する。
