# Docker 内の Python 環境

Python の追加依存と仮想環境の管理には uv を使い、ROS 2 用、AI 学習用、Notebook の VGL export・Web API・Raceline 用を分ける。Docker には uv `0.13.8` を固定導入する。用途ごとに独立した `pyproject.toml`・`uv.lock`・venv を持ち、学習・export 依存の更新が ROS 2 の追加依存へ波及しない構成とする。

## 用途と依存の管理

| 環境・対象 | 管理方法 | 方針 |
| --- | --- | --- |
| OS・ROS 2・Isaac ROS の配布パッケージ | Docker / apt | ベースイメージと `docker/dependencies.apt` に従う |
| ROS 2 用の Python 追加依存 | 独立した uv project と venv | システム Python と `--system-site-packages` を使用し、ROS 2 の Python 拡張と ABI を合わせる |
| AI 学習用の Python 依存 | 独立した uv project と venv | system site-packages と ROS 2 の Python 環境変数を引き継がない |
| Notebook の VGL export 用 | `vgl-export` project と venv | Python 3.12 / Linux x86_64 用の固定依存。ROS 用・学習用へ混在させない |
| Notebook の Web API 用 | `web` project と venv | FastAPI・領域処理・幾何検証用。ROS の Python 検索パスを引き継がず、Jetson では利用しない |
| Notebook の Raceline 最適化用 | `raceline` project と venv | Python 3.12 の固定依存。`mincurv` / `mincurv_iqp` helper を別プロセスで実行し、Web・ROS へ混在させない |
| 解析・推論等の追加用途 | 必要時に別 project / venv を追加 | ROS 2 用・学習用のどちらへ入れるかを曖昧にせず、依存の互換性に応じて決める |

ROS 2 の apt 依存を PyPI から入れ直す手順にはしない。`uv.lock` が固定する対象は各 project の追加 Python 依存であり、ベースイメージや apt パッケージの全状態ではない。再現にはそれらのバージョン・digest も記録する。

OS Python への `pip install`、`uv pip install --system`、`--break-system-packages` は標準の導入手順に含めない。追加・更新する依存は用途別の project へ記録し、更新した lockfile を管理する。

## 操作と保存先

コンテナは先に `./scripts/dev.sh` で起動する。Python 環境の同期・確認・実行は、ホストから [`scripts/tools/python.sh`](../scripts/tools/python.sh)、コンテナ内では `atd-python` を使う。引数なしでは fzf で用途・操作を選べる。Python 環境の操作は `sync`・`check`・`shell`・`run` に限定する。
ROS package の build は独立した [`scripts/build.sh`](../scripts/build.sh) を入口とし、用途名や操作名を指定しない。

```bash
./scripts/tools/python.sh
./scripts/tools/python.sh ros sync
./scripts/tools/python.sh ros check
./scripts/tools/python.sh train shell
./scripts/tools/python.sh train run -- python -c 'import sys; print(sys.executable)'
./scripts/build.sh
./scripts/build.sh --packages-up-to atd_web_bridge
```

| 操作 | 内容 |
| --- | --- |
| `sync` | 選択した用途の環境を `uv sync --locked` で同期する |
| `check` | 選択した用途の環境を確認する |
| `shell` | 同期後、その用途の対話シェルを開く |
| `run -- COMMAND ...` | 同期後、その用途の環境でコマンドを実行する |

ホストの `scripts/tools/python.sh` は `--host notebook|pc|jetson` と `--dry-run` に対応し、コンテナを自動起動しない。コンテナ内では、例えば `atd-python ros check` のように使用する。
`build.sh` も同じホスト指定・実行予定表示に対応し、コンテナ内の専用コマンド `atd-build` を呼ぶ。コンテナ内からは `atd-build --packages-up-to atd_web_bridge` と直接実行できる。`atd-build` がビルド対象 workspace への移動、`colcon build` の引数、CMake の Python interpreter 指定を扱い、`python.sh` / `atd-python` は呼び出さない。
両コマンドは共通の環境準備処理を利用し、ROS 用 Python の ABI 確認・uv による同期・ROS 環境の有効化・DDS 設定の維持を共有する。従来の `python.sh ros build` / `atd-python ros build` は廃止し、ビルドには `build.sh` / `atd-build` を使用する。
専用コマンドと共通処理を導入した構成へ更新するときは、利用中の処理を終了し、`./scripts/dev.sh --build` で Docker イメージを更新・コンテナを再作成する。
通常の `atd-shell` は ROS 2 のシステム環境を保ち、用途別環境を自動では有効化しない。`shell` を終了すれば呼び出し元の環境へ戻る。
ROS 2 用環境は Docker 起動時に root hook から `gosu admin` を通して自動同期する。通常ユーザーの権限で同期することで、venv・cache をその後も手動操作できる所有者に保つ。起動時に学習用環境は同期せず、`train sync`・`train run`・`train shell` で必要時に準備する。

| 対象 | 配置 |
| --- | --- |
| ROS 2 用 manifest / lock | `docker/python/ros/{pyproject.toml,uv.lock}` |
| 学習用 manifest / lock | `docker/python/train/{pyproject.toml,uv.lock}` |
| VGL export 用 manifest / lock | `docker/python/vgl-export/{pyproject.toml,uv.lock}` |
| Web API 用 manifest / lock | `docker/python/web/{pyproject.toml,uv.lock}` |
| Raceline 用 manifest / lock | `docker/python/raceline/{pyproject.toml,uv.lock}` |
| venv | `<選択した ROS workspace>/.atd-python/<architecture>/<purpose>` |
| uv cache | `<選択した ROS workspace>/.atd-python/cache` |

venv の architecture は `uname -m` の値を使う。workspace の bind mount へ保存するため、CLI が `--rm` でコンテナを削除しても保持される。colcon の探索対象にしないため `.atd-python` に `COLCON_IGNORE` を置く。manifest と lockfile は共通の参照元で管理し、venv と cache は管理対象外とする。
未設定時の workspace は実際の clone 配下の `ros2_ws`。標準配置なら venv は `~/workspaces/atd/ros2_ws/.atd-python/` に作成する。モデル・export bundle 等の永続資産は、venv と分けて clone の `data/results/` 以下に保存する。
`ros` / `train` の追加 Python 依存は空であり、学習用ライブラリの導入・動作確認までは含まない。`vgl-export` は PyTorch / torchvision・ONNX 等を固定する Notebook 専用環境で、モデルからの export を選んだときに同期する。依存の版は project と lockfile を参照する。
VGL export・既存 bundle の package・Jetson の engine build は [専用の操作手順](vgl_models.md)を使う。Jetson の engine build に Notebook の export 用 venv をコピーしない。

共通 Web UI は Notebook の Docker 内で `atd-web` を実行し、`web` 環境を同期・選択して起動する。ホストの `scripts/tools/webui.sh` は同じコンテナ内コマンドを呼ぶ補助入口である。ソースの backend / processing は ホストと共有する checkout から読み、frontend の build と node_modules は workspace の `.atd-web/frontend/` へ作成する。`web` は Linux x86_64 の Notebook 専用で、ROS CLI を呼ぶ子プロセスでだけシステムの ROS 環境を読み込む。通常起動時に `atd-python raceline sync` で最小曲率 helper 用環境も同期し、`ATD_RACELINE_PYTHON` にその Python を設定して別プロセスを起動する。`raceline` は Python 3.12 と NumPy・SciPy・quadprog・固定 commit の trajectory-planning-helpers 等を独立した lock で管理する。`atd-web build` は frontend の build だけで、Python 環境の同期とは別である。`atd-web check` は `web` / `raceline` 環境と公式・自作 ROS package の導入状態を診断する。操作は [Web アプリ](../apps/web/README.md)を参照する。

## ROS 2 と Python interpreter

ROS 2 用 venv は、コンテナの `/usr/bin/python3` を明示して作成する。uv が別の Python を自動取得し、その interpreter で apt の ROS 2 拡張を実行する構成を避ける。
`--system-site-packages` は、システムに導入済みの Python パッケージを参照するための設定である。ROS 2 用 venv はシステムから完全に隔離された環境ではなく、追加依存が apt 側のモジュールを隠す可能性もある。NumPy 等の ABI を持つ依存は、ROS 2 との import・実行を確認してから更新する。

venv の有効化だけで、すべての ROS 2 コマンドが venv の interpreter を使うとは限らない。apt コマンドの固定 shebang、CMake が検出した Python、install された node の shebang を区別して確認する。
Python node の package build は従来どおり CMake / `ament_cmake_auto` を使い、必要に応じて `ament_cmake_python` を併用する。uv の project は ROS package の build・install の代替ではない。
保存地図の点群取得に使う自作 `atd_mapping` も apt や Web 環境の同期だけでは build されない。初回は Notebook の Docker 内で `atd-build --packages-up-to atd_mapping` を実行する。画像計算は NVIDIA の C++ component、ROS の監視・snapshot 採取は ROS 互換 Python の別プロセスが担当し、Web / Raceline 用 venv に `rclpy` を混在させない。
自作 package の編集元はリポジトリの `ros2_ws/src/` とする。別の workspace を使う場合、コンテナの workspace では `src/atd/` に mount する。`build.sh` は workspace の `src` を探索対象にする。直接 `colcon build` を使う場合も `--base-paths src` を明示する。詳しくは [構成規約](project_structure.md)を参照する。
workspace にリポジトリの `ros2_ws` を選ぶ場合は、同じソースが workspace の `src/` にそのまま見えるため、追加の source mount は使わない。Python 環境の用途名 `ros` と `train`、`docker/python/ros/` 等の依存定義の名前は変更しない。

## AI 学習環境との分離

学習用のプロセスへ ROS 2 用の `PYTHONPATH`、`PYTHONHOME`、venv を引き継がせない。venv を別ディレクトリに置くだけでは、親シェルから引き継いだ Python の検索パスは消えないため、実行の入口で分離する。
bag の記録・ROS 2 API の使用と、抽出済みデータによる学習は、それぞれ対応する環境で実行する。ROS 2 と学習コードの直接連携が必要になった場合は、その依存と実行方式を個別に設計する。

`ros` / `train` は Python 3.12 を基準とする。AI 学習用の framework と CUDA 対応 wheel は未選定であり、学習用 Python の版もそれらの互換性に合わせて必要時に見直す。VGL export 用は独立した Python 3.12 の固定依存を使うが、Notebook の wheel を Jetson に流用しない。対象 OS・アーキテクチャ・CUDA・driver と実際の GPU 動作は別に確認する。
学習環境でも、コンテナが提供する CUDA 等の native library の検索パスは維持する。Python 依存を分ける構成であり、別 Docker イメージの全ライブラリを再現するものではない。
今回の環境分離は、Jetson でのモデル学習や個別 framework の利用を動作確認済みにするものではない。

## lockfile と環境の再作成

- 用途ごとの `pyproject.toml` と `uv.lock` は管理対象とする。
- venv と cache は管理対象にせず、各ホスト・コンテナで作成する。Notebook と Jetson の venv をコピーしない。
- 依存更新と通常実行を分け、通常の同期では lockfile の意図しない更新を防ぐ。
- 学習依存を追加するときは、同じ lockfile が対象アーキテクチャを扱えるかを確認し、必要なら project 自体を分ける。

## 検証の範囲

スクリプトの操作契約、環境の選択、lockfile と設定の整合を確認する自動テストと、Ubuntu / Jetson の Docker 内での import・GPU 演算は分けて記録する。
Raceline の実 QP / IQP solver を使う小規模テストはローカルで確認した。これは対象 Docker 内の導入、ROS package build、実 GPU / bag での地図処理の動作確認とは別であり、それらは未検証である。
対象機では ROS 2 用の `sys.executable`・`rclpy` import、CMake の Python 検出先、install 後の Python node 起動を確認する。学習用では ROS 2 の追加検索パスが混入していないことを確認し、framework を追加した段階で GPU 演算を別に検証する。

## 参照

- [uv: Docker での利用](https://docs.astral.sh/uv/guides/integration/docker/)
- [uv: Python 環境の作成と使用](https://docs.astral.sh/uv/pip/environments/)
- [uv: project の lock と同期](https://docs.astral.sh/uv/concepts/projects/sync/)
