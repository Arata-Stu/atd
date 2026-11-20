# VGL モデルの準備・転送・engine build

VGL モデルは Notebook で準備・ONNX export し、export bundle 全体を SCP で Jetson へ渡す。Jetson は受信済み bundle から対象 GPU 用の TensorRT engine を生成する。
操作の入口は [`scripts/vgl.sh`](../scripts/vgl.sh)。引数なしでは fzf で処理と資産を選べる。

```bash
./scripts/vgl.sh
```

この入口にはモデル export・検証・転送・engine build の処理を実装している。対象機の Isaac ROS 5 SDK / GPU を使った export、実 SCP、Jetson の engine build と VGL localization の通し検証は未実施である。

Notebook の Docker 内で `atd-vgl list` に表示される export は、Web UI の「地図とライン」→「VGLモデル一覧」でも確認できる。地図をまだ作っていない段階でモデル名・export ID・入力解像度を確認できる。export 直後に画面を開いたままの場合は「一覧を更新」を押す。

そのモデルを地図に使うには、rosbag から作成した cuVSLAM 地図を開き、「VGL地図を作成」→「使用するVGLモデル」で該当の export を選ぶ。「VGL地図の準備を開始」を押すと、この Notebook の GPU 用 engine と選択した地図の VGL 特徴を生成する。[Notebook の地図作成手順](../apps/web/README.md#map)

## 実行前の準備

Notebook と Jetson のそれぞれで setup の workspace・保存先を設定し、今回の処理を含む Docker イメージを build する。

```bash
./scripts/dev.sh --build
```

Docker 内で行う export・診断・build 等は、コンテナを起動したまま別端末から `vgl.sh` を実行する。スクリプトはコンテナを自動起動しない。PC / Jetson は通常の自動判定と `--host notebook|pc|jetson` に対応する。
VGL の Notebook export には独立した uv 環境 `vgl-export` を使い、ROS 用・学習用の環境とは依存を分ける。初回の同期や入力資産の取得には通信と空き容量が必要となる。

## 作業の分担

| 処理 | 実行場所 | 内容 |
| --- | --- | --- |
| 診断 | 各ホスト | コンテナ・処理に必要な環境の状態を確認 |
| export | Notebook | ALIKED-n16 の元モデルから FP32 / opset 17 の ONNX を生成 |
| 既存資産の package | Notebook | すでに export 済みの NVIDIA ONNX bundle を取り込み。再export は行わない |
| transfer | Notebook | manifest と hash を検証し、export bundle だけを SCP 転送 |
| build | Jetson | 公式の二つの C++ exporter で TensorRT engine を生成 |
| verify / list / logs | 操作したホスト | bundle、指定した engine の検証・bundle 一覧・ログ確認 |

Notebook の build 操作は、設定した Jetson 上の `scripts/vgl.sh build` へ SSH で渡す。Jetson で同じ入口を実行した場合は、そのホストのコンテナで build する。Notebook と Jetson の双方に、対応する checkout・起動済みコンテナ・保存先設定が必要である。

## メニューと直接実行

通常は引数なしのメニューで `doctor` → `export` → `transfer` → `build` と進める。asset / export ID は入力または fzf で選択する。export の既定値は `mode=aliked`、画像サイズ `848 × 480`、build の target は `jetson_orin_nano`。

直接指定する場合も同じ入口を使う。以下は Notebook の例で、ID は新しい資産を識別する名前に変えられる。

```bash
./scripts/vgl.sh doctor
./scripts/vgl.sh export --asset-id aliked_lightglue --export-id d455_v1 \
  --mode aliked --width 848 --height 480
./scripts/vgl.sh verify --asset-id aliked_lightglue --export-id d455_v1
./scripts/vgl.sh transfer --asset-id aliked_lightglue --export-id d455_v1
./scripts/vgl.sh build --asset-id aliked_lightglue --export-id d455_v1 \
  --target-id jetson_orin_nano --build-id orin_v1
./scripts/vgl.sh list
./scripts/vgl.sh logs
```

`verify`・`list`・`logs` は実行したホストの資産・ログを対象にする。Notebook の `build` だけは SSH で Jetson へ委譲する。engine を検証するときは Jetson 上で次を実行する。`--build-id` を省いた verify は export bundle の検証になる。

```bash
# Jetson の端末で実行
./scripts/vgl.sh verify --asset-id aliked_lightglue --export-id d455_v1 \
  --target-id jetson_orin_nano --build-id orin_v1
```

ID は96文字以内の英数字・`.`・`_`・`-` を使い、先頭は英数字とする。同じ export / build ID の資産は上書きしない。`--dry-run` は設定読込み・Docker / SSH / SCP・保存を行わず、予定だけを表示する。

## export と既存 bundle の取り込み

ALIKED-n16 の export と、既存 ONNX の package は別の操作として選択する。
標準の export は固定した入力ソース・重みを使い、モデルから ONNX を生成して checker と入出力契約を確認する。保存する bundle は `models/`・`configs/` と `manifest.json` を含み、asset / export ID、入力の由来、ファイル hash を対応付ける。
既存資産を取り込む package モードは、すでにある ONNX bundle をコピー・検証して管理対象にする操作であり、モデルからの export を済ませたものとして新しく計算し直す処理ではない。

`--mode package` を選び、必要なら `--source-dir` に `models/` と `configs/` を含む package share root を指定する。Notebook のコンテナから見える絶対パスを使う。省略した場合はインストール済み NVIDIA package の share を参照する。入力の画像サイズと指定値が異なる場合はエラーとなるため、対象 bundle の契約に合わせる。

package モードで取り込む ALIKED ONNX は、唯一の FP32 入力 `image` が `[1,3,H,W]`、FP32 出力が `keypoints=[2048,2]`、`descriptors=[2048,128]`、`scores=[2048]` であることを静的に確認する。出力には先頭 batch 次元を付けず、この契約を確認できない既存 ONNX は拒否する。モデルからの標準 export は固定 exporter の top-K 設定を由来として記録し、symbolic な出力形状を許容するため、既存モデルの静的な形状検査とは区別する。

export 対象の画像サイズは bundle の情報として記録し、Jetson の build でもその契約を使う。ONNX が生成・検査できることと、対象 SDK / TensorRT での対応演算・数値精度・localization の成功は分けて確認する。
依存の固定は [`docker/python/vgl-export/pyproject.toml`](../docker/python/vgl-export/pyproject.toml) と [`uv.lock`](../docker/python/vgl-export/uv.lock)、export 処理は [`tools/vgl/export_model.py`](../tools/vgl/export_model.py)を参照する。

## Jetson との接続設定

SSH の接続先、port、Jetson 上の ATD checkout と結果保存先は `config/vgl.env` に保持する。必要な値が未設定の場合は対話入力して保存し、権限は `0600` とする。SSH 鍵・接続先 alias 等はホストの SSH 設定を使う。
この設定は転送・遠隔 build のためのものであり、PC から車両の走行制御を与えるものではない。

| 設定 | 内容 |
| --- | --- |
| `ATD_VGL_SSH_TARGET` | `jetson` 等の SSH alias または `user@host` |
| `ATD_VGL_SSH_PORT` | SSH port。既定 `22` |
| `ATD_VGL_REMOTE_PROJECT` | Jetson 上の ATD checkout の絶対パス |
| `ATD_VGL_REMOTE_RESULT_DIR` | Jetson の結果保存先の絶対パス。`storage.env` の設定と対応させる |

設定例は [`config/vgl.env.example`](../config/vgl.env.example)。別ファイルを使う場合は `ATD_VGL_CONFIG` を指定する。SSH の秘密鍵や password をこのファイルへ保存しない。
標準配置なら Jetson の checkout は `~/workspaces/atd`、結果保存先はその `data/results` となる。保存先を見直した場合は、保存済みの `ATD_VGL_REMOTE_RESULT_DIR` も実パスへ合わせて更新する。旧データは自動移動しない。

SCP の対象は選択した `exported/<export_id>/` の bundle 全体で、元モデルの checkout や Notebook 用 engine を一括コピーしない。転送先で hash と manifest を検証してから正式な保存先へ反映し、同じ ID の既存資産を上書きしない。

## 保存先とログ

```text
/data/results/models/vgl/<asset_id>/
├── source/                              # Notebook の入力ソース・重み・設定
├── exported/<export_id>/
│   ├── models/                          # ONNX と関連データ
│   ├── configs/
│   └── manifest.json                    # ID・由来・hash 等
└── engines/jetson_orin_nano/<build_id>/   # Jetson で作る対象環境用 engine

/data/results/runs/<run_id>/             # 処理・build のログと結果
logs/vgl/<pc|jetson>/                    # ホスト側の操作ログ
```

結果保存先は既存の `ATD_PC_RESULT_DIR` / `ATD_JETSON_RESULT_DIR` がホストの実パスに対応し、未設定時は実際の clone 配下の `data/results` を使う。標準 clone 位置なら `~/workspaces/atd/data/results` となる。新しいデータ用 mount は増やさない。bundle・engine・run のログは Git 管理外の永続資産として保持し、cache として削除しない。保存構成全体は [構成規約](project_structure.md#データの保存構成)を参照する。
`logs` は対話端末でホストの実行ログを選び、非対話端末では最新ログを表示する。処理が失敗した場合も終了値とログを残す。

Jetson の build は model / export ID と hash、GPU 名・UUID・compute capability、TensorRT の版、CUDA driver version、関連 package の版、`atd_runtime_revision` とログの対応を記録する。`atd_runtime_revision` は Dockerfile・起動 script 等の構成 hash であり、Docker image digest 自体は現実装の記録に含まれない。target の名前だけで互換性を判断せず、実際の環境を確認する。
公式の `export_lightglue_engine`・`export_extractor_engine` は engine 生成の段階で使う。Notebook の ONNX export と同じ操作として扱わない。[公式 VGL localization](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/visual_global_localization/tutorials/tutorial_cuvgl_localization.html)

## 検証の範囲

自動テストでは一時ディレクトリと偽コマンドで、ID / hash 検査、bundle の保持、転送・build のコマンド、失敗時の扱いを確認する。export 用のソース・Python 処理の検査と、実 PyTorch / ONNX / TensorRT による処理結果は区別する。
実際の Notebook・Jetson・SSH 接続先でモデルを取得・export・転送・build した結果は、対象機のログとあわせて確認する。数値精度と Isaac ROS 5 SDK / VGL localization での実利用は未検証である。
