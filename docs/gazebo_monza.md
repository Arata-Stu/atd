# Monza の D455 ステレオ試作

1/10 RC 用に編集した Monza の Raceline を、スタートから最初の右・左シケインを抜けるまで 12.5 m たどる。Gazebo Jetty と OGRE 2 が同じ台に取り付けた左右のカメラを描画する。床にはアスファルト、壁にはコンクリートの撮影素材を実寸で貼り、低いコース境界、白いテープ、柱、シャッターと室内照明を配置する。既存の HDMap は走路の形状に使い、室内の配置は試作用に設定した。[Monza 地図](../samples/f1tenth/maps/monza/)・[写真素材と縮尺](../tools/gazebo_monza/materials/README.md)

## Notebook で再生・測定する

NVIDIA GPU と NVIDIA Container Toolkit が使える Ubuntu の Notebook ホストで、ATD のルートから実行する。Notebook ホストの `atd` メニューからも `gazebo` を選べる。

```bash
./scripts/gazebo.sh prepare
./scripts/gazebo.sh run
```

`prepare` は初回・Dockerfile 更新時にオンラインで実行する。ROS 2 Lyrical と Jetty の配布パッケージから専用イメージを作り、容量の明細を出力する。以降はコース・画像素材・プログラムを clone から読めるためオフラインで使える。既存の Isaac ROS 開発コンテナを起動する必要はない。Gazebo はサーバーモードで OGRE 2 の EGL 描画を使う。[公式の対応構成](https://gazebosim.org/docs/jetty/ros_installation/)・[Gazebo のヘッドレス描画](https://gazebosim.org/api/sim/10/headless_rendering.html)

`run` は動画用と速度計測用にシミュレーションを最初から個別に実行する。端末に各試行の保存先が出る。標準では `data/results/runs/gazebo_monza/`。保存先を設定済みなら Notebook 用 results の同じ相対位置に保存する。

| 操作 | 成果物 |
|---|---|
| `prepare` | `storage.json`、ビルド前後の `docker system df -v` と `docker buildx du`、`build.log` |
| `video` | `stereo.mp4`（左目／右目の横並び）、`left.mp4`、`right.mp4`、`video.json` |
| `benchmark` | `benchmark.json` に実時間で受信したペア数・Hz、シミュレーション時刻のHz、欠落・重複、最大到着間隔 |
| 両試行 | `run.json`、`scene/scene.json`、台の実際の位置 `scene/rig_motion.jsonl`、サーバー・ブリッジ・受信側のログ |

`stereo.mp4` を Notebook のブラウザにドラッグすると、左右を同時に再生・シークできる。実時間の配信と独立に、動画は同じタイムスタンプを持つ左右画像をシミュレーション時刻で30 fpsにサンプリングする。動画の既定長は10秒。受信が滞り前のペアを保持した動画フレームの数は `video.json` の `held_video_frames` で確認できる。

### 取付高・時間を変える

```bash
./scripts/gazebo.sh video --camera-height 0.14 --camera-pitch-deg 3
./scripts/gazebo.sh benchmark --duration 60
```

`--camera-height` は床から光学中心までの m。初期値は **0.12 m**、後軸から前方に 0.18 m、水平と仮定している。軸中心高さの仮定値は 0.0325 m。車体設定の後輪軸基準の z と区別する。左右の基線は公称 0.095 m、水平画角は公称 87°、640 × 480 の `mono8` 理想ピンホールカメラ。実機の取付高と個体別キャリブレーションは未測定。モノクロ画像は可視光材質の描画で、D455 の投光パターン、露出制御、振動による像の変化までは再現していない。[実車の座標定義](vehicle_layout.md)・[D455 仕様](https://realsenseai.com/products/stereo-depth-camera-d455/)

動画では指定時間で同じ区間を一度通る。ベンチマークでは既定の片道10秒でカメラ台が同じ姿勢列を往復し、描画中の負荷を維持する。移動は保存済み Raceline を用いた軌道再生で、映像と配信の検証用。タイヤや ESC、制御器による閉ループ走行の比較は既存の [2D シミュレーター](simulation.md) で行う。

## 90 Hz と容量の読み方

- `benchmark.json` の `unique_ordered_pairs` は画像と時刻を左右で照合して、重複を除いて数えた組数。60実秒なら目標は5,400組。購読接続と校正を確認し、ウォームアップを除いた区間で `paired_wall_hz` を測る。`paired_sim_hz` は画像に付いたシミュレーション時刻による周期。両者の差は実時間への追従を見る手掛かりになる。
- `pass_90hz` は実時間で89.1組/秒以上、シミュレーション周期が90 Hzの±1%、観測したペア間の欠落推定・ペアの逆順がゼロの場合に成立する。到着間隔の最大値も記録する。結果が閾値未達ならベンチマークは終了コード2を返す。描画やROSの起動失敗とはログと終了コードで区別する。この試行の受信側は計測専用で、cuVSLAM同時実行時の処理性能は別途測定する。
- `storage.json` の `filesystems[].net_used_bytes` はビルド前後の実ディスク差分、`peak_added_bytes_sampled` はビルド中に0.5秒間隔で観測した最大増分。共有レイヤー込みの論理イメージサイズ `image_size_after_bytes` と使い分ける。ファイルシステム全体の空き容量を読むため、測定中は同じディスクで別のビルドや大きなコピーを避ける。ビルドキャッシュ・既存イメージを自動削除する処理はない。生成した動画などの占有量は各試行の `output-size-kib.txt` に KiB で残る。

ROS は Notebook のループバックで Fast DDS を使う。標準ドメインは 74。動画・計測の起動中は ROS に `/clock`、`/tf_static`、`/camera/camera/infra{1,2}/image_rect_raw` と両方の `camera_info` が出る。Simulation 用の別プロセスで購読するときは同じドメイン・ループバック設定を使う。Jetson への接続や車両への指令は不要。

## 試作の確認結果

2026-11-20、macOS ARM64 のネイティブ Gazebo Jetty 10.7／OGRE 2（Metal）で、取付高 0.12 m のシーンを描画した。Gazebo Transport 側でシミュレーション時刻が一致する 640 × 480 の左右画像を899組取得し、観測区間の画像タイムスタンプは約11.11 ms間隔、重複と未対応画像は左右とも0だった。左右の投影行列から求めた基線長は 0.095 m。両眼それぞれ10秒・30 fpsのMP4を確認した。Notebook の Linux／EGL、ROS 2 ブリッジを含む実時間レート、専用イメージのビルド容量は Notebook ホストで測定する。
