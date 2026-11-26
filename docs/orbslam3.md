# ORB-SLAM3 のステレオ地図と初期自己位置推定

実験ブランチは `experiment/orbslam3-localization`。既定の `atd replay` は cuVSLAM を使う。ORB-SLAM3 を選ぶと、D455 または Gazebo の左右の rectified IR・CameraInfo と `/tf_static` からステレオ追跡できる。地図作成は Notebook、実車の初期定位・追跡は Jetson で実行する。

## Linux Notebook・Jetson の準備

各機体の checkout で対象ブランチを選択した後、ホストから専用イメージを構築する。`--orbslam3` は上流リビジョンを固定した CPU 版・ATD の固定地図用変更・ORB 語彙を追加する。上流のプログラムと語彙はビルド時に取得する。実行中の開発コンテナを再利用するときは、ORB 対応イメージで起動したものを使う。

```bash
./scripts/dev.sh --build --orbslam3
```

ORB-SLAM3 本体の修正を取り込むときも、ホストで実験ブランチを更新し、上のコマンドでイメージを再構築する。Notebook の `atd_pc_dev` が稼働中なら、使用中の処理を終了してからホストで `docker stop atd_pc_dev` を実行する。`atd-build` はイメージ内に導入済みの ORB ライブラリを使う。

起動したコンテナ内で ROS パッケージをビルドする。この指定では ORB ライブラリが見つからなければビルドに失敗する。通常の cuVSLAM イメージでは追加ノードのビルドを省略できる。

```bash
atd-build --packages-up-to atd_orbslam3 atd_bringup --cmake-args -DATD_REQUIRE_ORBSLAM3=ON
atd-shell ros2 pkg executables atd_orbslam3
```

最後の出力に `atd_orbslam3 orbslam3_node` があれば replay を開始できる。通常版から同じ workspace を引き継ぎ、パッケージが表示されても実行ファイルの一覧が空の場合は、ORB 対応コンテナ内でノードを再構成する。

```bash
atd-build --packages-up-to atd_orbslam3 --cmake-clean-cache --cmake-args -DATD_REQUIRE_ORBSLAM3=ON
atd-shell ros2 pkg executables atd_orbslam3
```

以後のコンテナ起動は `./scripts/dev.sh --orbslam3`。次の `atd replay` は Notebook で実行する。例の `day1/mapping` と `day2/lap` は、手元の `/data/record/rosbags` からの相対パスに置き換える。

## 地図作成と別走行での定位

Notebook の Web UI「地図」で「rosbag から地図を作成」を開き、ORB-SLAM3、記録データ、新しい地図 ID を選ぶ。作成後は「自己位置検証」でその地図と別走行の bag を選択できる。CLI で同じ試験を実行する例を示す。

```bash
atd replay day1/mapping --backend orbslam3 --save-map course_orb --headless
atd replay day2/lap --backend orbslam3 --map course_orb --start-offset 15 --headless
```

1行目は bag 末尾まで処理してから `/data/results/maps/course_orb/` に新しい地図を公開する。同じ ID への上書きを拒否する。保存物は `orbslam3/atlas.osa`、校正・地図座標変換の `orbslam3/camera.yml`、出典と SHA-256 の `orbslam3/manifest.json`、同じ地図座標系の `hdmap/map.json`。地図は最初に有効な姿勢を得たときの後輪軸中心を基準とする。保存した Atlas の特徴点は同じ変換で `hdmap/cloud.json` に書き出す。Web の地図エディタでこの点群に重ねて境界・走行ラインを作成できる。既存 NVIDIA 地図上で作成したラインを転用するには座標の対応が必要になる。

2行目は別走行の録画開始から15秒の画像で、選択した Atlas のキーフレームと3D地図点を探索する。初期座標と向きの手入力は不要。照合が成立しても、複数の連続する画像で時刻・移動量・回転が整合するまで `/localization/status` の姿勢を採用しない。追跡喪失後も同じ保存地図で再定位する。`--start-offset` を省くと先頭から検証する。短時間の再生で入力を処理しきれないときは `--rate 0.25` 等で倍率を下げる。

`/data/results/runs/orbslam3/<run>/` の状態遷移・軌跡・`summary.json` で初回確定までの時間、追跡した区間を確認する。Gazebo の地図作成時に正解座標との対応を記録できた場合、同一コースの別走行の位置・角度誤差を Notebook UI「VSLAM 軌跡誤差」で確認できる。固定地図の試験では地図作成時の座標変換を再利用する。実車 bag の地図照合・追跡率は、別途計測する実位置に対する誤差とは区別する。

## Jetson での起動

Notebook の転送画面で地図全体を Jetson に転送し、Jetson ホストで手動走行の観測を起動する。

```bash
./scripts/bringup.sh localization map_id:=course_orb
```

地図を省略すると一覧から選べる。ORB 地図を選ぶと、launch が Atlas・校正・背景点群の hash を照合して `orbslam3_node` を起動する。公開する姿勢は画像の時刻における `map` 上の `base_link`。姿勢確定前・喪失中・入力途絶時は走行に使用できる姿勢を出さない。Web UI「自己位置検証」でもこの地図と別の bag を選択できる。

## IMU を追加して比較する

```bash
atd replay day2/lap --backend orbslam3 --headless
atd replay day2/lap --backend orbslam3 --tracking vio --imu-calibration /path/to/imu-noise.yaml --headless
```

ステレオ＋IMU は、新規セッションでの追跡比較に実装した。固定 Atlas の作成・再定位にはステレオ単独を使う。VIO には同じ撮影時刻系の統合 IMU `/camera/camera/imu` と、カメラ・IMU 間の固定 TF が必要。D455 は既定で加速度・ジャイロを別々に配信するため、収録時に統合出力も準備する。IMU 校正 YAML の最上位に `gyro_noise_density`、`accel_noise_density`、`gyro_random_walk`、`accel_random_walk` を記載する。各値には ORB-SLAM3 の連続時間 IMU モデルの SI 単位を用いる。カメラの外部姿勢とステレオ基線は実測 TF と投影行列で照合する。

## 固定地図で途絶を補完し、再取得時の段差を見る

既存の `odaiba_orb_01` と、これまで定位に使ったお台場 3 m/s・2 周の走行を等速で再生する例。更新した checkout と ORB 対応コンテナ内で実行する。今回の変更は ROS パッケージの再ビルドで反映される。

```bash
atd-build --packages-up-to atd_orbslam3 atd_bringup --cmake-args -DATD_REQUIRE_ORBSLAM3=ON
bag=gazebo-odaiba-2026-11-23_01-07-25_640x480-90Hz_imu-400Hz_2laps_3mps
atd replay "$bag" --backend orbslam3 --map odaiba_orb_01 --rate 1 --gyro-bridge on
```

RViz の橙は地図で確認した姿勢、緑は補完を含む姿勢。`--headless` を加えると描画を省いて末尾で終了する。`--gyro-bridge off` で画像観測だけの試験に切り替えられる。既定の `auto` は収録した統合 IMU、または D455 の `/camera/camera/gyro/sample` と固定 TF があれば補完を有効にする。`on` はジャイロを必須とする。

直前 0.20 秒の同じ追跡区間にある 2 点以上の確定姿勢から、前後方向の速度を推定する。途絶中はこの速度を保ち、固定 TF で車体の上向き軸に射影したジャイロで平面位置とヨーを更新する。高さ・ロール・ピッチは最後の観測値を使う。予測期限は最後の観測から 2 秒、角速度を保持できる時間は 20 ms。画像自体が止まった場合も、ジャイロ時刻から 0.25 秒の画像処理待ちを取って更新する。再取得時には地図で確認した姿勢に戻る。

再生終了時に端末へ段差の中央値・最大値を表示する。結果は `/data/results/runs/orbslam3/<RUN>/` に保存する。

| ファイル | 見る値 |
| --- | --- |
| `gyro_bridge_summary.json` | 予測の出力数、再取得回数、段差を計測できた回数、予測できなかった理由 |
| `reacquisitions.csv` | `gap_s` は最後の観測から再取得までの秒数。`position_jump_m`・`abs_yaw_jump_deg` は**再取得と同時刻の予測**と画像観測の平面距離・方位差。速度履歴や IMU が足りない回も理由を残す |
| `reacquisitions.csv` の `display_step_m`・`display_step_abs_yaw_deg` | 直前の配信姿勢から再取得姿勢までの変位と回転。2 つの時刻の間に車体が動いた分も含む |
| `gyro_trajectory.csv` | 配信した姿勢、`source`（`observed`・`predicted`）、最後の画像観測からの秒数 `age_s` |

ROS 2 では `/orbslam3/continuous_odometry` に姿勢、対応する `/orbslam3/continuous_status` に観測・予測・予測不能の区別と経過時間が出る。Jetson で手動走行中に観測する場合は `./scripts/bringup.sh localization map_id:=odaiba_orb_01 orb_gyro_bridge:=true` を使う。単独ジャイロが既定。統合 IMU を配信する設定では `orb_gyro_topic:=/camera/camera/imu` を加える。走行制御が参照する `/localization/status` は画像観測の確定状態を返す。

## cuVSLAM と ORB-SLAM3 の比較

### お台場で 1 m/s の地図を作り、3 m/s で3方式を比べる

地図作成と評価には別々の bag を使う。下の例では評価走行を2周とした。両方とも片眼 640×480・90 Hz、ジャイロと統合 IMU は 400 Hz、加速度は 200 Hz。

1. Linux Notebook **ホスト**で地図作成用の1周を記録する。

   ```bash
   atd gazebo bag --course odaiba --speed 1 --laps 1 --resolution 640x480 --camera-hz 90 --imu 400
   ```

2. Notebook の **ORB 対応コンテナ**内で1周の bag から固定地図を作る。`MAP_BAG` は収録時に表示された `rosbags/` 以下の bag 名。ORB は Atlas を保存する。同じ bag を Web UI の「地図とライン」→「rosbag から地図を作成」で選び、cuVSLAM 地図を別 ID の `odaiba_s1_cu` として作成する。続いて「VGL地図を作成」で 640×480 対応の export を選び、Notebook 用 engine と VGL 地図の準備を完了する。

   ```bash
   read -rp '地図作成用の bag 名: ' MAP_BAG
   atd replay "$MAP_BAG" --backend orbslam3 --save-map odaiba_s1_orb --rate 0.5 --headless
   ```

3. Notebook **ホスト**で評価走行を記録する。

   ```bash
   atd gazebo bag --course odaiba --speed 3 --laps 2 --resolution 640x480 --camera-hz 90 --imu 400
   ```

4. 同じコンテナでテスト bag を等速で2回、順に再生する。1回目で画像で確定した ORB 姿勢とジャイロ補完後の姿勢の両方が残る。2回目は保存済み NVIDIA 地図に対する cuVSLAM のステレオ定位。比較では ORB を `--tracking vo` で動かし、IMU を途絶時の補完に使う。

   ```bash
   read -rp '評価用の bag 名: ' TEST_BAG
   atd replay "$TEST_BAG" --backend orbslam3 --map odaiba_s1_orb --gyro-bridge on --rate 1 --headless
   atd-shell ros2 run atd_localization validate_bag --map odaiba_s1_cu --bag "$TEST_BAG" --rate 1
   ```

5. 端末に表示された run ID を入力して、3方式の結果を保存する。ORB 側には `runs/orbslam3/` の ID を使う。

   ```bash
   read -rp 'cuVSLAM の runs/localization/ の ID: ' CU_RUN
   read -rp 'ORB の runs/orbslam3/ の ID: ' ORB_RUN
   atd replay --compare "$CU_RUN" "$ORB_RUN" --record-root /data/record/rosbags
   ```

結果は `runs/vslam_benchmark/<RUN>/report.json`。`output_time_fraction` は、隣り合う出力の間隔が 0.25 秒以下だった時間の合計を共通の評価区間の長さで割った値。3方式で同じ定義を使う。位置・方位は両バックエンドの最初の共通観測で Gazebo 正解に合わせる。ジャイロ補完には ORB と同じ座標変換を使う。`paired_accuracy` で画像を追跡できた共通時刻の精度を、`orbslam3_gyro.predicted_accuracy` で補完中の出力だけの対 GT 誤差を確認する。再取得の段差とその直前・直後の値は ORB run の `reacquisitions.csv` に残る。

### 画像観測のみの2方式を比較する

**保存地図への定位**を選定する場合は、同じ地図作成用 bag から作った NVIDIA 地図（VGL 準備済み）と ORB の Atlas を用意する。次の例で `cu_map` はその NVIDIA 地図 ID、`bag` は上記の評価用 bag。各コマンドが表示した `runs/localization/` の run ID を比較に渡す。

```bash
atd-shell ros2 run atd_localization validate_bag --map "$cu_map" --bag "$bag" --rate 1
atd-shell ros2 run atd_localization validate_bag --map odaiba_orb_01 --bag "$bag" --rate 1
atd replay --compare "$cu_run" "$orb_run" --record-root /data/record/rosbags
```

比較結果は `/data/results/runs/vslam_benchmark/<RUN>/report.json`。同じ評価区間で、初回観測までの時間、追跡カバー率、0.25 秒を超えた途絶、最長の観測間隔、再取得回数を比較する。両方式とも画像で確認した姿勢を用いる。`--record-root` で Gazebo 正解を指定すると、最初の共通観測で各地図を正解に合わせ、それ以降の共通観測で位置 RMSE・中央値・95 パーセンタイルと方位誤差を算出する。この相対精度には初回定位の絶対誤差は含まれない。実車など正解軌跡がない bag では `--record-root` を省略する。

**新規地図でのステレオ追跡**を比較したいときは、同じ Notebook で次の 2 本を順に実行する。比較に渡すのは各回の `runs/replay/` の ID。描画の有無と再生倍率を揃える。

```bash
atd replay "$bag" --backend cuvslam --rate 1 --headless
atd replay "$bag" --backend orbslam3 --rate 1 --headless
atd replay --compare "$cu_replay_run" "$orb_replay_run"
```

## 収録済みジャイロで姿勢が途切れた区間を調べる

Mac で開発・ネイティブ試験を行い、実データの評価は後から SSH で Linux Notebook に入り、既存の ATD コンテナで実行する。引数は Gazebo の正解軌跡と照合した固定地図 run の ID。対応する比較 run と bag は保存済みの metadata から特定する。bag を移動した場合は `--bag DIR` で指定できる。

```bash
atd-build --packages-select atd_bringup
atd replay --imu-gaps 20261125T154149Z-8963ca5d239f
```

記録した姿勢の間隔が 0.25 秒を超える区間について、位置保持・等速直進・定速旋回・収録ジャイロの4方式を同じ終点の Gazebo 正解と比較する。速度は途絶前 0.20 秒以内の同一追跡区間にある2姿勢以上から推定する。ジャイロ方式では `/tf_static` で計測角速度を車体の上向き軸に投影し、計測時刻順に旋回を積分する。前進速度は途絶開始時の推定値を使用する。ジャイロを 20 ms より長く保持する必要がある区間や、姿勢履歴・時刻対応が不足した区間は比較から除き、理由を記録する。

端末には4方式で比較できた区間数、終点の位置誤差の中央値、最長途絶での結果を表示する。各区間の位置・方位誤差は `/data/results/runs/orbslam3/<run-id>/imu_gap_report.json` に保存する。これは平面走行を想定したセンサー時刻ベースのオフライン評価である。

## 検証結果

macOS arm64 のネイティブ ROS 2 Lyrical で、奥行きの異なる模様を投影した 640×480、20 Hz の合成 MCAP を使った。100 組のステレオ画像で地図を作成し、Web API から同じ地図座標の特徴点 870 点を読み出せた。撮影開始位置と経路を変えた別 bag を 0.4 秒地点から再生したところ、26 組を処理し、3 組目で初期位置が確定した。地図作成時の座標変換を固定して既知の軌跡と比較した平面位置の RMS 誤差は 24 姿勢で 4.5 mm。0.10 秒という確定時間は画像のタイムスタンプ間隔で、語彙・地図の読み込み時間は含まない。

別の合成試験で、未観測の風景・無地画像での喪失と別地点での再定位、ステレオ 25 Hz ＋ IMU 200 Hz の初期化と姿勢出力も確認した。実機 D455、Jetson での処理速度、cuVSLAM との実走行精度比較は今後の検証項目。

Linux Notebook の Odaiba 90 Hz・2周の bag ではステレオ地図を549点で初期化し、再生約8.1秒で `SO3::exp failed! omega: -nan -nan -nan` により停止した。上流の `Sim3Solver` では、回転を伴わない平行移動でゼロ除算が起き、同じ例外を再現できた。ATD 版では推定した単位クォータニオンから回転行列を直接求め、回転を一意に決められない点の組は棄却する。

修正後の macOS ネイティブ試験では、正常な変換の推定と不正な対応点の棄却を確認した。合成ステレオ画像で作成した地図を別プロセスで読み込み、2か所で各8姿勢の再定位にも成功した。元の Odaiba bag での完走は未確認。[Sim3 の回帰試験](../docker/orbslam3/test/sim3.cpp)

Linux Notebook では、お台場を 3 m/s で1周した 90 Hz のステレオ bag から `odaiba_orb_01` を保存できた。別走行の同速度・2周を等速再生した固定地図試験（`20261125T154149Z-8963ca5d239f`、連続追跡の修正前）は完走した。初回確定は入力時刻で 0.033 秒。Gazebo 正解と照合できた 1,563 姿勢の平面位置誤差は中央値 8.95 cm、95 パーセンタイル 20.79 cm、最大 1.20 m だった。一方、追跡確定から喪失への遷移は172回あり、姿勢出力間隔が 0.25 秒を超えた区間は24回、最長 1.78 秒だった。出力のある時刻での位置誤差と、出力途絶中に進んだ距離は別々に評価する。

固定地図追跡では、前画像で確認した地図点を撮影時刻に基づく予測姿勢で次の画像に投影する。照合できなければ参照キーフレーム、次に地図全体から姿勢を求め、各画像で局所地図との整合を1回検査する。再定位後は ORB 内部の速度予測を初期化する。上記の保存地図をそのまま使って変更前後を比較できる。

上記 24 区間のうち直前に速度を推定できた 21 区間で、Linux Notebook の収録ジャイロによるオフライン評価を行った。途絶終点での Gazebo 正解に対する位置誤差の中央値は、位置保持 1.519 m、等速直進 0.882 m、定速旋回 0.325 m、収録ジャイロ 0.089 m。最長の 1.782 秒の区間でジャイロ使用時の終点誤差は 0.059 m だった。オンラインでの再取得段差は地図の観測姿勢との比較で別途計測する。

オンライン補完は macOS ネイティブの ROS 2 Lyrical で、保存した合成 Atlas とステレオ 34 組・合成ジャイロ 400 Hz の MCAP を使って確認した。8 組の左右画像を遮蔽し、通常の headless replay で末尾まで処理した。画像で確認できた 22 姿勢に加えて 10 姿勢を補完し、0.55 秒後の再取得で同時刻の位置差 2.59 cm・方位差 3.77° を記録した。お台場の収録 bag でのオンライン段差と、cuVSLAM との実測比較は Notebook で評価する。

ATD 版は headless で実行する。固定地図モードでは Atlas の単一地図を選び、地図更新を停止して追跡・再定位する。[公式リポジトリの使用リビジョン](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4)・[上流への変更](../docker/orbslam3/atd-orbslam3.patch)・[地図保存・再定位の再現試験](../docker/orbslam3/test/fixed_map.cpp)
