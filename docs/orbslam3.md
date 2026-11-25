# ORB-SLAM3 のステレオ地図と初期自己位置推定

実験ブランチは `experiment/orbslam3-localization`。既定の `atd replay` は cuVSLAM を使う。ORB-SLAM3 を選ぶと、D455 または Gazebo の左右の rectified IR・CameraInfo と `/tf_static` からステレオ追跡できる。地図作成は Notebook、実車の初期定位・追跡は Jetson で実行する。

## Linux Notebook・Jetson の準備

各機体の checkout で対象ブランチを選択した後、ホストから専用イメージを構築する。`--orbslam3` は上流リビジョンを固定した CPU 版・ATD の固定地図用変更・ORB 語彙を追加する。上流のプログラムと語彙はビルド時に取得する。実行中の開発コンテナを再利用するときは、ORB 対応イメージで起動したものを使う。

```bash
./scripts/dev.sh --build --orbslam3
```

起動したコンテナ内で ROS パッケージをビルドする。この指定では ORB ライブラリが見つからなければビルドに失敗する。通常の cuVSLAM イメージでは追加ノードのビルドを省略できる。

```bash
atd-build --packages-up-to atd_orbslam3 atd_bringup --cmake-args -DATD_REQUIRE_ORBSLAM3=ON
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

## 検証結果

macOS arm64 のネイティブ ROS 2 Lyrical で、奥行きの異なる模様を投影した 640×480、20 Hz の合成 MCAP を使った。100 組のステレオ画像で地図を作成し、Web API から同じ地図座標の特徴点 870 点を読み出せた。撮影開始位置と経路を変えた別 bag を 0.4 秒地点から再生したところ、26 組を処理し、3 組目で初期位置が確定した。地図作成時の座標変換を固定して既知の軌跡と比較した平面位置の RMS 誤差は 24 姿勢で 4.5 mm。0.10 秒という確定時間は画像のタイムスタンプ間隔で、語彙・地図の読み込み時間は含まない。

別の合成試験で、未観測の風景・無地画像での喪失と別地点での再定位、ステレオ 25 Hz ＋ IMU 200 Hz の初期化と姿勢出力も確認した。Linux コンテナのビルド、実機 D455、Jetson での処理速度、cuVSLAM との実走行精度比較は今後の検証項目。

ATD 版は headless で実行する。固定地図モードでは Atlas の単一地図を選び、地図更新を停止して追跡・再定位する。[公式リポジトリの使用リビジョン](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4)・[上流への変更](../docker/orbslam3/atd-orbslam3.patch)・[地図保存・再定位の再現試験](../docker/orbslam3/test/fixed_map.cpp)
