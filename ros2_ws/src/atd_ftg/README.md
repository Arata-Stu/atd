# atd_ftg

Depth の3D点と円弧に沿った車体箱を使い、地図なしで通過可能な進路を選ぶ。Gazebo の `ftg-3d` はこの ROS 2 ノードを自動起動する。Gazebo の Depth を ROS へ渡し、ノードの出力を模擬車両モデルへ戻す。D455 でも同じノードを使う。

## ROS 2 ノード

コンテナ内でビルドし、インストール済みの環境を読み込む。

```bash
atd-build --packages-up-to atd_ftg
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 run atd_ftg depth_ftg --ros-args \
  -r '~/depth:=/camera/camera/depth/image_rect_raw' \
  -r '~/camera_info:=/camera/camera/depth/camera_info' \
  -r '~/odometry:=/localization/rpm_imu/odom'
```

D455 の Depth を有効にし、カメラの optical frame から後輪軸中央の `base_link` への TF と、現在の車速を含むオドメトリを配信する。例の RPM/IMU オドメトリは初回の回転検出後に配信を開始するため、静止から評価する場合は停止中も新しい車速ゼロを配信する入力を指定する。bag を使う場合は `-p use_sim_time:=true` と再生側の `/clock` を使う。

| トピック | 型と内容 |
| --- | --- |
| `~/depth` | `sensor_msgs/msg/Image`。整列済み Depth の `16UC1` (mm) または `32FC1` (m) |
| `~/camera_info` | `sensor_msgs/msg/CameraInfo`。画像と同じ光学フレーム、解像度の投影行列 |
| `~/odometry` | `nav_msgs/msg/Odometry`。`child_frame_id` は車体フレーム。前進速度とヨーレートを使用 |
| `~/target_twist` | `geometry_msgs/msg/TwistStamped`。車体座標の目標前進速度と目標ヨーレート。既定名は `/depth_ftg/target_twist` |
| `~/status` | `std_msgs/msg/String`。FTG の状態・入力時刻・通過距離などの診断 JSON |

制御目標は平坦路面を前提とする。Depth またはオドメトリが未受信・150 ms超、TFが使えない場合や候補がない場合はゼロ速度を配信する。D455 の欠測値は未知として扱う。

起動時パラメータ `wheelbase`、`vehicle_length`、`vehicle_width`、`rear_axle_to_rear`、`rear_axle_height`、`body_height` は m、`steering_limit` は rad、`speed_limit` は m/s、`speed_tau` は s、`deceleration` は m/s²。既定値は Gazebo の仮定値。実車の評価では実測の寸法と減速特性を指定する。現時点のノードは目標を専用トピックに配信し、車両の操作権や ESC 出力には接続していない。
