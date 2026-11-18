# atd_vehicle_description

車両の実測寸法とカメラ取付位置を YAML v1 で保存し、起動時だけ static TF を配信する C++20 component / standalone node。`base_link` は後輪車軸中心、x前・y左・z上。`rear_axle` は同じ原点の別名として常に identity、`front_axle` は wheelbase 入力時だけ前方へ配信する。`camera_link` は明示した完全なxyz/RPYと enabled=true のときだけ配信する。RealSense の内部・optical TFは公式driverに任せる。

保存先は `/data/results/runs/vehicle/layout.yaml`。寸法は m、角度はrad。未測定値はnull、sourceはunmeasured / cad_candidate / manual。CAD参考値を実車の測定値として自動採用しない。body_heightは路面からの全高で、後輪車軸の路面からの高さ・最低地上高を表さず、TFのz位置には使わない。

`ros2 launch atd_vehicle_description vehicle.launch.py`。引数はconfig / profile、frame IDsは固定read_only。保存後はこのnodeを終了・再起動して反映する。TFの動的書換・再読込service・車両制御は持たない。`/vehicle/layout/status` は10HzのYAML Stringで、active_profile・profile_path・profile_valid・tf_published・published_frames・instance_idとstatusの元stampを配信する。tf_stampは起動時に送ったTFの時刻を保持する。

ホストのC++純粋計算とPython schema・永続化、bridge/APIのfake検証と、ROS build・static TF結合・カメラ実機・実車寸法の確認は分けて扱う。
