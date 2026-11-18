# ROS 2 パッケージ

自作パッケージを `ros2_ws/src/atd_<機能名>/` に配置する。入力・操作要求の経路は `atd_joy` → `atd_teleop` と、Pico監視USBの `atd_vehicle_interface` → `atd_rc_input`。`atd_interfaces` に運転要求・操作イベント・Pico受信型、`atd_web_bridge` に Notebook UI 用の観測・設定接続を置く。基板への走行指令・最終走行許可・記録実行は未実装。
パッケージごとに `package.xml` と `CMakeLists.txt` を持ち、C++ と `ament_cmake_auto` を基本とする。
そのパッケージの `include/`・`src/`・`launch/`・`config/`・`test/` はパッケージ内に置く。

D455 の能力列挙・設定・起動は `atd_sensors`、車体static TFと永続配置設定は `atd_vehicle_description`、複数機能の起動構成は `atd_bringup` が担当する。Jetson で `scripts/build.sh --packages-up-to atd_bringup` 後、`scripts/bringup.sh manual` でD455・車体TF・入力監視をまとめて起動する。`d455` はD455・TFのみ、`input` / `bridge` は従来の入力監視用。profile は実装済みの launch と対応させ、一覧を `scripts/lib/bringup.sh` で管理する。[bringup の操作](../../docs/bringup.md)を参照する。

ホストから編集する対象は、このリポジトリの `ros2_ws/src/`。
別の ROS workspace を選択した場合、Docker ではこのディレクトリを workspace の `src/atd/` に読書き可能で mount する。
そのホスト側 `src/atd/` は空の mount 先として扱い、ソースを手動コピーしない。
この `ros2_ws/` 自体を workspace に選んだ場合は `src/` をそのまま使う。

配置の詳細は [プロジェクト構成](../../docs/project_structure.md)、操作は [セットアップ](../../docs/setup/README.md)を参照する。
