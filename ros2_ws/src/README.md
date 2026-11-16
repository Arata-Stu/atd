# ROS 2 パッケージ

自作パッケージを `ros2_ws/src/atd_<機能名>/` に配置する。現在、走行用パッケージは未作成。
パッケージごとに `package.xml` と `CMakeLists.txt` を持ち、C++ と `ament_cmake_auto` を基本とする。
そのパッケージの `include/`・`src/`・`launch/`・`config/`・`test/` はパッケージ内に置く。

ホストから編集する対象は、このリポジトリの `ros2_ws/src/`。
別の ROS workspace を選択した場合、Docker ではこのディレクトリを workspace の `src/atd/` に読書き可能で mount する。
そのホスト側 `src/atd/` は空の mount 先として扱い、ソースを手動コピーしない。
この `ros2_ws/` 自体を workspace に選んだ場合は `src/` をそのまま使う。

配置の詳細は [プロジェクト構成](../../docs/project_structure.md)、操作は [セットアップ](../../docs/setup/README.md)を参照する。
