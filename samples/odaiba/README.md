# お台場 SKY TRACK の HDMap サンプル

Gazebo の `odaiba` が舗装面に使う左右の境界を、同じ向き・順序・座標で収録している。1/5 スケールのメートル単位で、原点は計時線の中央、走行開始時は −x 方向。元のコースは公式ガイドから推定した形状。[Gazebo の形状・出典](../../docs/gazebo_circuits.md)

`hdmap/map.json` に境界、region Centerline、mincurv Raceline と速度プロファイルを収録し、`lines/` に各ラインを個別に保存している。Web UI の「シミュレーター」で固定先読みと速度連動の2D走行を比較できる。[2Dベンチマーク](../../docs/simulation.md)

## 再生成

Web UI で編集用の地図を開き、Centerline、Raceline の順で生成できる。元の Gazebo 境界からファイル一式を作り直す場合は、`atd-web prepare` 済みの Notebook 開発コンテナで実行する。

```bash
cd /workspaces/atd
venvs="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}/.atd-python/$(uname -m)"
ATD_RACELINE_PYTHON="$venvs/raceline/bin/python" \
  "$venvs/web/bin/python" samples/odaiba/regenerate.py --output /tmp/atd-hdmap/odaiba
```

`--output` は未作成の出力先を指定する。両ラインに Web UI と同じ既定の生成設定を使い、境界・車体・曲率の検査がすべて通った時点で地図を保存する。既定の速度上限は 1.5 m/s。Gazebo の `--speed` はカメラ台の移動速度で、2Dの速度プロファイルはHDMapの生成設定から決まる。
