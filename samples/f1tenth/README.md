# F1 コースの RC 用 HDMap サンプル

`maps/<id>/` は ATD の手動 HDMap と同じ形式。左右の境界、生成済みの Centerline と Raceline を保持する。各地図の `source` に上流ファイル名・固定コミット・SHA-256・元の周長・変形量を記録した。全23会場と採否は [catalog.json](catalog.json) にある。

## 形状と縮尺

入力は F1TENTH の各コースの `*_centerline.csv`。上流で実コースを RC 用に縮小済みで、道路幅は 2.2 m。採用した20コースの元 CSV の周長は約 285～554 m ある。ATD の既定 0.10 m / 800 点では約 80 m が Centerline の上限なので、1/10 車両が走るコンパクトなレイアウトへ変換した。このサンプルと原寸サーキット、元の F1TENTH マップのラップタイムは直接比較できない。

- 元の XY の縦横比と進行順を保って縮小する。初期の上限を周長 72 m・境界追加前の外接矩形 300 m² とし、既定の領域格子と点数に収める。
- 小さいヘアピンや近接区間は、縮小した入力線からの変位を抑えつつ、曲がりの強さ・辺長と隣り合わない区間の間隔を調整する。Shanghai の複合コーナーは、元の点列を近似計算の初期値にして接続順を保つ。最終検査で曲率に余裕が必要だった5コースは設計中心を 7.5%、Shanghai は 4% 拡大してから境界を作った。
- 設計中心の左右 0.38 m に境界を置き、最大 6 mm の許容差で編集点を整理する。設計道路幅は全コース共通の 0.76 m。ポリライン化に伴い、個々の地点の境界間距離には微小な差がある。
- 元の CSV を同比率に縮小した折れ線と設計中心の最大形状距離を `reference_hausdorff_m` に、対応位置での変位 RMS・最大変位・その座標を `hdmap/map.json` の `source` に収録した。これらは RC 設計時の変形の指標である。
- 最終境界から通常の ATD の `region` で中心を抽出し、保存した中心線から `mincurv` で Raceline を生成した。ライン生成設定は全コースで共通の既定値。走行領域・車体余裕・曲率の検査も通常の生成経路で行う。

## 再生成

境界編集とラインの再生成は、Notebook の Web UI に編集用コピーを作成して「地図とライン」で行う。元 CSV からサンプル全体を再構築する場合は、`atd-web prepare` 済みの Notebook Docker のシェルで以下を実行する。

```bash
cd /workspaces/atd
atd_env_dir="${ISAAC_ROS_WS}/.atd-python/$(uname -m)"
ATD_RACELINE_PYTHON="${atd_env_dir}/raceline/bin/python" \
  "${atd_env_dir}/web/bin/python" samples/f1tenth/regenerate.py \
  --output /data/results/runs/f1tenth-regenerated --course spa
```

`--course` には [採用一覧](catalog.json) の ID を指定する（例 `spa`、`monza`）。省略すると20コースすべてを生成する。出力先には新しいディレクトリを選ぶ。コースごとに全ラインが生成できた時点で保存し、同名の既存地図は保護する。F1TENTH の元 CSV と SHA-256 を同梱しているため、準備後はオフラインで再生成できる。

## Python で別の操舵方式を比較する

Notebook の Web 用 Python 環境で、名前とファクトリの辞書を `evaluate_course` に渡せる。全方式に同じ前方軌道、FF＋PID、車体寸法、試行開始条件、衝突判定を適用する。以下は Austin で標準の2方式と、固定先読み 0.85 m の方式を比較する例。

```python
import json
import sys
from functools import partial
from pathlib import Path

root = Path("/workspaces/atd")
sys.path[:0] = [str(root / path) for path in (
    "apps/web/processing", "ros2_ws/src/atd_planning", "ros2_ws/src/atd_control",
)]
from atd_control.core import LookaheadConfig, pure_pursuit
from atd_processing.benchmark import default_controllers, evaluate_course

document = json.loads((root / "samples/f1tenth/maps/austin/hdmap/map.json").read_text())
settings = {"steering_delay_ms": 50}
controllers = default_controllers(settings)
controllers["fixed_0.85m"] = lambda: partial(pure_pursuit, config=LookaheadConfig(.85, 0., .85))
report = evaluate_course(document, settings, controllers=controllers)
for method in report["results"]:
    print(method["mode"], method["finish_rate"], method["mean_lap_time_s"], method["collisions"])
```

ファクトリは引数なしで呼ばれ、各試行専用の関数または callable インスタンスを返す。制御関数の引数は `steer(trajectory, *, x, y, yaw, speed_mps, wheelbase_m)`。軌道と後輪軸位置は map 座標の m、向きは rad、車速は m/s。成功時は `{"valid": True, "steering_rad": angle}`（左が正の等価前輪舵角）、計算不能時は `{"valid": False, "reason": "理由"}` を返す。計算不能の試行も完走率の分母に入る。

## 出典とライセンス

[sources/](sources/) の入力 CSV は F1TENTH の固定リビジョンから抽出したもの。上流の著作物とそれに由来する地図は GNU GPL v3 の条件に従う。[F1TENTH Racetracks](https://github.com/f1tenth/f1tenth_racetracks/tree/b95c4eff766f6367d66b310ea20cd2c9563712c0)・[ライセンス全文](LICENSE)

F1TENTH は Alexander Heilmeier / TUM Institute of Automotive Technology の [racetrack-database](https://github.com/TUMFTM/racetrack-database) をもとにしている。元の中心線は © OpenStreetMap contributors。[OpenStreetMap の著作権表示](https://www.openstreetmap.org/copyright)

上流の関連文献は Betz et al., *Autonomous Vehicles on the Edge: A Survey on Autonomous Vehicle Racing*, IEEE Open Journal of Intelligent Transportation Systems 3, 458–488 (2022)。[DOI](https://doi.org/10.1109/OJITS.2022.3181510)
