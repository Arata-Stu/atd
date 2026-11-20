# Monza の屋外コース材質

走路とコース外の芝には JPEG を使う。画像は checkout に同梱し、シミュレーションはオフラインで実行できる。

| 画像 | 用途と加工 | シーンでの繰り返し幅 |
|---|---|---|
| `monza_asphalt.jpg` | Monza 全周の本線・ピット・分岐走路用の灰色舗装。2048 × 2048 の材質に細かな骨材と弱い色むらを焼き込んだ。 | 3.20 m |
| `floor.jpg` | 第1シケイン単体・他コースの乾いた黒い舗装。元写真の粒の輝度差を抑えた。 | 0.80 m |
| `grass.jpg` | 短く刈った芝。葉の明暗を残し、色調を黄緑に寄せた。 | 1.00 m |

舗装には eye-candy.xyz による 1K の Diffuse 写真を使った。グレースケール化して 0.45 px のガウシアンぼかしをかけ、8 bit 輝度を `59 + 0.42 × (輝度 − 130)` に調整した。[Poly Haven Asphalt Floor](https://polyhaven.com/a/asphalt_floor)・[CC0 ライセンス](https://polyhaven.com/license)

Monza 全周の材質はこの `floor.jpg` から作った。骨材の縮尺を維持し、写真の部分平均から求めた濃淡を広い範囲に重ね、8 bit 輝度の中央値をおよそ 117 にした。路面に固定されるので左右のカメラが同じ場所の模様を観測する。色むらの形・配置は近似で、舗装の測量値ではない。再生成する場合は NumPy と Pillow のある環境で `python3 tools/gazebo_monza/prepare_monza_asphalt.py` を実行する。

芝には ambientCG の Grass 005（手続き生成とビットマップを組み合わせた素材）を使った。1K の Color マップの R・G・B にそれぞれ 23・26・4 を加え、8 bit の範囲に収めた。[ambientCG Grass 005](https://ambientcg.com/view?id=Grass005)・[CC0 ライセンス](https://docs.ambientcg.com/license/)

Monza 全周のレズモとアスカリでは、地図に記録された5区画の砂利トラップに `runoff_gravel.jpg` を使う。Charlotte Baglioni 撮影、Dario Barresi 処理の Clean Pebbles の 1K Diffuse JPEG を 512 × 512 に縮小した。写真の幅 2 m に対応して RC 上の 0.20 m ごとに貼る。粒の色や並びは代表素材による近似である。[Poly Haven Clean Pebbles](https://polyhaven.com/a/clean_pebbles)・[CC0 ライセンス](https://polyhaven.com/license)

色調と繰り返し幅はこの RC コースのために選んだ。配信する左右画像は可視光の材質をモノクロで描画したもので、実車 D455 での路面や芝の赤外反射率は未計測。

## 樹木

樹木は、このシーン用に ImageGen で生成した写真風の透過素材を使う。各画像は 512 × 512 の RGBA PNG。投影面の幅と高さで自然な樹形に戻るよう、幹の根元を下辺中央へそろえて格納した。樹冠の隙間にも透過を残している。

| 画像 | 生成時に指定した樹形 | 投影面の幅 / 高さ |
|---|---|---:|
| `tree_oak.png` | ヨーロッパナラ。分かれた太い枝と左右非対称に広がる丸い樹冠 | 1.14 |
| `tree_poplar.png` | イタリアポプラ。直立した幹と細長い黄緑色の樹冠 | 0.29 |
| `tree_pine.png` | ヨーロッパアカマツ。赤褐色の幹と斜めに張り出す不規則な枝 | 0.75 |

3種とも初夏の自然光を左上から受けた単独の立木を、幹の根元近くから望遠で正面撮影した構図を指定した。明暗のある葉のまとまりと透過する背景を指定し、生成画像を切り出して縮小した。実際の Monza の木の位置や樹種を測量した素材ではない。

Monza 全周の壁面は、同梱の `wall.jpg`（Rob Tuytel のコンクリート素材）を縮尺に合わせてタイル描画する。壁では水平方向と高さ方向に UV を割り当てる。グリッド付近の補修跡には本線と同じ舗装材質を少し暗く着色して用いる。`floor_normal.png` と `floor_roughness.png` は上記の Asphalt Floor に由来する。[Poly Haven Concrete](https://polyhaven.com/a/concrete)
