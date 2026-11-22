# お台場の舗装材質

走路とピット前の舗装面に同じ灰色の `asphalt.jpg` を2.4 m間隔で敷く。写真の局所的な明暗を地面の位置に固定し、左右のカメラが同じ模様を観測できるようにしている。素材は同梱しているため実行時のダウンロードは不要。

素材は eye-candy.xyz による 1K の舗装のDiffuse写真から作成した。グレースケール化、0.45 pxのガウシアンぼかし、8 bitの輝度調整を加えた中間画像が `floor.jpg`。NumPyとPillowを使って `python3 tools/gazebo_monza/prepare_asphalt.py` を実行すると、骨材と弱い色むらをもつ2048 × 2048の材質を再生成できる。色合いと粒の大きさは描画用の概算で、現地の赤外反射率は未測定。[Poly Haven Asphalt Floor](https://polyhaven.com/a/asphalt_floor)・[CC0 ライセンス](https://polyhaven.com/license)
