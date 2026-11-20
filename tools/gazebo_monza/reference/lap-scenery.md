# モンツァ全周の沿道景観

全周モードでは[第1シケインの景観](README.md)に [monza_lap_scenery.geojson](monza_lap_scenery.geojson) を加える。全地物に元の OSM ID、版、編集日時、識別に使ったタグがある。2026年11月20日に取得した WGS84 座標を走路と同じ東・北座標、1/10 縮尺へ変換する。データと派生地理データは **© OpenStreetMap contributors, ODbL 1.0**。[取得範囲](https://api.openstreetmap.org/api/0.6/map?bbox=9.2775,45.6095,9.3000,45.6340)・[OpenStreetMap の帰属](https://www.openstreetmap.org/copyright)・[ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/)

| 地物 | 実データに対応する部分 | シミュレーター用に補った部分 |
|---|---|---|
| 観客席 | GP走路から実距離120 m以内の `building=grandstand` の外周。全41棟のうち35棟を追加。Centrale、Laterale Sinistra、Laterale Destra、Parabolica、Vedano の5棟は主催者資料の同名の屋根付き席に対応させた。 | 長辺のうち走路に近い側へ向く段床、支柱、筋交い。屋根の範囲と高さも推定。小区画の名称と個別の席番号には未同定のものがある。 |
| ピット施設 | メインストレート沿いの [Palazzina Box](https://www.openstreetmap.org/way/34404652/history/7) と [Hospitality Building](https://www.openstreetmap.org/way/194458105/history/5) の平面外周。 | 高さをモデル上 1.25 m とした平屋根の外形。階数、窓、ピット開口、塗装は未再現。 |
| 森林 | GP走路から実距離100 m以内にある `landuse=forest` / `natural=wood` の12地物を追加。地図上の内周、建物・走路・砂地、第1シケインで使用済みの領域を除き、林縁を収録。 | 各林域内の幹位置・樹種・樹高・密度は描画用。追加域は 2.55 m 四方（RC縮尺）の格子あたり最大1本に間引き、地物IDで配置を固定する。 |
| 砂地 | `natural=sand` の8区域を追加。第1シケインとの合計10区域。Lesmo、Ascari、Parabolica にも砂地が現れる。 | 粒度・深さ・表面の細かな起伏は未計測。新旧の地物が近接する場所では走路を上に描画する。 |

観客席の屋根付き分類は公式図の凡例で照合した。仮設の観客席、フェンス、広告、橋、路面・地形の高低差も年・区間で変わる。画像上の目印の位置を定性的に比較できるが、現地の特定の日の映像と画素単位で照合したものではない。[2026年モンツァ公式観客席図および凡例](https://www.monzanet.it/wp-content/uploads/2026/03/GP_F1_2026_LISTINO_PREZZI_3_2.pdf)
