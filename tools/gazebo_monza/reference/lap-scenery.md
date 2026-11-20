# モンツァ全周の沿道景観

全周モードでは第1シケインに加え、ピット、沿道、遠景の配置を読み込む。地図にある地物は2026年11月20日に取得した WGS84 座標を走路と同じ東・北座標、1/10 縮尺へ変換した。沿道データの `placement` で OSM の線に沿う区間（`mapped`）と補った区間（`estimated`）を確認できる。データと派生地理データは **© OpenStreetMap contributors, ODbL 1.0**。[区間の地物](monza_lap_scenery.geojson)・[沿道設備](monza_trackside.geojson)・[遠景](monza_background.geojson)・[取得範囲](https://api.openstreetmap.org/api/0.6/map?bbox=9.2775,45.6095,9.3000,45.6340)・[帰属・ライセンス](https://www.openstreetmap.org/copyright)

| 地物 | 実データに対応する部分 | シミュレーター用に補った部分 |
|---|---|---|
| 観客席 | GP走路から実距離120 m以内の `building=grandstand` の外周。全41棟のうち35棟を追加。Centrale、Laterale Sinistra、Laterale Destra、Parabolica、Vedano の5棟は主催者資料の同名の屋根付き席に対応させた。 | 長辺のうち走路に近い側へ向く段床、支柱、筋交い。屋根の範囲と高さも推定。小区画の名称と個別の席番号には未同定のものがある。 |
| ピット施設 | Palazzina Box と Hospitality Building の外周、ピットレーンの中心線、ピットウォールの位置と高さ。 | 高さ 1.25 RC m の建物にガレージ開口、窓枠、ガラス帯、庇を設けた。ピットレーン幅や壁の緑色は推定。[ガレージ棟](https://www.openstreetmap.org/way/34404652/history/7)・[ピットレーン](https://www.openstreetmap.org/way/38168747/history/12)・[ピットウォール](https://www.openstreetmap.org/way/243090150) |
| グリッド | OSM のスタート信号位置と本線の向き、FIA 資料にあるスタートとコントロールラインの間隔309 m（RC 30.9 m）。 | 千鳥の22枠、停止位置ガイド、舗装補修の形と配置を写真に合わせて近似した。枠は周回開始点の手前にあり、1周の終わりに見える。[FIA Italian GP 競技資料 p.4](https://www.fia.com/system/files/decision-document/2026_italian_grand_prix_-_competition_visa.pdf) |
| 横断施設 | 旧オーバルと2本の歩道橋の平面位置を OSM の高架線から取得した。周回の始点から約330、413、442 RC m で本線と交差する。 | 橋の幅、高さ、橋脚、桁の断面を近似した。歩道橋のうち1本は地図の屋根タグを反映した。[旧オーバル](https://www.openstreetmap.org/way/34404729)・[屋根付き歩道橋](https://www.openstreetmap.org/way/194458113)・[もう1本の歩道橋](https://www.openstreetmap.org/way/194458112) |
| レール・フェンス・壁 | OSM の地物を使い、欠けた区間では本線・分岐・砂地の外形に沿って部分的に補った。 | レールの折れ曲がった断面、支柱、壁のパネル継ぎ目、表面の退色、最終右コーナー内側の縁石を近似した。参照写真の撮影地点や各支柱の実位置までは同定していない。 |
| 森林 | GP走路から実距離100 m以内にある `landuse=forest` / `natural=wood` の12地物を追加。地図上の内周、建物・走路・砂地、第1シケインで使用済みの領域を除き、林縁を収録。 | 各林域内の幹位置・樹種・樹高・密度は描画用。追加域は 2.55 m 四方（RC縮尺）の格子あたり最大1本に間引き、地物IDで配置を固定する。 |
| 砂地 | `natural=sand` の8区域を追加。第1シケインとの合計10区域。Lesmo、Ascari、Parabolica にも砂地が現れる。 | 粒度・深さ・表面の細かな起伏は未計測。新旧の地物が近接する場所では走路を上に描画する。 |
| 遠景 | 取得範囲内の建物112棟と18の森林地物を追加。森林の内周・建物・既存の林を除き、両眼とも遠クリップを RC 100 m とした。 | 遠方の林は走路から10–35 RC m を4.5 m格子、35–100 RC m を7 m格子で間引く。建物は階数のある2棟を除いて高さを実寸6 m相当と仮定した。[遠景データの再生成](../prepare_monza_background.py) |

スタート灯は OSM の信号位置に設け、グリッドから見上げる向きに並べた。灯器と支持体の形・高さは会場写真からの近似。観客席の屋根付き分類は公式図の凡例で照合した。路面・地形の高低差は平面として扱っている。[スタート信号](https://www.openstreetmap.org/node/1828499259/history/4)・[モンツァ公式の会場写真](https://www.monzanet.it/wp-content/uploads/2025/07/Monza_Hospitality_GP_2025.pdf)・[2026年モンツァ公式観客席図](https://www.monzanet.it/wp-content/uploads/2026/03/GP_F1_2026_LISTINO_PREZZI_3_2.pdf)
