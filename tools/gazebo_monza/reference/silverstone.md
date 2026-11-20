# Silverstone Grand Prix Circuit

2026年イギリスGPの主催者公表値は5.891 km。収録した走路の平面長は5,891.24 mで、RC縮尺1/10で589.12 mに相当する。走行方向は時計回り。Hamilton Straight を出発して Abbey、Village、Copse、Maggotts・Becketts、Stowe、Club を通って戻る。[Formula 1 の大会・コース案内](https://www.formula1.com/en/racing/2026/great-britain)・[サーキット公式案内図](https://www.silverstone.co.uk/sites/default/files/pdf/Drive-Silverstone-Joining-Instructions.pdf)

- [走路データ](circuits/silverstone/track.geojson)には OSM の GP リレーション v33 に含まれる27本の走路を収録した。重複メンバーは一度だけたどり、ピットレーンを除いた。Hamilton Straight で `raceway=start/finish` が付いたノード v2 を発着点とした。OSMには個別に `raceway=finish` の地点も存在する。走路幅は Wellington Straight、Brooklands の一部、Luffield に登録されている15 mを共通値とした。他の区間の幅、縁石、勾配の実測データは収録していない。[コースの元リレーション](https://www.openstreetmap.org/relation/51160/history/33)・[発着ノード](https://www.openstreetmap.org/node/13036050130/history/2)
- [沿道の地物](circuits/silverstone/scenery.geojson)には29棟の観客席、砂・小石のランオフ20区画、樹林10区域とピット施設 Silverstone Wing を収録した。観客席の位置と向きは建物外周を基にし、複雑な外形は座席を配置する四隅に近似した。名称を公式案内と対応できた12棟を屋根付きとした。National Pit Straight の部分的な屋根や、個別略称を確認できない臨時席の屋根は省略した。Wing の OSM タグには高さ30 m・地上3階の記載がある。斜めの屋根、ファサード、看板の形は近似対象。[公式観客席ガイド](https://www.silverstone.co.uk/news/silverstones-guide-grandstands)・[Wing の建物外周](https://www.openstreetmap.org/way/227332429/history/8)
- 樹林は走路中心から実距離130 m以内の地図上の範囲を切り出し、舗装、建物、ランオフを除いた。一本ごとの位置・樹種・高さは生成値。草地は平面を使う。フェンスや立体交差する歩道橋の位置・形状、路面高低差は再現していない。

地理データは2026年11月20日取得。各地物に OSM の ID・版・編集日時を収録した。データと派生地理データの帰属は **© OpenStreetMap contributors**、ライセンスは ODbL 1.0。[取得範囲](https://api.openstreetmap.org/api/0.6/map?bbox=-1.0315,52.0600,-1.0030,52.0850)・[OpenStreetMap の帰属](https://www.openstreetmap.org/copyright)・[ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/)

GP リレーションのピットレーンを Silverstone Wing の前に舗装する。中心線は OSM の International pit lane、幅は実寸換算6.5 mの仮定。Wing のガレージ・窓の割付は模式化した。[OSM のピットレーン](https://www.openstreetmap.org/way/227902927)
