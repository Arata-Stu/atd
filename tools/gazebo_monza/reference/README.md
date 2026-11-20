# モンツァGPコースと第1シケイン周辺の地理データ

他の19コースの平面座標・資料と再現範囲は[全20コースの一覧](../../../docs/gazebo_circuits.md)にまとめた。

収録した OpenStreetMap データと、このデータから派生する地理データのライセンスは ODbL 1.0。帰属表記は **© OpenStreetMap contributors**。PNG のカメラ画像や動画として外部に公開する際も、この帰属とリンクを添える。[OpenStreetMap の著作権・ライセンス](https://www.openstreetmap.org/copyright)・[Open Data Commons Open Database License 1.0](https://opendatacommons.org/licenses/odbl/1-0/)

## 走路の根拠

`monza-track.geojson` は2026年11月20日に取得した OpenStreetMap の WGS84 座標列を抜粋したもの。GeoJSON の座標順は経度、緯度。全5本の way の ID、取得したバージョン、編集日時と changeset を各 Feature の properties に保存した。両境界は北へ向かう直線からレース進行方向に一周する閉じた LineString。[取得元のOSM範囲](https://api.openstreetmap.org/api/0.6/map?bbox=9.2745,45.6140,9.2910,45.6305)・[モンツァGPコースのOSMリレーション](https://www.openstreetmap.org/relation/284565)

| role | 取得元と用途 |
|---|---|
| straight | スタート側の本線。シーンでは第1シケインに向かう最後の実距離120 mを走る。[way 19842206 v56](https://www.openstreetmap.org/way/19842206/history/56) |
| chicane | 自動車用 Variante del Rettifilo。2024年の改修を反映したとの編集記録を持つ全25点。[way 179968242 v9](https://www.openstreetmap.org/way/179968242/history/9)・[編集履歴](https://www.openstreetmap.org/changeset/183500424) |
| exit | シケイン出口から続く本線。出口側の実距離40 mを走る。[way 179968264 v6](https://www.openstreetmap.org/way/179968264/history/6) |
| left | 外周輪郭。編集者は衛星画像の白線に沿って作成したと記録している。[way 1569593286 v1](https://www.openstreetmap.org/way/1569593286/history/1)・[編集履歴](https://www.openstreetmap.org/changeset/190723393) |
| right | 内周輪郭。編集記録にはガードレール線とコース変更の修正が記載されている。走路の内縁近似として使う。[way 1569604642 v1](https://www.openstreetmap.org/way/1569604642/history/1)・[編集履歴](https://www.openstreetmap.org/changeset/190724198) |

公式案内では Variante del Rettifilo は右の直角カーブに急な左折が続く。2024年に第1シケインの走路拡幅と縁石の更新が行われた。ここでの両縁は地図上の輪郭であり、主催者による測量値や現地精度の保証はない。[サーキット公式の解説](https://www.monzanet.it/en/circuit/)・[ACI の改修報告](https://vercelli.aci.it/FORMULA-1-PIRELLI-GRAN-PREMIO-D-ITALIA-2024-SULLA-RINNOVATA-PISTA-DI-MONZA)

両縁は自動車用シケインに沿って曲がる。直進できるエスケープ路や二輪車用の別経路の中心線は分岐後に輪郭の外へ出る。シーン上の縁石は、第1コーナーの右縁と第2コーナーの左縁から曲率の大きい区間を選んだ近似。縁石の始点・終点を実測したデータはない。[直進経路](https://www.openstreetmap.org/way/179968267)・[二輪車用経路](https://www.openstreetmap.org/way/38168756)

## RC シーンの座標とカメラ台の軌跡

`geography.project(lon, lat)` は自動車用シケイン入口の OSM node 1828499271（経度9.2818939°、緯度45.6244492°）を原点とし、WGS84 楕円体の当該緯度における曲率半径を使った局所平面へ投影する。x は東、y は北。水平距離を一律に0.1倍にする。実コースの約240 mの区間が RC シーンの約24 mに相当する。標高とバンク角はこの地理データに含まれない。[原点](https://www.openstreetmap.org/node/1828499271)

既定の `chicane` ではカメラ台は OSM の本線を約5 cm間隔でサンプリングした経路をたどる。頂点で向きが急変するのを抑えるため、前後各10 cmの範囲で座標を平滑化する。スタート地点は動画用に選んだため公式のスタートラインとは異なる。

`lap` は本線の内外周から走行線を生成する。左右の境界頂点を舗装メッシュと同じ順に結んで各線分の中点を取り、RC換算で前後各20 cmを使って平滑化して約10 cm間隔でサンプリングする。発着位置は OSM の GP リレーションで `start` と `finish` の両方を割り当てられたノード。前後3 mで滑らかに位置を合わせ、同じ方位で次周に接続する。1周は約579.20 mで、公式発表の実コース長5,793 mを1/10にした579.30 mと比較できる。コース上の高低差は収録していない。[スタート／フィニッシュ地点の座標](https://api.openstreetmap.org/api/0.6/node/1828499259/4)・[GPリレーションの構成](https://api.openstreetmap.org/api/0.6/relation/284565/33)・[モンツァ公式のコース長と走行方向](https://www.monzanet.it/en/circuit/)

いずれの走行線にも実車テレメトリから得たレーシングラインやドライバーの速度履歴は含まれない。従来の F1TENTH コンパクトコースは2Dの比較用資産として別管理する。[F1TENTH サンプル](../../../samples/f1tenth/README.md)

## 観客席、森林、ランオフの根拠

[monza_scenery.geojson](monza_scenery.geojson) は同日に取得した OSM データを収録する。各地物に `osm_type`、`osm_id`、`version`、編集日時 `timestamp`、当時の `tags` がある。観客席の外周と砂地の輪郭は元のノード列を保持した。森林は multipolygon の内周を除き、GeoJSON の `woodland_clip_bbox_wgs84` で表示対象を切り出した。取得日の現地測量は行っていない。[OSM 地図で位置を確認](https://www.openstreetmap.org/#map=17/45.62445/9.28189)

| 地物 | 地図から使う情報 | 描画上の近似 |
|---|---|---|
| 観客席 6 棟 | 外形・位置。シケイン外側の [Prima Variante Esterna](https://www.openstreetmap.org/way/231026019/history/5)、アプローチ右側の [Alta Velocità](https://www.openstreetmap.org/way/231026024/history/4) など。 | 長辺のうち走路に近い側を前列にして観覧席を生成する。段数・高さ・席間の通路・色に加え、段床を地面で受ける支柱・斜材・筋交いの位置・太さも推定。名称のない2棟の席番号は未同定。 |
| 森林 4 区域 | [Bosco Bello](https://www.openstreetmap.org/way/34405183/history/15) と OSM relation [3100229](https://www.openstreetmap.org/relation/3100229/history/11)・[3115113](https://www.openstreetmap.org/relation/3115113/history/1)・[3307584](https://www.openstreetmap.org/relation/3307584/history/4) の林縁・空き地。 | 地物 ID で固定した乱数で森林内に樹木を配置。幹の座標・本数・樹高・樹種・枝ぶりは推定。 |
| ランオフ | コース脇の [砂地1](https://www.openstreetmap.org/way/231026022/history/2)・[砂地2](https://www.openstreetmap.org/way/231260688/history/2)、直進路と二輪用シケインの中心線。 | `natural=sand` の領域を着色。砂利の粒径や、2024年改修後のランオフの最新境界は未確認。別経路の舗装幅は推定。 |

観客席の名称・配置と、Alta Velocità 6A/B/C および Prima Variante Esterna 8A/B に屋根の表示がないことを2026年の主催者資料で照合した。公式案内図は略図のため、建物の位置・向きには OSM の外形を使用した。現地の仮設スクリーンや広告の位置・大きさは別途確認が必要。[会場公式の観客席案内](https://www.monzanet.it/en/circuit/)・[2026年公式案内図・屋根の凡例](https://www.monzanet.it/wp-content/uploads/2026/03/GP_F1_2026_LISTINO_PREZZI_3_2.pdf)

Bosco Bello 周辺では2023年7月の暴風で倒木が発生し、その後に再植林事業が始まった。採用した森林地物の版は2013〜2017年のため、2026年の樹冠密度や成木の分布には現地と差が出る。[サーキットの再植林計画](https://www.monzanet.it/reforestation-plan-approved/)・[ロンバルディア州の復旧計画](https://www.regione.lombardia.it/agricoltura/boschi-e-foreste/rimboschimento-aree-del-parco-di-monza)
