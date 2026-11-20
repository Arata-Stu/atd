# 周回用コースの沿道地物

2026年11月20日に OpenStreetMap の地図 API から取得した、コース沿い18か所のスナップショット。各コースのファイルに取得範囲・URL・元 XML の SHA-256 を収録し、各地物に OSM オブジェクトの種類・ID・版・最終編集日時・元のタグを保存した。地理データと派生データの帰属は **© OpenStreetMap contributors**、ライセンスは ODbL 1.0。[OpenStreetMap の帰属](https://www.openstreetmap.org/copyright)・[ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/)

コース名から景観の GeoJSON を開ける。数値は採用した重複のない OSM オブジェクト数で、分割した同じ観客席や水面は1件と数える。「建物」はピット棟を含む。「木」は単独で位置が記録された樹木で、樹林ポリゴン内に生成する木の本数は含まない。モンツァとシルバーストンは個別に収録している。[全コースの走路出典](track-sources.md)・[モンツァの景観](lap-scenery.md)・[シルバーストンの景観](silverstone.md)

| コース | ピット走路 | 観客席 | 建物 | 樹林 | 木 | 水面 | 砂地 | 草地 | 地図で確認できる地物の例 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| [austin](circuits/austin/scenery.geojson) | 1 | 8 | 26 | 0 | 0 | 4 | 0 | 0 | [展望塔（76.5 m）](https://www.openstreetmap.org/node/7911006730/history/3) |
| [brandshatch](circuits/brandshatch/scenery.geojson) | 1 | 5 | 38 | 9 | 48 | 0 | 0 | 0 | [Paddock Hill Grandstand](https://www.openstreetmap.org/way/115380274/history/2) |
| [budapest](circuits/budapest/scenery.geojson) | 1 | 1 | 20 | 7 | 12 | 1 | 0 | 0 | [Hungaroring 観客席](https://www.openstreetmap.org/way/230979479/history/7) |
| [catalunya](circuits/catalunya/scenery.geojson) | 3 | 18 | 46 | 11 | 0 | 0 | 0 | 0 | [Main grandstand](https://www.openstreetmap.org/way/33742643/history/8) |
| [hockenheim](circuits/hockenheim/scenery.geojson) | 1 | 6 | 66 | 1 | 35 | 2 | 16 | 0 | [LBBW-Haupttribüne](https://www.openstreetmap.org/way/34400180/history/7) |
| [melbourne](circuits/melbourne/scenery.geojson) | 1 | 2 | 56 | 0 | 446 | 1 | 5 | 0 | [Albert Park Lake](https://www.openstreetmap.org/relation/346429/history/11) |
| [mexicocity](circuits/mexicocity/scenery.geojson) | 2 | 25 | 59 | 4 | 203 | 0 | 0 | 0 | [スタジアム内の Palco Central](https://www.openstreetmap.org/way/828705061/history/3) |
| [montreal](circuits/montreal/scenery.geojson) | 1 | 0 | 22 | 27 | 31 | 8 | 0 | 0 | [Olympic Basin](https://www.openstreetmap.org/way/129924946/history/15) |
| [nuerburgring](circuits/nuerburgring/scenery.geojson) | 1 | 12 | 32 | 19 | 2 | 1 | 0 | 0 | [BMW Tribüne](https://www.openstreetmap.org/way/32895415/history/14) |
| [sakhir](circuits/sakhir/scenery.geojson) | 1 | 9 | 103 | 0 | 19 | 3 | 15 | 33 | [Sakhir Tower](https://www.openstreetmap.org/way/187123438/history/6) |
| [saopaulo](circuits/saopaulo/scenery.geojson) | 1 | 3 | 143 | 6 | 66 | 1 | 0 | 0 | [M 観客席](https://www.openstreetmap.org/way/138420844/history/6) |
| [sepang](circuits/sepang/scenery.geojson) | 2 | 2 | 40 | 5 | 270 | 2 | 0 | 0 | [Paddock / Pit Building](https://www.openstreetmap.org/way/144362327/history/7) |
| [shanghai](circuits/shanghai/scenery.geojson) | 2 | 16 | 38 | 0 | 0 | 21 | 0 | 0 | [Section B1](https://www.openstreetmap.org/way/156341844/history/6) |
| [sochi](circuits/sochi/scenery.geojson) | 0 | 1 | 121 | 2 | 57 | 1 | 0 | 0 | [Bolshoy Ice Dome](https://www.openstreetmap.org/way/54521221/history/33) |
| [spa](circuits/spa/scenery.geojson) | 1 | 8 | 42 | 27 | 1 | 1 | 1 | 0 | [Tribune F1](https://www.openstreetmap.org/way/174882363/history/3) |
| [spielberg](circuits/spielberg/scenery.geojson) | 1 | 9 | 14 | 7 | 2 | 3 | 0 | 0 | [Red Bull Tribüne](https://www.openstreetmap.org/way/118364945/history/4) |
| [yasmarina](circuits/yasmarina/scenery.geojson) | 2 | 6 | 83 | 0 | 10 | 2 | 0 | 365 | [Marina Grandstand](https://www.openstreetmap.org/way/133907079/history/9) |
| [zandvoort](circuits/zandvoort/scenery.geojson) | 1 | 1 | 6 | 4 | 0 | 13 | 4 | 0 | [Hoofdtribune](https://www.openstreetmap.org/way/52473180/history/6) |

## 地図と3D表現の対応

- ピット走路は17コースの23本の OSM way が対象。GP リレーションの pit メンバーを優先し、登録のない場合は発着地点に近い明示的なピット走路と、両端でつながる区間を使う。中心線を地図から取り、幅は実寸6.5 m（RC 0.65 m）で近似する。地下部分は省き、地表にある入口・出口は本線に接続する。ソチの旧 GP 用ピットレーンは確認できたデータに含まれない。[メルボルンの GP リレーション](https://www.openstreetmap.org/relation/280443)・[ヤス・マリーナの地下出口](https://www.openstreetmap.org/way/176695255)
- 建物と観客席は OSM の閉じた外周または完全な multipolygon から取得した。一般建物は走路から実距離115 m以内の近いものを最大150件、観客席・名前から識別できたピット棟は190 m以内を使う。階段の正面は走路に近い長辺から推定し、湾曲した棟は最大4分割する。屋根の根拠は `covered=yes`。高さ `height_m` はメートル表記の OSM 値を保持し、シーン側で RC の縮尺を掛ける。立面や意匠、その他の高さ、屋根の形状は概略である。
- Austin の展望塔の位置と高さ76.5 mは OSM に基づく。設計事務所の公表資料にある赤い鋼管と地上230 ft（約70.1 m）の展望デッキを表現した。塔の幅・向き・形状は模式化している。[Miró Rivera Architects の設計資料](https://www.mirorivera.com/observation-tower)
- 水面・砂地を走路から190 m以内で切り取り、内周や中庭を保った形で三角形を生成した。砂地は `natural=sand` の地物で、すべてがランオフとは限らない。Sakhir と Yas Marina では、乾いた地面に重ねる芝として記録済みの `landuse=grass/meadow/village_green` も抽出した。他コースの未分類地は共通シーンの地面を見せる。
- 樹林は `natural=wood` または `landuse=forest` に限り、走路から135 m以内の登録区域を使用する。走路、近接する別走路、建物、水面と砂地などを除いて、樹林内に個体を配置する。単独樹木は120 m以内の `natural=tree` の元座標を使い、過密な場所は12 m間隔で間引く。木の外見・樹高などには汎用モデルを用いる。
- 建物や観客席の近似矩形が走路に重なる場合は省くため、高架の建物や跨道橋の下を通る景観は対象外。OSM 登録外の仮設席・観客、レース当日の広告・装飾も省く。たとえばモントリオールには水面・パドック・樹林があるが、この抽出範囲に観客席として特定できた建物は0件。ソチでは旧 F1 走路と取得時点の周辺施設を組み合わせている。

## 再抽出

通常の周回では同梱 GeoJSON を読む。元の XML を保存していれば、SHA-256 の一致で同じ入力かどうか確認できる。更新するときは該当ファイルの `source` に記録した URL を使う。API は取得時点の地図を返すため、後日の再取得では地物の版や結果が変わる。

Shapely 2.1以降を含む開発用 Python 環境で、たとえば Austin の XML を以下のように処理する。結果は指定した走路ファイルと同じディレクトリの `scenery.geojson` に書き込まれる。距離・面積の抽出条件はすべて実寸である。[抽出処理](../prepare_scenery.py)

```sh
course=tools/gazebo_monza/reference/circuits/austin
url=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["source"])' "$course/scenery.geojson")
curl --fail --location "$url" --output /tmp/atd-austin.osm
python3 tools/gazebo_monza/prepare_scenery.py "$course/track.geojson" /tmp/atd-austin.osm --retrieved "$(date -u +%F)"
```
