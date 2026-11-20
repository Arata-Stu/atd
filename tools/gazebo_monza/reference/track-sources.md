# Gazebo 周回路の地理データ

各 `track.geojson` はスタート直線からレース方向に一周する WGS84 経緯度の閉曲線。表の長さは縮尺を掛ける前の m。平面長は始点緯度の WGS84 局所東北座標で点間距離を合計した。シーンでは平面形状を一律 1/10 に縮小する。公式の周長は照合用であり、中心線を引き伸ばす補正は行っていない。既存の [Monza](README.md) と [Silverstone](silverstone.md) には個別の資料がある。

| ID | レイアウト参照 | 公式長 (m) | 地図上の平面長 (m) | 中心線の出典 | 収録区間 |
|---|---|---:|---:|---|---|
| `austin` | [2019 米国 GP](https://www.formula1.com/en/racing/2019/united-states) | 5,513 | 5,502.6 | [OSM](https://www.openstreetmap.org/relation/6537729/history/14) | COTA の GP 本線。 |
| `brandshatch` | [現行 GP](https://www.brandshatch.co.uk/about/circuit-layouts) | 3,916 | 3,902.9 | [OSM](https://www.openstreetmap.org/relation/7218462/history/3) | 森を巡る現行 GP ロング。1986 年 F1 当時から改修あり。 |
| `budapest` | [2019 ハンガリー GP](https://www.formula1.com/en/racing/2019/hungary) | 4,381 | 4,372.2 | [OSM](https://www.openstreetmap.org/relation/284557/history/16) | 現行 Hungaroring の GP 本線。 |
| `catalunya` | [2025 スペイン GP](https://www.formula1.com/en/racing/2025/spain) | 4,657 | 4,663.9 | [OSM](https://www.openstreetmap.org/relation/284540/history/15) | 2023 年以降の最終シケインを通らない GP 仕様。2D ベンチマークは旧シケインを通る。 |
| `hockenheim` | [2019 ドイツ GP](https://www.formula1.com/en/racing/2019/germany) | 4,574 | 4,573.4 | [OSM](https://www.openstreetmap.org/relation/284588/history/7) | Parabolika と Spitzkehre を通るショート化後の GP 本線。 |
| `melbourne` | [2025 豪州 GP](https://www.formula1.com/en/racing/2025/australia) | 5,278 | 5,273.3 | [f1-circuits](https://github.com/bacinger/f1-circuits/blob/8094e957b8d6aa8b9f4510c93968556cccd30e02/circuits/au-1953.geojson) | 2022 年改修後の湖畔の高速区間を使用。2D ベンチマークは改修前。 市街路の交差点で四角く折れる OSM 道路中心線を避け、GP の点列を使用。 |
| `mexicocity` | [2025 メキシコシティ GP](https://www.formula1.com/en/racing/2025/mexico) | 4,304 | 4,308.4 | [OSM](https://www.openstreetmap.org/relation/16251935/history/5) | スタジアム区間を通る GP 本線。 |
| `montreal` | [2019 カナダ GP](https://www.formula1.com/en/racing/2019/canada) | 4,361 | 4,362.7 | [OSM](https://www.openstreetmap.org/relation/284595/history/19) | ノートルダム島を巡る GP 本線。 |
| `nuerburgring` | [2020 アイフェル GP](https://www.fia.com/sites/default/files/2020_eifel_grand_prix_preview.pdf) | 5,148 | 5,135.7 | [OSM](https://www.openstreetmap.org/relation/38567/history/35) | GP 本線とタイトな最終手前のシケインを使用。OSM の GP リレーションにある速い分岐を、FIA 図に対応するタイトな分岐 (way 1149161211) に置換。 |
| `sakhir` | [2019 バーレーン GP](https://www.formula1.com/en/racing/2019/bahrain) | 5,412 | 5,411.9 | [OSM](https://www.openstreetmap.org/relation/284538/history/16) | GP 本線。外周の短縮路や Endurance 仕様を除く。 |
| `saopaulo` | [2019 ブラジル GP](https://www.formula1.com/en/racing/2019/brazil) | 4,309 | 4,300.4 | [OSM](https://www.openstreetmap.org/relation/6781071/history/6) | Senna S を回るインテルラゴスの GP 本線。 |
| `sepang` | [2017 マレーシア GP](https://www.fia.com/sites/default/files/2017_malaysian_preview_1.pdf) | 5,543 | 5,541.7 | [OSM](https://www.openstreetmap.org/relation/284496/history/5) | 2 本の平行ストレートを通る GP 本線。 |
| `shanghai` | [2019 中国 GP](https://www.formula1.com/en/racing/2019/china) | 5,451 | 5,456.1 | [OSM](https://www.openstreetmap.org/relation/2094941/history/6) | 上海の GP 本線。 |
| `sochi` | [2021 ロシア GP](https://www.formula1.com/en/racing/2021/russia) | 5,848 | 5,839.7 | [f1-circuits](https://github.com/bacinger/f1-circuits/blob/8094e957b8d6aa8b9f4510c93968556cccd30e02/circuits/ru-2014.geojson) | 五輪公園を回る旧 F1 全周を使用。現在の施設には短縮・変更された区間があるため、沿道の地物とは年代差がある。 |
| `spa` | [2019 ベルギー GP](https://www.formula1.com/en/racing/2019/belgium) | 7,004 | 7,003.5 | [OSM](https://www.openstreetmap.org/relation/284560/history/23) | La Source、Eau Rouge、Bus Stop を通る GP 本線。 |
| `spielberg` | [2019 オーストリア GP](https://www.formula1.com/en/racing/2019/austria) | 4,318 | 4,308.8 | [OSM](https://www.openstreetmap.org/relation/5309181/history/8) | 四輪用の GP 本線を使用。二輪用シケイン、ロングラップとピットレーンを除く。 |
| `yasmarina` | [2025 アブダビ GP](https://www.formula1.com/en/racing/2025/united-arab-emirates) | 5,281 | 5,289.5 | [OSM](https://www.openstreetmap.org/relation/11378665/history/7) | 2021 年改修後の GP 仕様。2D ベンチマークは改修前。 |
| `zandvoort` | [2025 オランダ GP](https://www.formula1.com/en/racing/2025/netherlands) | 4,259 | 4,261.8 | [OSM](https://www.openstreetmap.org/relation/13545573/history/11) | 現代の GP 本線。バンク角の再現は含まない。 |

16 コースの中心線は OSM の `type=circuit` の本線を構成する way を端点のノード ID で連結した。使用したリレーションと各 way の ID・版を各ファイルに収録する。同じ GP が別仕様を併設する場合は、表の大会のコース図に対応する周回のみを使用した。各地点の座標は取得時点の OSM スナップショットで、表の大会開催日の測量値ではない。参照データの取得日は 2026-11-20。地理データの帰属は **© OpenStreetMap contributors**、派生データのライセンスは ODbL 1.0。[OpenStreetMap の帰属について](https://www.openstreetmap.org/copyright)・[ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/)

ソチはロシア GP の旧フルコース、メルボルンは公道交差点での GP 走路を表す f1-circuits の点列を使用した。メルボルンの発着点は OSM の発着ノードをこの点列のホームストレートへ投影した。ブダペスト、サヒール、ヤス・マリーナでは同資料の直線上の始点を OSM の本線へ投影した。他は本線に属する OSM の発着ノードを使う。別々のグリッド先頭と計時ノードがある場合は計時ノードを優先した。全ファイルの `origin_note` に根拠を記載する。[f1-circuits の取得版](https://github.com/bacinger/f1-circuits/tree/8094e957b8d6aa8b9f4510c93968556cccd30e02)

路面の縁は中心線から代表幅の半分だけ左右へ広げた近似。全区間に一貫した OSM の幅タグがあるモントリオール (8 m) とザントフォールト (10 m) はその値を使う。残りの16コースでは 12 m を仮定した。幅の変化、勾配、カント、縁石の測量結果は含まない。

## f1-circuits のライセンス

> Copyright (c) 2019-2025 Tomislav Bacinger
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all
> copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.
