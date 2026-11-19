# F1TENTH 収録コースと F1 開催履歴

全 23 件のうち、F1 グランプリで使用されたロードコースと同系統の **20 件**を ATD のベンチマークに採用した。2026 年のカレンダーに載る会場と、過去に開催した会場を含む。開催履歴は施設単位、採用判定は配布ファイルに描かれた周回路単位で付けている。[全件の判定・コース別の出典](../samples/f1tenth/catalog.json)・[F1TENTH の収録リスト](https://github.com/f1tenth/f1tenth_racetracks/tree/b95c4eff766f6367d66b310ea20cd2c9563712c0)

| 区分 | 件数 | 配布フォルダ名 |
| --- | ---: | --- |
| 採用・2026 年の開催会場 | 15 | Austin、Budapest、Catalunya、Melbourne、Mexico City、Montreal、Monza、SaoPaulo、Sepang、Shanghai、Silverstone、Spa、Spielberg、YasMarina、Zandvoort |
| 採用・過去の開催会場 | 5 | BrandsHatch、Hockenheim、Nuerburgring、Sakhir、Sochi |
| F1 開催施設の別レイアウト | 1 | IMS（インディアナポリスのオーバル） |
| F1 世界選手権の GP 開催記録なし | 2 | MoscowRaceway、Oschersleben |

2026 年は **開催会場**で分類した。同年の Bahrain GP はマレーシアのセパンで開催されたため、Sepang を当年の会場、Sakhir を過去の会場に数えている。カレンダー判定日は 2026 年 11 月 20 日で、同日時点の予定も含む。[F1 公式カレンダー](https://www.formula1.com/en/racing/2026)・[2026 年 Bahrain GP in Malaysia](https://www.formula1.com/en/racing/2026/bahrain)

## レイアウトを照合した箇所

- **Brands Hatch** の CSV は森を回る長い GP ループを含む。配布元では DTM と分類されている。F1 は 1986 年にもこの施設の GP コースで開催された。ATD では GP コース系統に含め、各コーナーが 1986 年当時の寸法と一致するとは仮定しない。[施設のコース図](https://www.brandshatch.co.uk/about/circuit-layouts)・[1986 年英国 GP](https://www.formula1.com/en/results/1986/races/506/great-britain/race-result)
- **Nürburgring** は下側のヘアピンまで回る Grand-Prix-Strecke。元の実寸 CSV は約 5.144 km で、FIA の GP 仕様 5.148 km に対応する。同施設の短縮 Sprint 周回路は 3.629 km。[FIA の構成別コース図](https://www.fia.com/sites/default/files/l10_04_circuits_2015.pdf)・[2020 年アイフェル GP](https://www.fia.com/sites/default/files/2020_eifel_grand_prix_preview.pdf)
- **Zandvoort** は 1999 年の拡張以降の長いループを持つ。F1 が 2021 年に復帰した際には一部のコーナー形状とバンクが変わっている。採用対象はこの周回路の 2D 近似。バンクを使った走行性能は評価の範囲外。[大会主催者によるコース解説](https://dutchgp.com/en/explore-the-track/)・[FIA による改修説明](https://www.fia.com/sites/default/files/2021_dutch_grand_prix_-_preview.pdf)
- **IMS** の配布形状は四つのコーナーを持つオーバル。1950〜1960 年の Indy 500 はドライバーズ世界選手権の得点対象だった。F1 米国 GP が 2000〜2007 年に使用したロードコースにはインフィールドがあるため、今回の GP ロードコースの比較からは外した。[配布図](https://github.com/f1tenth/f1tenth_racetracks/blob/b95c4eff766f6367d66b310ea20cd2c9563712c0/IMS/IMS_map.png)・[F1 による選手権史](https://www.formula1.com/en/latest/article/why-is-it-called-formula-1-and-12-other-questions-about-the-championships.1GHeel6u4jga6hMpX2eFs1)・[IMS による F1 開催記録](https://www.indianapolismotorspeedway.com/history/formula-one)
- **Moscow Raceway** と **Oschersleben** は DTM 等が開催された別のサーキット。F1 ロシア GP のソチは、配布元でも `Sochi` として別に収録されている。[Moscow Raceway の開催カテゴリー](https://moscowraceway.ru/info/facts)・[DTM の Oschersleben 開催記録](https://dtm.com/de/news/neue-era-neue-saison-dtm-auftakt-steigt-in-oschersleben)・[F1 ロシア GP](https://www.formula1.com/en/racing/2021/russia)

## 配布形状の縮尺と版

F1TENTH は走路幅を 2.20 m とする 1/10 車両向けの座標を配布している。全 23 件について、配布 CSV と引用元 TUM の実寸 CSV の点順を照合した。各コースで原点を合わせて x・y に同じ倍率を掛けると、すべての対応点が一致した。倍率はコースごとに約 0.0654〜0.0920。例えば Montreal は元の中心線 4,357.5 m に対し配布中心線は 285.05 m。周長は各点を順につなぎ、末点から始点の距離も加算した値である。`catalog.json` の `upstream_centerline_length_m` はこの配布中心線の長さで、ATD で生成した走行ラインの長さは各ラインのファイルで確認できる。[F1TENTH のデータ仕様](https://github.com/f1tenth/f1tenth_racetracks/blob/b95c4eff766f6367d66b310ea20cd2c9563712c0/README.md)・[照合した TUM の原データ](https://github.com/TUMFTM/racetrack-database/tree/e59595d1f3573b30d1ded6a08984935b957688e0/tracks)

ATD ではこの配布版を元に 1/10 RC 車用のコンパクトな周回路を作り、既定の車両幅・曲率制限で走行ラインを生成できるようにコーナーを丸めている。変換倍率・幅・元輪郭との距離は各 HDMap の `source` に残る。例えば Catalunya の元データは最終セクターにシケインのある旧版、Melbourne と YasMarina も大改修前の周回路である。走行アルゴリズムを比較する際は同じ版の HDMap・走行ラインを使う。[ATD の地図生成](../samples/f1tenth/regenerate.py)・[Catalunya の変更](https://corp.formula1.com/formula-1-aws-gran-premio-de-espana-2023-changes-its-track-configuration/)・[Melbourne の変更](https://www.formula1.com/en/latest/article/revealed-the-track-tweaks-set-to-make-melbournes-albert-park-circuit-5.6usfQo5O6o4KsVi3WTAqFA)・[Yas Marina の 2019 年版](https://www.formula1.com/en/racing/2019/united-arab-emirates)
