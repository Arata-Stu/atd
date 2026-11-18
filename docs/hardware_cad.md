# CAD・3Dプリント資料

CAD と印刷用データは `hardware/cad/` で Git 管理する。説明と取り込み元はこの文書に記録する。現在の保存先は印刷用の `tt02/` と組立参照用の `assemblies/` に分ける。

## 保存するファイル

| ファイル | 内容・形式 |
| --- | --- |
| [tt02/frame.3mf](../hardware/cad/tt02/frame.3mf) | TT-02-v3 の印刷フレーム。メッシュとスライサーのプロジェクト設定を含む3MF |
| [tt02/d455_mount.3mf](../hardware/cad/tt02/d455_mount.3mf) | D455 用マウントの3MF |
| [tt02/d435i_mount.3mf](../hardware/cad/tt02/d435i_mount.3mf) | D435i 用マウントの3MF |
| [tt02/imu_mount.3mf](../hardware/cad/tt02/imu_mount.3mf) | IMU マウントの3MF。対応機種は未確認 |
| [assemblies/d455_view.FCStd](../hardware/cad/assemblies/d455_view.FCStd) | D455 版の FreeCAD 組立モデル。印刷部品・デバイス・ハードウェアの参照形状を内包 |
| [assemblies/d435i_view.FCStd](../hardware/cad/assemblies/d435i_view.FCStd) | D435i 版の FreeCAD 組立モデル。印刷部品・デバイス・ハードウェアの参照形状を内包 |

FCStd と3MFは元の形式のまま保存する。今回のFCStdは形状と配置を保持する `Part::Feature` の組立モデルで、Sketch／PartDesign の寸法編集履歴を含む設計モデルとは区別する。3MFも印刷用メッシュ・プロジェクトの保存形式として扱い、FCStdへ変換しただけで元の寸法履歴が得られるとは扱わない。フレームの3MFには提供時の印刷設定・作成者メタデータも保持されている。

部品名が同じでも、TT-02-v3 の3MFとRev6組立モデルの部品が同じ設計版とは未確認。CADの保存は機材の採用決定を意味せず、センサの採用状況は [要件](project_requirements.md) を参照する。

## 取り込み元と確認結果

2026-11-18 にユーザー指定の次のファイルを取り込んだ。保存先の名前を英語へ整理し、ファイル内容は再保存・変換していない。元ファイルは保持し、自動同期しない。

- TT-02-v3：`iCloud Drive/lab/3d_printer/TT-02-v3/`
- Rev6組立モデル：`kart_bridge_board/manufacturing/rev6_dual_camera_2026-11-17/02_ASSEMBLY/`

| 元ファイル名 | 保存先 | サイズ（bytes） |
| --- | --- | ---: |
| `車両フレーム.3mf` | `tt02/frame.3mf` | 308,499 |
| `D455用のマウント.3mf` | `tt02/d455_mount.3mf` | 37,188 |
| `D435i用のマウント.3mf` | `tt02/d435i_mount.3mf` | 71,911 |
| `imuマウント.3mf` | `tt02/imu_mount.3mf` | 12,532 |
| `D455_VIEW.FCStd` | `assemblies/d455_view.FCStd` | 143,120 |
| `D435i_VIEW.FCStd` | `assemblies/d435i_view.FCStd` | 153,644 |

合計726,894 bytes（約710KiB）。全6ファイルで取り込み元とSHA-256が一致し、ZIP CRCと内部XMLの検査が通過した。3MFはmm単位で、モデルの参照先はZIP内に収まっている。FCStdもBREP形状を内包し、外部ファイルへのリンクは見つからなかった。D455版は6形状、D435i版は7形状を保持する。

今回の検証はコピーの同一性・アーカイブ・参照構造まで。CADアプリでの再保存や形状変更、実物との寸法一致・組付け・公差・強度・振動の検証は行っていない。

## 車両設定に使う寸法の参考

2026-11-18 に3MFの頂点とFreeCADの内蔵形状を読み取り、印刷フレームの外接寸法を確認した。CAD は再保存していない。

| 対象 | CAD 軸 X × Y × Z の外接寸法（mm） |
| --- | --- |
| `tt02/frame.3mf` のフレーム本体 | 120 × 210.5 × 19.2 |
| `assemblies/d455_view.FCStd` の `PrintPart01` | 120 × 210.5 × 19.2 |
| 同組立の左右脚 `PrintPart02` / `PrintPart03`（各1個） | 16.8 × 32.4 × 45 |

3MFのフレームSHA-256は `1a36617b0864c304103ca44e0005683183dd1aa6c68565f88f442f79a72708bd`。3MFのbuild配置には印刷台上の平行移動があり、上表はその影響を受けない寸法である。FCStdのフレームとは原点位置が異なる。D455用3MFの脚幅は15 mmで、組立参照の16.8 mmとは一致しないため、部品を同一版として合成しない。

このフレーム外接寸法は、タイヤを含む車体外形・ホイールベース・トレッドではない。現資料から後輪回転軸と路面の位置、RealSense driver の `camera_link` 原点までの取付変換は確定できない。UIには参考寸法を表示し、実際の車両寸法と取付TFは未計測のまま入力を待つ。[車両配置・TFの設定手順](vehicle_layout.md)に従って実測する。

## 更新時の扱い

CAD・印刷データを変更した場合は、対象ファイルと変更理由・対象部品を同じコミットで記録する。編集元や提供された3MFをcacheとして削除しない。FreeCADの自動バックアップ（`.FCStd1`等、`.FCBak`）はGit管理から除外する。
