# Rev6 / Raspberry Pi Pico ファームウェア

2026-11-20、**v0.2.0-led-bench**。対象は **JPBB Rev6のRP2040版Raspberry Pi Pico**。本体の緑色LED（GP25）で接続・制御・異常の6状態を表示します。[LEDの見方](LED_SIGNALS.md)

取り込み元は `/Users/at/project/tools/pcb/kart_bridge_board/firmware/releases/pico_rev6_v0.2.0_led_bench_2026-11-20/` の `source/firmware/` です。実行用 C/H はリリースと同一です。共通の JPB1 通信コードはこのディレクトリの `src/`・`include/` に、そのテストは `tests/test_protocol.c` に配置しています。親の `firmware/COLCON_IGNORE` で ROS と Pico のビルドを分けています。

ATDでホストテスト（UBSan併用）、LEDの点滅と制御状態の遷移、リリース同梱のネットリストとのGPIO 11信号照合を確認しました。RP2040向けに再ビルドしたBINとUF2は配布版とバイト単位で一致しました（UF2のSHA-256 `fc666b3353fd694a43a33b59f481e97beca6820e605fbd81b228a5ac1a5cd050`）。この版の実機書き込み・実点灯・PWM波形・走行は未検証です。

## 今回決めた動作

| 項目 | 仕様 |
| --- | --- |
| サーボ／ESC出力 | 両方100 Hz、1000〜2000 µs、中立1500 µs |
| 手動 | SBUSのCH1＝操舵、CH2＝ESCをPicoでPWMへ再生成 |
| 操作権切替 | CH3の2位置スイッチ。低側＝プロポ、高側＝Jetson許可 |
| Jetson停止中 | ESCの公称6 V BECだけでPicoと受信機が動き、手動操作可能 |
| 受信機なし | 起動後一度もSBUSを検出していない場合、Jetson／DualSense操作を許可 |
| 受信機喪失 | 一度検出した後は、再起動まで受信機必須。USBだけの操作へ自動移行しない |
| USB | 同じ物理USBからCDC0＝既存JPB1制御、CDC1＝センサ情報の2ポート |
| RPM | 立上り相当エッジの累積数・時刻・周期を取得。PPR未確定なのでRPM値を推定しない |
| IMU | 未選定。CS High、SPI無効で待機。モデル固有のドライバは今後追加 |

CHとは受信機が送る操作値の番号です。CH3には2位置スイッチを割り当てます。専用のSTOP位置は設けませんが、通信や電圧の異常では自動的に中立またはパルス抑止へ移ります。「Jetson許可」は自動運転そのものではなく、DualSenseを含むJetson側制御に操作権を渡す意味です。

**受信機なし運転にもESC/BECは必要です。** USBだけではサーボ・ESC用電源は供給されず、電圧保護回路が出力を抑止します。手動にもPicoの動作が必要で、ハードウェアのPWM直通経路はありません。

## 書き込み

1. ESC/BECを切り、PicoのBOOTSELを押しながら、データ通信対応Micro-BケーブルでPCへ接続します。
2. 表示される`RPI-RP2`ドライブへ **`jpbb_pico_rev6.uf2`** をコピーします。
3. 自動的に再起動します。OSの起動やUSB接続待ちは行いません。

通常更新は、ESC/BECを切りUSBを接続した状態でCDC1へ`BOOTSEL`と改行を送ることでも行えます。BEC ADCが1 V未満かつ電圧正常信号がLowの場合だけ受理します。初期化中はADCが未測定なので拒否します。1200 baud変更だけでは再起動しません。ファームウェア破損時はPico本体のBOOTSELを使います。

Jetson/Linuxでは2ポートが現れます。`ttyACM0/1`という番号は接続順で変わるため、`/dev/serial/by-id/`の **`if00`側を制御、`if02`側をセンサ** に使用します。シリアル番号はPicoの固有IDです。USBの列挙名は`JPBB Rev6 Pico`。旧STM32用のデバイスパスは選び直してください。CDCのbaud設定値はUSB実効速度やPWM周期を変更しません。

## 制御と異常復帰

1 ms周期のタイマー割込みで制御状態とPWMを更新します。UART受信も割込み処理です。USB送受信・診断ADCはmainで処理し、USB送信を待ちません。RPMはより低い割込み優先度とし、エッジ状態の更新だけ短く排他します。IMUの未接続は走行禁止条件にしません。

| 条件 | 動作 |
| --- | --- |
| 起動／電圧異常 | サーボ・ESC信号端子はLow、パルスなし |
| 電圧正常に復帰 | GP9 Highを100 ms確認。以前のHOST指令を破棄 |
| 手動選択 | CH3相当値1300 µs以下を40 ms確認 |
| Jetson許可 | CH3相当値1700 µs以上を100 ms確認 |
| CH3中間値 | 100 ms継続で不明扱い。常用するSTOP位置ではない |
| SBUS failsafe flag | 直ちに有効受信を解除、HOSTもdisarm |
| SBUS frame_lost flag | そのフレームを制御に使わず、前回正常値を最大50 ms保持 |
| SBUS正常値が50 msを超えて古い | 受信失効、HOSTもdisarm。CH3の確認時間を待たない |
| USB断／リセット／suspend | HOSTをdisarm。手動は継続可能 |
| HOST指令が200 msを超えて古い | 中立、HOSTをdisarm |
| heartbeatが500 msを超えて古い | 中立、HOSTをdisarm |
| 制御割込みが止まる | ハードウェアwatchdogが約100 ms設定でリセット |

HOST開始／再開には、現在の電源・USB・プロポ許可が正常な状態で、**新しい中立のHOST要求**が必要です。過去の走行指令、再接続後の非中立指令だけでは再開しません。一度でも構造正常SBUSを検出すると、failsafe/frame_lost付きの最初のフレームでも受信機を必須とします。

電源異常はGP9のLowと、GPIOハードウェアが記録した立下りを監視します。サンプリング間の短いLowも復帰待ちの対象にする設計です。独立回路によるPWM遮断・クランプはファームウェアとは別に動きます。実際に捕捉できる最小パルス幅と電源切替波形は未測定です。

PWM更新は次の周期境界へ反映されるため、ソフトウェアの判定後、端子波形への反映に最大約10 msを見込みます。中立1500 µs、パルス停止、実車の停止・制動は同じ意味ではありません。ESCの校正と実動作を確認してください。100 Hzはユーザーが選択した設定であり、TBLE-04SRのメーカー保証上限を確認したという意味ではありません。

watchdogを更新するのは制御処理だけです。ただしPico停止前にPWM peripheralが最後の値を繰り返す時間は残ります。独立したファームウェア生存監視回路はありません。またRPMへの大量ノイズでmain処理が飢餓になる限界は残り、実入力周波数・負荷・ノイズの実測が必要です。

## 実配線との対応

| 機能 | GPIO | Pico物理端子 |
| --- | --- | --- |
| SBUS UART1 RX | GP5 | 7 |
| 操舵PWM／ESC PWM | GP6／GP7 | 9／10 |
| 電圧正常 | **GP9** | **12** |
| SPI SCK／MOSI／MISO | GP10／11／12 | 14／15／16 |
| IMU CS／INT | GP13／14 | 17／19 |
| RPM | GP15 | 20 |
| BEC ADC | GP26／ADC0 | 31 |
| USB VBUS検出 | GP24 | Pico内部接続 |

以前の資料のGP8表記は誤記です。最終ネイティブネットリストでは物理12番＝GP9であり、GP8は未接続です。製造用Gerber・配線は変更していません。

SBUSは基板で反転済みのため、UARTは通常極性の100000 baud・8E2を使います。PWMも基板の反転段に合わせてGPIO側を反転します。GP26の電圧換算は100k/47kの分圧、`ADC × 3.3 V / 4096 × 147 / 47`。未校正の診断値で、電源正常判定にはGP9を使います。

公称6 V BEC、設計上限6.3 Vの構成です。IMU端子は3.3 V専用・設計消費電流10 mAまで。RPM端子の電源はBECそのままなので、5 V専用センサにはそのまま供給できません。RPM入力RAW側にはプルアップがなく、オープンコレクタ出力のセンサは別途適合確認が必要です。

## USBの契約

CDC0は既存のASCII JPB1を維持します。状態20 Hz、ホストの想定指令100 Hzです。

```text
JPB1,C,seq,steering,throttle,reverse,brake,flags,CRC
JPB1,S,seq,rx1,rx2,rx3,servo,esc,vbec,selector,active_path,fault_bits,CRC
```

CRCは末尾のCRC区切りより前に対するCRC16/CCITT-FALSE（初期値FFFF、多項式1021）。4桁hex、CRLFで送信します。`rx1/2/3`は操舵・ESC・権限のµs相当値で、SBUS raw値ではありません。Sのseqは基板の状態送信カウンタです。従来互換のためCの重複・逆順seqの拒否は加えていません。

HOST値はsteering −1000〜1000、throttle/reverse/brake 0〜1000。操舵は`1500 + steering/2`、前進は`1500 - throttle/2`、後退／brakeは`1500 + value/2`。throttleとreverseの同時非ゼロは拒否、brakeは従来どおり優先します。ホスト既定の操舵反転に重ねて反転しません。

flags bit0＝指令有効、bit1＝HOST要求。新しい中立要求は4つの操作値が厳密に0、flags=3です。flags=1の中立はdisarm兼異常履歴のクリアです。

`selector`は0＝プロポ、1＝HOST許可、`active_path`は0＝待機、1＝手動、2＝HOST、3＝異常。fault bitsはbit0電源、1 USB、2受信喪失、3切替不明、4受信failsafe、5指令timeout、6heartbeat timeout、7指令値異常。過去のHOST異常は、新しい有効な中立指令または正常な手動操作で解除します。現在の電源・受信異常は解消まで残ります。

対話コマンド`PING / HEARTBEAT / STATUS / HELP / MANUAL / SET / STREAM / AUTO`も使えます。`SET 1500 1500`→`AUTO`でHOSTをarmし、以後200 ms以内に`STREAM <servo> <esc>`、500 ms以内に`HEARTBEAT`を送ります。`MANUAL`はHOSTの解除で、実際の手動選択はプロポスイッチに従います。JPB1のSは最初のCを受けてから送信します。

CDC1の`INFO`は版名`v0.2.0-led`を返します。読み取り専用の`LED`で現在の表示状態と電源・受信・USB・arm・faultを確認できます。初期状態ではセンサstreamはOFFです。`STREAM ON`で50 Hzの`JPX1,T`を送信し、`STREAM OFF`で終了します。ATD の監視用nodeはCDC1へ`INFO`と`STREAM ON`を送り、全16chを受け取ります。

```text
JPX1,T,seq,board_us,sbus_age_ms,sbus_frames,sbus_lost_frames,sbus_failsafe_frames,uart_errors,flags,
ch1,...,ch16,rpm_edges,rpm_last_us32,rpm_period_us32,rpm_period_recent,rpm_seen,
vbec_mv,power_ready,sbus_fresh,active_path,fault_bits,tx0_dropped,tx1_dropped,rx0_rejected,rx1_rejected,CRC
```

上記は説明のため改行していますが実際は1行、CRC以外40フィールドです。`board_us`は起動からの64 bit時刻。`sbus_age_ms`は最後の構造正常フレームからの時間で、未受信時4294967295。flags bit0/1＝digital CH17/18、bit2＝frame_lost、bit3＝failsafe。ch1〜16は最新フレームのraw 0〜2047で、`sbus_fresh`とは分けて解釈します。UART errorsはreset callback回数で、同じUART障害を複数回数える場合があります。

RPMは外部立上り相当の累積エッジ数と32 bit µs時刻・周期です。時刻とcountはwrapします。周期の新鮮さは`max(1秒, 前周期×3)`で失効し、最初の1エッジだけでは周期を有効にしません。無パルスは停止・断線を区別できません。IMUデータは送っていません。

両ポートとも有限bufferです。送信FIFOが不足したフレームは丸ごと捨て、制御を止めません。過長行やNULを含む受信行は区切りまで捨て、切り詰めた文字列を指令として受理しません。

## ビルド・設定・検証

ホストテストに必要なのは通常のC compilerとPython3です。ATD repo のrootから実行します。

```bash
fw=ros2_ws/src/atd_vehicle_interface/firmware/pico_rev6
bash "$fw/scripts/test.sh"
```

ホストテストの出力先は `pico_rev6/build/host-tests/` で、`JPBB_PICO_TEST_BUILD_DIR` で変更できます。通常C実行にUBSanを併用し、`JPBB_SANITIZE=0` で sanitizer なしにできます。GPIO照合に必要なCAD/netlistは同梱していません。未指定時は明示的に `SKIP` を表示します。実ファイルを照合する場合は次のように指定します。

```bash
bash "$fw/scripts/test.sh" --board-dir /path/to/hardware/kicad/kart_bridge_board_rev6
```

`JPBB_REV6_BOARD_DIR` 環境変数でも指定でき、その配下の `review/netlist.xml` と `rev6_interface_contract.json` を読みます。明示したファイルがない場合やGPIOが一致しない場合は失敗します。headerは常に取り込み先の `include/rev6_hal.h` を照合します。

ARM向けビルドには別途Arm GNU Toolchain、CMake、Ninja、Python3が必要です。公式[Pico SDK 2.3.1](https://github.com/raspberrypi/pico-sdk/releases/tag/2.3.1)を commit `079c6f39023649b154152db30f1d781e884879bc`、TinyUSBを `86ad6e56c1700e85f1c5678607a762cfe3aa2f47` に固定しています。SDKの取得は明示的な `bootstrap.sh` だけが行い、`build.sh` は既存SDKを検査して使います。

```bash
bash "$fw/scripts/bootstrap.sh"
bash "$fw/scripts/build.sh"
```

`CMAKE`、`NINJA`、`ARM_GCC_PATH`（gccのあるbinディレクトリ）、`PICO_SDK_PATH`でツール位置を指定できます。SDKは既定で `pico_rev6/.cache/pico-sdk/`、ARM出力は `pico_rev6/build/pico/`（`JPBB_PICO_BUILD_DIR` で変更）です。SDK・toolchain・build成果物はGitに含めません。既存SDKが固定commitと異なる場合は、その内容を置換せず停止します。UF2はSDK生成binからRP2040専用変換を行い、family ID・書き込みアドレス・全payloadの一致を検証します。Pico 2やPico W用には使いません。USB VID/PIDはTinyUSBの開発例CAFE:4002です。ビルド処理は書き込みを実行しません。

チャネルと向きは`include/rev6_core.h`の`REV6_SBUS_*`設定で変更できます。番号は0始まり（0＝CH1）。初期raw端点172／992／1811は仮の校正値で、実受信機の端点を確認してください。SPIの速度・mode・ODR・FIFO・サンプル時刻はIMU選定後に確定します。

ATDのホストテストはcore・sbus・transport・JPB1 protocol・CH入替/反転・UF2異常系を実行し、電源確認・USB/受信喪失・中立再arm・timeout境界・時計wrap・過長行破棄・RPM周期失効を確認します。外部boardを指定したときだけ最終ネットリストとのGPIO照合も実行します。旧STM32 control/layoutテストは移設版の対象に含めません。元READMEではmac環境でASan実行が停止したと記録されており、ASanの合格は示していません。ホストテストの合格は、実機の電源・通信・停止動作を検証したことにはなりません。

実機で確定すべき項目と記録欄は[初回確認](BENCH_CHECK.md)を参照してください。
