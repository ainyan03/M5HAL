# i2c_slave — HIL: M5HAL I2C slave (clock-stretch backend) ↔ verification master

公開 M5HAL の **I2C slave**（`SlaveBus_espidf` LL クロックストレッチ backend）を実機で受け入れる。
CoreS3SE（ESP32-S3）を slave（addr 0x42）として動かし、別ボードの master がデータをスイープして
**byte 完全一致（BAD=0）** を確認する。

> **このテストは他の HIL と構成が違う**: uart_echo 等は「POSIX host ↔ 実機 1 台」だが、これは
> **実機 2 台（slave + master）で POSIX host を持たない**。判定は master のシリアル出力（OK/BAD）を
> 人間が読む。`hil-run.sh` は使わない。

## 構成

device firmware は 2 種類:

| device | accessor | master が確かめること |
|---|---|---|
| `device/i2c_slave.cpp` | `SlaveRegMapAccessor`（`SlaveStreamAccessor` の上に compose） | 256B register file を write-then-read（WTR / SPLIT / WTEST）で完全一致 |
| `device/i2c_echo.cpp` | `SlaveStreamAccessor`（直接） | **最大 1 KB のストリーム write→read 往復**が全データパターンで完全一致 |

echo device は **TX ストリーミング経路（read > `kTxCapacity`=64 → TX_EMPTY underrun stretch）** と
**RX リング（write > `kRxCapacity`=64 → RX_FULL stretch）** を端から端まで実機で叩く。

master firmware は 3 種類。**M5HAL 自身の master API を通す経路**と、**vendor トランスポートを直叩き
する経路**を分けてあるのは、M5HAL の master/slave 両実装が共有し得るバグを前者だけでは検出できない
ため。後者は「M5HAL 非依存の Arduino ライブラリ等がバスを共有しても、M5HAL が給仕する slave と正しく
通信できる」ことの証明になる。

| master | 経路 | 対象 device |
|---|---|---|
| `master/i2c_echo_master.cpp` | M5HAL master API（`Bus_espidf` + `MasterAccessor`） | `i2c_echo.cpp` |
| `master/i2c_regmap_sweep_master.cpp` | vendor 直叩き（Arduino `Wire` / ESP-IDF `i2c_master`） | `i2c_slave.cpp` |
| `master/i2c_echo_sweep_master.cpp` | vendor 直叩き（同上） | `i2c_echo.cpp` |

`device/i2c_slave.cpp` は **ESP-IDF app**（framework=espidf）。LL backend は IDF 5.x の stretch-cause
caps + `i2c_ll_*` を要求し、プロジェクトの Arduino platform = IDF 4.4 では動かない。

## 配線（2 台 HIL）

| 信号 | Core2 master | CoreS3SE slave |
|---|---|---|
| SDA | GPIO32 | GPIO2 |
| SCL | GPIO33 | GPIO1 |
| GND | 共通 | 共通 |

master は内部プルアップを有効化するが、800 kHz の確実性のため外付け ~2.2–4.7k を推奨。

## 実行 — レジスタマップ・フルスイープ

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli

# 1. slave（CoreS3SE）
pio run -e v2_hil_i2c_slave_device_esp32s3 -t upload --upload-port <CoreS3SE のポート>

# 2. master（Core2）— Arduino Wire 版 / ESP-IDF i2c_master 版のどちらか
pio run -e v2_hil_i2c_regmap_sweep_master_arduino_esp32 -t upload --upload-port <Core2 のポート>
pio run -e v2_hil_i2c_regmap_sweep_master_idf_esp32    -t upload --upload-port <Core2 のポート>

# 3. master のシリアルを開いて OK/BAD を読む
pio device monitor --port <Core2 のポート> -b 115200
```

検証項目: SPLIT read / WTR（repeated-START）read / window-base（データ書込み直後の純粋読出し）/
repeat-read / 0xFF wrap / 96B 連続 read・write / 間隔ゼロ WTR ストレス 200 回（T1-T10）に加え、
**WTR read 長スイープ（T11）/ SPLIT read 長スイープ（T12）/ WRITETEST 相当の write-then-verify
スイープ（T13、k=1..48）** で長さ次元（n=1..64 / k=1..48）を直接掃引する。100/400/**800** kHz の
3 パスで実行する。

**受入** = `RESULT: ok=... bad=0 stress_bad=.../600 (PASS)`（T1-T14 × 100/400/800 kHz の 3 パス）。
取引ごとの行は次の形:

```
#123 WTR   req=0x40 n=33 got=40 41 42 43 first_bad=-1 [OK]  err=ESP_OK elapsed=...us
```

96B の read・write はともに **1 取引**で行う。read は `serve()` が `kReplyWindowBytes`=64 のチャンク
単位で継続 compose しながらストリームする（8bit wrap の auto-increment、上限なし）。

**テストデータの作り方**: すべて 256 と互いに素な奇数ストライドか加算カウンタで生成し（mod 256 で
全単射）、検証窓内に同一値が存在しないようにする。バイトのズレ・重複・欠落・stale がすべて全バイト
不一致として検出できる（同一値の連続データではこれらが照合をすり抜ける）。T11-T13 の
(register, length) 対応は `kSweepRegSeq` のコメントを参照。

**`device/i2c_slave.cpp` との意味論差分**: レジスタ 0xFF は device 側で読み出す度に値が変わるライブ
カウンタ（`onReadRegister` の `COUNTER_REG` 分岐）で、書いた値をそのまま返す静的なレジスタファイル
ではない。write→verify モデルと噛み合わないため、`verify()` は 0xFF だけを比較対象から除外する
（他の全レジスタはバイト完全一致で検証する）。

## 実行 — ストリーム echo（最大 1 KB 往復）

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli

# 1. echo slave（CoreS3SE）
pio run -e v2_hil_i2c_slave_echo_device_esp32s3 -t upload --upload-port <CoreS3SE のポート>

# 2a. master（Core2）— M5HAL master API 経由
pio run -e v2_hil_i2c_echo_master_esp32 -t upload --upload-port <Core2 のポート>

# 2b. master（Core2）— vendor トランスポート直叩き
pio run -e v2_hil_i2c_echo_sweep_master_arduino_esp32 -t upload --upload-port <Core2 のポート>  # Wire
pio run -e v2_hil_i2c_echo_sweep_master_idf_esp32     -t upload --upload-port <Core2 のポート>  # i2c_master

# 3. master のシリアルを開いて OK/BAD を読む
pio device monitor --port <Core2 のポート> -b 115200
```

master は register byte を付けず **純粋なバイト列**を N バイト write→（STOP）→N バイト read し、
slave がそのまま echo した内容と完全一致（`first_bad=-1`）を確認する。長さリストは 1..1024B
（リング/FIFO 境界を跨ぐ）× 5 パターン（zero / ones / count / rand / bound）× 100/400/800 kHz。

**受入** = 全長・全パターンで BAD=0、特に **len=1024 の往復が clean**。

> **Arduino `Wire` 版は 255B まで**: classic ESP32 の legacy Wire ドライバは 256B 以上の read で内部的に
> mod 256 の桁あふれを起こす（ベンダードライバ側の制約で、call site の型では回避できない）。IDF
> `i2c_master` 版は 1KB まで検証する。

## ESP32-C61 リグ（クロスチップ・スモーク）

device は `v2_hil_i2c_slave_device_esp32c61`（pioarduino platform、SDA=GPIO5 / SCL=GPIO6）。master に
M5Stack BASIC を使う場合は Port A のピンを build flag で上書きする:

```sh
pio run -e v2_hil_i2c_slave_device_esp32c61 -t upload --upload-port <C61 のポート>
PLATFORMIO_BUILD_FLAGS="-DMASTER_PIN_SDA=21 -DMASTER_PIN_SCL=22" \
  pio run -e v2_hil_i2c_regmap_sweep_master_arduino_esp32 -t upload --upload-port <BASIC のポート>
```

## ESP32-C6 / ESP32-H2 リグ（controller clock コールドブート A/B）

C6/H2 は起動コードが I2C 機能クロックを明示的に gate off する（PCR `i2c_sclk_en`）ため、`init()` の
controller clock 有効化が効いていることの**決定的証拠はこのリグでのみ得られる**（S3/C61 はリセット値に
救われて未修正でも動く）。リグ = M5 NanoC6 ↔ NanoH2、Grove Port A 直結、両機 SDA=GPIO2 / SCL=GPIO1。

`slave.inl` の diag ノブ `-DM5HAL_I2C_SLAVE_NO_CONTROLLER_CLOCK` を注入したビルドと現行ビルドを
比べる。前者は 100kHz を含む全速度で全滅し、後者は PASS する。

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
PLATFORMIO_BUILD_FLAGS="-DM5HAL_I2C_SLAVE_NO_CONTROLLER_CLOCK" \
  pio run -e v2_hil_i2c_slave_device_esp32c6 -t upload --upload-port <C6 のポート>
pio run -e v2_hil_i2c_regmap_sweep_master_idf_esp32h2 -t upload --upload-port <H2 のポート>
# 方向を入れ替える場合は _esp32h2 device / _esp32c6 master で同型
```

## 診断ビルド（受入ランでは使わない）

echo device は 2 つの build flag で障害帯を人工的に再現できる。**受入ランでは絶対に定義しない**。

| flag | 再現するもの |
|---|---|
| `M5HAL_TEST_ECHO_SERVE_DELAY_MS=<ms>` | consumer 微遅延。STOP 時に RX ring 満杯 + HW FIFO 残尻尾という back-pressure の効かない帯（33〜64B write）を露出させる。長さリストの 33/40/47/48 はこの帯の恒久回帰カバレッジ |
| `M5HAL_TEST_ECHO_SERVE_STALL_EVERY=<N>` + `..._STALL_MS=<ms>` | N serve ごとに 1 回、master の HW SCL timeout（Wire 13〜20ms / IDF gen5 2ms）を超える stall を注入。sweep 全体が「abort → 復元」の反復検査になる |

stall ノブの受入 = BAD が stall 継続時間内の取引だけの孤立塊に収まり（塊長 ≤ stall/ラウンド所要で
説明可能な範囲）、塊間が全 OK であること（尾引き = 会計の復元失敗）。

> IDF gen5 master は HW SCL timeout 2ms のため、`SERVE_DELAY_MS` ノブとは併用できない（全取引が
> タイムアウトする）。

## ビルドゲート（HW 不要）

`v2_hil_i2c_slave_device_esp32s3` / `v2_hil_i2c_slave_echo_device_esp32s3` はいずれも device 単体で
ビルド可能なので、`local-ci.sh`（full / hil）の `check_hil()` が両 device を compile fence として
ビルドし、`check-iram-isr.sh` で ISR 到達シンボルが IRAM 常駐であることを確認する。

## 設計メモ

- **stretch-hold-until-write**: backend は read で「空なら stretch を保持して待つ」プリミティブ。
  app（accessor ループ）が write で応答を供給した時点で stretch を解除する。これにより**ポーリング
  型 accessor でも tight write-then-read を給仕できる**（master は SCL ストレッチで待たされる）。
- **read 給仕の契約**: write 後に `transactionComplete()` が真になるまで取引を開いたまま待つ。応答は
  取引の tx キューに在り、backend が長い read の TX_EMPTY ストレッチごとに 32B FIFO を再充填する。
  早く end すると read 途中で応答が破棄される。
- **pure-read の取引確保**: SPLIT の第 2 取引（reg 書込と read が別取引）は書き込み相が無いため、
  backend は read アドレス一致時にも取引を確保する（さもなくば accessor が開ける取引が無い）。
