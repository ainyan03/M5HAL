# i2c_slave — HIL: M5HAL I2C slave (clock-stretch backend) ↔ verification master

公開 M5HAL の **I2C slave**（`SlaveBus_espidf` LL クロックストレッチ backend）を実機で受け入れる。
CoreS3SE（ESP32-S3）を slave（addr 0x42）として動かし、別ボードの master がデータをスイープして
**byte 完全一致（BAD=0）** を確認する。device firmware は 2 種類:

| device | accessor | master が確かめること |
|---|---|---|
| `device/i2c_slave.cpp` | `SlaveRegMapAccessor`（`SlaveStreamAccessor` の上に compose） | 256B register file を write-then-read（WTR / SPLIT / WTEST）で完全一致 |
| `device/i2c_echo.cpp` | `SlaveStreamAccessor`（直接） | **最大 1 KB のストリーム write→read 往復**が全データパターンで完全一致 |

echo device は **TX ストリーミング経路（read > `kTxCapacity`=64 → TX_EMPTY underrun stretch）** と
**RX リング（write > `kRxCapacity`=64 → RX_FULL stretch）** を端から端まで実機で叩く。これが espidf
TX ストリーミング（native/IRAM 緑だが HW 未検証）の実機検証になる。

> **このテストは他の HIL と構成が違う**: uart_echo 等は「POSIX host ↔ 実機 1 台」だが、これは
> **実機 2 台（slave + master）で POSIX host を持たない**。master は別 firmware（下記）で、判定は
> master のシリアル出力（OK/BAD）を人間が読む。`hil-run.sh` は使わない。

- `device/i2c_slave.cpp` — CoreS3SE 上の slave firmware。`SlaveBus_espidf` を
  `tx_underrun=stretch` で init し、`SlaveStreamAccessor` のポーリングループ（begin → 書き込み相を
  drain → register file から応答を compose → write → **完了まで保持** → end）で 256B register file
  （`reg_file[i]==i`、ポインタ auto-increment、取引跨ぎで保持）を給仕する。**ESP-IDF app**
  （framework=espidf。LL backend は IDF 5.x の stretch-cause caps + `i2c_ll_*` を要求し、プロジェクトの
  Arduino platform = IDF 4.4 では不可）。
- **master**（別ツリーの実験 firmware）— プロジェクトの私的 experiments ツリーにある bench
  firmware（公開対象外）を `-DROLE_MASTER` 相当でビルドして Core2（classic ESP32）へ焼く。
  WTR / SPLIT / WTEST を 100/400/800 kHz・n=1..64 でスイープし、取引ごとに OK/BAD を print する。

## 配線（2 台 HIL）

| 信号 | Core2 master | CoreS3SE slave |
|---|---|---|
| SDA | GPIO32 | GPIO2 |
| SCL | GPIO33 | GPIO1 |
| GND | 共通 | 共通 |

master は内部プルアップを有効化するが、800 kHz の確実性のため外付け ~2.2–4.7k を推奨。

## 実行

```sh
# 1. slave（CoreS3SE）を焼く
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
pio run -e v2_hil_i2c_slave_device_esp32s3 -t upload --upload-port <CoreS3SE のポート>

# 2. master（Core2）を焼く（別ツリーの私的 bench firmware、ROLE_MASTER 相当）。
#    モード（WTR / SPLIT / WTEST）は build flag / env で選ぶ
pio run -e <esp32_master_*> -t upload --upload-port <Core2 のポート>

# 3. master のシリアルを開いて OK/BAD を読む（100|400|800 でクロック切替コマンドあり）
pio device monitor --port <Core2 のポート> -b 115200
```

## 期待結果（受入基準 = §✅ マトリクス）

master のログが全取引 `[OK]`（`first_bad=-1`）、`BAD` が出ないこと。WTR / SPLIT / WTEST ×
{100,400,800} kHz × n=1..64 で BAD=0。

```
#123 WTR   req=0x40 n=33 got=40 41 42 43 first_bad=-1 [OK]  err=ESP_OK elapsed=...us
```

## ストリーム echo の実行（最大 1 KB 往復）

```sh
# 1. echo slave（CoreS3SE）を焼く
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
pio run -e v2_hil_i2c_slave_echo_device_esp32s3 -t upload --upload-port <CoreS3SE のポート>

# 2. echo master（Core2）を焼く（私的 bench、ROLE_MASTER -DECHO_MODE）
pio run -e esp32_master_echo -t upload --upload-port <Core2 のポート>

# 3. master のシリアルを開いて OK/BAD を読む（len=1..1024 × {zero,ones,count,rand,bound}）
pio device monitor --port <Core2 のポート> -b 115200
```

master は register byte を付けず **純粋なバイト列**を N バイト write→（STOP）→N バイト read し、
slave がそのまま echo した内容と完全一致（`first_bad=-1`）を確認する。受入 = 全長・全パターンで
BAD=0、特に **len=1024 の往復が clean**。一括実行スクリプトは私的 experiments ツリー側にある。

## ビルドゲート（HW 不要）

`v2_hil_i2c_slave_device_esp32s3` / `v2_hil_i2c_slave_echo_device_esp32s3` はいずれも device 単体で
ビルド可能なので、`local-ci.sh`（full / hil）の `check_hil()` が両 device を compile fence として
ビルドし、`check-iram-isr.sh` で ISR 到達シンボルが IRAM 常駐であることを確認する（master ツリーは不要）。

## 設計メモ

- **stretch-hold-until-write**: backend は read で「空なら stretch を保持して待つ」プリミティブ。
  app（accessor ループ）が write で応答を供給した時点で stretch を解除する。これにより**ポーリング
  型 accessor でも tight write-then-read を給仕できる**（master は SCL ストレッチで待たされる）。
- **read 給仕の契約**: write 後に `transactionComplete()` が真になるまで取引を開いたまま待つ。応答は
  取引の tx キューに在り、backend が長い read の TX_EMPTY ストレッチごとに 32B FIFO を再充填する。
  早く end すると read 途中で応答が破棄される。
- **pure-read の取引確保**: SPLIT の第 2 取引（reg 書込と read が別取引）は書き込み相が無いため、
  backend は read アドレス一致時にも取引を確保する（さもなくば accessor が開ける取引が無い）。
