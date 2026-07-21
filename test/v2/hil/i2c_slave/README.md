# ESP-IDF I2C slave HIL

CoreS3上でM5HAL I2C slaveを動かし、独立したmasterから駆動する2台fixtureである。POSIX hostは使わず、
masterのserial出力を合否の正本とする。

## 配線

| 信号 | Core2 master | CoreS3 slave |
|---|---:|---:|
| SDA | GPIO32 | GPIO2 |
| SCL | GPIO33 | GPIO1 |
| GND | 共通 | 共通 |

slave addressは `0x42`。masterは内部pull-upを有効化するが、800 kHz passでは外付け2.2–4.7 kOhmを
推奨する。すべてのコマンドに対してconfigを一度設定する。

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
```

## Register-map sweep

CoreS3のregister-map deviceをflashし、Core2には独立masterのどちらかをflashする。

```sh
pio run -e v2_hil_i2c_slave_device_esp32s3 \
  -t upload --upload-port <CoreS3-port>

# Arduino WireまたはESP-IDF i2c_masterのどちらかを選ぶ:
pio run -e v2_hil_i2c_regmap_sweep_master_arduino_esp32 \
  -t upload --upload-port <Core2-port>
pio run -e v2_hil_i2c_regmap_sweep_master_idf_esp32 \
  -t upload --upload-port <Core2-port>

pio device monitor --port <Core2-port> -b 115200
```

masterはSPLIT/repeated-START read、write/read境界、wraparound、長いtransfer、length sweepを
100、400、800 kHzで検証する。最終行にbad transactionがなければ合格である。

```text
RESULT: ok=... bad=0 stress_bad=0/600 (PASS)
```

## Stream echo

stream deviceをflashし、M5HAL masterまたは独立vendor masterを選ぶ。

```sh
pio run -e v2_hil_i2c_slave_echo_device_esp32s3 \
  -t upload --upload-port <CoreS3-port>

# M5HAL master:
pio run -e v2_hil_i2c_echo_master_esp32 \
  -t upload --upload-port <Core2-port>

# または独立vendor master:
pio run -e v2_hil_i2c_echo_sweep_master_arduino_esp32 \
  -t upload --upload-port <Core2-port>
pio run -e v2_hil_i2c_echo_sweep_master_idf_esp32 \
  -t upload --upload-port <Core2-port>

pio device monitor --port <Core2-port> -b 115200
```

masterはFIFO/ring境界を跨ぐ長さとpatternでraw payloadをwrite/readする。合格条件は `BAD` roundが
0であり、独立masterは次の最終行を出す。

```text
RESULT: ok=... bad=0 (PASS)
```

classic ESP32のArduino `Wire` masterは255-byte readまでに制限される。1024-byteを含む全受入には
ESP-IDF masterを使う。

## Queue-driven lifecycle

この組はcaller-owned queue、分割pure read、RX drain、有限access終了、close/reinitializeによる再利用を
検証する。

```sh
pio run -e v2_hil_i2c_slave_queue_device_esp32s3 \
  -t upload --upload-port <CoreS3-port>
pio run -e v2_hil_i2c_slave_queue_master_idf_esp32 \
  -t upload --upload-port <Core2-port>
pio device monitor --port <Core2-port> -b 115200
```

両endpointで次の出力を確認する。

```text
RESULT: checks=10 failures=0 (PASS)
RESULT: PASS
```

## 別chipの配線

[`../../../../pio_envs/v2/hil.ini.cli`](../../../../pio_envs/v2/hil.ini.cli) には同じprogramを使う別chipの
device/master envもある。Core2/CoreS3の配線を推測で流用せず、configのpin flagに従う。C6/H2の組は
両方SDA=GPIO2/SCL=GPIO1、C61 deviceはSDA=GPIO5/SCL=GPIO6である。

別chip envもregister-mapの同じresult lineを使う。fault injectionによる比較とboard固有の調査は、
再利用可能な受入手順の対象外とする。
