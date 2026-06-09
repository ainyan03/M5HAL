# uart_echo — HIL: host POSIX UART variant ↔ ESP32 echo

ホストの M5HAL **posix UART variant** から送ったバイト列が、実機（M5Stack Core BASIC）上で
**M5HAL UART** が駆動するエコー firmware を経由して **byte 完全一致で戻る**ことを検証する。
両端とも M5HAL UART。

- `device/uart_echo.cpp` — UART0（GPIO1=TX / GPIO3=RX = USB ブリッジ）を M5HAL UART で駆動し、
  受信をそのまま返すだけの firmware。Arduino `Serial` のログ出力は無し（USB 経路を綺麗なエコー
  パイプにする）。**追加配線は不要**（USB ケーブルがそのままエコー経路）。
- `host/uart_echo.cpp` — `hil_host.hpp` でポートを開き（DTR/RTS を落として自動リセット抑制 →
  ブート待ち → 同期マーカーで疎通確認）、small / binary(NUL+LF含む) / 512B のエコーを照合する gtest。

## 配線

無し（M5Stack Core BASIC を USB で接続するだけ）。エコー経路は USB ブリッジ UART0。

## 実行

```sh
experiments/v1/test/hil-run.sh uart_echo                            # 115200
experiments/v1/test/hil-run.sh uart_echo /dev/cu.usbserial-X 3000000  # 3 Mbaud
```

`hil-run.sh` は device を指定 baud で焼き（`-DM5HAL_HIL_ECHO_BAUD`）、host をビルドして
`M5HAL_POSIX_UART_PORT`/`_BAUD` を渡して実行する。

## 期待結果

```
[ PASSED ] UartEcho.EchoesSmallPayload
[ PASSED ] UartEcho.EchoesBinaryIncludingNulAndNewline
[ PASSED ] UartEcho.EchoesLargePayloadInOrder
```

## 実績

- **115200**: Core BASIC で 3/3 PASS。
- **3 Mbaud**: Core BASIC **v2.7（CH9102）** ↔ MacBook で 3/3 PASS（macOS は posix variant が
  `IOSSIOSPEED` で 3M を設定）。CP2104 revision は実効 ~2 Mbaud 上限。

## メモ

- 高速 baud では device の RX リングバッファ既定 256B が 512B バーストで溢れるため、firmware は
  `Serial.setRxBufferSize(2048)` で広げている（バッファ深さ ≥ バースト長が必要）。
- ホスト open 時の USB ブリッジ自動リセット＋ROM ブートログは `hil_host.hpp::openSynced` の
  drain＋同期マーカー再試行で吸収する。
