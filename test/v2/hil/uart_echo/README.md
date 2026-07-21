# UART echo HIL

POSIX UART variantから送ったpayloadを、M5HAL UART echo firmwareを動かすESP32経由で返し、
byte完全一致を検証する。

## Hardware

- device: M5Stack Core BASIC
- link: onboard USB bridge経由のUART0、既定115200 baud
- 追加配線: なし

firmwareは `Serial` にlogを出さず、USB-UART経路をechoに使用する。

## 実行

```sh
test/v2/hil/hil-run.sh uart_echo
test/v2/hil/hil-run.sh uart_echo /dev/cu.usbserial-X 3000000
```

runnerは指定baudでdeviceをflashし、hostをbuildして、host programへ `M5HAL_POSIX_UART_PORT` と
`M5HAL_POSIX_UART_BAUD` を渡す。

## 合否

```text
[ PASSED ] UartEcho.EchoesSmallPayload
[ PASSED ] UartEcho.EchoesBinaryIncludingNulAndNewline
[ PASSED ] UartEcho.EchoesLargePayloadInOrder
```

device firmwareは512-byte burst用にRX bufferを拡張している。高速baudではUSB-UART bridgeとhostの
serial実装も指定rateへ対応している必要があり、その制約による失敗だけではM5HALの回帰を示さない。
