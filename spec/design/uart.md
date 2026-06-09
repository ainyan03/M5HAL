# design/uart — UART

UART kind は I2C / SPI と同じく `Bus` / `Accessor` / `Source` / `Sink`
の形で扱う。初期実装は同期 read/write に絞り、
TX/RX channel 分離 lock や async executor 連携は後続段階で扱う。

## 型

```cpp
namespace m5::hal::v1::uart {

enum class Parity : uint8_t {
    none,
    even,
    odd,
};

struct UARTBusConfig : public bus::BusConfig {
    gpio_number_t pin_tx;
    gpio_number_t pin_rx;
    gpio_number_t pin_rts;
    gpio_number_t pin_cts;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
};

struct UARTAccessConfig : public bus::AccessConfig {
    uint32_t baud_rate;
    uint32_t timeout_ms;             // bus lock timeout
    uint32_t first_byte_timeout_ms;  // read first byte wait
    uint32_t inter_byte_timeout_ms;  // read continuation wait
    uint32_t write_timeout_ms;       // physical TX drain wait
    uint8_t data_bits;
    uint8_t stop_bits;
    parity_t parity;
    bool invert;
};

class UARTAccessor : public bus::Accessor {
    expected<size_t, error_t> write(data::ConstDataSpan tx);
    expected<size_t, error_t> write(data::Source& tx, size_t len);
    expected<size_t, error_t> write(const uint8_t* tx, size_t len);

    expected<size_t, error_t> read(data::DataSpan rx);
    expected<size_t, error_t> read(data::Sink& rx, size_t len);
    expected<size_t, error_t> read(uint8_t* dst, size_t len);

    expected<size_t, error_t> readableBytes();
};

}
```

`timeout_ms` は I2C / SPI と同じく Accessor が `beginAccess()` で使う
bus lock timeout。UART の受信待ちは `first_byte_timeout_ms` と
`inter_byte_timeout_ms`、送信 drain 待ちは `write_timeout_ms` で表す。

## read semantics

`read(rx, len)` は最大 `len` byte を `Sink` に書き込む。最初の byte を
待つ時間は `first_byte_timeout_ms`、1 byte 以上受けた後に次 byte を待つ
時間は `inter_byte_timeout_ms`。timeout は正常な短い read として扱い、
それまでに受信した byte 数を返す。

## write semantics

`write(tx, len)` は最大 `len` byte を `Source` から送信する。戻り値は driver
へ受け渡した byte 数。ESP-IDF backend は送信後に `uart_wait_tx_done()` を
呼び、`write_timeout_ms` 以内に物理 TX FIFO が drain されることを待つ。
Arduino backend は `HardwareSerial::flush()` を使う。

## variants

- `variants::frameworks::arduino` は `HardwareSerial` に委譲する。
  Arduino ESP32 では `Serial` が USB CDC (`HWCDC` / `USBCDC`) になる設定が
  あるため、`port_num` から `Serial` を内部解決しない。variant 固有の
  `uart::BusConfig` に caller-owned `HardwareSerial*` を明示して渡す。
  `attach(HardwareSerial&)` でも同じく caller-owned serial を利用できる。
- `variants::frameworks::espidf` は ESP-IDF UART driver
  (`uart_driver_install`, `uart_read_bytes`, `uart_write_bytes`) に委譲する。
  variant 固有の `uart::BusConfig` は `port_num` を持ち、負値の場合は
  `UART_NUM_0` を既定値にする。IDF 4 / 5 / 6 の代表 build で確認する。
- `variants::frameworks::posix` は POSIX host の termios serial に委譲する。
  variant 固有の `uart::BusConfig` は `device_path`（`/dev/ttyUSB0` 等）を持つ。
  `open(device_path, baud)` で device を所有開放し、`attach(int fd)` で
  caller-owned fd（pty の片端など）を採用する。termios は最初の write/read で
  per-access `UARTAccessConfig` から適用（baud / 8bit / stop / parity）、timeout は
  `select()` で実装する。read/write 以外の line は raw（`cfmakeraw`）。素の POSIX
  host で既定有効の opt-out（`M5HAL_DISABLE_POSIX` で抑止）。高速 baud: Linux glibc/musl は
  `B460800`〜`B4000000` の定数経由、 macOS は B230400 超を `IOSSIOSPEED` ioctl で任意 baud 設定
  （いずれも 3 Mbaud を実機実績 — Core BASIC v2.7 の CH9102 ↔ MacBook）。round-trip は pty を
  使った native gtest で検証 (`test/v1/native/bus/test_posix_uart`)。

## 今後

- TX/RX channel を別々に lock する UART-specific session。
- loopback example / experiment による実機確認。
- remote transport との接続。
