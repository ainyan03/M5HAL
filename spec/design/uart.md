# design/uart — UART

UART kind は I2C / SPI と同じく `Bus` / `Accessor` / `Source` / `Sink`
の形で扱う。ただし UART は full-duplex stream なので、TX と RX を
別 channel として lock する。初期実装は同期 read/write に絞り、
async executor 連携は後続段階で扱う。

## 型

```cpp
namespace m5::hal::v1::uart {

enum class Parity : uint8_t {
    none,
    even,
    odd,
};

enum class Channel : uint8_t {
    none,
    tx,
    rx,
    txrx,
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

class UARTTxAccessor : public bus::Accessor, public data::StreamWriter {
    expected<size_t, error_t> write(data::ConstDataSpan tx) override;  // StreamWriter
    expected<size_t, error_t> write(data::Source& tx, size_t len);
    expected<size_t, error_t> write(const uint8_t* tx, size_t len);
};

class UARTRxAccessor : public bus::Accessor, public data::StreamReader {
    expected<size_t, error_t> read(data::DataSpan rx) override;  // StreamReader
    expected<size_t, error_t> read(data::Sink& rx, size_t len);
    expected<size_t, error_t> read(uint8_t* dst, size_t len);

    expected<size_t, error_t> readableBytes() override;  // StreamReader
};

class UARTAccessor {
    UARTTxAccessor& tx();
    UARTRxAccessor& rx();

    expected<size_t, error_t> write(data::ConstDataSpan tx);
    expected<size_t, error_t> read(data::DataSpan rx);
    expected<size_t, error_t> readableBytes();
};

}
```

`timeout_ms` は `UARTTxAccessor::beginTxAccess()` /
`UARTRxAccessor::beginRxAccess()` / facade の `UARTAccessor::beginAccess()`
で使う channel lock timeout。UART の受信待ちは
`first_byte_timeout_ms` と `inter_byte_timeout_ms`、送信 drain 待ちは
`write_timeout_ms` で表す。

## channel semantics

`UARTBus` は `Channel::tx` と `Channel::rx` を別々に lock する。
`UARTTxAccessor` は TX channel のみ、`UARTRxAccessor` は RX channel のみを
開く。これにより write と read は同時に進められる。同一 channel に対する
複数 owner の競合は `BUSY`。

`UARTAccessor` は `UARTTxAccessor` と `UARTRxAccessor` を内包する
convenience facade。`UARTAccessor::beginAccess()` は `txrx` を開き、
`write()` / `read()` sugar はそれぞれ内包する split accessor へ委譲する。
設計上の主 API は split accessor 側とし、facade は単純なコマンド応答型の
利用を短く書くために残す。

split accessor は最小ストリーム I/O インタフェース (`data::StreamReader` /
`data::StreamWriter`) を実装しており、`data::StreamSource` / `data::StreamSink`
アダプタを介して Source / Sink としても消費できる (frame codec
([frame.md](frame.md)) 等の Source/Sink consumer との接続点。契約は
[data_io.md](data_io.md) §Stream アダプタ)。

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

## error semantics

- API contract 違反（bus kind 不一致、baud/data bits/stop bits/parity の未対応値、POSIX の null path 等）は `INVALID_ARGUMENT`。
- 受信 timeout は `TIMEOUT_ERROR` ではなく正常な短い read（0 byte または partial）として扱う。ESP-IDF backend の TX drain timeout は driver が timeout として返すため `TIMEOUT_ERROR`。
- POSIX の `open` / `termios` / `select` / `read` / `write` 失敗、および ESP-IDF driver の未分類 `esp_err_t` は transport/OS/driver 障害として `IO_ERROR`。
- driver install や一時 buffer 確保など bounded resource の不足は `OUT_OF_RESOURCE`。

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
  host で既定有効の opt-out（`M5HAL_CONFIG_POSIX_UART=0` で抑止）。高速 baud: Linux glibc/musl は
  `B460800`〜`B4000000` の定数経由、 macOS は B230400 超を `IOSSIOSPEED` ioctl で任意 baud 設定
  （いずれも 3 Mbaud を実機実績 — Core BASIC v2.7 の CH9102 ↔ MacBook）。round-trip は pty を
  使った native gtest で検証 (`test/v1/native/bus/test_posix_uart`)。
  ポート列挙ユーティリティ `listSerialPorts` / `rankSerialPortName` (`hal/uart/ports.hpp`) が
  ホストのシリアルポート候補を有力順に返す (macOS: `cu.usbserial*`/`cu.usbmodem*` 優先、
  Linux: `/dev/serial/by-id` 優先)。名前ヒューリスティクスは候補出しまでで、確定は呼び出し側の
  プロトコル疎通で行う — リモートバスの確立は `connectRemoteSerial`
  (`hal/uart/remote_connect.hpp`、[remote.md](remote.md) §接続ユーティリティ) がこの列挙と
  `hello` を合成する。

## 今後

- TX/RX channel を別々に lock する UART-specific session。
- loopback example / experiment による実機確認。
- remote transport との接続。
