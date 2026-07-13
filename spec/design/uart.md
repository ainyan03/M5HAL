# design/uart — UART

> **読者**: 実装者・レビュー向け（設計仕様）。

UART kind は I2C / SPI と同じく `Bus` / `Accessor` / `Source` / `Sink`
の形で扱う。ただし UART は full-duplex stream なので、TX と RX を
別 channel として lock する。初期実装は同期 read/write に絞り、
async executor 連携は後続段階で扱う。

## Bus の入手と所有

共通機構は [bus_accessor.md](bus_accessor.md) §Bus の保持 を参照。本 kind 固有の差分のみ以下に示す。

- **identity = TX / RX**（両必須）。TX-only / RX-only は直接構築で。
- RTS / CTS と buffer size は identity 外だが、同一 TX/RX identity の typed acquire で値が食い違う場合は
  `INVALID_STATE` を返す (既存 bus は再構成しない)。backend 固有 selector (ESP-IDF `port_num`、
  POSIX `device_path` 等) は現行 identity では表現しないため、初回 acquire の config が有効。
- UART は **static-backend policy**。`commitBuses()` は no-op、`hardwareInUse()` は 0。
  `acquire(LogicalBusConfig)` は I2C/SPI と同じ surface と validation を持つが、現時点では
  有効な logical request に `NOT_IMPLEMENTED` を返す。bus 生成は `acquire<CfgT>(cfg)` が担い、
  backend は初回 acquire の config 型で固定される。

```cpp
auto uart = M5_Hal.UART.acquire(m5hal::uart::BusConfig{m5hal::uart::Tx{17}, m5hal::uart::Rx{16}});
```

## 型

```cpp
namespace m5::hal::v2::uart {

enum class Parity : uint8_t {
    None,
    Even,
    Odd,
};

enum class Channel : uint8_t {
    None,
    Tx,
    Rx,
    TxRx,
};

struct Tx { gpio_number_t value; };  // explicit ctor のみ (暗黙変換なし)
struct Rx { gpio_number_t value; };

struct IBusConfig : public bus::IBusConfig {
    gpio_number_t pin_tx;
    gpio_number_t pin_rx;
    gpio_number_t pin_rts;
    gpio_number_t pin_cts;
    size_t rx_buffer_size;
    size_t tx_buffer_size;

    constexpr IBusConfig(Tx, Rx);  // タグ型 ctor (順不同の 2 順列)
    constexpr IBusConfig(Rx, Tx);
};

struct AccessConfig : public bus::IAccessConfig {
    uint32_t baud_rate;
    uint32_t first_byte_timeout_ms;  // read first byte wait
    uint32_t inter_byte_timeout_ms;  // read continuation wait
    uint32_t write_timeout_ms;       // physical TX drain wait
    uint8_t data_bits;
    uint8_t stop_bits;
    parity_t parity;
    bool invert;
};

class TxAccessor : public bus::IAccessor, public data::StreamWriter {
    expected<size_t, error_t> write(data::ConstDataSpan src) override;  // StreamWriter
    expected<size_t, error_t> write(data::Source& src, size_t len);
    expected<size_t, error_t> write(const uint8_t* src, size_t len);
};

class RxAccessor : public bus::IAccessor, public data::StreamReader {
    expected<size_t, error_t> read(data::DataSpan dst) override;  // StreamReader
    expected<size_t, error_t> read(data::Sink& dst, size_t len);
    expected<size_t, error_t> read(uint8_t* dst, size_t len);

    expected<size_t, error_t> readableBytes() override;  // StreamReader
};

class Accessor {
    TxAccessor& tx();
    RxAccessor& rx();

    expected<size_t, error_t> write(data::ConstDataSpan src);
    expected<size_t, error_t> read(data::DataSpan dst);
    expected<size_t, error_t> readableBytes();
};

}
```

pin の設定はタグ型 ctor とフィールド代入の 2 本立て（`uart::BusConfig cfg{uart::Tx{17}, uart::Rx{16}};` 順不同）。
タグは必須 2 本ピン (TX / RX) のみで、RTS / CTS と buffer サイズはフィールド代入で設定する。
**posix variant は例外**として ctor 継承を行わない: 接続の同一性は `device_path` であり
pin フィールドを一切読まないため、タグ構築が「完成した config」に見えてしまう事故を避ける。
規約の詳細 (positional ctor を置かない理由、variant への ctor 継承) は [i2c.md](i2c.md) §pin の設定 と共通。

## channel semantics

`IBus` は `Channel::Tx` と `Channel::Rx` を**別々に lock する**（チャネルごとに `runtime::Mutex` を 1 本持つ）。
`TxAccessor` は TX channel のみ、`RxAccessor` は RX channel のみを開く。
これにより **write と read は同時に進められる**（full-duplex）。
remote proxy でも TX/RX channel の所有権は独立だが、標準 `RemoteSession` は 1 in-flight のため、
個々の wire RPC は共通 session gate で直列化される。1 RPC 内の全二重は複合 transfer が担う
([remote.md](remote.md) §SEQ)。

lock の取得待ちは以下の呼び出し引数で制御する（config には置かない）:
- `TxAccessor::beginAccess(timeout_ms)` — TX channel
- `RxAccessor::beginAccess(timeout_ms)` — RX channel
- `Accessor::beginAccess(timeout_ms)` — 両チャネル (TxRx)

省略 = 無限待ち、0 = 即時 try-lock。同一 channel への複数 owner の競合は
timeout まで待って `TIMEOUT_ERROR`（詳細は [bus_accessor.md](bus_accessor.md) §排他制御の意味論）。

複合 lock (`TxRx`) は TX → RX の順に取得し、2 本目には timeout の残余を充てる
（呼び出し全体として `timeout_ms` を守る）。RX 取得失敗時は取得済みの TX を巻き戻す。

`Accessor` は `TxAccessor` と `RxAccessor` を内包する convenience facade。
`Accessor::beginAccess()` は `TxRx` を開き、`write()` / `read()` sugar はそれぞれ
内包する split accessor へ委譲する。設計上の主 API は split accessor 側とし、facade は
単純なコマンド応答型の利用を短く書くために残す。

split accessor は最小ストリーム I/O インタフェース (`data::StreamReader` / `data::StreamWriter`)
を実装しており、`data::StreamSource` / `data::StreamSink` アダプタを介して Source / Sink としても
消費できる（frame codec ([frame.md](frame.md)) 等の Source/Sink consumer との接続点。契約は
[data_io.md](data_io.md) §Stream アダプタ）。

## state mutex と再設定

TX / RX チャネルロックとは別に、各 variant backend (`Bus_espidf` / `Bus_posix` / `Bus_arduino`) は
config・coalesce 状態を保護する内部 leaf mutex (`runtime::Mutex`) を持つ。ロック取得順序は
**channel lock → state mutex の一方向のみ**。唯一の公認例外は `uart::IBus::tryAcquireOppositeChannel`
(非ブロッキングの反対チャネル静止ゲート) — state mutex 保持中に反対チャネルの mutex を
timeout 0 で試すだけなので、デッドロックし得ない。

再設定 (`applyConfig` に渡された `AccessConfig` が適用済みのものと異なる場合) の意味論:

- **初回適用**（そのバスでまだ何も適用されていない）はゲート不要で、どちらのチャネル入口
  (write / read / readableBytes) から来ても即座に適用する。
- **再設定**は**両チャネルが静止しているときのみ**適用する。反対チャネルが使用中で静止を
  確認できない場合は**適用せず現行設定のまま転送を続行**し、診断カウンタ (`reconfigSkips()`)
  を増やす（相手の転送中に線路設定を書き換えないための安全策）。
  - **静止 (quiescent) の定義**: 反対チャネルが未保持、または保持者が呼び出し元自身か、
    その combined accessor の相方 (`bus::IAccessor::lockPeer` — `uart::Accessor` の
    TX/RX 子同士を指す)。**同一スレッドの無関係な accessor が保持している場合も busy
    として skip する** — 自スレッドが既に保持している mutex への try-lock は POSIX
    `std::timed_mutex` 上で未定義動作となるため構造的に試行できず、また意味論上も
    「反対チャネル使用中」に変わりはないため。

state mutex は `init` / `release` / `attach` と raw I/O の生存期間競合までは守らない —
それは既存の managed facade 契約（アクセスウィンドウ外でのみ再 init / release / attach する）
の管轄で、本節の対象外。

POSIX variant の write coalescing (`tx_coalesce_bytes`) は state mutex 下でバッファに
まとめ書きし、RX 側の `read` / `readableBytes` が呼ばれた際に flush する。flush は
state mutex を保持したまま行うため、TX 側が未 flush のバイトを溜めている間に RX 側が
呼ばれると、その flush が完了するまで（最大 `write_timeout_ms`、既定 1000 ms — この
timeout は flush を要求した RX 側の `AccessConfig::write_timeout_ms` が効く）待たされる。
これは意図したトレードオフである（flush をロック外で行うと TX の write と RX 起点の flush が
同時に raw write することになり、かえって危険なため）。

## read semantics

`read(dst, len)` は最大 `len` byte を `Sink` に書き込む。最初の byte を
待つ時間は `first_byte_timeout_ms`、1 byte 以上受けた後に次 byte を待つ
時間は `inter_byte_timeout_ms`。timeout は正常な短い read として扱い、
それまでに受信した byte 数を返す。

行指向プロトコル (NMEA / AT 応答等) には `RxAccessor::readUntil(delim,
dst, max_len)` を使う (facade にも転送あり)。1 つの RX チャネルロック窓の
中で delimiter まで読み、**delimiter を含めた** byte 数を返す — 「最終
byte が delimiter か」だけで完結行と部分行が区別できる（Arduino の
`readBytesUntil` が delimiter を捨てて両者を区別不能にする不満への回答）。
timeout は部分行（0 を含む短い戻り）で表れ、エラーではない。中核ロジックは
`data::readUntil(StreamReader&, delim, DataSpan)` にあり、UART以外の`StreamReader`にも同じ契約で使える。

## write semantics

`write(src, len)` は最大 `len` byte を `Source` から送信する。戻り値は driver
へ受け渡した byte 数。

**完了保証の正準契約は ESP-IDF backend の意味 (timeout 付き物理 drain 待ち)**。
他 backend は実装手段の制約により完全には一致しない。完全統一は不可能なため、差は
仕様として下表に固定し、この契約表を正本とする:

| backend | write 復帰タイミング | `write_timeout_ms` の扱い |
|---|---|---|
| ESP-IDF | `uart_wait_tx_done()` で物理 TX FIFO の drain まで | drain 待ちの上限。超過は `TIMEOUT_ERROR` |
| Arduino | `HardwareSerial::flush()` で物理送信完了まで | **使われない** (flush に timeout 引数がなく無期限待ち)。利用者は baud と len から所要時間を見積もれる |
| POSIX | OS の tx buffer へ受理された時点 (**drain しない**。await-reply パターンが buffered output でデッドロックしない設計上の選択) | 使われない |

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
  **plain `Stream` 束縛の例外** (`attach(Stream&)` / `setSerial(Stream&)` —
  HWCDC / USBCDC もここに入る): 既に構成済みの byte stream を採用する経路であり、
  線路フォーマット (baud / parity / stop bits / invert) は適用も検証もしない
  (CDC には線路の概念が無く、物理 UART を `Stream` として渡した場合は外部構成が正)。
  per-access 設定で適用されるのは `first_byte_timeout_ms` のみ。線路フォーマットの
  適用・検証契約 (未対応値の `INVALID_ARGUMENT` を含む) は `HardwareSerial` 束縛時のみ有効。
- `variants::frameworks::espidf` は ESP-IDF UART driver
  (`uart_driver_install`, `uart_read_bytes`, `uart_write_bytes`) に委譲する。
  variant 固有の `uart::BusConfig` は `port_num` を持ち、負値の場合は
  `UART_NUM_0` を既定値にする。IDF 4 / 5 / 6 の代表 build で確認する。
- `variants::frameworks::posix` は POSIX host の termios serial に委譲する。
  variant 固有の `uart::BusConfig` は `device_path`（`/dev/ttyUSB0` 等）を持つ。
  `open(device_path, baud)` で device を所有開放し、`attach(int fd)` で
  caller-owned fd（pty の片端など）を採用する。termios は最初の write/read で
  per-access `AccessConfig` から適用（baud / 8bit / stop / parity）、timeout は
  `select()` で実装する。read/write 以外の line は raw（`cfmakeraw`）。素の POSIX
  host で既定有効の opt-out（`M5HAL_CONFIG_POSIX_UART=0` で抑止）。高速 baud: Linux glibc/musl は
  `B460800`〜`B4000000` の定数経由、macOS は B230400 超を `IOSSIOSPEED` ioctl で任意 baud 設定
  （いずれも 3 Mbaud を実機実績 — Core BASIC v2.7 の CH9102 ↔ MacBook）。round-trip は pty を
  使った native gtest で検証 (`test/v2/native/bus/test_posix_uart`)。
  ポート列挙ユーティリティ `listSerialPorts` / `rankSerialPortName` (`hal/uart/ports.hpp`) が
  ホストのシリアルポート候補を有力順に返す (macOS: `cu.usbserial*`/`cu.usbmodem*` 優先、
  Linux: `/dev/serial/by-id` 優先)。名前ヒューリスティクスは候補出しまでで、確定は呼び出し側の
  プロトコル疎通で行う。remote 接続の公開 API は `Hal::connect("uart:<path>")` または
  `Hal::initUart(port, cfg = {})`。実装ヘッダは
  `src/m5_hal/variants/frameworks/posix/hal/remote/uart_connection.hpp`
  (`PosixUartConnection::create(port, cfg)`)。
