# design/uart — UART

> **読者**: 実装者・レビュー向け（設計仕様）。

UART kind は I2C / SPI と同じく `Bus` / `Accessor` / `Source` / `Sink`
の形で扱う。ただし UART は full-duplex stream なので、TX と RX を
別 channel として lock する。現行APIは同期 read/writeを正準契約とし、
async executor連携はスコープ外とする。

## Bus の入手と所有

共通機構は [bus_accessor.md](bus_accessor.md) §Bus の保持 を参照。本 kind 固有の差分のみ以下に示す。

- **portable acquireのidentity projection = `Pins` tagのTX / RX**。`-1`の未指定roleもidentityの値として保持し、対応可否はbackendが判定する。
  したがってTX-only / RX-onlyやbackend既定pinをBusView自体は拒否しない。
- RTS / CTS と buffer size は identity 外だが、同一 TX/RX identity の acquire で値が食い違う場合は
  `INVALID_STATE` を返す (既存 bus は再構成しない)。通常はportable `acquire(cfg)`を使う。
  対応providerではNative / Path identityを、同名`acquire`の`native::borrowed` / `native::managed` policyで指定する。
- UART は **static-backend policy** ([bus_accessor.md](bus_accessor.md) §UART / I2S / PDM: static-backend policy 参照)。

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
    result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endAccess(uint32_t timeout_ms = 1000);
    result_t<bus::TransferStatus> getLastTransferStatus() const;
    result_t<size_t> write(data::ConstDataSpan src) override;  // StreamWriter
    result_t<size_t> write(data::Source& src, size_t len);
    result_t<size_t> write(const uint8_t* src, size_t len);
};

class RxAccessor : public bus::IAccessor, public data::StreamReader {
    result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endAccess(uint32_t timeout_ms = 1000);
    result_t<bus::TransferStatus> getLastTransferStatus() const;
    result_t<size_t> read(data::DataSpan dst) override;  // StreamReader
    result_t<size_t> read(data::Sink& dst, size_t len);
    result_t<size_t> read(uint8_t* dst, size_t len);

    result_t<size_t> readableBytes() override;  // StreamReader
};

class Accessor {
    TxAccessor& tx();
    RxAccessor& rx();

    result_t<size_t> write(data::ConstDataSpan src);
    result_t<size_t> read(data::DataSpan dst);
    result_t<size_t> readableBytes();
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
現行 UART bytecode には operation begin/end opcode がないため、remote の `beginOperation` は
local channel exclusion と configure script の先行送信までを担い、`endOperation` は local close のみ。
peer 側 Access を wire 越しに保持することはまだできず、distributed Access は将来の wire seam である。

lock の取得待ちは以下の呼び出し引数で制御する（config には置かない）:
- `TxAccessor::beginAccess(timeout_ms)` — TX channel
- `RxAccessor::beginAccess(timeout_ms)` — RX channel
- `Accessor::beginAccess(timeout_ms)` — 両チャネル (TxRx)

省略 = 無限待ち、0 = 即時 try-lock。同一 channel への複数 owner の競合は
timeout まで待って `TIMEOUT_ERROR`（詳細は [bus_accessor.md](bus_accessor.md) §排他制御の意味論）。

Access は **non-nestable** であり、同じ accessor に対する二重 `beginAccess()` は
`INVALID_STATE`。開始時は channel lock の取得後に backend の
`beginOperation(context)` を正確に1回呼び、終了時は
`endOperation(context)` を正確に1回呼んでから channel lock を解放する。
開始 hook が失敗した場合は取得済み lock を巻き戻し、Access は開始されない。
`write` / `read` / `transfer` sugar は、該当 channel の Access が既に active ならそれを
borrowし、inactiveなら一時 Access を開閉する。旧 `beginTransaction` / `endTransaction` と
depth/totals 集約は `beginAccess` / `endAccess` および転送単位 status に統合され、公開 API から削除する。

Contextのチェック機構は [bus_accessor.md](bus_accessor.md) §OperationContext capabilityとchecked facade と同一
(TX/RX別slot前提もそこで一般化済み)。providerの派生点はprotected `beginOperationBackend` /
`endOperationBackend` / `writeBackend` / `readBackend` / `transferBackend` / `readableBytesBackend`へ統一し、
raw owner/configを公開virtual引数として受けない。

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

再設定 (`beginOperation` の `OperationContext::config` が適用済みのものと異なる場合) の意味論:

- **初回適用**（そのバスでまだ何も適用されていない）はゲート不要で、TX/RXいずれの
  `beginOperation` でも即座に適用する。I/O 本体は設定変更を行わない。
- **再設定**は**両チャネルが静止しているときのみ**適用する。反対チャネルが使用中で静止を
  確認できない場合は設定も I/O も行わず **`BUSY` を返す**。診断カウンタ
  (`reconfigSkips()`) は互換性のためこの拒否回数を数える。現行設定のまま成功させることは禁止する。
  - **静止 (quiescent) の定義**: 反対チャネルが未保持で、非ブロッキングに取得できること。
    呼び出し元自身または combined accessor の相方が保持していても、異なる線路設定を要求する
    operation は `BUSY`。同一スレッドの無関係な accessor が保持している場合も busy
    として拒否する — 自スレッドが既に保持している mutex への try-lock は POSIX
    `std::timed_mutex` 上で未定義動作となるため構造的に試行できず、また意味論上も
    「反対チャネル使用中」に変わりはないため。

state mutex は`init` / `close`とraw I/Oの生存期間競合までは守らない —
それはfacade契約（アクセスウィンドウ外でのみ再init / closeする）
の管轄で、本節の対象外。

POSIX variant の write coalescing (`tx_coalesce_bytes`) は state mutex 下でバッファに
まとめ書きし、TX `endOperation` または RX 側の `read` / `readableBytes` で flush する。flush は
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
へ受け渡した byte 数。I/O 本体の error は `endOperation` error より優先する。
I/O 本体が成功して物理 drain / flush を担う `endOperation` が失敗した場合、公開戻り値は
その終了 error とするが、個別 I/O の `TransferStatus` は成功状態と受理済み prefix を保持する。
caller は status の totals を retry boundary とし、受理済みprefixの重複送信を避ける。

各 `write` / `read` と複合 `transfer` は転送ごとに `TransferStatus` を更新し、
`getLastTransferStatus()` で取得できる。正常な個別 I/O は `Complete`、短い正常転送は
`Partial`、I/O 開始後の失敗は `Aborted`。`endAccess` / `endOperation` の結果は lifecycle の
戻り値であり、個別 I/O status へ混ぜない。一時 Access の開始自体が失敗した場合も新しい転送を
開始していないため、直前の status を保持する。

**完了保証の正準契約は ESP-IDF backend の意味 (timeout 付き物理 drain 待ち)**。
他 backend は実装手段の制約により完全には一致しない。完全統一は不可能なため、差は
仕様として下表に固定し、この契約表を正本とする:

| backend | write 復帰タイミング | `write_timeout_ms` の扱い |
|---|---|---|
| ESP-IDF | `endAccess()` の `uart_wait_tx_done()` で物理 TX FIFO の drain まで | `endAccess(timeout_ms)` が drain 待ちの上限。失敗時の受理済み量は `TransferStatus` に残る |
| ESP-IDF USB CDC | TinyUSB queue投入後にflush | 0 byte受理時のflush失敗はerror、受理済みならshort success |
| ESP-IDF USB Serial/JTAG | driver bufferへ受理された時点 | 0 byteのtimeoutは`TIMEOUT_ERROR`、受理済みならshort success |
| Arduino | `endAccess()` の `HardwareSerial::flush()` で物理送信完了まで | `write_timeout_ms` は **使われない** (flush に timeout 引数がなく無期限待ち)。利用者は baud と len から所要時間を見積もれる |
| POSIX | OS の tx buffer へ受理された時点 (**drain しない**。await-reply パターンが buffered output でデッドロックしない設計上の選択) | 使われない |
| remote | peer backendが要求stream全体を受理し、終端Responseが届いた時点 | 正のshort writeはpeer側でsuffixを継続するexact stream。応答期限とRX short readは[remote.md](remote.md) §UART stream timeout / short transfer |

## error semantics

- API contract 違反（bus kind 不一致、baud/data bits/stop bits/parity の未対応値、POSIX の null path 等）は `INVALID_ARGUMENT`。
- 受信 timeout は `TIMEOUT_ERROR` ではなく正常な短い read（0 byte または partial）として扱う。送信は受理済みbyteが無いtimeoutを`TIMEOUT_ERROR`とし、受理済みprefixがあればそのbyte数をshort successとして返す。
- POSIX の `open` / `termios` / `select` / `read` / `write` 失敗、および ESP-IDF driver の未分類 `esp_err_t` は transport/OS/driver 障害として `IO_ERROR`。
- driver install や一時 buffer 確保など bounded resource の不足は `OUT_OF_RESOURCE`。

## variants

- `variants::frameworks::arduino` は `HardwareSerial` に委譲する。
  Arduino ESP32 では `Serial` が USB CDC (`HWCDC` / `USBCDC`) になる設定が
  あるため、`port_num` から `Serial` を内部解決しない。caller-owned serialは
  `acquire(cfg, native::borrowed(HardwareSerial&))`で利用する。
  **plain `Stream` 束縛の例外** (`acquire(cfg, native::borrowed(Stream&))` —
  HWCDC / USBCDC もここに入る): 既に構成済みの byte stream を採用する経路であり、
  線路フォーマット (baud / parity / stop bits / invert) は適用も検証もしない
  (CDC には線路の概念が無く、物理 UART を `Stream` として渡した場合は外部構成が正)。
  per-access 設定で適用されるのは `first_byte_timeout_ms` のみ。線路フォーマットの
  適用・検証契約 (未対応値の `INVALID_ARGUMENT` を含む) は `HardwareSerial` 束縛時のみ有効。
- `variants::frameworks::espidf` は ESP-IDF UART driver
  (`uart_driver_install`, `uart_read_bytes`, `uart_write_bytes`) に委譲する。
  portable `BusConfig`はport番号を持たない。通常のportable取得は既定UARTをprovider側で選び、特定portを
  direct利用する場合は`Bus_espidf::init(cfg, native::managed(NativePort{port}))`で明示する。
  IDF 5 / 6 の代表 buildで両経路を確認する。
  `Bus_console` の自動選択順は、利用可能なら TinyUSB CDC、USB Serial/JTAG、console UART の順とする。
  USB経路はそれぞれ標準の `Bus_espidf_usb_cdc` / `Bus_espidf_usb_jtag` backendを合成するため、timeout、
  callback teardown、受信cache、`readableBytes()` の契約は直接利用時と同じである。USB Serial/JTAG driverが
  既にinstall済みなら非所有で借用し、自身がinstallした場合だけ`closeBackend()`でuninstallする。console UARTは
  既存console設定を変更しない借用経路を維持するため、通常の`Bus_espidf`へは置き換えない。
- `variants::frameworks::posix` は POSIX host の termios serial に委譲する。
  device path（`/dev/ttyUSB0`等）は`acquire(cfg, native::managed(NativePath{path}))`で
  M5HAL所有として開き、caller-owned fd（ptyの片端など）は
  `acquire(cfg, native::borrowed(NativeFd{fd}))`で採用する。termiosは最初のwrite/readで
  per-access `AccessConfig` から適用（baud / 8bit / stop / parity）、timeout は
  `select()` で実装する。read/write 以外の line は raw（`cfmakeraw`）。素の POSIX
  `Bus_console`はcallerから渡された`FILE*` / fdを借用し、termiosを設定しないため、このtty backendとは別経路である。
  host で既定有効の opt-out（`M5HAL_CONFIG_POSIX_UART=0` で抑止）。高速 baud: Linux glibc/musl は
  `B460800`〜`B4000000` の定数経由、macOS は B230400 超を `IOSSIOSPEED` ioctl で任意 baud 設定する。
  round-trip contractはptyを使ったnative gtest (`test/v2/native/bus/test_posix_uart`)で固定する。
  ポート列挙ユーティリティ `listSerialPorts` / `rankSerialPortName` (`hal/uart/ports.hpp`) が
  ホストのシリアルポート候補を有力順に返す (macOS: `cu.usbserial*`/`cu.usbmodem*` 優先、
  Linux: `/dev/serial/by-id` 優先)。名前ヒューリスティクスは候補出しまでで、確定は呼び出し側の
  プロトコル疎通で行う。remote 接続の公開 API は `Hal::connect("uart:<path>")` または
  `Hal::initUart(port, cfg = {})`。実装ヘッダは
  `src/m5_hal/variants/frameworks/posix/hal/remote/uart_connection.hpp`
  (`PosixUartConnection::create(port, cfg)`)。
    result_t<void> beginAccess(uint32_t timeout_ms = TIMEOUT_FOREVER);
    result_t<void> endAccess(uint32_t timeout_ms = 1000);
    result_t<bus::TransferStatus> getLastTransferStatus() const;
