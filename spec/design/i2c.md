# design/i2c — I2C kind 固有設計

> **読者**: 実装者・レビュー向け（設計仕様）。

I2C kind の Bus / Accessor / 設定型 / sugar の仕様。 共通基底は [bus_accessor.md](bus_accessor.md)、 データ実体は [data_io.md](data_io.md)、 per-call メタは [transfer_desc.md](transfer_desc.md) を参照。

## 設定型

```cpp
namespace m5::hal::v2::i2c {

struct Scl { types::gpio_number_t value; };  // explicit ctor のみ (暗黙変換なし)
struct Sda { types::gpio_number_t value; };

struct IBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_scl = -1;  // default invalid
    types::gpio_number_t pin_sda = -1;

    constexpr IBusConfig(Scl, Sda);  // タグ型 ctor (順不同の 2 順列)
    constexpr IBusConfig(Sda, Scl);
};

struct MasterAccessConfig : public bus::IAccessConfig {
    uint32_t freq                   = 100000;  // Hz
    uint32_t wire_timeout_ms        = 1000;    // ms (ワイヤ進行の上限。lock は呼び出し引数)
    uint16_t i2c_addr               = 0;       // 7-bit (上位 9 bit を 0 padding)、 10-bit 時は実 address
    bool     address_is_10bit       = false;
    uint8_t  register_address_bytes = 0;       // register アドレス幅 (全 register sugar 共通): 0/1 = 1 byte, 2 = 2 byte
    bool     use_restart            = true;    // write-then-read で repeated start を使う
};

}
```

`use_restart` は write-then-read 時の repeated start 制御に使う。 `wire_timeout_ms` の default は 1000ms。 I2C scan 等で短縮したい場合は `IBus::probe(addr, freq, timeout_ms)` の sugar が default 50ms を採用する (下記、 lock とワイヤの両方をこの予算で抑える)。

`register_address_bytes` は **全ての register sugar (`writeRegister`/`readRegister`) が参照する唯一のアドレス幅決定源**。 `0` と `1` は default の 1-byte register address として扱い、 `2` を明示した場合だけ 2-byte register address を big-endian で組み立てる。 それ以外の値は API 契約外で、debug build では assert、 release build では `INVALID_ARGUMENT` を返す。 register 番号の C++ 型 (リテラル / `uint8_t` / `uint16_t` 定数) は wire 幅に影響しない (幅はこの field のみで決まる)。 2-byte address のデバイスはこれを `2` に設定する。

### pin の設定 (タグ型 ctor / フィールド代入)

pin の設定手段は 2 つで、 どちらも型安全:

- **タグ型 ctor = 1 行構築の正**: `i2c::BusConfig cfg{i2c::Scl{22}, i2c::Sda{21}};`。 タグの順序は自由 (どちらの順で書いても正しいフィールドに入る)。 タグの ctor は `explicit` で、 整数からの暗黙変換は無い。
- **フィールド代入 = プログラマブルな構築の正**: `cfg.pin_scl = SCL;`。 条件分岐やループでピンを組み立てる場合はこちら。

タグ無しの positional ctor (`BusConfig{22, 21}`) は**意図的に存在しない**: SCL と SDA は同じ整数型を共有するため、 引数順の取り違えがコンパイルを通ってワイヤ上でだけ壊れる。 タグが役割を型で運ぶ。 タグが要るのは**同型 (`gpio_number_t`) のピン引数同士のみ**で、 型が異なる引数 (`TwoWire*` 等) はオーバーロード解決が型で守るためタグ不要 — variant 固有フィールドはタグ構築後のフィールド代入で設定する。

Framework依存のnative handle / portはportable `IBusConfig`へ置かない。通常取得は全providerで
`acquire(BusConfig)`に統一し、buildのwinner bindingがproviderを選ぶ。既存native resourceを使う場合だけ、
対応providerのownership policy overloadを使う。

- Arduino variant: `acquire(cfg, native::borrowed(TwoWire&))`を提供し、caller-owned `TwoWire`を借用する。managed取得は提供しない。coreの`begin()`が`bool`を返す場合の失敗とRX buffer上限の契約はportable取得と同じ。
- ESP-IDF variant: 現行native取得はgen5 driverの`native::borrowed(NativeMasterBus)`だけ。legacy driverやmanaged native取得へ一般化しない。
  gen5のportable取得はdriverのport自動選択 (`i2c_port=-1`)を使い、空いているHP controllerを選ぶ。特定controllerを
  必須・優先指定する場合はportable configへfieldを足さず、logical allocation APIを使う。gen4 backendはlegacy
  driver制約によりcontroller 0を既定とする。
- software variant: native resource policyを提供せず、portable configだけを受ける。

provider固有初期化が必要な実装者は`Bus_<variant>::init(...)`を直接使えるが、variant configを
公開acquireのselectorにはしない。

## IBus (Bus 抽象基底)

```cpp
namespace m5::hal::v2::i2c {

class IBus : public bus::IBus {
public:
    result_t<void> transfer(
        bus::OperationContext<MasterAccessConfig>& context,
        const TransferDesc& desc,
        data::Source* src,
        size_t tx_len,
        data::Sink* dst,
        size_t rx_len);

    result_t<void> probe(
        uint16_t i2c_addr,
        uint32_t freq = 100000,
        uint32_t timeout_ms = 50);

protected:
    virtual result_t<void> transferBackend(
        bus::OperationContext<MasterAccessConfig>& context,
        const TransferDesc& desc,
        data::Source* src,
        size_t tx_len,
        data::Sink* dst,
        size_t rx_len);

    // 簡易 probe sugar。
    // Accessor を構築せず単一 device の存在確認ができる短縮 API。
    // 内部で stack-allocated な MasterAccessor を sentinel として組み立て、
    // 同じ probe path を呼ぶ。 default `timeout_ms = 50` は I2C scan 用途を
    // 想定 (`MasterAccessConfig` 全体 default の 1000ms とは別)。
};

}
```

`transfer` は **atomic な 1 回の I2C トランザクション** を表す。

公開`transfer(context, ...)`はactive ContextのBus・Accessor・generationを検査するnon-virtual入口であり、
providerはprotected `transferBackend(context, ...)`だけをoverrideする。設定は`context.config`、診断用ownerは
検査済みContextから取得し、raw `IAccessor*`を所有証明として受けない。

I2C masterの`beginOperation`はBus排他と共通runtimeを確立するが、START/STOPやdevice handleを生成しない。
target address、frequency、wire timeoutは一つの物理transactionに属するため、`transfer`がwireへ触れる前に
検証・適用する。Arduinoの`setClock/setTimeOut`とESP-IDFのdevice handleはapplied値をcacheし、同じAccess内の
後続transferで無用な再設定を行わない。このkind固有例外でもrequested ConfigはAccess中に不変である。

### transfer の wire semantics

`transfer` が wire 上に出すバイト列の順序:

```
START
  ADDR | W   (cfg.i2c_addr + write bit、 ACK 待ち)
  desc.prefix[0..prefix_len]   (ACK 待ち)
  src Source の中身            (peek → advance を繰り返して全部送る、 ACK 待ち)
[RESTART or STOP]              (write 終了)
  ADDR | R   (dst がある場合のみ、 cfg.use_restart で RESTART か STOP+START か決まる)
  dst Sink に書き込み          (reserve → commit を繰り返して全部受ける、 末尾 NACK)
STOP
```

- `desc.prefix_len == 0` かつ `src == nullptr` の場合は write phase をスキップ (dst がある場合は ADDR|R から開始)
- `dst == nullptr` の場合は read phase をスキップ
- `desc.prefix_len == 0` かつ `src == nullptr` かつ `dst == nullptr` の **全空** = **probe path** として扱う (下記)
- 成功時の戻り値は **caller data phase の転送量**。`read` / `readRegister` は `dst` Sink に入った受信バイト数、`write` / `writeRegister` は `src` Source から送った書き込みバイト数を `result_t<size_t>` で返す。 write 系は full-or-fail なので成功時の値は要求 `len` と一致する。 `desc.prefix` は数えない (SPI の command/address 相と同じ分離。`readRegister(reg, buf, 4)` の成功は受信した 4 を返す)
- `MasterAccessConfig::wire_timeout_ms` は **転送 1 回のワイヤ進行の全体上限** (espidf の per-transfer 意味に全 backend を統一)。software はクロックストレッチ個別上限に加えて転送全体デッドラインを持ち、arduino は `Wire::setTimeOut` へ遅延適用する。bus-lock の取得待ちはここに含まれない (`beginAccess` の呼び出し引数)
- **master 側の終端責務**: master がバスを放棄するとき (timeout / abort) は、可能な限り
  速やかに定義された終端 (STOP、または stuck な SDA からの回復パルス列 + STOP) を打って
  からエラーを返す契約とする。slave 側はこれを in-band で正確に判定する手段を持たないため、
  master 異常を推測しない (任意長の mid-transaction 静止は合法であり、推測はヒューリス
  ティクスにしかならない)。ESP-IDF gen5 driver は自身の回復を次の取引開始まで遅延させるため、
  backend は `TIMEOUT_ERROR` / `I2C_BUS_ERROR` を返す直前に `i2c_master_bus_reset()` を
  挟んで即時化する (NACK による `I2C_NO_ACK` は driver が既に STOP まで面倒を見るため対象外)。
  legacy (gen4) driver は `i2c_master_cmd_begin` 内部で同型の回復を同期的に行うため追加対応
  不要。software (bit-bang) backend は個別フェーズ・全体デッドラインの両方のエラー経路で
  STOP を試みる。他社製 master と共存する M5HAL slave にはこの契約の外側 (M5HAL 以外の
  master がこの契約を守らない場合) の暴露が残ることは既知の限界とする。espidf gen5 driver
  自体の ISR/timeout レース (upstream の既知バグと同系統) に起因して slave 側 SoC が
  SW 再初期化でも回復しない状態に陥りうる既知の制約もある — 詳細・実用推奨は
  [i2c_slave.md](i2c_slave.md) §backend 実装参照。

### probe path

`IBus::transfer` の契約として:

> prefix / src / dst が全て空 (`desc.prefix_len == 0` && `src == nullptr` && `dst == nullptr`) の場合、 wire 上に `ADDR | W` を送出して ACK を待ち、 結果を返す。

実装上の注意:
- variant の `transfer` 実装が「全空だから何もしない」 と短絡してはいけない
- 必ず wire 上に address+W を送出して ACK / NACK チェック
- `IBus::probe` / `MasterAccessor::probe` は `result_t<void>`を返し、ACK時success、NACK時
  `error_t::I2C_NO_ACK`とする。内部transferのtx/rx totalsはいずれも0

`MasterAccessor::probe()` がこの path を sugar として提供する (下記)。

### Bus (runtime facade) と acquire (所有レジストリ — `M5_Hal.I2C`)

共通機構は [bus_accessor.md](bus_accessor.md) §Bus の保持 を参照。本 kind 固有の差分のみ以下に示す。

- **facade と直接構築**: 無印 `i2c::Bus` は `IBus` を継承するruntime facadeで、`init(cfg)`は
  buildで選ばれたprovider backendを作る。`i2c::Bus bus; bus.init(cfg);`の直接構築は引き続き可。
- **portable acquireのidentity projection = `Pins` tagのコアピン`{scl, sda}`**。freqはaccessor設定なので含めず、
  backend/Wire/port も「駆動手段」なので identity ではない。同一ピンを異なる取得policyで
  再 acquire しても**最初の backend が勝つ** (差し替えは下記 §intent 駆動の HW 割当 の `commitBuses()`
  経由)。

```cpp
auto sp = m5::hal::v2::M5_Hal.I2C.acquire(i2c::BusConfig{i2c::Scl{22}, i2c::Sda{21}});
if (!sp) { /* INVALID_ARGUMENT (pin 未設定) / backend init 失敗 / OUT_OF_RESOURCE (満杯) */ }
i2c::MasterAccessor dev{sp.value(), acc_cfg};  // shared_ptr直渡しでaccessorがbusをco-own
```

### intent 駆動の HW 割当

portable `acquire(cfg)`はbuildで選ばれたproviderを使う。これとは別に
`acquire(LogicalBusConfig)`は**配線 (ピン) と「意図」だけ**を述べ、HWコントローラの割当はファクトリに
任せる。ESP32 系は HW I2C コントローラが有限 (例 2 系統) なのに、M5StickC のように内蔵 / PortA / HAT の
3 系統を欲しがるボードがある — どれを HW にしどれを software (bit-bang) にするかは、固定指名でなく
**能力 (capability) への要望 + ファクトリの bin-packing** で解く。要望は内部的には capability マスク
(`AllocationIntent`) だが、利用者は kind ヘルパーで書く (マスクは露出しない)。

```cpp
i2c::LogicalBusConfig req{i2c::Scl{22}, i2c::Sda{21}, i2c::requireHardware()};  // 配線 + 意図を同時宣言
auto sp = M5_Hal.I2C.acquire(req);                    // commit 前は software で即利用可
// ... 他の配線も acquire(intent) ...
auto r = M5_Hal.I2C.commitBuses();                    // 一括解決: HW を優先度順に配り、残りは software
```

- **意図ヘルパー** ([i2c](../../src/m5_hal/hal/v2/i2c/i2c.hpp)、いずれも `AllocationIntent` を返す):
  `requireHardware()` (HW 必須、取れなければ commit でエラー) / `preferHardware()` (空きがあれば HW、無ければ
  software へ降格) / `automatic()` (既定、余れば HW) / `software()` (常に bit-bang、HW を他へ譲る) /
  `requireController(n)` (特定コントローラ必須) / `preferController(n)` (特定コントローラ優先、満杯なら他 HW) /
  `requireLowPower()` (低電力domainのHW必須) / `preferLowPower()` (低電力domainを優先し通常のHWへfallback)。
  内部表現は `AllocationIntent{require, prefer, forbid, controller_id, mode}` で、`HARDWARE` 等は capability
  ビット (software = `HARDWARE` を持たない backend)。`require` と `forbid` が衝突する intent は commit が
  `INVALID_ARGUMENT` で弾く。
- **解決は明示 `commitBuses()`**。`acquire(LogicalBusConfig)` は miss 時 **software backend で生成**して
  即利用可能にし intent を記録するだけ。`commitBuses()` が全 live I2C バスを走査し、優先度順 (特定コントローラ
  必須 → HW 必須 → HW/特定コントローラ優先 → 自動) に HW コントローラ (silicon 予算、総数キャップとは別) を
  配る。HW 必須 / 特定コントローラ必須が充足できなければ**副作用前に `OUT_OF_RESOURCE`** (どのバスも中途半端な
  状態に残さない)。優先 / 自動は黙って software へ降格する。controller の tie-break は最小番号で決定的。
- **commit 計画は snapshot 駆動**。resolver は live bus 一覧から `managed` / `intent` /
  `backendKind` / `controllerId` を snapshot し、その固定入力だけで割当計画を作る。事前検出できる
  over-subscription や intent 衝突は swap 前に失敗する。backend swap 自体の失敗は first error を返し、
  終了時に live backend 状態から controller pool を再同期するため、`hardwareInUse()` は観測可能な live
  state に戻る。同じkindのlogical再acquireによるintent更新とcommitのintent snapshotは短い専用lockで
  直列化し、snapshot後の更新は進行中のswapには混ぜず**次回commitの入力**になる。計画・pin-domain判定・
  backend factoryにはsnapshotした同じintentを渡すため、一回のcommit内で新旧intentが混在しない。
  re-acquireが返った後に開始したcommitは更新後のintentを必ず見る。kindが異なるcommitは独立である。
- **付け替え (reassign) は実行時**。intent を上げて (再 `acquire` が intent を再タグ) `commitBuses()` を
  呼び直すと、論理バスの backend が**バスロック下で hot-swap** される。不変条件: ① HW 必須は **付け替え免疫**
  (現コントローラを保持し続ける) ② HW の受け渡しは「譲る側を software 化 → コントローラをプール返却 →
  受け取る側がプールから確保」の**プール経由 2 ステップ** (2 ロック同時保持を避ける)。進行中の転送がある間は
  swap がその完了を待つ (有限 timeout なら `TIMEOUT_ERROR`)。
- **query API** (`bus::IBus`、スワップで能力が変わるため**バス自身に問い合わせ**):
  `backendKind()` (`Hardware`/`Software`) / `controllerId()` (HW のペリフェラル番号、software は -1) /
  `maxFrequency()` (能力上限 Hz、0 = 未申告) / `backendGeneration()` (swap 世代カウンタ、poll baseline)。
  software 降格を検知したい利用者は `backendGeneration()` の差分か `backendKind()` を再 query する。
- **HW backend が無いビルド** (software-only / native) は silicon 予算 0 = すべて software、HW 必須
  (`requireHardware()` / `requireController()`) は `OUT_OF_RESOURCE`。intent パスは「同型のまま HW⇔software を
  runtime 選択」する仕組みなので、ビルドに
  HW variant がある (espidf) かどうかで自然に振る舞いが決まる。
- backend 選択の機構詳細・段階化は [bus_accessor.md](bus_accessor.md) §Bus の保持 を参照。

### ESP-IDF LP_I2C

LP controllerはopt-inであり、`automatic()`とplain `requireHardware()`では選ばれない。
`requireLowPower()` / `preferLowPower()`でLOW_POWER能力を明示するか、`requireController(n)` /
`preferController(n)`でLP controllerを名指しした場合だけ候補になる。`requireLowPower()`はLPを必須とし、
`preferLowPower()`はLPを優先して通常のHW controllerへfallbackする。どちらもHW自体は必須なので、
利用可能なHWが無ければ`commitBuses()`は`OUT_OF_RESOURCE`を返す。

- **C5/C6型の固定IOMUX**: local ESP-IDF acquireで`requireLowPower()`を指定し、SCL/SDAを両方省略した
  場合だけSoC固定pinを補完する。片方だけの省略または固定pin以外は`INVALID_ARGUMENT`。
  `preferLowPower()`とremote acquireは補完しない
- **P4型のLP GPIO matrix**: pinを補完しない。SCL/SDA双方にRTC/LP GPIOを明示し、非対応pinまたは
  pin省略は`INVALID_ARGUMENT`
- **公開gate**: modern bus-device driver、`SOC_LP_I2C_SUPPORTED`、ESP-IDF 5.4以上を全て満たす場合だけ
  LP controllerをpoolへ公開する。LP GPIO matrixを使うP4型はESP-IDF 5.5以上が必要。legacy backendと
  gate外buildはHP controllerだけを公開する
- **周波数上限**: LP backendの`maxFrequency()`は400kHz。SoC/driverがより低いfail-safe上限を返す場合は
  その値を使う

## MasterAccessor (Accessor)

```cpp
namespace m5::hal::v2::i2c {

class MasterAccessor : public bus::IAccessor {
public:
    MasterAccessor(IBus& bus, const MasterAccessConfig& cfg);
    IBus& getBus(void) const;

    // 通信パラメータ差し替え。
    // 「同じ Accessor を使い回して address だけ変えていく」 scan パターン用 sugar。
    // 排他制御中 (`inAccess() == true`) は INVALID_STATE で reject する。
    result_t<void> setConfig(const MasterAccessConfig& cfg);

    result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endAccess(uint32_t timeout_ms = 1000);
    bool transferBusy();
    result_t<bus::TransferStatus> getLastTransferStatus() const;

    result_t<bus::TransferTotals> transfer(
        const TransferDesc& desc,
        data::ConstDataSpan src,
        data::DataSpan dst);

    // 基本形 (canonical): Source/Sink で len バイトを授受。下記 span / raw はこの形への糖衣。
    result_t<size_t> write(data::Source& src, size_t len);
    result_t<size_t> read(data::Sink& dst, size_t len);

    result_t<size_t> write(data::ConstDataSpan src);
    result_t<size_t> read(data::DataSpan dst);

    // raw pointer overload: C 配列を直接渡す用途。
    result_t<size_t> write(const uint8_t* src, size_t len);
    result_t<size_t> read(uint8_t* dst, size_t len);

    // register sugar: アドレス幅は register_address_bytes (config) が単一の決定源。
    // reg は値 (任意の整数。0x00 リテラルも uint8_t/uint16_t 定数も可) で、型は幅に無関係。
    result_t<size_t> writeRegister(int reg, data::ConstDataSpan value);
    result_t<size_t> writeRegister(int reg, uint8_t value);
    result_t<size_t> writeRegister(int reg, const uint8_t* src, size_t len);  // raw pointer
    result_t<size_t> readRegister(int reg, data::DataSpan dst);
    result_t<size_t> readRegister(int reg, uint8_t* dst, size_t len);       // raw pointer
    result_t<uint8_t> readRegister(int reg);
    // 基本形 (canonical): register + Source/Sink で len バイトを授受。
    result_t<size_t> writeRegister(int reg, data::Source& src, size_t len);
    result_t<size_t> readRegister(int reg, data::Sink& dst, size_t len);

    result_t<void> probe();

private:
    bus::OperationContext<MasterAccessConfig> _context;
    bus::TransferStatus _last_transfer_status;
    uint32_t _next_transfer_id;
};

}
```

### transfer中のエラー

一回の`transfer` descriptor内のsegment群は一個のatomic wire操作を成す
(register pointer書込み → repeated START → read等)。同期pre-flight errorと
`waitTransfer`で表面化するwire errorは、そのI/Oの戻り値と`TransferStatus`に記録する。
errorはAccess-wideにlatchせず、backendがbrokenでなければ同じAccess内の後続transferを試せる。
`endAccess`はoutstanding I/O完了とbackend cleanupだけを返し、過去のI/O errorを再返却しない。

- 「失敗した segment を飛ばして続行」「同一 segment のその場リトライ」は許可しない。
  segment 間に依存が無い操作は、そもそも別 transaction に分ける。
- backend (IBus 実装者) の義務: 同期エラーを返す場合は**ワイヤに触れる前に拒否する**。
  これは transaction の latch 意味論とは独立に、wire 状態の整合を保証するための契約。
- 不採用案 — pre-flight 拒否は transaction を汚さない (何も起きていないので継続可能とする)
  案は、segment 単位の戻り値を確認しない呼び出し元に「segment が抜けたのに end は成功」
  という silent break を許すため退けた。一時的失敗のその場リトライを失うが、
  チェーン全体の再実行の方が常に意味的に安全。

### register sugar の挙動

`writeRegister` / `readRegister` は内部で `TransferDesc` を組み立てて `transfer` に委譲する。 **アドレス幅の決定源は `MasterAccessConfig::register_address_bytes` のみ** (`0`/`1` = 1 byte、`2` = 2 byte、2 byte は big-endian = MSB first)。 register 番号は値であり、 **引数の C++ 型は wire 幅に影響しない** (`readRegister(0x00)` も `static constexpr uint8_t REG = 0xD0;` も同じ経路)。 アドレス幅はデバイス固有の固定属性なので accessor 設定時に一度決める。 register address 組み立て規則・`TransferDesc` ctor 制約は [transfer_desc.md](transfer_desc.md) を参照。

- register 値は幅に対して範囲チェックされる (1-byte device で `> 0xFF` の register は `INVALID_ARGUMENT`)。 未対応の幅は debug で assert、 release で `INVALID_ARGUMENT`。
- **2-byte address のデバイス (一部 EEPROM / sensor) は `register_address_bytes = 2` を設定する**。 多数派は 1-byte default。 型駆動・呼び出しごとの幅は無いので、 **幅が無言で誤る/呼び出しスタイル間で食い違うことは起きない** (旧 API の「型 sizeof 経路 vs config 経路」フットガンを解消)。
- value 側のサイズ・バイト順は呼び出し側責任 (`ConstDataSpan`/`DataSpan` または `Source`/`Sink`)。 big/little-endian の value helper は M5UU 層 (`M5UnitComponent`) の役割。
- **register address 専用の strong type (`RegAddr` 等) は追加しない** — typed register 定数 (`static constexpr uint8_t REG = …;`) が幅を型に持たせる正準経路として既にあり、第三の流派を増やさない。 register address は 1 引数で順序事故が構造的に起きず、 需要が顕在化すれば非 BREAKING で後から追加できる。

### raw pointer overload の位置付け

`write` / `read` / `writeRegister` / `readRegister` には `data::*Span` 版に加えて `(const uint8_t* src, size_t len)` / `(uint8_t* dst, size_t len)` の raw pointer overload を備える。 内部実装は Span overload に転送するだけ。 `uint8_t*` と `data::*Span` は別型なので overload 解決の曖昧性は出ない。

### SPI の `writeCommand` 相当が無い理由

SPI には `writeCommand` / `writeCommandAddress` / `writeCommandData` があるが、 I2C には**ない**。 これは欠落ではなく、 プロトコル構造の違いを反映した意図的な非対称:

- **SPI** は command / address / data という独立した phase を 1 つの CS 窓内に連ねる構造を持ち、 `writeCommand*` 群がその phase 合成を表す。
- **I2C** に独立した command phase は存在しない。 「コマンドを送る」 は (a) レジスタを持つデバイスなら `writeRegister(reg, …)`、 (b) 単純なコマンドバイト列なら `write(…)` で表す。 これらが I2C における command 送出の正準形。

SPI 由来で `writeCommand` を探した場合は `write` / `writeRegister` を見る。 register/command の前置と value の分離は `transfer(TransferDesc{…}, src, dst)` の `desc.prefix` が担う (SPI の command/address 相と同じ分離。 [§transfer の wire semantics](#transfer-の-wire-semantics))。

### setConfig の位置付け

`setConfig(cfg)` は I2C scan のように「同じ Accessor で address だけを差し替えていく」 用途のための sugar。 通常は Accessor を都度再構築すれば足りるが、 scan loop で 112 個の Accessor を構築 → 1 個 + 112 回の setConfig に集約できる。 `inAccess() == true` の状態で呼ぶと transfer 途中の cfg が未定義状態になるため `INVALID_STATE` で reject する。 caller は ScopedAccess の外側で呼ぶこと。

## software I2C variant の実装方針

`variants::frameworks::software` の I2C master は、 GPIO `Pin` を open-drain 相当で駆動する bit-bang 実装として扱う。 START / STOP / byte write / byte read / transaction の各 service は、 呼び出し元が測った経過 (`ServiceContext::elapsed`、 [service.md](service.md) §時間契約) で **service private の仮想時計**を進めて駆動する。

**転送全体デッドライン (`wire_timeout_ms`) の測定クロック**: 仮想時計は gap-drop ([service.md](service.md)) で実時間より遅れうるため、 デッドラインには使わない。 コア間共有の単調クロック (`service::sharedNowUs()`) で開始時刻との差分により別途判定する (クロックストレッチ個別上限は従来どおり仮想時計上で毎回判定する)。

内部アルゴリズム (fast tick 変換、 位相維持、 fast path 分岐等) は [i2c.inl](../../src/m5_hal/variants/frameworks/software/hal/i2c/i2c.inl) の実装コメントを正本とする。
transaction service が公開する開始操作は、 製品の組立て経路で使う address / buffer read-write / STOP に限定する。 START や単発 byte の primitive は各専用 service を直接 test し、 transaction 側に test 専用 dispatch を持たせない。

### timing と物理層の注意

software I2C の設定周波数は「service が目標とする SCL half period」であり、 実際の wire 周波数を保証しない。 特に I2C の HIGH は pull-up と bus capacitance に依存するため、 SCL / SDA の rise time が遅いと software hot path が十分速くても実測周波数は頭打ちになる。 400kHz を超える検証では、 ロジアナの digital 表示だけでなく、 可能ならオシロで SCL/SDA の analog rise time を確認すること。

software I2C を高め (例: 2MHz) に設定しても、wire 実測は pull-up 強度に支配される。 SCL pull-up を強めると実測周波数は上がる一方、ベンチマークの内部推定 (write buffer timed 等) はそれより高い値を示す。 実機上限の一部が code hot path ではなく bus 物理条件に支配されることに注意する。

100kHz / 400kHz の実用設定では、十分な pull-up なら設定値にほぼ追従し、弱い pull-up では低速化するが通信は正常に成立する。 これは code 側の余分な遅延は補正しつつ、SCL の物理的な立ち上がりが遅い場合は安全側に低速化する設計意図と一致する。 残る追加作業は、特殊な bus 条件の検証、I2C slave service との協調テスト強化、ESP-IDF variant 側の driver 整備に置く。

## I2C slave

I2C slave はmaster体系と相似のBus型 (`ISlaveBus` / `SlaveBus_<variant>` / `SlaveBusConfig`) と、
caller-owned queueを持つ正準`SlaveAccessor`で提供する。`beginAccess/endAccess`が外部受付期間、Access外でも使える
`read/write`がRX drain/TX preloadを表す。blocking `SlaveStreamAccessor::serve()`とレジスタマップadapterは
`legacy_wire_frame_window=true`のopt-in互換面である。frame/segment、queue、timeout、backend capabilityの詳細は
[i2c_slave.md](i2c_slave.md)を参照する。

詳細は [i2c_slave.md](i2c_slave.md) を参照。

## ESP-IDF I2C variant の実装方針

`variants::frameworks::espidf` の I2C master は単一 variant とし、ESP-IDF 世代差は `detail/espidf_version.hpp` の feature detection と backend 実装で吸収する。 `ARDUINO` 定義の有無では無効化しない。 Arduino-on-IDF や ESP-IDF project with Arduino component では arduino / espidf variants が同時に存在しうるため、既定の勝者選択は scan 順に任せ、espidf 実装は suffix 付き実名 (`i2c::Bus_espidf`) で明示利用できる状態を維持する。

ESP-IDF gen5 I2C master backend (`driver/i2c_master.h`) は `freq == 0`、アドレス範囲外、`i2c_master_probe` で表現できない 10-bit probe を driver 呼び出し前に `INVALID_ARGUMENT` として扱う。 10-bit address の通常 transfer は device config 経由で扱い、probe path だけを制限する。

通常 transfer は `i2c_master_bus_add_device` で得た device handle を Bus 内に保持し、同じ address / frequency / address bit length / SCL wait 設定の連続アクセスでは再利用する。 `probe()` は scan 用の軽量経路として `i2c_master_probe` を直接使い、device handle cache とは独立させる。 設定が変わった場合や`closeBackend()`ではcached deviceを外してからbus handleを終了する。

`driver/i2c_master.h` が無く `driver/i2c.h` がある ESP-IDF 世代では gen4 master backend (`driver/i2c.h`) を使う。これは Arduino-ESP32 2.x 系のように SPI master driver はあるが gen5 I2C master driver は無い環境で、ESP-IDF I2C variant を明示利用できるようにするためである。 gen4 backend は 7-bit address の master transfer / probe を対象とし、10-bit address は driver 呼び出し前に `INVALID_ARGUMENT` とする。

master の SCL クロック (`MasterAccessConfig::freq`) は、ESP ターゲットでフェールセーフ上限 (`M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ`、`hal/v2/i2c/master_clock_limit.hpp`) に**クランプ**する。これは spec / バス品質の制限ではなく、過大なクロックで I2C master ペリフェラルが**異常動作** (波形停止・ライン張り付き) に陥り利用者をデバッグ沼に陥れることを防ぐ安全弁である (弱い pull-up による通信不能は利用者の配線責務であり対象外)。上限超過は**エラーにせず最も近い設定可能クロックに丸める** (利用者が上限値を知らなくても「最速を設定」が機能する)。gen4 / gen5 / Arduino-on-ESP の各 master backend に適用し、超過時は `M5_LIB_LOGW` を一度出す。既定はクロスターゲット保守値で、HW 実測済みは ESP32 classic のみ (~1.25MHz 動作 / 1.3MHz で異常動作)。非 ESP ターゲット (software bit-bang は守るべきペリフェラルが無い / 他コアへの Arduino port) は上限 0 = 無効で、`M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ` で上書きできる。

## RAII (ScopedAccess) との組み合わせ

各 sugar は内部で `beginAccess` → `transfer` → `endAccess` を行う。 連続アクセスを `ScopedAccess` で外側を囲う一般形は [bus_accessor.md](bus_accessor.md) §利用形 を参照。

## 関連

- [i2c_slave.md](i2c_slave.md) — I2C slave機構 (queue駆動SlaveAccessor + legacy Stream/RegMap)
- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離 + RAII
- [transfer_desc.md](transfer_desc.md) — `i2c::TransferDesc` 詳細
- [data_io.md](data_io.md) — Source / Sink + Limited 装飾
- [variants.md](variants.md) — variant 機構 (arduino / software / espidf の配置)
