# design/i2c — I2C kind 固有設計

I2C kind の Bus / Accessor / 設定型 / sugar の仕様。 共通基底は [bus_accessor.md](bus_accessor.md)、 データ実体は [data_io.md](data_io.md)、 per-call メタは [transfer_desc.md](transfer_desc.md) を参照。

## 設定型

```cpp
namespace m5::hal::v1::i2c {

struct IBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_scl = -1;  // default invalid
    types::gpio_number_t pin_sda = -1;
};

struct MasterAccessConfig : public bus::IAccessConfig {
    uint32_t freq                   = 100000;  // Hz
    uint32_t wire_timeout_ms        = 1000;    // ms (ワイヤ進行の上限。lock は呼び出し引数)
    uint16_t i2c_addr               = 0;       // 7-bit (上位 9 bit を 0 padding)、 10-bit 時は実 address
    bool     address_is_10bit       = false;
    uint8_t  register_address_bytes = 0;       // signed literal sugar: 0/1 = 1 byte, 2 = 2 byte
    bool     use_restart            = true;    // write-then-read で repeated start を使う
};

}
```

`use_restart` は write-then-read 時の repeated start 制御に使う。 `wire_timeout_ms` の default は 1000ms。 I2C scan 等で短縮したい場合は `IBus::probe(addr, freq, timeout_ms)` の sugar が default 50ms を採用する (下記、 lock とワイヤの両方をこの予算で抑える)。

`register_address_bytes` は `readRegister(0x00)` のような signed literal sugar だけが参照する。 `0` と `1` は default の 1-byte register address として扱い、 `2` を明示した場合だけ 2-byte register address を big-endian で組み立てる。 それ以外の値は API 契約外で、debug build では assert、 release build では `INVALID_ARGUMENT` を返す。 `uint8_t` / `uint16_t` の型付き register 定数を渡した場合は型の幅が優先され、この field は参照されない。

Framework 依存の native handle / port は共通 `IBusConfig` には置かない。 各 variant は共通 config を継承 (または alias) した `variant::<name>::BusConfig` を**必ず公開**し、 `init` は **その型を直接受ける非 virtual メンバ** (`init(const BusConfig&)`) として宣言する (基底 `bus::IBus` に virtual `init` は無い。 variant 固有情報が必須な操作を kind 汎用にはできないため。 [variants.md](variants.md) §offer 要件):

- Arduino variant: `TwoWire* wire` を明示する。`init(BusConfig)` はその `TwoWire` に `begin` / `end` を行い、`attach(TwoWire&)` は caller-owned lifecycle として扱う。
- ESP-IDF variant: ESP-IDF driver 世代に応じた `i2c_port` を持つ。pin / buffer など共通にできる値は基底 `IBusConfig` 側に残す。
- software variant: native handle を持たないため、`using BusConfig = IBusConfig;` の alias を公開して共通 config をそのまま受ける。

抽象 `IBusConfig` を拡張フィールド持ち variant (Arduino 等) の `init` に渡す誤用、 および別 variant の config を渡す誤用は、 **どちらもコンパイルエラー**になる (派生参照に基底オブジェクトは束縛できない)。 拡張なし variant への variant config 渡しは upcast として正しく通る。

## IBus (Bus 抽象基底)

```cpp
namespace m5::hal::v1::i2c {

class IBus : public bus::IBus {
public:
    virtual result_t<size_t> transfer(
        bus::IAccessor* owner,
        const MasterAccessConfig& cfg,
        const TransferDesc& desc,
        data::Source* tx,
        data::Sink*   rx) = 0;

    // 簡易 probe sugar (spec_polish A2)。
    // Accessor を構築せず単一 device の存在確認ができる短縮 API。
    // 内部で stack-allocated な MasterAccessor を sentinel として組み立て、
    // 同じ probe path を呼ぶ。 default `timeout_ms = 50` は I2C scan 用途を
    // 想定 (`MasterAccessConfig` 全体 default の 1000ms とは別)。
    result_t<void> probe(
        uint16_t addr,
        uint32_t freq         = 100000,
        uint32_t timeout_ms   = 50);
};

}
```

`transfer` は **atomic な 1 回の I2C トランザクション** を表す。

### transfer の wire semantics

`transfer` が wire 上に出すバイト列の順序:

```
START
  ADDR | W   (cfg.i2c_addr + write bit、 ACK 待ち)
  desc.prefix[0..prefix_len]   (ACK 待ち)
  tx Source の中身             (peek → advance を繰り返して全部送る、 ACK 待ち)
[RESTART or STOP]              (write 終了)
  ADDR | R   (rx がある場合のみ、 cfg.use_restart で RESTART か STOP+START か決まる)
  rx Sink に書き込み           (reserve → commit を繰り返して全部受ける、 末尾 NACK)
STOP
```

- `desc.prefix_len == 0` かつ `tx == nullptr` の場合は write phase をスキップ (rx がある場合は ADDR|R から開始)
- `rx == nullptr` の場合は read phase をスキップ
- `desc.prefix_len == 0` かつ `tx == nullptr` かつ `rx == nullptr` の **全空** = **probe path** として扱う (下記)
- 成功時の戻り値は **データ相のバイト数 (`tx + rx`) のみ**。`desc.prefix` は wire に送出されるが数えない (SPI の command/address 相と同じ分離。`readRegister(reg, buf, 4)` の成功は 4 を返す)
- `MasterAccessConfig::wire_timeout_ms` は **転送 1 回のワイヤ進行の全体上限** (espidf の per-transfer 意味に全 backend を統一)。software はクロックストレッチ個別上限に加えて転送全体デッドラインを持ち、arduino は `Wire::setTimeOut` へ遅延適用する。bus-lock の取得待ちはここに含まれない (`beginAccess` の呼び出し引数)

### probe path

`IBus::transfer` の契約として:

> prefix / tx / rx が全て空 (`desc.prefix_len == 0` && `tx == nullptr` && `rx == nullptr`) の場合、 wire 上に `ADDR | W` を送出して ACK を待ち、 結果を返す。

実装上の注意:
- variant の `transfer` 実装が「全空だから何もしない」 と短絡してはいけない
- 必ず wire 上に address+W を送出して ACK / NACK チェック
- `expected<size_t, error_t>` の戻り値は ACK 時 `0` (transferred bytes は 0)、 NACK 時 `error_t::I2C_NO_ACK`

`MasterAccessor::probe()` がこの path を sugar として提供する (下記)。

## MasterAccessor (Accessor)

```cpp
namespace m5::hal::v1::i2c {

class MasterAccessor : public bus::IAccessor {
public:
    MasterAccessor(IBus& bus, const MasterAccessConfig& cfg);
    inline IBus& getBus() const noexcept;

    // 通信パラメータ差し替え (spec_polish A2)。
    // 「同じ Accessor を使い回して address だけ変えていく」 scan パターン用 sugar。
    // 排他制御中 (`inAccess() == true`) は INVALID_ARGUMENT で reject する。
    result_t<void> setConfig(const MasterAccessConfig& cfg);

    result_t<size_t> transfer(
        const TransferDesc& desc,
        data::ConstDataSpan tx,
        data::DataSpan rx);
    result_t<size_t> write(data::ConstDataSpan tx);
    result_t<size_t> read(data::DataSpan rx);

    // raw pointer overload (spec_polish A3): C 配列を直接渡す用途。
    result_t<size_t> write(const uint8_t* tx, size_t len);
    result_t<size_t> read(uint8_t* dst, size_t len);

    template <typename TReg /* unsigned integral, sizeof ≤ 2 */>
    result_t<size_t> writeRegister(TReg reg, data::ConstDataSpan value);

    template <typename TReg /* 同上 */>
    result_t<size_t> writeRegister(TReg reg, uint8_t value);

    // raw pointer overload (spec_polish A3): register address + N byte 直書き。
    template <typename TReg /* 同上 */>
    result_t<size_t> writeRegister(TReg reg, const uint8_t* tx, size_t len);

    template <typename TReg /* 同上 */>
    result_t<size_t> readRegister(TReg reg, data::DataSpan dst);

    // raw pointer overload (spec_polish A3): register address + N byte 直読み。
    template <typename TReg /* 同上 */>
    result_t<size_t> readRegister(TReg reg, uint8_t* dst, size_t len);

    template <typename TReg /* 同上 */>
    result_t<uint8_t> readRegister(TReg reg);

    // signed literal overload: register_address_bytes で 1 / 2 byte を決める。
    result_t<size_t> writeRegister(int reg, data::ConstDataSpan value);
    result_t<size_t> writeRegister(int reg, uint8_t value);
    result_t<size_t> writeRegister(int reg, const uint8_t* tx, size_t len);
    result_t<size_t> readRegister(int reg, data::DataSpan dst);
    result_t<size_t> readRegister(int reg, uint8_t* dst, size_t len);
    result_t<uint8_t> readRegister(int reg);

    result_t<void> probe();

private:
    MasterAccessConfig _access_config;
};

}
```

### register sugar の挙動

`writeRegister` / `readRegister` は内部で `TransferDesc` を組み立てて `transfer` に委譲する。 型付き register 定数 (`uint8_t` / `uint16_t`) は型から幅を決める。 signed literal (`0x00` のような `int`) は `MasterAccessConfig::register_address_bytes` から幅を決める。 register address の組み立て規則、 `TransferDesc` の ctor 制約、 型付き定数の扱いは [transfer_desc.md](transfer_desc.md) を参照。

**register address 幅の二経路** (どちらの経路かは引数の型だけで決まる):

| 呼び出し形 | 幅の決定 | `register_address_bytes` |
|---|---|---|
| `readRegister(uint8_t{0xD0})` | `sizeof` = **1 byte** | 参照しない |
| `readRegister(uint16_t{0x1234})` | `sizeof` = **2 byte** (big-endian) | 参照しない |
| `readRegister(0xD0)` (signed literal) | config から | `0`/`1` = 1 byte、`2` = 2 byte |

**罠 — 二経路を同じデバイスで混在させない**: `register_address_bytes = 2` の
デバイスに `uint8_t` 定数を渡すと型幅が勝って **1 byte しか送られない**。
2-byte address のデバイスでは `uint16_t` 定数 (または literal) に統一する。
推奨スタイルは型付き定数 (`static constexpr uint8_t REG_X = 0xD0;`) —
幅が宣言に現れ、config の値に依存しない。

### raw pointer overload の位置付け (spec_polish A3)

`write` / `read` / `writeRegister` / `readRegister` には `data::*Span` 版に加えて `(const uint8_t* tx, size_t len)` / `(uint8_t* dst, size_t len)` の raw pointer overload を備える。 内部実装は Span overload に転送するだけ。 `uint8_t*` と `data::*Span` は別型なので overload 解決の曖昧性は出ない。

### setConfig の位置付け (spec_polish A2)

`setConfig(cfg)` は I2C scan のように「同じ Accessor で address だけを差し替えていく」 用途のための sugar。 通常は Accessor を都度再構築すれば足りるが、 scan loop で 112 個の Accessor を構築 → 1 個 + 112 回の setConfig に集約できる。 `inAccess() == true` の状態で呼ぶと transfer 途中の cfg が未定義状態になるため `INVALID_ARGUMENT` で reject する。 caller は ScopedAccess の外側で呼ぶこと。

## software I2C variant の実装方針

`variants::frameworks::software` の I2C master は、 GPIO `Pin` を open-drain 相当で駆動する bit-bang 実装として扱う。 実装は START / STOP / byte write / byte read / transaction を小さな service に分け、 同期 runner から比較可能な 32-bit tick (`ServiceContext::now_tick`) を渡して進める。 通常の同期 transfer path では `fastTick()` を使い、 `MasterAccessConfig::freq` から得た half period を fast tick 単位へ変換する。 これにより `micros()` / `esp_timer_get_time()` の呼び出しコストを hot path から外す。 `now_tick` の単位は runner が選ぶ (同期 path = 生 `fastTick()`、 native test = 素の数値)。 service は due 値を同じ単位で持ち、 加算と mod 2^32 比較しかしない。

write buffer は頻出経路なので、 `MasterTransactionService` 側に fast path を持つ。 具体的には `Operation::WriteBuffer` の dispatch を先頭で処理し、 byte write service を直接呼び、 2 byte 目以降は同じ line driver / timing を保持したまま byte state だけを restart する。 これは service 概念を維持したまま、 byte 列送信中の呼び出し層と分岐を減らすための最適化である。

byte write / byte read の定常クロックは、各 edge の実行時刻から `now + half_period` で次回予約するのではなく、前回 due に half period を加算して理想位相を維持する。 これにより `service()` dispatch や GPIO 操作の処理時間が SCL half period に毎回上乗せされることを避け、100kHz/400kHz のような低めの設定でも wire 周波数が設定値から下振れしにくくなる。 ただし service の遅延が大きく、次の due が現在時刻を過ぎている場合は `now + half_period` に再同期する。これは遅れを取り戻そうとして複数 edge を runner 速度で連続出力し、設定より大幅に速いクロック burst になることを避けるためである。 START / STOP の setup/hold や clock stretch 解除後は、実際に SCL/SDA の条件が成立した時刻から half period を取り直す。

SCL は `MasterLineDriver::writeSclHigh()` / `writeSclLow()` に分ける。 SCL は全 bit で立ち上げ/立ち下げが発生するため、 bool 引数経由の分岐を避け、 GPIO variant が high/low 専用 path (例: ESP32 の set/clear register) へ落としやすくする。 SDA は bit 値が data に依存するため `writeSda(bool)` のままとする。

### timing と物理層の注意

software I2C の設定周波数は「service が目標とする SCL half period」であり、 実際の wire 周波数を保証しない。 特に I2C の HIGH は pull-up と bus capacitance に依存するため、 SCL / SDA の rise time が遅いと software hot path が十分速くても実測周波数は頭打ちになる。 400kHz を超える検証では、 ロジアナの digital 表示だけでなく、 可能ならオシロで SCL/SDA の analog rise time を確認すること。

software I2C を高め (例: 2MHz) に設定しても、wire 実測は pull-up 強度に支配される。 SCL pull-up を強めると実測周波数は上がる一方、`I2CHotPathBenchmark` の内部推定 (write buffer timed 等) はそれより高い値を示す。 実機上限の一部が code hot path ではなく bus 物理条件に支配されることに注意する。

100kHz / 400kHz の実用設定では、十分な pull-up なら設定値にほぼ追従し、弱い pull-up では低速化するが通信は正常に成立する。 これは code 側の余分な遅延は補正しつつ、SCL の物理的な立ち上がりが遅い場合は安全側に低速化する設計意図と一致する。 残る追加作業は、特殊な bus 条件の検証、I2C slave service との協調テスト強化、ESP-IDF variant 側の driver 整備に置く。

## SlaveService の位置付け

`m5::hal::v1::i2c::SlaveService` は、現時点では実機の I2C slave peripheral driver ではなく、START / address / ACK / write byte / read byte / master ACK / STOP を扱う cooperative な slave protocol engine として置く。 `SlaveLineDriver` 越しに SCL/SDA を観測・drive するため、native test では `VirtualOpenDrainBus` と組み合わせて software I2C master の出力を同じ仮想 bus 上で読み取れる。

これにより、当初の `MockSlave` 案は test-only mock ではなく、将来の `SlaveDriver` protocol engine 候補として本体側に残せる形になった。 native test では probe ACK、write、read-only、write-then-read、address NACK、data NACK、clock stretch timeout、STOP 時 SDA stuck-low、read 末尾 master NACK 観測を固定している。

実機 slave driver 化は別段階で扱う。実機 slave は edge 捕捉、ACK setup、clock stretching の timing 制約が master より厳しいため、まずは native virtual bus + protocol engine として protocol semantic を固定し、callback / register-map adapter や MCU peripheral driver との接続は後続設計とする。

## ESP-IDF I2C variant の実装方針

`variants::frameworks::espidf` の I2C master は単一 variant とし、ESP-IDF 世代差は `detail/espidf_version.hpp` の feature detection と backend 実装で吸収する。 `ARDUINO` 定義の有無では無効化しない。 Arduino-on-IDF や ESP-IDF project with Arduino component では arduino / espidf variants が同時に存在しうるため、既定の勝者選択は scan 順に任せ、espidf 実装は suffix 付き実名 (`i2c::Bus_espidf`) で明示利用できる状態を維持する。

ESP-IDF gen5 I2C master backend (`driver/i2c_master.h`) は `freq == 0`、アドレス範囲外、`i2c_master_probe` で表現できない 10-bit probe を driver 呼び出し前に `INVALID_ARGUMENT` として扱う。 10-bit address の通常 transfer は device config 経由で扱い、probe path だけを制限する。

通常 transfer は `i2c_master_bus_add_device` で得た device handle を Bus 内に保持し、同じ address / frequency / address bit length / SCL wait 設定の連続アクセスでは再利用する。 `probe()` は scan 用の軽量経路として `i2c_master_probe` を直接使い、device handle cache とは独立させる。 設定が変わった場合や `release()` / `attach()` では cached device を外してから bus handle を切り替える。

`driver/i2c_master.h` が無く `driver/i2c.h` がある ESP-IDF 世代では gen4 master backend (`driver/i2c.h`) を使う。これは Arduino-ESP32 2.x 系のように SPI master driver はあるが gen5 I2C master driver は無い環境で、ESP-IDF I2C variant を明示利用できるようにするためである。 gen4 backend は 7-bit address の master transfer / probe を対象とし、10-bit address は driver 呼び出し前に `INVALID_ARGUMENT` とする。

## RAII (ScopedAccess) との組み合わせ

各 sugar は内部で `beginAccess` → `transfer` → `endAccess` を行う。 連続アクセスは `ScopedAccess` で外側を囲う。

```cpp
{
    m5::hal::v1::bus::ScopedAccess access{accessor};
    if (access.has_error()) return;

    accessor.writeRegister(REG_CTRL_MEAS, VAL_MODE);
    accessor.readRegister(REG_DATA, dst_span);
    // 内部の beginAccess/endAccess は ScopedAccess の 1 段で吸収される
}
```

## 関連

- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離 + RAII
- [transfer_desc.md](transfer_desc.md) — `i2c::TransferDesc` 詳細
- [data_io.md](data_io.md) — Source / Sink + Limited 装飾
- [variants.md](variants.md) — variant 機構 (arduino / software / espidf の配置)
