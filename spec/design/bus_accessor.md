# design/bus_accessor — Bus と Accessor の責務分離

通信バスへのアクセスは **Bus と Accessor の 2 層構造** で表現する (I2C / SPI / UART 共通)。 利用者には「Accessor を通してバスを操作する」 単一の流儀を提供しつつ、 高度な利用者には sugar を介さず Bus を直接呼び出す低レイヤ経路も開けておく。

## なぜ 2 層構造か

I2C / SPI / UART は **物理バス上に複数の通信相手を載せる** モデル。 同じ Bus に複数 Accessor (異なる通信相手) を作って共有することが自然。

- **Bus** = 物理バス (例: ESP32 の `I2C_NUM_0`) を表す。 1 つの物理リソースに対し排他制御を持つ
- **Accessor** = 1 つの通信相手 (例: アドレス 0x76 の BME280) を表す。 通信パラメータ (速度・タイムアウト等) を保持

利用者は Accessor 経由で通信する (Bus は排他制御の単位として裏で動く)。

## Bus の責務 (最小化)

- 物理バスの初期化と解放 (`init` / `release`)
- 外部 native handle (Arduino `TwoWire` 等) との紐付け (`attach`)
- 排他制御 (`lock(Accessor*)` / `unlock(Accessor*)`)
- atomic な通信動作 (`transfer`)

Bus は **Accessor を所有しない**。 利用者が Bus と Accessor を別個に保持する (v0.0.x の `Bus::beginAccess(AccessConfig&) -> Accessor*` 形 factory は撤廃済)。

## Accessor の責務

- 1 つの通信相手の表現 (I2C アドレス + 通信パラメータ等を `AccessConfig` で保持)
- アクセス期間の宣言 (`beginAccess` / `endAccess`、 内部で Bus を lock/unlock)
- 利用者向け sugar (`write` / `read` / `transfer` / `writeRegister` / `readRegister` / `probe`)
- 連続 atomic のための **depth counter** (`_access_depth`) — sugar 内部呼び出しと外側の明示 `beginAccess` を入れ子で扱えるよう、 1 段の lock で吸収する

SPI の CS assert/deassert のような kind 固有の電気的 session は、 bus 共通の
`beginAccess` / `endAccess` には含めない。 `beginAccess` はあくまで排他期間で、
複数の kind 固有 transaction を含んでよい。SPI では `beginTransaction` /
`endTransaction` が CS 区間を表し、単発 sugar は内部で transaction を自動開始・終了する。

## 二層の利用路線 (混在禁止)

通信路は以下 2 路線が併存。 **同じ Bus 上で路線をまたいで操作することは未サポート** (利用者責任)。

### 通常路線 (Accessor sugar 利用)

```cpp
m5::hal::v1::i2c::I2CMasterAccessor accessor{i2c_bus, acc_cfg};

// 単発: sugar 内部で beginAccess → transfer → endAccess
accessor.writeRegister(REG_CTRL, VAL_MODE);

// 連続 atomic: ScopedAccess で beginAccess / endAccess を RAII
{
    m5::hal::v1::bus::ScopedAccess access{accessor};
    if (access.has_error()) { /* lock 競合 */ return; }
    accessor.writeRegister(REG_CTRL, VAL_MODE);
    accessor.readRegister(REG_DATA, dst_span);
}  // ここで endAccess
```

- 利用者は排他制御を意識しない
- `beginAccess`/`endAccess` を sugar 内部でも外側でも呼べる (depth counter で吸収)

SPI では、単発 sugar は内部で `beginAccess → beginTransaction → transfer →
endTransaction → endAccess` を行う。明示的に CS を維持したい場合は、
`beginAccess` ではなく SPI 固有の `beginTransaction` を使う。

```cpp
m5::hal::v1::spi::SPIMasterAccessor dev{spi_bus, spi_cfg};

// 単発: CS はこの write の前後だけ assert される
dev.write(tx_span);

// 複数 transfer で CS を維持する場合
dev.beginTransaction();
dev.write(command_span);
dev.write(data_span);
dev.endTransaction();
```

### 低レイヤ路線 (Bus 直接呼び出し、 「熟知している前提」)

```cpp
// bus.lock(&accessor) → bus.transfer(&accessor, ...) → bus.unlock(&accessor)
m5::hal::v1::bus::ScopedLock lock{i2c_bus, &accessor};
if (lock.has_error()) return;

i2c_bus.transfer(&accessor, cfg, desc, &tx_source, &rx_sink);
```

- Accessor を作るのは必須 (`lock` owner は常に valid な `Accessor*`、 nullptr 不可)
- 「他者が unlock しちゃう」 誤用は identity 比較で検出可能

## なぜ「Accessor* nullptr 不可」 か

`lock` / `unlock` の owner identifier に Accessor へのポインタを使う。 これにより:

- 排他保有者が runtime で確認可能 (誰が lock しているか)
- 「他人の lock を奪う」 「他人が unlock する」 等の誤用が identity 比較で検出可能
- nullptr 可にすると「誰が持っているか不明な lock」 が生まれ、 検出機構が成立しない

## 動詞規約 (要約)

| 動詞 | 用途 |
|---|---|
| `init` / `release` | Bus lifecycle |
| `attach` / (`detach`) | 外部 native handle |
| `lock` / `unlock` | Bus 排他制御 (公開、 `Accessor*` 必須) |
| `beginAccess` / `endAccess` | Accessor アクセス期間 (depth counter で nest) |
| `beginTransaction` / `endTransaction` | kind 固有 transaction 期間 (SPI の CS assert 区間など) |
| `transfer` | atomic 通信 |
| `read` / `write` | Accessor sugar |

旧 API (`startWrite` / `startRead` / `stop` chain や `Bus::beginAccess(AccessConfig&)` factory) は **使わない** (削除済)。 詳細な動詞使い分けは [../style/coding_style.md](../style/coding_style.md) §動詞規約。

## RAII 型

| 型 | 対象 | コンストラクタ引数 |
|---|---|---|
| `m5::hal::v1::bus::ScopedAccess` | Accessor の `beginAccess` / `endAccess` | `Accessor&` (+ timeout_ms) |
| `m5::hal::v1::bus::ScopedLock` | Bus の `lock` / `unlock` | `Bus&`, `Accessor*` (+ timeout_ms) |

両者とも:
- ctor 内で `beginAccess` / `lock` を試行
- 失敗 (lock 競合) は `has_error()` / `error()` で確認
- dtor は失敗時には `unlock` しない (誤って他者の lock を解かないため)

## クラス階層

```cpp
namespace m5::hal::v1::bus {
    struct BusConfig { /* 共通基底 (空マーカ) */ };
    struct AccessConfig { /* 共通基底 (空マーカ) */ };
    struct TransferDesc { /* 共通基底 (空マーカ、 詳細は design/transfer_desc.md) */ };

    class Bus {
        virtual error_t init(const BusConfig& cfg)  = 0;
        virtual void    release()                   = 0;
        virtual error_t lock(Accessor* owner, uint32_t timeout_ms = 0);    // depth 1 段
        virtual error_t unlock(Accessor* owner);
        // attach / transfer は kind 別派生 (I2CBus / SPIBus / ...) で定義
    };

    class Accessor {
        Bus& _bus;
        size_t _access_depth = 0;
        error_t beginAccess(uint32_t timeout_ms = 0);   // 0→1 のみ bus.lock
        error_t endAccess();                            // 1→0 のみ bus.unlock
        bool inAccess() const;
    };
}

namespace m5::hal::v1::i2c {
    struct I2CBusConfig          : public bus::BusConfig    { /* pin_scl, pin_sda */ };
    struct I2CMasterAccessConfig : public bus::AccessConfig {
        /* freq, timeout_ms, i2c_addr, address_is_10bit, register_address_bytes, use_restart */
    };
    struct TransferDesc          : public bus::TransferDesc { /* inline prefix buffer, 詳細は design/transfer_desc.md */ };

    class I2CBus : public bus::Bus {
        virtual error_t attach(/* TwoWire&, i2c_master_bus_handle_t, etc */) = 0;
        virtual m5::stl::expected<size_t, error_t> transfer(
            bus::Accessor* owner,
            const I2CMasterAccessConfig& cfg,
            const TransferDesc& desc,
            data::Source* tx,
            data::Sink*   rx) = 0;
    };

    class I2CMasterAccessor : public bus::Accessor {
        I2CMasterAccessConfig _access_config;
        // ctor は I2CBus& 受け (コンパイル時 kind verify)
        I2CMasterAccessor(I2CBus& bus, const I2CMasterAccessConfig& cfg);

        // sugar (全て内部で beginAccess → transfer → endAccess)
        expected<size_t, error_t> transfer(const TransferDesc& desc, data::ConstDataSpan tx, data::DataSpan rx);
        expected<size_t, error_t> write(data::ConstDataSpan tx);
        expected<size_t, error_t> read(data::DataSpan rx);

        // register sugar (template + SFINAE: unsigned integral && sizeof ≤ 2、 M5UU M5UnitComponent と整合)
        template <typename TReg /* ≤ 2 byte */> expected<size_t, error_t> writeRegister(TReg reg, data::ConstDataSpan value);
        template <typename TReg /* ≤ 2 byte */> expected<size_t, error_t> writeRegister(TReg reg, uint8_t value);
        template <typename TReg /* ≤ 2 byte */> expected<size_t, error_t> readRegister(TReg reg, data::DataSpan dst);
        template <typename TReg /* ≤ 2 byte */> expected<uint8_t, error_t> readRegister(TReg reg);

        // signed literal overload (`readRegister(0x00)`): register_address_bytes で幅を決める
        expected<size_t, error_t> writeRegister(int reg, data::ConstDataSpan value);
        expected<size_t, error_t> writeRegister(int reg, uint8_t value);
        expected<size_t, error_t> writeRegister(int reg, const uint8_t* tx, size_t len);
        expected<size_t, error_t> readRegister(int reg, data::DataSpan dst);
        expected<size_t, error_t> readRegister(int reg, uint8_t* dst, size_t len);
        expected<uint8_t, error_t> readRegister(int reg);

        // probe (0-byte write、 device 存在確認、 wire 上で address+W 送出 + ACK チェック)
        expected<void, error_t> probe();
    };
}
```

I2C kind 固有の詳細 (BusConfig / AccessConfig のフィールド意味、 sugar の挙動、 probe path の wire 規約) は [i2c.md](i2c.md)、 TransferDesc 詳細は [transfer_desc.md](transfer_desc.md)、 Source/Sink 詳細は [data_io.md](data_io.md)。

## M5UnitUnified との関係

M5UnitUnified などの上位ライブラリから直接利用できるよう、 v1 では sugar と低レイヤ API の責務を明確に分ける。 詳細は [../style/migration.md](../style/migration.md)。

register sugar の typed signature (`template <TReg> writeRegister/readRegister`、 `sizeof(TReg) ≤ 2`) は M5UnitComponent 系の利用形態に合わせている。 Arduino-style の直値呼び出しは signed literal overload が受け、 `I2CMasterAccessConfig::register_address_bytes` (`0` / `1` = 1 byte、 `2` = 2 byte) で幅を決める。
