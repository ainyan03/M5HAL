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
m5::hal::v1::i2c::MasterAccessor accessor{i2c_bus, acc_cfg};

// 単発: sugar 内部で beginAccess → transfer → endAccess
accessor.writeRegister(REG_CTRL, VAL_MODE);

// 連続 atomic: ScopedAccess で beginAccess / endAccess を RAII
// (timeout は明示が本筋。省略 = 無限待ちのシュガー)
{
    m5::hal::v1::bus::ScopedAccess access{accessor, 100};
    if (access.has_error()) { /* lock 競合 (timeout) */ return; }
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
m5::hal::v1::spi::MasterAccessor dev{spi_bus, spi_cfg};

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

## Bus の保持 (登録制 BusGroup)

**所有と保持は別の関心事**: v1 が撤廃したのは「HAL が bus を生成・所有する」
ことであり、参照表を持つことではない。`M5_Hal` は kind ごとに **非所有の
bus レジストリ** (`bus::BusGroup<IBus>`、`M5_Hal.I2C` / `M5_Hal.SPI` /
`M5_Hal.UART` / `M5_Hal.I2S`) を持つ:

```cpp
static m5hal::spi::Bus spi_bus;                  // 所有は利用者
M5_Hal.SPI.addBus(&spi_bus, 1);                  // スロット 1 として公開
auto* bus = M5_Hal.SPI.getBus(1);                // 参照 (未登録 = nullptr)
```

- **多対一エイリアシング**: 同じポインタを複数スロットに登録できる —
  「スロット 1 (SD) と 2 (LCD) は同一物理バス」がそのまま表現できる
  (レジストリが実体でなく参照を持つ設計にした時点で問題が消える)
- **スロットの意味規約は上位層が持つ**: ボードサポート層が named constant
  でスロット番号を定義する。HAL は注入点 (表) だけを提供する
- **寿命規約**: 登録する bus は static 寿命を推奨。破棄や `release()` の
  前に `removeBus` で登録を外す。レジストリは決して delete しない
- 登録は起動時操作で、以後は読み取り専用として扱う (GPIOGroup と同じ。
  ロックなし)
- 登録しなければ空のまま — bus を直接渡す既存の書き方は何も変わらない。
  レジストリは配線記述の利便であって強制ではない

## 遅延バインド (unbound 構築 + typed bind)

Accessor は **bus なしで構築して後から束ねられる** — 「グローバルにドライバ
object を置き、`setup()` で `begin(bus)`」という M5UU / Adafruit 系の標準形を
直接書くための形:

```cpp
m5::hal::v1::i2c::MasterAccessor dev{acc_cfg};   // unbound (config のみ)

void setup()
{
    ...
    dev.bind(i2c_bus);   // kind-typed: 非 I2C bus はコンパイルエラー
}
```

- `bind()` は **kind typed** (`i2c::MasterAccessor::bind(i2c::IBus&)`)。typed init
  と同じ思想で、kind 違いの bus を渡す誤りはコンパイル時に落ちる
- アクセスウィンドウが開いている間の `bind()` は `INVALID_ARGUMENT`
  (開いている窓は旧 bus の lock を保持しているため)。窓の外での rebind は許される
- **unbound ゲートは窓の入口のみ**: `beginAccess` / UART の `beginTxAccess` /
  `beginRxAccess` が unbound を検出する (debug = assert、release =
  `INVALID_ARGUMENT`)。sugar はすべて窓の入口を通るため実質全経路が
  カバーされ、`transfer` などのホットパスにはチェックを置かない
- `isBound()` で束縛状態を確認できる。unbound のまま `getBus()` を呼ぶのは
  契約違反 (ゲートなしの null 参照)

## なぜ「Accessor* nullptr 不可」 か

`lock` / `unlock` の owner identifier に Accessor へのポインタを使う。 これにより:

- 排他保有者が runtime で確認可能 (誰が lock しているか)
- 「他人の lock を奪う」 「他人が unlock する」 等の誤用が identity 比較で検出可能
- nullptr 可にすると「誰が持っているか不明な lock」 が生まれ、 検出機構が成立しない

## 排他制御の意味論 (常時 mutex)

`Bus` は [runtime.md](runtime.md) の `runtime::Mutex` を **常時内蔵** し、 `lock` は実際の
待ち合わせを行う:

- `lock(owner, timeout_ms)` は保有者がいる間 `timeout_ms` まで **待つ**。 取得できなければ
  **`TIMEOUT_ERROR`** (`timeout_ms == 0` は即時 try-lock、 `types::TIMEOUT_FOREVER` = 既定 =
  取得まで無限に待つ)。 lock timeout は **呼び出しコンテキストの属性**であり config には
  置かない: 明示の待ち時間を渡すのが本筋で、 省略 (無限待ち) は「タイムアウト後の
  処理が面倒な場面」向けのシュガー。 sugar (readRegister 等) の内部 lock も無限待ち —
  単発呼び出しと `ScopedAccess` で挙動が揃う
- **非再帰**: 同一タスクの再 lock (同一 owner、 または同一バス上の別 Accessor) も timeout
  まで待って `TIMEOUT_ERROR` — 無限待ち (既定) では **デッドロック** になり task watchdog が
  検出する (fail-loud)。 入れ子は `Accessor::beginAccess` の depth counter が吸収する
  ので、 `Bus::lock` 自体はアクセスウィンドウあたり高々 1 回しか呼ばれない
- **task-context only**: ISR から呼ばない (runtime kind の契約に従う)
- **timeout 粒度**: runtime variant 依存 (FreeRTOS 環境では tick = 既定 10 ms)
- `_lock_owner` (owner ポインタ) の更新は lock / unlock とも mutex 保持下で行う。 owner 不一致の
  `unlock` は mutex に触れず `INVALID_ARGUMENT`

> **互換性注記**: 以前のリリースの `lock` は owner ポインタ比較のみの即時判定で、 競合は
> `BUSY` を返し timeout は予約引数だった。 mutex 実体化と同時に競合エラーは `TIMEOUT_ERROR`
> へ変更された (**BREAKING**)。

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

SPI には CS assert/deassert 区間用の `spi::ScopedTransaction` がある
(`beginTransaction` / `endTransaction` を同じ polarity 規約で包む。
[spi.md](spi.md))。

### guarded (解放エラーも観測する厳格イディオム)

RAII の destructor は解放 (`endAccess` / `endTransaction`) の失敗を報告
できない。解放エラーまで観測したい厳格な用途 (bring-up コード等) には
`bus::guarded(begin, body, end)` を使う:

```cpp
auto r = m5::hal::v1::bus::guarded(
    [&] { return dev.beginTransaction(); },
    [&] { return dev.write(init_seq); },
    [&] { return dev.endTransaction(); });
```

policy (全 accessor sugar が内部で従うものと同一):
- `begin` 失敗は即 return (解放するものがない)
- `body` のエラーは `end` のエラーより優先
- **body 成功 + end 失敗は end のエラーを返す** — depth counter の破損を
  黙殺しない。`end` は `begin` が成功した限り必ず 1 回呼ばれる

## クラス階層

```cpp
namespace m5::hal::v1::bus {
    struct IBusConfig { /* 共通基底 (空マーカ) */ };
    struct IAccessConfig { /* 共通基底 (空マーカ) */ };
    struct ITransferDesc { /* 共通基底 (空マーカ、 詳細は design/transfer_desc.md) */ };

    class IBus {
        virtual error_t init(const IBusConfig& cfg)  = 0;
        virtual void    release()                    = 0;
        virtual error_t lock(IAccessor* owner, uint32_t timeout_ms = types::TIMEOUT_FOREVER);  // mutex 待ち合わせ、 競合 = TIMEOUT_ERROR
        virtual error_t unlock(IAccessor* owner);
        // attach / transfer は kind 別派生 (i2c::IBus / spi::IBus / ...) で定義
        runtime::Mutex _mutex;   // 常時内蔵 (§排他制御の意味論)
    };

    class IAccessor {
        IBus& _bus;
        size_t _access_depth = 0;
        error_t beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);  // 0→1 のみ bus.lock
        error_t endAccess();                            // 1→0 のみ bus.unlock
        bool inAccess() const;
    };
}

namespace m5::hal::v1::i2c {
    struct IBusConfig          : public bus::IBusConfig    { /* pin_scl, pin_sda */ };
    struct MasterAccessConfig : public bus::IAccessConfig {
        /* freq, wire_timeout_ms, i2c_addr, address_is_10bit, register_address_bytes, use_restart */
    };
    struct TransferDesc          : public bus::ITransferDesc { /* inline prefix buffer, 詳細は design/transfer_desc.md */ };

    class IBus : public bus::IBus {
        virtual error_t attach(/* TwoWire&, i2c_master_bus_handle_t, etc */) = 0;
        virtual result_t<size_t> transfer(
            bus::IAccessor* owner,
            const MasterAccessConfig& cfg,
            const TransferDesc& desc,
            data::Source* tx,
            data::Sink*   rx) = 0;
    };

    class MasterAccessor : public bus::IAccessor {
        MasterAccessConfig _access_config;
        // ctor は IBus& 受け (コンパイル時 kind verify)
        MasterAccessor(IBus& bus, const MasterAccessConfig& cfg);

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

register sugar の typed signature (`template <TReg> writeRegister/readRegister`、 `sizeof(TReg) ≤ 2`) は M5UnitComponent 系の利用形態に合わせている。 Arduino-style の直値呼び出しは signed literal overload が受け、 `MasterAccessConfig::register_address_bytes` (`0` / `1` = 1 byte、 `2` = 2 byte) で幅を決める。
