# design/transfer_desc — Transfer 補助型 (TransferDesc)

`Bus::transfer` に渡す **per-call メタ情報** は `m5::hal::v1::bus::TransferDesc` を空 marker base とし、 kind ごとに派生 (`i2c::TransferDesc`, `spi::TransferDesc`, …) を持つ。 Source/Sink が「データ実体」 を表すのと対照に、 TransferDesc は「この 1 回の transfer に付随する付帯情報 (register address 等の先頭バイト列、 protocol 固有のフラグ等)」 を表す。

## 設計原則

- 共通基底 `bus::TransferDesc` は **vtable も `getBusKind` も持たない空 marker** (派生サイズに影響しない、 EBO で cost ゼロ)。 polymorphism 経由で扱う想定なし
- per-call メタ情報の構造は **kind ごとに独立** に定義する。 共通フィールドは基底に上げない (I2C は `prefix` バイト列、 SPI は `command + address + dummy_cycles` 等の構造化フィールド)
- prefix 系の先頭バイト列は **inline buffer で値型保持** する (caller がローカル配列を作ってポインタを渡す手間を省く、 lifetime も desc に揃う)
- `BusConfig` / `AccessConfig` と同じ「共通基底 + kind ごとの派生」 パターンを踏襲

## bus::TransferDesc (空 marker base)

```cpp
namespace m5::hal::v1::bus {

struct TransferDesc {};   // empty marker base、 virtual も持たない

}
```

「kind ごとの TransferDesc は `bus::TransferDesc` という型カテゴリの一員」 という規約を型で示すだけのマーカー。 polymorphism 経由で扱う場面がない (`Bus::transfer` は kind ごとに固有の `<kind>::TransferDesc const&` を取るため、 基底ポインタへの up-cast 経路が存在しない)。

## i2c::TransferDesc

```cpp
namespace m5::hal::v1::i2c {

struct TransferDesc : public bus::TransferDesc {
    static constexpr size_t PREFIX_CAPACITY = 8;
    uint8_t prefix[PREFIX_CAPACITY] = {};
    uint8_t prefix_len              = 0;

    constexpr TransferDesc() = default;

    // 1-引数 ctor: unsigned integral 型を big-endian で展開 (sizeof ≤ 4)
    template <typename T, typename std::enable_if<
        std::is_integral<T>::value && std::is_unsigned<T>::value && sizeof(T) <= 4, int>::type = 0>
    constexpr explicit TransferDesc(T reg)
        : prefix{}, prefix_len{sizeof(T)} {
        // big-endian (MSB first) で prefix に格納
        for (size_t i = 0; i < sizeof(T); ++i) {
            prefix[i] = static_cast<uint8_t>((reg >> ((sizeof(T) - 1 - i) * 8)) & 0xFF);
        }
    }

    // 2〜4 引数 ctor: 個別 byte 指定 (little-endian で送りたい等の用途)
    constexpr TransferDesc(uint8_t, uint8_t);
    constexpr TransferDesc(uint8_t, uint8_t, uint8_t);
    constexpr TransferDesc(uint8_t, uint8_t, uint8_t, uint8_t);
};

}
```

### prefix の意味

- I2C transfer で先頭に送るバイト列 (典型は register address 1〜4 byte + sub-command 用の余裕)
- 通常は `prefix_len > 0` で wire 上に prefix を送出してから `tx` / `rx` を処理
- `prefix_len == 0` (空 prefix) + `tx == nullptr` + `rx == nullptr` の場合は **probe path** として扱う (address+W 送出 + ACK チェック、 詳細は [i2c.md](i2c.md))
- `PREFIX_CAPACITY = 8` を超える prefix は **仕様外**。 必要なら `tx` Source 側に積む (異常系として `INVALID_ARGUMENT` を返す)

### 1-引数 ctor (template + SFINAE)

`sizeof(T) ≤ 4` 制約の template ctor。 big-endian (MSB first) で `prefix` に展開する。

| 呼び出し例 | prefix の中身 |
|---|---|
| `TransferDesc{uint8_t{0xD0}}` | `[0xD0]` (1 byte) |
| `TransferDesc{uint16_t{0x1234}}` | `[0x12, 0x34]` (2 byte BE) |
| `TransferDesc{uint32_t{0xDEADBEEF}}` | `[0xDE, 0xAD, 0xBE, 0xEF]` (4 byte BE) |
| `TransferDesc{0xD0}` | **コンパイルエラー** (`int` リテラルは SFINAE で弾かれる) |

caller は **型付き定数定義スタイル** が前提 (M5UU `M5UnitComponent` と同じ慣習):

```cpp
static constexpr uint8_t  REG_CTRL = 0xF4;
static constexpr uint16_t REG_X    = 0x1234;
accessor.transfer(TransferDesc{REG_CTRL}, tx, rx);
accessor.transfer(TransferDesc{REG_X},    tx, rx);
```

アドホックに register read/write sugar を呼ぶだけなら
`accessor.readRegister(0xD0)` の signed literal overload も使える。
この場合の幅は `I2CMasterAccessConfig::register_address_bytes`
(`0` / `1` = 1 byte、 `2` = 2 byte) で決まる。 `TransferDesc`
自体の 1-引数 ctor は低レベル API として型明示を要求する。

### 2〜4 引数 ctor

個別 byte 指定 (例: register address を little-endian で送りたい、 型で表現できないビット列、 等の用途)。 template 1-引数 ctor と用途を分ける。

### 将来の拡張候補

- `bool use_restart_override` — `cfg.use_restart` の per-call override
- `bool last_byte_nack` — read 末尾の NACK 制御

これらは必要が見えた時点で追加する (先回り追加なし)。

## sugar (≤ 2) と TransferDesc ctor (≤ 4) の非対称

| API | SFINAE 制約 | 用途 |
|---|---|---|
| sugar `writeRegister<TReg>` / `readRegister<TReg>` | `sizeof(TReg) ≤ 2` | M5UU `M5UnitComponent` と signature 整合。 typical case (1-byte / 2-byte register address) |
| sugar `writeRegister(int)` / `readRegister(int)` | `register_address_bytes` が `0` / `1` / `2` | Arduino-style 直値呼び出し。 `0` / `1` は 1 byte、 `2` は 2 byte |
| `TransferDesc` 1-引数 ctor | `sizeof(T) ≤ 4` | 低レイヤ利用者向け自由度。 4-byte register address (極めて稀) は caller が `TransferDesc{uint32_t{...}}` で対応 |

「典型ケースは sugar、 高度ケースは TransferDesc 直接組み立て」 の棲み分け。 型付き sugar は `TransferDesc{reg}` template ctor に委譲する。 signed literal sugar は `register_address_bytes` を正規化してから `TransferDesc` を組み立てる。

sugar 仕様は [i2c.md](i2c.md) §register sugar を参照。

## byte order の規約

- **register address は wire 上で big-endian (MSB first)** 固定 (I2C 業界標準)
- caller は host endian の `uint16_t` 等を渡す → library が big-endian で送る
- value 側のサイズと byte order は **caller 責任** (M5UU `M5UnitComponent` が `writeRegister16BE` 等の name embedding 方式で提供する想定、 M5HAL HAL 層は持たない)

little-endian register address が必要なケースは想定外 (実用上ほぼ存在しない)。 もし必要なら caller が `TransferDesc{lo, hi}` のように 2 引数 ctor で個別 byte 指定する。

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| 共通基底に prefix フィールド | kind ごとに prefix の意味/構造が異なる (SPI は command + address + dummy 等)。 共通化のうまみがない |
| `getBusKind()` virtual hook | polymorphism 経由で扱う場面がない (`Bus::transfer` は kind ごとに固有の `<kind>::TransferDesc const&` を取る) |
| virtual dtor | 同上 (基底ポインタ経由で派生を delete する経路がない) |
| 動的長 prefix (heap allocation) | inline buffer で typical をカバーできる。 過剰サイズは tx 側に積む規約で十分 |

## spi::TransferDesc

SPI は I2C の prefix buffer ではなく、command / address / dummy / D/C phase を
構造化フィールドで表す:

```cpp
namespace m5::hal::v1::spi {

struct TransferDesc : public bus::TransferDesc {
    bool     dc_level_valid;
    bool     dc_level;
    uint32_t command;
    uint32_t address;
    uint8_t  command_bytes;
    uint8_t  address_bytes;
    uint8_t  dummy_cycles;
    int8_t   command_dc_level;
    int8_t   address_dc_level;
    int8_t   data_dc_level;
};

}
```

- per-call メタ情報の構造は kind ごとの protocol に最適化して自由設計
- 「揃えるのは配置 (位置) と命名 (型名 `TransferDesc`) だけ」、 共通フィールド名や順序を揃えない
- 「prefix を inline buffer で値型保持」 は I2C 固有の判断 (短いバイト列を caller の手間なく渡す)。 SPI のように構造化されているなら inline buffer は不要
- 1-引数 template ctor を持たせるかは kind ごとに判断 (SPI は構造化中心なので不要、 UART は raw bytes 中心なので I2C と同じパターンが効くかも)

## 関連

- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離
- [data_io.md](data_io.md) — データ実体 (Source / Sink)
- [i2c.md](i2c.md) — I2C kind 固有の transfer 仕様 (probe path 等)
