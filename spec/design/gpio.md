# design/gpio — GPIO 抽象設計

M5HAL v1 の GPIO 抽象は `Pin`、 `IPort`、 `IGPIO`、 `GPIOGroup` で構成する。 caller には `gpio_number_t` 空間と `Pin` 値型を提供し、 encoded 表現は variant 実装側に閉じ込める。

## 設計原則

- **encoded 隠蔽** — Pin / IPort / IGPIO 操作 API は `gpio_local_pin_t` (= IGPIO 内 local pin 空間) で受ける。 internal 表現 (絶対 gpio / bit 位置 / pin_mask / index 等) は variant 実装が支配し、 利用者には見せない。 グローバル `gpio_number_t` の解決は `GPIOGroup` が担う
- **高速 path を阻害しない** — `writeHigh` / `writeLow` の独立 hook を持つ
- **契約ベース (assert + UB)** — 範囲外 `gpio_local_pin_t` は variant 実装の `_fromLocalPin` 内で **assert で debug 即死、 release UB**。 `expected` で穢さない
- **constexpr 化可能** — `IPort` / `IGPIO` は protected non-virtual dtor を持つ
- **expander 統合** — MCU GPIO と I/O expander を単一の `gpio_number_t` 空間で扱う
- **抽象に `I` prefix** — variant 具象との衝突を避ける

## 主要型 (m5::hal::v1::gpio)

| 型 | 役割 | 配置 |
|---|---|---|
| `Pin` | 単一 pin 操作の値型 facade (`IPort*` + encoded_num) | `hal/v1/gpio/port.hpp` |
| `IPort` | bank-level GPIO 操作 (write/read/setMode、 encoded hook を抱える) | `hal/v1/gpio/port.hpp` |
| `IGPIO` | IPort のコンテナ + dispatch + Pin factory | `hal/v1/gpio/gpio.hpp` |
| `GPIOGroup` | MCU + expander 統合 (グローバル resolver、 IGPIO とは独立、 virtual なし単一実装) | `hal/v1/gpio/group.hpp` |

## Pin (値型 facade)

```cpp
namespace m5::hal::v1::gpio {

class Pin {
public:
    Pin();   // default = invalid

    void write(bool v) const;
    void writeHigh() const;
    void writeLow() const;
    bool read() const;
    void setMode(types::gpio_mode_t mode) const;

    types::gpio_local_pin_t getLocalPin() const;
    IPort* getPort() const;
    bool   isValid() const;       // _owner != nullptr

private:
    Pin(IPort* owner, uint32_t encoded_num);   // private + friend IPort
    IPort*   _owner;
    uint32_t _encoded_num;
    friend class IPort;
};

}
```

### 契約

- **値型**: caller がローカル変数として保持できる
- **ctor は private** — `IPort::getPin(gpio_local_pin_t)` factory 経由でのみ生成 (`IGPIO::getPin` も同 factory に委譲)。 caller が encoded_num を直接組み立てる経路を持たせない
- **encoded_num の意味は IPort 実装に依存** — 絶対 gpio 番号 / bit 位置 / pin_mask / 任意 index など。 `uint32_t` 幅で pin_mask 直格納も許容 (espressif::esp32 が採用)
- **操作 hook = 1 段 virtual** — Pin の操作は `IPort` の encoded hook (`_writePinEncoded` 等) を使う
- **default 構築は invalid** — `Pin{}` は `_owner == nullptr`。 `isValid()` で判定可

## IPort (bank-level 操作)

```cpp
namespace m5::hal::v1::gpio {

class IPort {
public:
    // public 操作 API (gpio_local_pin_t 受け)
    void write(types::gpio_local_pin_t pin_index, bool v);
    void writeHigh(types::gpio_local_pin_t pin_index);
    void writeLow(types::gpio_local_pin_t pin_index);
    bool read(types::gpio_local_pin_t pin_index);
    void setMode(types::gpio_local_pin_t pin_index, types::gpio_mode_t mode);

    // Pin factory
    Pin getPin(types::gpio_local_pin_t pin_index);

protected:
    // encoded hook (variant が override 必須)
    virtual void _writePinEncoded(uint32_t encoded_num, bool v) = 0;
    virtual bool _readPinEncoded(uint32_t encoded_num)          = 0;
    virtual void _setPinModeEncoded(uint32_t encoded_num, types::gpio_mode_t mode) = 0;

    // 高速 path (default = _writePinEncoded に dispatch、 variant 任意 override)
    virtual void _writePinEncodedHigh(uint32_t encoded_num);
    virtual void _writePinEncodedLow(uint32_t encoded_num);

    // 変換 API
    virtual types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const  = 0;
    virtual uint32_t _fromLocalPin(types::gpio_local_pin_t pin_index) const  = 0;

    ~IPort() = default;   // protected non-virtual → polymorphic delete 静的禁止
    friend class Pin;
};

}
```

### 契約

- **public API は gpio_local_pin_t 受け、 内部で encoded 変換** — `_fromLocalPin(pin_index)` を呼んでから encoded hook へ dispatch
- **invalid pin_index は assert で debug 即死、 release UB** — `_fromLocalPin` 実装の責務 (`expected` で wrap しない)
- **`writeHigh` / `writeLow` 高速 path** — default 実装は `_writePinEncoded(encoded, true/false)` に委譲し、 variant は override できる
- **Pin factory** — `getPin(pin_index)` 経由でのみ Pin 値型を発行。 `_fromLocalPin` で encoded 化し、 `Pin{this, encoded}` の private ctor (`IPort` が friend) を呼ぶ
- **protected non-virtual dtor** — `delete (IPort*)` を許可しない

## IGPIO (IPort のコンテナ + 解決機)

```cpp
namespace m5::hal::v1::gpio {

class IGPIO {
public:
    // dispatch / 解決
    virtual IPort* portForPin(types::gpio_local_pin_t local_pin) const = 0;
    virtual IPort* getPort(uint8_t port_index) const                   = 0;

    // 容量
    virtual uint16_t getPinCount() const  = 0;
    virtual uint8_t  getPortCount() const = 0;

    // 範囲判定 (default 実装: local_pin < getPinCount())
    virtual bool isValid(types::gpio_local_pin_t local_pin) const;

    // Pin factory (default 実装: portForPin → IPort::getPin)
    virtual Pin getPin(types::gpio_local_pin_t local_pin) const;

protected:
    ~IGPIO() = default;   // protected non-virtual
};

}
```

### 役割の純化

`IGPIO` は **IPort の管理に専念**。 ピン操作 API (write / read / setMode 等) は持たず、 caller は IPort または Pin を経由する:

| 用途 | 推奨 path |
|---|---|
| ピン単位の操作 | `gpio.getPin(num).write(v)` |
| バンク単位の操作 | `gpio.portForPin(num)->write(num, v)` |
| 特定の Port を直接掴む | `gpio.getPort(port_index)` |

### 契約

- **`portForPin(local_pin)`** — variant 内部で dispatch。 複数 IPort を持つ variant (ESP32-S3 等 `SOC_GPIO_PIN_COUNT > 32`) はここで bank dispatch。 範囲外は variant 実装で assert
- **`getPort(port_index)`** — 旧 API 仕様踏襲 (port 番号で直接取得)
- **`getPinCount()`** — variant が知る pin 総数 (例: ESP32 = `SOC_GPIO_PIN_COUNT`、 Arduino = `NUM_DIGITAL_PINS`)
- **`getPortCount()`** — 内蔵 Port 数
- **`isValid(local_pin)`** — default は `local_pin < getPinCount()` (`gpio_local_pin_t` は unsigned なので下限チェック不要)。 IGPIO 自身の **ローカル空間内** での判定のみを担う (グローバル空間判定は `GPIOGroup::isValid` の責務)
- **`getPin(local_pin)`** — default は `portForPin(local_pin)->getPin(local_pin)`。 IGPIO はローカル空間 (0〜pin_count-1) を扱う、 グローバル `gpio_number_t` 解決は `GPIOGroup` 経由
- **protected non-virtual dtor** — IPort と同じ

### variant 注入の構図

variant が `<variant_ns>::hal::v1::gpio::getGPIO()` の inline definition を提供し、 `_offer.hpp` の flat-injection 経由で利用者に提供される:

- `m5::hal::v1::gpio::getGPIO()` — MCU 内蔵 GPIO (`const IGPIO*`) を返す (ローカル空間)。 `M5HALCore::ctor` が slot 0 bootstrap source として使用する seam

グローバル resolver および pin shortcut は `M5HALCore::Gpio` singleton に統一する:

- `m5::hal::v1::M5_Hal.Gpio` — グローバル resolver (`GPIOGroup&`、 singleton)。 expander 登録 / 解除はこの instance に対して行う
- `m5::hal::v1::M5_Hal.Gpio.getPin(num)` — `Pin` を返す (旧 `gpio::pin(num)` shortcut の唯一の置き換え)

> **caller 向け正本**: 上記 `M5_Hal.Gpio.*` を **唯一の path** として使う。 旧 `getGPIOGroup()` / `gpio::pin()` free function は廃止された。
>
> **caveat**: `M5_Hal` は eager-init alias、 lazy-safe は `getM5_Hal()` のみ。 namespace-scope initializer や他 lib の global ctor から触る場合は `getM5_Hal()` を経由する。

## GPIOGroup (グローバル resolver)

`GPIOGroup` は最大 `kMaxEntries`(=16) エントリの密配列の単一実装で MCU + expander を統合する。 slot 番号空間は 0〜127 を維持し、 caller は任意の slot 番号を指定できるが、 同時に登録できる IGPIO は `kMaxEntries` 個まで (sparse key / dense storage)。

### gpio_number_t の bit layout

```
int16_t:
  bit 15:    invalid sentinel (1 = invalid、 負値帯はすべて invalid)
  bit 14-8:  slot   (0〜127、 GPIOGroup 内の IGPIO 通し番号)
  bit 7-0:   local pin (0〜255、 その IGPIO 内の通し番号)
```

- **slot 0 は MCU GPIO 予約**
- caller はビット演算で合成・分解しない。 必要時は public helper `types::makeGpioNumber(slot, local_pin)` / `types::extractSlot(num)` / `types::extractLocalPin(num)` を使う

### Interface (virtual なし単一実装)

```cpp
namespace m5::hal::v1::gpio {

class GPIOGroup {
public:
    static constexpr size_t kSlotCount  = 128;  // slot 番号空間の上限 (番号は 0〜127)
    static constexpr size_t kMaxEntries = 16;   // 物理ストレージ容量 (同時登録できる IGPIO 数)

    constexpr GPIOGroup() noexcept = default;
    explicit constexpr GPIOGroup(const IGPIO* mcu_gpio) noexcept;     // mcu_gpio を slot 0 に load

    // 非コピー / 非ムーブ
    GPIOGroup(const GPIOGroup&) = delete;
    GPIOGroup(GPIOGroup&&)      = delete;

    // 登録 / 解除 (checked、 startup 時のみ、 [[nodiscard]] で握り潰し防止)
    [[nodiscard]] m5::stl::expected<void, error::error_t>
    addGPIO(const IGPIO* gpio, types::gpio_slot_t slot);
    [[nodiscard]] m5::stl::expected<void, error::error_t>
    removeGPIO(types::gpio_slot_t slot);

    // 問い合わせ (checked)
    const IGPIO* getGPIO(types::gpio_slot_t slot) const;
    bool         hasGPIO(types::gpio_slot_t slot) const;
    bool         isValid(types::gpio_number_t gpio_num) const;

    // Pin 解決 (checked sugar、 [[nodiscard]] で expected の握り潰し防止)
    [[nodiscard]] m5::stl::expected<Pin, error::error_t>
    tryGetPin(types::gpio_number_t gpio_num) const;

    // Pin 解決 (unchecked fast path、 contract violation 時 assert/UB)
    Pin getPin(types::gpio_number_t gpio_num) const;

private:
    struct Entry {
        const IGPIO*       gpio;
        types::gpio_slot_t slot;
    };
    const Entry* _find(types::gpio_slot_t slot) const;  // slot → Entry を線形探索
    Entry  _entries[kMaxEntries] = {};
    size_t _count                = 0;
};

}
```

### 構造

- **virtual なし単一実装** — variant 非依存の共通ロジック (slot → IGPIO* dispatch) を 1 class に集約する
- **`IGPIO` とは独立** — IGPIO はローカル空間、 GPIOGroup はグローバル resolver を担う
- **最大 `kMaxEntries`(=16) エントリの密配列** — slot 番号空間 0〜127 (`kSlotCount = 128`) を維持しつつ物理 storage は密配列で持つ (sparse key / dense storage)。 128 全 slot を物理確保する疎配列ではなくメモリ削減を優先
- **MCU GPIO は ctor で slot 0 に load**
- **slot の `IGPIO*` は `const IGPIO*`**

### `addGPIO` 規約

- 引数: `const IGPIO* gpio, gpio_slot_t slot`
- **拒絶条件 (全て `INVALID_ARGUMENT`)**:
  1. `gpio == nullptr`
  2. `slot >= 128` (valid 範囲超え)
  3. `gpio->getPinCount() == 0` または `> 256` (gpio_local_pin_t = uint8_t 上限超、 `makeGpioNumber` で表現不可)
  4. 同 slot に既に登録済 (密配列を線形探索して重複検出)
  5. `_count >= kMaxEntries` (容量超過、 重複 slot 検査の後に判定)

### dispatch ロジック

- **`getPin(gpio_num)`**: `extractSlot` + `extractLocalPin` で slot / local を抽出し、 `_find(slot)` で密配列を線形探索して得た IGPIO の `getPin(local)` に委譲する
- **`isValid(gpio_num)`**: 負値即 false、 `_find(slot)` が未登録なら false、 valid なら当該 IGPIO の `isValid(local)` に委譲
- **`tryGetPin(gpio_num)`**: `isValid` + `getPin` の合成。 invalid 入力は `expected<Pin, error_t>` の error path で recover

### checked / unchecked 境界

- **`getPin`** — unchecked fast path。 既知 valid な番号 (board 定数、 caller が isValid 確認済) でのみ呼ぶ。 contract violation は assert/UB
- **`tryGetPin`** — checked sugar。 外部入力 / 起動時構成依存の番号など、 caller が valid か事前確定できない場合に使う

### chain 非サポート

`GPIOGroup` の `addGPIO` に別 `GPIOGroup` をぶら下げる多層構造はサポートしない。

### thread safety / lifetime 規約

- **規約**: `addGPIO` / `removeGPIO` は startup 時のみ、 以降 immutable
- runtime には read-only access のみ走る前提 → **lock を持たない**
- 規約違反 (runtime register / 走査中 register) の動作は未定義

## I2CBusConfig / SPIBusConfig との関係

`*BusConfig` の `pin_*` フィールドは `gpio_number_t` 単一 path (default = -1 invalid)。 variant の `init()` 内で:

1. `num >= 0` チェック (`INVALID_ARGUMENT` を返す)
2. `m5::hal::v1::M5_Hal.Gpio.getPin(num)` で `Pin` 値型を解決
3. 必要なら Pin / Port 経由で `setMode` 等を実施

### expander pin を SCL/SDA に使う場合

`M5_Hal.Gpio` (singleton GPIOGroup) に expander の IGPIO を slot 指定で登録し、 `makeGpioNumber(slot, local)` で global `gpio_number_t` を組み立てる:

```cpp
constexpr m5::hal::v1::types::gpio_slot_t EXPANDER_SLOT = 1;   // slot 0 は MCU 予約
m5::hal::v1::M5_Hal.Gpio.addGPIO(&pca9554_gpio, EXPANDER_SLOT);

m5::hal::v1::i2c::I2CBusConfig bus_cfg{
    m5::hal::v1::types::makeGpioNumber(EXPANDER_SLOT, 0),   // expander local pin 0 = SCL
    m5::hal::v1::types::makeGpioNumber(EXPANDER_SLOT, 1)};  // expander local pin 1 = SDA
i2c_bus.init(bus_cfg);   // software bit-bang variant が M5_Hal.Gpio.getPin で Pin 解決して driving
```

## PinBackup（ピン退避・復元）

I2C / SPI 等のペリフェラルに割り当て済みの MCU ピンへ、 特殊デバイス向けの GPIO 制御を
**一時的に割り込ませる**ためのユーティリティ。 ピンのルーティング状態（GPIO マトリクス +
IO_MUX）を構成するレジスタ群を退避し、 制御後に元のペリフェラル役割へ完全復元する。

- **公開名**: `m5::hal::v1::gpio::PinBackup`（v1 のみ。 `<M5HAL_v1.hpp>`）
- **配置 / 公開**: Espressif platform variant（`variants/platforms/espressif/esp32/hal/gpio/pin_backup.hpp`）。
  PinBackup は HAL kind 実装（`IPort` 等）ではなく **chip capability**（チップ固有ユーティリティ）なので、
  GPIO HAL の勝者選択（offer スキャンで Port / GPIO 提供 variant を1つ選ぶ機構）とは**独立**に、
  platform 層から公開名へ **`using` 宣言で明示注入**する。 platform header は GPIO 勝者に関係なく
  Espressif 検出時は必ず include されるため、 将来 framework variant（arduino / espidf）が GPIO 勝者に
  なっても、 ESP32 ファミリ全機種（ESP32 / S3 / C3 / C6 / H2 / P4）で上記の公開名から到達できる。
  「HAL kind は勝者総取り、 追加型の chip capability は platform からの明示公開」 という棲み分け
- **対象**: MCU ピンのみ（`gpio_number_t` の slot 0）。 invalid（負値）や expander（slot≠0）は no-op
- **退避するレジスタ**（LovyanGFX `gpio::pin_backup_t` と同一）:
  1. `IO_MUX_GPIOn_REG` — pin function / drive strength / pull up-down / input enable
  2. `GPIO_PINn_REG` — open-drain / 割り込み設定等
  3. `GPIO_FUNCn_OUT_SEL_CFG_REG` — 出力信号ルーティング
  4. `GPIO_FUNCn_IN_SEL_CFG_REG` — 入力信号ルーティング（このピンが入力マトリクスに繋がる場合のみ）
  5. `GPIO_ENABLE(1)_REG` の該当ビット — 出力イネーブル

API は **明示退避**（コンストラクタは対象ピンを記録するだけ。 退避は `backup()` を呼ぶ）:

```cpp
#include <M5HAL_v1.hpp>
using m5::hal::v1::gpio::PinBackup;

PinBackup bk{m5::hal::v1::types::gpio_number_t{21}};
bk.backup();   // I2C/SPI に割当て済みのピン状態を退避
// ... ここで pin 21 を手動 GPIO 制御（特殊シーケンス等） ...
bk.restore();  // ペリフェラル役割へ完全復元
```

`setPin()` / `getPin()` で対象ピンを後から差し替え可能。 `backup(pin)` は `setPin` + `backup` の糖衣。

### ScopedPinBackup（RAII スコープガード）

`PinBackup` を内包し、 **コンストラクタで `backup()`、 デストラクタで `restore()`** を行う RAII ラッパ
（`m5::hal::v1::gpio::ScopedPinBackup`）。 早期 return や例外を含めスコープ離脱時に必ず復元される。

- **ムーブ専用**（`std::unique_lock` 相当）。 コピーは二重 restore 防止のため禁止。 ムーブ元は disarm され
  `restore()` はちょうど一度だけ走る
- **`dismiss()`**: 復元を取り消す（新しいピン設定をそのまま残したい場合）。 `armed()` で復元保留中か確認できる
- デフォルト構築は空ガード（何も復元しない）

```cpp
#include <M5HAL_v1.hpp>
using m5::hal::v1::gpio::ScopedPinBackup;

{
    ScopedPinBackup guard{m5::hal::v1::types::gpio_number_t{21}};  // 退避
    // ... pin 21 を手動 GPIO 制御（特殊シーケンス等） ...
}  // スコープ離脱で自動 restore
```

基本機能（明示退避）は `PinBackup`、 スコープ寿命に縛りたい場合は `ScopedPinBackup` を使い分ける。

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| `expected` で範囲外を表現 (`getPin`) | recover が必要な呼び出しは `tryGetPin` を使う |
| `IGPIO` に操作 API (write/read/setMode 等) | 操作は Pin / IPort に集約する |
| bulk path (bitmask 一括 write 等) | 現状スコープ外 |
| `IGPIO` 継承の Registry (旧 GlobalGPIORegistry) | IGPIO とグローバル resolver の責務を分離する |
| chain (Group の Group) | 多層 slot 解決はサポートしない |
| `start_index` ベース疎配置 | 番号空間に gap を作らない |
| slot 番号を直接 index にする 128 物理配列 (旧 `kMaxSlots` 疎配列) | 実登録数は MCU + expander 数個。 密配列 (最大 `kMaxEntries`) + slot key 線形探索でメモリを削減する |
| Pin / encoded_num の public 化 | encoded の意味は variant 実装側に閉じ込める |

## 関連

- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離 (`*BusConfig::pin_*` で gpio_number_t 単一 path)
- [variants.md](variants.md) — variant 機構 (各 variant が GPIO 具象を flat 注入)
- [i2c.md](i2c.md) — `I2CBusConfig` の pin 指定例
- [../reference/directory-layout.md](../reference/directory-layout.md)
