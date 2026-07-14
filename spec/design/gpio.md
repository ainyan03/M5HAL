# design/gpio — GPIO 抽象設計

> **読者**: 実装者・レビュー向け（設計仕様）。

M5HAL v2 の GPIO 抽象は `Pin`、 `IPort`、 `IGPIO`、 `GPIOGroup` で構成する。 caller には `gpio_number_t` 空間と `Pin` 値型を提供し、 encoded 表現は variant 実装側に閉じ込める。

## caller 向け唯一の entry point

```cpp
// Pin 取得 (唯一の path)
m5::hal::v2::gpio::Pin pin = m5::hal::v2::M5_Hal.Gpio.getPin(gpio_num);

// checked variant (外部入力 / 起動時構成依存など valid か不明な場合)
auto result = m5::hal::v2::M5_Hal.Gpio.tryGetPin(gpio_num);
```

- `M5_Hal.Gpio` は `GPIOGroup` singleton。 MCU + expander を統合するグローバル resolver
- **旧 `getGPIOGroup()` / `gpio::pin()` free function は廃止**。 `M5_Hal.Gpio.getPin` を唯一の path として使う
- `M5_Hal` は eager-init alias。 namespace-scope initializer や他 lib の global ctor から使う場合は lazy-safe な `getM5_Hal().Gpio.getPin(num)` を経由する
- `M5_Hal` object 層の位置づけ (ライフサイクル・singleton 初期化) は [architecture.md](../architecture.md) §HAL object 層 を参照。 expander の登録と pin-path への載せ方は本ファイル §expander pin を SCL/SDA に使う場合 を参照

## 設計原則

- **encoded 隠蔽** — Pin / IPort / IGPIO 操作 API は `gpio_local_pin_t` (= IGPIO 内 local pin 空間) で受ける。 internal 表現 (絶対 gpio / bit 位置 / pin_mask / index 等) は variant 実装が支配し、 利用者には見せない。 グローバル `gpio_number_t` の解決は `GPIOGroup` が担う
- **高速 path を阻害しない** — `writeHigh` / `writeLow` の独立 hook を持つ
- **契約ベース (assert + UB)** — 範囲外 `gpio_local_pin_t` は variant 実装の `_fromLocalPin` 内で **assert で debug 即死、 release UB**。 `expected` で穢さない
- **constexpr 化可能** — `IPort` / `IGPIO` は protected non-virtual dtor を持つ
- **expander 統合** — MCU GPIO と I/O expander を単一の `gpio_number_t` 空間で扱う
- **抽象に `I` prefix** — variant 具象との衝突を避ける

## 主要型 (m5::hal::v2::gpio)

| 型 | 役割 | 配置 |
|---|---|---|
| `Pin` | 単一 pin 操作の値型 facade (`IPort*` + encoded_num) | `hal/v2/gpio/port.hpp` |
| `IPort` | bank-level GPIO 操作 (write/read/setMode、 encoded hook を抱える) | `hal/v2/gpio/port.hpp` |
| `IGPIO` | IPort のコンテナ + dispatch + Pin factory | `hal/v2/gpio/gpio.hpp` |
| `GPIOGroup` | MCU + expander 統合 (グローバル resolver、 IGPIO とは独立、 virtual なし単一実装) | `hal/v2/gpio/group.hpp` |

## Pin (値型 facade)

```cpp
namespace m5::hal::v2::gpio {

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
namespace m5::hal::v2::gpio {

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
- **invalid pin_index は assert で debug 即死、 release UB** — `_fromLocalPin` 実装の責務 (`expected` で wrap しない)。 IGPIO 内 local pin 空間の一部だけを担う Port (複数 bank 構成や base 付き Port) では、 **自 Port に所属しない pin_index も invalid** として assert する (他 bank の pin が bit 折り返しで自 bank へ alias する事故の防止)
- **`writeHigh` / `writeLow` 高速 path** — default 実装は `_writePinEncoded(encoded, true/false)` に委譲し、 variant は override できる
- **Pin factory** — `getPin(pin_index)` 経由でのみ Pin 値型を発行。 `_fromLocalPin` で encoded 化し、 `Pin{this, encoded}` の private ctor (`IPort` が friend) を呼ぶ
- **protected non-virtual dtor** — `delete (IPort*)` を許可しない

## IGPIO (IPort のコンテナ + 解決機)

```cpp
namespace m5::hal::v2::gpio {

class IGPIO {
public:
    struct PinLocation {
        uint8_t port_index;
        uint8_t bit_index;
    };

    // dispatch / 解決
    virtual IPort* portForPin(types::gpio_local_pin_t local_pin) const = 0;
    virtual IPort* getPort(uint8_t port_index) const                   = 0;

    // local pin と 32-bit port mask の対応
    virtual PinLocation locatePin(types::gpio_local_pin_t local_pin) const;
    bool tryLocatePin(types::gpio_local_pin_t local_pin, PinLocation* out) const;
    bool localPinForLocation(uint8_t port_index, uint8_t bit_index,
                             types::gpio_local_pin_t* out) const;

    // 容量
    virtual uint16_t getPinCount() const  = 0;
    virtual uint8_t  getPortCount() const = 0;

    // 範囲判定 (default 実装: local_pin < getPinCount())
    virtual bool isValid(types::gpio_local_pin_t local_pin) const;

    // Pin factory (default 実装: portForPin → IPort::getPin)
    virtual Pin getPin(types::gpio_local_pin_t local_pin) const;

    // GPIOGroup の watch poll pass が対象を判定するための申告
    // (default = false = poll 対象。true を返す IGPIO は poll pass が
    // 完全にスキップし、状態は notifyPinStateChanged 経由でのみ届く)
    virtual bool hasPushEvents() const;

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
- **`locatePin(local_pin)`** — local pin を `getPort()` のordinalと、そのportの32-bit mask上のbitへ写像する。defaultは`portForPin()`と`getPort()`のpointer一致からordinalを求め、bitを`local_pin & 31`とする。1個のstateless `IPort`を複数のlogical portで共有するvariantや、別のbit配置を持つexpanderはoverrideする
- **mappingの一意性** — validな各local pinは一意な`(port_index, bit_index)`を持ち、`port_index < getPortCount()`、`bit_index < 32`、`portForPin(local) == getPort(port_index)`を満たす。`tryLocatePin()`がこの境界をcheckedに検証する
- **`localPinForLocation()`** — `(port_index, bit_index)`からlocal pinを逆引きする。追加RAMを持たず、最大256個のlocal pinを走査する。watchの変化bitをglobal pinへ戻す経路で使用する
- **`getPinCount()`** — variant が知る pin 総数 (例: ESP32 = `SOC_GPIO_PIN_COUNT`、 Arduino = `NUM_DIGITAL_PINS`)
- **`getPortCount()`** — `getPort()` で参照できるlogical port数。1個のstateless `IPort`を複数ordinalで共有する場合も、pin mappingに必要なlogical port数を返す
- **`isValid(local_pin)`** — default は `local_pin < getPinCount()` (`gpio_local_pin_t` は unsigned なので下限チェック不要)。 IGPIO 自身の **ローカル空間内** での判定のみを担う (グローバル空間判定は `GPIOGroup::isValid` の責務)
- **`getPin(local_pin)`** — default は `portForPin(local_pin)->getPin(local_pin)`。 IGPIO はローカル空間 (0〜pin_count-1) を扱う、 グローバル `gpio_number_t` 解決は `GPIOGroup` 経由
- **`hasPushEvents()`** — default `false`。 `GPIOGroup` の watch poll pass がこの IGPIO を対象にするかの申告。 `true` を返す IGPIO (remote GPIO 等) は poll pass が完全にスキップし、 状態は `GPIOGroup::notifyPinStateChanged` 経由でのみ届く (単一ソース規約、 §GPIOGroup の watcher API を参照)
- **protected non-virtual dtor** — IPort と同じ

### variant 注入の構図

variant が `m5::hal::v2::gpio` 直下に `getGPIO_<variant>()` の inline definition を提供し ([variants.md](variants.md) §offer 要件)、 勝者選択が無印の wrapper を生成する:

- `m5::hal::v2::gpio::getGPIO()` — MCU 内蔵 GPIO (`const IGPIO*`) を返す (ローカル空間)。 `M5HALCore` ctor が slot 0 bootstrap source として使用する seam

グローバル resolver は `M5_Hal.Gpio` singleton (`GPIOGroup&`) に統一する。 expander 登録 / 解除もこの instance に対して行う。 caller 向け entry point は本ファイル冒頭の §caller 向け唯一の entry point を参照。

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
namespace m5::hal::v2::gpio {

class GPIOGroup {
public:
    static constexpr size_t kSlotCount     = 128;  // slot 番号空間の上限 (番号は 0〜127)
    static constexpr size_t kMaxEntries    = 16;   // 物理ストレージ容量 (同時登録できる IGPIO 数)

    static constexpr uint32_t kDefaultWatchIntervalUs = 1000;

    enum class Edge : uint8_t {
        Rising,
        Falling,
    };

    using WatchSink = void (*)(void* ctx, types::gpio_number_t pin, bool level, Edge edge);

    GPIOGroup() noexcept;
    explicit GPIOGroup(const IGPIO* mcu_gpio) noexcept;     // mcu_gpio を slot 0 に load

    // 非コピー / 非ムーブ
    GPIOGroup(const GPIOGroup&) = delete;
    GPIOGroup(GPIOGroup&&)      = delete;

    // 登録 / 解除 (checked、 startup 時のみ、 [[nodiscard]] で握り潰し防止)
    [[nodiscard]] result_t<void>
    addGPIO(const IGPIO* gpio, types::gpio_slot_t slot);
    [[nodiscard]] result_t<void>
    removeGPIO(types::gpio_slot_t slot);

    void bindServiceRunner(service::ServiceRunner* runner);

    [[nodiscard]] result_t<void>
    setWatchSink(WatchSink sink, void* ctx, uint32_t poll_interval_us = kDefaultWatchIntervalUs);

    [[nodiscard]] result_t<void> watch(types::gpio_number_t gpio_num);
    result_t<void> unwatch(types::gpio_number_t gpio_num);
    void clearWatchers();

    [[nodiscard]] result_t<void>
    notifyPinStateChanged(types::gpio_number_t gpio_num, bool level);

    // 問い合わせ (checked)
    const IGPIO* getGPIO(types::gpio_slot_t slot) const;
    bool         hasGPIO(types::gpio_slot_t slot) const;

    result_t<void> setDenyMask(types::gpio_slot_t slot, uint8_t port_index, uint32_t mask);

    struct PortAccess {
        IPort* port;
        uint32_t deny_mask;
    };

    result_t<PortAccess> getPort(types::gpio_slot_t slot, uint8_t port_index) const;

    bool         isValid(types::gpio_number_t gpio_num) const;

    // Pin 解決 (checked sugar、 [[nodiscard]] で expected の握り潰し防止)
    [[nodiscard]] result_t<Pin>
    tryGetPin(types::gpio_number_t gpio_num) const;

    // Pin 解決 (unchecked fast path、 contract violation 時 assert/UB)
    Pin getPin(types::gpio_number_t gpio_num) const;

private:
    static constexpr size_t kMaxPortsPerEntry = 2;

    struct Entry {
        const IGPIO*       gpio;
        types::gpio_slot_t slot;
        uint32_t           deny_mask[kMaxPortsPerEntry];
    };
    const Entry* _find(types::gpio_slot_t slot) const;  // slot → Entry を線形探索
    Entry* _findMut(types::gpio_slot_t slot);
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
- **watcher API** — `bindServiceRunner` で service runner に接続し、`setWatchSink` / `watch` / `unwatch` / `clearWatchers` / `notifyPinStateChanged` で GPIO 変化通知を扱う (§watcher API 参照)。remote GPIO push event はこの watch 基盤を使う
- **deny mask** — `setDenyMask(slot, port_index, mask)` がGPIOGroupの対応範囲であるlogical port 0/1の禁止bitを登録する。存在しないportまたはport 2以降は`INVALID_ARGUMENT`。`isValid` / `tryGetPin`、watch、port一括操作は`IGPIO::locatePin()`と同じmappingでdeny bitを公開不可 pinとして扱う
- **PortAccess** — `getPort(slot, port_index)` は `IPort*` と `deny_mask` をまとめて返し、remote の `GpioPortRead` / `GpioPortWrite` がポート一括操作時に deny mask を適用できるようにする

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
- **`isValid(gpio_num)`**: 負値即 false、 `_find(slot)` が未登録なら false、`tryLocatePin()`でmappingが不正ならfalse、deny mask対象ならfalse
- **`tryGetPin(gpio_num)`**: `isValid` + `getPin` の合成。 invalid 入力は `expected<Pin, error_t>` の error path で recover

### checked / unchecked 境界

- **`getPin`** — unchecked fast path。 既知 valid な番号 (board 定数、 caller が isValid 確認済) でのみ呼ぶ。 contract violation は assert/UB
- **`tryGetPin`** — checked sugar。 外部入力 / 起動時構成依存の番号など、 caller が valid か事前確定できない場合に使う

### chain 非サポート

`GPIOGroup` の `addGPIO` に別 `GPIOGroup` をぶら下げる多層構造はサポートしない。

### watcher API (port mask 方式)

`watch` / `unwatch` はピン単位の atomic ポートマスクへの RMW で実装され、ロックフリー・任意
スレッドから並行呼び出し可能。コールバックはグループ**単一** sink (`setWatchSink` で登録・
差し替え・解除) で受け、per-pin コールバック・per-pin edge filter・イベントキューは持たない
(旧 `watch_id_t` / `WatchConfig` / per-pin edge filter / event queue は削除済み)。

- **単一ソース規約**: 各 (slot, port) の状態源は poll (ServiceRunner 経由のポーリング) か push
  (`notifyPinStateChanged`) の**どちらか一方**。`IGPIO::hasPushEvents()` が push 側を申告し、
  poll pass は該当 entry を完全にスキップする。これにより shadow (直前値のスナップショット) の
  書き手がポート単位で一系統になり、edge 検出 (XOR による変化ビット抽出) がレースフリーになる。
  `notifyPinStateChanged` は `hasPushEvents() == true` の IGPIO 専用 (poll 対象 pin へ呼ぶと
  単一ソース規約が崩れ、動作は未定義)
- **sink はグループ単一**: 登録・差し替え・解除は `setWatchSink` に一本化。並行する
  `setWatchSink` 同士は未サポート (結果未定義)。`watch` / `unwatch` は任意スレッドから
  `setWatchSink` と並行して呼べる。sink 解除 (`setWatchSink(nullptr, ...)` / `clearWatchers`)
  が戻った時点で以後 sink は呼ばれない。ただし sink コールバック自身から自己解除した場合は
  自分の in-flight 完了を待たずに戻る (再入可能。コールバックは短時間・非ブロッキングで、
  `notifyPinStateChanged` / `runOnce` / bus・remote トランザクション開始を呼んではならない)
- **`setWatchSink` の失敗セマンティクス**: `OUT_OF_RESOURCE` (service runner のテーブル満杯)
  で失敗した場合の状態は決定的 — sink 未登録・poll service 未登録。差し替えの失敗でも
  旧 sink は復元されない (旧 service は解除済みで、再登録が同様に失敗しうるため復元は
  保証できない)。呼び出し側はリトライ可
- **`unwatch` は in-flight 完了を待たない**: 呼び出しが戻った直後にも、当該ピンの飛行中
  イベントが 1 回届きうる (待つのは sink 解除のみ)
- **監視可能 pin はlogical port 0/1**: `locatePin()`がそれ以外のportへ写像するpin、またはdeny mask対象pin
  の `watch()` は `INVALID_ARGUMENT`
- **poll 周期はグループ単一** (`setWatchSink` の `poll_interval_us`、 0 は `kDefaultWatchIntervalUs`
  に丸める)。per-pin 周期・debounce は提供しない (必要なら sink 側 / 上層で実装する)
- **`watch()` の検出保証**: 呼び出しが戻った時点以降の遷移を検出する。呼び出し中に跨いだ
  遷移は初回イベントに畳まれうる

### thread safety / lifetime 規約

- **規約**: `addGPIO` / `removeGPIO` は startup 時のみ、 以降 immutable
- runtime には read-only access と watcher dispatch が走る。`addGPIO` / `removeGPIO` / `setDenyMask` は startup 時に確定させる
- 規約違反 (runtime register / 走査中 register / 走査中 mask 変更) の動作は未定義
- **Hal 管理 remote slot の例外**: `Hal::connect` / `initUart` / `initTcp` は watcher と connection
  service を外した切替区間で remote `IGPIO` を内部的に remove/add する。caller が runtime に
  `GPIOGroup::removeGPIO` してよいという意味ではない。切替前に取得済みの `Pin` / `PortAccess` は
  raw `IPort*` を含むため、旧 remote port storage はその `Hal` の破棄まで保持される。旧 handle は
  close 済みなので新 peer へ付け替わらず、read は最終 cache、write / mode 変更は no-op となる
  ([remote.md](remote.md) §Hal facade)
- watcher API 自体の並行性契約は上の §watcher API (port mask 方式) を参照
- watch サービスは `ServiceRunner` に登録して駆動される。sink コールバックの実行コンテキスト・
  再入可否・ロック外呼び出しの一般契約は [service.md](service.md) の R6 (コールバック契約) /
  R7 (ロック階層) を参照 (上記の watcher 固有規約はその具体化)

## IBusConfig との関係

`*BusConfig` の `pin_*` フィールドは `gpio_number_t` 単一 path (default = -1 invalid)。 variant の `init()` 内で:

1. `num >= 0` チェック (`INVALID_ARGUMENT` を返す)
2. `m5::hal::v2::M5_Hal.Gpio.getPin(num)` で `Pin` 値型を解決
3. 必要なら Pin / Port 経由で `setMode` 等を実施

config が `Pin` handle ではなく global `gpio_number_t` を保持することで、MCU pin と登録済み expander pin を
同じ宣言的な値で表し、backend は `init()` 時に解決した `Pin` を hot path 用に保持できる。番号と `Pin` の
dual path は優先順位と初期化分岐を生むため提供しない。

### expander pin を SCL/SDA に使う場合

expander 登録の全体像: I/O expander (PCA9554 等) は `IGPIO` を実装し、 `M5_Hal.Gpio.addGPIO(&expander_gpio, slot)` で MCU 以外の slot (slot 0 は MCU 予約) に登録する。 登録後、 その expander の pin は `makeGpioNumber(slot, local)` で組んだ global `gpio_number_t` で MCU pin と同じ pin-path に載り、 `*BusConfig::pin_*` にそのまま渡せる。 以下は I2C の SCL/SDA に使う差分:

```cpp
constexpr m5::hal::v2::types::gpio_slot_t EXPANDER_SLOT = 1;   // slot 0 は MCU 予約
m5::hal::v2::M5_Hal.Gpio.addGPIO(&pca9554_gpio, EXPANDER_SLOT);

m5::hal::v2::i2c::IBusConfig bus_cfg{
    m5::hal::v2::types::makeGpioNumber(EXPANDER_SLOT, 0),   // expander local pin 0 = SCL
    m5::hal::v2::types::makeGpioNumber(EXPANDER_SLOT, 1)};  // expander local pin 1 = SDA
i2c_bus.init(bus_cfg);   // software bit-bang variant が M5_Hal.Gpio.getPin で Pin 解決して driving
```

## PinBackup（ピン退避・復元）

I2C / SPI 等のペリフェラルに割り当て済みの MCU ピンへ、 特殊デバイス向けの GPIO 制御を **一時的に割り込ませる**ためのユーティリティ。 ピンのルーティング状態（GPIO マトリクス + IO_MUX を構成するレジスタ群）を退避し、 制御後に元のペリフェラル役割へ完全復元する。

- **公開名**: `m5::hal::v2::gpio::PinBackup`（v2 のみ。 `<M5HAL_v2.hpp>`）
- **配置**: Espressif platform variant（`variants/platforms/espressif/esp32/hal/gpio/pin_backup.hpp`）から `using` 宣言で公開名へ注入する
- **なぜ platform 注入か**: PinBackup は HAL kind 実装（`IPort` 等）ではなく chip capability（チップ固有ユーティリティ）であるため、 GPIO 勝者選択（offer スキャン）とは独立に platform 層から明示注入する。 これにより、 将来 framework variant が GPIO 勝者になっても ESP32 ファミリ全機種で同じ公開名から到達できる（「HAL kind は勝者総取り、 追加型の chip capability は platform から明示公開」という棲み分け）
- **対象**: MCU ピンのみ（slot 0）。 invalid（負値）や expander（slot≠0）は no-op
- **退避レジスタ**（LovyanGFX `gpio::pin_backup_t` と同一）: `IO_MUX_GPIOn_REG` / `GPIO_PINn_REG` / `GPIO_FUNCn_OUT_SEL_CFG_REG` / `GPIO_FUNCn_IN_SEL_CFG_REG` / `GPIO_ENABLE(1)_REG` の該当ビット
- **`restore()` 安全性**: `backup()` が実際に退避できた場合のみレジスタを書き戻す。 `captured()` で退避状態の有無を確認できる

**明示退避 API** （コンストラクタはピンを記録するだけ。 退避は `backup()` で行う）:

```cpp
#include <M5HAL_v2.hpp>
using m5::hal::v2::gpio::PinBackup;

PinBackup bk{m5::hal::v2::types::gpio_number_t{21}};
bk.backup();   // I2C/SPI に割当て済みのピン状態を退避
// ... pin 21 を手動 GPIO 制御（特殊シーケンス等） ...
bk.restore();  // ペリフェラル役割へ完全復元
```

補助 API: `setPin()` / `getPin()` で対象ピンを後から差し替え可能。 `backup(pin)` は `setPin` + `backup` の糖衣。

### ScopedPinBackup（RAII スコープガード）

`PinBackup` を内包し、 コンストラクタで `backup()`、 デストラクタで `restore()` を行う RAII ラッパ（`m5::hal::v2::gpio::ScopedPinBackup`）。 早期 return を含めスコープ離脱時に必ず復元される。

- **ムーブ専用**（`std::unique_lock` 相当）。 コピーは二重 restore 防止のため禁止。 ムーブ元は disarm され `restore()` はちょうど一度だけ走る
- **`dismiss()`**: 復元を取り消す（新しいピン設定をそのまま残したい場合）。 invalid / expander pin のように退避が no-op だった場合は最初から disarm される

```cpp
#include <M5HAL_v2.hpp>
using m5::hal::v2::gpio::ScopedPinBackup;

{
    ScopedPinBackup guard{m5::hal::v2::types::gpio_number_t{21}};  // ctor で退避
    // ... pin 21 を手動 GPIO 制御（特殊シーケンス等） ...
}  // スコープ離脱で自動 restore
```

明示退避が必要なら `PinBackup`、 スコープ寿命に縛りたいなら `ScopedPinBackup` を使い分ける。

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| `expected` で範囲外を表現 (`getPin`) | recover が必要な呼び出しは `tryGetPin` を使う |
| `IGPIO` に操作 API (write/read/setMode 等) | 操作は Pin / IPort に集約する |
| `IGPIO` 継承の Registry (旧 GlobalGPIORegistry) | IGPIO とグローバル resolver の責務を分離する |
| chain (Group の Group) | 多層 slot 解決はサポートしない |
| `start_index` ベース疎配置 | 番号空間に gap を作らない |
| slot 番号を直接 index にする 128 物理配列 (旧 `kMaxSlots` 疎配列) | 実登録数は MCU + expander 数個。 密配列 (最大 `kMaxEntries`) + slot key 線形探索でメモリを削減する |
| Pin / encoded_num の public 化 | encoded の意味は variant 実装側に閉じ込める |

## 関連

- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離 (`*BusConfig::pin_*` で gpio_number_t 単一 path)
- [variants.md](variants.md) — variant 機構 (各 variant が GPIO 具象を flat 注入)
- [i2c.md](i2c.md) — `IBusConfig` の pin 指定例
- [../reference/directory-layout.md](../reference/directory-layout.md)
