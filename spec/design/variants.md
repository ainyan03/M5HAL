# design/variants — variant 機構

ハードウェア依存・フレームワーク依存のコードを、 コンパイル時に差し替える仕組み。

判断軸の詳細は [../reference/directory-layout.md](../reference/directory-layout.md)。

## variant の分類 (3 軸)

| 軸 | 性質 | 例 |
|---|---|---|
| **platform variant** | MCU や ホスト OS の物理層に直接依存。 ビルドごとに 1 つだけ選ばれる | `espressif/esp32` (ESP32 family 統合) |
| **framework variant** | ソフトウェア抽象層 (Arduino SDK, ESP-IDF, POSIX, ソフト実装、 リモート等) に依存。 複数共存可能 | `arduino`, `espidf`, `posix`, `software`, `stub` |
| **stub variant** | 常時末尾フォールバック。 実装済みの no-op 具象だけを申告する (現行は GPIO) | `frameworks/stub/` |

複数 variant が同じ HAL kind を申告した場合、 **scan 順で最初に申告した variant** が勝者となり、 無印名 (`i2c::Bus` 等) の型 alias を得る。 勝者以外の variant も suffix 付きの実名 (`i2c::Bus_software` 等) で常に参照できる。

## ディレクトリ配置

```
src/m5_hal/variants/
  frameworks/
    _checker.hpp          framework 軸の検出 (M5HAL_FRAMEWORK_HAS_*)
    arduino/
      _offer.hpp            capability 申告
      hal.hpp / hal.inl     per-kind hub (hal/gpio/gpio.hpp + hal/i2c/i2c.hpp + hal/spi/spi.hpp を include)
      hal/
        gpio/gpio.{hpp,inl}   Pin / Port / GPIO + 実装
        i2c/i2c.{hpp,inl}     IBus + 実装 (caller-provided TwoWire 委譲)
        spi/spi.{hpp,inl}     IBus + 実装 (caller-provided SPIClass 委譲)
        uart/uart.{hpp,inl}   IBus + 実装 (caller-provided HardwareSerial 委譲)
    espidf/
      _offer.hpp
      detail/espidf_version.hpp
      hal.hpp / hal.inl
      hal/gpio/gpio.hpp       GPIO 実装 (ESP-IDF GPIO driver)
      hal/i2c/i2c.{hpp,inl}   IBus + 実装 (ESP-IDF I2C master driver、 gen5/gen4 backend)
      hal/i2s/i2s.{hpp,inl}   IBus + 実装 (ESP-IDF gen5 I2S standard driver)
      hal/spi/spi.{hpp,inl}   IBus + 実装 (ESP-IDF SPI master driver)
      hal/uart/uart.{hpp,inl}  IBus + 実装 (ESP-IDF UART driver)
    software/
      _offer.hpp
      hal.hpp / hal.inl
      hal/i2c/i2c.{hpp,inl}   bit-bang I2C 実装
      hal/spi/spi.{hpp,inl}   bit-bang SPI 実装
    posix/
      _offer.hpp
      hal.hpp / hal.inl
      hal/uart/uart.{hpp,inl}  IBus + 実装 (termios serial)
      hal/uart/ports.{hpp,inl}          ポート列挙 (listSerialPorts)
      hal/uart/remote_connect.{hpp,inl} remote 接続確立 (列挙 × hello)
    stub/
      _offer.hpp              実装済み no-op capability だけ申告
      hal.hpp                 no-op 具象 (inline 定義のみ、 hal.inl 不要)
      hal/gpio/gpio.hpp
  platforms/
    _checker.hpp          platform 軸の検出 (M5HAL_V1_TARGET_PLATFORM_VARIANT_ID / _PATH。 番号は ../ids.hpp)
    espressif/
      esp32/                chip-family 単位 (ESP32 / S2 / S3 / C2 / C3 / C5 / C6 / C61 / H2 / P4 を統合)
        _offer.hpp
        hal.hpp / hal.inl   chip family 内で必要なら CONFIG_IDF_TARGET_* で ifdef 分岐、
                            chip 別差分はサブフォルダ (例: hal/gpio/esp32s3/...) に格納
        hal/gpio/gpio.hpp   ESP32 family 統合 GPIO (W1TS/W1TC 直叩き、 per-bank dispatch)
```

namespace は物理階層と厳密 1:1:

- `src/m5_hal/variants/frameworks/arduino/hal/i2c/i2c.hpp` ⇔ `m5::variants::frameworks::arduino::hal::v1::i2c::*`
- `src/m5_hal/variants/platforms/espressif/esp32/hal/gpio/gpio.hpp` ⇔ `m5::variants::platforms::espressif::esp32::hal::v1::gpio::*`

frameworks 配下は vendor 階層なし (`variants::frameworks::<name>`)。 platforms 配下は vendor + chip-family の 2 階層 (`variants::platforms::<vendor>::<chip-family>`)。 chip-family は同一 SDK / 同一レジスタ抽象を共有する chip 群を 1 variant にまとめる単位。 詳細は [../reference/directory-layout.md](../reference/directory-layout.md)。

## variant 間の依存関係

各 variant は **HAL 層の抽象基底のみに依存** する。 他の variant の具象には直接依存しない (依存性逆転原則)。

例: `variants::frameworks::software` のビットバン I2C を実装する際、 `m5::hal::v1::gpio::IPort` / `m5::hal::v1::gpio::Pin` の抽象に対してコードを書く。 具体的にどの variant の Pin 具象が注入されるかは知らない。 BusConfig は `gpio_number_t` 単一 path で受け取り、 init() 内で `m5::hal::v1::M5_Hal.Gpio.getPin(num)` 経由で Pin を解決する。

Framework variant が SDK native object を必要とする場合は、共通 `IBusConfig` / `IBusConfig` / `IBusConfig` を直接肥大化させず、variant 固有 `BusConfig` を追加する。Arduino なら `TwoWire*` / `SPIClass*` / `HardwareSerial*`、ESP-IDF なら `i2c_port` / `spi_host_device_t` / `uart_port` のように、その framework で意味を持つ値だけを variant 側に置く。これにより、Arduino の `Serial` が USB CDC になるような board/config 差異を M5HAL が暗黙解決しない。

この設計により、 ビットバン実装は `stub::Port` のモック注入で native ユニットテスト可能。 任意の expander IGPIO を `M5_Hal.Gpio` に slot 指定で register すれば、 同じ単一 path で driving できる。

## variant 内部の構造

各 variant は **self-contained に `_offer.hpp` + `hal.hpp` (+ `hal.inl`) を持つ**。

- `_offer.hpp` — capability 自己申告 (マクロ宣言のみ)
- `hal.hpp` — per-kind hub (kind 別ファイルを include する薄いファイル)
- `hal.inl` — 実装 hub (.inl 形式の kind 別実装を include、 必要時)
- `hal/<kind>/<kind>.{hpp,inl}` — kind 別の宣言と実装

```cpp
// 例: variants/frameworks/arduino/hal.hpp
#pragma once
#include "hal/gpio/gpio.hpp"
#include "hal/i2c/i2c.hpp"
```

kind 別ファイルは自己完結 (include guard、 namespace スキャフォールド、 `ARDUINO` 等のガード) を持つ。 bus kind と gpio の公開型は `m5::hal::v1::<kind>` 直下に定義するため (§offer 要件)、 kind 共通型 (`types::` / `bus::` / `result_t` 等) は enclosing namespace から修飾なしで解決できる。 variant 固有の内部構造 (software I2C の `detail::` service 群、 esp32 の `lowlevel::` レジスタ層、 posix の serial port ユーティリティ等) は従来どおり variant namespace (`m5::variants::...::hal::v1::<kind>`) に置く。

`stub/` は 1 kind しか持たない場合でも hub `hal.hpp` を置く (`M5HAL_v1.{hpp,cpp}` 側の include 名を variant ごとに統一するため)。

## `_offer.hpp` (capability 自己申告) 仕様

各 variant が「自分が提供できる機能」 を宣言するマクロ宣言ファイル。

- **include guard / `#pragma once` を持たない** — `offer_all.inl` の繰り返し include を許す
- マクロ宣言のみで構成 (型 / template / constexpr は使わない)
- 宣言する内部マクロは全て `M5HAL_VARIANT_CURRENT_*_` 形式 (末尾アンダースコア):

| マクロ | 用途 | 例 |
|---|---|---|
| `M5HAL_VARIANT_CURRENT_ALIAS_` | namespace alias 用の variant 短縮名 | `arduino` |
| `M5HAL_VARIANT_CURRENT_BASE_NS_` | variant ベース namespace path (`m5::` 直下からの相対、 層名 `hal` 等は含めない) | `variants::frameworks::arduino` |
| `M5HAL_VARIANT_CURRENT_HAS_HAL_<KIND>_` | HAL kind 単位の capability flag | `M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_ 1` |

- これらのマクロは `offer_all.inl` 側で消費後に undef される。 `_offer.hpp` 自身では undef しない

### offer 要件 (申告する kind が公開すべき型)

bus kind を申告する variant は、 **`m5::hal::v1::<kind>` 直下**に
**`Bus_<variant>` と `BusConfig_<variant>`** (例: `i2c::Bus_arduino` /
`i2c::BusConfig_arduino`) を必ず定義する。 勝者選択 (offer scan) はこの 2 つに
無印 alias (`using Bus = Bus_<variant>; using BusConfig = BusConfig_<variant>;`)
を張るので、 `m5::hal::<kind>::Bus` / `BusConfig` という綴りが**どの build でも
常に存在**し、 「`BusConfig` を作って `Bus::init` に渡す」イディオムが全 variant
で型整合する。 この命名の利点:

- エラーメッセージ / デバッガ表示が短い (`m5::hal::v1::i2c::Bus_arduino`)
- `i2c::Bus` の正体が型 alias 1 段で追える
- IDE 補完で `Bus_` から backend 一覧が見える。 variant 明示も
  `using MyBus = i2c::Bus_espidf;` と短い
- suffix が異なるので多 variant 共存 (arduino + espidf 同居) も自然

拡張フィールドが不要な variant も **alias ではなく空派生**で定義する
(`struct BusConfig_software : public IBusConfig { using IBusConfig::IBusConfig; };`) —
typed init が variant 固有型を受けることで、 抽象 config や兄弟 variant の config の
流入がコンパイルエラーになる (型安全の一貫性)。 kind の config 基底 `IBusConfig` は
**常に派生される抽象基底**であり、 直接インスタンス化しない
([../style/coding_style.md](../style/coding_style.md) §抽象基底命名)。

gpio kind は `Port_<variant>` / `GPIO_<variant>` と
`getMCUGPIO_<variant>()` / `getGPIO_<variant>()` を同じ規約で公開する
(勝者選択は型 alias + inline wrapper 関数を生成する)。

例外: **runtime kind は bus kind ではない** ([runtime.md](runtime.md))。 公開すべき面は
`millis` / `micros` / `delayMs` / `delayUs` の free function 4 本と `Mutex` クラス 1 個で、
型 alias では運べないため、 この kind のみ variant namespace に実装を置き
`using namespace` 注入を維持する。

`init` は **variant の `BusConfig` を直接受ける非 virtual メンバ**として宣言する
(`init(const BusConfig&)`)。 基底 `bus::IBus` に virtual `init` は置かない — 初期化には variant 固有
情報 (native handle / port / device path) が必須で kind 汎用の `init` は成立せず、 旧 virtual +
ダウンキャスト形は「兄弟 config を渡せてしまい UB になる」誤用経路だけを提供していた。
typed init では誤用 (抽象 config や別 variant の config を拡張フィールド持ち variant へ渡す) が
**コンパイルエラー**になる。 `release()` は引数を取らないため基底 virtual のまま。

### `_offer.hpp` の例 (arduino framework)

```cpp
// src/m5_hal/variants/frameworks/arduino/_offer.hpp
// include guard 無し (re-include 前提)

#define M5HAL_VARIANT_CURRENT_ALIAS_   arduino
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::arduino

#define M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_ 1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_UART_ 1
// 具象が揃わない kind は申告しない (offer されず alias も生成されない)
```

## `offer_all.inl` (勝者 alias 生成) 仕様

各 `_offer.hpp` の直後に再 include され、 以下を実行:

1. `M5HAL_VARIANT_CURRENT_HAS_HAL_<KIND>_` の有無を確認
2. その kind の **first hit のみ**、 勝者 alias を `m5::hal::v1::<kind>` に生成:
   - bus kind: `using Bus = Bus_<ALIAS>; using BusConfig = BusConfig_<ALIAS>;`
     (token paste で suffix を合成)
   - gpio: `using Port = Port_<ALIAS>; using GPIO = GPIO_<ALIAS>;` +
     `getMCUGPIO()` / `getGPIO()` の inline wrapper
   - runtime: `using namespace ::m5::<BASE_NS>::hal::v1::runtime;` (型 alias で
     運べない free function 群のため、 この kind のみ namespace 注入)
3. 全 `M5HAL_VARIANT_CURRENT_*_` マクロを undef

旧機構の variant alias namespace (`m5::hal::<kind>::variant::<ALIAS>` /
`::variant::platform`) は**廃止済み**。 variant 修飾参照は suffix 付き実名
(`i2c::Bus_software`) を使う。 runtime の variant 実装だけは variant namespace の
フル名 (`m5::variants::frameworks::stub::hal::v1::runtime::Mutex` 等) で参照する。

alias 生成 (手順 2) は kind 非依存なので `_macro/offer_kind.inl` に一本化されており、
`offer_all.inl` の各 kind ブロックはパラメータマクロ (`M5HAL_OFFER_KIND_NS_` /
`M5HAL_OFFER_KIND_EMIT_FLAT_` / 形状フラグ `M5HAL_OFFER_KIND_GPIO_` /
`M5HAL_OFFER_KIND_RUNTIME_`) を立てて再 include するだけ。
kind ブロック側に残るのはマクロ名が kind 固有でディレクティブを生成できない部分
(HAS ゲート / selected-variant marker の `#elif` チェーン) のみ。

注意: `BASE_NS_` は runtime kind の注入にのみ使われ、 値に層名 `::hal` を含めない
(`offer_kind.inl` が `::hal` を挟む。 [runtime.md](runtime.md))。

**runtime kind の early scan**: runtime だけは `M5HAL_v1.hpp` 末尾の本 scan より前に
`hal/v1/runtime/runtime.hpp` が選択を済ませる (`bus::IBus` が `runtime::Mutex` を値で内蔵する
ため)。 early scan は `_macro/offer_runtime_only.inl` で runtime 以外の HAS フラグをマスクして
同じ `offer_all.inl` に委譲するので、 ディスパッチブロックや申告の書き方は他 kind と変わらない。
詳細は [runtime.md](runtime.md) §early scan。

`_macro/offer_all.inl` / `_macro/offer_kind.inl` / `_macro/offer_runtime_only.inl` のみマクロ展開の都合で **1 行ネスト形式の namespace 宣言を維持** (`namespace m5 { namespace hal { namespace ... { ... } } }`)。 これは [../style/coding_style.md](../style/coding_style.md) §namespace 宣言形式 の唯一の例外。

### 選択 variant の診断 (selected-variant marker)

**`variants/ids.hpp` が variant 識別番号の唯一のレジストリ**。 platform 検出
(`M5HAL_V1_TARGET_PLATFORM_VARIANT_ID`、 §検出) と kind 選択結果の両方がこのレジストリの値を
使うため、 検出値・選択値・定数のどの 2 つを比較しても意味が成立する。

- 値域 (10 進グループ、 各レンジ内は **append-only**): 0 = `M5HAL_V1_VARIANT_ID_NONE`
  (未検出/未選択)、 **1〜49 = framework** (`..._FRAMEWORK_<NAME>`)、 **50〜99 = platform (host OS)**、
  **100〜249 = platform (MCU)** (いずれも `..._PLATFORM_<NAME>`)、 250〜65535 = 予約。
- **ワイヤ形式は 2 バイト (u16) 固定** (将来プロトコルに載せる場合)。 現行値は読みやすさのため
  250 未満に収める。 予約域は将来の分散割当 (例: out-of-tree variant の名前ハッシュ由来 ID) に
  充てうる。
- **凍結規約**: v0.2 公開済みリリースに含まれた値は永久凍結 (改番禁止)。 廃止は欠番として残す
  (再利用禁止)。 利用者は**定数名で参照する**こと (生数値のハードコードは契約外)。
- 「検出されるが variant 未実装」のエントリ (AVR 等) も同じレジストリに置く — 実装有無は属性で
  あって、 番号空間を分ける理由にしない。

`offer_all.inl` は flat 注入 (first hit) と同時に、 勝者の variant ID を
**`M5HAL_V1_SELECTED_VARIANT_<KIND>`** (KIND = GPIO / I2C / SPI / I2S / UART) に焼き付ける。
どの variant も offer しなかった kind は `M5HAL_V1_VARIANT_ID_NONE` になる
(`M5HAL_v1.hpp` が scan 後に補完)。
各 `_offer.hpp` は自分の ID を `M5HAL_VARIANT_CURRENT_ID_` で申告する。

プリプロセッサ整数定数なので **`#if` と `static_assert` の両方**で使える:

```cpp
// 期待 variant の固定 (ずれたらコンパイルエラー)
static_assert(M5HAL_V1_SELECTED_VARIANT_I2C == M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO,
              "this sketch assumes the arduino I2C variant");

// 選択結果による条件コンパイル (ctor 引数が variant で異なる場合など)
m5hal::i2c::BusConfig cfg;
cfg.pin_scl = SCL;
cfg.pin_sda = SDA;
#if M5HAL_V1_SELECTED_VARIANT_I2C == M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
cfg.wire = &Wire;  // 選択 variant 固有のフィールドだけ条件分岐
#endif
```

型レベルの確認イディオムも marker と独立に常に成立する: 勝者の無印名は型 alias なので、
無印名と suffix 付き実名は**同一エンティティ**を指す。

```cpp
static_assert(std::is_same<m5hal::i2c::Bus, m5hal::i2c::Bus_arduino>::value,
              "the winner alias resolved to an unexpected variant");
```

実装注: `#define` は置換リストを展開しないため、 first-hit 時の `M5HAL_VARIANT_CURRENT_ID_` の
**値**を別マクロへ転送することはプリプロセッサでは不可能 — `offer_all.inl` の各 kind ブロックに
variant ID ごとの `#elif` チェーンを書き下しているのはこの制約による。 チェーン末尾の `#else`
は `#error`: レジストリに居ない ID で offer した variant は静かに次点へ flat 注入を譲らず、
コンパイルエラーで止まる。

### 型付きミラー (`variant_id_t` / `variantIdName`)

`variants/ids.hpp` は `#define` レジストリ (上記、 `#if` で使える正本) に加えて、 同じ並びの
X-macro リスト `M5HAL_V1_VARIANT_ID_LIST_` と、 そこから導出する型付きミラーを提供する:

- `m5::hal::v1::variant_id_t` — `enum class : uint16_t` (u16 = ワイヤ幅)。 値は `#define` と同一
- `m5::hal::v1::variantIdName(id)` — レジストリ名を返す `constexpr` 関数 (`variant_id_t` /
  生 u16 の両オーバーロード。 未登録値は `"UNKNOWN"`)。 marker 値の診断表示向け

プリプロセッサは `#define` をマクロ展開から生成できないため `#define` リストは手書きのまま残るが、
両リストの整合はコンパイル時に検証される: X リストへの追加は対応する `#define` が無いと
コンパイルエラーになり、 値列の昇順 static_assert (レジストリは並び順 = 値順、 append-only) が
重複・順序ずれを検出する。

検出値も同じレジストリなので、 「この platform variant が GPIO を勝ったか」のような
検出×選択の跨ぎ比較が直接書ける:

```cpp
static_assert(M5HAL_V1_SELECTED_VARIANT_GPIO == M5HAL_V1_TARGET_PLATFORM_VARIANT_ID,
              "the detected platform's variant should win GPIO here");
```

## 検出 (`_checker.hpp`)

各軸の `variants/` 配下に `_checker.hpp` を配置 (アンダースコア先頭でメタファイルを明示)。 軸自体が分離されているので検出機構もそれに従う。

検出機構は v0.0.x コードを継承しつつ、 platform 系マクロは `M5HAL_V1_` プレフィックスで世代分離する
(無印の `M5HAL_TARGET_PLATFORM_*` は凍結 v0 ツリーが所有。 v1 は名前だけでなく**番号も継承しない**
— 識別番号は §選択 variant の診断 のレジストリ `variants/ids.hpp` が正本。
詳細は [v0_v1_coexistence.md](v0_v1_coexistence.md) §制約):

| マクロ | 用途 |
|---|---|
| `M5HAL_V1_TARGET_PLATFORM_VARIANT_ID` | 検出された platform の variant ID (`variants/ids.hpp` のレジストリ値。 `M5HAL_V1_VARIANT_ID_NONE` = 不明 = native ビルド) |
| `M5HAL_V1_TARGET_PLATFORM_PATH` | variant ヘッダの動的 include パス (例: `m5_hal/variants/platforms/espressif/esp32`) |
| `M5HAL_FRAMEWORK_HAS_<NAME>` | framework 検出フラグ (例: `M5HAL_FRAMEWORK_HAS_ARDUINO`)。 v0 と同名共有 (定義は token 同一を維持) |

新規検出マクロ (例: `M5HAL_DETECTED_FRAMEWORK_*`) は必要が見えた時点で追加 (先回り追加なし)。

### arduino variant の対応コア (build gate)

arduino variant は **arduino-esp32 コア専用** (`TwoWire::begin(sda, scl)`、 `SPIClass::transferBytes`
等のコア拡張 API に依存)。 非 ESP32 Arduino コア (AVR / SAMD / RP2040 等) を担える variant は
現状ほかにもない (software も `<thread>` に依存) ため、 `frameworks/_checker.hpp` は
`ARDUINO && !ESP_PLATFORM` のとき **`#error` で早期に明示する** (variant ヘッダ深部の不可解な
コンパイルエラーで死なせない)。 非 ESP32 コアを担える variant 構成ができた時点でこのゲートを外す。

## 走査順 (`M5HAL_v1.hpp` 内)

```
1. platform variant      (M5HAL_V1_TARGET_PLATFORM_PATH 経由で動的 include)
2. arduino framework     (M5HAL_FRAMEWORK_HAS_ARDUINO のとき)
3. espidf framework      (M5HAL_FRAMEWORK_HAS_ESPIDF のとき。 Arduino と併存可)
4. posix framework       (M5HAL_FRAMEWORK_HAS_POSIX のとき = 素の POSIX host。 UART のみ申告)
5. software framework    (ビットバン fallback、 常に scan)
6. stub fallback         (常に末尾、 必ず scan)
```

将来 freertos, remote 等が追加されたら適切な位置に挿入する。

「最初に申告した variant」 が flat 注入される。 stub は実装済みの no-op 具象だけを申告し、 上位 variant が先に注入した HAL では fallback として控える。

posix は素の POSIX host (Arduino / ESP-IDF SDK を含まないビルド) で **UART と runtime** を申告し、 host serial を既定の UART provider にする (host の UART スロットは他の host variant が埋めないため)。 UART は **opt-out**: 既定で有効、 `M5HAL_CONFIG_POSIX_UART=0` で抑止する (host で UART を意図的に未提供へ戻したいテスト等)。 この opt-out は **UART kind のみ** に作用し variant 自体は止めない — runtime まで止めると host の Bus が stub フェイク mutex へ静かに退行するため ([runtime.md](runtime.md))。 stub は host 実装を持たない HAL kind の no-op を引き続き担う。

## 追加時チェックリスト

### framework variant を追加する

1. `variants/frameworks/<name>/` を作り、 `_offer.hpp` / `hal.hpp` / 必要なら `hal.inl` を置く
2. `variants/frameworks/_checker.hpp` に `M5HAL_FRAMEWORK_HAS_<NAME>` を追加する。ユーザーが切り替える挙動なら
   `M5HAL_CONFIG_*` として [configuration.md](configuration.md) に登録する
3. `variants/ids.hpp` に `M5HAL_V1_VARIANT_ID_FRAMEWORK_<NAME>` を追加し (framework レンジ 1〜49 の
   末尾に +1、 append-only。 X-macro リスト `M5HAL_V1_VARIANT_ID_LIST_` にも同位置に追加)、
   `_offer.hpp` で `M5HAL_VARIANT_CURRENT_ID_` として申告する。
   `_macro/offer_all.inl` の各 kind の `#elif` チェーンにも 1 行ずつ追加する
   (§選択 variant の診断 の実装注を参照。 漏れはチェーン末尾の `#error` で止まる)
4. `M5HAL_v1.hpp` の scan order に header include + `_offer.hpp` + `offer_all.inl` を追加する。実装が `.inl`
   を持つ場合は `M5HAL_v1.cpp` にも include を追加する
5. 既存 kind を提供する場合は `_offer.hpp` で `M5HAL_VARIANT_CURRENT_HAS_HAL_<KIND>_` を申告し、 kind の
   namespace 直下に `Bus_<variant>` と `BusConfig_<variant>` を公開する (§offer 要件)。未完成の kind は申告しない
6. `test/v1/build_check/build_check.hpp` に suffix 付き variant 型（例:
   `m5::hal::v1::<kind>::Bus_<name>`）の compile fence を足す
7. `spec/reference/directory-layout.md` と本ページの配置表 / 走査順を更新する

### HAL kind を追加する

1. `src/m5_hal/hal/v1/<kind>/<kind>.hpp`（必要なら `.inl`）で共通抽象と config / accessor / desc を定義する
2. `types::BusKind` に kind tag を追加し、 `BusConfig` / `AccessConfig` 派生の ctor でその tag を設定する
3. `M5HAL_v1.hpp` / `M5HAL_v1.cpp` に共通 header / impl を include する
4. `_macro/offer_all.inl` に新 kind のディスパッチブロックを追加する (既存 kind のブロックを複製して
   kind トークンを差し替える: HAS ゲート / `M5HAL_OFFER_KIND_NS_` / platform 束縛ガード /
   `M5HAL_V1_SELECTED_VARIANT_<KIND>` チェーン + `offer_kind.inl` の include)。 `M5HAL_v1.hpp` の
   scan 後 NONE 補完にも 1 ブロック追加する
5. 各 variant の `_offer.hpp` は、具象が揃った variant だけ新 kind を申告する。stub が fallback を担う場合は
   stub の具象も同時に用意する
6. native build_check と、最低1つの API-level unit test を追加する

手順 1〜2 は bus kind の形。 bus 構造を持たない設備 kind (現行は runtime のみ) は手順 1 を
「kind 契約の定義」 に読み替え、 手順 2 を省く ([runtime.md](runtime.md)。 NONE 補完も runtime
は持たない — 選択不成立は early scan の `#error` で止める)

### chip capability を追加する

chip capability は `PinBackup` のように、HAL kind の勝者選択とは独立したチップ固有ユーティリティを指す。
当面は **platform header から公開 namespace へ named `using` 宣言で明示公開**する。`offer_all.inl` の
HAL kind 申告には載せない。

1. platform variant 内の適切な `hal/<area>/` に型を置く
2. platform header が常に include される経路で、`m5::hal::v1::<area>` へ named `using` 宣言を追加する
3. `test/v1/build_check/build_check.hpp` で公開名から到達できることを compile fence に入れる
4. `spec/design/<area>.md` に、HAL kind ではなく chip capability として公開する理由、対象 chip / no-op 条件、
   実機検証状況を書く

chip capability が複数増えて named `using` が煩雑になった時点で、HAL kind とは別の capability 用 offer
機構を検討する。それまでは明示公開の方が読みやすく、勝者選択機構との責務境界も保ちやすい。

## v1 初期スコープ

| カテゴリ | variant | 役割 |
|---|---|---|
| frameworks | `arduino/` | Arduino-ESP32 framework の HAL 具象 + runtime (Arduino core time + FreeRTOS mutex) |
| frameworks | `stub/` | 実装済み kind の no-op / フェイク fallback (現行は GPIO と runtime、 test 用) |
| frameworks | `software/` | software bit-bang による I2C / SPI の framework 中立具象 |
| frameworks | `espidf/` | ESP-IDF framework の HAL 具象 + runtime (esp_timer + FreeRTOS mutex) |
| frameworks | `posix/` | POSIX host の UART 具象 (termios serial。 opt-out = 既定有効、 `M5HAL_CONFIG_POSIX_UART=0` で UART kind のみ抑止) + runtime (CLOCK_MONOTONIC + std::timed_mutex) |
| platforms | `espressif/esp32/` | ESP32 family (`esp32` / `s2` / `s3` / `c2` / `c3` / `c5` / `c6` / `c61` / `h2` / `p4`) のレジスタ直叩き具象 |

将来追加候補:
- frameworks: `freertos/`, `stdcpp/`, `remote/`, `zephyr/`、 posix への TCP/SPI/I2C 追加
- platforms: `host/`、 他 vendor (`stmicroelectronics::stm32` 等)

## 関連

- [../architecture.md](../architecture.md)
- [../style/coding_style.md](../style/coding_style.md) §variant 機構の規則
- [../reference/directory-layout.md](../reference/directory-layout.md) (ディレクトリ ⇔ namespace 1:1 規則 + variants 配下の chip 別分離 + `hal/<kind>` の 3 パターン規則)
