# design/variants — variant 機構

> **読者**: 実装者・レビュー向け（設計仕様）。

ハードウェア依存・フレームワーク依存のコードを、 コンパイル時に差し替える仕組み。

配置とinclude先は[../reference/directory-layout.md](../reference/directory-layout.md)、追加手順は
[../porting_guide/framework.md](../porting_guide/framework.md)を参照。本書は選択・申告・識別・依存の
規範契約を扱う。

## variant の分類 (3 軸)

| 軸 | 性質 | 例 |
|---|---|---|
| **platform variant** | MCU や ホスト OS の物理層に直接依存。 ビルドごとに 1 つだけ選ばれる | `espressif/esp32` (ESP32 family 統合) |
| **framework variant** | ソフトウェア抽象層 (Arduino SDK, ESP-IDF, POSIX, ソフト実装、 リモート等) に依存。 複数共存可能 | `arduino`, `espidf`, `posix`, `software`, `stub` |
| **stub variant** | 常時末尾フォールバック。 実装済みの no-op 具象だけを申告する (現行は GPIO) | `frameworks/stub/` |

kind別overrideがない場合、複数 variant が同じ HAL kind を申告すると **scan 順で最初に申告した variant**
が勝者となり、無印名 (`i2c::BusConfig` 等) の型 alias を得る。`M5HAL_CONFIG_VARIANT_<KIND>`へ
stable variant IDを指定した場合は、一致するvariantの最初のofferだけが勝者になる。勝者以外のvariantも
suffix付きの実名 (`i2c::Bus_software` 等) で常に参照できる。

scan 順と勝者 alias の詳細は §走査順・§`_offer.hpp`・§`offer_all.inl` で扱う。

ordered first-hitは、既存buildの無印aliasをprovider追加だけで変えず、単純なcompile-time選択を維持するための
互換既定である。kind別stable ID overrideにより明示選択と誤指定のcompile errorは得られるため、offer registryと
selection policyの全面分離は、複数の同格providerを動的な規則で選ぶ実需が生じるまで導入しない。

## 走査順 (`M5HAL_v2.hpp` 内)

```
1. platform variant      (M5HAL_V2_DETECTED_PLATFORM_VARIANT_PATH 経由で動的 include)
2. freertos framework    (M5HAL_FRAMEWORK_HAS_FREERTOS のとき。 OS 基本プリミティブ: Mutex, Task)
3. arduino framework     (M5HAL_FRAMEWORK_HAS_ARDUINO のとき)
4. espidf framework      (M5HAL_FRAMEWORK_HAS_ESPIDF のとき。 Arduino と併存可)
5. posix framework       (M5HAL_FRAMEWORK_HAS_POSIX のとき = 素の POSIX host。 UART のみ申告)
6. remote framework      (M5HAL_CONFIG_REMOTE_VARIANT=1 のとき。 I2C/SPI/UART/I2S/PDM の proxy backend を申告。 opt-in: 既定 off)
7. software framework    (ビットバン fallback、 常に scan)
8. stub fallback         (常に末尾、 必ず scan)
```

freertos は arduino / espidf より前に scan することで RUNTIME_MUTEX / RUNTIME_TASK / RUNTIME_EVENT を勝ち取る。
arduino / espidf は RUNTIME (time functions) を勝つ。 同一環境で freertos + arduino (or espidf) が
同時に有効化し、 sub-kind ごとに別 variant が勝者になる。

新しい framework variant を追加する場合も、kind ごとのwinnerが意図したproviderになる位置へ
挿入し、既存の優先順位を偶発的に変えない。

「最初に申告した variant」 が flat 注入される。 stub は実装済みの no-op 具象だけを申告し、 上位 variant が先に注入した HAL では fallback として控える。

posix は素の POSIX host (Arduino / ESP-IDF SDK を含まないビルド) で **UART と runtime** を申告し、 host serial を既定の UART provider にする (host の UART スロットは他の host variant が埋めないため)。 UART は **opt-out**: 既定で有効、 `M5HAL_CONFIG_POSIX_UART=0` で抑止する (host で UART を意図的に未提供へ戻したいテスト等)。 この opt-out は **UART kind のみ** に作用し variant 自体は止めない — runtime まで止めると host の Bus が stub フェイク mutex へ静かに退行するため ([runtime.md](runtime.md))。 stub は host 実装を持たない HAL kind の no-op を引き続き担う。

## 配置境界

framework、platform、メタファイル、kind実装の物理treeとnamespace対応は
[../reference/directory-layout.md](../reference/directory-layout.md)を正本とする。本書で規定するのは、
各variantがself-containedな申告単位であることと、公開provider symbolとvariant内部構造の境界である。

## variant 間の依存関係

各 variant は **HAL 層の抽象基底のみに依存** する。 他の variant の具象には直接依存しない (依存性逆転原則)。

例: `variants::frameworks::software` のビットバンI2Cを実装する際、`gpio::IPort` / `gpio::Pin`の抽象に
対してコードを書く。factoryは`LocalResourceContext`を受け、具体的なPin具象を知らずに注入された
`GPIOGroup` / `ServiceRunner` / `Allocator`へ束縛する。BusConfigは`gpio_number_t`単一pathで受け取り、
init時にraw configを検証してからcontextの`gpio->tryGetPin(num)`でPinを解決する。

Framework variant が SDK native object を必要とする場合も、公開 `BusConfig` はportableなまま維持する。native objectやprovider固有optionはconfig fieldへ混ぜず、対応providerが同名の`acquire(config, native::borrowed(resource))` / `acquire(config, native::managed(args...))` overloadで受ける。Arduinoなら`TwoWire` / `SPIClass` / `HardwareSerial`、ESP-IDFならcontroller handle等をpolicy側のprovider-private型として扱う。これにより、Arduinoの`Serial`がUSB CDCになるようなboard/config差異をM5HALが暗黙解決せず、通常callerへvariant固有型も露出しない。

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

kind 別ファイルは自己完結 (include guard、 namespace スキャフォールド、 `ARDUINO` 等のガード) を持つ。 bus kind と gpio の公開型は `m5::hal::v2::<kind>` 直下に定義するため (§offer 要件)、 kind 共通型 (`types::` / `bus::` / `result_t` 等) は enclosing namespace から修飾なしで解決できる。 variant 固有の内部構造 (software I2C の `detail::` service 群、 esp32 の `lowlevel::` レジスタ層、 posix の serial port ユーティリティ等) は従来どおり variant namespace (`m5::variants::...::hal::v2::<kind>`) に置く。

`stub/` は 1 kind しか持たない場合でも hub `hal.hpp` を置く (`M5HAL_v2.{hpp,cpp}` 側の include 名を variant ごとに統一するため)。

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
| `M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_MUTEX_` | runtime sub-kind: Mutex | `1` |
| `M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_TASK_` | runtime sub-kind: Task | `1` |
| `M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_EVENT_` | runtime sub-kind: Event | `1` |

- これらのマクロは `offer_all.inl` 側で消費後に undef される。 `_offer.hpp` 自身では undef しない

### offer 要件 (facade bus kind)

facade bus kind (I2C / SPI / UART / I2S / PDM) を申告するvariantは、
**`m5::hal::v2::<kind>`直下**に次を定義する。

- provider実体`Bus_<variant>`
- portable `IBusConfig`からprovider backendを作る`makePortableBackend_<variant>`
- native ownership policyを提供する場合だけ`NativeProvider_<variant><Policy>`の対応specialization

公開`BusConfig`はkind headerが定義するportable configであり、variant suffixを持たない。
勝者選択はconfig型ではなく、portable factoryと`NativeProvider`をbindする。従って通常利用者は
どのproviderでも`BusConfig cfg; Hal.<kind>.acquire(cfg)`と書き、provider名を覚えない。
既存native resourceを使う場合も、対応providerに限り同じ`acquire`へ
`native::borrowed(...)` / `native::managed(...)`を追加する。

`Bus_<variant>`の直接利用はprovider固有初期化が必要なadvanced escape hatchである。provider固有の
private configやnative型を持つことはできるが、公開acquireのselectorとして
`BusConfig_<variant>`を追加してはならない。

gpio kind は `Port_<variant>` / `GPIO_<variant>` と
`getMCUGPIO_<variant>()` / `getGPIO_<variant>()` を同じ規約で公開する
(勝者選択は型 alias + inline wrapper 関数を生成する)。

例外: **runtime kind は bus kind ではない** ([runtime.md](runtime.md))。 runtime は 3 つの
独立 sub-kind に分割されている:

| sub-kind | 公開面 | 注入方式 | 典型勝者 (embedded) |
|---|---|---|---|
| **RUNTIME** (time) | `millis` / `micros` / `delayMs` / `delayUs` free function 4 本 | `using namespace` 注入 | arduino / espidf |
| **RUNTIME_MUTEX** | `Mutex` クラス | `using Mutex = Mutex_<variant>;` 型 alias | freertos |
| **RUNTIME_TASK** | `Task` クラス | `using Task = Task_<variant>;` 型 alias | freertos |
| **RUNTIME_EVENT** | `Event` クラス | `using Event = Event_<variant>;` 型 alias | freertos |

time の free function は型 alias では運べないため `using namespace` 注入を維持する。
Mutex / Task / Event は型 alias で注入し、 bus kind と同じ first-hit 規約に従う。
sub-kind ごとに独立した勝者が選ばれるため、 同一ビルドで freertos が Mutex/Task を、
arduino/espidf が time functions を供給する構成が自然に成立する。

**facade kind (I2C / SPI / UART / I2S / PDM) の無印 `Bus` は runtime facade クラス** ([i2c.md](i2c.md)
§Bus 他)。offer scanは`Bus` / `BusConfig` aliasを張らず、勝者のportable factoryを
`makeSelectedPortableBackend`へ、native policy templateを`NativeProvider`へbindする。
facadeの`init(const IBusConfig&)`はportable factoryを通じて勝者`Bus_<variant>`を選ぶ。
マクロ機構では各 facade kind 分岐が `M5HAL_OFFER_KIND_FACADE_` を立て、 `offer_kind.inl` がその kind の
`using Bus` を抑止する。 **全 bus kind (I2C/SPI/UART/I2S/PDM) が facade 化済**。
I2C / SPI は intent 駆動 HW 割当を持つ managed policy、UART / I2S / PDM は同じ public surface を持つ
static-backend policy。

facadeの`init`はportable `IBusConfig`を直接受ける非virtualメンバである。基底`bus::IBus`に
virtual `init`やpublic `close`は置かない。local providerのdirect concrete/facade wrapperだけがpublic `close()`を持ち、
成功後は同じobjectを`init()`で再利用できる。registry-bound busは
`Hal.<kind>.close(shared_ptr&)`だけでconsuming final closeする。provider終了hookはprotected
`closeBackend()`で、`NoMutation`失敗はrollback、`PartialOrUnknown`失敗はquarantineへ分類する。
remoteの`Bus_remote`は`RemoteBackend`が生成するconnection-bound proxyおよびprotocol test seamであり、
local direct provider escape hatchではない。利用者のremote取得・終了は常にremote `Hal`の
`acquire` / `close`を使い、proxyのconstructor / `init`をdirect lifecycle APIとして扱わない。

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

1. `M5HAL_VARIANT_CURRENT_HAS_HAL_<KIND>_` の有無と、kind別selectorが`NONE`または
   `M5HAL_VARIANT_CURRENT_ID_`に一致することを確認
2. 条件を満たすその kind の **first hit のみ**、 勝者bindingを `m5::hal::v2::<kind>` に生成:
   - **facade kind (i2c / spi / i2s / uart / pdm)**: `makeSelectedPortableBackend`と
     `NativeProvider<Policy>`を勝者providerへbindする。`Bus` / `BusConfig` aliasは張らない
   - gpio: `using Port = Port_<ALIAS>; using GPIO = GPIO_<ALIAS>;` +
     `getMCUGPIO()` / `getGPIO()` の inline wrapper
   - runtime (time): `using namespace ::m5::<BASE_NS>::hal::v2::runtime;` (型 alias で
     運べない free function 群のため、 この sub-kind のみ namespace 注入)
   - runtime_mutex: `using Mutex = ::m5::<BASE_NS>::hal::v2::runtime::Mutex;` (型 alias)
   - runtime_task: `using Task = ::m5::<BASE_NS>::hal::v2::runtime::Task;` (型 alias)
3. 全 `M5HAL_VARIANT_CURRENT_*_` マクロを undef

旧機構の variant alias namespace (`m5::hal::<kind>::variant::<ALIAS>` /
`::variant::platform`) は**廃止済み**。 variant 修飾参照は suffix 付き実名
(`i2c::Bus_software`) を使う。 runtime の variant 実装だけは variant namespace の
フル名 (`m5::variants::frameworks::stub::hal::v2::runtime::Mutex` 等) で参照する。

alias 生成 (手順 2) は kind 非依存なので `_macro/offer_kind.inl` に一本化されており、
`offer_all.inl` の各 kind ブロックはパラメータマクロ (`M5HAL_OFFER_KIND_NS_` /
`M5HAL_OFFER_KIND_EMIT_FLAT_` / 形状フラグ `M5HAL_OFFER_KIND_GPIO_` /
`M5HAL_OFFER_KIND_RUNTIME_` / `M5HAL_OFFER_KIND_FACADE_`) を立てて再 include するだけ。
kind ブロック側に残るのはマクロ名が kind 固有でディレクティブを生成できない部分
(HAS ゲート / selected-variant marker の `#elif` チェーン) のみ。

注意: `BASE_NS_` は runtime kind の注入にのみ使われ、 値に層名 `::hal` を含めない
(`offer_kind.inl` が `::hal` を挟む。 [runtime.md](runtime.md))。

**runtime kind の early scan**: runtime (time / mutex / task / event の 4 sub-kind) だけは
`M5HAL_v2.hpp` 末尾の本 scan より前に `hal/v2/runtime/runtime.hpp` が選択を済ませる
(`bus::IBus` が `runtime::Mutex` を値で内蔵するため)。 early scan は
`_macro/offer_runtime_only.inl` で runtime 以外の HAS フラグをマスクして同じ `offer_all.inl`
に委譲するので、 ディスパッチブロックや申告の書き方は他 kind と変わらない。 early scan の
走査順は本 scan と同じ: freertos → arduino → espidf → posix → stub (freertos が先に
RUNTIME_MUTEX / RUNTIME_TASK / RUNTIME_EVENT を勝ち取る)。
詳細は [runtime.md](runtime.md) §early scan。

`_macro/offer_all.inl` / `_macro/offer_kind.inl` / `_macro/offer_runtime_only.inl` のみマクロ展開の都合で **1 行ネスト形式の namespace 宣言を維持** (`namespace m5 { namespace hal { namespace ... { ... } } }`)。 これは [../style/coding_style.md](../style/coding_style.md) §namespace 宣言形式 の唯一の例外。

### 選択 variant の診断 (selected-variant marker)

**`variants/ids.hpp` が variant 識別番号の唯一のレジストリ**。 platform 検出
(`M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID`、 §検出) と kind 選択結果の両方がこのレジストリの値を
使うため、 検出値・選択値・定数のどの 2 つを比較しても意味が成立する。

- 値域 (10 進グループ、 各レンジ内は **append-only**): 0 = `M5HAL_V2_VARIANT_ID_NONE`
  (未検出/未選択)、 **1〜49 = framework** (`..._FRAMEWORK_<NAME>`)、 **50〜99 = platform (host OS)**、
  **100〜249 = platform (MCU)** (いずれも `..._PLATFORM_<NAME>`)、 250〜65535 = 予約。
- **ワイヤ形式は 2 バイト (u16) 固定**。現行値は250未満に収め、250〜65535は予約域とする。
  現行selectorで予約域をout-of-tree variantへ割り当てることはできない。
- **変更不可規約**: 1.x 公開済みリリースに含まれた値は以後変更不可 (改番禁止)。 廃止は欠番として残す
  (再利用禁止)。 利用者は**定数名で参照する**こと (生数値のハードコードは契約外)。
- 実装を持たないplatformの識別entry (AVR等) も同じレジストリに置く。実装有無は属性であり、
  番号空間を分けない。

選択の入力と結果は別macroで表す:

| 役割 | macro | `M5HAL_V2_VARIANT_ID_NONE`の意味 |
|---|---|---|
| user input | `M5HAL_CONFIG_VARIANT_<KIND>` | overrideなし。ordered first-hitで自動選択 |
| read-only output | `M5HAL_V2_SELECTED_VARIANT_<KIND>` | scan参加variantがそのkindを一つもofferしなかった |

`<KIND>`はGPIO / I2C / SPI / I2S / PDM / UART / RUNTIME / RUNTIME_MUTEX / RUNTIME_TASK /
RUNTIME_EVENT。入力にはレジストリの名前付きIDを使う。指定IDが未登録、buildのscanに不参加、対象kindを
offerしない、またはfeature gateでofferが無効ならcompile errorとなり、出力を`NONE`へ黙って縮退させない。
入力macroは定義済みのC++整数定数式へ展開する必要があり、名前付きIDの綴り誤りも`NONE`扱いにはならない。
たとえばremote指定には別途`M5HAL_CONFIG_REMOTE_VARIANT=1`が必要で、POSIX UARTを指定しながら
`M5HAL_CONFIG_POSIX_UART=0`にする構成も成立しない。

`offer_all.inl` はflat注入（selector一致後のfirst hit）と同時に、勝者のvariant IDを
**`M5HAL_V2_SELECTED_VARIANT_<KIND>`** (KIND = GPIO / I2C / SPI / I2S / PDM / UART / RUNTIME / RUNTIME_MUTEX / RUNTIME_TASK / RUNTIME_EVENT) に焼き付ける。
どの variant も offer しなかった kind は `M5HAL_V2_VARIANT_ID_NONE` になる
(`M5HAL_v2.hpp` が scan 後に補完)。
各 `_offer.hpp` は自分の ID を `M5HAL_VARIANT_CURRENT_ID_` で申告する。

プリプロセッサ整数定数なので **`#if` と `static_assert` の両方**で使える:

```cpp
// 期待 variant の固定 (ずれたらコンパイルエラー)
static_assert(M5HAL_V2_SELECTED_VARIANT_I2C == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO,
              "this sketch assumes the arduino I2C variant");

// 選択結果による条件コンパイル (native provider を直接指定する場合など)
m5hal::i2c::BusConfig cfg;
cfg.pin_scl = SCL;
cfg.pin_sda = SDA;
#if M5HAL_V2_SELECTED_VARIANT_I2C == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
auto bus = M5_Hal.I2C.acquire(cfg, m5hal::native::borrowed(Wire));
#else
auto bus = M5_Hal.I2C.acquire(cfg);
#endif
```

facade bus kindの無印`Bus`はruntime facadeであり、`Bus_<variant>`の型aliasではない。勝者の確認には
selected-variant markerを使う。GPIOの`Port` / `GPIO`とruntimeの`Mutex` / `Task` / `Event`は型aliasである。

実装注: `#define` は置換リストを展開しないため、 first-hit 時の `M5HAL_VARIANT_CURRENT_ID_` の
**値**を別マクロへ転送することはプリプロセッサでは不可能 — `offer_all.inl` の各 kind ブロックに
variant ID ごとの `#elif` チェーンを書き下しているのはこの制約による。 チェーン末尾の `#else`
は `#error`: レジストリに居ない ID で offer した variant は静かに次点へ flat 注入を譲らず、
コンパイルエラーで止まる。

### 型付きミラー (`variant_id_t` / `variantIdName`)

`variants/ids.hpp` は `#define` レジストリ (上記、 `#if` で使える正本) に加えて、 同じ並びの
X-macro リスト `M5HAL_V2_VARIANT_ID_LIST_` と、 そこから導出する型付きミラーを提供する:

- `m5::hal::v2::variant_id_t` — `enum class : uint16_t` (u16 = ワイヤ幅)。 値は `#define` と同一
- `m5::hal::v2::variantIdName(id)` — レジストリ名を返す `constexpr` 関数 (`variant_id_t` /
  生 u16 の両オーバーロード。 未登録値は `"UNKNOWN"`)。 marker 値の診断表示向け

プリプロセッサは `#define` をマクロ展開から生成できないため `#define` リストは手書きのまま残るが、
両リストの整合はコンパイル時に検証される: X リストへの追加は対応する `#define` が無いと
コンパイルエラーになり、 値列の昇順 static_assert (レジストリは並び順 = 値順、 append-only) が
重複・順序ずれを検出する。

検出値も同じレジストリなので、 「この platform variant が GPIO を勝ったか」のような
検出×選択の跨ぎ比較が直接書ける:

```cpp
static_assert(M5HAL_V2_SELECTED_VARIANT_GPIO == M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID,
              "the detected platform's variant should win GPIO here");
```

## 検出 (`_checker.hpp`)

各軸の `variants/` 配下に `_checker.hpp` を配置 (アンダースコア先頭でメタファイルを明示)。 軸自体が分離されているので検出機構もそれに従う。

検出機構は v0.0.x コードを継承しつつ、 platform 系マクロは `M5HAL_V2_` プレフィックスで世代分離する
(無印の `M5HAL_TARGET_PLATFORM_*` は変更不可の v0 ツリーが所有。 v2 は名前だけでなく**番号も継承しない**
— 識別番号は §選択 variant の診断 のレジストリ `variants/ids.hpp` が正本。
詳細は [v0_v2_coexistence.md](v0_v2_coexistence.md) §v2 実装者が破ってはならない唯一の不変条件):

| マクロ | 用途 |
|---|---|
| `M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID` | 検出された platform の variant ID (`variants/ids.hpp` のレジストリ値。 `M5HAL_V2_VARIANT_ID_NONE` = 不明 = native ビルド) |
| `M5HAL_V2_DETECTED_PLATFORM_VARIANT_PATH` | variant ヘッダの動的 include パス (例: `m5_hal/variants/platforms/espressif/esp32`) |
| `M5HAL_FRAMEWORK_HAS_<NAME>` | framework 検出フラグ (例: `M5HAL_FRAMEWORK_HAS_ARDUINO`)。 v0 と同名共有 (定義は token 同一を維持) |

新規検出マクロ (例: `M5HAL_DETECTED_FRAMEWORK_*`) は必要が見えた時点で追加 (先回り追加なし)。

### arduino variant の対応コア (build gate)

`frameworks/_checker.hpp`のallowlistは、arduino-esp32、RP2040 / RP2350 ARM
(arduino-pico)、UNO R4 Minima、SAMD51、STM32 (公式STコア)、nRF52840 (Adafruit nRF52)、
ESP8266、SPRESENSE MainCoreを受理する。それ以外のArduinoコアは`#error`で拒否する。
Arduino-mbed、RP2040 RISC-V、SPRESENSE SubCore、sandeepmistry nRF5は対象外である。

ESP32専用APIは`ESP_PLATFORM`で分岐し、他コアはportable Arduino APIへfallbackする。
ESP8266はESP-IDF API面を持たないため原則fallback経路を使い、利用可能なI2C pin指定と
UART `SerialConfig`だけを専用分岐で使う。

**スコープと既知の制約**:
- v0はarduino-esp32専用であり、非ESP32 Arduinoではv2だけを利用できる。`M5HAL_v0.cpp`は
  配布treeの全TU compileを許すempty TUとなるが、v0 headerのincludeは`#error`で拒否する
- 非ESP32コアの保証範囲はbuild-checkであり、I2C / SPI / UARTの実機動作を保証しない
- 非`ESP_PLATFORM`コアのMutex / Task / Eventは単一task・非ISR前提のstubとなる。nRF52840でも
  FreeRTOS variantは選ばれないため、M5HALの呼び出しを単一taskに限定する
- GPIO `read()` は Arduino core の `digitalRead()` をそのまま使う。Arduino-ESP32ではM5HALの
  `Output` / `OutputOpenDrain` 設定後も入力経路を有効にし、出力中のpad levelとremote watcherを
  ESP-IDF variantと同じ契約で保証する。非ESP coreはbuild-checkのみで、Output設定中のread意味論は
  core依存 (例: Adafruit nRF52は出力latchを返す) のため保証しない。物理入力の観測が必要なら
  `Input` 系modeを使う
- SPRESENSEのNuttX socketはsemantics未検証のためBSD TCP transportを無効とする。SubCore、audio、
  multicore、peripheral実動作は対象外である
- UART の `SERIAL_*` フォーマット定数は、 非 ESP32 コアではコアが macro として定義する分
  だけを使い、未定義の組み合わせを`INVALID_ARGUMENT`で拒否する。信号反転はarduino-esp32と
  ESP8266だけが対応する。この契約は`HardwareSerial`束縛時だけで、plain `Stream`には適用しない
  ([uart.md](uart.md) §variants)
- remote bus server (`hal/v2/remote/server_bus_pool.*`) は引き続き arduino-esp32 専用
  (SPIClass のコア別レイアウト差、 ESP-IDF 専用 API に依存するため)。 非 ESP32 Arduino コアでは
  無効化 (`M5_HAL_REMOTE_SERVER_BUS_POOL_HAS_ARDUINO_BUS_CONFIG_` が ESP_PLATFORM 必須)
- ESP8266 固有の縮退 (コアのペリフェラル実能力に追従):
  - GPIO の pulldown は GPIO16 のみ (`INPUT_PULLDOWN_16`)。 他 pin は素の `INPUT` へ縮退
    (OUTPUT+pull 欠落と同じ縮退ポリシー)
  - `TwoWire::end()` が存在しないため、 I2C の`closeBackend()`はペリフェラルを構成したまま残す
    (`begin()` が再初期化する)
  - UART の RX バッファ malloc はコアの `begin()` 内で行われ、 失敗するとポートが不能のまま
    残る (`HardwareSerial::operator bool()` が false)。 M5HAL は `begin()` 後にこれを検査し
    `OUT_OF_RESOURCE` を返す (成功扱いにしない)。 `rx_buffer_size` は指定があれば
    `setRxBufferSize()` で反映する (TX は FIFO+blocking のため `tx_buffer_size` は適用対象なし)
  - lx106のatomic RMWはM5HALが使う演算だけinterrupt-mask実装で提供し、未提供演算はlink errorとする
  - M5Utilityの`TL_ASSERT`はlx106 newlibとの互換性のためno-opとなる
- 非ESP32 ArduinoのI2C `wire_timeout_ms`とSCL / SDA指定は適用されない。ただしESP8266のpin指定は
  適用する。SPIは指定pinを適用できない場合に`UNSUPPORTED`を返す
- native resourceは`acquire(config, native::borrowed(resource))`を提供する。Arduino core間で生成・
  破棄契約を統一できないためmanaged取得は`UNSUPPORTED`とする
- RP2040 / RP2350のcore1からM5HALを使う構成は未検証であり、allocator / serviceの保護を保証しない
- SAMD51 / STM32のCMSIS `DAC` / `ADC` macroとの衝突を避けるため、`types.hpp`はこれらを`#undef`する

## 追加方法

framework variant、HAL kind、chip capabilityの追加手順とチェックリストは
[../porting_guide/framework.md](../porting_guide/framework.md)を正本とする。選択規則や申告形式を
変更する場合は、本書の該当契約も同時に更新する。

## 現行の提供 variant

| カテゴリ | variant | 役割 |
|---|---|---|
| frameworks | `freertos/` | FreeRTOS OS プリミティブ (Mutex, Task, Event)。 arduino / espidf と同時有効化し、 RUNTIME_MUTEX / RUNTIME_TASK / RUNTIME_EVENT を供給 |
| frameworks | `bsd/` | BSD socket TCP transport (posix / espidf 共通)。 offer 機構非参加 (transport は bus kind ではない)。 `M5HAL_FRAMEWORK_HAS_BSD_SOCKET` で検出 |
| frameworks | `arduino/` | allowlist対象Arduino coreのHAL具象 + runtime time (Arduino core millis/micros/delay) |
| frameworks | `espidf/` | ESP-IDF framework の HAL 具象 + runtime time (esp_timer + vTaskDelay) |
| frameworks | `posix/` | POSIX host の UART 具象 (termios serial。 opt-out = 既定有効、 `M5HAL_CONFIG_POSIX_UART=0` で UART kind のみ抑止) + runtime (CLOCK_MONOTONIC + std::timed_mutex) |
| frameworks | `remote/` | I2C/SPI/UART/I2S/PDM の remote proxy 型。 `Hal::connect` 経由の remote acquire 自体はフラグ不要 (proxy と backend は umbrella `M5HAL_v2.hpp` と `M5HAL_v2.cpp` が無条件に include / コンパイルする)。 `M5HAL_CONFIG_REMOTE_VARIANT=1` は winner scan への参加 (remote variant の型 alias compile fence / host-side remote build) だけを opt-in する |
| frameworks | `software/` | software bit-bang による I2C / SPI の framework 中立具象 |
| frameworks | `stub/` | 実装済み kind の no-op / フェイク fallback (現行は GPIO と runtime、 test 用) |
| platforms | `espressif/esp32/` | ESP32 family (`esp32` / `s2` / `s3` / `c2` / `c3` / `c5` / `c6` / `c61` / `h2` / `p4`) のレジスタ直叩き具象 |

## 関連

- [../architecture.md](../architecture.md)
- [../style/coding_style.md](../style/coding_style.md) §variant 機構の規則
- [../reference/directory-layout.md](../reference/directory-layout.md) (ディレクトリ ⇔ namespace 1:1 規則 + variants 配下の chip 別分離 + `hal/<kind>` の 3 パターン規則)
