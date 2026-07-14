# porting_guide/framework — framework variant の追加手順

> **読者**: 新しい framework variant を実装する実装者向け。

SPI バックエンドを例に、framework variant を M5HAL に追加する全手順を示す。
I2C / UART / I2S も同じパターンに従う。

## 1. ディレクトリを作る

```
src/m5_hal/variants/frameworks/<name>/
  _offer.hpp              capability 自己申告 (マクロ宣言のみ)
  hal.hpp                 per-kind hub (kind 別ヘッダを include)
  hal.inl                 per-kind 実装 hub (kind 別 .inl を include)
  hal/spi/
    spi.hpp               BusConfig_<name> + Bus_<name> + BackendFor 特殊化
    spi.inl               init / release / beginTransaction / endTransaction / transfer の実装
```

`hal.hpp` と `hal.inl` は薄い hub ファイル:

```cpp
// hal.hpp
#pragma once
#include "hal/spi/spi.hpp"
```

```cpp
// hal.inl
#include "hal/spi/spi.inl"
```

## 2. `_offer.hpp` を書く

**include guard を持たない** (re-include 前提):

```cpp
// clang-format off

#define M5HAL_VARIANT_CURRENT_ALIAS_   myfw
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::myfw
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V2_VARIANT_ID_FRAMEWORK_MYFW

#define M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_ 1

// clang-format on
```

- `ALIAS_` — suffix に使う短縮名 (`Bus_myfw`, `BusConfig_myfw`)
- `BASE_NS_` — `m5::` からの相対 namespace (層名 `hal` は含めない)
- `ID_` — `variants/ids.hpp` に登録する variant ID 定数
- `HAS_HAL_<KIND>_` — 提供する kind を 1 つずつ申告。未完成の kind は申告しない

## 3. variant ID を登録する

`src/m5_hal/variants/ids.hpp` を編集:

1. `#define` レジストリに追加 (framework レンジ 1〜49、append-only):
   ```cpp
   #define M5HAL_V2_VARIANT_ID_FRAMEWORK_MYFW 7
   ```
2. X-macro リスト `M5HAL_V2_VARIANT_ID_LIST_` に同位置で追加:
   ```cpp
   X(FRAMEWORK_MYFW, 7)
   ```

値は既存の末尾 + 1。**一度公開した値は以後変更不可** (改番・再利用禁止)。

## 4. framework 検出を追加する

`src/m5_hal/variants/frameworks/_checker.hpp` に検出マクロを追加:

```cpp
#if defined(MYFW_SDK_VERSION)
#define M5HAL_FRAMEWORK_HAS_MYFW 1
#endif
```

SDK 固有のプリプロセッサマクロで判定する。

## 5. scan order に組み込む

`src/M5HAL_v2.hpp` に variant の include を追加。
挿入位置は走査順に従う (詳細は [../design/variants.md](../design/variants.md) §走査順):

```cpp
// -- myfw framework ---
#if M5HAL_FRAMEWORK_HAS_MYFW
#include "./m5_hal/variants/frameworks/myfw/hal.hpp"
#include "./m5_hal/variants/frameworks/myfw/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif
```

実装が `.inl` を持つ場合は `src/M5HAL_v2.cpp` にも:

```cpp
#if M5HAL_FRAMEWORK_HAS_MYFW
#include "./m5_hal/variants/frameworks/myfw/hal.inl"
#endif
```

## 6. offer_all.inl の #elif チェーンに追加

`src/m5_hal/_macro/offer_all.inl` の各 kind ブロックの `#elif` チェーンに 1 行追加:

```cpp
#elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_MYFW
#define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_FRAMEWORK_MYFW
```

漏れはチェーン末尾の `#error` で検出される。

## 7. BusConfig と Bus を実装する

variant のヘッダは `namespace m5::hal::v2::spi { ... }` 内に定義する。
C++ の名前探索が enclosing namespace を辿るため、sibling / 親 namespace の型は
短い相対名で参照できる:

| 完全修飾名 | 省略形 | 解決経路 |
|---|---|---|
| `::m5::hal::v2::result_t<T>` | `result_t<T>` | 親 namespace |
| `::m5::hal::v2::bus::IAccessor` | `bus::IAccessor` | sibling |
| `::m5::hal::v2::data::Source` | `data::Source` | sibling |
| `::m5::hal::v2::spi::MasterAccessConfig` | `MasterAccessConfig` | 同 namespace |
| `::m5::hal::v2::spi::TransferDesc` | `TransferDesc` | 同 namespace |
| `::m5::hal::v2::bus::TransferTotals` | `bus::TransferTotals` | sibling |
| `::m5::hal::v2::types::backend_kind_t` | `types::backend_kind_t` | sibling |

以下のコード例はすべてこの省略形で書く。

### 7.1 BusConfig (spi.hpp)

`m5::hal::v2::spi` namespace 直下に定義:

```cpp
namespace m5::hal::v2::spi {

struct BusConfig_myfw : public IBusConfig {
    using IBusConfig::IBusConfig;
    // framework 固有フィールド (例: native handle)
    MyFwSpiHandle* native_handle = nullptr;
};
```

**規約**:
- `IBusConfig` を直接インスタンス化せず、空派生でも必ず variant 固有の型を作る
- これにより typed init が兄弟 variant の config を**コンパイルエラー**で弾く

### 7.2 Bus (spi.hpp)

```cpp
class Bus_myfw : public IBus {
public:
    result_t<void> init(const BusConfig_myfw& config);
    result_t<void> release(void) override;

    result_t<void> beginTransaction(bus::IAccessor* owner,
                                    const MasterAccessConfig& cfg) override;
    result_t<void> endTransaction(bus::IAccessor* owner,
                                  const MasterAccessConfig& cfg) override;
    result_t<void> transfer(bus::IAccessor* owner,
                            const MasterAccessConfig& cfg,
                            const TransferDesc& desc,
                            data::Source* src, size_t tx_len,
                            data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner,
                                               const MasterAccessConfig& cfg) override;
    bool transferBusy(bus::IAccessor* owner) override;

    types::backend_kind_t backendKind() const override;
    int8_t controllerId() const override;

private:
    // variant 固有の内部状態
};
```

### 7.3 BackendFor 特殊化 (spi.hpp)

同じ namespace 内なので `BackendFor` も短く書ける:

```cpp
template <>
struct BackendFor<BusConfig_myfw> {
    using type = Bus_myfw;
};
```

facade (`spi::Bus`) が `init(BusConfig_myfw)` を受けたとき、
この特殊化経由で `Bus_myfw` を自動選択する。

## 8. Bus メソッドの実装契約

### init

- config を `_config` にコピー保存する
- raw config の pin は、`-1` がそのfieldで未指定を表せるかを先に判定する。指定されたpinは
  `M5_Hal.Gpio.tryGetPin(num)` で解決し、範囲外・未登録ならエラーを返す。asserting APIの
  `getPin()` は検証済みinvariantにだけ使い、利用者入力へ直接使わない
- framework 固有のリソースを確保する
- 失敗時は `INVALID_ARGUMENT` または `IO_ERROR` を返す

### release

- `init` で確保したリソースを解放する
- 解放済みの再呼び出しは安全に無視する

### beginTransaction / endTransaction

- CS pin の assert / deassert を行う
- D/C pin をdata側idleのHighへ初期化する (§D/C pin 契約 参照)
- accessorがlockを取得してから呼ぶため、backendはlock済みとして動作する。`owner`は必要なら
  transactionの整合確認に使うが、基底`lock`/`unlock`を再実行しない

### transfer

- `TransferDesc` の各フィールドに従い、command / address / dummy / data の各 phase を実行する
- `src` / `dst` は nullable (`nullptr` = その方向のデータなし)
- command / address phase のバイト数は `TransferTotals` に**含めない**
- エラー時は適切な `error_t` を返す

### waitTransfer / transferBusy

- 同期実装なら `waitTransfer` は即座に totals を返し、`transferBusy` は常に `false`
- 非同期実装なら進行中の DMA / タスクの完了を待つ

### backendKind / controllerId

- HW ペリフェラルを使うなら `Hardware` を返す
- `controllerId` はM5HAL controller poolの0始まりindex。ESP-IDF SPIなら
  `SPI2_HOST → 0`、`SPI3_HOST → 1`で、native host値そのものではない。software・台帳外など
  controllerを申告しないbackendは`-1`を返す。native値が必要ならvariant固有query
  (`Bus_espidf::nativeHost()`等)を使う

## 9. D/C pin 契約 (SPI 固有)

SPI の D/C (Data/Command) pin は 3 層で解決される:

| 層 | フィールド | 意味 |
|---|---|---|
| Bus | `IBusConfig::pin_dc` | bus 全体のデフォルト D/C pin |
| Accessor | `MasterAccessConfig::pin_dc` | device 別 override (-1 = bus default) |
| Transfer | `TransferDesc::command_dc_level` 等 | per-call の phase 別制御 |

**実装が守るべき契約**:

1. `beginTransaction` でD/C pinをdata側idleの **High** にする。このAPIは`TransferDesc`を
   受け取らないため、per-transfer levelはここでは参照しない
2. `transfer` の command phase で `desc.command_dc_level` に従い D/C を切り替える
3. `transfer` の address phase で `desc.address_dc_level` に従い D/C を切り替える
4. command / address phase の後、data phase に入る前にD/Cを復帰させる
   - phase別指定が1つでもあり、`desc.data_dc_level < 0`ならHigh
   - `desc.data_dc_level >= 0`ならその値
   - phase別指定が無い場合だけlegacy `desc.dc_level_valid` / `desc.dc_level`をtransfer全体へ適用
5. `write()` (plain data write) はtransaction開始時のHighを維持し、`TransferDesc`由来の追加切替は行わない

**実装例** (擬似コード):

```cpp
result_t<void> transfer(..., const TransferDesc& desc, ...) {
    auto dc_pin = resolvePin(cfg.pin_dc, _config.pin_dc);
    bool has_phase_dc = (desc.command_dc_level >= 0)
                     || (desc.address_dc_level >= 0)
                     || (desc.data_dc_level >= 0);
    if (!has_phase_dc && desc.dc_level_valid) setDC(dc_pin, desc.dc_level);
    // command phase
    if (desc.command_bytes > 0) {
        if (desc.command_dc_level >= 0) setDC(dc_pin, desc.command_dc_level);
        sendMeta(desc.command, desc.command_bytes);
    }
    // address phase
    if (desc.address_bytes > 0) {
        if (desc.address_dc_level >= 0) setDC(dc_pin, desc.address_dc_level);
        sendMeta(desc.address, desc.address_bytes);
    }
    // data level restore
    if (has_phase_dc) setDC(dc_pin, desc.data_dc_level >= 0 ? desc.data_dc_level : 1);
    // data phase
    transferData(src, tx_len, dst, rx_len);
    return {};
}
```

## 10. Source / Sink の読み書き

`data::Source` と `data::Sink` はストリーム抽象。variant は以下のメソッドを使う:

```cpp
// Source からデータを読む (tx 方向)
auto span = src->peek(requested_len);   // 利用可能なバイト列を覗く
// span.data(), span.size() でアクセス
src->advance(actually_consumed);        // 消費した分を進める

// Sink にデータを書く (rx 方向)
auto span = dst->reserve(requested_len);  // 書き込み先を確保
// span にデータを書き込む
dst->commit(actually_written);            // 書き込んだ分を確定
```

DMA 転送する場合は、peek/reserve で得た span を DMA バッファにコピーし、
完了後に advance/commit する。ゼロコピーが可能なら span を直接渡す。

## 11. エラーコードの使い分け

| エラー | 用途 |
|---|---|
| `OK` | 成功 |
| `INVALID_ARGUMENT` | 不正な config (nullptr, pin 範囲外等) |
| `NOT_IMPLEMENTED` | この variant がサポートしない操作 |
| `IO_ERROR` | HW 通信エラー |
| `BUSY` | リソースが他で使用中 |
| `TIMEOUT_ERROR` | 操作がタイムアウト |
| `INVALID_STATE` | 不正な操作順序 (例: init 前の transfer) |

## 12. build_check にコンパイルフェンスを追加

`test/v2/build_check/build_check.hpp` に、variant 固有型が到達できることを検証する
static assertion を追加する:

```cpp
#if M5HAL_FRAMEWORK_HAS_MYFW
static_assert(sizeof(m5::hal::v2::spi::Bus_myfw) > 0, "Bus_myfw reachable");
static_assert(sizeof(m5::hal::v2::spi::BusConfig_myfw) > 0, "BusConfig_myfw reachable");
#endif
```

## 13. 検証

variant 追加の最低限の検証:

1. **native ビルド** — build_check のコンパイルが通る
2. **ターゲットビルド** — 対象プラットフォームで PlatformIO / idf.py ビルドが通る
3. **HowToUse サンプル** — `examples/v2/HowToUse/SPI/` 等で実機動作を確認
4. **CI** — `clang-format` + PlatformIO matrix + IDF component build が全 green

## チェックリスト

正式なチェックリスト (ディレクトリ一式・`_offer.hpp`・variant ID 登録・検出マクロ・scan order・
`offer_all.inl`・build_check・配置表更新) は [../design/variants.md](../design/variants.md)
§追加時チェックリストが正本。framework variant 固有で同ページに無い項目のみここに残す:

- [ ] `BusConfig_<name>` — `IBusConfig` の空派生 (最低限) または拡張 (§7.1)
- [ ] `Bus_<name>` — `IBus` の全 virtual override (§7.2)
- [ ] `BackendFor<BusConfig_<name>>` — 特殊化 (§7.3)

## 参照実装

| variant | 特徴 | 参照先 |
|---|---|---|
| `software` | 最もシンプル (bit-bang、framework 非依存) | `variants/frameworks/software/hal/spi/` |
| `arduino` | SPIClass 委譲、D/C pin キャッシュ | `variants/frameworks/arduino/hal/spi/` |
| `espidf` | polling_start/polling_end ダブルバッファ、DMA、ワーカータスク | `variants/frameworks/espidf/hal/spi/` |

新しい variant を書くときは `software` を最初に読み、最小の実装像を掴んでから、
必要に応じて `arduino` / `espidf` の高度なパターンを参考にする。

## 関連

- [../design/variants.md](../design/variants.md) — variant 機構の仕様
- [../design/transfer_desc.md](../design/transfer_desc.md) — TransferDesc
- [../design/bus_accessor.md](../design/bus_accessor.md) — Bus / Accessor の責務
- [../design/data_io.md](../design/data_io.md) — Source / Sink
- [../reference/directory-layout.md](../reference/directory-layout.md) — 配置規約
