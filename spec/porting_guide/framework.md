# porting_guide/framework — variant と関連実装単位の追加手順

> **読者**: 新しい framework variant、HAL kind、chip capability を実装する実装者向け。

SPI バックエンドを例に、framework variant を M5HAL に追加する全手順を示す。
I2C / UART / I2S / PDM も同じパターンに従う。HAL kindとchip capabilityの追加手順は
§14と§15に示す。

## 1. ディレクトリを作る

```
src/m5_hal/variants/frameworks/<name>/
  _offer.hpp              capability 自己申告 (マクロ宣言のみ)
  hal.hpp                 per-kind hub (kind 別ヘッダを include)
  hal.inl                 per-kind 実装 hub (kind 別 .inl を include)
  hal/spi/
    spi.hpp               Bus_<name> + portable factory + optional NativeProvider
    spi.inl               init / closeBackend / beginOperation / endOperation / transfer の実装
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

- `ALIAS_` — provider symbolのsuffixに使う短縮名 (`Bus_myfw`, `NativeProvider_myfw`)
- `BASE_NS_` — `m5::` からの相対 namespace (層名 `hal` は含めない)
- `ID_` — `variants/ids.hpp` に登録する variant ID 定数
- `HAS_HAL_<KIND>_` — 提供する kind を 1 つずつ申告。未完成の kind は申告しない

## 3. variant ID を登録する

`src/m5_hal/variants/ids.hpp` を編集:

1. `#define` レジストリに追加 (framework レンジ 1〜49、append-only):
   ```cpp
   #define M5HAL_V2_VARIANT_ID_FRAMEWORK_MYFW 8
   ```
2. X-macro リスト `M5HAL_V2_VARIANT_ID_LIST_` に同位置で追加:
   ```cpp
   X(FRAMEWORK_MYFW, 8)
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

この手順はsource treeへの統合を前提とする。`M5HAL_CONFIG_VARIANT_SPI`はscan済みproviderから勝者を
指定する入力であり、未登録・未scanの外部providerを発見するhookではない。250〜65535の予約域も任意の
out-of-tree IDとしては使用できない。

## 7. portable provider と Bus を実装する

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

### 7.1 portable config boundary (spi.hpp)

公開取得はkind共通の`spi::BusConfig` (`IBusConfig`)だけを受ける。variantごとの
`BusConfig_<name>`を公開acquireのselectorとして追加しない。framework固有native resourceは
portable configへ混ぜず、必要なproviderだけが§7.4のownership policyを実装する。remoteも同じ
portable configだけをwireへ送り、native policyは送信前に`UNSUPPORTED`とする。

### 7.2 Bus (spi.hpp)

```cpp
class Bus_myfw : public IBus {
public:
    result_t<void> init(const IBusConfig& config);

    types::backend_kind_t backendKind() const override;
    int8_t controllerId() const override;
    bus::BusCapabilities capabilities() const override;

protected:
    bus::CloseOutcome closeBackend(void) override;
    result_t<void> beginOperationBackend(
        bus::OperationContext<MasterAccessConfig>& context) override;
    result_t<void> endOperationBackend(
        bus::OperationContext<MasterAccessConfig>& context) override;
    result_t<void> transferBackend(
        bus::OperationContext<MasterAccessConfig>& context,
        const TransferDesc& desc,
        data::Source* src, size_t tx_len,
        data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransferBackend(
        bus::OperationContext<MasterAccessConfig>& context) override;
    bool transferBusyBackend(
        bus::OperationContext<MasterAccessConfig>& context) override;

private:
    // variant 固有の内部状態
};
```

### 7.3 portable factory (spi.hpp)

offer scanが勝者providerをbindできるよう、同じnamespaceにfactoryを置く:

```cpp
inline result_t<std::unique_ptr<IBus>> makePortableBackend_myfw(
    const bus::LocalResourceContext& resources,
    const IBusConfig& config)
{
    return bus::makePortableBackend<IBus, Bus_myfw, IBusConfig>(resources, config);
}
```

facade (`spi::Bus`) と`Hal.SPI.acquire(cfg)`は、このfactory経由でbuildの勝者providerを使う。

### 7.4 optional native ownership policy (spi.hpp)

既存native resourceを安全に借用・所有できるproviderだけが`NativeProvider_myfw<Policy>`を特殊化する。
公開呼出しは`acquire(cfg, native::borrowed(resource))`または
`acquire(cfg, native::managed(resource))`であり、別名の`attach` / `open`は追加しない。
借用だけが安全ならmanaged policyは未対応のままにする。未対応policyは`UNSUPPORTED`を返す。

## 8. Bus メソッドの実装契約

### init

- config を `_config` にコピー保存する
- raw config の pin は、`-1` がそのfieldで未指定を表せるかを先に判定する。registry/local factory経路では
  受け取った`LocalResourceContext.gpio->tryGetPin(num)`で解決し、範囲外・未登録ならエラーを返す。
  backendからglobal `M5_Hal`を直接参照しない。直接構築Busのcompatibility bootstrapだけは
  `IBus::localResources()`のdefault contextへ集約する。asserting APIの
  `getPin()` は検証済みinvariantにだけ使い、利用者入力へ直接使わない
- framework 固有のリソースを確保する
- 失敗時は `INVALID_ARGUMENT` または `IO_ERROR` を返す

### capabilities

- 共通契約とID一覧は[bus_capabilities.md](../design/bus_capabilities.md)を正本とする
- 実装済みoperationと現在のinstance configだけを申告し、kind名や将来予定から能力を推測しない
- provider固有のhard ceilingだけをlimitへ載せ、未知値を0として載せない
- `bus::IBus::capabilities()`をbuilderのbaseに使うと、`backendKind()`と非0の`maxFrequency()`が共通射影される
- query内でheap allocation、driver I/O、registry探索を行わない

### closeBackend

- `init`で確保したリソースを終了するprotected backend hook
- 成功時は`Closed`へ遷移する。direct concrete/facadeのpublic `close()`成功後は同じobjectを`init()`で再利用できる
- 失敗は`CloseOutcome`で`NoMutation`（rollback可能）か`PartialOrUnknown`（quarantine必須）に分類する
- common `IBus`へpublic close/releaseを追加しない。registry管理busは`Hal.SPI.close(shared_ptr&)`だけでconsuming final closeする

### beginOperationBackend / endOperationBackend

- CS pin の assert / deassert を行う
- D/C pin をdata側idleのHighへ初期化する (§D/C pin 契約 参照)
- Accessorがlock取得後、common non-virtual `beginOperation(context)`の検査・slot登録を経て一度だけ呼ぶため、
  backendはlock済み・Context検査済みとして動作する。requested configは`context.config`、timeout残余・
  generation・modeは`context.runtime`から得る。raw owner引数を追加しない
- `beginOperationBackend`のpartial setup失敗は同関数内でrollbackする。common入口もslotを失効させる。
  成功後はI/Oが失敗しても`endOperationBackend`が一度呼ばれ、戻り値がerrorでもcommon入口がContextを失効させる

### transferBackend

- `TransferDesc` の各フィールドに従い、command / address / dummy / data の各 phase を実行する
- public `transfer(context, ...)`は検査済みContextだけをこのhookへ渡す。providerはpublic入口をoverrideしない
- `src` / `dst` は nullable (`nullptr` = その方向のデータなし)
- command / address phase のバイト数は `TransferTotals` に**含めない**
- エラー時は適切な `error_t` を返す

### waitTransferBackend / transferBusyBackend

- 同期実装なら `waitTransferBackend` は即座に totals を返し、`transferBusyBackend` は常に `false`
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

1. `beginOperation`でD/C pinをdata側idleの **High** にする。このAPIは`TransferDesc`を
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
| `NOT_IMPLEMENTED` | 選択した build / variant に API の実装経路自体がない |
| `UNSUPPORTED` | API は有効だが、この instance / config / peer が要求する操作・能力を提供しない |
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
static_assert(sizeof(m5::hal::v2::spi::BusConfig) > 0, "portable BusConfig reachable");
#endif
```

## 13. 検証

variant 追加の最低限の検証:

1. **native ビルド** — build_check のコンパイルが通る
2. **ターゲットビルド** — 対象プラットフォームで PlatformIO / idf.py ビルドが通る
3. **HowToUse サンプル** — `examples/v2/HowToUse/SPI/` 等で実機動作を確認
4. **CI** — `clang-format` + PlatformIO matrix + IDF component build が全 green

## 14. HAL kind を追加する

1. `src/m5_hal/hal/v2/<kind>/<kind>.hpp`（必要なら`.inl`）で共通抽象とconfig / accessor / descを定義する
2. bus kindでは`types::BusKind`へkind tagを追加し、`BusConfig` / `AccessConfig`派生のctorでそのtagを設定する
3. `M5HAL_v2.hpp` / `M5HAL_v2.cpp`に共通header / implをincludeする
4. `_macro/offer_all.inl`へ新kindのディスパッチブロックを追加する。HASゲート、
   `M5HAL_OFFER_KIND_NS_`、platform束縛ガード、`M5HAL_V2_SELECTED_VARIANT_<KIND>`チェーン、
   `offer_kind.inl`のincludeを既存kindと同形にする。`M5HAL_v2.hpp`のscan後NONE補完も追加する
5. 具象が揃ったvariantだけ`_offer.hpp`で新kindを申告する。stubがfallbackを担う場合はstub具象も同時に用意する
6. native build_checkと最低1つのAPI-level unit testを追加する

bus構造を持たない設備kind（現行はruntimeのみ）は、手順1をkind契約の定義に読み替え、手順2を
省く。runtimeはNONE補完を持たず、選択不成立をearly scanの`#error`で止める
（[../design/runtime.md](../design/runtime.md) §early scan (bus kind との違い)）。

## 15. chip capability を追加する

chip capabilityは、HAL kindの勝者選択とは独立したチップ固有ユーティリティを指す。`offer_all.inl`の
HAL kind申告には載せず、platform headerから公開namespaceへのnamed `using`で明示公開する。

1. platform variant内の適切な`hal/<area>/`に型を置く
2. platform headerが常にincludeされる経路で、`m5::hal::v2::<area>`へnamed `using`を追加する
3. `test/v2/build_check/build_check.hpp`で公開名からの到達をcompile fenceに入れる
4. `spec/design/<area>.md`へ、chip capabilityとする理由、対象chip、no-op条件、検証範囲を書く

## チェックリスト

framework variant追加の完了判定は本チェックリストを正本とする。

- [ ] `variants/frameworks/<name>/`に`_offer.hpp`、`hal.hpp`、必要なら`hal.inl`とkind実装を配置した (§1)
- [ ] `_checker.hpp`へ`M5HAL_FRAMEWORK_HAS_<NAME>`を追加し、利用者設定ならconfigurationにも登録した (§4)
- [ ] `variants/ids.hpp`のframeworkレンジ末尾へ名前付きIDとX-macro entryをappendした (§3)
- [ ] `_offer.hpp`でalias、base namespace、ID、完成済みkindだけを申告した (§2)
- [ ] `M5HAL_v2.hpp`の意図したscan位置と、必要なら`M5HAL_v2.cpp`へincludeを追加した (§5)
- [ ] `offer_all.inl`の全selected-marker chainへIDを追加した (§6)
- [ ] portable `IBusConfig`を受ける`makePortableBackend_<name>`を実装した (§7.1, §7.3)
- [ ] `Bus_<name>`はprotected `*Backend` virtualを実装し、public checked入口をoverrideしていない (§7.2, §8)
- [ ] `closeBackend()`のfailure classificationを実装した (§7.2, §8)
- [ ] native resourceが必要で安全に提供できるpolicyだけ`NativeProvider_<name>`をspecializeした (§7.4)
- [ ] suffix付き型の到達とkind別selectorによる明示選択をcompile fenceで検証した (§12)
- [ ] native、対象target、HowToUse、CIの該当範囲を検証した (§13)
- [ ] [../reference/directory-layout.md](../reference/directory-layout.md)の検索台帳または構造が変わる場合は更新した
- [ ] [../design/variants.md](../design/variants.md)のscan順・申告契約が変わる場合は更新した

現行のselectorはsource treeへ統合済みのproviderだけを選ぶ。未登録・未scanのout-of-tree providerを
発見せず、250〜65535の予約域も任意のout-of-tree IDとして利用できない。中央レジストリ、checker、scan、
selected-marker chainのいずれかを欠くvariantの指定はcompile errorとなる。

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
