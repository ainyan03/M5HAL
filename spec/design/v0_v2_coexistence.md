# v0/v2 共存戦略

> **読者**: 実装者・レビュー向け（設計仕様）。

公開済の v0 API と v2 API を 1 ライブラリ内で同時提供し、 既存利用者を保護しながら v2 へ移行できる構成を定義する。 README / goals / migration からはこのファイルを正本として参照する。

## v2 実装者が破ってはならない唯一の不変条件

> **両世代が同名で定義する macro (`M5HAL_FRAMEWORK_HAS_ARDUINO` / `_FREERTOS` / `_SDL`、 `M5HAL_STATIC_MACRO_*`) は token 単位で定義を同一に保つこと。**

v0 API と ESP32 implementation は互換維持のため機能変更しない。例外は配布
library のビルド成立に必要な target 境界のみ。同一性維持の責任は v2 側の編集に
かかる。逸脱すると、v0 対応環境で同一 TU に両エントリを include するビルド
(`test_coexist_include` / `v0v2_check_*`) で redefinition エラーが発生する。

## 基本方針

1. **物理共存** — v0 対応環境では v0 と v2 を同じライブラリ内で同時ビルド・同時リンクする。非 ESP32 Arduino では v2 のみを生成する
2. **namespace 分離** — v0 は `m5::hal::v0::*`、 v2 は `m5::hal::v2::*` に置く
3. **inline 切替** — `inline namespace` の有無でどちらが `m5::hal::*` として解決されるかを切り替える (既定は v0 が inline)
4. **マクロ制御** — `M5HAL_V0_INLINE` / `M5HAL_V2_INLINE` で既定 namespace を制御する
5. **v0 freeze** — v0 側は配布ビルド用の target 境界を除いて機能変更せず、 v2 の配置規約は適用しない
6. **variants は v2 オンリー** — variant 機構は v2 用のみ提供する

### v0の検証方針

v0の主検証は、公開entry・対応chip/framework・v0/v2同居を守るcompile/link fenceと、既存下流
consumerのbuildである。ただしcompileだけでは、過去に修正したzero-length処理のようなgeneric
実行時回帰を検出できない。このため`test/v0/native/`には、既知の互換契約を守る最小smokeだけを置き、
通常の`test_native`で毎回実行する。

このsmokeはv0の機能拡張や網羅的なdriver testを再開するものではない。v0 sourceを例外的に修正する
場合は、hostで再現可能なら当該回帰をここへ追加し、hardware固有なら既存下流buildに加えて対象実機で
修正点を確認する。新規機能と新規chipの一次対応はv2にのみ追加する。

## namespace 配置 (1 か所のみ定義)

| 世代 | 完全修飾 namespace | `m5::hal::*` として解決される条件 |
|------|-------------------|-----------------------------------|
| v0   | `m5::hal::v0::*` | `M5HAL_V0_INLINE=1`（既定） |
| v2   | `m5::hal::v2::*` | `M5HAL_V2_INLINE=1` かつ `M5HAL_V0_INLINE=0` |

`m5::hal::*` は常にどちらか一方の world を指す。 v2 側の inline 性は `M5HAL_v2.hpp` 冒頭の forward declaration で確定させる:

```cpp
namespace m5 { namespace hal { M5HAL_INLINE_V2 namespace v2 {} } }
```

## 切替マクロ

```cpp
#ifndef M5HAL_V0_INLINE
#define M5HAL_V0_INLINE 1
#endif

#ifndef M5HAL_V2_INLINE
#define M5HAL_V2_INLINE 0
#endif

#if M5HAL_V0_INLINE && M5HAL_V2_INLINE
#error "M5HAL_V0_INLINE and M5HAL_V2_INLINE are mutually exclusive"
#endif
```

既定では v0 が inline。 利用者が v2 への移行をライブラリ全体で切り替えるには `-DM5HAL_V2_INLINE=1 -DM5HAL_V0_INLINE=0` をビルドフラグに追加する。

## エントリヘッダ

| ヘッダ | 用途 | 公開する namespace |
|---|---|---|
| `M5HAL.hpp` | 後方互換 shim | v0 (= `m5::hal::*`) |
| `M5HAL_v0.hpp` | 明示的に v0 を選ぶコード | v0 (= `m5::hal::*`) |
| `M5HAL_v2.hpp` | 明示的に v2 を選ぶコード | v2 (= `m5::hal::v2::*`) |

v0 対応環境では同一 TU での両エントリ include も安全: include ガードの世代分離に加え、 platform checker の macro 名前空間も世代分離されている (v0 = 無印 `M5HAL_TARGET_PLATFORM_*`、 v2 = `M5HAL_V2_DETECTED_PLATFORM_VARIANT_*`)。

ただし v0 の公開 API は ESP32 専用である。非 ESP32 Arduino では、install 済み library
の全 TU をビルドできるよう `M5HAL_v0.cpp` のみ empty TU となるが、`M5HAL.hpp`
または `M5HAL_v0.hpp` を利用者が include すると `#error` で拒否する。それらの target
では `M5HAL_v2.hpp` を明示的に使う。プラットフォームによって `M5HAL.hpp` の解決先を
v2 へ変えることはしない。

## 同一 TU 安全性の保証 (coexist fence)

v0 対応環境で、同一 TU での両エントリ同時 include が安全である根拠:

1. **include ガード分離** — v0 は `M5_HAL_V0_` プレフィックス、 v2 は別系統。 重複定義なし。
2. **platform macro 分離** — 世代間で値が異なり得る macro は名前ごと世代分離する。 v0 = 無印 `M5HAL_TARGET_PLATFORM_*`、 v2 = `M5HAL_V2_DETECTED_PLATFORM_VARIANT_*`。
3. **ODR 非衝突** — namespace が分離されるため、 同名クラス・関数が両世代に存在しても ODR 衝突しない。
4. **macro 同一性** — 上記「唯一の不変条件」により、 両世代が定義する共通名 macro はトークン単位で同一。 同一定義の再定義は C++ 規格上無害。

fence テスト: native = `test_coexist_include` (gtest)、 device = `v0v2_check_*` env (esp32 / esp32s3 の arduino + espidf ビルドで両エントリを 1 TU に include し static_assert で双方の macro 値を検査)。

## 物理 layout

```text
src/
  M5HAL.hpp              後方互換 shim
  M5HAL_v0.{hpp,cpp}     v0 entry
  M5HAL_v2.{hpp,cpp}     v2 entry
  m5_hal_config.hpp
  m5_hal/
    hal/
      v0/                freeze 例外ツリー
      v2/                v2 配置規約に従う
    variants/            v2 のみ
```

## v0 の既知制限

v0 は公開互換のため原則として機能変更しない。配布 library のビルド成立に必要な
target 境界は追加するが、以下の API 制限は v0 を拡張せず v2 への移行で解消する:

- **対応 framework**: v0 の Arduino API は arduino-esp32 専用。非 `ESP_PLATFORM` Arduino では
  v0 implementation を生成せず、v0 public entry の include も明示的に拒否する
- **対応 chip**: v0 の platform checker が知るのは ESP32 (無印) / S2 / S3 / C3 / C6 / H2 / P4 系の当時の一覧まで。 それ以降の新 chip (C5 / C61 等) は generic fallback で動作し、 platform 固有最適化は乗らない。 新 chip の一次対応は v2 のみ。
- **software I2C / SPI**: 複数インスタンス管理と排他制御が未整備 (単一インスタンス前提)。 v0 の機能拡張は行わず v2 で対応する。
- **エラーコード**: 細分化されていない (I2C 系 + 汎用のみ)。 詳細な分類は v2 `error_t` を使う。

## v3/v4 への前方互換レイアウト

将来の世代 (v3/v4) が追加される場合も同じパターンを踏襲する:

- `m5::hal::v3::*` / `m5::hal::v4::*` を追加する
- エントリヘッダ `M5HAL_v3.hpp` / `M5HAL_v4.hpp` を追加する
- 切替マクロ `M5HAL_V3_INLINE` / `M5HAL_V4_INLINE` を同じ排他ガードで追加する
- platform macro は `M5HAL_V3_DETECTED_PLATFORM_VARIANT_*` 系へ分離する
- `m5::hal::*` が指す世代は引き続き 1 つのみ

この設計により、 利用者は移行の準備ができるまで古い世代を明示的に include し続けられる。

## 関連

- [../reference/directory-layout.md](../reference/directory-layout.md)
- [../style/migration.md](../style/migration.md)
