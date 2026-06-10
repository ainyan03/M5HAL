# v0/v1 共存戦略

公開済の v0 API と v1 API を 1 ライブラリ内で同時提供し、 既存利用者を保護しながら v1 へ移行できる構成を定義する。

## 基本方針

1. **物理共存** — v0 と v1 を同じライブラリ内で同時ビルド・同時リンクする
2. **namespace 分離** — v0 は `m5::hal::v0::*`、 v1 は `m5::hal::v1::*` に置く
3. **inline 切替** — 既定動作は `inline namespace` で切り替える
4. **マクロ制御** — `M5HAL_V0_INLINE` / `M5HAL_V1_INLINE` で既定 namespace を制御する
5. **v0 freeze** — v0 側は freeze 例外として保持し、 v1 の配置規約は適用しない
6. **variants は v1 オンリー** — variant 機構は v1 用のみ提供する

## 切替マクロ

```cpp
#ifndef M5HAL_V0_INLINE
#define M5HAL_V0_INLINE 1
#endif

#ifndef M5HAL_V1_INLINE
#define M5HAL_V1_INLINE 0
#endif

#if M5HAL_V0_INLINE && M5HAL_V1_INLINE
#error "M5HAL_V0_INLINE and M5HAL_V1_INLINE are mutually exclusive"
#endif
```

`M5HAL_V1_INLINE=1` と `M5HAL_V0_INLINE=0` を組み合わせると、 `m5::hal::Foo` は v1 に resolve される。

## namespace 配置

- v0: `m5::hal::v0::*`
- v1: `m5::hal::v1::*`
- 既定 API: `inline namespace` によって `m5::hal::*`

v1 側の inline 性は `M5HAL_v1.hpp` 冒頭の forward declaration で確定させる。

```cpp
namespace m5 { namespace hal { M5HAL_INLINE_V1 namespace v1 {} } }
```

## 物理 layout

```text
src/
  M5HAL.hpp              後方互換 shim
  M5HAL_v0.{hpp,cpp}     v0 entry
  M5HAL_v1.{hpp,cpp}     v1 entry
  m5_hal_config.hpp
  m5_hal/
    hal/
      v0/
      v1/
    variants/
```

`hal/v0/` は freeze 例外として保持し、 `hal/v1/` は v1 の配置規約に従う。 `variants/` 配下の HAL 実装は v1 のみを提供する。

## エントリヘッダ

| ヘッダ | 用途 | 公開する namespace |
|---|---|---|
| `M5HAL.hpp` | 後方互換 shim | v0 (= `m5::hal::*`) |
| `M5HAL_v0.hpp` | 明示的に v0 を選ぶコード | v0 (= `m5::hal::*`) |
| `M5HAL_v1.hpp` | 明示的に v1 を選ぶコード | v1 (= `m5::hal::v1::*`) |

同一 sketch 内での併用が可能。 同一 TU での両エントリ include も安全: include ガードの世代分離に加え、 platform checker の macro 名前空間も世代分離されている (v0 = 無印 `M5HAL_TARGET_PLATFORM_*`、 v1 = `M5HAL_V1_TARGET_PLATFORM_*`)。 fence は native が `test_coexist_include` (gtest)、 device が `v0v1_check_*` env (esp32 / esp32s3 の arduino + espidf ビルドで両エントリを 1 TU に include し、 static_assert で双方の macro 値を検査)。

## 制約

- v0 と v1 は namespace 分離により ODR 衝突しない
- macro 名はライブラリ全体で共有されるため、 `M5HAL_` プレフィックスで統一する (include ガードも同様で、 v0 側は `M5_HAL_V0_` プレフィックスにより v1 と衝突しない。 同一 TU での両エントリ include は `test_coexist_include` と `v0v1_check_*` が fence)
- 世代間で値が異なり得る macro は名前ごと世代分離する: platform checker は v0 = 無印、 v1 = `M5HAL_V1_` プレフィックス
- 両世代が同名で定義する macro (`M5HAL_FRAMEWORK_HAS_ARDUINO` / `_FREERTOS` / `_SDL`、 `M5HAL_STATIC_MACRO_*`) は **定義を token 単位で同一に保つ**こと (同一定義の再定義は規格上無害)。 v0 側は凍結のため、 この同一性維持の義務は v1 側の編集にかかる — 逸脱すると coexist fence のビルドで redefinition warning として現れる
- cross-cutting な型は v0 / v1 のそれぞれが同名型を持つ

## v0 の既知制限 (凍結対象)

v0 は公開互換のための凍結ツリーであり、 以下の制限は修正せず v1 への移行で解消する:

- **対応 chip**: v0 の platform checker が知るのは ESP32 (無印) / S2 / S3 / C3 / C6 / H2 / P4 系の当時の一覧まで。 それ以降の新 chip (C5 / C61 等) は generic fallback で動作し、 platform 固有最適化は乗らない。 新 chip の一次対応は v1 のみ
- **software I2C / SPI**: 複数インスタンス管理と排他制御が未整備 (単一インスタンス前提)。 ソース内の TODO は凍結のため対応しない
- **エラーコード**: 細分化されていない (I2C 系 + 汎用のみ)。 詳細な分類は v1 `error_t` を使う

## 関連

- [../reference/directory-layout.md](../reference/directory-layout.md)
- [../style/migration.md](../style/migration.md)
