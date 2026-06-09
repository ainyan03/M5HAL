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

同一 sketch 内での併用はできるが、 同一 TU ではどちらか一方に絞る。

## 制約

- v0 と v1 は namespace 分離により ODR 衝突しない
- macro 名はライブラリ全体で共有されるため、 `M5HAL_` プレフィックスで統一する
- cross-cutting な型は v0 / v1 のそれぞれが同名型を持つ

## 関連

- [../reference/directory-layout.md](../reference/directory-layout.md)
- [../style/migration.md](../style/migration.md)
