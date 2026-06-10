# style/coding_style — M5HAL v1 コーディング規約

M5HAL v1 のコード記述に関する規約をまとめる。 全体構造は [../architecture.md](../architecture.md) を参照。

## C++ 規格

- **C++17 を基本** とする (`platformio.ini` で `-std=gnu++17` / `-std=c++17` 指定)
- C++20 / C++23 機能は組み込み環境の polyfill 状況を確認した上で導入する。 `m5::stl::expected` 等は M5Utility 経由で提供されるものを利用する
- RTTI / 例外には依存しない。 `dynamic_cast` / `typeid` / `try`-`catch` を要件とする設計は採用しない

## コードフォーマット

- リポジトリルートの `.clang-format` に従う
- フォーマットは `clang-format` で自動適用する (運用詳細は [../verification.md](../verification.md) を参照)
- 個人の好みでフォーマット規則を変更しない。 変更が必要な場合は議論を経て `.clang-format` 自体を更新する

## namespace 運用

- すべての公開シンボルは `m5::hal::` 配下に配置する
- ディレクトリ階層と namespace 階層を一致させる (詳細は [../reference/directory-layout.md](../reference/directory-layout.md) を参照)
- ライブラリ内部のコードでは **フルパス記述を基本** とする
- 利用者向けに `using namespace m5::hal;` の使用は非推奨とする
- 長さが辛い場合は **namespace alias** を許容する (`namespace i2c = m5::hal::i2c;` 等)

### namespace 宣言形式

C++17 **nested namespace specifier** で宣言する:

```cpp
namespace m5::variants::frameworks::arduino::hal::v1::i2c {
using namespace ::m5::hal::v1;  // resolve unqualified types::/bus:: refs
...
}  // namespace m5::variants::frameworks::arduino::hal::v1::i2c
```

- 旧来の `namespace m5 { namespace ... { ... } }` の入れ子は使わない
- closing brace のコメントは fully qualified namespace パスを書く
- 例外として `_macro/offer_all.inl` は 1 行ネスト形式を許容する

## 命名規則

| 対象 | スタイル | 例 |
|---|---|---|
| クラス・構造体 | UpperCamelCase | `BusConfig`, `I2CMasterAccessor` |
| public 関数・メソッド | lowerCamelCase | `beginAccess`, `getBusKind` |
| **private / protected メンバ関数** | **先頭アンダースコア + lowerCamelCase** | `_writePinEncoded`, `_fromLocalPin` |
| public メンバ変数 | lowerCamelCase または snake_case (既存コードに揃える) | `freq`, `i2c_addr` |
| **private / protected メンバ変数** | **先頭アンダースコア + lower_snake_case** | `_bus`, `_access_config` |
| ローカル変数・引数 | lower_snake_case | `delay_cycle`, `cb_obj` |
| 定数 (constexpr) | 用途に応じる (`kCamelCase` または `UPPER_SNAKE_CASE`) | `kDefaultTimeoutMs` |
| enum class 値 | **基本** UpperCamelCase。 **例外として** エラーコード等 C 互換 / POSIX 慣習を意識する定数群は UPPER_SNAKE_CASE 許容 | `BusKind::I2C`, `GpioMode::Output` / 許容例: `error_t::OK`, `error_t::I2C_NO_ACK` |
| 機能・設定マクロ | `M5HAL_` プレフィックス (アンダースコアなし) + UPPER_SNAKE_CASE。 世代間で値が異なり得るものは `M5HAL_V1_` で世代分離 | `M5HAL_V1_TARGET_PLATFORM_NUMBER`, `M5HAL_ASSERT` |
| ヘッダガード | `M5_HAL_<PATH>_HPP` (`M5_HAL_` = アンダースコアあり、 パスベース) | `M5_HAL_TYPES_HPP`, `M5_HAL_GPIO_GROUP_HPP_` |

### 先頭アンダースコア方式の制約

private / protected メンバ (**変数・関数の両方**) は先頭アンダースコア方式 (`_member`) を採用するが、 C++ 規格上の予約識別子との衝突を避けるため、 以下を厳守する。

- **必ず小文字始まり** にする (`_foo` のみ可)
  - `_Foo` / `_FOO` (先頭アンダースコア + 大文字始まり) は **C++ 規格上どのスコープでも予約** されている。 使用禁止
- **連続アンダースコア禁止** (`__foo` / `foo__bar` 等)
  - 連続アンダースコアは実装側マクロ (`__GNUC__`, `__cplusplus` 等) と衝突する。 使用禁止
- **public メンバには使わない**
  - 接頭アンダースコアは private / protected メンバ専用の目印とする
  - public メンバはそのまま `foo` で書く
- **コンストラクタ / デストラクタは対象外**
  - クラス名と同名なので接頭アンダースコアを付けない (ctor/dtor は命名規則の一般則に従う)
- **メンバ関数のスタイルは lowerCamelCase + 先頭 `_`** (例: `_writePinEncoded`)、 **メンバ変数のスタイルは lower_snake_case + 先頭 `_`** (例: `_encoded_num`) で、 それぞれ public のスタイルに `_` プレフィックスを足した形

## 動詞規約

通信バス API に出てくる動詞は、 役割ごとに使い分ける。 新 API で使う動詞は以下に限定する。

| 動詞ペア | 用途 | 例 |
|---|---|---|
| `init` / `release` | オブジェクトのライフサイクル | `Bus::init(BusConfig)`, `Bus::release()` |
| `attach` / (`detach`) | 外部 native handle との紐付け | `Bus::attach(TwoWire&)` |
| `lock` / `unlock` | Bus 排他制御 (引数 `Accessor*` 必須) | `Bus::lock(Accessor*, timeout)`, `Bus::unlock(Accessor*)` |
| `beginAccess` / `endAccess` | Accessor アクセス期間 (depth counter で nest 対応) | `Accessor::beginAccess(timeout)`, `Accessor::endAccess()` |
| `beginTransaction` / `endTransaction` | kind 固有 transaction 期間。SPI では CS assert/deassert 区間 | `SPIMasterAccessor::beginTransaction()`, `SPIMasterAccessor::endTransaction()` |
| `transfer` | atomic な I/O 動作 | `Bus::transfer(...)`, `Accessor::transfer(...)` |
| `read` / `write` | Accessor の利用者 sugar | `Accessor::write(tx)` |
| `peek` / `advance` | Source: 借用 Span 取得 / cursor 前進 (連続 peek は monotonic non-decreasing 冪等) | `Source::peek(max_len)`, `Source::advance(N)` |
| `reserve` / `commit` | Sink: 書き込み領域取得 / 書き込み量報告 (契約違反は UB、 派生実装の単純化を優先) | `Sink::reserve(max_len)`, `Sink::commit(N)` |
| `eof` / `closed` | 終端 final state の問い合わせ (Source 側 / Sink 側) | `Source::eof()`, `Sink::closed()` |

RAII 型:

- `ScopedAccess` — Accessor の `beginAccess` / `endAccess` を RAII で
- `ScopedLock` — Bus の `lock` / `unlock` を RAII で (引数は `Accessor*` 必須)

### 開始/終了動詞の使い分け原則

- `init` / `release`: lifecycle 全体 (構築後 1 回 ↔ 破棄前 1 回)
- `attach` / `detach`: 外部の物との接続/切断 (M5HAL が所有しない handle と紐付ける)
- `lock` / `unlock`: 排他のみ (mutex 風)
- `begin*` / `end*`: scope (RAII 風期間) の開始 / 終了
- `transfer`: 1 アクションで完結する atomic I/O
- `peek` / `advance`: cursor 操作 (peek は冪等な lookahead、 advance は副作用ある cursor 前進)
- `reserve` / `commit`: 書き込み領域の借用と確定 (transactional)

### 新 API で使わない動詞

以下は v1 では使用を **禁止** する:

- `start*` / `stop*` (例: `startWrite`, `startRead`, `stop`) — chain 中間状態を持つ旧設計の遺物、 atomic な `transfer` で代替
- `Bus::beginAccess(AccessConfig&)` / `endAccess(Accessor*)` — 旧 factory + lock 混在 API、 利用者が Accessor を直接構築する方式に変更

責務分離の詳細は [../design/bus_accessor.md](../design/bus_accessor.md)。

## 型選択

組み込み環境での移植性を優先し、 サイズが明確な型を基本とする。

| 用途 | 推奨型 |
|---|---|
| バッファサイズ・配列インデックス | `size_t` |
| エラーコード | `error::error_t` (M5HAL 自前定義) |
| 構造体メンバ (固定幅・メモリ効率重視) | `int8_t` / `int16_t` / `int32_t` / `uint8_t` 等 |
| 関数引数・ローカル変数 (演算速度重視) | `int_fast16_t` / `int_fast32_t` |
| 真偽値 | `bool` |

`int` のような実装依存サイズの型は避ける。

## ファイル構成

- ヘッダ拡張子: `.hpp` (C++ 専用) / `.h` (C 互換も意識する場合)
- 実装ファイル: `.cpp` または `.inl` (テンプレート / インライン実装の分離用)
- ヘッダには宣言と簡単な inline 実装のみ。 複雑な実装は別ファイルに分離する

### include 規則

- 同一ライブラリ内のヘッダは **ダブルクォート** + 相対パスで include する
  - 例: `#include "../error.hpp"`
- 標準ライブラリ・外部ライブラリ (M5Utility 等) は **山括弧** で include する
  - 例: `#include <M5Utility.hpp>`, `#include <stdint.h>`
- include search path 前提の絶対パス的記述 (例: `#include "m5_hal/hal/error.hpp"`) は避ける

## マクロ

M5HAL のマクロは用途で 2 系統に分かれる (実態に基づく規約):

- **機能・設定マクロは `M5HAL_` プレフィックス** (アンダースコアなし) + UPPER_SNAKE_CASE — 機能フラグ・外部定義の上書き・assert 等。 例: `M5HAL_FRAMEWORK_HAS_*`, `M5HAL_V1_PLATFORM_NUMBER_*`, `M5HAL_VARIANT_CURRENT_*`, `M5HAL_ASSERT`。 世代間で値が異なり得るものは `M5HAL_V1_` で世代分離する (無印は凍結 v0 が所有。 [../design/v0_v1_coexistence.md](../design/v0_v1_coexistence.md) §制約)
- **ヘッダガードは `M5_HAL_<PATH>_HPP` プレフィックス** (`M5_HAL_` = アンダースコアあり、 ファイルパスベース) — 機能マクロの `M5HAL_` と区別する。 例: `M5_HAL_TYPES_HPP`, `M5_HAL_GPIO_GROUP_HPP_`, `M5_HAL_ASSERT_HPP`
- 内部用途のマクロは末尾アンダースコアを付けて区別する (例: `M5HAL_VARIANT_CURRENT_*_`)

## variant 機構の規則

variant 機構の実装に関する規則。 全体像は [../design/variants.md](../design/variants.md)。

### `_offer.hpp` (capability 申告ファイル)

- **include guard / `#pragma once` を持たない** — `_macro/offer_all.inl` から繰り返し include される前提
- マクロ宣言のみで構成 (型 / template / constexpr は使わない)
- 宣言する内部マクロは全て `M5HAL_VARIANT_CURRENT_*_` 形式 (末尾アンダースコア):
  - `M5HAL_VARIANT_CURRENT_ALIAS_`
  - `M5HAL_VARIANT_CURRENT_BASE_PATH_` (現状 reserved、 未使用)
  - `M5HAL_VARIANT_CURRENT_BASE_NS_`
  - `M5HAL_VARIANT_CURRENT_HAS_HAL_<KIND>_`
- これらのマクロは `offer_all.inl` 側で消費後に undef される。 `_offer.hpp` 自身では undef しない
- ファイル全体を `// clang-format off` / `// clang-format on` で囲む
- 詳細は [../verification.md](../verification.md) の `_offer.hpp` に関する注意を参照

### `*.inl` ファイル (re-include 前提)

- `_macro/offer_all.inl` のように **繰り返し include される前提** のファイルは include guard を持たない
- ファイル冒頭にコメントで「re-include 前提」 を明示する
- 各 `*.inl` の責務 (どのマクロを消費し、 どの alias を生成するか) もコメントで明示する

### 抽象基底命名

抽象基底の命名は **具象 (variant 側) と衝突しない形を選ぶ**。 v1 では 2 パターンを許容する:

- **`I` プレフィックス採用** — 抽象基底と具象が同じ namespace 内で同じ概念名を共有しやすい場合に採用する。 例:
  - `m5::hal::v1::gpio::IGPIO` (抽象) ↔ `m5::variants::frameworks::arduino::hal::v1::gpio::GPIO` (具象)
  - `m5::hal::v1::gpio::IPort` (抽象) ↔ 同 namespace 内の `Port` (具象)
- **プレフィックスなし** — 具象が kind / role プレフィックスで自然に区別できる場合に採用する。 例:
  - `m5::hal::v1::bus::Bus` (抽象) ↔ `m5::hal::v1::i2c::I2CBus` (具象、 kind プレフィックスで区別)
  - `m5::hal::v1::bus::Accessor` (抽象) ↔ `I2CMasterAccessor` / `SPIMasterAccessor` (具象)
  - `m5::hal::v1::data::Source` / `Sink` (抽象) ↔ `MemorySource` / `LimitedSink` (具象、 役割プレフィックス)

既存抽象基底名 (`Pin` / `Port` / `GPIO` / `Source` / `Sink` / `Bus` / `Accessor` / `Input` / `Output` 等) は維持する。 新規抽象基底を導入する場合は本原則 (具象との衝突回避) に従ってどちらかのパターンを選択する。

### variant 名前空間

- variant 具象は `m5::variants::{frameworks,platforms}::<name>[::<chip>]::hal::v1::*` 配下に配置 (variant は層横断のメタ階層、 各 variant 内の `hal::v1::` が HAL 層への提供物)
- ディレクトリ階層と一致 (例: `src/m5_hal/variants/frameworks/arduino/hal/i2c/i2c.hpp` 内の宣言は `m5::variants::frameworks::arduino::hal::v1::i2c::*`)
- variant 内では必要に応じて `using namespace ::m5::hal::v1;` を置ける
- `m5::hal::v1::types::*` は `::m5::hal::v1::types::*` の fully qualified 形式で書く
- `using namespace` による flat 注入は `_macro/offer_all.inl` のみで行う

## アサーション・ロギング

- **contract 違反**は `M5HAL_ASSERT(cond, fmt, ...)` で検出する (`hal/v1/assert.hpp`)。 fmt は printf 形式。 debug (NDEBUG 未定義) は診断出力 + abort、 release (NDEBUG) は cond を評価せず no-op (= UB 流儀、 [../design/gpio.md](../design/gpio.md) §契約ベース)。 recoverable なエラーは `expected` で返し、 assert と使い分ける
- assert の出力は **log level 非依存** (黙って死なせない)。 format は M5Utility の M5_LIB_LOG を踏襲 (`[A][file:line] func(): msg`、 native の gtest death test は stderr にマッチ)。 `M5_LIB_LOGE` 自体は log level None (デフォルト) で silent のため assert には使わない
- contract 違反以外のロギングは M5Utility の `M5_LIB_LOGE / W / I / D / V` を使う

## コメント

コメントは 2 系統に分けて運用する。 用途・対象読者・抽出範囲が異なるため、 形式を意図的に使い分ける。

| 用途 | 形式 | 言語 | 対象読者 |
|---|---|---|---|
| 公開 API ドキュメント | `/*! ... */` ブロック (Doxygen) | **英語必須** | ライブラリ利用者 (生成 doc を読む) |
| 設計意図 / why / workaround | `// ...` (通常コメント) | 英語推奨、 日本語混在許容 | メンテナ (ソースを読む) |

### 言語ポリシー

- **公開 API ドキュメント (Doxygen) は英語で書く** — Doxygen 生成物がエンドユーザー向けリファレンスになるため
- **設計意図 / why コメントは英語推奨、 日本語混在許容** — 段階移行中。 新規 / 改変箇所は英語で書く (boy scout rule)
- 既存日本語コメントはファイル単位で英訳していく。 1 commit で全ファイル一括書き換えはしない (差分把握 / レビュー困難を避ける)
- **訳語のブレを防ぐため [glossary.md](glossary.md) を併読** すること。 主要概念 (bus / accessor / access window / contract violation 等) の英訳は固定済

### Doxygen の最低限ライン

公開 API ヘッダの以下に Doxygen コメントを書く。 形式は `/*!` ブロック (M5Stack 公式ライブラリ群と整合)。

| 対象 | 必須 | 任意 |
|---|---|---|
| 公開 namespace | `@brief` 1 行 | — |
| 公開 class / struct | `@brief` | detailed description |
| public method | `@brief` | `@param` (引数あり時)、 `@return` / `@retval` (戻り値あり時) |
| public enum 値 | inline `///<` 1 行 | — |
| private / protected / static helper / inline detail | **対象外** (`//` で書く) | — |

書き方の原則:

- `@brief` は **1 行 (80 文字以下目安)**。 詳細説明が必要なら空行 1 行を挟んで続ける
- **コードで自明な内容は書かない** — `@return The result.` 等の冗長な記述は禁止
- **why は doxygen の対象外**、 通常コメント (`//`) 側に書く。 生成 API doc には実装意図を含めない
- `@param` には引数の意味 / 制約 / 単位を書く (型は宣言から自明)
- `@return` には戻り値の意味、 成功 / 失敗条件、 範囲を書く
- `expected<T, E>` を返す関数では `@retval` で代表的エラーを列挙してよい

例:

```cpp
/*!
  @brief Master-side accessor for an I2C bus.

  Owns the per-target configuration (slave address, frequency, timeout) and
  serializes transfers against the underlying bus via lock/unlock.
 */
struct I2CMasterAccessor : public bus::Accessor {
    /*!
      @brief Issue a single I2C transfer (combined write/read).
      @param desc      Per-transfer flags (start/restart/stop, prefix bytes)
      @param tx_bytes  Bytes to transmit. Empty span = read-only transfer
      @param rx_bytes  Buffer for received bytes. Empty span = write-only transfer
      @return Number of bytes actually transferred, or error code
     */
    m5::stl::expected<size_t, error::error_t> transfer(const TransferDesc& desc,
                                                       data::ConstDataSpan tx_bytes,
                                                       data::DataSpan rx_bytes);
};
```

### 通常コメント (`//`) の書き方

- **why を中心に書く**。 what はコードで表現する
- 公開 API ヘッダ内でも、 private / protected メンバ、 inline detail、 workaround の説明には `/*!` ではなく `//` を使う
- 新規・改変分は英語で書くが、 既存日本語コメントを保つことを優先 (段階移行)

### TODO コメント

- `// TODO(@user): ...` 形式とする
- Doxygen の `@todo` は使わない (生成 API doc に出さない)
- `src/m5_hal/hal/v0/` 配下の既存 TODO は freeze 例外とし、 取り込み元との差分を維持するため、 現行 v1 作業のタスク対象外として扱う

### Doxyfile

Doxyfile 自体は将来整備する。 整備時は以下を満たす設定とする:

- `JAVADOC_AUTOBRIEF = NO` — 明示的に `@brief` を要求
- `EXTRACT_PRIVATE = NO` / `EXTRACT_STATIC = NO` — 公開 API のみを生成 doc に出す
- `INPUT` は `src/m5_hal/hal/v1/` および公開 variant 群に限定 (`_macro/` 等のメタは除外)
