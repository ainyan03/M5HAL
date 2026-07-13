# style/coding_style — M5HAL v2 コーディング規約

> **読者**: メンテナ向け（ビルド・運用・規約）。

M5HAL v2 のコード記述に関する規約をまとめる。 全体構造は [../architecture.md](../architecture.md) を参照。

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
namespace m5::variants::frameworks::software::hal::v2::i2c {
using namespace ::m5::hal::v2;  // resolve unqualified types::/bus:: refs
...
}  // namespace m5::variants::frameworks::software::hal::v2::i2c
```

- 旧来の `namespace m5 { namespace ... { ... } }` の入れ子は使わない
- closing brace のコメントは fully qualified namespace パスを書く
- 例外として `_macro/offer_all.inl` / `_macro/offer_kind.inl` は 1 行ネスト形式を許容する

## 命名規則

| 対象 | スタイル | 例 |
|---|---|---|
| クラス・構造体 | UpperCamelCase | `BusConfig`, `MasterAccessor` |
| public 関数・メソッド | lowerCamelCase | `beginAccess`, `getBusKind` |
| **private / protected メンバ関数** | **先頭アンダースコア + lowerCamelCase** | `_writePinEncoded`, `_fromLocalPin` |
| public メンバ変数 | lowerCamelCase または snake_case (既存コードに揃える) | `freq`, `i2c_addr` |
| **private / protected メンバ変数** | **先頭アンダースコア + lower_snake_case** | `_bus`, `_access_config` |
| ローカル変数・引数 | lower_snake_case | `delay_cycle`, `cb_obj` |
| 定数 (constexpr) | 用途に応じる (`kCamelCase` または `UPPER_SNAKE_CASE`) | `kDefaultTimeoutMs` |
| enum class 値 | **基本** UpperCamelCase。 **例外として** エラーコード等 C 互換 / POSIX 慣習を意識する定数群は UPPER_SNAKE_CASE 許容 | `BusKind::I2C`, `GpioMode::Output` / 許容例: `error_t::OK`, `error_t::I2C_NO_ACK` |
| 機能・設定マクロ | `M5HAL_` プレフィックス (アンダースコアなし) + UPPER_SNAKE_CASE。 世代間で値が異なり得るものは `M5HAL_V2_` で世代分離 | `M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID`, `M5HAL_ASSERT` |
| ヘッダガード | `M5_HAL_<PATH>_HPP` (`M5_HAL_` = アンダースコアあり、 パスベース) | `M5_HAL_TYPES_HPP`, `M5_HAL_GPIO_GROUP_HPP_` |

> **enum 値の補足 (acronym の扱い)**: UpperCamelCase 化では一般的な略語は **語として扱う** (`Gpio` / `Io` / `Dc` / `Tx` / `Rx` → 例 `SpiDataMode::DualIo`, `Channel::Tx`, `SpiDataMode::HalfDuplexWithDcPin`)。 一方、 確立したプロトコル名 / 業界表記は **大文字を保つ** (`BusKind::I2C` / `SPI` / `I2S` / `UART`)。
>
> **UPPER_SNAKE を enum 値の基本にしない理由**: `INPUT` / `OUTPUT` / `HIGH` / `LOW` 等は Arduino がマクロ定義しており、 enum 値名に使うとプリプロセッサ衝突でビルドが壊れる (これが `GpioMode` を `Input` / `Output` にしている理由)。 全 enum を安全に統一できるのは UpperCamelCase だけ。 error code 群のみ errno / ESP_ERR 慣習で UPPER_SNAKE を例外許容する (`error_t`)。

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
| `init` / `release` | オブジェクトのライフサイクル | `Bus::init(BusConfig)`, `IBus::release()` |
| `attach` / (`detach`) | 外部 native handle との紐付け | `Bus::attach(TwoWire&)` |
| `lock` / `unlock` | Bus 排他制御 (引数 `IAccessor*` 必須) | `IBus::lock(IAccessor*, timeout)`, `IBus::unlock(IAccessor*)` |
| `beginAccess` / `endAccess` | Accessor アクセス期間 (depth counter で nest 対応) | `IAccessor::beginAccess(timeout)`, `IAccessor::endAccess()` |
| `beginTransaction` / `endTransaction` | kind 固有 transaction 期間。SPI では CS assert/deassert 区間 | `MasterAccessor::beginTransaction()`, `MasterAccessor::endTransaction()` |
| `transfer` | atomic な I/O 動作 | `IBus::transfer(...)`, `MasterAccessor::transfer(...)` |
| `read` / `write` | Accessor の利用者 sugar | `MasterAccessor::write(tx)` |
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

以下は v2 では使用を **禁止** する:

- `start*` / `stop*` (例: `startWrite`, `startRead`, `stop`) — chain 中間状態を持つ旧設計の遺物、 atomic な `transfer` で代替
- `Bus::beginAccess(AccessConfig&)` / `endAccess(Accessor*)` — 旧 factory + lock 混在 API、 利用者が Accessor を直接構築する方式に変更

責務分離の詳細は [../design/bus_accessor.md](../design/bus_accessor.md)。

## 型選択

組み込み環境での移植性を優先し、 サイズが明確な型を基本とする。

| 用途 | 推奨型 |
|---|---|
| バッファサイズ・配列インデックス | `size_t` |
| エラーコード | `error::error_t` (M5HAL 自前定義) |
| **API 戻り値 (値 or エラー)** | **`result_t<T>`** (`m5::hal::v2` 直下、 定義 = `hal/v2/error.hpp`)。 `m5::stl::expected<T, error::error_t>` の **pure alias** (同一型) — 派生クラスにしない (monadic 連鎖の decay、 基底↔派生変換、 将来の `std::expected` 乗り換え、 M5UU との語彙分裂を避ける)。 v2 のシグネチャは `expected` を直接綴らずこの alias を使う |
| 構造体メンバ (固定幅・メモリ効率重視) | `int8_t` / `int16_t` / `int32_t` / `uint8_t` 等 |
| 関数引数・ローカル変数 (演算速度重視) | `int_fast16_t` / `int_fast32_t` |
| 真偽値 | `bool` |

`int` のような実装依存サイズの型は避ける。

## ファイル構成

- ヘッダ拡張子: `.hpp` (C++ 専用) / `.h` (C 互換も意識する場合)
- 実装ファイル: `.cpp` または `.inl` (テンプレート / インライン実装の分離用)
- ヘッダには宣言と簡単な inline 実装のみ。 複雑な実装は別ファイルに分離する

### .hpp / .inl の分離基準

`.inl` は `src/M5HAL_v2.cpp` (variant 側は各 `hal.inl` hub 経由) から**単一 TU で 1 回だけ
コンパイル**される実装ファイルであり、ヘッダに実装を置くことは「全 includer での再コンパイル +
inline 展開」を意味する。基準:

- **`.hpp` に置いてよい実装**: ①テンプレート本体 (定義可視性が必要) ②おおむね 10 行以下の
  trivial なアクセサ・constexpr ヘルパ ③inline 化に性能上の実利がある明示的なホットパス
  (例: GPIO レジスタ直叩き層)
- **それ以外の関数本体は `.inl` へ**。`.inl` 側の定義は非 `inline` (単一 TU 前提)
- **新設 `.inl` は `M5HAL_v2.cpp` (または該当 variant の `hal.inl`) への include 登録が必須**
  (登録漏れはリンクエラーになる)
- 後方互換の forwarding ヘッダは作らない。ファイル移動時は includer を全書き換えする

### include 規則

- 同一ライブラリ内のヘッダは **ダブルクォート** + 相対パスで include する
  - 例: `#include "../error.hpp"`
- 標準ライブラリ・外部ライブラリ (M5Utility 等) は **山括弧** で include する
  - 例: `#include <M5Utility.hpp>`, `#include <stdint.h>`
- include search path 前提の絶対パス的記述 (例: `#include "m5_hal/hal/error.hpp"`) は避ける

## マクロ

M5HAL のマクロは用途で 2 系統に分かれる (実態に基づく規約):

- **機能・設定マクロは `M5HAL_` プレフィックス** (アンダースコアなし) + UPPER_SNAKE_CASE — 機能フラグ・外部定義の上書き・assert 等。 例: `M5HAL_FRAMEWORK_HAS_*`, `M5HAL_V2_VARIANT_ID_*`, `M5HAL_VARIANT_CURRENT_*`, `M5HAL_ASSERT`。 世代間で値が異なり得るものは `M5HAL_V2_` で世代分離する (無印は変更不可の v0 が所有。 [../design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §v2 実装者が破ってはならない唯一の不変条件)
- **ヘッダガードは `M5_HAL_<PATH>_HPP` プレフィックス** (`M5_HAL_` = アンダースコアあり、 ファイルパスベース) — 機能マクロの `M5HAL_` と区別する。 例: `M5_HAL_TYPES_HPP`, `M5_HAL_GPIO_GROUP_HPP_`, `M5_HAL_ASSERT_HPP`
- 内部用途のマクロは末尾アンダースコアを付けて区別する (例: `M5HAL_VARIANT_CURRENT_*_`)

## variant 機構の規則

variant 機構の実装に関する規則。 全体像は [../design/variants.md](../design/variants.md)。

共通機構は [../design/variants.md](../design/variants.md) §`_offer.hpp` (capability 自己申告) 仕様 を参照。本 kind 固有の差分のみ以下に示す。

### 抽象基底命名

**`I` プレフィックス = 直接インスタンス化しない抽象基底、 無印 = 具象** (利用者が直接インスタンス化する型)。

- `I` は pure interface に限らない — 非仮想 sugar 実装を持つ抽象基底にも使う (`gpio::IPort`、 `i2c::IBus` の probe sugar 等が前例)。 判断基準は「利用者が直接インスタンス化するか」だけ
- 共通基底と kind 基底は namespace で区別する: `bus::IBus` (kind 共通) ↔ `i2c::IBus` (kind 基底、 variant が派生する)。 同名でも namespace が異なれば別型として共存できる
- 具象例: `i2c::MasterAccessor` / `uart::TxAccessor` (kind namespace 内では kind プレフィックスを重ねない)、 `i2c::TransferDesc` (基底は `bus::ITransferDesc`)
- `Base` サフィックス方式は採用しない (二流派の併存を避ける)
- `data::Source` / `Sink` 系は歴史的例外として無印の抽象を維持する (`MemorySource` 等の具象が役割プレフィックスで区別される)

過半の利用場面を占める具象には短い別名を与えてよい (`using AccessConfig = MasterAccessConfig;` — Master が大多数のため短名を多数派に割り当てる)。

### variant 名前空間

- variant の**公開型** (`Bus_<variant>` / `BusConfig_<variant>` / gpio の `Port_<variant>` 等) は `m5::hal::v2::<kind>` 直下に variant suffix 付きで定義する ([../design/variants.md](../design/variants.md) §offer 要件。 ディレクトリ階層 1:1 規約の明示的例外 — [../architecture.md](../architecture.md) §namespace と配置)
- variant の**内部構造** (service 群・レジスタ層・固有ユーティリティ) は `m5::variants::{frameworks,platforms}::<name>[::<chip>]::hal::v2::*` 配下でディレクトリ階層と一致させる (例: `software/hal/i2c/i2c.hpp` の detail 群は `m5::variants::frameworks::software::hal::v2::i2c::detail::*`)
- **`m5::hal::v2::<kind>` namespace 内** (= variant 公開型の定義場所) では `::m5::hal::v2::` を省略し、 親・sibling namespace を短い相対名で参照する (`result_t<T>` / `bus::IAccessor` / `data::Source` / `types::backend_kind_t` 等)。 ただし **`detail::`** は sibling kind の `detail` namespace が include 経由で見えて曖昧になるため、 `::m5::hal::v2::detail::` のフル修飾を維持する
- **`m5::variants::...` namespace 内** (= variant 内部構造) では `::m5::hal::v2::` は名前探索の経路にないためフル修飾が必要。 `using namespace ::m5::hal::v2;` を置いて省略することもできる
- 勝者バインドの alias / wrapper 生成は `_macro/offer_kind.inl` (offer_all.inl の per-kind 展開) のみで行う (`using namespace` 注入は runtime kind 限定)

## アサーション・ロギング

- **contract 違反**は `M5HAL_ASSERT(cond, fmt, ...)` で検出する (`hal/v2/assert.hpp`)。 fmt は printf 形式。 debug (NDEBUG 未定義) は診断出力 + abort、 release (NDEBUG) は cond を評価せず no-op (= UB 流儀、 [../design/gpio.md](../design/gpio.md) §契約ベース)。 recoverable なエラーは `expected` で返し、 assert と使い分ける
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
struct MasterAccessor : public bus::IAccessor {
    /*!
      @brief Issue a single I2C transfer (combined write/read).
      @param desc      Per-transfer flags (start/restart/stop, prefix bytes)
      @param tx_bytes  Bytes to transmit. Empty span = read-only transfer
      @param rx_bytes  Buffer for received bytes. Empty span = write-only transfer
      @return Number of bytes actually transferred, or error code
     */
    result_t<size_t> transfer(const TransferDesc& desc, data::ConstDataSpan tx_bytes, data::DataSpan rx_bytes);
};
```

### `@brief` 定型句ロスター

API 種別ごとに canonical な書き出しを固定し、 同じ役割の型・関数が別表現で書かれてブレるのを防ぐ。 末尾の `...` には対象を補う (例: `Abstract base for an I2C bus.`)。

| API 種別 | canonical な書き出し |
|---|---|
| 抽象基底 | `Abstract base for ...` |
| 設定構造体 | `Configuration for ...` (per-call は `Per-call descriptor for ...`) |
| アクセサ | `Master-side accessor for ...` / `Slave-side accessor for ...` |
| RAII | `RAII helper that wraps ...` |
| 動作系メソッド | `Issue a single ...` / `Begin a ... window` / `End the ... window` |
| 取得系 | `Return the ...` / `Return a reference to the ...` |
| 判定系 | `Check whether ...` / `Return whether ...` |

### 避ける表現

| 避ける | 理由 / 代替 |
|---|---|
| bus kind の意味で `type` を使う | v2 は `bus_kind_t` / `getBusKind` に統一済。 `type` は v0 の語彙 |
| access window の意味で `scope` を使う | C++ の scope と衝突する。 access window と書く |
| `method` (クラスメンバ) と `function` (自由関数) を混同 | メンバは method、 自由関数は function と書き分ける |
| `@param` / `@return` の無情報な `the data` / `the value` / `the result` | 意味・制約・単位を書く (型は宣言から自明) |
| 直訳の `this function` / `this class` | 動詞 (`Issue ...`) または名詞 (`Abstract base for ...`) で始める |

### 通常コメント (`//`) の書き方

- **why を中心に書く**。 what はコードで表現する
- 公開 API ヘッダ内でも、 private / protected メンバ、 inline detail、 workaround の説明には `/*!` ではなく `//` を使う
- 新規・改変分は英語で書くが、 既存日本語コメントを保つことを優先 (段階移行)

### TODO コメント

- `// TODO(@user): ...` 形式とする
- Doxygen の `@todo` は使わない (生成 API doc に出さない)
- `src/m5_hal/hal/v0/` 配下の既存 TODO は freeze 例外とし、 取り込み元との差分を維持するため、 現行 v2 作業のタスク対象外として扱う

### Doxyfile

Doxyfile 自体は将来整備する。 整備時は以下を満たす設定とする:

- `JAVADOC_AUTOBRIEF = NO` — 明示的に `@brief` を要求
- `EXTRACT_PRIVATE = NO` / `EXTRACT_STATIC = NO` — 公開 API のみを生成 doc に出す
- `INPUT` は `src/m5_hal/hal/v2/` および公開 variant 群に限定 (`_macro/` 等のメタは除外)
