# style/glossary — M5HAL 英語用語集

公開コードコメント (Doxygen + `//`) を英訳する際の用語の固定。 同じ日本語概念が複数の英語訳で書かれてブレることを防ぐためのリファレンス。

スコープ:

- 本ファイルは **コード内コメントで使う訳語の指針**。 v1 識別子は英語コメントと整合する命名に統一済 (`bus_kind_t` / `getBusKind` / `BusKind` 等)。 新規ファイル / 改変箇所は本 glossary に揃えて書く
- 仕様文書 (`spec/`) はバイリンガル運用 (日本語が正本) を継続。 本ファイルは「コードコメント英訳時に揺れないため」 の用途
- 規約全体は [coding_style.md](coding_style.md) §コメント

## バス通信 (bus communication)

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| 通信バス | bus | コード上 `Bus` |
| バス種別 | bus kind | v1 では識別子も `bus_kind_t` / `BusKind` / `getBusKind()` で統一済 (旧 `bus_type_t` 等は v0 のみ) |
| 通信相手 / アクセス対象 | accessor | コード上 `Accessor`、 固有名詞扱い |
| 通信本体 (atomic I/O) | transfer | 動詞・名詞共通 |
| 1 回の transfer に付随するメタ情報 | per-call transfer metadata | `TransferDesc` の説明 |
| 接頭バイト列 | prefix bytes | I2C/SPI の register address 等 |
| 排他制御 | mutual exclusion / locking | `Bus::lock` / `Bus::unlock` |
| 同時に 1 owner のみ保有可能 | exclusive | "exclusive ownership" |
| トランザクション (SPI CS assert 区間) | transaction | SPI 専用、 `begin/endTransaction` |
| アクセス期間 | access window | `begin/endAccess` 間。 "scope" でも可だが C++ scope と紛らわしい |
| 入れ子の access | nested access | `Accessor::beginAccess` を再帰的に呼ぶ場合 |
| depth counter による吸収 | absorbed via depth counter | nested access の自然な扱い |
| 再 lock / 二重 lock | re-lock / re-acquire | |
| アクセス権限の所有者 | lock owner | コード上 `_lock_owner` |
| タイムアウト (lock 取得待ち) | acquisition timeout | lock 系 API の `timeout_ms` 引数 (省略 = `types::TIMEOUT_FOREVER` = 無限待ち、 0 = 即時 try-lock) |
| runtime 設備 (time / mutex) | runtime kind | `m5::hal::v1::runtime` ([design/runtime.md](../design/runtime.md))。 bus 構造を持たない設備 kind |
| 非再帰 (mutex) | non-recursive | 保有タスク自身の再 lock も timeout まで待って失敗 |

## Source / Sink (stream I/O)

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| 借用 Span | borrowed span | `peek` / `reserve` の戻り値の有効期間 |
| 借用期間 | borrow lifetime | 「次の `peek` または `advance` まで有効」 |
| cursor 前進 | advance the cursor | `Source::advance(N)` |
| cursor 操作 | cursor operation | |
| 連続 peek の冪等性 | idempotent across consecutive calls | `peek` の monotonic non-decreasing 性質 |
| 単調非減少 | monotonic non-decreasing | `peek(max_len)` の戻り長 |
| 終端 (Source) | end-of-stream | `eof() == true` |
| 終端 (Sink) | closed | `closed() == true` |
| 書き込み領域取得 | reserve a write region | `Sink::reserve` |
| 書き込み量報告 | commit the written length | `Sink::commit` |
| 書き込み量と reserve の対称性 | transactional reserve / commit pair | |
| 契約違反 | contract violation | undefined behavior in release |
| 復帰可能エラー | recoverable error | returned via `expected<T, E>` |
| stream 通信 | streaming transport | UART 等の将来用途 |

## エラー / 契約 (error / contract)

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| 契約違反 | contract violation | `M5HAL_ASSERT` の対象 |
| 復帰可能エラー | recoverable error | `expected` で返す |
| エラーコード | error code | `error::error_t` |
| 成功 / 失敗 | success / failure | `has_value()` での判定 |
| 未実装 | not implemented | `error_t::NOT_IMPLEMENTED` |
| 不正引数 | invalid argument | `error_t::INVALID_ARGUMENT` |
| ビジー | busy | `error_t::BUSY` |
| I/O エラー | I/O error | `error_t::IO_ERROR` |
| 閉じている | closed | `error_t::CLOSED` |
| 資源不足 | out of resource | `error_t::OUT_OF_RESOURCE` |
| バッファオーバーフロー | buffer overflow | `error_t::BUFFER_OVERFLOW` |
| バッファアンダーフロー | buffer underflow | `error_t::BUFFER_UNDERFLOW` |
| stream 終端 | end-of-stream | `error_t::END_OF_STREAM` |
| checksum 不一致 | checksum error | `error_t::CHECKSUM_ERROR` |
| protocol 違反 | protocol error | `error_t::PROTOCOL_ERROR` |
| 切断 | disconnected | `error_t::DISCONNECTED` (remote transport 喪失) |
| リモート内部エラー | remote fault | `error_t::REMOTE_FAULT` (未知のリモートコードの写像先含む) |
| 非対応 | unsupported | `error_t::UNSUPPORTED` (リモートが当該機能を持たない) |
| 短絡終了 / early return | early exit / short-circuit return | エラー時の処理中断 |
| no-op (release) | no-op in release builds | `M5HAL_ASSERT` の release 挙動 |
| sentinel (番兵) | sentinel | `_lock_owner = nullptr` 等 |

## 型 / 識別子 (type / identifier)

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| 派生クラス | derived class | |
| 抽象基底 | abstract base | |
| マーカ基底 (vtable なし) | tag-only base / marker base | `TransferDesc` 等 |
| 種別タグ | kind tag | `bus_kind_t` を保持するフィールド (`BusConfig::bus_kind` 等) |
| 共変戻り型 | covariant return type | `getConfig()` の派生 override |
| 単一情報源 | single source of truth | "kind は X が握る" の意 |
| アップキャスト | upcast | `static_cast<Base&>(derived)` |
| ダウンキャスト | downcast | RTTI 非依存の static downcast |
| 派生 ctor で base に渡す | forwarded through the derived ctor | |
| SFINAE 制約 | SFINAE constraint | |
| unsigned integral 型 | unsigned integral type | register sugar の引数制約 |

## GPIO

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| ポート | port | `IPort` 抽象 |
| 個別ピン | pin | `IPin` |
| 統合 GPIO 番号 | unified GPIO number | `gpio_number_t` |
| ローカル pin | local pin | `IPort` 内の通し番号 (0〜255) |
| slot | slot | `GPIOGroup` の slot 0..N、 固有名詞扱い |
| MCU GPIO | MCU GPIO | 「slot 0 は MCU GPIO 予約」 |
| I/O expander | I/O expander | PCA9554 等 |
| pin の合成・分解 | pin encoding / decoding | ビット演算で組み立て・分解 |
| リテラル直書き禁止 | literal construction is discouraged | `gpio_number_t{0x0123}` |

## レイヤ構造 (architecture)

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| HAL | HAL | 固有名詞、 大文字 |
| variant | variant | 固有名詞、 小文字 |
| framework variant | framework variant | arduino / espidf / software 等 |
| platform variant | platform variant | esp32 / esp32s3 等 |
| 抽象階層 | abstract layer | `m5::hal::v1::` |
| 具象階層 | concrete layer | `m5::variants::frameworks::*` |
| 勝者バインド (winner alias) | winner binding via type aliases (runtime kind のみ `using namespace`) | `_macro/offer_all.inl` |
| capability 申告 | capability declaration | `_offer.hpp` |
| re-include 前提 | re-includable | `*.inl` の意 |

## v0 / v1 共存

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| v0 / v1 | v0 / v1 (= API generation) | small letters |
| inline namespace 切替 | inline namespace switch | `M5HAL_INLINE_V0` / `M5HAL_INLINE_V1` |
| 上書き (override) | override | C++ override と意味同じ |
| 下位互換 | backward compatibility | v0 API の継続提供 |

## 説明文の書き出し (Doxygen `@brief`)

ブレやすいので、 よく使う書き出しパターンを 1 つに固定する。

| 用途 | 推奨形 |
|---|---|
| 抽象基底の説明 | "Abstract base for ..." |
| 設定構造体 | "Configuration for ..." / "Per-call descriptor for ..." |
| アクセサ | "Master-side accessor for ..." / "Slave-side accessor for ..." |
| RAII ヘルパ | "RAII helper that wraps ..." |
| メソッド (動作系) | "Issue a single ..." / "Begin a ... window" / "End the ... window" |
| メソッド (取得系) | "Return the ..." / "Return a reference to the ..." |
| メソッド (判定系) | "Check whether ..." / "Return whether ..." |

## 避ける表現

- "type" を bus 種別の意味で使う (v1 は識別子も `bus_kind_t` / `getBusKind` に統一済、 旧 `bus_type_t` 等は v0 専用)
- "scope" を access window の意味で使う (C++ scope と紛らわしい)
- "method" と "function" の混用 — class メンバは "method"、 freestanding は "function"
- "the data" / "the value" / "the result" のような無情報な表現を `@param` / `@return` に書く
- 直訳調の "this function" / "this class" — Doxygen は対象が自明なので冒頭から動詞 / 名詞で始める

## 未確定 / 議論余地

- `peek` / `reserve` の借用 Span を "view" と呼ぶか "borrowed span" と呼ぶか → **borrowed span** で固定 (現状コードコメントとの連続性、 "view" は `std::string_view` 連想で誤解の余地)
- `transfer` を動詞として "perform a transfer" にするか単に "transfer" にするか → **issue a transfer** / **perform a transfer** の 2 系統許容
- access window vs access session → **access window** で固定 (session は network 連想で誤解の余地)
