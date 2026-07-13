# style/glossary — M5HAL 英語用語集

> **読者**: メンテナ向け（ビルド・運用・規約）。

公開コードコメント (Doxygen + `//`) を英訳する際の用語の固定。 同じ日本語概念が複数の英語訳で書かれてブレることを防ぐためのリファレンス。

スコープ:

- 本ファイルは **コード内コメントで使う訳語の指針**。 v2 識別子は英語コメントと整合する命名に統一済 (`bus_kind_t` / `getBusKind` / `BusKind` 等)。 新規ファイル / 改変箇所は本 glossary に揃えて書く
- 仕様文書 (`spec/`) はバイリンガル運用 (日本語が正本) を継続。 本ファイルは「コードコメント英訳時に揺れないため」 の用途
- 規約全体は [coding_style.md](coding_style.md) §コメント

## バス通信 (bus communication)

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| 通信バス | bus | コード上 `Bus` |
| バスインターンレジストリ | bus registry | `M5_Hal` が全 kind の bus を配線 (identity) で intern する weak registry。寿命は返された strong owner が決める ([design/bus_accessor.md](../design/bus_accessor.md)) |
| 共有取得 | shared acquisition / acquire | `M5_Hal.<kind>.acquire(cfg)` が所有権を持つ共有ハンドル (`shared_ptr<IBus>`) を返す。直接構築はエスケープ |
| 共同所有 | co-own | `shared_ptr` からアクセサを構築するとアクセサがバスの生存を共有保持する (`Accessor dev{handle, cfg}`) |
| 明示解放 | explicit release / consuming close | `result_t<void> release(std::shared_ptr<IBus>& bus)`。exact instance + sole owner のときだけ成功し、caller の handle を消費する |
| 自然解放 | natural release / final-holder destruction | 最後の strong owner が消えたときの bus dtor 起点の解放 |
| 隔離済み墓標 | quarantined tombstone | 外部解放を確認できず、identity / remote bus ID の再利用を止める registry 状態 |
| バス種別 | bus kind | v2 では識別子も `bus_kind_t` / `BusKind` / `getBusKind()` で統一済 (旧 `bus_type_t` 等は v0 のみ) |
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
| runtime 設備 (time / mutex) | runtime kind | `m5::hal::v2::runtime` ([design/runtime.md](../design/runtime.md))。 bus 構造を持たない設備 kind |
| 非再帰 (mutex) | non-recursive | 保有タスク自身の再 lock も timeout まで待って失敗 |
| セッションゲート | session gate / operation gate | 1 `RemoteSession` の complete RPC を直列化する canonical mutex。bus/channel → session の順で取得 |

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
| 短絡終了 / early return | early exit / short-circuit return | エラー時の処理中断 |
| no-op (release) | no-op in release builds | `M5HAL_ASSERT` の release 挙動 |
| sentinel (番兵) | sentinel | `_lock_owner = nullptr` 等 |

`error_t` 列挙値 (NOT_IMPLEMENTED / BUSY / IO_ERROR 等) の英語名一覧は [design/errors.md](../design/errors.md) §toString (ログ用の名前) を参照。

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

### 不採用の理由 (rejected alternatives)

- `view` は不採用 — `std::string_view` を連想させる (accessor はデータの view ではなくバス上の操作主体。`shared_ptr` から構築すると対象バスを co-own してその生存を担保する)
- `session` は不採用 — ネットワーキングの session を連想させる (排他期間は access window と呼ぶ)
- transfer の動詞形は `issue a transfer` / `perform a transfer` の 2 形を許容する

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
| 抽象階層 | abstract layer | `m5::hal::v2::` |
| 具象階層 | concrete layer | `m5::variants::frameworks::*` |
| 勝者バインド (winner alias) | winner binding via type aliases (runtime kind のみ `using namespace`) | `_macro/offer_all.inl` |
| capability 申告 | capability declaration | `_offer.hpp` |
| re-include 前提 | re-includable | `*.inl` の意 |

## v0 / v2 共存

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| v0 / v2 | v0 / v2 (= API generation) | small letters |
| inline namespace 切替 | inline namespace switch | 利用者設定は`M5HAL_V0_INLINE` / `M5HAL_V2_INLINE`。展開用の`M5HAL_INLINE_V0` / `M5HAL_INLINE_V2`は内部macro |
| 上書き (override) | override | C++ override と意味同じ |
| 下位互換 | backward compatibility | v0 API の継続提供 |

## コメント規約 (`@brief` 書き出し / 避ける表現)

詳細は [coding_style.md](coding_style.md) §コメント を参照。
