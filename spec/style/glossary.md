# style/glossary — M5HAL 英語用語集

> **読者**: メンテナ向け（ビルド・運用・規約）。

公開コードコメント(Doxygen + `//`)で使う日本語概念と推奨英語表記の検索表。
記述規約は[coding_style.md](coding_style.md) §コメントを参照する。

## バス通信 (bus communication)

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| 通信バス | bus | コード上 `Bus` |
| 資源ドメイン | resource domain | local registry / GPIO / Services / Memoryをco-ownするidentity namespace。`M5_Hal`はdefault domain入口 |
| バスインターンレジストリ | bus registry | 各ResourceDomainが全kindのbusをexact `ResourceKey`でinternするweak registry。寿命は返されたstrong ownerが決める ([design/bus_accessor.md](../design/bus_accessor.md)) |
| 共有取得 | shared acquisition / acquire | `M5_Hal.<kind>.acquire(cfg)` が所有権を持つ共有ハンドル (`shared_ptr<IBus>`) を返す。直接構築はエスケープ |
| 共同所有 | co-own | `shared_ptr` からアクセサを構築するとアクセサがバスの生存を共有保持する (`Accessor dev{handle, cfg}`) |
| 明示終了 | explicit consuming close | `result_t<void> close(std::shared_ptr<IBus>& bus)`。exact instance + sole owner のときだけ成功し、caller の handle を消費する |
| 自然終了 | natural close / final-holder destruction | 最後の strong owner が消えたときの bus dtor 起点の終了 |
| portable取得 | portable acquisition | 共通`BusConfig`を`acquire(cfg)`へ渡す。provider選択はbuildのwinner bindingが行い、config型では選ばない |
| native所有方針 | native ownership policy | 対応providerだけが受理する`native::borrowed(resource)` / `native::managed(resource)`。bus取得用の`attach` / `open`は公開しない |
| 隔離済み墓標 | quarantined tombstone | 外部解放を確認できず、identity / remote bus ID の再利用を止める registry 状態 |
| バス種別 | bus kind | v2 では識別子も `bus_kind_t` / `BusKind` / `getBusKind()` で統一済 (旧 `bus_type_t` 等は v0 のみ) |
| 通信相手 / アクセス対象 | accessor | コード上 `Accessor`、 固有名詞扱い |
| 通信本体 (atomic I/O) | transfer | 動詞・名詞共通 |
| 1 回の transfer に付随するメタ情報 | per-call transfer metadata | `TransferDesc` の説明 |
| 接頭バイト列 | prefix bytes | I2C/SPI の register address 等 |
| 排他制御 | mutual exclusion / locking | Accessor内部の`acquireAccessLock` / `releaseAccessLock` |
| 同時に 1 owner のみ保有可能 | exclusive | "exclusive ownership" |
| wire frame (SPI CS assert区間 / I2C START〜STOP) | wire frame | protocol境界。Accessor lifecycle名とは分離する |
| Access | Access / access scope | `beginAccess`〜`endAccess`。排他・設定・backend開始終了を含む |
| 入れ子の Access | nested Access | 禁止。二重`beginAccess`は`INVALID_STATE` |
| active Accessの借用 | borrow the active Access | sugarは二重beginせず既存Accessへ参加する |
| 再 lock / 二重 lock | re-lock / re-acquire | |
| アクセス権限の所有者 | lock owner | コード上 `_lock_owner` |
| タイムアウト (lock取得待ち) | acquisition timeout | `beginAccess(timeout_ms)`の予算 (省略 = `TIMEOUT_FOREVER`、0 = 即時try) |
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
| stream 通信 | streaming transport | UART / remote等 |

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

| 避ける表記 | 推奨表記 | 備考 |
|---|---|---|
| view (Accessorの意味) | accessor | data viewではなくbus上の操作主体 |
| session (Accessの意味) | Access / access scope | network sessionと区別する |
| execute a transfer | issue / perform a transfer | transferの動詞形 |

## GPIO

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| ポート | port | `IPort` 抽象 |
| 個別ピン | pin | `IPin` |
| 統合 GPIO 番号 | unified GPIO number | `gpio_number_t` |
| ローカル pin | local pin | `IGPIO` 内のローカル番号 (0〜255)。port ordinal / mask bit との対応は `IGPIO::locatePin()` が定義 |
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

`capability`は対象を省略すると混同しやすいため、次の修飾を付ける。

| 概念 | 推奨表記 | 対象 |
|---|---|---|
| variant offer | variant offer / build-time offer | build時にkind実装を提供するか (`_offer.hpp`) |
| allocation capability | allocation capability | local controller resolver用の`backend_caps_t` |
| connection capability | remote connection capability | Helloで広告するGPIO、BusCreate、静的Bus一覧 (`remote::Capabilities`) |
| Bus instance capability | Bus instance capability / capability snapshot | 取得済みBusのoperationとlimit (`bus::BusCapabilities`) |

## v0 / v2 共存

| 日本語 / 概念 | 英語 (推奨) | 備考 |
|---|---|---|
| v0 / v2 | v0 / v2 (= API generation) | small letters |
| inline namespace 切替 | inline namespace switch | 利用者設定は`M5HAL_V0_INLINE` / `M5HAL_V2_INLINE`。展開用の`M5HAL_INLINE_V0` / `M5HAL_INLINE_V2`は内部macro |
| 上書き (override) | override | C++ override と意味同じ |
| 下位互換 | backward compatibility | v0 API の継続提供 |
