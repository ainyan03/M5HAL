# design/data_io — Source/Sink I/O モデル

> **読者**: 実装者・レビュー向け（設計仕様）。

**メンタルモデル**: **Source** = バイトの入力源（caller が供給・pull される側）。**Sink** = バイトの出力先（caller が受け取る・push される側）。**Accessor** が Source/Sink を橋渡しし、バス種別を問わない統一 API を提供する。

通信バス境界での入出力データは、 **Source/Sink 抽象** で表現する。 caller の手元バッファ (メモリ系)、 ringbuffer (stream 系)、 容量制限ラッパ等を共通 API で渡せる。

## 設計目標

- Bus 抽象側 (`IBus::transfer` の signature) では Source/Sink 一本に統一、 利用者向けの `uint8_t* / size_t` sugar は Accessor 側に集約
- zero-copy 通信経路を阻害しない (借用 Span の lifetime と冪等性を明確化)
- 同期 API として最小限の状態モデル (cursor 位置のみ) で説明できる

### なぜ `peek` は借用 Span を返すのか

**RAM / ROM に既にあるデータを、 無駄に複製しないため。** 同時に、 流れてくるストリームのデータも
同じ `Source` 型で扱えるようにするため。

`peek` がコピーを返す設計にすると、 ROM 上の定数配列を読むだけの利用者にも一時バッファを持たせる
ことになる。 借用にすれば、 バッファは本当に必要な側 (`StreamSource` の caller 提供 scratch) だけが
持てばよい。

無駄なコピーを省いた結果として実装が単純になり、 速度も上がる。 速度のために汎用性を捨てた選択では
ない ([../architecture.md](../architecture.md) §最適化の判断 の 1)。 ただし借用の lifetime 規則は
API の形に出るため、 本文書 §Source の契約 に明文化する (同 3)。

## Accessor のデータ授受 API 規約 (全バス共通)

ユーザがデータをやり取りする Accessor の API は、 **Source/Sink を基本形 (canonical)** とし、 `(ptr, size)` / `ConstDataSpan` / `DataSpan` は **糖衣 (sugar)** とする。 バス種別ごとの不統一を抑え、 一つのバスで覚えた形が他バスでもそのまま通じるようにするのが狙い。

- **基本形**: `write(data::Source&, size_t len)` / `read(data::Sink&, size_t len)` (および register 系は `writeRegister(reg, data::Source&, len)` / `readRegister(reg, data::Sink&, len)`)。`len` は授受するバイト数の上限。
- **糖衣**: 生ポインタ・span のオーバーロードは、 内部で `MemorySource` / `MemorySink` を構築 (または `LimitedSource` / `LimitedSink` で `len` を上限化) して基本形 (もしくは共通核) へ委譲する。 糖衣が基本形より多くの能力を持つことはない。
- **核**: transactional バス (`i2c` / `spi`) では `transfer(desc, Source*, Sink*)` が核で、 write/read は片側 null の委譲。 stream バス (`uart` / `i2s`) では `write(Source&, len)` / `read(Sink&, len)` 自体が核 (バックエンドの Source/Sink 受け口へ委譲)。
- **適用状況**: `spi` / `uart` / `i2s` は従来から基本形を提供済み。 `i2c` は `transfer(Source*,Sink*)` が核だが、 これに揃えて `write(Source&,len)` / `read(Sink&,len)` と register 系の Source/Sink 形を追加した (生/span から見た糖衣関係を明示)。
- **戻り値**: 本規約は引数側 (Source/Sink 基本化) の統一であり、 戻り値の型 (`result_t<void>` / `result_t<size_t>`) には関与しない (戻り値の意味論はバス種別ごとの別軸)。

## 主要型

namespace は `m5::hal::v2::data` で統一 (namespace 表記の規約は [../architecture.md](../architecture.md) §namespace 帰属)。

| 型 | 役割 |
|---|---|
| `ConstDataSpan` | read-only な `(ptr, size)` ラッパ。 非所有 |
| `DataSpan` | mutable な `(ptr, size)` ラッパ。 `ConstDataSpan` に implicit 変換 |
| `Source` | 送信データの供給源の抽象 (producer; `peek`/`advance` で pull される側、 引数では `src`) |
| `Sink` | 受信データの書き込み先の抽象 (consumer; `reserve`/`commit` で push される側、 引数では `dst`) |

Span は constexpr の軽量 helper を持つ: `empty()` / `begin()` / `end()` (range-for 可) / `first(n)` / `subspan(offset, count)`。 `first` / `subspan` は範囲外を **clamp** する (UB にしない)。

## 向き (direction) の規約

`src` / `dst` の命名は **subject-free**。 「master」 や 「ワイヤ」 を基準にせず、 **その API を通るバイトの動き** だけで決まる:

- **`src` (Source)** = バイトが **pull され出ていく**源。 caller が供給する側。
- **`dst` (Sink)** = バイトが **push され入ってくる**先。 caller が受け取る側。

この規約は **役割が反転しても崩れない** のが要点 (remote の polarity 逆転や slave 視点でも同じ語が正しい):

| API | `src` (供給する) | `dst` (受け取る) |
|---|---|---|
| master `transfer(desc, src, dst)` | master が **送信** (TX → ワイヤ) するバイト | master が **受信** (RX ← ワイヤ) するバイト |
| slave `serve(src, dst)` | slave の **応答** (master の read に返す) | master が **書いた** バイト (slave が受信) |

ワイヤ上の向きは役割で反転するが、 API 名 (`src` = pull 源 / `dst` = push 先) は不変。 **型が分かれている (`Source` ≠ `Sink`) ため、 向きを取り違えて渡すとコンパイルエラー** になる (逆渡し事故は型で防がれる。 残るは意味の理解だけで、 それがこの規約)。

## Source の契約

```cpp
namespace m5::hal::v2::data {

class Source {
public:
    virtual ~Source() = default;
    virtual result_t<ConstDataSpan> peek(size_t max_len) = 0;
    virtual result_t<void>          advance(size_t N)    = 0;
    virtual bool                    eof() const          = 0;
    virtual bool                    closed() const { return eof(); }
};

}
```

- `result_t<T>` = `m5::stl::expected<T, error::error_t>` の alias ([`hal/v2/error.hpp`](../../src/m5_hal/hal/v2/error.hpp)、 規約は [coding_style.md](../style/coding_style.md) §型選択)
- **`peek(max_len)`**: cursor 位置から最大 `max_len` bytes を借用 Span として返す。 連続呼び出し (advance を挟まない) では **monotonic non-decreasing** — 前回返した先頭部分のバイト列は不変、 size は減少せず末尾に追加されることがある。 pointer は同一とは限らないが、 内容の整合性は保証
- **借用 Span の lifetime**: 次の `peek` または `advance` 呼び出しまで有効
- **空 Span = 現時点の no progress**: `peek` で size 0 の Span が返っても、 それだけでは終端を意味しない。 ringbuffer が空、 stream が idle、 skip 予約の消化待ち、有限 Source の終端などを同じ値で表す。 caller は `eof()` / `closed()` で「今後増えない」かを別途判定する
- **`advance(N)`**: cursor を N 進める独立 cursor 操作。 peek の有無・サイズと無関係 (peek なしで advance を呼ぶことも許可)。 N の上限は仕様で制約しない
- **シーケンシャル読み出しでの読み捨ては正規の使い方** — protocol parse 時の skip-ahead や preamble 読み飛ばし等で普遍的なパターン
- **未到達範囲を advance 要求された場合**:
  - stream 系 Source: 内部に **skip 予約** として保持し、 データが届くたびに自動消費される (caller の while ループを不要にする)
  - メモリ系 Source: 不足分を破棄し end-of-stream へ遷移
- **`eof()`**: 「これ以上データは来ない」 final state の問い合わせ。 caller は advance 後に `eof()` を確認することで skip 不足を検知できる
- **`closed()`**: **生産側が閉じたか**の問い合わせ。 true = いま peek できるバイトが最終で、 以後追加されることはない (TCP half-close の意味論 — 読み残しは残り得る)。 `eof()` = `closed()` かつ飲み干した、 の関係。 default 実装は `eof()` を返す (stream 系は接続中 false で安全側)。 有限派生は override する — `MemorySource` は内容が構築時確定なので常に true。 consumer はこれで「再試行で進み得る短読み」 と 「永遠に進まない短読み」 を弁別する (`FrameReader` の端数末尾 livelock 対策)
- **`peek(0)` は契約違反**: 0 byte 要求と no-progress の空 Span を区別しにくくするため、 `max_len` は 1 以上であること。 違反時の挙動は派生実装定義 (size 検証は不要、 debug assert 任意 — Sink の契約違反と同じ扱い)。 `Sink::reserve(0)` も同様

## Sink の契約

```cpp
namespace m5::hal::v2::data {

class Sink {
public:
    virtual ~Sink() = default;
    virtual result_t<DataSpan> reserve(size_t max_len) = 0;
    virtual result_t<void>     commit(size_t N)        = 0;
    virtual bool                                 closed() const          = 0;
    virtual size_t partialCommitAccepted() const { return 0; }
};

}
```

- **`reserve(max_len)`**: cursor 位置から最大 `max_len` bytes の書き込み領域を借用 `DataSpan` として返す。 連続呼び出し (commit を挟まない) では Source と対称的に **monotonic non-decreasing**
- **借用 DataSpan の lifetime**: 次の `reserve` または `commit` 呼び出しまで有効
- **空 DataSpan = 現時点の no progress**: `reserve` で size 0 の DataSpan が返っても、 それだけでは closed を意味しない。 ringbuffer full や stream backpressure のような一時的状態と、有限 Sink の終端を同じ値で表す。 caller は `closed()` で最終状態かを別途判定する
- **`commit(N)`**: 通常は直前の `reserve` で返した DataSpan に書き込んだ量を報告する。 `N ≤ reserve size` であること
- **契約違反は未定義動作 (UB)** — `commit size > reserve size`、 `reserve` なしで `commit`、 等は派生実装に依存し一般に未定義。 派生実装は size 検証ロジックを **持たなくてよい** (任意で debug assert を持つことを推奨)
- **`closed()`**: 「これ以上書き込めない」 final state の問い合わせ
- **`partialCommitAccepted()` (部分 commit の受理数)**: ほとんどの派生では `commit(N)` は atomic (成功なら N 全量、 失敗なら 0) だが、 `commit` 自体が内部でブロッキング I/O を行う派生 (`StreamSink` が transport `write` をラップする場合など) は、 その I/O が N より短いプレフィックスだけ受理してからエラーを返し得る。 `partialCommitAccepted()` は直近の失敗した `commit()` が実際に受理したバイト数を返す。 デフォルト実装は 0 で、 commit が atomic な派生 (`MemorySink`/`RingFIFO::SinkView` 等) はこれで正しい。 デコレータ (`LimitedSink`) は base の受理数を転送する。`StdioSink` は `fwrite` の短い戻りまたは `fflush` 失敗を `IO_ERROR` として返し、`fwrite` が受理したプレフィックス長を `partialCommitAccepted()` で公開する。 `Source`/`Sink` を介して転送を中継する caller (§stream 系: `remote::detail::drainToSink` 等) は、 `commit` 失敗時にこの値だけ upstream の `Source::advance` を呼ぶことで、 「受理済みバイトを二度と再送しない・未受理バイトを捨てない (残りは次回の pump で再開)」 という中継契約を実装する

## Source と Sink の意図的な非対称性

両者は cursor 操作的構造で対称的だが、 用途の違いから以下の点で意図的に非対称:

| 操作 | Source | Sink |
|---|---|---|
| 「内容なしで cursor 進める」 | **仕様内挙動** (skip 用途、 シーケンシャル読み捨てが普遍的) | **未定義動作** (書かずに進む状況は稀、 派生実装の単純化を優先) |
| 未到達範囲の予約 | あり (skip 予約として保持) | なし (sync 前提では caller が wait する仕事) |

判断軸: 「対称のために合理性のない API を作らない」。

## error path の責務

Source / Sink の 4 つの core API (`peek` / `advance` / `reserve` / `commit`) は全て `result_t<...>` を返す。 これは **将来の stream 通信派生 (TCP/UDP/network ringbuffer/DMA/remote bus 等、 真の I/O error を発生させ得る派生) を視野に入れた抽象基底の規約**。 typical な同期メモリ系派生 (`MemorySource` / `MemorySink`) が現状 error を返さないのは **派生実装の現状であり、 抽象基底の規約ではない**。`LimitedSource` / `LimitedSink` は自ら新しい error を生成しないが、base の error はそのまま伝播する。

### caller 側の遵守事項

`Source*` / `Sink*` を受け取る一般 API (例: `IBus::transfer(..., Source*, Sink*)`) を実装する側、 もしくは Source / Sink を直接利用する caller は以下を遵守:

- **4 API すべての戻り値で `has_value()` チェックを行い、 error 時の path を持つ**
- 「現状 error を返さない派生 (`MemorySource` 等) を渡している」 を根拠に caller 側で error path を **省略してはいけない**。 同じ caller 関数が将来 stream 派生を渡されたとき、 error path 不在は silent fail を生む
- `IBus::transfer` のように渡される派生を限定しない契約の API では、 caller の error path 整備は必須。 派生型を限定して compile-time に省略可能と判断する path は、 将来別途 trait / template ベースで設計余地があるが、 現状の virtual API ではサポート対象外

### 派生実装者の責務

各派生 (現状の Memory 系を含む全派生) の docstring に以下を明記:

- error を返す condition (どの I/O event で何の `error_t` を返すか、 もしくは「error を返さない」 の明示宣言)
- error 後の cursor / 内部状態 (`advance` / `commit` を続けてよいか、 reset 規約、 二度目以降の呼び出しでも同 error を返すか)
- 現状 error を返さない派生でも「**caller は error path を省略しない**」 旨を docstring に書く (将来 stream 派生を念頭に置いた抽象規約)

### 将来の stream 派生で想定する error_t

参考想定 (`error_t` 細分化の検討余地と整合):

| 派生想定 | 返し得る error_t |
|---|---|
| network/remote bus Source/Sink | `TIMEOUT_ERROR`, `IO_ERROR`, `CLOSED`, `PROTOCOL_ERROR` 等 |
| ringbuffer Sink | `BUFFER_OVERFLOW` (commit 量 > 内部空き)、 `BUFFER_UNDERFLOW` (read 要求に対してデータ不足)、 `END_OF_STREAM` |
| framed stream Source/Sink | `CHECKSUM_ERROR`, `PROTOCOL_ERROR`, `BUFFER_OVERFLOW`, `TIMEOUT_ERROR` (**注**: 現状 header CRC 失敗は `DecodeStatus::InvalidCheck` による読み捨て — [frame.md §フレーム読み捨て](frame.md)。 `CHECKSUM_ERROR` は将来の payload checkpoint 層向け予約)|
| DMA backed Source/Sink | `IO_ERROR`, `TIMEOUT_ERROR` 等のハードウェア検出 transport error |

stream / frame / remote 系で必要になる粒度は v2 `error_t` に追加済み。 個別実装は上表を目安に、 可能な限り `UNKNOWN_ERROR` へ潰さず recoverable error の意味を保つ。

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| `Chunk` (lifetime 管理 + frame hint) | I2C / SPI register access 中心のスコープで過剰。 所有権移譲が必要になったら別途 `OwnedSpan` 等で導入 |
| `TransferResult` (`{transferred, error}`) | 既存 `result_t<size_t>` で十分。 「途中まで成功」 を細かく表現する必要が出てきたら別途検討 |
| `Completion` (非同期 handle) | sync 通信前提。 非同期サポートは将来の拡張余地として残す |
| NVI (Non-Virtual Interface) パターン | 素直な virtual API を採る ([../style/coding_style.md](../style/coding_style.md) 参照) |

## 派生具象一覧

| 型 | ファイル | 役割 |
|---|---|---|
| `MemorySource` | `hal/data/memory.hpp` | 固定 `ConstDataSpan` を起点に Source として yield する基本実装。 ctor は span 版に加え `(const uint8_t*, len)` 糖衣あり |
| `MemorySink` | 同上 | 固定 `DataSpan` に書き込む基本実装。 ctor は span 版に加え `(uint8_t*, len)` 糖衣。 進捗系 = `written()` / `capacity()` / `remaining()`。 `commit(N)` の oversize は契約違反で debug assert (release は cursor を clamp して overrun させない) |
| `LimitedSource` | `hal/data/limited.hpp` | base となる Source を「先頭 N byte だけ」 に制限する装飾。 base が先に eof / closed になればその時点で eof / closed。 cap 消費後も closed。base の error はそのまま伝播し、`advance` 失敗時は local cap を消費しない。error 後の cursor 状態と retry 可否は base の契約に従う。 **ctor 2 種**: 参照版は base 必須、 ポインタ版は optional で **null = 意図的な空 Source** (即 eof) |
| `LimitedSink` | 同上 | base となる Sink を「N byte だけ」 に制限する装飾。 ringbuffer 等の容量不明 / 無限 Sink から「N byte だけ受信」 を実現するのが典型用途。 ctor 2 種は `LimitedSource` と同様 (ポインタ版 null = 意図的な空 Sink、 即 closed) |
| `StreamReader` / `StreamWriter` | `hal/data/stream.hpp` | pull/push 型バイトストリームの最小能力を表す抽象 (§Stream アダプタ)。 UART split accessor 等の transport が実装する |
| `StreamSource` | 同上 | `StreamReader` を Source に持ち上げるアダプタ。 caller 提供 scratch で `peek` の借用契約を実現 |
| `StreamSink` | 同上 | `StreamWriter` を Sink に持ち上げるアダプタ。 `reserve` は scratch を貸し出し、 `commit` で write へパススルー |
| `TapReader` / `TapWriter` | `hal/data/tap.hpp` | StreamReader / StreamWriter の観測ミラー装飾。主経路を流れたバイトを mirror (`StreamWriter*`、null 可) へ best-effort 複製 (§Tap 装飾) |

### Limited 装飾の意義 (partial transfer)

`IBus::transfer` は Source / Sink から長さを決める設計のため、「長いバッファの一部だけ送信」や「容量不明 / 無限な Sink に N byte だけ受信」を素朴には表現できない。

**契約**: transfer signature を変えず、 `LimitedSource{base, N}` / `LimitedSink{base, N}` を caller が必要時に wrap して composable に解決する。 sugar 経路 (writeRegister 等) には影響なし。

## Stream アダプタ (StreamReader / StreamWriter / StreamSource / StreamSink)

consume 型のバイトストリーム (UART 受信、 将来の TCP / remote transport 等) を Source / Sink として消費可能にするアダプタ群 (`hal/data/stream.hpp`)。

### 最小ストリームインタフェース

transport 側は以下の最小能力だけを実装する:

```cpp
struct StreamReader {   // pull 側
    expected<size_t, error_t> read(DataSpan dst);   // 実装側の timeout 規約でブロック。 0 = 期限内に未着
    expected<size_t, error_t> readableBytes();      // ブロックせず読める byte 数
};
struct StreamWriter {   // push 側
    expected<size_t, error_t> write(ConstDataSpan src);  // 受理 byte 数を返す (timeout で短い write があり得る)
};
```

UART の split accessor (`TxAccessor` / `RxAccessor`、 [uart.md](uart.md)) はこのインタフェースを実装する。

### アダプタの契約

`Source::peek` は借用 API (連続 peek で prefix 不変) のため、 consume 型ストリームの上に直接は実装できない。 `StreamSource` は **caller 提供の非所有 scratch buffer** に peek 済み・未 advance のバイトを保持してこれを実現する。

- **idle は空 Span、hard error は error**: データ未着のまま実装側 timeout が切れた `peek` は、 届いた分だけの短い span を返し、 バッファが空のままなら size 0 の span を返す。これは recoverable な no-progress で、 caller は再 peek してよい。 transport の I/O error や invalid state は error path で返す
- **ブロッキング規約**: 要求 (`max_len`) をバッファで満たせない `peek` は、 不足分を実装側 read で 1 回ブロックして補充する (UART なら first_byte / inter_byte timeout 準拠)。要求が既にバッファ内にあれば即座に返る。 ブロックさせたくない caller は先に `readableBytes` を確認し、 その分だけ peek する
- **peek 上限 = scratch 容量**: `peek(max_len)` は scratch 容量までしか返さない (契約は「最大 max_len」 なので適合)。 consumer は短い peek を前提に書く
- **skip 予約**: バッファを超える `advance` は超過分を予約として保持し、 readable な分を即時読み捨て、 残りは後続の `peek` / `advance` が自動消費する (§Source の stream 系規約)
- **`StreamSink` はパススルー**: `reserve` は scratch を貸し出し (連続 reserve は同一 span で冪等)、 `commit(N)` が scratch 先頭 N byte を `write` する。 writer が N byte 未満しか受理しなかった場合は `TIMEOUT_ERROR` (retryable。 short write の支配的要因は writer 自身の write timeout で、 fatal に見せる `IO_ERROR` より read 側と対称な TIMEOUT_ERROR が適切 — `stream.inl` 参照)。 detached (null writer) への `commit` は黙って捨てず `CLOSED` を返す。 このとき writer が実際に受理した prefix 長は `partialCommitAccepted()` (§Sink の契約) 経由で読める — 中継 caller はこれで受理済み分だけ upstream を進め、 未受理分を次回に残す
- **接続開始時の flush**: `StreamSource::discardBuffered()` がアダプタ内のバッファ済みバイトと skip 予約を破棄する (相手のブートノイズや前回接続の残骸を次の peek に持ち越さないための入口。 TCP server の接続受入が利用 — [remote.md](remote.md) §TCP トランスポート、 実装 = `variants/frameworks/bsd/hal/remote/tcp_server.inl`)。 transport 側の受信キューは対象外 — 完全な flush が要る場合は transport レベル (posix なら `tcflush`) と併用する。 過去の peek で借用した span は無効化される

### Tap 装飾 (観測ミラー)

`TapReader` / `TapWriter` (`hal/data/tap.hpp`) は StreamReader / StreamWriter を装飾し、主経路を流れたバイトを観測用の mirror (`StreamWriter*`) へ複製する。ホスト側 wire dump ([remote.md](remote.md) §診断) の基盤だが、transport 非依存の汎用装飾。

**契約**:

- **mirror は観測者であり consumer ではない**: mirror の戻り値・エラーは主経路の結果に一切影響しない (結果は無視し、状態も持たない)。確実な複数ターゲット配信の機構ではない
- **実際に転送されたバイトだけを複製する**: reader 側は格納された n byte、writer 側は inner が受理した n byte (short write なら受理 prefix のみ)。n == 0 (timeout) とエラーは複製しない
- **呼び出し粒度 = レコード粒度**: 主経路 1 回の read/write につき mirror へ 1 回の write。mirror 側はこの粒度でレコード境界を観測できる
- mirror は null 可 (null = 純粋パススルー、仮想呼び出し 1 段のみ)
- **mirror はブロックしてはならない**: tap は転送ホットパスに挟まる前提。ブロックしうる出力先へは mirror 側でバッファリングを挟む (利用者責務)
- 1→N 分配は tap の入れ子で表現する (専用機構は持たない)

## 配置と命名 (namespace 1:1)

- 抽象 (`Source`, `Sink`, `ConstDataSpan`, `DataSpan`) は `src/m5_hal/hal/v2/data.hpp` (namespace `m5::hal::v2::data`)
- 具象 (`MemorySource`, `MemorySink`) は `src/m5_hal/hal/v2/data/memory.hpp` (同 namespace `m5::hal::v2::data`)
- 装飾派生 (`LimitedSource`, `LimitedSink`) は `src/m5_hal/hal/v2/data/limited.hpp` (同 namespace `m5::hal::v2::data`)
- Stream アダプタ (`StreamReader`, `StreamWriter`, `StreamSource`, `StreamSink`) は `src/m5_hal/hal/v2/data/stream.hpp` (同 namespace `m5::hal::v2::data`)
- Stream 装飾 (`TapReader`, `TapWriter`) は `src/m5_hal/hal/v2/data/tap.hpp` (同 namespace `m5::hal::v2::data`)
- 一時バッファ所有 (`memory::TempBuffer`) は [memory.md](memory.md) (namespace `m5::hal::v2::memory`)。 `MemorySource` / `MemorySink` は非所有 view のままとし、所有権を混ぜない。
- **ファイル名 (拡張子除く) は namespace 末尾と一致** が原則: `data.hpp` ⇔ `…::data` namespace
- 別 namespace を与えたい派生は `data/<sub>/` のようにディレクトリで階層化する (`hal/v2/data/io.hpp` のように「`data` フォルダ内に別概念を意味するファイル名」 は避ける)

詳細な配置規約は [../reference/directory-layout.md](../reference/directory-layout.md) を参照。

## 動詞規約

詳細は [../style/coding_style.md](../style/coding_style.md) §動詞規約 を参照。
