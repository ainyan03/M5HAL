# design/data_io — Source/Sink I/O モデル

通信バス境界での入出力データは、 **Source/Sink 抽象** で表現する。 caller の手元バッファ (メモリ系)、 ringbuffer (stream 系)、 容量制限ラッパ等を共通 API で渡せる。

## 設計目標

- Bus 抽象側 (`Bus::transfer` の signature) では Source/Sink 一本に統一、 利用者向けの `uint8_t* / size_t` sugar は Accessor 側に集約
- zero-copy 通信経路を阻害しない (借用 Span の lifetime と冪等性を明確化)
- 同期 API として最小限の状態モデル (cursor 位置のみ) で説明できる

## 主要型

namespace は `m5::hal::v1::data` で統一 (namespace 表記の規約は [../architecture.md](../architecture.md) §namespace 帰属)。

| 型 | 役割 |
|---|---|
| `ConstDataSpan` | read-only な `(ptr, size)` ラッパ。 非所有 |
| `DataSpan` | mutable な `(ptr, size)` ラッパ。 `ConstDataSpan` に implicit 変換 |
| `Source` | 送信側データ供給源の抽象 |
| `Sink` | 受信側データ書き込み先の抽象 |

## Source の契約

```cpp
namespace m5::hal::v1::data {

class Source {
public:
    virtual ~Source() = default;
    virtual m5::stl::expected<ConstDataSpan, error_t> peek(size_t max_len) = 0;
    virtual m5::stl::expected<void, error_t>          advance(size_t N)    = 0;
    virtual bool                                      eof() const          = 0;
};

}
```

- **`peek(max_len)`**: cursor 位置から最大 `max_len` bytes を借用 Span として返す。 連続呼び出し (advance を挟まない) では **monotonic non-decreasing** — 前回返した先頭部分のバイト列は不変、 size は減少せず末尾に追加されることがある。 pointer は同一とは限らないが、 内容の整合性は保証
- **借用 Span の lifetime**: 次の `peek` または `advance` 呼び出しまで有効
- **空 Span = end-of-stream**: `peek` で size 0 の Span が返れば end-of-stream 確定 (`eof()` も true)
- **`advance(N)`**: cursor を N 進める独立 cursor 操作。 peek の有無・サイズと無関係 (peek なしで advance を呼ぶことも許可)。 N の上限は仕様で制約しない
- **シーケンシャル読み出しでの読み捨ては正規の使い方** — protocol parse 時の skip-ahead や preamble 読み飛ばし等で普遍的なパターン
- **未到達範囲を advance 要求された場合**:
  - stream 系 Source: 内部に **skip 予約** として保持し、 データが届くたびに自動消費される (caller の while ループを不要にする)
  - メモリ系 Source: 不足分を破棄し end-of-stream へ遷移
- **`eof()`**: 「これ以上データは来ない」 final state の問い合わせ。 caller は advance 後に `eof()` を確認することで skip 不足を検知できる

## Sink の契約

```cpp
namespace m5::hal::v1::data {

class Sink {
public:
    virtual ~Sink() = default;
    virtual m5::stl::expected<DataSpan, error_t> reserve(size_t max_len) = 0;
    virtual m5::stl::expected<void, error_t>     commit(size_t N)        = 0;
    virtual bool                                 closed() const          = 0;
};

}
```

- **`reserve(max_len)`**: cursor 位置から最大 `max_len` bytes の書き込み領域を借用 `DataSpan` として返す。 連続呼び出し (commit を挟まない) では Source と対称的に **monotonic non-decreasing**
- **借用 DataSpan の lifetime**: 次の `reserve` または `commit` 呼び出しまで有効
- **空 DataSpan = closed**: `reserve` で size 0 の DataSpan が返れば書き込み終端確定 (`closed()` も true)
- **`commit(N)`**: 通常は直前の `reserve` で返した DataSpan に書き込んだ量を報告する。 `N ≤ reserve size` であること
- **契約違反は未定義動作 (UB)** — `commit size > reserve size`、 `reserve` なしで `commit`、 等は派生実装に依存し一般に未定義。 派生実装は size 検証ロジックを **持たなくてよい** (任意で debug assert を持つことを推奨)
- **`closed()`**: 「これ以上書き込めない」 final state の問い合わせ

## Source と Sink の意図的な非対称性

両者は cursor 操作的構造で対称的だが、 用途の違いから以下の点で意図的に非対称:

| 操作 | Source | Sink |
|---|---|---|
| 「内容なしで cursor 進める」 | **仕様内挙動** (skip 用途、 シーケンシャル読み捨てが普遍的) | **未定義動作** (書かずに進む状況は稀、 派生実装の単純化を優先) |
| 未到達範囲の予約 | あり (skip 予約として保持) | なし (sync 前提では caller が wait する仕事) |

判断軸: 「対称のために合理性のない API を作らない」。

## error path の責務

Source / Sink の 4 つの core API (`peek` / `advance` / `reserve` / `commit`) は全て `m5::stl::expected<..., error_t>` を返す。 これは **将来の stream 通信派生 (TCP/UDP/network ringbuffer/DMA/remote bus 等、 真の I/O error を発生させ得る派生) を視野に入れた抽象基底の規約**。 typical な同期メモリ系派生 (`MemorySource` / `MemorySink` / `LimitedSource` / `LimitedSink`) が現状 error を返さないのは **派生実装の現状であり、 抽象基底の規約ではない**。

### caller 側の遵守事項

`Source*` / `Sink*` を受け取る一般 API (例: `Bus::transfer(..., Source*, Sink*)`) を実装する側、 もしくは Source / Sink を直接利用する caller は以下を遵守:

- **4 API すべての戻り値で `has_value()` チェックを行い、 error 時の path を持つ**
- 「現状 error を返さない派生 (`MemorySource` 等) を渡している」 を根拠に caller 側で error path を **省略してはいけない**。 同じ caller 関数が将来 stream 派生を渡されたとき、 error path 不在は silent fail を生む
- `Bus::transfer` のように渡される派生を限定しない契約の API では、 caller の error path 整備は必須。 派生型を限定して compile-time に省略可能と判断する path は、 将来別途 trait / template ベースで設計余地があるが、 現状の virtual API ではサポート対象外

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
| framed stream Source/Sink | `CHECKSUM_ERROR`, `PROTOCOL_ERROR`, `BUFFER_OVERFLOW`, `TIMEOUT_ERROR` |
| DMA backed Source/Sink | `IO_ERROR`, `TIMEOUT_ERROR` 等のハードウェア検出 transport error |

stream / frame / remote 系で必要になる粒度は v1 `error_t` に追加済み。 個別実装は上表を目安に、 可能な限り `UNKNOWN_ERROR` へ潰さず recoverable error の意味を保つ。

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| `Chunk` (lifetime 管理 + frame hint) | I2C / SPI register access 中心のスコープで過剰。 所有権移譲が必要になったら別途 `OwnedSpan` 等で導入 |
| `TransferResult` (`{transferred, error}`) | 既存 `m5::stl::expected<size_t, error_t>` で十分。 「途中まで成功」 を細かく表現する必要が出てきたら別途検討 |
| `Completion` (非同期 handle) | sync 通信前提。 非同期サポートは将来の拡張余地として残す |
| NVI (Non-Virtual Interface) パターン | 素直な virtual API を採る ([../style/coding_style.md](../style/coding_style.md) 参照) |

## 派生具象一覧

| 型 | ファイル | 役割 |
|---|---|---|
| `MemorySource` | `hal/data/memory.hpp` | 固定 `ConstDataSpan` を起点に Source として yield する基本実装 |
| `MemorySink` | 同上 | 固定 `DataSpan` に書き込む基本実装 |
| `LimitedSource` | `hal/data/limited.hpp` | base となる Source を「先頭 N byte だけ」 に制限する装飾。 base が先に eof になればその時点で eof |
| `LimitedSink` | 同上 | base となる Sink を「N byte だけ」 に制限する装飾。 ringbuffer 等の容量不明 / 無限 Sink から「N byte だけ受信」 を実現するのが典型用途 |

### Limited 装飾の意義 (partial transfer)

`Bus::transfer` は Source / Sink から長さを決める設計のため、 以下が素朴には表現できない:

- 長いバッファの一部だけ送信したい (例: `MemorySource{base, 1024}` から先頭 128 byte だけ送る)
- 容量不明 / 無限な Sink (ringbuffer 等) に「N byte だけ受信」 を指定したい

これを **transfer signature を変えず装飾派生で composable に解決** する方針。 `LimitedSource{base, N}` / `LimitedSink{base, N}` を caller が必要時に wrap する。 sugar 経路 (writeRegister 等) には影響なし。

```cpp
// 1024 byte buffer の先頭 128 byte だけ送信
m5::hal::v1::data::MemorySource base{ConstDataSpan{buf, 1024}};
m5::hal::v1::data::LimitedSource tx{base, 128};
bus.transfer(&accessor, cfg, desc, &tx, nullptr);

// 無限 ringbuffer Sink から N byte だけ受信
RingbufferSink rb_sink{rb};
m5::hal::v1::data::LimitedSink rx{rb_sink, 256};
bus.transfer(&accessor, cfg, desc, nullptr, &rx);
```

代替案として transfer に長さ引数を追加せず、 装飾型で長さ制限を与える。

## 配置と命名 (namespace 1:1)

- 抽象 (`Source`, `Sink`, `ConstDataSpan`, `DataSpan`) は `src/m5_hal/hal/v1/data.hpp` (namespace `m5::hal::v1::data`)
- 具象 (`MemorySource`, `MemorySink`) は `src/m5_hal/hal/v1/data/memory.hpp` (同 namespace `m5::hal::v1::data`)
- 装飾派生 (`LimitedSource`, `LimitedSink`) は `src/m5_hal/hal/v1/data/limited.hpp` (同 namespace `m5::hal::v1::data`)
- 一時バッファ所有 (`memory::TempBuffer`) は [memory.md](memory.md) (namespace `m5::hal::v1::memory`)。 `MemorySource` / `MemorySink` は非所有 view のままとし、所有権を混ぜない。
- **ファイル名 (拡張子除く) は namespace 末尾と一致** が原則: `data.hpp` ⇔ `…::data` namespace
- 別 namespace を与えたい派生は `data/<sub>/` のようにディレクトリで階層化する (`hal/v1/data/io.hpp` のように「`data` フォルダ内に別概念を意味するファイル名」 は避ける)

詳細な配置規約は [../reference/directory-layout.md](../reference/directory-layout.md) を参照。

## 動詞規約

| 動詞 | 用途 |
|---|---|
| `peek` / `advance` | Source: 借用 Span 取得 / cursor 前進 (連続 peek は monotonic non-decreasing 冪等) |
| `reserve` / `commit` | Sink: 書き込み領域取得 / 書き込み量報告 (契約違反は UB) |
| `eof` / `closed` | 終端 final state 問い合わせ (Source 側 / Sink 側) |
