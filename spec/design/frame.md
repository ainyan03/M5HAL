# design/frame — byte-stream framing codec

UART / TCP のような **境界を持たないバイトストリーム**の上に、フレーム境界と整合性検査を与えるコーデック。namespace は `m5::hal::v1::frame` (`src/m5_hal/hal/v1/frame/`)。

純粋 codec (span ベースの stateless encode/decode、I/O なし) と、それを [data_io.md](data_io.md) の Source/Sink モデルに載せる `FrameReader` / `FrameWriter` の 2 層で構成する。

由来: LovyanAPI lxyz `stream_frame` の移植。チェックサムを CHECK8 (CRC8-ATM) から CRC16-CCITT へ強化した非互換改版のため、本フォーマットを **M5HAL frame v1** と呼ぶ。

## ワイヤフォーマット (M5HAL frame v1)

```
checked frame : [SIZE:1][KIND:1][CHECK16:2 BE][KIND_BODY:SIZE-2]
padding       : [0x00]
delimiter     : [0x00][0x55]
```

| フィールド | サイズ | 内容 |
|---|---|---|
| SIZE | 1 | **KIND より後のバイト数** (= CHECK16 2 + kind body)。checked frame では最小 2、最大 255 |
| KIND | 1 | フレーム種別 (下表) |
| CHECK16 | 2 | CRC16-CCITT、big endian。CHECK16 自身を 0 とみなして wire 全体を計算 (self-verifying) |
| KIND_BODY | SIZE-2 | kind 固有 payload。`data` では `[stream_id:1][payload:0..240]` |

- **padding** `0x00` 1 byte は読み飛ばし対象 (アイドルフィルやアライメント用)
- **delimiter** `[0x00][0x55]` は check を持たない 2 byte の区切りマーカー。破損からの **resync 境界**として機能する
- 定数: `kPrefixSize=2` / `kCheckSize=2` / `kMaxBodySize=255` / `kMaxWireSize=257` / `kMaxDataPayload=240`

### CRC16-CCITT

- 多項式 `0x1021`、初期値 `0xFFFF`、反転なし (CRC-16/CCITT-FALSE)。test vector: `"123456789"` → `0x29B1`
- 実装はテーブルなしの bit ループ (`crc16CcittUpdate`)。フットプリント優先で、必要になればテーブル化は内部最適化として自由
- 検査対象は wire frame 全体。格納位置 (offset 2-3) を 0 とみなして計算するため、受信側は「再計算 == 格納値」で検証できる

### kind 一覧

| kind | 値 | 本 codec での扱い |
|---|---|---|
| `padding` | 0x00 | 読み飛ばし (SIZE 位置の 0x00 と合わせて 1 byte) |
| `data` | 0x01 | kind body 先頭 1 byte を `stream_id` とする汎用 data frame |
| `credit` | 0x03 | **予約** (将来の mux / flow control 層) |
| `checkpoint` | 0x05 | 予約 |
| `control` | 0x07 | 予約 |
| `negotiate` | 0x09 | 予約 |
| `management` | 0x0B | 予約 |
| `credit_delta` | 0x0D | 予約 |
| `delimiter` | 0x55 | check なしの区切りマーカー |

codec が意味解釈するのは `data` の stream_id 規則と padding / delimiter のみ。予約 kind のフレームも CHECK16 検証と透過運搬は行う (kind body は解釈しない)。フロー制御等の意味論は将来の mux / transport 層の仕様で定める。

## decode の意味論

`decode(src, view)` は `DecodeResult{status, consumed}` を返す。**非 ok もストリーム処理の正常系** (padding 読み飛ばし、破損からの resync) なので、error path ではなく status で表す。

| status | consumed | 意味 |
|---|---|---|
| `ok` | フレーム全長 | 1 フレーム取得。`View` が入力 span を借用 |
| `need_more` | 0 | バイト不足。追加受信後に再試行 |
| `padding` | 1 | padding 1 byte 読み飛ばし |
| `invalid_prefix` | 自己記述長 | 未知 kind。SIZE を信じてフレーム候補ごと読み捨て |
| `invalid_size` | フレーム全長 | 既知 kind だが SIZE が不正 (delimiter に body、checked で SIZE<2) |
| `invalid_check` | フレーム全長 | CRC16 不一致。フレーム読み捨て |

- 未知 kind かつ偶数値で SIZE=0 のものは padding 同様に 1 byte 消費する (ゼロ連続への耐性)
- **resync**: SIZE byte 自体が破損した場合、自己記述長に従った読み捨ては誤った位置に進み得る。その場合も padding / delimiter / 次の有効フレームのいずれかで再同期する。確実な再同期点が必要な送信側は、フレーム列の合間に delimiter を打つ
- **検出力の限界**: CRC16 は 257 byte 以下のフレームに対し多ビットバーストの見逃し確率 約 2^-16。これを超える保証が必要な用途は上位層で対処する

## encode

| API | 戻り値 | エラー |
|---|---|---|
| `encodeDelimiter(dst)` | 書き込みバイト数 | dst 不足 → `BUFFER_OVERFLOW` |
| `encodeChecked(dst, kind, kind_body)` | 同上 | kind が checked でない / body 超過・null → `INVALID_ARGUMENT`、dst 不足 → `BUFFER_OVERFLOW` |
| `encodeData(dst, stream_id, payload)` | 同上 | payload > 240・null → `INVALID_ARGUMENT`、dst 不足 → `BUFFER_OVERFLOW` |

いずれも `m5::stl::expected<size_t, error_t>`。エラー時に部分出力は生じない。dst へ直接構築する (中間バッファなし)。

## FrameReader / FrameWriter (Source/Sink 統合)

```cpp
// UART RX accessor → StreamSource → FrameReader
uint8_t scratch[m5::hal::v1::frame::kMaxWireSize];
data::StreamSource src{dev.rx(), data::DataSpan{scratch, sizeof scratch}};
frame::FrameReader reader{src};

frame::View view;
auto r = reader.next(view);   // expected<DecodeResult, error_t>
```

`FrameReader::next(View&)`:

- `Source::peek` が decode に必要な lookahead をそのまま提供する (peek → decode → advance(consumed))
- **ok 時の advance は次回 `next()` 呼び出しまで遅延する**。`View` は Source の借用 span を指しており、Source 契約 (借用は次の peek/advance まで有効) と整合させるため。よって **View の有効期間は次の `next()` 呼び出しまで**
- padding は内部で読み飛ばす。`invalid_*` は該当バイトを advance 済みの状態で status として返す (caller が resync 事象を計数・記録できる)
- Source のエラーは素通しする: `StreamSource` の `TIMEOUT_ERROR` は「まだ来ていない、再試行可」。Source が EOF (空 span) を返したら `END_OF_STREAM`
- **取得は二段階**: 2 byte の prefix を peek してフレーム実寸を確定し、その寸法だけを要求する。バッファ済みのフレームは待たずに即 decode され、`StreamSource` がブロックする (timeout まで) のはフレームが本当に未完のときだけ
- **要件: Source は各フレームの wire 実寸を一度の `peek` で貸せること** (最大寸フレームには `kMaxWireSize` = 257。`StreamSource` の scratch を 257 byte にしておけば常に十分)。貸出上限を超えるフレームは `need_more` から進まない

`FrameWriter` は `Sink::reserve` で得た span に直接 encode して `commit` する (中間コピーなし)。`writeDelimiter()` / `writeChecked()` / `writeData()` がフレーム 1 枚ずつを Sink へ書く。Sink がフレーム長を貸せない場合は `CLOSED` (closed 時) / `BUFFER_OVERFLOW`、Sink 由来のエラーは素通し。`StreamSink` と組むと UART TX へのフレーム送信になる。

## 互換性と版管理

- 本フォーマット (M5HAL frame v1) は**実験段階**であり、公開リリースノートで凍結を宣言するまでは非互換変更があり得る
- 凍結後に wire 非互換の変更が必要になった場合は **v2 として別フォーマット名**を与え、v1 と混在させない。版の判別・切替が必要になった時点で、予約済みの `negotiate` kind を用いた合意手順をその版の仕様で定める
- 予約 kind の値は将来の版でも再利用しない (衝突防止のための予約)

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| STX/SEQ/TYPE 型フレーミング (request-response 意味論) | seq 管理・ACK は transport 層の責務。フレーミングは境界 + 整合性検査に限定する |
| CRC8 (CHECK8) | フレーム長 ≤257B に対し検出力が不足気味 (バースト見逃し 2^-8)。+1 byte/frame で 2^-16 を取る |
| byte-feed 型 decoder 状態機械 | span + lookahead の方が Source::peek 契約と素直に合成でき、状態を持たない分テストが単純 |
| flow control (credit) の意味論 | mux / transport 層の判断事項。kind 値のみ予約 |

## 関連

- Source/Sink 契約: [data_io.md](data_io.md) (§Stream アダプタ含む)
- UART アクセサとの接続: [uart.md](uart.md)
- 検証: [../verification.md](../verification.md) (native gtest `test_frame_codec` / posix UART end-to-end)
