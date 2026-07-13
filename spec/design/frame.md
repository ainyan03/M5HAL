# design/frame — byte-stream framing codec

> **読者**: 実装者・レビュー向け（設計仕様）。

UART / TCP のような **境界を持たないバイトストリーム**の上に、フレーム境界と整合性検査を与えるコーデック。namespace は `m5::hal::v2::frame` (`src/m5_hal/hal/v2/frame/`)。

純粋 codec (span ベースの stateless encode/decode、I/O なし) と、それを [data_io.md](data_io.md) の Source/Sink モデルに載せる `FrameReader` / `FrameWriter` の 2 層で構成する。

由来: LovyanAPI lxyz `stream_frame` の移植。ヘッダ整形と検査範囲を見直した非互換改版のため、本フォーマットを **M5HAL frame v1** と呼ぶ。

## ワイヤフォーマット (M5HAL frame v1)

```
checked frame : [LEN:1][KIND:1][CHECK8:1][B3:1][payload:0..252]
Padding       : [0x00]
Delimiter     : [0x00][0x55]
```

| フィールド | サイズ | 内容 |
|---|---|---|
| LEN | 1 | **KIND より後のバイト数** (= CHECK8 1 + B3 1 + payload)。checked frame では最小 2、最大 254。`LEN=0` は padding / delimiter |
| KIND | 1 | フレーム種別 (下表) |
| CHECK8 | 1 | CRC-8 ATM、検査対象は **(LEN, KIND) のヘッダ 2 byte のみ**。payload は意図的に含めない |
| B3 | 1 | KIND 依存の 1 byte (`Data` → `stream_id`、`Request`/`Response` → `seq` 等) |
| payload | LEN-2 | kind 固有 payload。最大 `kMaxPayload=252` |

- **フレーム最大長 = 256 byte** (header 4 + payload 252)。これは TempBuffer 1 ブロックにちょうど収まる寸法 (= 設計上の上限根拠)
- **Padding** `0x00` 1 byte は読み飛ばし対象 (アイドルフィルやアライメント用)
- **Delimiter** `[0x00][0x55]` は check を持たない 2 byte の区切りマーカー。破損からの **resync 境界**として機能する
- 定数: `kHeaderSize=4` (LEN/KIND/CHECK8/B3) / `kCheckSize=1` / `kMaxPayload=252` (3 でも 4 でも割り切れる) / `kMaxFrameSize=256`

### CHECK8 (CRC-8 ATM)

- 多項式 `0x07`、初期値 `0x00`、反転なし。test vector: `"123456789"` → `0xF4`
- 実装はテーブルなしの bit ループ。フットプリント優先で、必要になればテーブル化は内部最適化として自由
- **検査対象は (LEN, KIND) のヘッダ 2 byte のみ**。受信側は「再計算 == 格納値」でヘッダの整合を検証する
- **payload を意図的に含めない**: payload の完全性は上位の **checkpoint 層**の責務とし、frame 層はフレーム境界とヘッダ整合の確立に専念する (二重に payload CRC を持たない)

### kind 一覧

| kind | 値 | B3 の意味 | 本 codec での扱い |
|---|---|---|---|
| `Padding` | 0x00 | — | 読み飛ばし (`LEN=0`、後続が 0x55 でなければ 1 byte 消費) |
| `Data` | 0x01 | `stream_id` | 汎用 data frame。payload はストリーム本体 |
| `Credit` | 0x03 | 受け入れ可能 Data frame 数 (絶対値・255 飽和) | flow control (mux 層)。意味論は [remote.md](remote.md) §フロー制御 |
| `Checkpoint` | 0x05 | kind 固有 | payload 完全性検査点 (上位層) |
| `Control` | 0x07 | kind 固有 | 制御 |
| `Request` | 0x09 | `seq` | リクエスト |
| `Response` | 0x0B | `seq` | レスポンス |
| `HelloReq` | 0x0D | kind 固有 | ハンドシェイク要求 |
| `HelloResp` | 0x0F | kind 固有 | ハンドシェイク応答 |
| `Ping` | 0x11 | kind 固有 | 死活監視 |
| `Pong` | 0x13 | kind 固有 | 死活応答 |
| `Event` | 0x15 | 送信側がインクリメントするイベント sequence 番号 | GPIO push event。意味論は [remote.md](remote.md) 参照 |
| `Delimiter` | 0x55 | — | check なしの区切りマーカー (`[0x00][0x55]`) |

codec が意味解釈するのは `Data` の stream_id 規則と Padding / Delimiter のみ。その他 kind のフレームも CHECK8 検証と透過運搬は行う (B3 / payload は解釈しない)。各 kind の意味論・B3 の詳細は上位層 ([remote.md](remote.md)) が定める。

`Data` の stream_id (B3) の割当規約 (0..15 全てデータストリーム、 予約チャネルなし) は [remote.md](remote.md) §stream_id レジストリが定める。

## decode の意味論

`decode(src, view)` は `DecodeResult{status, consumed}` を返す。**非 Ok もストリーム処理の正常系** (Padding 読み飛ばし、破損からの resync) なので、error path ではなく status で表す。

| status | consumed | 意味 |
|---|---|---|
| `Ok` | フレーム全長 | 1 フレーム取得。`View` が入力 span を借用 |
| `NeedMore` | 0 | バイト不足。追加受信後に再試行 |
| `Padding` | 1 | padding 1 byte 読み飛ばし |
| `Delimiter` | 2 | delimiter `[0x00][0x55]` を消費 (resync 境界として認識) |
| `InvalidPrefix` | 自己記述長 | 未知 kind。LEN を信じてフレーム候補ごと読み捨て |
| `InvalidSize` | フレーム全長 | 既知 kind だが LEN が不正 (checked で LEN<2 等) |
| `InvalidCheck` | フレーム全長 | CHECK8 不一致 (ヘッダ破損)。フレーム読み捨て |

- `LEN=0` の場合、後続が `0x55` なら `Delimiter`、そうでなければ `Padding` として 1 byte 消費する (ゼロ連続への耐性)
- **resync**: LEN byte 自体が破損した場合、自己記述長に従った読み捨ては誤った位置に進み得る。その場合も Padding / Delimiter / 次の有効フレームのいずれかで再同期する。確実な再同期点が必要な送信側は、フレーム列の合間に Delimiter を打つ
- **検出力の限界**: CHECK8 はヘッダ 2 byte のみを守る。payload のビット誤りは frame 層では検出せず、上位の checkpoint 層で検出する設計

## encode

| API | 戻り値 | エラー |
|---|---|---|
| `encodeDelimiter(dst)` | 書き込みバイト数 | dst 不足 → `BUFFER_OVERFLOW` |
| `encodeChecked(dst, kind, b3, payload)` | 同上 | kind が checked でない / payload 超過・null → `INVALID_ARGUMENT`、dst 不足 → `BUFFER_OVERFLOW` |
| `encodeData(dst, stream_id, payload)` | 同上 | payload > 252・null → `INVALID_ARGUMENT`、dst 不足 → `BUFFER_OVERFLOW` |

- `encodeChecked` は B3 を明示引数で受け取り、`[LEN][KIND][CHECK8][B3][payload]` を直接構築する (旧 `kind_body` を B3+payload に分離)。CHECK8 は (LEN, KIND) から算出して格納する
- `encodeData` は `encodeChecked(dst, Kind::Data, stream_id, payload)` の薄いラッパ
- いずれも `result_t<size_t>`。エラー時に部分出力は生じない。dst へ直接構築する (中間バッファなし)
- **`buildDataFrame(block, stream_id, Source&)`**: TempBuffer ブロックを宛先に取り、Source から payload を引き込みつつ 1 枚の Data フレームを組み立てるヘルパ。フレーム最大長 = ブロック長 (256B) なので、ブロック 1 つで 1 フレームが完結する

## FrameReader / FrameWriter (Source/Sink 統合)

```cpp
// UART RX accessor → StreamSource → FrameReader
uint8_t scratch[m5::hal::v2::frame::kMaxFrameSize];
data::StreamSource src{dev.rx(), data::DataSpan{scratch, sizeof scratch}};
frame::FrameReader reader{src};

frame::View view;
auto r = reader.next(view);   // expected<DecodeResult, error_t>
```

`FrameReader::next(View&)`:

詳細は [data_io.md](data_io.md) §Source の契約 を参照。本 kind 固有の差分のみ以下に示す。

- **Ok 時の advance は次回 `next()` 呼び出しまで遅延する**。`View` は Source の借用 span を指しており、借用 Span の lifetime (次の peek/advance まで有効) と整合させるため。よって **View の有効期間は次の `next()` 呼び出しまで**
- Padding / Delimiter は内部で読み飛ばす。`Invalid*` は該当バイトを advance 済みの状態で status として返す (caller が resync 事象を計数・記録できる)
- Source のエラーは素通しする。Source が空 span を返し、かつ `closed()` なら `END_OF_STREAM`、未 close なら `NeedMore`
- **取得は二段階**: LEN byte を peek してフレーム実寸 (= 2 + LEN) を確定し、その寸法だけを要求する。バッファ済みのフレームは待たずに即 decode され、`StreamSource` がブロックする (timeout まで) のはフレームが本当に未完のときだけ
- **要件: Source は各フレームの wire 実寸を一度の `peek` で貸せること** (最大寸フレームには `kMaxFrameSize` = 256。`StreamSource` の scratch を 256 byte にしておけば常に十分)。貸出上限を超えるフレームは `NeedMore` から進まない

`FrameWriter` は `Sink::reserve` で得た span に直接 encode して `commit` する (中間コピーなし)。`writeDelimiter()` / `writeChecked()` / `writeData()` がフレーム 1 枚ずつを Sink へ書く。Sink がフレーム長を貸せない場合は `CLOSED` (closed 時) / `BUFFER_OVERFLOW`、Sink 由来のエラーは素通し。`StreamSink` と組むと UART TX へのフレーム送信になる。

## 互換性と版管理

- 本フォーマット (M5HAL frame v1) の段は **`experimental`** ([../stability.md](../stability.md))。`stable` を宣言するまでは非互換変更があり得る
- `stable` 宣言後に wire 非互換の変更が必要になった場合は **v2 として別フォーマット名**を与え、v1 と混在させない
- 予約済みの kind 値は将来の版でも再利用しない (衝突防止のための予約)

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| payload を含む CRC (CHECK16 等) | payload 完全性は上位 checkpoint 層の責務。frame 層で二重に持たず、ヘッダ整合のみを CHECK8 で守る |
| STX/SEQ/TYPE 型フレーミング (request-response 意味論) | seq 管理・ACK は transport 層の責務。フレーミングは境界 + ヘッダ整合に限定する |
| byte-feed 型 decoder 状態機械 | span + lookahead の方が Source::peek 契約と素直に合成でき、状態を持たない分テストが単純 |

## 関連

- Source/Sink 契約: [data_io.md](data_io.md) (§Stream アダプタ含む)
- 本 codec 上のリモートメッセージ層: [remote.md](remote.md)
- UART アクセサとの接続: [uart.md](uart.md)
- 検証: [../verification.md](../verification.md) (native gtest `test_frame_codec` / posix UART end-to-end)
