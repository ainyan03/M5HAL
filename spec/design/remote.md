# design/remote — リモートバス機構 (proxy / server)

ホスト側のコードから、バイトストリーム (UART 等) の先にあるリモートデバイス上のバス / GPIO を**ローカルのバスと同じ API で**操作するための機構。namespace は `m5::hal::v1::remote` (`src/m5_hal/hal/v1/remote/`)。

下層は確定済みの 3 仕様を組み合わせる: [data_io.md](data_io.md) の Source/Sink + Stream アダプタ、[frame.md](frame.md) のフレーミング、[bytecode.md](bytecode.md) の命令列と対称パイプライン。本仕様が新たに定めるのは**その上のメッセージ層** (request/response の対応付け、能力交換、エラー伝播、安全境界) だけである。

```
host                                      device
────                                      ──────
RemoteI2CBus 等の proxy                    RemoteServerService (service runner 統合)
  └ RemoteSession                            └ Server
     (seq / timeout / resync /                  ├ FrameReader / FrameWriter
      remote error 回収 / event hook)           ├ BytecodeRunner (登録制 dispatch)
     └ FrameReader / FrameWriter                └ 登録 = 公開範囲 (allowlist)
        └ StreamSource / StreamSink
           └ UART accessor (posix / arduino / espidf)
```

- **proxy** はローカルの Bus / Accessor インタフェースを実装する。呼び出し側はローカルバスと型レベルで同じ扱いができ、リモートか否かは構築時にだけ現れる
- **server** は受信した bytecode script を BytecodeRunner で実行し、応答 script を返す受動エンドポイント。応答 script をホスト側 Runner で実行して結果を回収する**対称パイプライン** ([bytecode.md](bytecode.md)) をそのまま使う

## メッセージ層 (M5HAL remote v1)

リモートメッセージは frame v1 の **`data` kind に載せる** (新しい frame kind は定義しない)。`stream_id` がリモートチャネルを区画し、payload 先頭 2 byte がメッセージヘッダになる。

```
data frame の payload : [TYPE:1][SEQ:1][BODY:0..238]
```

### stream_id レジストリ

| stream_id | 用途 |
|---|---|
| `0x00` | リモートメッセージチャネル (既定) |
| `0x01` 以降 | アプリケーション固有の raw データトンネル等。本仕様は関知しない |

複数チャネル (マルチノード等) への拡張は将来の mux 層の仕様で stream_id 割当として定める。本仕様は **point-to-point (1 ストリーム = 1 デバイス)** を前提とする。

### TYPE

| bit | 意味 |
|---|---|
| 3-0 | メッセージ種別 (下表) |
| 7 | **NORESP** — fire-and-forget。`request` でのみ意味を持つ。server は応答を返さず、エラーは pending error として保持する |
| 6 | **MORE** — チャンク継続 (予約)。本版では常に 0。受信側は 1 が立ったメッセージを破棄する |
| 5-4 | 予約 (0 固定) |

| 種別 | 値 | 方向 | BODY |
|---|---|---|---|
| `hello` | 0x0 | host → device | 空 |
| `hello_resp` | 0x1 | device → host | 能力一覧 (下記) |
| `request` | 0x2 | host → device | bytecode script |
| `response` | 0x3 | device → host | 応答 bytecode script (`store_data*` + `report_*`) |
| `error` | 0x4 | device → host | `[error:i8][detail...]` (detail は前方互換の末尾拡張枠、本版は空) |
| `ping` | 0x5 | host → device | 空 |
| `pong` | 0x6 | device → host | 空 |
| `event` | 0x7 | device → host | push 通知。BODY = bytecode script (§push イベント) |
| — | 0x8-0xF | — | 予約 |

未知の種別、または予約 bit が 0 でないメッセージは**受信側が黙って破棄**する (新旧実装の混在で安全側に倒す)。

### SEQ

- 8 bit 循環。host 発のメッセージ (`hello` / `request` / `ping`) ごとに採番する
- device は対応する応答 (`hello_resp` / `response` / `error` / `pong`) に**同じ SEQ を返す**。host は SEQ 不一致の応答を破棄する
- `event` は device 管理の独立した採番 (8 bit 循環、情報提供のみ — 照合には使わない)

### hello — 最小能力交換

接続確立後に host が最初に送る。`hello_resp` の BODY:

```
[proto_ver:1][flags:1][n:1]([bus_kind:1][bus_id:1]) * n
```

- `proto_ver` = 1 (本版)。host は自分の知らない版を `UNSUPPORTED` として扱う
- `flags` bit0 = GPIOGroup 登録あり。他 bit は予約 0
- `(bus_kind, bus_id)` は server の BytecodeRunner に登録済みのバス一覧。**bus_id とリモート物理バスの対応はこの一覧が唯一の wire 規約** (割当自体は server アプリの構成)
- 末尾へのフィールド追加は前方互換とする (受信側は既知部分だけ読み、残りを無視する)

### request / response

- `request` の BODY は bytecode script そのもの。server は実行し、`response` の BODY に応答 script (`store_data*` + `report_error`/`report_complete` + 終端) を返す
- host は応答 script を自分の BytecodeRunner で実行し、store スロットと report を回収する
- **NORESP request**: 応答を返さない。実行エラーは server が **pending error (最新 1 件、上書き)** として保持し、次に同期メッセージ (`hello` / `request` / `ping`) を受けた時、その応答の**直前に同じ SEQ の `error` メッセージを先行送信**してからクリアする。host session は `error` を記録して応答待ちを継続する
- `error` はこのほか、protocol レベルの拒否 (サイズ超過・実行前検査の不合格など) を `response` の代わりに返す用途にも使う

### サイズ上限

frame v1 の data payload 上限 240 から、メッセージ BODY は **238 byte** が上限。そこから:

- **request script ≤ 238** — proxy は組み立て時に検査し、超過は `INVALID_ARGUMENT` で拒否する (wire に出さない)
- **response 側は 1 転送あたり受信データ ≤ 230 を保証** (`store_data` 命令と `report_complete` のオーバーヘッド差引)。proxy は `rx_len > 230` の転送を `INVALID_ARGUMENT` で拒否する
- これを超える転送の分割 (チャンク化) は MORE bit を使う将来版で定める

### timeout / resync

- host は応答 timeout 時、**delimiter (`00 55`) を送出してから**次の送信を行う (server 側 FrameReader の確実な再同期点、[frame.md](frame.md) §decode)
- **自動再送は行わない** (既定)。リモート操作は非冪等であり得るため、timeout は `TIMEOUT_ERROR` として呼び出し側に返し、再試行の判断は呼び出し側に委ねる。再送する場合も新しい SEQ で発行する (旧 SEQ の遅延応答は照合で破棄される)
- 接続喪失の検出は `ping` の失敗、または下層 stream のエラーによる。session は以後 `DISCONNECTED` を返す

## server の実行モデル

- 実行は **CHECK16 検証済みの完成フレームから**のみ行う (kind body を `MemorySource` 化して `BytecodeRunner::run` へ渡す)。受信途中の stream を直接 Runner に与えてはならない — 停滞した相手が `BUFFER_UNDERFLOW` (回復不能) を誘発するため
- **実行前検査**: 命令は全て長さ前置なので、実行前に script を 1 パス走査できる。`delay_ms` が server 設定の `max_delay_ms` を超える script は実行せず `report_error(INVALID_ARGUMENT)` で拒否する (黙って切り詰める clamp はしない)。`bus_configure` が運ぶ timeout は server 設定の上限で制限する
- したがって server の **worst-case ブロック時間 = `max_delay_ms` の総和 + 制限後の bus timeout** で有界になる。これを超える長時間処理 (resumable 実行) は将来版の課題
- server は service runner ([m5_hal/hal/v1/service](../../src/m5_hal/hal/v1/service/service.hpp)) に登録する poll 型を正本とする。**server 用 rx accessor は first_byte timeout を 0 または極小に構成する** (StreamSource の peek は不足分を timeout までブロックするため、既定値のままではアイドル poll が停止する)
- rx scratch (≥ `frame::kMaxWireSize` = 257) と応答バッファ (≥ 257) は呼び出し側が提供する。応答 script がバッファに収まらない場合は `report_error(BUFFER_OVERFLOW)` のみの応答に差し替える

## push イベント (GPIO 変化通知)

host のポーリングなしに、device 側の GPIO 変化を `event` メッセージで通知する仕組み。
**イベント本体も bytecode script** (対称設計): `evt_gpio_state` は非 critical opcode なので、
購読機構を持たない古い host も安全に読み飛ばせる。

### wire

| opcode | payload | 意味 |
|---|---|---|
| `gpio_subscribe` (0x24) | `([gpio_num:u16])*` | 列挙ピンを購読に追加 (重複は冪等)。購読時点のレベルを基準とし、**以後の変化から**通知する |
| `gpio_unsubscribe` (0x25) | `([gpio_num:u16])*` (空 = 全解除) | 購読解除 |
| `evt_gpio_state` (0x60) | `([gpio_num:u16][level:u8])*` | 変化したピンと新レベル (複数ピンの同時変化は 1 命令にバッチされ得る) |

- `event` メッセージの BODY = `evt_gpio_state` 命令を含む script。payload の末尾拡張は前方互換
  (エッジフィルタ等は将来 `gpio_subscribe` payload の拡張で追加する)
- subscribe の失敗は通常の response で返る: 購読枠の枯渇 = `OUT_OF_RESOURCE`、未登録ピン =
  `INVALID_ARGUMENT`、購読機構を持たない server (handler 未設定の runner) = `UNSUPPORTED`
- **注意 — 本 opcode 定義以前の server**: 0x24/0x25 は値が非 critical なので、subscribe を知らない
  古い server は黙ってスキップし応答は OK になる (イベントは来ない)。対応有無の機械的判別が
  必要になったら `hello_resp` の末尾拡張 (feature bits) で扱う

### 意味論

- **検出は server の poll**: `RemoteServerService` の poll サイクルごとに購読ピンを読み、
  前回値と異なれば event を送る。**poll 幅より短いパルスは取りこぼす** (エッジ割り込みではない。
  ISR ベースの検出は将来ユニット)。通知は**変化のみ・両エッジ** (エッジ指定なし)
- **通知はベストエフォート**: event は応答確認を持たず、送信失敗時の再送もない。
  確実性が要る読み出しは従来どおり同期 `gpio_read` を使う
- **購読は接続単位**: `hello` (新しい接続の開始) で server の購読テーブルはクリアされる。
  再接続した host は subscribe をやり直す
- host 側は `RemoteSession::poll()` (アイドル時) と応答待ちループ (要求中) の両方で event を
  受信し、event script を自分の runner で実行する。`evt_gpio_state` の通知先は runner に登録した
  ハンドラ (未登録なら黙って無視)。ハンドラが受け取る `gpio_num` は**リモート側の番号空間**

## stream credit (連続書き込みのフロー制御)

I2S 再生のような**連続ストリーム書き込み**は、同期 request/response のロックステップでは
RTT 律速で帯域が出ない。そこで NORESP の書き込みを連射し、device 側バッファの状態を
credit として返すフロー制御をメッセージ層に置く (frame 層の予約 kind `credit` /
`credit_delta` は将来の mux/リンク層用に温存し、ここでは使わない)。

### wire

| opcode | payload | 意味 |
|---|---|---|
| `bus_write_stream` (0xB0, critical) | `[kind:1][bus_id:1][data...]` | 登録済み stream 系バス (現在 I2S) への書き込み。data 長は命令 size から自己記述。通常 **NORESP** で送る |
| `bus_stream_status` (0xB1, critical) | `[kind:1][bus_id:1][store_id:1]` | 応答スロットに `[free:u32][submitted:u32]` (LE) を格納。credit の初期化・再同期に使う |
| `evt_stream_credit` (0x61, 非 critical) | `[kind:1][bus_id:1][free:u32][submitted:u32]` | (イベント) device 側バッファの空きと累積受理量のスナップショット |

書き込み系 2 op が **critical** なのは意図的: これらを知らない古い server に黙殺されると
「応答は OK なのに無音」という追いにくい障害になるため、即 `PROTOCOL_ERROR` で検出させる
(イベント 0x61 は 0x60 と同じく非 critical — 知らない host は安全に読み飛ばす)。

### credit の意味論 — 絶対値ペアによる自己回復

- device は binding ごとに **`submitted` = これまで受理した累積バイト数** (mod 2^32) を持ち、
  `free` = いまブロックせず受理できる量 ([i2s.md](i2s.md) の `writableBytes`) とのペアで報告する
- host は自分の累積送信量 `sent` (mod 2^32) を持ち、報告を受けるたびに
  **`credit = free − (sent − submitted)`** を再計算する。`sent − submitted` =
  リンク上を飛行中でまだ device に受理されていない量 (u32 の modular 減算で wrap 安全)
- host は credit の範囲でのみ `bus_write_stream` を送る。これにより device 側では
  到着時点で必ずバッファに収まる
- **差分でなく絶対値**なので、イベントが失われても次のイベント/status で完全に回復する
  (累積ドリフトしない)。再接続・再構成時は `bus_stream_status` で `sent = submitted` に同期する
- **消失した write の回収**: NORESP write 自体も CRC 不合格・受信溢れで破棄され得る。消失分は
  `sent − submitted` を恒久的に膨らませ credit を収縮させるため、host は**同期
  `bus_stream_status` の応答受領時点で必ず `sent = submitted` に再基準化する** — in-order
  リンクでは、応答は「それ以前に送った write は全て処理済みか消失済み」を保証するので、
  応答時点の in-flight は 0 である。これにより定期再同期が消失からの回復点を兼ねる
- **イベント送出は server の poll**: 前回イベント時点からの消費量 (Δsubmitted + Δfree で
  capacity なしに計算できる) が閾値以上になったとき送る。ベストエフォート (GPIO イベントと
  同じ lossy 許容)。`hello` でイベント送出の管理状態はクリアされる (購読テーブルと同様、接続単位)
- **in-flight 窓**: host は credit に加えて未確認送信量 (`sent − submitted`) に上限を課す
  (`RemoteI2SBus::setStreamWindow`、既定 16 KiB)。credit は device の DMA 空きしか表さず、
  全 credit の一括バーストは device の UART 受信リング / FIFO を限界まで押し込み得る
  (実機で受信停止を誘発)。窓があってもスループットは落ちない — credit イベントが DMA
  排出レートで窓を補充し続けるため。窓の下限は
  `2 × イベント閾値 + レート × イベント到達遅延の上限` (実測例: 48 KB/s ストリームは
  4 KiB 窓で公称レートを維持)。device の UART 受信リングは窓以上に取る
- **credit 待ちの poll は 1 フレームずつ**: 窓/credit 枯渇で待つ側は「1 フレーム処理したら
  即 credit を再評価」しなければならない。イベントが rx timeout より速い周期で届き続ける
  リンクでは、複数フレームを束ねてブロッキング drain する poll が送信側を到着ペースで
  捕獲し (フレーム間で都度 first_byte timeout まで待つため)、その間 device が排出し切って
  ストップ&ウェイト化する — 小さいバッファ構成ほど顕著にレートが崩壊する
- **credit 違反** (空きを超える書き込みが届いた) は `BUFFER_OVERFLOW`。NORESP 書き込みの
  失敗なので pending error として次の同期交換で host に届く。**underrun はエラーではない**
  (device は無音を出力し続ける — [i2s.md](i2s.md))

## kind 別 proxy の意味論

I2C と同じく、各 proxy はローカルのインタフェースを実装する。kind 固有の差分:

- **SPI** (`RemoteSPIBus`): 転送形は I2C と同型。CS / トランザクション窓は server 側 accessor が
  **転送ごとに**実現する (ローカルの `beginTransaction` / `endTransaction` は no-op のまま)。
  複数転送に跨る CS 窓は非サポート — command / address / dummy は 1 つの `TransferDesc` に
  まとめて表現する
- **UART** (`RemoteUARTBus`):
  - `read` は 1 回あたり `kMaxTransferRx` まで (超過分は clamp — UART read はもともと short return
    が正常系)。リモート側の short read は応答スロット経由でそのまま伝わる
  - `write` は成功時に要求長を返す。bytecode v1 は書き込み済みバイト数を応答に載せないため、
    リモートの short write は区別できない (失敗は `report_error` で返る)
  - `readableBytes` は **UNSUPPORTED** (該当 opcode がない)。小さな
    `first_byte_timeout_ms` を設定した `read` でポーリングする
  - **timeout 合成**: session の応答 timeout は呼び出しごとに
    `first_byte_timeout_ms + (len-1) × inter_byte_timeout_ms + マージン` (read)、
    `write_timeout_ms + マージン` (write) まで一時的に引き上げ、呼び出し後に復元する。
    server 側は実行前検査でこれらの timeout を `max_bus_timeout_ms` 以下に制限している
    (§server の実行モデル) ため、合成後も有界
- **I2S** (`RemoteI2SBus`): `write` は §stream credit に従い NORESP 連射 + credit 待ちで送る
  (`write_timeout_ms` 内に送れた分を返す — short write はローカル同様正常)。`writableBytes` は
  ホスト側 credit 推定値を返す (同期 RPC しない)。AccessConfig の変更は次の write の前に
  `bus_configure` + `bus_stream_status` の同期 script で適用し、credit を再同期する。
  device 側の `write_timeout_ms` は 0 (non-blocking) を送る — 待ち合わせは credit が担うため
  server の poll を塞がない
- **GPIO** (`RemoteGPIO` + 内蔵 `RemotePort`): ホスト側 `GPIOGroup` の空き slot に I/O expander と
  同じ作法で登録し ([gpio.md](gpio.md))、得られた `Pin` ハンドルはローカルと同一に振る舞う
  - **番号変換**: ホスト側 local pin `n` は wire 上でリモート側の `makeGpioNumber(remote_slot, n)`
    に変換される (ホスト slot とリモート slot は独立した番号空間で、変換は proxy が担う)
  - `write` / `setMode` は **NORESP** で送る (`IPort` のフックは戻り値を持たないため)。
    リモート側の失敗は pending error となり、次の同期交換後に
    `session.lastRemoteError()` で観測できる
  - `read` は同期 RPC。失敗時は `false` を返し、エラーは同様に session 側で観測する
  - ハンドルは stale 化しない: transport 喪失中は操作が失敗する (read は false) だけで、
    `Pin` / registry は有効なまま。session の再確立後そのまま使える

## 接続ユーティリティ

接続まわりの責務は 3 層に分ける。transport の継ぎ目は [data_io.md](data_io.md) §Stream アダプタの
`StreamReader` / `StreamWriter` であり、メッセージ層以上はこの継ぎ目より上で transport 非依存になる。

| 層 | 役割 | 置き場所 |
|---|---|---|
| 候補列挙 | 名前ヒューリスティクスで接続先候補を有力順に出す | 各 transport の variant (posix は `listSerialPorts`、[uart.md](uart.md)) |
| 配線バンドル `RemoteLink` | `StreamReader`/`StreamWriter` → Stream アダプタ → `RemoteSession` の定型配線 | remote コア (transport 非依存) |
| 確立ユーティリティ | 候補列挙 × `hello` 疎通の合成ループ。**正しい相手かはプロトコルが決める** (名前は候補を出すだけ) | 各 transport の variant (posix は `connectRemoteSerial`) |

- `RemoteLink` は **scratch を内蔵する利便 (sugar) 層** (2 × `kMaxWireSize`)。バッファ管理を自分で
  行いたい呼び出し側は従来どおり `StreamSource` / `StreamSink` / `RemoteSession` を直接組む
- `RemoteSession::reset()` が「同じオブジェクトで次の接続を始める」入口。ハンドルは接続単位であり、
  transport の再接続・候補切替時は reset してから hello し直す
- posix の `connectRemoteSerial(SerialRemoteEndpoint&, ConnectOptions)`: 明示パス指定または自動探索
  (rank 上限つき、ノイズポート除外)。候補を open → reset → hello し、最初の応答者で確定する。
  プローブ用の短い timeout は確立後に構成値へ復元され、確定したパスは `devicePath()` で観測できる
- **hello 試行ごとの完全 flush**: リセット直後の相手は ROM ブートログ等のノイズを流すため、
  各試行の前に transport の受信キュー (`tcflush`) と Stream アダプタのバッファ
  (`StreamSource::discardBuffered`、`RemoteLink::reset()` 経由) の両方を破棄する。
  これを怠ると FrameReader の resync がノイズを「サイズ自己記述の偽フレーム候補」として
  読み進める過程で本物の応答まで消費し得る
- **接続時のハードウェアリセット (既定有効)**: 素の `open()` は DTR/RTS の遷移タイミングを制御せず、
  ESP32 系の自動リセット配線 (RTS→EN / DTR→IO0) ではこの競合で稀にボードが ROM ブートローダへ
  落ちたまま hello に応答しなくなる。`connectRemoteSerial` は open 後に決定論的な
  RUN モードリセット (DTR 解除 → RTS で EN をパルス) を打ち、毎回アプリケーション起動状態から
  hello を試行する (起動 ~1 秒は有力候補へのリトライで吸収)。modem 線を持たないポート (pty 等) では
  no-op。接続でリセットされては困る相手には `ConnectOptions::hardware_reset = false`
- transport を増やす場合 (TCP 等) は、候補列挙と確立ユーティリティをその variant に追加するだけでよい
  (`RemoteLink` 以上は不変)。マルチノード discovery への発展は「最初の応答者で確定」を
  「応答者を収集」に広げる形で同じ骨格に載る

## 安全境界

- **BytecodeRunner の登録制 dispatch が許可リスト (allowlist) を兼ねる**。server に登録したバス / GPIOGroup 以外にはリモートから到達できない
- GPIO を公開する場合は、**公開専用の GPIOGroup を別に組み、公開してよいピンの IGPIO だけを register する**。platform の GPIO 実装を丸ごと登録すると、flash 接続ピン (ESP32 の GPIO6-11 等) まで露出する
- **認証・認可は本仕様のスコープ外** (transport の責務)。物理 UART は物理アクセスを信頼境界とする。TCP 等の到達性が広い transport に載せる場合は、その transport 仕様で接続相手の検証を定めること

## エラー写像

- リモート起因のエラーとして `error_t` に `DISCONNECTED` (transport 切断) / `REMOTE_FAULT` (リモート内部エラー) / `UNSUPPORTED` (リモートが当該機能を持たない) を用いる
- `report_error` / `error` メッセージが運ぶ error code は i8。**host が知らない値は `REMOTE_FAULT` に写像する** (新しい server が新コードを返しても旧 host は壊れない)

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| 新しい frame kind (request/response 専用) | data + stream_id で表現でき、frame codec の改版 (未知 kind は `invalid_prefix` 扱い) を避けられる。予約 kind はリンク層 (mux / flow control) のために温存する |
| フレームをまたぐリモート排他 (remote lock) | 1 転送 = 1 自己完結 script (`bus_configure` + `bus_transfer`) とし、セッション状態の回復問題ごと排除する。複数転送のアトミック性は「1 script に複数命令を詰める」拡張で扱う |
| 既定での自動再送 | リモート操作は非冪等であり得る。再試行の判断は呼び出し側に委ねる |
| node addressing (wire 上のノード ID) | point-to-point 前提。マルチノードは stream_id 割当または mux 層の将来仕様で扱う |
| bytecode への認証埋め込み | 層が違う。transport の責務とする |

## 将来拡張 (方向性のみ)

- **チャンク転送**: MORE bit による分割。230/238 byte 上限を超える転送に対応する
- **リンク層 flow control**: 連続書き込みのフロー制御は §stream credit (メッセージ層) で導入済み。
  予約 frame kind (`credit` / `credit_delta` 等) は、複数 stream の mux / マルチノードが必要に
  なった時点でリンク層の仕様として定める。本メッセージ層は変更しない

## 互換性と版管理

- 本仕様 (M5HAL remote v1) は**実験段階**であり、公開リリースノートで凍結を宣言するまでは非互換変更があり得る
- メッセージ層の版は `hello` の `proto_ver` で判別する。凍結後の非互換変更は `proto_ver` を増やし、種別値・予約 bit の意味は再利用しない
- 前方互換の原則: 未知の種別・予約 bit 付きメッセージは破棄、`hello_resp` の末尾拡張は無視、bytecode の未知 opcode は critical bit に従う ([bytecode.md](bytecode.md))

## 関連

- 実例: [`examples/v1/HowToUse/RemoteBus`](../../examples/v1/HowToUse/RemoteBus/) — device 側 sketch (M5Stack Core BASIC、内蔵 I2C と許可リスト化したボタン GPIO を公開) + PC 側ホストプログラム (`host/remote_bus_host.cpp`)。接続は `SerialRemoteEndpoint` + `connectRemoteSerial` の 2 行 (§接続ユーティリティ)、以降リモートスキャン / レジスタ読み / リモートボタンの変化監視ループ
- 実例: [`examples/v1/HowToUse/RemoteI2S`](../../examples/v1/HowToUse/RemoteI2S/) — device 側 sketch (M5Stack Core2 V1.1、内蔵スピーカーの I2S TX を公開) + PC 側ホストプログラム (`host/remote_i2s_host.cpp`)。PC から 440Hz 正弦波を `RemoteI2SBus` で連続 write し、§stream credit の NORESP 連射 + credit フロー制御でリモート再生する
- 下層仕様: [data_io.md](data_io.md) / [frame.md](frame.md) / [bytecode.md](bytecode.md)
- バス / GPIO 抽象: [bus_accessor.md](bus_accessor.md), [i2c.md](i2c.md), [gpio.md](gpio.md)
- 検証: [../verification.md](../verification.md) (native gtest `test_remote` / posix pty end-to-end)
