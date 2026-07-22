# design/remote — リモートバス機構 (frame mux transport)

> **読者**: 実装者・レビュー向け（設計仕様）。

ホスト側のコードから、バイトストリーム (UART / USB CDC 等) の先にあるリモートデバイス上のバス / GPIO を操作するための機構。namespace は `m5::hal::v2::remote` (`src/m5_hal/hal/v2/remote/`)。

下層は frame mux 多重化層 (`data::MuxFrameEncoder` / `data::MuxFrameDecoder`、`hal/v2/data/mux.hpp`) と [bytecode.md](bytecode.md) の命令列・対称パイプラインを組み合わせる。**wire 上のフレーム形式 (LEN/KIND/CHECK8/B3/payload、kind 一覧、resync) は [frame.md](frame.md) が正本**。本仕様が定めるのは **frame KIND ごとのメッセージ意味論** (request/response の対応付け、能力交換、データストリーム、エラー伝播、安全境界) である。

なお `frame` / `bytecode` / `data::mux` は**リモート機構が使う独立プロトコル層**であり、remote 配下ではなく `hal/v2/` 直下に置く (単体テスト・build fence を持つ公開面。ソース配置は host 側 = `variants/frameworks/remote/`、protocol 契約と server 側 = `hal/v2/remote/` という 3 責務で分かれる)。

```
host                                      device
────                                      ──────
RemoteSession                             RemoteServerAdapter
  ├ MuxFrameEncoder → wire Sink (TX)        ├ MuxFrameEncoder → wire Sink (TX)
  └ MuxFrameDecoder ← wire Source (RX)      ├ MuxFrameDecoder ← wire Source (RX)
       └ frame handler (onFrame)            │    └ frame handler → RemoteServerHandler
                                            └ Server (processScript)
                                                 └ BytecodeRunner (登録制 dispatch)
```

- **host** は `RemoteSession` が request/hello/ping/reset の各フレームを `MuxFrameEncoder::writeFrame()` で送出し、応答フレームを `MuxFrameDecoder` の frame handler で受ける。応答スクリプトを自分の `BytecodeRunner` で実行して結果を回収する**対称パイプライン** ([bytecode.md](bytecode.md)) をそのまま使う
- **server** は `RemoteServerAdapter` が受信フレームを `RemoteServerHandler::handler` に渡し、`Request` フレームを `Server::processScript` で実行して `Response` フレームを返す受動エンドポイント
- 両端とも wire I/O は **`MuxFrameEncoder::pump()` / `MuxFrameDecoder::pump(wire_rx)`** で駆動する。`RemoteWireService` (§server の実行モデル) を使うと service poll ループの中で協調駆動される

## 目次

- [Transport 層 — frame mux 多重化](#transport-層--frame-mux-多重化)
  - [TCP トランスポート](#tcp-トランスポート)
  - [診断 (wire dump)](#診断-wire-dump)
  - [arduino ビルドの動的バス生成](#arduino-ビルドの動的バス生成)
  - [stream_id レジストリ](#stream_id-レジストリ)
- [メッセージ層 (frame KIND)](#メッセージ層-frame-kind)
  - [SEQ](#seq)
  - [hello — 最小能力交換](#hello--最小能力交換)
  - [request / response](#request--response)
  - [サイズ上限](#サイズ上限)
  - [timeout / resync](#timeout--resync)
- [server の実行モデル](#server-の実行モデル)
- [データチャネル](#データチャネル)
  - [BusStreamTransfer + attachStream](#busstreamtransfer--attachstream)
  - [I2C / SPI atomic transfer](#i2c--spi-atomic-transfer)
  - [UART stream timeout / short transfer](#uart-stream-timeout--short-transfer)
- [push イベント (GPIO 変化通知)](#push-イベント-gpio-変化通知)
  - [wire](#wire)
  - [意味論](#意味論)
- [GPIO ポート操作](#gpio-ポート操作)
  - [wire](#wire-1)
  - [意味論](#意味論-1)
- [動的バス生成 (BusCreate / BusRelease)](#動的バス生成-buscreate--busrelease)
  - [wire](#wire-2)
  - [意味論](#意味論-2)
- [安全境界](#安全境界)
- [エラー写像](#エラー写像)
- [採用しない要素](#採用しない要素)
- [Hal facade](#hal-facade)
- [互換性と版管理](#互換性と版管理)
- [関連](#関連)

## Transport 層 — frame mux 多重化

wireは **frame v1 codecの単一フレーム列**である。多重化は **frameのKINDと`Data`フレームの
B3 (= stream_id)** で表現し、別系統のchannel-id枠、CRC16形式、管理チャネルは持たない。

- `MuxFrameEncoder::writeFrame(kind, b3, payload)` が 1 メッセージ = 1 フレームを内部バッファへ encode し、`pump()` が出力 Source を介して wire Sink へ流す。writeFrame 系は `result_t<void>` で失敗理由を返す (allocator 未束縛 = `INVALID_STATE`、allocator 枯渇・block queue 満杯 = `OUT_OF_RESOURCE`、wire 形式違反 = encode 層のエラーをそのまま伝播)。呼び出し側は独自のエラー code へ読み替えず伝播する
- `MuxFrameDecoder::pump(wire_rx)` が wire からフレームを取り出し、`Data` フレームは stream_id 対応の Sink へ demux、それ以外の KIND は frame handler コールバックへ渡す
- `MuxFrameDecoder::pump(wire_rx)` は 1 呼び出しあたり最大 `kPumpMaxFramesPerCall = 32` frame で返る。密なインバウンドイベント列で応答待ち側へ制御が戻らなくなる livelock の防止が目的
- 最大 `kMaxStreams` (16) のデータストリーム
- **フロー制御**: 2 層構造。ハードな backpressure は従来どおり Source/Sink の `reserve`/`commit` が短く返すことで伝わる (受信側が詰まると decoder が wire を advance せず送信が滞る)。加えて `RemoteSession` / `RemoteServerAdapter` / `RemoteWireService` は **credit 通知** (`Credit` KIND、[frame.md](frame.md)) を自動送出する:
  - B3 = 受け入れ可能な Data frame 数の**絶対値** (受信側 TempBuffer pool の空きブロック数と、block-mode stream の残キュースロット (`BlockSource::kMaxBlocks` 基準、active な block stream の最小値) の小さい方。255 で飽和。heap fallback 分は数えない)
  - 値が前回送信値から変化した pump 時のみ送信 (セッション開始後の最初の pump では必ず 1 回送る)
  - 送信側 `MuxFrameEncoder` は**最初の `Credit` 受信で gating を有効化**し (未受信なら無制限 = 従来と後方互換)、Data frame を 1 つ生成するごとに 1 減算、0 で Data 生成を停止する。`writeFrame()` (制御 frame) は gating 対象外 (制御面が詰まると deadlock するため)
  - 絶対値方式は loss-tolerant (通知が欠落しても次の通知で自己回復) だが、in-flight frame の分だけ一時的な過送出があり得る。credit は保守的ヒントであり、ハードな上限は上記 wire 停止 + pool 限定割当が担保する

### TCP トランスポート

TCP は bus kind ではなくトランスポートである。確立後のソケットは `detail::BsdTcpStream`
(`StreamReader`/`StreamWriter`) として UART と同じ `StreamSource`/`StreamSink` に載り、
frame codec 以上の層は transport を区別しない。

host側もUART/TCP固有classはbyte streamのopenだけを担い、session構築、hello decode、backend / GPIO
投影、service poll、teardownは`PosixRemoteConnectionCore`へ集約する。`Hal`へのconnection入替えも
transport別経路ではなく単一のadopt手順を通し、旧service / GPIO / sessionのcleanup順を揃える。

- **リンク層は責務外**: WiFi / Ethernet (内蔵 EMAC / SPI Ethernet) の bring-up はアプリの
  責務。lwIP netif に合流した後の BSD socket API は同一のため、本層は netif 非依存。
- **接続確立のみ非対称**: host = `PosixTcpConnection::create` (dial)、device =
  `remote::BsdTcpRemoteServer` (listen/accept)。確立後は対称。
- **マルチ接続**: `BsdTcpRemoteServer` は接続ごとに独立の `Server` (bus_id 名前空間・
  capabilities) と `ServerBusPool` を持ち、物理バスのみ `ServerPhysicalBusPool` で共有する。
  同一 pin_config (wire 正規形のバイト一致) は refcount 共有、pin 一致・設定不一致は
  `INVALID_STATE`。スクリプト実行は単一 service ループでフレーム単位に直列化される。
- **接続数 query**: `connectionCount()` は、別 thread が `service()` を駆動している間にも監視側から
  呼べる best-effort snapshot である。返却値は呼出し時点付近の active slot 数であり、slot 内の
  stream / server / adapter の生存期間を固定する barrier ではない。これらの payload は引き続き
  service thread だけが触る。
- **切断 = lease 解放**: stream の `CLOSED`/`IO_ERROR` または adapter エラーでその接続の
  `releaseAll()` が走り、bus lease と物理 refcount が戻る。他接続は影響を受けない。
- **信頼境界**: 認証・暗号化は持たない (§安全境界の transport 責務の原則どおり)。
  信頼できる LAN / loopback での利用を前提とする。

### 診断 (wire dump)

host 側の posix transport (UART / TCP) はバイト列キャプチャを持つ
(`variants/frameworks/posix/hal/remote/wire_dump.hpp`)。ビルドフラグは持たず、環境変数
`M5HAL_WIRE_DUMP=<path>` のみで有効化する (host 専用コードなので runtime ゲートで十分)。

- `path` は追記 (append) モードで開く。書き込み失敗時は該当接続のキャプチャが無効になるだけで、
  接続自体は失敗しない
- 1 レコード = 1 行: 単調クロックの秒数・transport タグ (`uart`/`tcp`)・向き
  (`<`=device→host、`>`=host→device)・バイト数・全量 hex ダンプ
- **全 transport 共通の tap**: `PosixUartConnection` / `PosixTcpConnection` のどちらも汎用の
  `data::TapReader`/`data::TapWriter` ([data_io.md](data_io.md) §Tap 装飾) + `WireDumpWriter`
  (hex 行整形 StreamWriter) で RX/TX ストリームを包む。未設定時 (`M5HAL_WIRE_DUMP` 未定義)
  は仮想呼び出しと null チェックだけの純粋な転送になる

### arduino ビルドの動的バス生成

arduino ビルドの server pool は、I2C/SPI の物理バスを Arduino のグローバルインスタンス
(`Wire`/`Wire1`、`SPI` + 空きコントローラ) から確保する (owned: pool が begin/end を所有)。
加えて `ServerPhysicalBusPool::adoptI2C` / `adoptSPI` で**外部初期化済みインスタンスを
実配線ピンの申告付きで登録**できる (adopted: pool はピン割当・begin・end を行わず借用のみ。
refcount が 0 に戻っても解体せず保持する)。

- adopt は「登録 = 公開」を意味する。内部バス (電源 IC 等) を晒すかは §安全境界の
  登録原則に従いアプリが判断する。
- adopt したバスへのアプリ側アクセスも M5HAL アクセサ経由に揃える (アクセサロックの
  外から直接叩かない)。
- ピン申告と実配線の不一致は検出できない (アプリローカルの契約)。
- 申告したピンはpoolの照合だけでなく、借用backend自身のbus configにも保存される。SPIの
  MISO有無など、転送前の配線capability判定はこの申告値を使う。
- 転送パラメータ (クロック・タイムアウト) はアクセサ契約に従い転送ごとに適用される —
  adopt が保証するのは「バスの存在と配線に触れない」ことのみ。

### stream_id レジストリ

`Data` フレームの B3 が **stream_id**。リモートメッセージ (Request/Response/Hello*/Ping/Pong/Control) は `Data` ではなく**独立した frame KIND**として送られるので、stream_id 空間を消費しない (旧 mux の「制御 = ch1 / データ = ch2+」のような予約チャネルは無い)。

| stream_id (Data B3) | 用途 |
|---|---|
| 0..15 | データストリーム (バスデータの直接転送)。`RemoteSession::attachStream` が空きを動的に割り当てる (§データチャネル) |

- 制御系メッセージは frame KIND で区別されるため、stream_id の予約値は無い
- frame KIND の値・B3 の意味の正本は [frame.md](frame.md) §kind 一覧

## メッセージ層 (frame KIND)

各リモートメッセージは固有の frame KIND を持つ。B3 と payload の意味は KIND ごとに定まる。

| メッセージ | frame KIND | 方向 | B3 | payload |
|---|---|---|---|---|
| Hello | `HelloReq` | host → device | seq | 空 |
| HelloResp | `HelloResp` | device → host | seq | 能力一覧 (下記) |
| Request | `Request` | host → device | seq (+ NORESP) | bytecode script |
| Response | `Response` | device → host | seq | 応答 bytecode script (`StoreData*` + `Report*`) |
| Ping | `Ping` | host → device | seq | 空 |
| Pong | `Pong` | device → host | seq | 空 |
| Error | `Control` | device → host | seq | `[error:i8]` |
| Reset | `Control` | host → device | seq | 空 |
| Event | `Event` | device → host | event seq | イベント bytecode script |
| Data | `Data` | 双方向 | stream_id | ストリーム本体 (§データチャネル) |

- **`Control` KIND は方向と payload で役割が分かれる**: host→device の空 payload = デバイスリセット要求 (device は空 `Response` を書いてから再起動、host API は応答を待たない)、device→host の 1 byte payload = エラー報告 (`[error:i8]`)
- **NORESP request**: `Request` の B3 (seq) の bit7 を立てる (`seq | 0x80`)。server は `processScript` を実行するが応答フレームを返さない (fire-and-forget)。実行エラーは捨てられる
- **fire-and-forget 送信の drain**: `NORESP request` と host→device reset `Control` は、送信後に応答待ち read が発生しない。host 実装は `Bus_posix` の TX coalescing のような「次回 read で flush」型 transport でもフレームを滞留させないよう、送信直後に wire pump / read-before-flush 相当を 1 回走らせる
- 未知 KIND・解釈不能なフレームは frame codec / handler が黙って読み飛ばす ([frame.md](frame.md) §decode の意味論)。新旧実装の混在で安全側に倒す

### SEQ

- **7 bit 循環** (`(seq + 1) & 0x7F`、0..127)。B3 の bit7 は NORESP フラグ専用に予約されており、採番カウンタ自体はそこへ溢れない。host 発のメッセージ (`HelloReq` / `Request` / `Ping` / `Control`(reset)) ごとに採番し、B3 の下位 7 bit に載せる
- device は対応する応答 (`HelloResp` / `Response` / `Pong` / `Control`(error)) に**同じ SEQ を返す**。host は B3 (seq) と期待 KIND の双方が一致する応答のみ受理する
- **応答順序は要求順を保証しない**。server はstream transferをpendingにしたまま後続Requestを処理できるため、
  後続の同期応答が先行し得る。clientはSEQと期待KINDで対応付ける。標準`RemoteSession`は現在
  **1 in-flight**。Hal facade とその proxy は session 単位の operation gate で、SEQ 採番、
  stream attach/detach、request、response 解釈、config cache 更新までを 1 RPC として直列化する。
  これは bus/channel mutex とは別の排他境界で、lock 順序は bus/channel → session。
  UART/I2S の TX/RXを別taskから呼ぶこと、およびPDM RXを別taskから呼ぶことは安全だが、標準sessionでの
  wire RPCは直列に進む。
  真に 1 RPC 内で全二重転送する場合は複合 `transfer()` を使う。multi-in-flight client は別設計とする
- NORESP request は seq の bit7 に NORESP フラグを載せる (server 側で `(seq & 0x80) != 0` を判定)
- stream_id の検疫中 (§timeout / resync) は、host はその検疫エントリが待つ seq を採番から除外する (`RemoteSession::nextSeq()`)。除外しないと 128 循環で同じ seq が別の要求に再割当てられ、その応答が無関係な検疫を誤って解放しうる

### hello — 最小能力交換

接続確立後に host が最初に送る。`HelloResp` の payload:

```
[proto_ver:1][flags:1][n:1]([bus_kind:1][bus_id:1]) * n  [gpio_port_count:1][gpio_pin_count:u16]? [extension_envelope]?
```

- `proto_ver` = 1 (本版)。host は自分の知らない版を `UNSUPPORTED` として扱う
- `flags` bit0 = GPIO 公開あり、bit1 = `supports_bus_create` (動的バス生成対応 / ServerBusPool あり)、
  bit2 = Bus instance capability extensionあり。他 bit は予約 0
- `(bus_kind, bus_id)` は静的登録バス一覧 (`n` 件)。動的バスプールのみの server は `n = 0` を返す
- `flags` bit0 が立つときのみ末尾に `[gpio_port_count:1][gpio_pin_count:u16 LE]` が続く
- bit2が立つ場合、既知prefixの後に`[envelope_len:u8]`と、その長さ内の
  `[tag:u8][len:u8][body:len]` TLV列が続く。tag 1のbodyは
  `[entry_count:u8]([bus_kind:u8][bus_id:u8][record_len:u8][record])*`で、recordは
  [bus_capabilities.md](bus_capabilities.md)のschemaである
- hostは未知tagをlengthでskipする。bit2なしの余剰tailは旧host互換のため無視する。bit2ありで
  envelope/TLV/recordがtruncateまたは長さ不整合なら`PROTOCOL_ERROR`
- serverは全静的entryを一つのbounded payloadへ収められる場合だけbit2とextensionを付ける。収まらない場合は
  旧prefixを変えずextension全体を省略する
- 末尾へのフィールド追加は前方互換とする。旧hostは既知prefixだけ読み、extensionを無視する。新hostは
  bit2なしの旧Helloを保守的な空instance snapshotとして受理する
- protocol v1 の hello は board ID を運ばず、backend の `variant_id_t` に board を混在させない。将来
  device 固有情報が必要になった場合も、既存フィールドの意味を変えず hello 末尾の独立した
  device descriptor として additive に追加できる
- host 側のデコードは `detail::decodeHelloCaps`、結果は `Capabilities` 構造体に格納する

### request / response

- `Request` の payload は bytecode script そのもの。server は実行し、`Response` の payload に応答 script (`StoreData*` + `ReportError`/`ReportComplete` + 終端) を返す
- host は応答 script を自分の BytecodeRunner (受信専用) で実行し、store スロットと report を回収する
- protocol レベルの拒否 (handler 不在・サイズ超過・実行前検査の不合格など) は `Response` の代わりに `Control` フレーム (`[error:i8]`) で返す。host はこれを `mapRemoteError` で `error_t` に写像する

### サイズ上限

1 フレームの payload 上限は `frame::kMaxPayload` (252) で、これがメッセージ payload 上限を兼ねる:

- `kMaxScriptSize` = `frame::kMaxPayload` = **252** — request script の上限。`RemoteSession::request` が組み立て時に検査し、超過は `INVALID_ARGUMENT` で拒否する
- **`kMaxTransferRx` = 244** — Response 1 フレームで持ち帰れる受信データ上限 (`StoreData` 命令と `ReportComplete` のオーバーヘッド 8 byte を差引)

### timeout / resync

- host は応答 timeout (`Config::response_timeout_ms`) 時、`TIMEOUT_ERROR` として呼び出し側に返す。**自動再送は行わない** — リモート操作は非冪等であり得るため、再試行の判断は呼び出し側に委ねる
- **stream_id 再利用ハザードと検疫**: `Data` フレームには世代も要求 seq も乗らない (B3 = stream_id のみ、§stream_id レジストリ)。host 側の応答 timeout は device がその stream_id への書き込みを止めたことを意味しない (server 側の処理はまだ継続中のことがある)。timeout した stream_id を即座に空きへ戻すと、旧転送の残骸 `Data` が次に同じ id を割り当てられた新しい転送の受信 Sink へ無警告で混入し得る。host はこれを避けるため、timeout した stream_id を**検疫**に置く: encoder/decoder の attach は即座に外す (以降その id 宛の `Data` は宛先を失い無害に破棄される) が、id 自体は `attachStream` の割当対象から外したまま保持する。検疫の解放は次のいずれかで起きる:
  1. timeout した要求と同じ seq を持つ `Response`/`Control` をその後観測する。wire は FIFO (点対点の TCP/UART) なので、device がその要求に対応する終端フレームを書き終えた時点で、それより前に送出された `Data` は全て通過済みと言い切れる — これが正確な解放点
  2. `Config::stream_quarantine_ms` の保険タイマー (既定 6000ms、無活動タイマー)。終端フレーム自体が失われる wire ノイズ等のケースを回収する。server 側の pending timeout もチャンク毎に活動時刻を更新する無活動タイマーであり (固定寿命ではない)、host 側もそれに合わせて検疫中の id 宛 `Data` を観測するたびにこのタイマーを更新する — さもないと進行の遅い転送が固定期限より先に切れて再利用ハザードが再発する
  3. `hello()` の成功。セッション再確立で全検疫を一掃する。**`reset()` は検疫を一掃しない**: host 実装は device のリセット応答を待たない fire-and-forget であり (上記 `Control` の項)、送出できたことは device が旧転送を停止した証拠にならない
  - 16 本すべてが検疫中の間は `attachStream` が新規割当を拒否し `0xFF` を返す (静かな id 再利用より安全側)
- 接続喪失の検出は `ping` の失敗、または下層 stream のエラーによる
- wire 上の破損からの再同期は frame codec の Delimiter / resync が担う ([frame.md](frame.md) §decode の意味論)

## server の実行モデル

- `RemoteServerAdapter` が受信フレームを `RemoteServerHandler::handler(ctx, kind, seq, payload, enc)` に渡し、KIND で分岐する (HelloReq → 能力応答、Ping → Pong、Control → reset、Request → `Server::processScript`)
- ハンドラは応答を `MuxFrameEncoder::writeFrame()` で書き、`flushTx()` が wire Sink へ送る
- **実行前検査**: 命令は全て長さ前置なので、実行前に script を 1 パス走査できる。`DelayMs` が server 設定の `max_delay_ms` を超える script は実行せず `ReportError(INVALID_ARGUMENT)` で拒否する (黙って切り詰める clamp はしない)。`BusConfigure` が運ぶ timeout は server 設定の上限で制限する
- したがって server の **worst-case ブロック時間 = `max_delay_ms` の総和 + 制限後の bus timeout** で有界になる
- `RemoteWireService` (`service::IService` 実装) を `ServiceRunner` に登録すると、frame mux の wire I/O (encoder pump → wire Sink、wire Source → decoder pump) が service poll ループの中でバスサービスと協調駆動される

## データチャネル

request/response は RTT に律速される。連続データ転送 (I2S ストリーミング等) には `Data` フレームのストリームを直接使う。

remote proxyはwire正規形にした実効`AccessConfig`が、そのproxyで前回成功確認した
設定と異なる場合だけ`BusConfigure`を送る。UART / I2S / PDM の stream transfer は同じ bytecode
script内で`BusStreamTransfer`の直前に置く。I2C / SPI の atomic transfer は transfer payload 上限を
初回だけ縮めないため、独立した bus transfer を伴わない request で configure を成功させてから`BusTransfer` requestを送る。
初回transfer、`Hello`後、前回requestがエラーになった場合も送る。設定が同一なら省略するが、
各transferで渡されるaccessorの`AccessConfig`が実効設定の正本である。server側の動的バス作成時に
置くaccessor初期値は最初のconfigureまでのfallbackとなる。`TransferDesc`側のmetadataは設定cacheと
別に毎transfer送る。

SPI の `BusConfigure` では `pin_cs == -1` / `pin_dc == -1` を「server 登録時の物理 pin 設定を保持する」
sentinel として扱う。remote host は device の物理配線を知らないため、既定値を送って CS / D/C を無効化
してはならない。非負値だけが server 側設定を明示 override する。

### BusStreamTransfer + attachStream

ストリーム確立は bytecode opcode と frame mux の attach を組み合わせる:

`BusStreamTransfer` は UART / I2S / PDM の stream 系 kind 用である。非zero長の I2C / SPI request は、
frame chunkやSourceの短い`peek()`ごとにSTART/prefixまたはcommand/address/dummy phaseを再適用して
単一transferの意味を変えるため、server prescanがbus副作用前に`UNSUPPORTED`で拒否する。

1. host が `RemoteSession::attachStream(tx_source, rx_sink)` を呼ぶ。空き stream_id (0..15) を割り当て、tx 側を `encoder.attach(id, *tx)`、rx 側を `decoder.setSink(id, *rx)` に登録する
2. host が `BusStreamTransfer` (0x12) opcode を含む Request を送る。payload = `[kind:1][bus_id:1][stream_id:1][meta_size:1][tx_len:u32 LE][rx_len:u32 LE][meta]`
3. device 側 BytecodeRunner の `BusStreamTransfer` ハンドラ (`_stream_transfer_fn`) が `StreamTransferDesc` を受け、同じ stream_id にバスサービスを紐付ける
4. 以降、`Data` フレーム (B3 = stream_id) が両端の attach 済み Source/Sink 経由でバスデータの直接パスを提供する。`pump()` がフレーム化と demux を駆動する
5. 終了時は host が `RemoteSession::detachStream(stream_id)` で encoder/decoder の登録を外す。host 側で応答が確定しない終了 (応答 timeout に限らず、enqueue 後の曖昧な失敗全般) は代わりに `RemoteSession::quarantineStream(stream_id, seq)` を使う (即座の再割当ハザード — §timeout / resync)。`seq` はその要求が使った seq そのもの (`request()` の out-param) を渡す

**stream transfer は script の最終命令でなければならない** (`tx_len > 0` または `rx_len > 0` のとき)。転送が pending になると Response は遅延し、遅延 Response はslot 0の
`StoreData([actual_tx:u32 LE][actual_rx:u32 LE])`と`Report*`だけを運ぶ — 後続命令がresponse slotへ
格納したデータは応答に乗らない。このためdevice側prescanはpendingになるstream transferの後に
命令が続くscriptを実行前に`INVALID_ARGUMENT`で拒否する (`tx_len == rx_len == 0` のinline実行は
対象外)。実長を持たない旧protocol v1 serverに新hostが接続した場合だけ、互換動作として要求長を
実長とみなす。実長が要求長を超える応答は`PROTOCOL_ERROR`である。

転送の完了条件は `tx_consumed >= tx_len` **かつ** `rx_produced >= rx_len` である。device 側 stream
実行はチャンク単位で write/read を呼ぶため、転送全体にわたる TX+RX 両チャネルロック保持や真の
同時性は wire 越しには保証されない (ローカル既定実装も方向独立の逐次合成であり、remote だけが
劣るわけではない)。

### I2C / SPI atomic transfer

I2C / SPI remote proxyはinline `BusTransfer` opcodeを使い、公開1 transferをserver backendの
1 `transfer()` callへ写像する。host側Sourceはbounded scratchへ要求長を集約してからscriptへ埋めるため、
Sourceが短いspanを返してもwire transactionは分割されない。responseの`StoreData`はSinkへ順次commitする。
集約中にSourceが空になった場合、`closed()`または`eof()`なら`BUFFER_UNDERFLOW`、openならretry可能な
`WOULD_BLOCK`としてwire送信前に返す。現在のatomic APIはSource到着待ちの期限を別に持たないため、暗黙に
pollや無期限待機を行わない。
zero-length の transfer も script として送る。device は zero-length チャンクを 1 回実行し、リモート
`probe` の開通確認はこの経路を使う (NACK はエラーとして返る)。

protocol v1 frame内にrequest/responseを収める保証上限は次の通り。これを超えるatomic要求は
`UNSUPPORTED`であり、暗黙分割しない。

| kind / direction | 上限 |
|---|---:|
| I2C TX data | `243 - TransferDesc::prefix_len` byte |
| SPI TX data | 229 byte |
| I2C / SPI RX data | 244 byte |

I2Cのprefix、SPIのcommand/address/dummyはdata countに含めない。大容量atomic transferを将来提供する場合は、
frame分割とは別にbus operation continuation protocolを追加する。

### UART stream timeout / short transfer

UART remote proxyはUARTの設定値からhost側の応答期限を保守的に求める。TXは最悪1 byteずつ
進む場合を含めて`tx_len * write_timeout_ms`、RXは
`first_byte_timeout_ms + (rx_len - 1) * inter_byte_timeout_ms`をnominal期限とし、複合transferでは
両者を加算する。最後にtransport往復margin 250 msを一度だけ加え、32 bit overflowと
`TIMEOUT_FOREVER` sentinelを避けて飽和させる。

他のproxy busも固定定数でなく同じ方式で応答期限を実設定から導出する。I2C/SPIはclock周波数から
求めた期待wire時間をnominalとし（I2Cは9 bit/byte + address相当バイト、さらにserverのtransaction
予算に対応するSCL stall許容 `wire_timeout_ms` を加算）、I2S/PDMはPCM実時間
（sample_rate/bits/channelsから算出。全二重I2Sは両方向の長い方）にDMA待ち予算
（`write_timeout_ms` / `read_timeout_ms`）を加える。margin・飽和の扱いはUARTと同一。
PCM導出はpayload実効レートがtransportレートを下回ることを前提とする（現行サポート形状では成立。
より高速な形状を追加する場合は再検討する）。

device側のpending stream timeout（既定5000 ms、進捗ごとに更新）はData欠落や停止したpeerを
回収するtransport安全網であり、UART readの論理期限ではない。UART backendが要求長未満を正常に
返した時点でRXを完了し、上記の実長をhostへ返す。frame上限を跨ぐreadでは最初のchunkだけ
`first_byte_timeout_ms`、後続chunkの最初の待ちには`inter_byte_timeout_ms`を使うため、chunk境界で
first-byte期限を再開しない。

TX Dataはhostの`Source`をadvanceしながらwireへ流れるため、remote TXは要求したstreamを最後まで
受理させるexact operationとする。device backendの正のshort writeは未受理suffixへ再試行する。
これは未受理suffixをcallerへ残せるlocal UARTのshort-write契約との、stream transport固有の差である。
0 byteのtimeout/errorは通常どおりerrorとなり、自動的にrequest全体を再送しない。

ストリーミングは`BusStreamTransfer`と`Data`フレームのstream_idだけで表現する。
`BusWriteStream` / `EvtStreamCredit`のような専用opcodeや、channel-idベースの
`ChannelBind` / `MuxDataPump`はwire契約に含めない。

## push イベント (GPIO 変化通知)

host の GPIO read は通信せずキャッシュを読む。device 側で購読された GPIO が変化した場合は、
`frame::Kind::Event` (`0x15`) で event bytecode script を push し、host 側の
`RemoteSession::setEventHandler` が受けた script を `BytecodeRunner::runEvent()` で処理する。
**イベント本体も bytecode script** (対称設計): `EvtGpioState` は非 critical opcode なので、
購読機構を持たない古い host も安全に読み飛ばせる。

### wire

| opcode | payload | 意味 |
|---|---|---|
| `GpioSubscribe` (0x24) | `([gpio_num:u16])*` | 列挙ピンを購読に追加 (重複は冪等)。公開 GPIOGroup で有効なピンは購読時点のレベルを基準とし、初期 snapshot を `EvtGpioState` event で 1 回通知した後、**以後の変化から**通知する |
| `GpioUnsubscribe` (0x25) | `([gpio_num:u16])*` (空 = 全解除) | 購読解除 |
| `EvtGpioState` (0x60) | `([gpio_num:u16][level:u8])*` | 変化したピンと新レベル (複数ピンの同時変化は 1 命令にバッチされ得る) |

- イベント通知の payload = `EvtGpioState` 命令を含む script。payload の末尾拡張は前方互換
- subscribe 成功時の初期 snapshot も `EvtGpioState` を使う。これは edge ではなく state update であり、
  host cache の初期化 / 再同期に使う
- subscribe の失敗は通常の response で返る: 購読枠の枯渇 = `OUT_OF_RESOURCE`、
  購読機構を持たない server (handler 未設定の runner) = `UNSUPPORTED`
- Hello は現状 port 数 / pin 数だけを運び per-pin allow mask を持たないため、公開 GPIOGroup で無効な pin
  (deny mask 対象や未登録 pin) は購読対象から除外される
- **購読は接続単位**: `hello` で server の購読テーブルと GPIO 変化監視マスクはクリアされる

### 意味論

- **検出は server の poll**: server 内部の購読状態は `(slot, port_index, subscribed_mask, last_value)` の
  port 単位。poll サイクルごとに購読 port を `readPort()` で 1 回読み、`subscribed_mask` に含まれる
  変化 bit を `EvtGpioState` の pin/level event に展開して送る。**poll 幅より短いパルスは取りこぼす**
  (エッジ割り込みではない)
- **変化通知は `GpioSetMode` 済み pin だけが対象**: 変化 event の対象は、remote API の `GpioSetMode`
  を明示実行した pin を port 単位の監視マスクに記録し、`subscribed_mask` と AND した範囲に限る
  (opt-in)。mode 値そのものは監視マスクの on/off 判定に使わず、明示的に mode API を通った事実だけを
  使う。I2S / SPI など bus backend が駆動する pin はこの GPIO mode API を通らないため、既定では
  変化 event の対象外になる。動的バス生成が成功した場合、そのバスが占有する pin は監視マスクから
  除外される。`BusRelease` では自動復帰しないため、再び監視したい場合は `GpioSetMode` で明示的に
  opt-in し直す
- **初期 snapshot は subscribe request の成功処理中に送る**: `GpioSubscribe` を含む request が成功すると、
  server は response の前に購読対象 pin の現在値を `Event` frame として送る。host は接続時に event handler
  を設定してから subscribe するため、この snapshot で cache を更新できる。snapshot は host cache の seed
  であり、`GpioSetMode` 監視マスクではフィルタしない。encoderのqueueまたは一時allocatorが輻輳して両frameを
  保持できない場合は、terminal `Response` のblockを先に予約してsnapshot `Event` を省略する。これにより
  subscribeの成立可否は必ず`Response`でhostへ届き、best-effort eventがhost timeoutを引き起こさない。
  stream transferでResponseが遅延する間もencoderの最終1枠をResponse用に空け、GPIO eventは保留せず省略する
- **通知はベストエフォート**: event は応答確認を持たず、送信失敗時の再送もない
- **観測対象ピンの入力有効化は利用者の責務**: subscribe / port read はピンの pad 設定を**暗黙に変更しない**
  (ユーザーが指示していない GPIO モード変更を勝手に行わないという設計方針)。
  入力バッファが無効なピン (例: ESP32 は電源投入後 IE=0 がデフォルト) は物理レベルに関わらず
  IN レジスタが常に 0 を返すため、subscribe/watch/ポート読みは**先に該当ピンを入力有効
  (`gpio mode <pin> in` 相当の GpioSetMode) にしてから**行うこと。出力駆動中ピンのループバック観測を
  したい場合も、そのピンの入力パスが有効なmodeを利用者が指定する。variant は mode API で Output 系を
  指定したピンも入力パスを有効に保つ。ESP-IDF variant と ESP32 上の Arduino variant は Output 系を
  `GPIO_MODE_INPUT_OUTPUT[_OD]` にマップする。非ESP Arduino coreでのOutput readはcore自身の
  `digitalRead()` 契約に従い、remote serverの保証対象ではない (対応matrixはvariants.md)
- `RemoteServerAdapter::service()` は GPIO poll hook を受信 pump の前後で呼ぶ。汎用 server はこの hook から
  `RemoteServerHandler::poll()` を実行する
- host 側は `Hal::pumpRemote()` で remote connection service と GPIO watcher dispatch を協調駆動できる。
  `remote::PumpConfig::keepalive` を有効にすると、USB-JTAG 汎用 server が idle で止まらないよう間引きされた
  `Ping` も同じ入口から送る。ただし prompt で何もしていない間まで常時 fresh な background pump を保証するものではない

## GPIO ポート操作

ピン列操作 (0x20-0x26) に加え、`IPort` の 32-bit レジスタを一括操作するポートレベル opcode を提供する。

### wire

| opcode | payload | 意味 |
|---|---|---|
| `GpioPortRead` (0x27) | `[store_id:1][slot:1][port_index:1]` | ポート全ビット読み出し。`deny_mask` 対象ビットは 0 にマスク。結果 u32 LE をスロットへ |
| `GpioPortWrite` (0x28) | `[slot:1][port_index:1][set_mask:u32 LE][clear_mask:u32 LE]` | W1TS/W1TC 方式の一括書き込み。`deny_mask` 対象ビットはマスクされて書き込まれない |

### 意味論

- `slot` は `GPIOGroup` の GPIO スロット (登録済み `IGPIO` を識別)
- `port_index` は`IGPIO::getPort()`のlogical port ordinal。個別pinとの対応は`IGPIO::locatePin()`が決める (ESP32ではport0 = GPIO0-31、port1 = GPIO32以降)
- **deny_mask**: `GPIOGroup` が`(slot, port_index)`ごとに管理。個別pinのsubscribe/monitorとport opcodeは同じ`locatePin()` mappingを使う。`GpioPortRead`は読み出し値を`~deny_mask`でマスクし、`GpioPortWrite`はset_mask / clear_maskの両方を`~deny_mask`でマスクしてから適用する
- **非 read-modify-write 契約**: `GpioPortWrite` は `set_mask` / `clear_mask` 以外のビットへ影響してはならない。実装はポート値の read-modify-write を行わず、set/clear 専用レジスタ等のアトミック経路で書く (並行する pin 書き込み・watcher poll と競合してもマスク外のビットを壊さない)

## 動的バス生成 (BusCreate / BusRelease)

静的登録 (`registerI2C` 等) に加えて、host から**実行時にバスを作る**仕組み。device がボード固有のピンマップを持たず、host がバス構成を決める「汎用ペリフェラルサーバ」パターンを可能にする。

### wire

`BusCreate` (0x92) / `BusRelease` (0x93) は critical opcode。非対応サーバは `PROTOCOL_ERROR` で明示拒否する。pin_config のレイアウトは [bytecode.md](bytecode.md) §BusCreate の pin_config payload。

### 意味論

- **server 側**: `Server::setBusCreateHandler` でアプリ提供のコールバックを登録。コールバック内で物理バスを構築し `registerI2C/SPI/UART/I2S/PDM` で binding に追加する。handler 未設定で `BusCreate` が届いた場合は `UNSUPPORTED`。callback 成功後に binding / capability 登録または response store が失敗した場合は同じ callback の release を呼び、失敗した要求から hidden binding / pool entry を残さない。補償 release も失敗した場合は、その release error を応答する
- **custom handler の identity 契約**: host は応答喪失時に create / release の到達を確定できない。
  `setBusCreateHandler` の実装は、同一 kind + core pin identity の create を同じ物理bus/lockへ
  internして参照計数し、存在しないbindingのreleaseを冪等成功にする。標準
  `ServerPhysicalBusPool` はこの契約を満たす。応答喪失を理由に別の物理busを同じ配線へ作る
  handlerは非対応である。
- **hello との関係**: `setBusCreateHandler` 登録済みであれば `HelloResp` の `flags` bit1 が立つ。Hello 時に全動的バスを自動 release する (新規接続は白紙から開始)
- **bus_id 空間**: kind 別に独立 (I2C bus_id=1 と UART bus_id=1 は無関係)。上限 = `kMaxBusBindings` (4)。超過は `OUT_OF_RESOURCE`
- **host registry identity**: connection ownerが発行するnonzero monotonic 64-bit session generationと
  exact targetを`Remote` tagの`ResourceKey`へ含める。同じpin targetでもreconnect後は別keyとなり、旧sessionの
  bus ID / targetが新proxyへABA再利用されたものとしてinternされない。generation発行はconnection構築時だけで、
  transfer hot pathには入らない
- **host proxy config**: portable acquire は要求 `IBusConfig` を、logical acquire は送信済み `pin_config` から復元した core pins / buffer / role を proxy の `getConfig()` に保持する。close と同一 identity 再取得時の設定比較はこの proxy config を正本にし、wire に乗るフィールドのみ・wire 単位に正規化して行う。native policy は wire に載せず、peer へ `BusCreate` を送る前に `UNSUPPORTED` で拒否する。
- **create の曖昧失敗**: host は kind ごとの bus ID を `BusCreate` 送信前に予約し、proxy と
  lifecycle lease を先に作る。応答喪失など「peer へ届いたか不明」の失敗でも同じ lease が
  best-effort `BusRelease` を行う。解放成功を確認できない ID は再割当せず隔離するため、遅延した
  create と新 binding が同じ ID で衝突しない。
- **host 側 explicit close**: `hal.<KIND>.close(bus)` は [bus_accessor.md](bus_accessor.md) の
  exact-instance consuming close 規約に従う。alias/Accessor co-own が無いことを確定してから
  peer へ `BusRelease` を送り、peer 成功時だけ registry slot、bus_id lease、caller の
  `shared_ptr` をまとめて解放する。peer/transport/protocol error 時は local state と caller
  ownership を保持する。`BusRelease` は存在しない ID に対しても冪等なため、応答喪失後も
  同じ ID で再試行でき、成功確定前に ID を再利用しない。最終 holder の自然破棄は
  session gateへ再帰待ちしない non-blocking best-effort `BusRelease` とする。解放確認に成功した
  ときだけ bus ID と identity slotを再利用し、確認不能なら `Quarantined` tombstoneとID隔離を
  connection cleanupまで維持する。接続破棄は server 側の per-connection cleanup が最終安全網となる。

## 安全境界

信頼モデルは**対称**: host と device のどちらも、相手から届いた script に自分側のハードウェア・メモリ・時間を奪わせない。

- **BytecodeRunner の登録制 dispatch が許可リスト (allowlist) を兼ねる**。server に登録したバス / GPIOGroup 以外にはリモートから到達できない
- GPIO を公開する場合は、**公開専用の GPIOGroup を別に組み、公開してよいピンの IGPIO だけを register する**。platform の GPIO 実装を丸ごと登録すると、flash 接続ピン (ESP32 の GPIO6-11 等) まで露出する
- **ポート操作の deny_mask**: `GPIOGroup` が管理する deny_mask でポート操作時もピン単位のアクセス制御を維持する
- **device 側の実行予算** (`Server::Config`): script 1 本あたりの `DelayMs` 合計 (`max_delay_ms`)、`BusConfigure` が運ぶタイムアウト上限 (`max_bus_timeout_ms`)、`BusTransfer` が要求できる rx 確保量の上限 (`max_transfer_rx`)。違反は実行前の prescan が `ReportError(INVALID_ARGUMENT)` で拒否する
- **host 側の受信制限**: host の runner は受信専用 (`BytecodeRunner::setReceiveOnly`)。応答 / イベント script が実行できるのは `StoreData` / `Report*` / `Evt*` のみで、実行系 opcode は `PROTOCOL_ERROR` で拒否する
- **認証・認可は本仕様のスコープ外** (transport の責務)。物理 UART は物理アクセスを信頼境界とする

## エラー写像

詳細は [errors.md](errors.md) §エラー対処 hint 表 / §toString を参照。

- リモート起因のエラーとして `error_t` に `CLOSED` (transport 切断。 現状は `CLOSED` が担う; `DISCONNECTED` は将来の明示切断通知層向け予約 — [errors.md](errors.md) §エラー対処 hint 表) / `REMOTE_FAULT` (リモート内部エラー) / `UNSUPPORTED` (リモートが当該機能を持たない) を用いる
- `ReportError` が運ぶ status / `Control` フレームの error code は i8。**host が知らない値は `REMOTE_FAULT` に写像する** (`mapRemoteError`。新しい server が新コードを返しても旧 host は壊れない)

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| ストリーム専用 opcode (旧 `BusWriteStream` / `EvtStreamCredit` 等, 0x61-62) | `BusStreamTransfer` + `Data` フレーム stream_id に一本化。bytecode 層にストリーム状態管理を持たない (`BusBeginTransaction`/`BusEndTransaction` = 0xB4/0xB5 はwire互換名を維持する現役opcodeで、RunnerがAccessへadapterするため対象外) |
| channel-id ベース mux (旧 CRC16 + credit 管理チャネル) | frame v1 codec の単一フレーム列 (KIND + Data B3 stream_id) で多重化を表現。別系統のフレーム形式を持たない |
| 既定での自動再送 | リモート操作は非冪等であり得る。再試行の判断は呼び出し側に委ねる |
| bytecode への認証埋め込み | 層が違う。transport の責務とする |
| WiFiClient 依存の transport variant 分裂 | lwIP socket 直叩きの単一実装で足りる。framework ごとに transport を分岐しない |
| transport 層での DeviceSlot 多重デバイス管理・delimiter 再同期 | 多重化とフレーム境界は frame codec が担う。transport に別系統の多重化機構を持たない |
| push イベントのレート制限 | opt-in マスクで「監視対象」を意味論として定義する方が、帯域も判定コストも構造的に消える |

## Hal facade

`Hal::initUart(port)` / `Hal::initTcp(endpoint)` で接続を確立し、`hal.I2C.acquire()` 等のローカルと同一の API でリモートバスを操作する。内部で `RemoteBackend` (`IHalBackend` 実装) が `RemoteI2CProxyBus` 等の proxy バスを自動生成し、バス取得から転送までがローカルと同一のコードパスで動く。ユーザーは proxy の存在を意識しない (`Hal.I2C.acquire()` が透過的に返す)。

**接続束縛は排他 — 1 つの `Hal` は 1 デバイスの窓**: `initUart` / `initTcp` は
その `Hal` の全 kind view のバックエンドを接続先へ丸ごと差し替える (旧接続・PC ビルドの
host-local バスとの同居はしない)。host-local バスとリモートを同時に使う場合は `Hal` を
もう 1 つ構築する。複数リモートへの同時接続も同様に `Hal` を接続先の数だけ持つ。
接続に使うトランスポート (UART/TCP) 自体は facade の acquire を通らない variant 内部の
インフラであり、この排他の対象外。

connection / session の寿命は proxy bus の `shared_ptr` 寿命と分離する。connection は小型の
shared session handle を backend / GPIO / proxy へ配布し、各 RPC は handle の lease 中だけ
`RemoteSession` へ触る。再接続・local 復帰・`Hal` 破棄は handle を close し、進行中 RPC の
完了を待ってから session/transport を破棄する。旧 proxy 自体はメモリ上生存できるが、
以後の操作は `CLOSED` で失敗し、旧 connection を延命や操作し続けない。
handle は connection 側が別々に作るのではなく `RemoteSession` が持つ canonical な 1 個であり、
`RemoteSession&` を受ける低レベル互換 constructor もその handle を取得する。従って同一 session を
包む proxy 経路の違いで serializer が分裂しない。

kind別`Bus_remote`型は`RemoteBackend`がbus id / leaseと組み合わせて生成するconnection-bound proxyで、
低レベルprotocol test seamとしても使うが、local `Bus_<variant>`のdirect provider escape hatchではない。
そのconstructor / `init` / session handleは対応providerを利用者が選ぶための公開lifecycle APIとは扱わず、
通常callerはremote `Hal.<kind>.acquire(config)`と`Hal.<kind>.close(handle)`だけを使う。従ってlocal direct busの
`bus.close()`後`init()`再利用契約はproxyへ適用しない。

`remote::RemoteSession* Hal::session(void)` は低レベル用の**借用 escape hatch**である。戻り値は
次の `connect` / `initUart` / `initTcp` 呼び出し、またはその `Hal` の破棄までしか有効でない。
pointer から `RemoteSession` の raw API を直接呼ぶ操作は facade の operation gate を自動取得しないため、
proxy 操作、`pumpRemote()`、reconnect と caller 自身で直列化する。通常の bus / GPIO 操作には
`Hal` facade を使う。

`Hal::hasRemoteConnection()`は、その`Hal`がremote connection objectを現在所有しているかを返す
binding-kind snapshotである。未束縛またはlocal backendへの束縛ではfalse、remote connection所有中はtrueと
なる。transportの生存確認ではなく、通信失敗を検出してもconnection objectが破棄されるまではtrueのままである。
通信可否の正本は各operationの`result_t`とし、呼出し前のliveness probeには使わない。
同期的な`connect`には公開reconnecting状態がない。`isReady()` / `isBound()` / `connectionState()`のような
追加queryは、proxy operation gateやtransport変化との競合を持つ別の状態契約を増やし、operation結果と二重の
正本になり得るため、具体的なconsumer要求が生じるまで設けない。追加時は必要な意味と同期境界を定義する。

`Hal::connect(endpoint, cfg)` は即時確立 API で、endpoint は厳密一致・大文字小文字区別で解釈する。
`nullptr` / `""` / `"local"` は、その`Hal`が保持するlocal connection stateを`Hal`の`ResourceDomain`へ
束縛する。default `M5_Hal`だけは`M5HALCore`がfacadeをbootstrapする。
`"uart:<path>"` は `initUart(<path>, cfg)`、`"tcp:<host>:<port>"` は `initTcp(<host>:<port>, cfg)` に委譲する。
その他の文字列は `INVALID_ARGUMENT`。remote transport / connectionの作成が切替前に失敗した場合と、
切替中の登録失敗を完全にrollbackできた場合は旧束ねを保持し、切替を失敗させた元のerrorを返す。
service / GPIO登録のrollback自体が失敗した場合は、rollback errorで元のerrorを上書きせず、旧・新sessionを
closeし、両serviceをpoll対象から外し、facadeを未束縛へquarantineする。この状態では
`hasRemoteConnection()==false`、`backend()==nullptr`、`session()==nullptr`、`pumpRemote()`は
`NOT_CONNECTED`となり、後続の`connect()`で再確立できる。GPIO entryのbest-effort解除が失敗しても、
entryが参照し得るownerは`Hal`寿命まで保持してdangling pointerを作らない。
localへ戻す場合は、remote GPIOを一時解除して`Hal.Gpio`のslot 0へMCU GPIOを確保する。失敗時はremote
GPIOを復元し、旧backend / service / connectionを保持する。確保成功後にremote service / connectionを破棄して
local backendへ再束ねする。`Hal::init()` は冪等な ensure-bound で、既に local/remote
どちらかへ束ね済みなら no-op 成功、未束ねなら `connect(nullptr)` と同じ local 束ねを行う。
そのため、ユーザーが remote 接続した後にライブラリ側が `init()` を呼んでも接続先は壊れない。
未束ね `Hal` の bus acquire / commit と remote connection の無い `pumpRemote()` は `NOT_CONNECTED` を返す。

GPIO も接続時に `Hal.Gpio` に自動登録される。peer が GPIO を公開しない場合は GPIO 登録を行わず、
接続自体は成功する。公開する場合は、旧 remote slot を再利用可能な空きとして扱い、slot key 0〜127 の
うち最小の空きを事前選択する。`GPIOGroup` の同時登録上限 16 entries、または remote `IGPIO` の
pin count 1〜256 契約を満たせない場合は `INVALID_ARGUMENT` を返し、新しく作成した connection を破棄して
旧 backend / connection / GPIO / service を保持する。事前検証の成功後だけ旧 connection を切り替える。
`read()` / `readPort()` はホスト側キャッシュを返し、
接続時の `seedCache` と `GpioSubscribe` 後の `EvtGpioState` push で更新される。物理 wire の値を
キャッシュから切り分けたい場合は、host 側から `GpioPortRead` を含む bytecode request を発行する。
reconnect 前に取得した `Pin` / `PortAccess` は新 peer へ付け替えない。raw port pointer の値型契約を
安全に保つため旧 `Port_remote` のみを `Hal` の寿命まで保持し、旧 connection / transport は延命しない。
旧 handle close 後は read が最終 cache、void の write / mode 変更が no-op、`syncRead()` が `CLOSED`
となる。`Hal` 自体の破棄後まで `Pin` / `PortAccess` を使えるという意味ではない。
host tool は待ち窓や idle loop で `Hal::pumpRemote()` を呼ぶことで remote I/O、keepalive、
watch callback dispatch を 1 つの入口から進められる。使用例は
`examples/v2/HowToUse/Remote/`、`examples/v2/HowToUse/RemoteI2S/`、`examples/v2/RemoteServer/`、
`examples/v2/RemoteServerTCP/`、`examples/v2/RemoteTest/`。

push watch callback は canonical session lease 内で同期 dispatch されるため、同じ session の
remote transaction、`Hal::pumpRemote()`、その `Hal` の reconnect / 破棄へ再入してはならない。
通常の proxy `shared_ptr` を callback 内で手放すことは許可され、自然解放は session gate を
再帰待ちせず、確認不能な bus ID を connection cleanup まで隔離する。

## 互換性と版管理

- 本仕様 (M5HAL remote v1) の段は **`experimental`** ([../stability.md](../stability.md))。`stable` を宣言するまでは非互換変更があり得る
- メッセージ層の版は `hello` の `proto_ver` で判別する。`stable` 宣言後の非互換変更は `proto_ver` を増やし、frame KIND 値の意味は再利用しない
- 前方互換の原則: 未知の frame KIND は破棄 ([frame.md](frame.md))、`HelloResp` の末尾拡張は無視、bytecode の未知 opcode は critical bit に従う ([bytecode.md](bytecode.md))

## 関連

- 実例: [`RemoteServer`](../../examples/v2/RemoteServer/) / [`RemoteServerTCP`](../../examples/v2/RemoteServerTCP/) (device) + [`RemoteTest`](../../examples/v2/RemoteTest/) (host) — device 側は frame mux transport で内蔵バス + GPIO を公開し、host 側は自動テストでリモート操作する。[`HowToUse/Remote`](../../examples/v2/HowToUse/Remote/) と [`HowToUse/RemoteI2S`](../../examples/v2/HowToUse/RemoteI2S/) は host facade の使用例
- 下層仕様: [frame.md](frame.md) (wire フレーム形式) / [data_io.md](data_io.md) / [bytecode.md](bytecode.md)
- バス / GPIO 抽象: [bus_accessor.md](bus_accessor.md), [i2c.md](i2c.md), [gpio.md](gpio.md)
- 検証: [../verification.md](../verification.md) (native gtest `test_mux_remote` / `test_mux` / posix UART・TCP end-to-end)
