# I2S バス設計 (v2)

> **読者**: 実装者・レビュー向け（設計仕様）。

I2S (オーディオ用シリアルバス) の v2 抽象。当面のスコープは **TX (再生) / RX (録音)・
Philips standard・16bit・mono/stereo** で、raw PCM の搬送までを役割とする。TX と RX は
同一コントローラ上の独立 DMA 経路で、**全二重** (同時 write/read) も可能。クロックの向きは
`role` (master = BCLK/WS を生成 / slave = ピアのクロックに追従) で選ぶ。
WAV 等のコンテナ解釈・デコード・ミキシングは上位 (アプリケーション / example) の責務
([goals.md](../goals.md) の「音声のドメインロジックは含めない」を維持する)。

## write の意味論

I2S の最も重要な契約 — Audio 層実装者が最初に必要とする前提。

- 戻り値は **受理できたバイト数** (DMA バッファへコピーできた量)。`write_timeout_ms` 内に
  全量を受理できなければ短い戻りになるが、これは**正常** (uart の short read と同じ思想)。
  `write_timeout_ms = 0` は non-blocking (いま入る分だけ受理)。
- **underrun はエラーにしない**: DMA が枯れたら無音を出力し、次の write から再開する。
  エラー扱いにしない理由は、連続再生では「途切れたら静かに継続」が常に正しい縮退であり、
  呼び出し側に回復処理を強いる価値がないため。
- `writableBytes()` は「いま write してもブロックしない量」。送信側のフロー制御の源泉。
  リモートバス搬送のワイヤ上 flow control (credit 通知) は backend の write/read とは別に、
  [remote.md](remote.md) §Transport 層 — frame mux 多重化 が受信側バッファ空きを追跡して行う。
- **戻り値の切れ目契約**:
  - 受理量は **サンプル境界 (`bits_per_sample / 8` byte の倍数) で切れる**。espidf
    backend はこれを能動的に保証する — DMA 満杯間際の non-blocking 受理は奇数バイトで
    止まり得るが、1 byte の位相ズレは以後の全再生を fs/2 サイドバンドノイズに変える
    ため、半サンプルは短いブロッキング書きで必ず完結させてから戻る (mono→stereo
    複製モードでは物理フレーム単位で同じ完結を行う)
  - **フレーム境界 (channels × sample) までは保証しない**: stereo で L サンプルだけ
    受理されて戻ることはある。ただし DMA はバイト列として連続するので、**次の write
    を続きのバイトから再開すれば壊れない** — 呼び出し側の端数持ち回りは「受理されな
    かった残りを次回先頭に回す」だけでよく、チャネル整合の再計算は不要
  - 入力の与え方: フレーム整列したバイト列を渡すのが基本 (16bit stereo なら 4 byte
    単位)。整列していない端数を渡しても上記契約により音声は壊れない

## read の意味論 (RX)

write の鏡像。RX チャネル (`pin_din` 配線時) の DMA が取り込んだ PCM を読み出す。

- 戻り値は **読み出せたバイト数** (DMA から sink へ渡せた量)。`read_timeout_ms` 内に満たせ
  なければ短い戻り (連続ストリームゆえ short read は正常)。`read_timeout_ms = 0` は non-blocking。
- `readableBytes()` は「いまブロックせず read できる量」(DMA 取込済み未排出のバイト数)。
- **overrun はエラーにしない**: 読み手が遅れて DMA が最古ディスクリプタを上書きしても、
  欠落として静かに継続する (underrun を無音で継続するのと対称の縮退)。
- **切れ目契約は write と同じ**: 半サンプルで戻さない (1 byte 位相ズレ防止に短いブロッキング
  読みで完結)。mono→stereo モードでは物理フレーム単位で完結し、左スロットを論理 mono として返す。

## API 形 — uart 同型の非トランザクショナル bus

I2S は DMA 駆動の連続ストリームであり、I2C / SPI のような「開始 → 転送 → 終了」の
トランザクション境界を持たない。このため transfer 形ではなく
[uart](uart.md) と同型の write 系 API とする:

```cpp
struct IBus : bus::IBus {
    virtual expected<size_t, error_t> write(bus::IAccessor* owner, const AccessConfig& cfg,
                                            data::Source* tx, size_t len);
    virtual expected<size_t, error_t> writableBytes(bus::IAccessor* owner, const AccessConfig& cfg);
    virtual expected<size_t, error_t> read(bus::IAccessor* owner, const AccessConfig& cfg,
                                           data::Sink* rx, size_t len);
    virtual expected<size_t, error_t> readableBytes(bus::IAccessor* owner, const AccessConfig& cfg);
    // 独立 TX/RX チャネルロック (uart 同型)。lock/unlock は TxRx 合成。
    result_t<void> lockChannel(bus::IAccessor* owner, Channel ch, uint32_t timeout_ms);
};
struct TxAccessor : bus::IAccessor, data::StreamWriter;  // 再生 = StreamWriter
struct RxAccessor : bus::IAccessor, data::StreamReader;  // 録音 = StreamReader
struct Accessor;  // TX + RX 束ね (全二重を 1 つで)
```

- `TxAccessor` / `RxAccessor` はそれぞれ [data_io.md](data_io.md) の `StreamWriter` /
  `StreamReader` を実装する。汎用ストリームコードがそのまま音声の出力先 / 入力源に使える。
- **独立 2-mutex** (uart 同型): TX と RX は別チャネルロックを握るため、全二重バスでは
  片方が write 中でももう片方が read できる (単一ロックなら直列化される)。`Accessor` (束ね)
  は両チャネルを TX→RX 順で取る。`readUntil` は連続ストリームの I2S には無い (uart 専用)。
  remote proxy の channel 所有権も同じだが、標準 `RemoteSession` の個々の wire RPC は共通
  session gate で直列化される。1 RPC 内の全二重は複合 transfer が担う
  ([remote.md](remote.md) §SEQ)。

## Bus の入手

共通機構は [bus_accessor.md](bus_accessor.md) §Bus の保持 を参照。本 kind 固有の差分のみ以下に示す。

- **identity = BCLK / WS / DOUT / DIN**。MCLK / buffer size / role は identity 外だが、
  同一 identity の typed acquire で食い違う場合は `INVALID_STATE` を返す (既存 bus は再構成しない)。
- I2S backend は **espidf のみ提供** (software/host では未提供)。
- I2S は **static-backend policy**。`commitBuses()` は no-op、`hardwareInUse()` は 0。
  `acquire(LogicalBusConfig)` は I2C/SPI と同じ surface と validation を持つが、現時点では
  有効な logical request に `NOT_IMPLEMENTED` を返す。bus 生成は `acquire<CfgT>(cfg)` が担い、
  backend は初回 acquire の config 型で固定される。
- 直接構築 (`i2s::Bus bus; bus.init(cfg);`) も可。

## 設定の分担

| 構造体 | 内容 | 所有 |
|---|---|---|
| `IBusConfig` | pins (bclk / ws / dout / din / mclk)、`tx_buffer_size` / `rx_buffer_size` (各方向 DMA 総量の目安)、`role` (Master / Slave、既定 Master・identity 外) | bus を生成・登録する側 (device) |
| `AccessConfig` | `sample_rate_hz` / `bits_per_sample` (当面 16) / `channels` (1=mono, 2=stereo) / `write_timeout_ms` / `read_timeout_ms` | アクセスする側 |

他バスと同じく AccessConfig は呼び出しごとに渡され、backend は前回設定と異なる場合のみ
再構成する (明示的な start / stop API は置かない。DMA channel は最初の write / read で
遅延開始し、停止は release / 再構成で行う)。

**既知の制約 — 全二重 reconfigure 境界の bus-level quiesce**: IDF に drain API が無いため、
reconfigure (sample rate / channels 変更) の境界で in-flight バッファはそのまま切れる。
定常状態のデータは常にクリーンだが、reconfigure 直後は slave 側の再同期に ESP32-S3 で
1-2 フレーム、classic ESP32 で最大 ~40 フレームのズレが生じ得る。実行中に頻繁に
reconfigure するユースケースでのみ顕在化するため、既知トレードオフとして受容する
(fix にはプロトコルレベルの事前通知機構が要り、変更の割に効果が薄いと判断)。

コア配線ピンは **タグ型 ctor** で与えられる: `BusConfig{i2s::Bclk{34}, i2s::Ws{33}, i2s::Dout{13}}`
(TX-only の標準形)。i2c / spi / uart と同じく順序取り違えがコンパイルエラーになる。**チャネルは
pin 駆動**: `pin_dout` 設定で TX、`pin_din` 設定で RX、両方で全二重。`pin_din` (RX) / `pin_mclk` /
`rx_buffer_size` / `role` は構築後にフィールド代入する (`role` は wiring でないので identity 外)。
variant config は `using IBusConfig::IBusConfig;` でこのタグ ctor を継承する。

## variant 規定

- **espidf**: ESP-IDF gen5 ドライバ (`driver/i2s_std.h`) を backend とする。legacy (gen4)
  ドライバは対応しない。ヘッダの有無で variant ごと有効化する (I2S 非搭載 SoC では IDF が
  ヘッダを公開しないことに依拠)。I2S ポートは自動割当 (`I2S_NUM_AUTO`。BusConfig に
  ポート番号フィールドは置かない — 必要になった時点で追加する)。`writableBytes` の源泉は
  **in-flight (送信済み未消費) バイト数の直接追跡**: write で加算し、送信完了コールバックで
  **0 でクランプしながら減算**する。クランプが本質 — underrun 中の無音バッファも送信完了
  コールバックを発火させるため、累積 submitted/consumed のカウンタ対では無音分がドリフトし
  free が容量に張り付く (実機で確認)。underrun 時は無音クリアを有効にする。
  **チャネルは pin 駆動**: `i2s_new_channel(&cfg, dout>=0?&tx:null, din>=0?&rx:null)` で TX/RX/全二重を
  生成 (同一 chan_cfg・BCLK/WS/slot 形式を共有)。`role` は `I2S_CHANNEL_DEFAULT_CONFIG` の
  `I2S_ROLE_MASTER`/`SLAVE` に渡すだけで、別クラスは不要。`readableBytes` の源泉は **取込済み未排出
  バイト数の直接追跡**: `on_recv` で加算 (容量クランプ)、read で減算 (`writableBytes` の鏡像)。
  **release のピン後始末は backend が担う**: `i2s_del_channel` は GPIO matrix 経路と
  peripheral 制御の output enable を残置するため、削除後もコアピンが凍結レベルを能動駆動し
  続ける (ESP32 / ESP32-S3 実測 — 外部 pull を無視する)。SPI (`gpio_reset_pin`) /
  I2C (`gpio_output_disable`) の vendor 後始末と揃え、チャネル破棄時に設定済みピンを
  `gpio_reset_pin` で chip default (high-Z + pull-up) へ戻す。
- **arduino**: 専用 variant は置かない。arduino-esp32 3.x は IDF 5.x を内包するため
  espidf variant を直接使う (2.x = IDF 4.4 は legacy ドライバのため対象外。ヘッダ検出に
  より自動的に無効となる)。

## リモートバス搬送

詳細は [remote.md](remote.md) §データチャネル / §Transport 層 — frame mux 多重化 (credit 通知) を参照。remote I2S の TX/RX は
`BusStreamTransfer` + `Data` frame stream で搬送する。remote proxy は stream request の前に
同じ bytecode script 内で `BusConfigure` を送るため、`AccessConfig` の sample rate / bit depth /
channel count / timeout は転送ごとに server 側 accessor へ反映される。

## 将来拡張

- 8 / 24 / 32 bit、PCM short / MSB 等のスロット形式
