# I2S バス設計 (v2)

> **読者**: 実装者・レビュー向け（設計仕様）。

I2S (オーディオ用シリアルバス) の v2 抽象。当面のスコープは **TX (再生) / RX (録音)・
Philips standard・16bit・mono/stereo** で、raw PCM の搬送までを役割とする。TX と RX は
同一コントローラ上の独立 DMA 経路で、**全二重** (同時 write/read) も可能。クロックの向きは
`role` (master = BCLK/WS を生成 / slave = ピアのクロックに追従) で選ぶ。
WAV 等のコンテナ解釈・デコード・ミキシングは上位 (アプリケーション / example) の責務
([goals.md](../goals.md) の「音声のドメインロジックは含めない」を維持する)。
内蔵 DAC は I2S と異なるペリフェラルであり、同じ設定・転送契約へ統合すると bus abstraction の
責務が歪むため、I2S kind には吸収しない。

### Standard I2S と必須 pin

- `i2s::BusConfig` は master / slave とも **BCLK と WS を必須**とし、TX の DOUT または
  RX の DIN を最低1本必要とする。BCLK / WS のどちらかが負値、または DOUT / DIN が
  ともに負値なら、`init()` / typed `acquire()` / remote `BusCreate` は
  `INVALID_ARGUMENT` で拒否する。エラーを初回 I/O まで遅延させない。
- MCLK は任意で、`-1` は無効を表す。
- この `i2s` API は BCLK / WS を持つ standard I2S 専用であり、WS を持たない PDM を
  mode 分岐として受け入れない。PDM は公開 API / bus kind を分離し、物理的な I2S
  controller の占有だけを standard I2S と共有する。PDM側の契約は [pdm.md](pdm.md) を参照。

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
    ため、半サンプルは短いブロッキング書きで必ず完結させてから戻る。ESP-IDF backend が
    論理 `channels=1` の sample を内部で 2 physical slot へ展開する場合も、物理フレーム単位で
    同じ完結を行う。この展開は backend 内部処理であり、公開 PCM 形状は mono のまま。
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
  読みで完結)。ESP-IDF backend が 2 physical slot の capture を内部で collapse する場合は
  物理フレーム単位で完結し、左スロットを 1 sample の論理 mono として返す。この正規化は
  backend 内部処理であり、公開契約は一貫して `channels=1`。
- `channels` は DMA バッファの**論理 PCM 形状**を表す。`channels=1` の read は物理配線や
  backend にかかわらず 1 sample/frame の mono 列を返す。`channels=2` は L/R interleaved 列を返すため、
  送信元が同じ mono sample を両スロットへ出していれば `x,x,y,y...` と見える。mono 入力を
  stereo 配列へソフト複製する設定は持たず、それは上位のチャネル変換の責務とする。

## API 形 — uart 同型の非トランザクショナル bus

I2S は DMA 駆動の連続ストリームであり、I2C / SPI のような「開始 → 転送 → 終了」の
物理トランザクション境界を持たない。単方向は[uart](uart.md)と同型のwrite/read系を主とし、
全二重用にはTX/RXを一つの呼出しで進める複合`transfer`も持つ:

```cpp
struct IBus : bus::IBus {
    result_t<void> beginOperation(bus::OperationContext<AccessConfig>& context);
    result_t<void> endOperation(bus::OperationContext<AccessConfig>& context);
    result_t<size_t> write(bus::OperationContext<AccessConfig>& context,
                           data::Source* tx, size_t len);
    result_t<size_t> writableBytes(bus::OperationContext<AccessConfig>& context);
    result_t<size_t> read(bus::OperationContext<AccessConfig>& context,
                          data::Sink* rx, size_t len);
    result_t<bus::TransferTotals> transfer(
        bus::OperationContext<AccessConfig>& tx_context,
        bus::OperationContext<AccessConfig>& rx_context,
        data::Source* tx, size_t tx_len, data::Sink* rx, size_t rx_len);
    result_t<size_t> readableBytes(bus::OperationContext<AccessConfig>& context);

protected:
    // providerの派生点。上記non-virtual入口でContext検査後にだけ呼ばれる。
    virtual result_t<void> beginOperationBackend(bus::OperationContext<AccessConfig>&);
    virtual result_t<void> endOperationBackend(bus::OperationContext<AccessConfig>&);
    virtual result_t<size_t> writeBackend(bus::OperationContext<AccessConfig>&, data::Source*, size_t);
    virtual result_t<size_t> writableBytesBackend(bus::OperationContext<AccessConfig>&);
    virtual result_t<size_t> readBackend(bus::OperationContext<AccessConfig>&, data::Sink*, size_t);
    virtual result_t<bus::TransferTotals> transferBackend(
        bus::OperationContext<AccessConfig>&, bus::OperationContext<AccessConfig>&,
        data::Source*, size_t, data::Sink*, size_t);
    virtual result_t<size_t> readableBytesBackend(bus::OperationContext<AccessConfig>&);
    // 独立 TX/RX チャネルロック (uart 同型)。lock/unlock は TxRx 合成。
    virtual result_t<void> lockChannel(
        bus::IAccessor& owner,
        Channel ch,
        uint32_t timeout_ms = types::TIMEOUT_FOREVER);
};
struct TxAccessor : bus::IAccessor, data::StreamWriter;  // 再生 = StreamWriter
struct RxAccessor : bus::IAccessor, data::StreamReader;  // 録音 = StreamReader
struct Accessor;  // TX + RX 束ね (全二重を 1 つで)
```

- `TxAccessor` / `RxAccessor` はそれぞれ [data_io.md](data_io.md) の `StreamWriter` /
  `StreamReader` を実装する。汎用ストリームコードがそのまま音声の出力先 / 入力源に使える。
- **TX/RX二重ロックの契約は [uart.md](uart.md) §channel semantics と同型** (TX→RX順取得・非ネストAccess・
  sugarのborrow規則、remote proxyのchannel所有権とsession gateによる直列化を含む)。I2S固有差分:
  `readUntil` は連続ストリームの I2S には無い (uart 専用)。
- 公開ライフサイクルは `beginAccess()` / `endAccess()` であり、**非ネスト**。開始時に方向別
  lock を取得して `Bus::beginOperation(context)` を1回呼び、終了時に
  `Bus::endOperation(context)` を1回呼んでから lock を解放する。ここでの
  Operation は物理フレーム境界ではなく、要求設定を適用し同一方向を排他する連続 stream の
  利用区間である。`write` / `read` sugar は既存 Access を借用し、無ければ一時 Access を開閉する。
- Context検査契約は [bus_accessor.md](bus_accessor.md) §OperationContext capabilityとchecked facade と同一
  (TX/RX別slot前提もそこで一般化済み)。providerはprotected `*Backend` hookだけをoverrideし、raw owner/configを
  公開virtual引数として受けない。
- `Accessor` の複合 Access は TX→RX の順に開始し、RX 開始失敗時は TX を rollback する。終了は
  RX→TX。子 accessor が直接 Access 中なら複合開始を `INVALID_STATE` で拒否する。
- 各 `write` / `read` / 複合 `transfer` は個別の `result_t` を返し、成功・short transfer・失敗を
  `getLastTransferStatus()` でも直近1件として取得できる。Access 全体の totals は集計しない。

## Bus の入手

共通機構は [bus_accessor.md](bus_accessor.md) §Bus の保持 を参照。本 kind 固有の差分のみ以下に示す。

- **portable acquireのidentity projection = `Pins` tagのBCLK / WS / DOUT / DIN**。MCLK / buffer size / role は identity 外だが、
  同一 identity の acquire で食い違う場合は `INVALID_STATE` を返す (既存 bus は再構成しない)。
- I2S backend は **espidf のみ提供** (software/host では未提供)。
- I2S は **static-backend policy** ([bus_accessor.md](bus_accessor.md) §UART / I2S / PDM: static-backend policy 参照)。
- 直接構築 (`i2s::Bus bus; bus.init(cfg);`) も可。

PDMとの物理controller共有規則は [pdm.md](pdm.md) §standard I2Sとのcontroller排他 を参照 (この選択はbackend内部
実装であり、static-backend policyの`hardwareInUse() == 0`という公開契約は変えない)。

## 設定の分担

| 構造体 | 内容 | 所有 |
|---|---|---|
| `IBusConfig` | pins (bclk / ws / dout / din / mclk)、`tx_buffer_size` / `rx_buffer_size` (各方向 DMA 総量の目安)、`role` (Master / Slave、既定 Master・identity 外) | bus を生成・登録する側 (device) |
| `AccessConfig` | `sample_rate_hz` / `bits_per_sample` (当面 16) / `channels` (論理PCM形状: 1=mono, 2=L/R interleaved stereo) / `write_timeout_ms` / `read_timeout_ms` | アクセスする側 |

`AccessConfig` は accessor 内の `OperationContext` が値として保持する。backend は
`beginOperation` で要求設定を適用し、同じ Access 内の個々の I/O では再構成しない。DMA channel は
最初の `beginAccess` で遅延開始するが、`endAccess` は連続 DMA を物理停止・drain する境界ではない。
停止はclose / 再構成で行う。全二重で TX/RX Access が同時に存在する場合、sample rate / width /
channels が一致しない後発 Access は `INVALID_STATE` とし、先行方向の設定を破壊しない。

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
`beginOperation` は `BusConfigure` を送って、`AccessConfig` の sample rate / bit depth /
channel count / timeout を Access 開始時に server 側 accessor へ反映する。個々の wire RPC は
session gate の都合で短く完結し、現行 protocol は server 側の物理 Access を複数 RPC にまたがって
保持しない。
