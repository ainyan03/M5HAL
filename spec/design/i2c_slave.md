# i2c slave — スレーブ機構

> **読者**: 実装者・レビュー向け（設計仕様）。

I2C slave の Bus / Accessor / 設定型 / 給仕モデルの仕様。 master 体系は [i2c.md](i2c.md)、 共通基底は [bus_accessor.md](bus_accessor.md) を参照。

I2C slave は基本機能として master 体系と相似の型で提供する:
`ISlaveBus` (抽象基底) / `SlaveBus_<variant>` (具象 backend) / `SlaveBusConfig` /
`SlaveStreamAccessor` (基底のストリームアクセサ)。 応答ポリシーは後述のとおり
ストリーム基底 + レジスタマップ・アダプタ (`SlaveRegMapAccessor`) に分岐する。

**高レベル API = `serve(Source* src, Sink* dst, timeout)`**: master の
`transfer(desc, Source*, Sink*)` ([i2c.md](i2c.md) §transfer の wire semantics)
と対称な per-transaction blocking 給仕。 1 取引 = master の 1 `transfer()` を、 write 取引なら
受信を `dst` Sink へ流し込み、 read 取引なら `src` Source から応答を送出して、 STOP まで処理する。
`src`/`dst` の polarity は master と一致 — **`src` Source = その側が送出するデータ / `dst` Sink =
受信するデータ** (向きの正準規約と役割反転不変性は [data_io.md](data_io.md) §向き)。 取引あたりの
容量上限は無く (back-pressure で律速)、 ストリーム消費者は
backend の ISR / service tick が起こす (polling 固定遅延ではない)。 SPI の `transfer` と同じ
使い勝手で、 タスク↔Source/Sink のコールバックは内部実装としてユーザーに開示しない。
低レベルの窓 seam (`beginTransaction`/`read`/`write`/`endTransaction`) は serve() が内部で合成
する素として残し、 自作 tick ループや特殊プロトコル向けに直接叩ける (`beginTransaction` =
ロック内側の取引単位を開く seam。 master 体系の `beginAccess` ロック取得とは別概念軸 —
[bus_accessor.md](bus_accessor.md) §排他制御の意味論 (常時 mutex))。
**Bus 型は master と分離** — 実装実態 (ESP-IDF は master/slave が
別ドライバ) と排他意味論が別物のため。 同一ピンの役割切替は
**「一方を `release()` → 他方を `init()`」を正規パターン**とする (バスを手放さない
役割反転が必要になったら統合ファサードを後から非破壊で追加できる)。

```cpp
namespace m5::hal::v2::i2c {

enum class TxUnderrun : uint8_t { fill, stretch };

struct SlaveBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_scl = -1;
    types::gpio_number_t pin_sda = -1;
    uint16_t address             = 0;      // 7-bit address 値
    bool address_is_10bit        = false;  // 形 parity 用。現 backend は true / >0x7F を INVALID_ARGUMENT で reject
    uint32_t timeout_ms          = 1000;   // 低レベル read() のデータ待ち予算 (期限切れ = 0byte の正常短読み)
    TxUnderrun tx_underrun       = TxUnderrun::fill;
    uint8_t tx_fill_byte         = 0xFF;
    uint32_t stretch_timeout_ms  = 100;    // tx_underrun=stretch の応答待ち上限 (超過で fill へ)
};

class SlaveStreamAccessor /* : StreamReader, StreamWriter */ {
    // 高レベル: 1 取引を Source/Sink で給仕 (master transfer(Source/Sink) の対称物)。
    // master が write した取引 → 受信を dst Sink へ / master が read した取引 → src Source
    // から応答を送出。STOP まで blocking。src/dst は片方 nullptr 可 (その向きの access を
    // 期待しない指定。下の挙動表参照)。
    // 戻り値 = 受信したデータ量 (write 取引 = master の書込を dst Sink に受けた数 = dst.written()、
    //   read 取引 = 0)。応答 (src からの送出) は戻り値に含めない。write 取引の検知は戻り値 > 0
    //   または dst.written() > 0。
    // timeout_ms = wall-clock 総時間でなく「無進展 (stall) の最大許容時間」で、取引開始待ち
    //   にも効く (idle のバスで取引が来ない間にも適用)。
    //   既定 TIMEOUT_FOREVER = 取引が来るまで永久待ち + 取引中も永久 hold (ゼロロス)。
    //   有限値 = 開始待ち or 取引中 stall がその期限を超えたら TIMEOUT_ERROR (取引中なら
    //   残り write を捨てて master を完走させ wedge を回避。容量制限 Sink / null dst 用)。
    result_t<size_t> serve(data::Source* src, data::Sink* dst,
                           uint32_t timeout_ms = TIMEOUT_FOREVER);

    // 自作 serve ループ用: backend のアクティビティ (RX 着信 / STOP / TX 空き) で起きる
    // イベント駆動の待ち。ISR backend は通知で起こす (既定 = 1ms poll)。バスは止めない。
    // 戻り値 = true:活動 wake を確認 / false:timeout 経過 (poll fallback は常に false)。
    // unbound は INVALID_ARGUMENT。bool は advisory で、いずれにせよ readableBytes /
    // transactionComplete で状態を再確認する。
    result_t<bool> waitForActivity(uint32_t timeout_ms);

    // 低レベル窓 seam — serve() / 取引ステップ API が内部で合成する素。自作 tick ループや
    // 特殊プロトコル向けに直接叩ける (通常は serve() を使う)。
    result_t<void> beginTransaction(uint32_t timeout_ms = TIMEOUT_FOREVER);
    result_t<void> endTransaction();
    result_t<size_t> read(data::DataSpan dst);       // 現窓のデータのみ
    result_t<void> write(data::ConstDataSpan src); // 現窓への応答キュー
    result_t<size_t> readableBytes();
    result_t<bool> transactionComplete();            // master が STOP したか
};
// RAII: ScopedSlaveTransaction (spi::ScopedTransaction と同 polarity)

class SlaveRegMapAccessor {  // SlaveStreamAccessor を内部に合成するアダプタ
    SlaveRegMapAccessor(ISlaveBus& bus, data::DataSpan reg_file);

    // アプリ側レジスタアクセス (ワイヤ非依存、hook を発火しない)
    uint8_t getRegister(uint8_t reg) const;
    void    setRegister(uint8_t reg, uint8_t value);
    uint8_t pointer() const;

    // hook (関数ポインタ + void* ctx、std::function 不使用)
    void setOnRead(uint8_t (*cb)(uint8_t reg, void* ctx), void* ctx);
    void setOnWrite(void (*cb)(uint8_t reg, uint8_t val, void* ctx), void* ctx);

    // ブロッキング便利関数: 1 取引を給仕 (espidf の while ループ用)
    result_t<void> serve(uint32_t timeout_ms = TIMEOUT_FOREVER);

    // 取引ステップ API (純レジスタマップ・ロジック、bus I/O 非依存)。
    // serve() が検証済みの順序で合成する素。ServiceRunner 用の自作 tick
    // ループの素材にもなり、native 決定論テストが叩く seam でもある。
    void   beginExchange();                  // 次の ingest バイト = ポインタ
    void   ingest(data::ConstDataSpan src);  // 書込相: 先頭=ポインタ、以降=書込
    size_t composeReply(data::DataSpan dst); // ポインタ窓から応答 compose
    SlaveStreamAccessor& stream();           // 合成された stream accessor
};

}
```

**backend の構築**: 実機 (ESP32-S3 系) は `SlaveBus_espidf bus; bus.init(cfg);` で済む
(`cfg` は `SlaveBusConfig`)。 software / native backend (`SlaveBus_software`) は SCL/SDA を
駆動する `SlaveLineDriver` を渡す `init(SlaveLineDriver&, cfg)` を使う (`init(cfg)` 単独呼びは
`INVALID_ARGUMENT`)。 software backend を native の ServiceRunner tick から進めるモデルでは
`ScopedSlaveServiceRegistration{runner, bus}` で service を runner に登録する (`serve()` の
ブロッキング poll が使えない単一スレッド向け、後述の取引ステップ API と組む)。

**`serve(src, dst, timeout)` の polarity と nullptr** (`src`/`dst` は master API と同じ「この
accessor が送る / 受ける」向き。 slave 視点ではなく master 動作を主語に読むと迷わない):

| 引数 | 型 | これが効く master 取引 | slave 側の意味 | `nullptr` のとき |
|---|---|---|---|---|
| `src` | `Source*` | master read | slave が**送出**する応答データ | read 応答を作らない → backend の `tx_underrun` policy (fill / stretch) が出る |
| `dst` | `Sink*` | master write | slave が**受信**する要求データ | write 受信を捨てる → drain しないので有限 `timeout_ms` 必須 (さもないと RX_FULL stretch で wedge) |

**timeout は 3 つあり対象が違う** (いずれも `*timeout*` 名なので取り違え注意):

| 設定 | 対象 | 期限時の挙動 |
|---|---|---|
| `serve(…, timeout_ms)` | 取引開始待ち + 取引中の無進展 (stall)。**wall-clock 総時間ではない** | `TIMEOUT_ERROR`。取引中 stall なら残り write を discard して master を完走させる |
| `SlaveBusConfig::timeout_ms` | 低レベル `read()` が現窓の RX byte を待つ予算 | 0 byte の短読み (通常 error ではない) |
| `SlaveBusConfig::stretch_timeout_ms` | `tx_underrun=stretch` で窓未オープン / 応答未投入の read を stretch 保持する上限 | `tx_fill_byte` 送出へ fallback |

**トランザクション窓モデル**: 1 窓 = master の 1 `transfer()` (write-then-read を
repeated start で繋いだ全体、STOP まで)。 「トランザクション = データの寿命スコープ」:

- **repeated start は窓を跨がない** — レジスタ読み出しエミュレーションは
  「アドレスを読む → 応答を `write` → restart の read で応答が出る」と 1 窓で完結
- **窓の中はストリーム契約** (UART と同型: 溜まり分から取れるだけ、 timeout 待ち、
  期限切れは正常な短読み)。 受信は背後で進み続け、 窓は読み出しカーソルの区切り —
  窓 N に窓 N+1 のデータは混入しない
- **Tx 自動消滅**: master が STOP したら未送出の応答キューは破棄される。 積み残しが
  次のトランザクションへ混入しないことを構造的に保証する (明示的な取下げ API は無い —
  必要になれば非破壊で追加できる)
- 窓未オープンで master に読まれたら **`tx_underrun` policy**: `fill` = `tx_fill_byte`
  を送出 / `stretch` = SCL stretch で slave の `write` を待ち、 `stretch_timeout_ms`
  超過で fill にフォールバック。 stretch は backend / SoC の capability に依存する
  (software backend は両対応。 ESP32 無印の HW slave は stretch 非対応)

**どちらのアクセサを使うか**:
- レジスタアドレス + auto-increment + ポインタ保持の I2C 定番デバイス (センサ / PMIC / 設定
  レジスタ) を作る → **`SlaveRegMapAccessor`**。 `serve()` を回すだけで、 `onRead` で live 値・
  `onWrite` でコマンド副作用を挿す。 ただし 1 read は最大 64B (`kReplyWindowBytes`、 超える読みは
  ポインタ再シードの複数 read)。
- 可変長フレーム / echo / ブリッジ / 64B 超の一括 read / 独自プロトコル → 基底
  **`SlaveStreamAccessor::serve(Source*, Sink*)`** で能動給仕。

最小 echo (基底 Stream、 完全版は `experiments/v2/test/i2c_slave/device/i2c_echo.cpp`):

```cpp
uint8_t echo_buf[1024];
size_t  echo_len = 0;
SlaveStreamAccessor slave{bus};
for (;;) {
    // Source/Sink は cursor を持つので取引ごとに作り直す
    data::MemorySource reply{data::ConstDataSpan{echo_buf, echo_len}};  // master read の応答
    data::MemorySink   request{data::DataSpan{echo_buf, sizeof(echo_buf)}};  // master write の受信先
    auto r = slave.serve(&reply, &request);  // serve は 1 取引ごとに返る → ループ必須
    if (!r.has_value()) { continue; }
    if (request.written() > 0) { echo_len = request.written(); }  // 書込長は written() で取る
}
```

I2C echo は **write 取引で受けた内容を、続く別取引の read で返す split プロトコル** (同一取引内で
即返すのではない)。 そのため write 長を `echo_len` として取引を跨いで保持する。

**backend = stretch プリミティブ / accessor = 応答ポリシー**: backend が提供する土台は
「master の read 要求に対しデータが無ければ SCL stretch でバスを保持し、 `write` が来たら
応答を載せて解除する」プリミティブ (= 窓モデルの `tx_underrun=stretch` 経路)。 この土台の上に
**アクセサが応答ポリシーを載せる**:

- **`SlaveStreamAccessor`** (基底): プロトコルブリッジやカスタムデバイス向けの純ストリーム。
  通常は **`serve(Source* src, Sink* dst, timeout)`** で 1 取引を給仕する (`while (running)
  acc.serve(&src, &sink);`)。 master が write した取引なら受信を `dst` Sink へ、 read した取引なら
  `src` Source から応答を送出し、 STOP で**受信したデータ量** (write 取引は dst Sink に受けた数、 read 取引は 0) を返す。 direction は serve() 内で実行時に
  判別する (バスは direction を露出しない) — 受信が無いときだけ応答を compose するので、 write
  する master が無駄な応答生成を起こさない。
  - **back-pressure は自動 (backend の RING/HW 層が保証)**: serve() は欠落保証ロジックを持たない。
    **write 側** = `dst` Sink が一杯 (`reserve()` が短い / ゼロ span を返す) になると drain を止め、
    backend が **RX_FULL stretch** で master を待たせる。 serve() が Sink を捌いて ring に空きが
    できると stretch が解け、 master が再開する (1 取引で `kRxCapacity` を遥かに超える write を
    欠落なく受けられる)。 **read 側** = 応答は **`kTxCapacity` = 64B の 2 の冪リング** (RX リングの
    対称物) を介して送出され、 long read が送出 FIFO を空にするたびに `src` Source から再充填される
    (空になる手前で先回り補充する。 reactive な **TX_EMPTY stretch** は fallback)。 これで read
    側も取引あたり上限が無く、 1 取引で 1KB の応答をストリーミングできる。
  - **stall escape (`timeout_ms`)**: `timeout_ms` は「無進展 (stall) で master を SCL stretch
    保持する最大時間」。 既定 `TIMEOUT_FOREVER` は永久 hold = **ゼロロス** — `dst` Sink が master の
    write 全量を受けられる容量を持つ前提なら常にこれでよい。 容量制限の Sink (や `dst == nullptr`)
    が write の途中で満杯になると `reserve()` が 0 を返し drain が止まる → RX_FULL stretch のまま
    master が待たされる。 **Sink が `closed()` を返すなら** (固定 `MemorySink` が満杯 / `LimitedSink` が
    cap 到達) `TIMEOUT_FOREVER` でも **即 escape** する — closed Sink がバスを wedge することはない
    (残りを破棄して `TIMEOUT_ERROR`)。 一方 **`closed()==false` の full-but-open な Sink** (リング等で
    一時的に満杯・後で空く) を無期限に待たせたくないときは **有限値**を渡す: その期限を超えて
    1 バイトも drain/fill が進まなければ serve() は **escape** — 残りの write を破棄バッファへ流して
    stretch を解除し master を完走させ、 `TIMEOUT_ERROR` を返す。 backend 非依存 (公開面の
    `read`/`readableBytes` で完結)。 **`TIMEOUT_ERROR` を受けた caller の作法**: `dst` Sink に入った
    分は有効 (`sink.written()` で回収可) — 部分成功 + エラーの二重結果として扱う。 捨てた tail は
    具象 backend の診断 `rxOverflowCount()` (software / espidf にあり、 抽象 `ISlaveBus` には無い) でも
    surface されるが、 **移植性が要るコードは `serve()` の `TIMEOUT_ERROR` を一次シグナルにする**。
    最大 write 長が読めないなら有限 `timeout_ms` を必ず付ける (`TIMEOUT_FOREVER` + 小 Sink は wedge の素)。
  - **イベント駆動 consumer**: serve() の待ちは固定 poll 遅延ではなく `waitForActivity` で
    backend のアクティビティ (RX 着信 / STOP / TX 空き) を待つ。 ISR を持つ backend (espidf LL) は
    割込通知で起こすため、 sub-1ms の小 write でも ring 境界でテールを落とさず即ドレインできる。
    polling backend (software / v2 driver) は既定 1ms poll にフォールバックする。
  - **低レベル窓 seam**: 自作 tick ループや特殊プロトコルでは `read` → `write` で応答を能動構成
    できる。 応答を `write` した後は `transactionComplete()` が真になるまで取引を開いたまま待って
    から `endTransaction` する (早い `endTransaction` は read 途中の応答を破棄)。 純粋な write 取引
    は STOP で即 `transactionComplete()` が真。 serve() はこの seam を検証済み順序で合成したもの。
- **`SlaveRegMapAccessor`** (基底の上のアダプタ): レジスタファイル
  (`data::DataSpan`、 アプリ所有の backing 配列) + 8-bit ポインタ + auto-increment を持ち、
  master の write `[reg][data...]` でポインタ設定 + レジスタ書込み、 read はポインタから
  auto-increment 供給 (write-then-read は 1 窓完結) を**自動給仕**する。 アプリ側は
  `getRegister`/`setRegister` で backing 配列を直接読み書きし (hook は発火しない)、 任意 hook
  (`onWrite(reg,val)` = 書込副作用 / `onRead(reg)->val` = just-in-time の live・計算値) を挿せる。
  hook は **関数ポインタ + `void* ctx`** (hal/v2 house style、 `std::function` 不使用)。
  内部に `SlaveStreamAccessor` を合成する。 **byte 意味論**: read 1 byte =
  `onRead(p)` があればその戻り値、 無ければ `reg_file[p]` (p は窓内 auto-increment、 8-bit wrap)。
  write の先頭 byte = ポインタ設定、 以降の byte = `reg_file[p+offset]=val` + `onWrite(p+offset,val)`。
  **ポインタは取引を跨いで保持**され、 SPLIT (register write → STOP → 別取引の pure read) は
  先行 write が設定したポインタに対して解決する。 アクセサは受信を逐次適用するため自前バッファの
  上限を持たない。 backend の RX は **`kRxCapacity` の 2 の冪リング** (software backend = 64B、
  espidf LL backend = 32B)。 これは取引あたりの通算上限ではなく **未読バックログの上限** — 利用側 (`serve()` /
  tick ループ) が `read()` で捌き続ける限り、 1 取引で `kRxCapacity` を遥かに超えるバイト
  (例: 256B レジスタファイルの一括 write) を受信できる。 **未読が `kRxCapacity` に達した時のみ
  超過バイトが破棄される** (= consumer が追いつかなかった合図)。 破棄は silent でなく backend の
  `rxOverflowCount()` (software / espidf) で検知でき、 非ゼロ = consumer 遅延 (または read 未実行) を表す。
  なお espidf の **非 LL callback 経路** は STOP 後に取引全体を一括受領しドレイン余地が無いため、
  そこでは `kRxCapacity` が硬い取引あたり上限のまま (末尾切り詰め)。 **read 側 (master の >64B
  読み出し)**: backend の TX は上記のとおり `kTxCapacity` リング + ストリーミングなので取引あたり
  上限は無いが、 `SlaveRegMapAccessor` は取引ごとに `kReplyWindowBytes` = 64B の応答窓を 1 つだけ
  compose する設計のため、 **1 回の RegMap read は最大 64B** (レジスタマップ規約であって backend 制約
  ではない)。 レジスタデバイスから 64B 超を読むときはポインタを再シードした複数 read に分ける。
  取引あたり無制限の純ストリーム read が要るなら基底の `SlaveStreamAccessor` を直接使う
  (応答を逐次 `write` で供給する。 echo デバイスがこの形)。

  最小例 (温度センサ風、 `onRead` で live 値を just-in-time 合成):

  ```cpp
  struct State { uint8_t regs[256] = {}; int16_t temp = 0; };
  State state;

  uint8_t onRead(uint8_t reg, void* ctx) {        // hook 無しなら reg_file[reg] が返る
      auto* s = static_cast<State*>(ctx);
      if (reg == 0x10) return uint8_t(s->temp & 0xFF);
      if (reg == 0x11) return uint8_t((s->temp >> 8) & 0xFF);
      return s->regs[reg];                         // それ以外は backing 配列を返す
  }

  SlaveRegMapAccessor regs{bus, data::DataSpan{state.regs, sizeof(state.regs)}};
  regs.setOnRead(onRead, &state);
  for (;;) { state.temp = readSensor(); regs.serve(); }  // serve は 1 取引 → ループ必須
  ```

  給仕は 2 形態:
  - **`serve(timeout)`** = ブロッキング便利関数 (`while(running) acc.serve();`)。 backend ISR が
    bus を進める espidf 系の app ループ向け。 検証済みの順序 = 「初回 read でポインタ確定 →
    応答 compose (書込適用前) → write → 書込相を complete まで完全ドレイン (available を全部
    読んでから complete 判定) → end」。 待ちは基底アクセサと同じ `waitForActivity` で
    イベント駆動 (ISR backend は通知で起床)。 `timeout_ms` の stall-escape も同義 (既定
    `TIMEOUT_FOREVER` は完走まで給仕、 有限値は無進展でその取引を諦めて返る)。
  - **取引ステップ API** (`beginExchange`/`ingest`/`composeReply`) = 純レジスタマップ・ロジックを
    bus I/O から分離して公開したもの。 `serve()` が上記順序で合成する素であり、 **単一スレッドで
    bus が ServiceRunner tick で進むモデル** (`serve()` のブロッキング poll が使えない) 向けに
    自作 tick ループを組む素材になる。 同じ seam を native 決定論テストが叩く。

  **実行モデルの要点**: espidf は `serve()` をブロッキングで app ループから回せる (bus は
  ISR で進む)。 software backend の native は単一スレッドで bus が ServiceRunner tick で進むため、
  `serve()` のブロッキング poll はデッドラインを生む — そのモデルでは取引ステップ API を tick
  ループから駆動する。

応答の組成は既定で**タスク文脈** (stretch が master を保持する間にアクセサ層が応答を作る)。
stretch を持つ SoC では応答に遅延があっても master を確実に待たせられるため、 ISR 同期でなく
タスク文脈での任意ロジックが成立する。 レジスタマップを ISR 内で直接給仕する高速パスは
stretch 対応 SoC 限定の最適化として後続段階に置く。

backend 実装:
- `SlaveBus_software` は `SlaveLineDriver` 越しに SCL/SDA を観測・drive する cooperative
  protocol engine (`service::IService`、 ServiceRunner で poll)。 native test では
  `VirtualOpenDrainBus` と組み合わせ、 probe ACK、 write、 read-only、 write-then-read、
  address NACK、 data NACK、 clock stretch timeout、 STOP 時 SDA stuck-low、 read 末尾
  master NACK 観測に加え、 窓の分離・Tx 自動消滅・underrun fill を固定している。
- 実機向け ESP-IDF backend は ESP32-S3 の HW clock stretch を使い上記プリミティブを提供する。
  write-then-read の HW 堅牢化は **stretch 対応 SoC (ESP32-S3) 限定** — ESP32 無印は HW slave
  stretch 非対応のため fill のみの best-effort (≤400kHz) となる。 remote 公開と arduino
  backend は後続段階。
  - **ISR の IRAM 配置 (`M5HAL_ESPIDF_I2C_SLAVE_IRAM_ISR`、 既定 1)**: LL stretch backend の
    slave ISR と到達コードは既定で IRAM に置き、 `ESP_INTR_FLAG_IRAM` で登録する — flash cache が
    無効な間 (別タスクの OTA / NVS / SPIFFS write 中) も外部 master にクロックされる slave が応答を
    続けられるようにするため (ISR が遅延すると stretch 中の master が取引途中で stall / timeout する)。
    この堅牢性は IRAM を ~1〜2KB 消費する。 **I2C slave 稼働中に flash を書かないと保証できるビルドは
    `M5HAL_ESPIDF_I2C_SLAVE_IRAM_ISR=0` で opt-out** でき、 IRAM_ATTR と IRAM 割込フラグの両方が外れて
    IRAM を回収する (代償 = flash-cache 無効窓中の slave 応答は保証されない)。
- 実機 bit-bang slave は edge 捕捉・ACK setup の timing 制約が厳しいため低クロック実験用と
  位置付ける。

## 関連

- [i2c.md](i2c.md) — I2C master 体系・設定型・pin 規約
- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離 + RAII + 排他制御の意味論
- [data_io.md](data_io.md) — Source / Sink + Limited 装飾
