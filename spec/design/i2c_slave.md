# i2c slave — スレーブ機構

> **読者**: 実装者・レビュー向け（設計仕様）。

I2C slave の Bus / Accessor / 設定型 / 給仕モデルの仕様。 master 体系は [i2c.md](i2c.md)、 共通基底は [bus_accessor.md](bus_accessor.md) を参照。

## 目次

- [API 概要](#api-概要)
- [backend の構築とコントローラの占有](#backend-の構築とコントローラの占有)
- [serve() の polarity と timeout](#serve-の-polarity-と-timeout)
- [トランザクション窓モデル](#トランザクション窓モデル)
- [アクセサの選択 (Stream vs RegMap)](#アクセサの選択-stream-vs-regmap)
- [応答ポリシーの詳細](#応答ポリシーの詳細)
- [ISR regmap fast path (`bindIsrRegMap`)](#isr-regmap-fast-path-bindisrregmap)
- [backend 実装](#backend-実装)
- [関連](#関連)

## API 概要

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
    int8_t controller            = -1;     // 占有するハードウェアコントローラ (§コントローラの占有)
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

## backend の構築とコントローラの占有

**backend の構築**: 実機 (ESP32-S3 系) は `SlaveBus_espidf bus; bus.init(cfg);` で済む
(`cfg` は `SlaveBusConfig`)。 software / native backend (`SlaveBus_software`) は SCL/SDA を
駆動する `SlaveLineDriver` を渡す `init(SlaveLineDriver&, cfg)` を使う (`init(cfg)` 単独呼びは
`INVALID_ARGUMENT`)。 software backend を native の ServiceRunner tick から進めるモデルでは
`ScopedSlaveServiceRegistration{runner, bus}` で service を runner に登録する (`serve()` の
ブロッキング poll が使えない単一スレッド向け、後述の取引ステップ API と組む)。

**コントローラの占有 (`SlaveBusConfig::controller`)**: 実機の I2C ハードウェアコントローラは
master 体系と同じ有限プールを共有する物理資源であり、 台帳を経ずに固定ポートへ直書きすると
同じポートを使う master バスのレジスタを破壊しうる。 `SlaveBusConfig::controller` はどの
コントローラを占有するかを指定する欄で、 正しい値は `bus::BusView::claimController(intent)`
(既定 = Auto、 最小空きコントローラ) で取得する:

```cpp
auto claim = M5_Hal.I2C.claimController();  // 既定 = Auto
if (!claim.has_value()) { /* 全コントローラ使用中、または不適格 */ }
cfg.controller = claim.value();
auto init = slave_bus.init(cfg);
if (!init.has_value()) {
    // init に失敗した slave は港を掴んでいない — claim だけ返却する
    M5_Hal.I2C.releaseClaimedController(claim.value());
}
// 使い終わったら「先に slave を止め、止まってから」claim を返却する。
// 逆順 (claim を先に返す) は、slave がまだ ISR とレジスタを掴んでいる間に
// master 側の再配置が同じ港を確保・再設定できてしまう
if (slave_bus.release().has_value()) {
    M5_Hal.I2C.releaseClaimedController(claim.value());
}
```

占有した区画は master 側の `commitBuses()` (バス再配置) を生き延び、 自動割当も明示指名
(`requireController`/`preferController`) もこの区画を master へ渡さない。 逆に master が
先に同じコントローラを保持している場合、 `claimController` は `OUT_OF_RESOURCE` を返す
(明示指名で不適格なコントローラを指した場合は `INVALID_ARGUMENT`)。

`controller = -1` (既定値) は「backend の既定ポートを直接使う」を意味し、 **プールの台帳に
一切載らない** (自己責任 — 同じポートを使う master バスとの衝突を検出できない)。 単体テストや
HIL ベンチなど、 プールを経由しない生成経路との互換のために残した既定であり、 実運用では
`claimController` 経由での取得を推奨する。

低電力ドメインのコントローラ (LP_I2C) は指定できない — clock-stretch / ISR ベースの slave
機構は HP コントローラ専用で、 LP 区画を指した `controller` は `init()` が `INVALID_ARGUMENT`
で拒否する。

## serve() の polarity と timeout

**`serve(src, dst, timeout)` の polarity と nullptr** (`src`/`dst` は master API と同じ「この
accessor が送る / 受ける」向き。 slave 視点ではなく master 動作を主語に読むと迷わない):

| 引数 | 型 | これが効く master 取引 | slave 側の意味 | `nullptr` のとき |
|---|---|---|---|---|
| `src` | `Source*` | master read | slave が**送出**する応答データ | read 応答を作らない → backend の `tx_underrun` policy (fill / stretch) が出る |
| `dst` | `Sink*` | master write | slave が**受信**する要求データ | write 受信を捨てる → drain しないので有限 `timeout_ms` 必須 (さもないと RX_FULL stretch で wedge) |

**timeout は 3 つあり対象が違う** (いずれも `*timeout*` 名なので取り違え注意):

| 設定 | 対象 | 期限時の挙動 |
|---|---|---|
| `serve(…, timeout_ms)` | 取引開始待ち + 取引中の無進展 (stall)。**wall-clock 総時間ではない** | `TIMEOUT_ERROR`。取引中 stall なら残り write を discard して master を完走させる (discard 中も master が無活動のままなら deadline もう 1 回分で取引ごと放棄 — 有限 timeout は必ず返る) |
| `SlaveBusConfig::timeout_ms` | 低レベル `read()` が現窓の RX byte を待つ予算 | 0 byte の短読み (通常 error ではない) |
| `SlaveBusConfig::stretch_timeout_ms` | `tx_underrun=stretch` で窓未オープン / 応答未投入の read を stretch 保持する上限 | `tx_fill_byte` 送出へ fallback |

## トランザクション窓モデル

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
  (software backend は両対応。 ESP32 無印の HW slave は stretch 非対応で
  `TxUnderrun::Stretch` は `init()` が `INVALID_ARGUMENT` で拒否する — 無印の
  レジスタマップ給仕は後述の **ISR fast path** が担う)

## アクセサの選択 (Stream vs RegMap)

**どちらのアクセサを使うか**:
- レジスタアドレス + auto-increment + ポインタ保持の I2C 定番デバイス (センサ / PMIC / 設定
  レジスタ) を作る → **`SlaveRegMapAccessor`**。 `serve()` を回すだけで、 `onRead` で live 値・
  `onWrite` でコマンド副作用を挿す。 read 長は無制限 (`kReplyWindowBytes` = 64B はストリームの
  compose チャンクであって上限ではない。 master が読み続ける限り 8bit wrap で auto-increment
  し続ける — 定番デバイスと同じ)。 注意: 応答はワイヤより先行して compose される (上界 =
  backend TX キュー深さ + 1 チャンク。 同梱 backend では 1 byte read でも最大 2 チャンク =
  128B 分 `onRead` が発火し得る。 旧・単窓設計でも read 長に依らず 64B 分発火していた)。
  read-to-clear 型レジスタは先行発火を許容できる設計にする。
- 可変長フレーム / echo / ブリッジ / 独自プロトコル → 基底
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

## 応答ポリシーの詳細

**backend = stretch プリミティブ / accessor = 応答ポリシー**: backend が提供する土台は
「master の read 要求に対しデータが無ければ SCL stretch でバスを保持し、 `write` が来たら
応答を載せて解除する」プリミティブ (= 窓モデルの `tx_underrun=stretch` 経路)。 stretch を
持たない SoC (ESP32 無印) ではこのプリミティブは提供できず、 代わりに backend が
`bindIsrRegMap` (後述の ISR fast path) を提供する。 この土台の上に
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
  上限を持たない。 backend の RX は **2 の冪リング**で、 上限は取引あたりの通算でなく
  **未読バックログ** に対して効く — 利用側 (`serve()` / tick ループ) が `read()` で捌き続ける
  限り、 1 取引でリングを遥かに超えるバイト (例: 256B レジスタファイルの一括 write) を受信
  できる。 バックログ超過時の挙動は backend で異なる: **software backend** (64B) は超過バイトを
  破棄し `rxOverflowCount()` で検知できる (非ゼロ = consumer 遅延または read 未実行)。
  **espidf LL backend** は未読が back-pressure 閾値 `kRxCapacity` (32B) に達すると **SCL
  stretch hold で master を停止し、 破棄しない** (read() の空き待ち)。 STOP 時だけは以後の
  stretch で master を止められないため、 HW FIFO に残った尻尾 (≤32B) を閾値の先の **STOP 尻尾
  予備領域** (リング物理 `kRxArrayCapacity` = 64B) へ格納する — consumer が微遅延しても
  33〜64B write の尻尾は失われず、 `rxOverflowCount()` は防御経路 (取引スロット枯渇等) でしか
  増えない。 **espidf BE flavor** (ESP32 無印の既定、 後述) は ISR が逐次 drain するが
  stretch が無いため back-pressure は掛けられない — リング超過は破棄され
  `rxOverflowCount()` が増える。 ただし regmap ISR fast path が bind されている間は受信を
  リング非経由でレジスタファイルへ直接適用するため、 この経路では増えない。 なお espidf の
  **非 LL callback 経路** (v2 driver、 BE の LL ヘッダが揃わない構成のみのフォールバック) は
  STOP 後に取引全体を一括受領しドレイン余地が無いため、 そこでは `kRxArrayCapacity` (64B)
  が硬い取引あたり上限 (末尾切り詰め、 `rxOverflowCount()` で検知)。 **read 側 (master の >64B
  読み出し)**: backend の TX は上記のとおり `kTxCapacity` リング + ストリーミングで取引あたり
  上限が無く、 `SlaveRegMapAccessor::serve()` もリングの空きに合わせて `kReplyWindowBytes` = 64B
  チャンクを継続 compose するため、 **1 回の RegMap read に長さ上限は無い** (8-bit wrap の
  auto-increment で読み続けられる。 参考実装 ESP32_I2C_slave_example と同じ意味論)。 compose は
  ワイヤより最大 1 チャンク先行し、 読まれなかった分は STOP で破棄される (`onRead` の先行発火に
  注意 — §どちらのアクセサを使うか)。 独自プロトコルの純ストリーム read は基底の
  `SlaveStreamAccessor` を直接使う (応答を逐次 `write` で供給する。 echo デバイスがこの形)。

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
    `TIMEOUT_FOREVER` は完走まで給仕、 有限値は無進展でその取引を諦めて返る)。 regmap の
    consumer はワイヤが動く限り必ず進む (受信は即レジスタ適用・応答 pump は tx ring 有界) ため、
    取引中の stall = master 側の無活動のみ — escape は discard 段階を持たず即時放棄で、 放棄前に
    適用済みのレジスタ書込はそのまま残る (書込途中で切断された実レジスタデバイスと同じ意味論)。
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
タスク文脈での任意ロジックが成立する。

## ISR regmap fast path (`bindIsrRegMap`)

**ISR regmap fast path (`bindIsrRegMap`)**: stretch を持たない SoC (ESP32 無印) では
タスク文脈の組成が write-then-read (repeated start) の間合い (400kHz で数十 µs) に原理的に
間に合わないため、 レジスタマップの給仕自体を backend の ISR へ委譲する経路を持つ。
契約は `ISlaveBus` の optional 仮想関数 (既定実装は `false` = 未対応):

```cpp
using RegMapOnReadFn  = uint8_t (*)(uint8_t reg, void *ctx);
using RegMapOnWriteFn = void (*)(uint8_t reg, uint8_t value, void *ctx);
struct IsrRegMapBinding {
    data::DataSpan reg_file{};
    RegMapOnReadFn on_read   = nullptr;
    void *on_read_ctx        = nullptr;
    RegMapOnWriteFn on_write = nullptr;
    void *on_write_ctx       = nullptr;
    uint8_t pointer       = 0;
    bool pointer_received = false;
    uint32_t write_offset = 0;
    uint32_t tx_offset    = 0;
};
virtual bool bindIsrRegMap(IsrRegMapBinding *binding);
virtual void unbindIsrRegMap(IsrRegMapBinding *binding);
```

`binding` は呼び出し側 (`SlaveRegMapAccessor`) が所有し、bind の間じゅう存続させる (デストラクタで
unbind する)。backend は単一の可変スロットだけを持ち、bind は常に「後勝ち」— 新しい bind は前の
binding に触れず単にスロットを差し替える。unbind は渡された `binding` が現在のスロットと一致する
ときだけ解除する (所有権照合。一致しなければ no-op — 既に他の bind に上書きされた古い binding を
誤って解除しない)。

`SlaveRegMapAccessor` が構築時 (および hook 差し替え時) に自動で bind を試み、 成立した
backend では受信の解釈 (先頭 byte = ポインタ、 以降 = レジスタ書込み + `onWrite`) と読出し
応答の先読み充填 (`onRead` / `reg_file` から TX FIFO を再構成) を **ISR が直接**行う。
`serve()` は取引完了待ちに縮退し、 アプリの使い方は変わらない。 制約:

- **hook は ISR 文脈で呼ばれる**: `onRead`/`onWrite` は IRAM 配置 (`IRAM_ATTR`)・非ブロッキング・
  短時間で返ることが**アプリの責務**になる (タスク文脈給仕の backend では従来どおり)
- `reg_file` へのアプリ直接アクセス (`getRegister`/`setRegister`) は byte 単位アトミック前提
  (従来と同じ保証水準 — 複数 byte のスナップショット一貫性は保証しない)
- 読出し応答は wire より最大 TX FIFO 深さ (32B) 先行して充填される。 `onRead` の live 値は
  「master が読む瞬間」でなく「充填された瞬間」の値になり、 呼び出し回数も読出し byte 数とは
  一致しない (per-read 厳密カウンタ的な意味論は stretch 対応 SoC 限定)
- best-effort: stretch が無いため ISR が間に合わない場合 (長い割込み禁止区間との重なり等) に
  古い応答が返る・ゼロ間隔連続 write が連結解釈される可能性は残る (稀。 確実性が要る用途は
  stretch 対応 SoC を使う)

## backend 実装

- `SlaveBus_software` は `SlaveLineDriver` 越しに SCL/SDA を観測・drive する cooperative
  protocol engine (`service::IService`、 ServiceRunner で poll)。 native test では
  `VirtualOpenDrainBus` と組み合わせ、 probe ACK、 write、 read-only、 write-then-read、
  address NACK、 data NACK、 clock stretch timeout、 STOP 時 SDA stuck-low、 read 末尾
  master NACK 観測に加え、 窓の分離・Tx 自動消滅・underrun fill を固定している。
- 実機向け ESP-IDF backend (`SlaveBus_espidf`) は 2 flavor をコンパイル時に自動選択する:
  **LL flavor** (stretch-cause SoC: S2/S3/C3/C6/H2/P4 等) は HW clock stretch で上記
  プリミティブを提供し、 write-then-read を HW 保証する。 **BE (best-effort) flavor**
  (ESP32 無印) は同じ `i2c_ll_*` 直叩き構造 (IDF driver / Kconfig 非依存) で、 stretch の
  代わりに上記 ISR regmap fast path で write-then-read を給仕する — 受入基準は
  100/400kHz、 800kHz は参考 (informational)。 fast path を bind しない純ストリーム利用
  (`SlaveStreamAccessor`) の BE は **fill 意味論**: read 開始までに応答が投入されて
  いなければ、 その read は `tx_fill_byte` で埋まる (途中投入の反映は FIFO 水位補充以降)。
  remote 公開と arduino backend は後続段階 (BE flavor 自体は Kconfig 非依存のため
  arduino ビルドでもコンパイル対象になる)。
  - **ISR の IRAM 配置 (`M5HAL_CONFIG_ESPIDF_I2C_SLAVE_IRAM_ISR`、 既定 1)**: LL stretch backend の
    slave ISR と到達コードは既定で IRAM に置き、 `ESP_INTR_FLAG_IRAM` で登録する — flash cache が
    無効な間 (別タスクの OTA / NVS / SPIFFS write 中) も外部 master にクロックされる slave が応答を
    続けられるようにするため (ISR が遅延すると stretch 中の master が取引途中で stall / timeout する)。
    この堅牢性は IRAM を ~1〜2KB 消費する。 **I2C slave 稼働中に flash を書かないと保証できるビルドは
    `M5HAL_CONFIG_ESPIDF_I2C_SLAVE_IRAM_ISR=0` で opt-out** でき、 IRAM_ATTR と IRAM 割込フラグの両方が外れて
    IRAM を回収する (代償 = flash-cache 無効窓中の slave 応答は保証されない)。
  - **既知の限界 (LL flavor 一部 SoC・800kHz)**: H2 系など一部 SoC では、 multi-slave バス上で
    エラーが高頻度に連続する 800kHz 運用下において、 slave 側が SW 再初期化 (release/init・
    クロックゲート再構成を含む) でも回復しない状態に陥りうることが実測で確認されている
    ([esp-idf#15444](https://github.com/espressif/esp-idf/issues/15444) と同系統の master 側
    ISR/timeout レースが引き金であり、 M5HAL 固有のバグではない)。 この制約により
    **実用推奨は 400kHz まで**とし、 800kHz は BE flavor と同様に参考 (informational) 止まりとする。
    回復にはチップリセット相当の操作が必要。
- 実機 bit-bang slave は edge 捕捉・ACK setup の timing 制約が厳しいため低クロック実験用と
  位置付ける。

## 関連

- [i2c.md](i2c.md) — I2C master 体系・設定型・pin 規約
- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離 + RAII + 排他制御の意味論
- [data_io.md](data_io.md) — Source / Sink + Limited 装飾
