# design/bus_accessor — Bus と Accessor の責務分離

> **読者**: 実装者・レビュー向け（設計仕様）。

通信バスへのアクセスは **Bus と Accessor の 2 層構造** で表現する (I2C / SPI / UART 共通)。 利用者には「Accessor を通してバスを操作する」 単一の流儀を提供しつつ、 高度な利用者には sugar を介さず Bus を直接呼び出す低レイヤ経路も開けておく。

- **Bus** = 物理バス (例: ESP32 の `I2C_NUM_0`) を表す。 1 つの物理リソースに対し排他制御を持つ
- **Accessor** = 1 つの通信相手 (例: アドレス 0x76 の BME280) を表す。 通信パラメータ (速度・タイムアウト等) を保持

利用者は Accessor 経由で通信する (Bus は排他制御の単位として裏で動く)。 「なぜこの形なのか」の設計根拠は後半 (§なぜ 2 層構造か 以降) にまとめる。

## Bus の責務 (最小化)

- 物理バスの初期化と解放 (`init` / `release`)
- 外部 native handle (Arduino `TwoWire` 等) との紐付け (`attach`)
- 排他制御 (`lock(Accessor*)` / `unlock(Accessor*)`)
- atomic な通信動作 (`transfer`)

Bus は **Accessor を所有しない**。 利用者が Bus と Accessor を別個に保持する (v0.0.x の `Bus::beginAccess(AccessConfig&) -> Accessor*` 形 factory は撤廃済)。

## Accessor の責務

- 1 つの通信相手の表現 (I2C アドレス + 通信パラメータ等を `AccessConfig` で保持)
- アクセス期間の宣言 (`beginAccess` / `endAccess`、 内部で Bus を lock/unlock)
- 利用者向け sugar (`write` / `read` / `transfer` / `writeRegister` / `readRegister` / `probe`)
- 連続 atomic のための **depth counter** (`_access_depth`) — sugar 内部呼び出しと外側の明示 `beginAccess` を入れ子で扱えるよう、 1 段の lock で吸収する

SPI の CS assert/deassert のような kind 固有の電気的 session は、 bus 共通の
`beginAccess` / `endAccess` には含めない。 `beginAccess` はあくまで排他期間で、
複数の kind 固有 transaction を含んでよい。

**`beginAccess` と `beginTransaction` は別概念軸**で、 動詞の使い分けはこの区別を反映する
(両者があるのは分散ではなく二層構造):

| 動詞 | 表すもの | 使う kind |
|---|---|---|
| `beginAccess` / `endAccess` | **排他期間 (bus ロックの取得・解放)** — 複数の transaction を内包し得る | i2c / spi / uart / i2s 共通 (基底) |
| `beginTransaction` / `endTransaction` | **1 つの取引区間** (下表。`endTransaction` は区間の `TransferTotals` を返す) | i2c / spi / uart / i2s 共通 |

`beginTransaction` は**全 kind 共通の「相手との 1 取引区間」**であり、区間中は他 accessor
(= 他スレッド) が同じチャネル/バスに割り込めない。区間が物理的に何を維持するかだけが
kind ごとに異なる:

| kind | transaction 区間の物理保証 |
|---|---|
| SPI | CS assert 区間 (1 フレーム) |
| I2C master | bus 占有セッション (multi-master arbitration の窓)。slave 側は master 1 取引の窓 (低レベル seam、`serve()` が内部で合成) |
| UART | TX (RX) チャネル排他区間のみ (物理的な区間概念は無い。排他 + 集計に縮退) |
| I2S | 同上 |

UART/I2S へ transaction を置く動機は「複数スレッドが同じ Bus を共有し、それぞれが複数 write を
1 まとまりで送る」場面の排他を、kind に依らず同じ動詞で書けるようにすること。
非同期進行 (`transferBusy` / `waitTransfer`) は I2C/SPI のみで、stream kind には提供しない。

**共有の単位 (契約)**: `Bus` はスレッド間で共有してよいが、**`Accessor` は共有してはならない**
(スレッドごとに accessor を作って同じ Bus に bind する)。depth counter は同一 owner の再入用で
あり、accessor 自体はスレッド安全ではない。UART/I2S の TX と RX は独立チャネルロックなので、
transaction も `TxAccessor` / `RxAccessor` それぞれが独立に表現する (全二重で書きながら読める)。

## 二層の利用路線 (混在禁止)

通信路は以下 2 路線が併存。 **同じ Bus 上で路線をまたいで操作することは未サポート** (利用者責任)。

### 通常路線 (Accessor sugar 利用)

```cpp
m5::hal::v2::i2c::MasterAccessor accessor{i2c_bus, acc_cfg};

// 単発: sugar 内部で beginAccess → transfer → endAccess
accessor.writeRegister(REG_CTRL, VAL_MODE);

// 連続 atomic: ScopedAccess で beginAccess / endAccess を RAII
// (timeout は明示が本筋。省略 = 無限待ちのシュガー)
{
    m5::hal::v2::bus::ScopedAccess access{accessor, 100};
    if (access.has_error()) { /* lock 競合 (timeout) */ return; }
    accessor.writeRegister(REG_CTRL, VAL_MODE);
    accessor.readRegister(REG_DATA, dst_span);
}  // ここで endAccess
```

- 利用者は排他制御を意識しない
- `beginAccess`/`endAccess` を sugar 内部でも外側でも呼べる (depth counter で吸収)

SPI では、単発 sugar は内部で `beginAccess → beginTransaction → transfer →
endTransaction → endAccess` を行う。明示的に CS を維持したい場合は、
`beginAccess` ではなく SPI 固有の `beginTransaction` を使う。

```cpp
m5::hal::v2::spi::MasterAccessor dev{spi_bus, spi_cfg};

// 単発: CS はこの write の前後だけ assert される
dev.write(tx_span);

// 複数 transfer で CS を維持する場合
dev.beginTransaction();
dev.write(command_span);
dev.write(data_span);
dev.endTransaction();
```

### 低レイヤ路線 (Bus 直接呼び出し、 「熟知している前提」)

```cpp
// bus.lock(&accessor) → bus.transfer(&accessor, ...) → bus.unlock(&accessor)
m5::hal::v2::bus::ScopedLock lock{i2c_bus, &accessor};
if (lock.has_error()) return;

i2c_bus.transfer(&accessor, cfg, desc, &tx_source, &rx_sink);
```

- Accessor を作るのは必須 (`lock` owner は常に valid な `Accessor*`、 nullptr 不可)
- 「他者が unlock しちゃう」 誤用は identity 比較で検出可能

## Bus の保持

bus の入手・共有は **全 kind 共通の所有レジストリモデル**。I2C / SPI はさらに intent 駆動の
HW 割当を持つ。UART / I2S は同じ `commitBuses()` / `hardwareInUse()` / logical acquire
surface を持つが、現時点では static-backend policy で運用する。

### 全 kind = 所有レジストリ (identity acquire) — `M5_Hal.<KIND>`

I2C / SPI / UART / I2S はすべて **`M5_Hal` が所有する共有 `bus::BusRegistry` に bus をインターンする**。
利用者は物理配線 (ピン) で `acquire(cfg)` し、所有権を持つ `shared_ptr` を取得する:

```cpp
auto i2c  = M5_Hal.I2C.acquire(m5hal::i2c::BusConfig{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}});
auto spi  = M5_Hal.SPI.acquire(spi_cfg);    // identity = CLK / MOSI / MISO
auto uart = M5_Hal.UART.acquire(m5hal::uart::BusConfig{m5hal::uart::Tx{17}, m5hal::uart::Rx{16}});
auto i2s  = M5_Hal.I2S.acquire(i2s_cfg);    // identity = BCLK / WS / DOUT / DIN
if (!i2c) { /* INVALID_ARGUMENT (pin 未設定) / backend init 失敗 / OUT_OF_RESOURCE / NOT_CONNECTED (init()/connect() 前の自作 Hal — M5_Hal では発生しない) */ }
m5hal::i2c::MasterAccessor dev{i2c.value(), acc_cfg};  // shared_ptr 直渡し = accessor が bus を co-own
```

- **identity = 物理配線のコアピンのみ** (kind 別: I2C={SCL,SDA} / SPI={CLK,MOSI,MISO} / UART={TX,RX} /
  I2S={BCLK,WS,DOUT,DIN})。**同一配線 = 単一インスタンス** — ボード層とユーザコードが同じピンを acquire
  すると**同じ bus (同じ lock) を共有**する (別々に作ると別 mutex になりバスが壊れる、を防ぐ correctness
  要件)。どの backend / Wire / port が駆動するかは identity でなく「配線の駆動手段」で、**最初の acquire が
  勝つ**。ただし、同一 identity の既存 bus に対して identity 外の bus-level config
  (SPI `pin_dc` / quad data pins、UART RTS/CTS・buffer size、I2S MCLK・buffer size・role 等) が
  食い違う typed acquire は `INVALID_STATE` を返す。これは first-config-wins によるサイレントな
  設定取り違えを避ける診断であり、既存 bus の再構成はしない。配線の駆動手段そのもの
  (Arduino `Wire*`、POSIX `device_path` 等) は現行 registry identity では表現しないため、初回
  acquire の config が有効である点は変わらない。
  *(リモートは実装済み: `Hal::initUart` / `initTcp` で接続した `Hal` インスタンスの
  `hal.<KIND>.acquire` が同型でリモートプロキシを返す — [remote.md](remote.md) §Hal facade。
  接続束縛は排他 = 1 `Hal` は 1 デバイスの窓。)*
- **戻り値 = `result_t<shared_ptr<kind::IBus>>`** — 成功なら shared_ptr、失敗は明示 error。
  「範囲外 / 未登録がどちらも null」という旧 slot API の曖昧は構造的に消える。
- **寿命 = registry が weak インターン**。返した `shared_ptr` が最後まで保持されている間 bus は生き、
  **最後の保持者が手放すと自然解放** (bus dtor → backend release) される。通常のローカル bus は
  weak pointer の失効後、次の registry 操作でスロットを回収する。remote のように外部資源の
  解放確認が必要な bus は lifecycle tombstone を併用し、確認できた場合だけ identity / 外部 ID を
  再利用する。確認不能なら tombstone を保持する (後述および [remote.md](remote.md) §動的バス生成)。
  **accessor を
  `shared_ptr` から直接構築すると accessor が bus を co-own する** (`MasterAccessor dev{i2c.value(), cfg}`) ので、
  accessor が生きている間は bus も生き、利用者が別途 `shared_ptr` を保持し続けなくてよい (acquire の一時値から
  直接渡しても安全)。`IBus&` を渡す構築 (`dev{*i2c.value(), cfg}`) は自前所有のエスケープ用で、その場合は
  従来どおり bus を accessor より長く生かすのが利用者の契約。ボード層 (M5Unified) が ref を持ち続ければ内蔵
  バスは生存する。
- **明示 release = exact-instance の consuming close**。`hal.<KIND>.release(bus)` は non-const
  `shared_ptr&` を取る (各 kind の宣言形は
  `result_t<void> release(std::shared_ptr<IBus>& bus)`)。registry 内の実体がその pointer と一致し、
  かつ caller が唯一の
  strong owner のときだけ実行する。Accessor や別の `shared_ptr` が co-own 中なら `BUSY`、
  同一 identity の別実体や他 registry の bus なら `INVALID_ARGUMENT`。成功時は引数の
  `shared_ptr` を reset する。release 中の同一 identity は acquire できず `BUSY` とする。
  release 判定後に外部 `weak_ptr` が旧実体を復活させても、共有 lifecycle gate がRPC完了前に
  旧実体を閉じるため、その操作は `CLOSED` となる。外部資源の自然解放を確認できない場合は
  registry slotを `Quarantined` tombstoneとして保持し、同一identityの再取得を`BUSY`にする。
  これにより、操作可能な旧 bus と新 bus が別 mutex で同じ配線を駆動する状態を作らない。

明示 release の前に Accessor と alias を破棄する。成功時は `bus` 自体が空になるため、caller が
追加の `reset()` を行う必要はない。失敗時は所有権を caller へ戻すので、原因を除いて同じ handle で
再試行できる。

```cpp
auto bus = hal.I2C.acquire(cfg);
if (!bus) { /* handle error */ }
{
    m5hal::i2c::MasterAccessor device{bus.value(), access_cfg};
    // use device
}  // Accessor の co-own を先に終える
auto released = hal.I2C.release(bus.value());
// success: bus.value() == nullptr; BUSY: alias/Accessor/release がまだ競合中
```

外部 identity を持つ bus は proxy と registry で同じ `BusLifecycle` を共有する。状態遷移は次の
一方向を正本とする。

```text
Open --close開始--> Releasing --成功--> Closed
  ^                      |
  +------失敗rollback----+

Open --自然解放の確認不能--> Quarantined
```

operation は `Open` の間だけ開始でき、開始済み operation と close は同じ gate で直列化される。
`Closed` / `Quarantined` から操作は再開せず `CLOSED`。`Releasing` 中は registry の同一 identity
acquire と別 release を `BUSY` にする。`Quarantined` は接続/backend 全体の cleanup まで identity を
占有し、確認していない外部 ID の ABA 再利用を防ぐ。

- **総数キャップ** (全 kind 合計の固定上限 `BusRegistry::kCapacity`) があり、満杯の acquire は `OUT_OF_RESOURCE`。
- **直接構築も可**: `<kind>::Bus bus; bus.init(cfg);` も引き続き可能 (acquire はインターン共有が要るときの
  導線)。無印 `<kind>::Bus` は runtime facade で、backend は `init` に渡す config 型で選ぶ
  ([variants.md](variants.md) §facade kind / 各 kind の spec)。

### I2C / SPI: intent 駆動の HW 割当

I2C / SPI は上記に加え `acquire(LogicalBusConfig{pins, intent})` + 明示 `commitBuses()` で
HW コントローラ割当を遅延解決し、ロック下で backend を hot-swap (reassign) できる。
`acquire<CfgT>(cfg)` は従来通り config 型で backend を明示固定する経路で、intent 管理対象にはならない。
詳細・`AllocationIntent` / query API は [i2c.md](i2c.md) §intent 駆動の HW 割当、および
[spi.md](spi.md) §Bus の入手。

### UART / I2S: static-backend policy

UART / I2S も `BusView` の形は I2C / SPI と揃える (実装は共有 — §BusView の実装共有)。つまり
`commitBuses()` は呼べるが no-op、`hardwareInUse()` は 0 (`BusTraits::MANAGED_ALLOCATION = false`)、
`acquire(LogicalBusConfig)` は surface と入力 validation だけを持つ。
有効な logical request は `NOT_IMPLEMENTED`、identity 不正または `AllocationIntent` の require/forbid
衝突は `INVALID_ARGUMENT`。実際の bus 生成は `acquire<CfgT>(cfg)` が担い、backend は初回 acquire の
config 型で固定される。将来 UART/I2S に controller allocation policy を入れる場合も、利用者が覚える
top-level API 名は変えない。

### BusView の実装共有 — `bus::BusViewCore<Traits>`

4 kind の `BusView` は `bus::BusViewCore<Traits>` を共有し、 各 kind の `BusView` は
**`createBusConfig()` のオーバーロードだけを持つ薄い派生**である。 kind 差は `BusTraits` が持つ。

- `Traits::MANAGED_ALLOCATION` — `true` = managed policy (I2C / SPI、 `hardwareInUse()` は backend へ
  委譲)、 `false` = static-backend policy (UART / I2S、 `hardwareInUse()` は 0)
- `createBusConfig()` はピン型 (`Scl`/`Sda`、 `Clk`/`Mosi`/`Miso` 等) が kind 固有のため共有できず、
  各 kind の派生に残る

**なぜ共有するのか**: `BusView` は `IHalBackend` の共通フックを呼び出す唯一の場所である。 kind ごとに
コピーを持つと、 **新しいフックを足しても呼び出し側の一部が更新されず、 エラーも出さずに黙って
何もしない**経路ができる (実例: `completeLogicalRequest` は 4 kind 共通の仕組みだが、 共有前は
I2C の `BusView` からしか呼ばれていなかった)。 実装を 1 本にすれば呼び忘れが起こらない。

**フック呼び出しの契約**: `acquire(const LogicalBusConfig&)` は identity 導出の**前に**
`IHalBackend::completeLogicalRequest()` を必ず呼ぶ。 補完フックを持たない kind では既定の no-op が
返るだけで、 呼び出し自体は省略しない。

**なぜ typed acquire は非 virtual テンプレートなのか**: `acquire<CfgT>(cfg)` は具象 config 型を
必要とし (variant backend の選択に使う)、 具象型は virtual 境界を越えられない。 この経路は本質的に
ローカルであり、 remote backend は別機構を提供する。 一方 logical 経路と commit は型消去された
要求を扱うため virtual で、 local / remote の双方が実装できる。

**local / remote 透過はテンプレート化と独立**: `BusViewCore` は `IHalBackend*` を保持する。
backend の差し替え可能性はこのポインタが担保しており、 テンプレートかどうかとは関係しない。

### `bus::BusGroup` (slot 公開テーブル) — utility

`bus::BusGroup<IBus>` は slot に bus ポインタを `addBus` / `getBus` で公開する非所有テーブル。 スロット公開が
要る上位層向けの汎用 utility。 通常のバス入手は `M5_Hal.<KIND>.acquire` を使う。

## 遅延バインド (unbound 構築 + typed bind)

Accessor は **bus なしで構築して後から束ねられる** — 「グローバルにドライバ
object を置き、`setup()` で `begin(bus)`」という M5UU / Adafruit 系の標準形を
直接書くための形:

```cpp
m5::hal::v2::i2c::MasterAccessor dev{acc_cfg};   // unbound (config のみ)

void setup()
{
    ...
    dev.bind(i2c_bus);   // kind-typed: 非 I2C bus はコンパイルエラー
}
```

- `bind()` は **kind typed** (`i2c::MasterAccessor::bind(i2c::IBus&)`)。typed init
  と同じ思想で、kind 違いの bus を渡す誤りはコンパイル時に落ちる
- アクセスウィンドウが開いている間の `bind()` は `INVALID_STATE`
  (開いている窓は旧 bus の lock を保持しているため)。窓の外での rebind は許される
- **unbound ゲートは窓の入口のみ**: `beginAccess` (UART の Tx/Rx accessor も同名) が
  unbound を検出する (debug = assert、release =
  `INVALID_ARGUMENT`)。sugar はすべて窓の入口を通るため実質全経路が
  カバーされ、`transfer` などのホットパスにはチェックを置かない
- `isBound()` で束縛状態を確認できる。unbound のまま `getBus()` を呼ぶのは
  契約違反 (ゲートなしの null 参照)

## なぜ 2 層構造か

I2C / SPI / UART は **物理バス上に複数の通信相手を載せる** モデル。 同じ Bus に複数 Accessor (異なる通信相手) を作って共有することが自然。 ゆえに物理リソース (Bus) と通信相手 (Accessor) を分離し、 Bus を排他制御の単位、 Accessor を通信パラメータの保持単位とする。

## なぜ「Accessor* nullptr 不可」 か

`lock` / `unlock` の owner identifier に Accessor へのポインタを使う。 これにより:

- 排他保有者が runtime で確認可能 (誰が lock しているか)
- 「他人の lock を奪う」 「他人が unlock する」 等の誤用が identity 比較で検出可能
- nullptr 可にすると「誰が持っているか不明な lock」 が生まれ、 検出機構が成立しない

## transaction 契約 (short transfer / sugar の区間参加)

### short transfer: `tx_len` / `rx_len` は上限、totals が真実

`transfer(desc, Source*, tx_len, Sink*, rx_len)` の長さ引数は**上限**であり、`Source` が
`tx_len` より先に EOF に達した場合・`Sink` が先に閉じた場合は**エラーではなく自然な転送終了**
として扱う。実際に授受した量は `TransferTotals` (次の `transfer` 開始時または `endTransaction`
で確定) にのみ記録され、不足の検出は呼び手が totals と要求長を比較して行う。span 版 sugar は
長さが既知なのでこの規約の影響を受けない (write 系は full-or-fail — 成功時は要求長を返す)。

**Why**: `Source`/`Sink` は `peek`/`reserve` が要求より短く返せる設計であり、「長さは正確な契約」
という前提の方が層の設計と合わない。remote 経由の可変長ストリームとも整合する。

### sugar は開いている transaction に参加する

単発 sugar (`write` / `read` / `readRegister` 等) は内部で transaction を自動開始・終了するが、
**呼び出し時点で明示 transaction が開いていれば depth 合成でそこに参加**し、物理区間
(SPI の CS、I2C の占有、UART/I2S のチャネル排他) は閉じない。§通常路線の
`beginTransaction(); write(); write(); endTransaction();` は全 kind で有効な公式パターン。

計上規則: sugar 自身の戻り値 (`result_t<size_t>`) は**当該呼び出し分**、`endTransaction()` の
`TransferTotals` は**区間内の累計** (sugar 分も含む)。descriptor 由来のバイト (I2C prefix、
SPI command/address/dummy) は数えない。in-flight の非同期転送 (I2C/SPI) がある状態で sugar を
呼んだ場合は、次の transfer 開始時の wait 規約に従い先行転送の完了を待ってから実行される。

## 排他制御の意味論 (常時 mutex)

`Bus` は [runtime.md](runtime.md) の `runtime::Mutex` を **常時内蔵** し、 `lock` は実際の
待ち合わせを行う:

- `lock(owner, timeout_ms)` は保有者がいる間 `timeout_ms` まで **待つ**。 取得できなければ
  **`TIMEOUT_ERROR`** (`timeout_ms == 0` は即時 try-lock、 `types::TIMEOUT_FOREVER` = 既定 =
  取得まで無限に待つ)。 lock timeout は **呼び出しコンテキストの属性**であり config には
  置かない: 明示の待ち時間を渡すのが本筋で、 省略 (無限待ち) は「タイムアウト後の
  処理が面倒な場面」向けのシュガー。 sugar (readRegister 等) の内部 lock も無限待ち —
  単発呼び出しと `ScopedAccess` で挙動が揃う
- **非再帰**: 同一タスクの再 lock (同一 owner、 または同一バス上の別 Accessor) も timeout
  まで待って `TIMEOUT_ERROR` — 無限待ち (既定) では **デッドロック** になり task watchdog が
  検出する (fail-loud)。 入れ子は `Accessor::beginAccess` の depth counter が吸収する
  ので、 `Bus::lock` 自体はアクセスウィンドウあたり高々 1 回しか呼ばれない
- **task-context only**: ISR から呼ばない (runtime kind の契約に従う)
- **timeout 粒度**: runtime variant 依存 (FreeRTOS 環境では tick = 既定 10 ms)
- `_lock_owner` (owner ポインタ) の更新は lock / unlock とも mutex 保持下で行う。 owner 不一致の
  `unlock` は mutex に触れず `INVALID_ARGUMENT`
- **ロックの所有は facade が握る**: accessor が競合するロックは `Bus` facade が所有し、 facade が
  選んだ backend 自身の mutex は休眠する (二重ロックにしない)。 これにより、 将来ロック下で backend を
  差し替えても (hot-swap) accessor を再束縛せずに済む — accessor は facade のロックだけを見ており、
  backend の差し替えは facade の内側で完結するため、 hot-swap が安全に成立する構造になっている

> **互換性注記**: 以前のリリースの `lock` は owner ポインタ比較のみの即時判定で、 競合は
> `BUSY` を返し timeout は予約引数だった。 mutex 実体化と同時に競合エラーは `TIMEOUT_ERROR`
> へ変更された (**BREAKING**)。

## 動詞規約

詳細は [../style/coding_style.md](../style/coding_style.md) §動詞規約 を参照。 本 doc 固有の `beginAccess` (排他期間) と `beginTransaction` (kind 固有 transaction) の区別は §Accessor の責務 を参照。

## RAII 型

| 型 | 対象 | コンストラクタ引数 |
|---|---|---|
| `m5::hal::v2::bus::ScopedAccess` | Accessor の `beginAccess` / `endAccess` | `Accessor&` (+ timeout_ms) |
| `m5::hal::v2::bus::ScopedLock` | Bus の `lock` / `unlock` | `Bus&`, `Accessor*` (+ timeout_ms) |

両者とも:
- ctor 内で `beginAccess` / `lock` を試行
- 成否は `ok()` (成功 = true) / 失敗詳細は `has_error()` / `error()` で確認 (`ok() == !has_error()`)。極性曖昧さ回避のため `operator bool` は意図的に持たない
- dtor は失敗時には `unlock` しない (誤って他者の lock を解かないため)

SPI には CS assert/deassert 区間用の `spi::ScopedTransaction` がある
(`beginTransaction` / `endTransaction` を同じ polarity 規約で包む。
[spi.md](spi.md))。

### guarded (解放エラーも観測する厳格イディオム)

RAII の destructor は解放 (`endAccess` / `endTransaction`) の失敗を報告
できない。解放エラーまで観測したい厳格な用途 (bring-up コード等) には
`bus::guarded(begin, body, end)` を使う:

```cpp
auto r = m5::hal::v2::bus::guarded(
    [&] { return dev.beginTransaction(); },
    [&] { return dev.write(init_seq); },
    [&] { return dev.endTransaction(); });
```

policy (全 accessor sugar が内部で従うものと同一):
- `begin` 失敗は即 return (解放するものがない)
- `body` のエラーは `end` のエラーより優先
- **body 成功 + end 失敗は end のエラーを返す** — depth counter の破損を
  黙殺しない。`end` は `begin` が成功した限り必ず 1 回呼ばれる

## クラス階層

```cpp
namespace m5::hal::v2::bus {
    struct IBusConfig { /* 共通基底 (空マーカ) */ };
    struct IAccessConfig { /* 共通基底 (空マーカ) */ };
    struct ITransferDesc { /* 共通基底 (空マーカ、 詳細は design/transfer_desc.md) */ };

    class IBus {
        virtual error_t init(const IBusConfig& cfg)  = 0;
        virtual void    release()                    = 0;
        virtual error_t lock(IAccessor* owner, uint32_t timeout_ms = types::TIMEOUT_FOREVER);  // mutex 待ち合わせ、 競合 = TIMEOUT_ERROR
        virtual error_t unlock(IAccessor* owner);
        // attach / transfer は kind 別派生 (i2c::IBus / spi::IBus / ...) で定義
        runtime::Mutex _mutex;   // 常時内蔵 (§排他制御の意味論)
    };

    class IAccessor {
        IBus& _bus;
        size_t _access_depth = 0;
        error_t beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);  // 0→1 のみ bus.lock
        error_t endAccess();                            // 1→0 のみ bus.unlock
        bool inAccess() const;
    };
}

namespace m5::hal::v2::i2c {
    struct IBusConfig          : public bus::IBusConfig    { /* pin_scl, pin_sda */ };
    struct MasterAccessConfig : public bus::IAccessConfig {
        /* freq, wire_timeout_ms, i2c_addr, address_is_10bit, register_address_bytes, use_restart */
    };
    struct TransferDesc          : public bus::ITransferDesc { /* inline prefix buffer, 詳細は design/transfer_desc.md */ };

    class IBus : public bus::IBus {
        virtual error_t attach(/* TwoWire&, i2c_master_bus_handle_t, etc */) = 0;
        virtual result_t<size_t> transfer(
            bus::IAccessor* owner,
            const MasterAccessConfig& cfg,
            const TransferDesc& desc,
            data::Source* tx,
            data::Sink*   rx) = 0;
    };

    class MasterAccessor : public bus::IAccessor {
        MasterAccessConfig _access_config;
        // ctor は IBus& 受け (コンパイル時 kind verify)
        MasterAccessor(IBus& bus, const MasterAccessConfig& cfg);

        // sugar (全て内部で beginAccess → transfer → endAccess)
        expected<size_t, error_t> transfer(const TransferDesc& desc, data::ConstDataSpan tx, data::DataSpan rx);
        expected<size_t, error_t> write(data::ConstDataSpan tx);
        expected<size_t, error_t> read(data::DataSpan rx);

        // register sugar: アドレス幅は register_address_bytes (config) が単一源。reg は値 (型は幅に無関係)。
        expected<size_t, error_t> writeRegister(int reg, data::ConstDataSpan value);
        expected<size_t, error_t> writeRegister(int reg, uint8_t value);
        expected<size_t, error_t> writeRegister(int reg, const uint8_t* tx, size_t len);
        expected<size_t, error_t> readRegister(int reg, data::DataSpan dst);
        expected<size_t, error_t> readRegister(int reg, uint8_t* dst, size_t len);
        expected<uint8_t, error_t> readRegister(int reg);

        // probe (0-byte write、 device 存在確認、 wire 上で address+W 送出 + ACK チェック)
        expected<void, error_t> probe();
    };
}
```

I2C kind 固有の詳細 (BusConfig / AccessConfig のフィールド意味、 sugar の挙動、 probe path の wire 規約) は [i2c.md](i2c.md)、 TransferDesc 詳細は [transfer_desc.md](transfer_desc.md)、 Source/Sink 詳細は [data_io.md](data_io.md)。

## M5UnitUnified との関係

M5UnitUnified などの上位ライブラリから直接利用できるよう、 v2 では sugar と低レイヤ API の責務を明確に分ける。 詳細は [../style/migration.md](../style/migration.md)。

register sugar (`writeRegister` / `readRegister`) は register 番号を値で受け (`readRegister(0x00)` も型付き定数も可)、 **アドレス幅は `MasterAccessConfig::register_address_bytes` のみ** (`0` / `1` = 1 byte、 `2` = 2 byte) で決まる — 引数の型は幅に影響しない。 M5UnitComponent 系の typed-constant 利用形態は、 デバイス設定時に `register_address_bytes` を一度合わせることで橋渡しする (将来の M5UU アダプタは `sizeof(Reg)` をここへ写すだけ)。
