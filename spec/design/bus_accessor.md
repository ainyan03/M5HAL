# design/bus_accessor — Bus と Accessor の責務分離

> **読者**: 実装者・レビュー向け（設計仕様）。

通信バスへのアクセスは **Bus と Accessor の 2 層構造** で表現する
(I2C / SPI / UART / I2S / PDM 共通)。利用者の開始・終了APIは
`beginAccess` / `endAccess` に統一し、Bus backendの開始・終了hookは
`beginOperationBackend` / `endOperationBackend` に統一する。Busの同名
`beginOperation` / `endOperation`は、backendへ到達する前にAccessor-owned Contextを検査する
non-virtual入口である。

- **Bus** = 配線・native handle・remote pathなど、`ResourceKey`で識別する1つの通信リソースを表す。
  backend controllerはそのリソースを駆動する手段であり、identityそのものではない。Busはリソース単位の
  排他制御を持つ
- **Accessor** = 1 つの通信相手 (例: アドレス 0x76 の BME280) を表す。 通信パラメータ (速度・タイムアウト等) を保持

利用者は Accessor 経由で通信する (Bus は排他制御の単位として裏で動く)。 「なぜこの形なのか」の設計根拠は後半 (§なぜ 2 層構造か 以降) にまとめる。

## Bus の責務 (最小化)

- 物理バスの初期化と終了 (`init` / `close`)
- 対応providerにおけるnative resource ownership policyの適用
- Accessor lifecycleからだけ使う内部排他制御
- Access開始・終了時のchecked入口 (`beginOperation` / `endOperation`) とbackend hook
  (`beginOperationBackend` / `endOperationBackend`)
- atomic な通信動作のchecked入口とbackend hook (`transfer` / `transferBackend`)
- 取得済みinstanceの不変能力snapshot (`capabilities()`。詳細は[bus_capabilities.md](bus_capabilities.md))

Bus は **Accessor を所有・生成しない**。利用者が Bus と Accessor を別個に保持する。

## Accessor の責務

- 1 つの通信相手の表現 (外部構築・copy・move不能な`OperationContext` capabilityが
  `AccessConfig`実体と小さなruntime状態を常設保持)
- Access期間の宣言 (`beginAccess` / `endAccess`)
- 利用者向け sugar (`write` / `read` / `transfer` / `writeRegister` / `readRegister` / `probe`)
- I/Oごとの同期結果と `TransferStatus` の更新

Accessはlockだけではない。開始時にlockを取得してAccess全体に必要な設定・backend状態を確立し、終了時に最後の
I/O完了、backend終了、lock解放までを行う。最外Access一回につきBus hookを各一回だけ呼ぶ。

```text
beginAccess(timeout)
  context runtime初期化 → lock取得 → beginOperation(context) [検査・slot登録] → beginOperationBackend(context)
  transfer / read / write ... （各I/Oが自身の結果を返す）
endAccess(timeout)
  outstanding I/O完了 → endOperation(context) → endOperationBackend(context) → Context失効 → lock解放
```

Accessは **non-nestable** であり、同じAccessorへの二重`beginAccess()`は`INVALID_STATE`。
sugarはAccessがinactiveなら一時Accessを開閉し、activeなら既存Accessをborrowするため、内部から
二重beginしない。`endAccess()`はlifecycle終了だけを返し、Access-wide totalsを返さない。

kindごとのwire効果は同じhook名の実装差として表す。

| kind / role | Access中のbackend保証 |
|---|---|
| SPI master | `beginOperation`でCSをassertし、`endOperation`でdeassertする。複数transferで一つのCS frameを構成できる |
| I2C master | busを占有する。一つのAccess内に複数の独立したSTART〜STOP transferを置ける。target address、wire timeout、frequencyは物理transferの入力として各transfer直前に検証・cached適用する |
| UART / I2S / PDM | 方向別channelを占有し、必要な設定・flush/drainを開始終了hookで行う |
| I2C / SPI slave | `beginAccess`で外部masterからの受付を開始し、`endAccess`で受付停止・進行中frameの有界cleanupを行う |

I2C slaveの旧`SlaveStreamAccessor::openWireFrame` / `closeWireFrame`は、外部masterが開始した
START〜STOP frameをclaim/releaseするprotocol seamであり、Access lifecycleではない。新しいqueue駆動
`SlaveAccessor`は`beginAccess` / `endAccess`を使う。

**共有の単位 (契約)**: `Bus` はスレッド間で共有してよいが、**`Accessor` は共有してはならない**
(スレッドごとにAccessorを作って同じBusへbindする)。UART/I2SのTXとRXは独立channel lockなので、
`TxAccessor` / `RxAccessor`は同時にAccessを持てる。

## OperationContext capabilityとchecked facade

`OperationContext<Config>`はAccessor constructorだけが作れる常設memberであり、利用者やcustom backendは
新規構築、copy、moveできない。各Busは同時実行可能なlock/channelごとに固定長`OperationSlot`を持ち、
`beginOperation(context)`でContext address、構築元Accessor、Bus address、generationを登録する。
`transfer/read/write`等のnon-virtual入口は、Contextがそのslotでactiveか、Bus・Accessor・generationが一致するかを
pointer/integer比較だけで検査してからprotected virtual `*Backend` hookへdispatchする。

未開始、終了済み、別Accessor、別Bus、旧generationは`INVALID_STATE`で拒否する。backend begin失敗時は登録を
rollbackし、backend end失敗またはgeneration不一致のcleanup時も、そのBusに登録済みのContextは必ず失効させてから
Accessor lockを解放する。従って失敗したAccessが偽active slotを残して後続Accessを塞がない。hot pathへheap allocation、
registry探索、`shared_ptr` copy/refcount操作を追加しない。

UART/I2SはTXとRXに別slotを持つ。full-duplex I2Sの入口はTX/RX二つのContextを検査し、一つのContextやslotへ
合成しない。slave worker/ISRがContext pointerを保持できるのはAccess期間内だけであり、`endOperationBackend`は
producer/workerをquiesceしてからchecked入口へ戻る。

## 利用形

```cpp
m5::hal::v2::i2c::MasterAccessor accessor{i2c_bus, acc_cfg};

// 単発: sugar 内部で beginAccess → transfer → endAccess
accessor.writeRegister(REG_CTRL, VAL_MODE);

// 連続 atomic: ScopedAccess で一つのAccessを保持
{
    m5::hal::v2::bus::ScopedAccess access{accessor, 100};
    if (access.has_error()) { /* lock 競合 (timeout) */ return; }
    accessor.writeRegister(REG_CTRL, VAL_MODE);
    accessor.readRegister(REG_DATA, dst_span);
}  // ここで endAccess
```

- 単発sugarは一時Accessを開閉する
- 明示Access中のsugarはそのAccessをborrowする
- `ScopedAccess::finish(timeout)`を使うとdestructorでは報告できない終了errorも観測できる

SPIではAccessそのものがCS frameである。

```cpp
m5::hal::v2::spi::MasterAccessor dev{spi_bus, spi_cfg};

// 単発: CS はこの write の前後だけ assert される
dev.write(tx_span);

// 複数 transfer で CS を維持する場合
dev.beginAccess();
dev.write(command_span);
dev.write(data_span);
dev.endAccess();
```

raw lock APIと`ScopedLock`は公開しない。Busの`transfer` / `waitTransfer`はbackend実装と
Accessor coreのseamであり、通常利用者はtyped Accessor経由で呼ぶ。高度な利用でも明示
`beginAccess`を先に開き、Accessorのcore `transfer`を使う。

## Bus の保持

bus の入手・共有は **全 kind 共通の所有レジストリモデル**。I2C / SPI はさらに intent 駆動の
HW 割当を持つ。UART / I2S / PDM は同じ `commitBuses()` / `hardwareInUse()` / logical acquire
surface を持つが、現時点では static-backend policy で運用する。

### 全 kind = domain-local所有レジストリ — `hal.<KIND>`

I2C / SPI / UART / I2S / PDM はすべて、**`Hal`の`ResourceDomain`が所有する共有
`bus::BusRegistry`にbusをinternする**。`M5_Hal`はdefault domainを使う同じ仕組みの便利入口である。
利用者は物理配線 (ピン) で `acquire(cfg)` し、所有権を持つ `shared_ptr` を取得する:

```cpp
auto i2c  = M5_Hal.I2C.acquire(m5hal::i2c::BusConfig{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}});
auto spi  = M5_Hal.SPI.acquire(spi_cfg);    // identity = CLK / MOSI / MISO
auto uart = M5_Hal.UART.acquire(m5hal::uart::BusConfig{m5hal::uart::Tx{17}, m5hal::uart::Rx{16}});
auto i2s  = M5_Hal.I2S.acquire(i2s_cfg);    // identity = BCLK / WS / DOUT / DIN
auto pdm  = M5_Hal.PDM.acquire(pdm_cfg);     // identity = CLK / DIN
if (!i2c) { /* backend init 失敗 / OUT_OF_RESOURCE / NOT_CONNECTED (init()/connect() 前の自作 Hal — M5_Hal では発生しない) */ }
m5hal::i2c::MasterAccessor dev{i2c.value(), acc_cfg};  // shared_ptr 直渡し = accessor が bus を co-own
```

- **identity = 全対応targetで32 B固定のtagged `ResourceKey`の完全一致**。keyは`Pins` / `Native` / `Path` / `Remote`を
  表現し、domain IDはpayloadへ入れずregistry namespaceで分離する。portable
  `acquire(config)`は`Pins` tagへ射影する (kind 別: I2C={SCL,SDA} / SPI={CLK,MOSI,MISO} / UART={TX,RX} /
  I2S={BCLK,WS,DOUT,DIN} / PDM={CLK,DIN})。**同一配線 = 単一インスタンス** — ボード層とユーザコードが同じピンを acquire
  すると**同じ bus (同じ lock) を共有**する (別々に作ると別 mutex になりバスが壊れる、を防ぐ correctness
  要件)。どの backend / Wire / port が駆動するかは identity でなく「配線の駆動手段」で、**最初の acquire が
  勝つ**。ただし、同一 identity の既存 bus に対して identity 外の bus-level config
  (SPI `pin_dc` / quad data pins、UART RTS/CTS・buffer size、I2S MCLK・buffer size・role 等) が
  食い違う acquire は `INVALID_STATE` を返す。これは first-config-wins によるサイレントな
  設定取り違えを避ける診断であり、既存 bus の再構成はしない。通常取得は共通`BusConfig`による
  `acquire(cfg)`であり、config型でproviderを選ばない。対応providerは同じ名前の
  `acquire(cfg, native::borrowed(resource))` / `acquire(cfg, native::managed(resource))`を追加できる。
  `Native` / `Path` tokenはそのnative identityを表す。未対応policyを全providerが受理するとは限らず、
  bus取得用の`attach` / `open`は公開しない。
  pin の `-1` sentinel も他の値と同様に identity の一部として比較し、BusView は未指定roleを理由に
  事前拒否しない。その構成で動作可能かは選択backendの `init()` が判定し、失敗したbusはregistryへ
  登録されない。これにより、backend既定配線を使う全pin未指定configや、特定roleを物理出力しない
  backendを共通identity機構の外へ追い出さない。
  remote keyはconnection ownerが発行するnonzero 64-bit session generationとexact targetを含み、
  reconnect後の同じtargetを旧sessionと同一視しない。remoteはportable `BusConfig`だけをwireへ送り、
  native ownership policyは送信前に`UNSUPPORTED`で拒否する。
  *(リモートは実装済み: `Hal::initUart` / `initTcp` で接続した `Hal` インスタンスの
  `hal.<KIND>.acquire` が同型でリモートプロキシを返す — [remote.md](remote.md) §Hal facade。
  接続束縛は排他 = 1 `Hal` は 1 デバイスの窓。)*
- **戻り値 = `result_t<shared_ptr<kind::IBus>>`** — 成功なら所有権を持つshared_ptr、失敗なら
  原因を表すerrorを返す。
- **寿命 = registry がweak intern、Busがdomain stateをco-own**。返した`shared_ptr`が最後まで保持されている間
  busは生き、`Hal` / `ResourceDomain` facadeが先に破棄されても注入済みGPIO / Services / Memoryとregistryは
  生存する。**最後の保持者が手放すと自然終了** (registry `Closing`予約 → bus dtor → backend close →
  entry commit) される。`Live` entryのweak ownerが失効してからdestructorが`Closing`を予約するまでの隙間も
  同一keyのacquireは`BUSY`とし、二つのBusを作らない。remoteのように外部資源の
  解放確認が必要な bus は lifecycle tombstone を併用し、確認できた場合だけ identity / 外部 ID を
  再利用する。確認不能なら tombstone を保持する (後述および [remote.md](remote.md) §動的バス生成)。
  **accessor を
  `shared_ptr` から直接構築すると accessor が bus を co-own する** (`MasterAccessor dev{i2c.value(), cfg}`) ので、
  accessor が生きている間は bus も生き、利用者が別途 `shared_ptr` を保持し続けなくてよい (acquire の一時値から
  直接渡しても安全)。`IBus&` を渡す構築 (`dev{*i2c.value(), cfg}`) は自前所有のエスケープ用で、その場合は
  従来どおり bus を accessor より長く生かすのが利用者の契約。ボード層 (M5Unified) が ref を持ち続ければ内蔵
  バスは生存する。
  専用`BusHandle` wrapperは、現行`shared_ptr`より安全性、型サイズ、allocation、close error回収のいずれかで
  測定可能な改善を示していないため導入しない。Accessorの転送hot pathは保持済みBusを参照し、`shared_ptr`
  copyやrefcount操作を行わない。具体的な改善を実測できた場合だけ所有型を再検討する。
- **明示 close = exact-instance の consuming close**。`hal.<KIND>.close(bus)` は non-const
  `shared_ptr&` を取る (各 kind の宣言形は
  `result_t<void> close(std::shared_ptr<IBus>& bus)`)。registry 内の実体がその pointer と一致し、
  かつ caller が唯一の
  strong owner のときだけ実行する。Accessor や別の `shared_ptr` が co-own 中なら `BUSY`、
  backend未束ねなら`NOT_CONNECTED`、null handle・同一 identity の別実体・他 registry の busなら
  `INVALID_ARGUMENT`。成功時は引数の
  `shared_ptr` を reset する。close 中の同一 identity は acquire できず `BUSY` とする。
  close 判定後に外部 `weak_ptr` が旧実体を復活させても、共有 lifecycle gate がRPC完了前に
  旧実体を閉じるため、その操作は `CLOSED` となる。外部資源の自然終了を確認できない場合は
  registry slotを `Quarantined` tombstoneとして保持し、同一identityの再取得を`BUSY`にする。
  これにより、操作可能な旧 bus と新 bus が別 mutex で同じ配線を駆動する状態を作らない。

  backend teardownはprotected `closeBackend()`へ閉じ、失敗を`NoMutation` / `PartialOrUnknown`に分類する。
  前者は元の状態へ戻し、後者はregistry entryとBusを`Quarantined`にして、同じsole-owner handleからの
  close再試行だけを許す。

  `IBus`は操作対象であってregistry所有権を持たないため、public `close()`を持たない。取得handleを
  `bus->close()`してregistryを迂回する形はcompile errorになる。終了はhandleを所有・消費できる
  `hal.<KIND>.close(bus)`だけを使う。registryは全格納Busへbound状態も記録し、concrete型へdowncastして
  direct-owner用wrapperへ到達した場合も`INVALID_STATE`で拒否する。

明示 close の前に Accessor と alias を破棄する。成功時は `bus` 自体が空になるため、caller が
追加の `reset()` を行う必要はない。失敗時は所有権を caller へ戻すので、原因を除いて同じ handle で
再試行できる。

```cpp
auto bus = hal.I2C.acquire(cfg);
if (!bus) { /* handle error */ }
{
    m5hal::i2c::MasterAccessor device{bus.value(), access_cfg};
    // use device
}  // Accessor の co-own を先に終える
auto closed = hal.I2C.close(bus.value());
// success: bus.value() == nullptr; BUSY: alias/Accessor/close がまだ競合中
```

外部 identity を持つ bus は proxy と registry で同じ `BusLifecycle` を共有する。状態遷移は次の
一方向を正本とする。

```text
Open --close開始--> Releasing --成功--> Closed
  ^                      |
  +--NoMutation rollback-+

Releasing --PartialOrUnknown--> Quarantined

Open --自然終了の確認不能--> Quarantined
```

operation は `Open` の間だけ開始でき、開始済み operation と close は同じ gate で直列化される。
`Closed` / `Quarantined` から操作は再開せず `CLOSED`。`Releasing` 中は registry の同一 identity
acquire と別 close を `BUSY` にする。`Quarantined` は接続/backend 全体の cleanup まで identity を
占有し、確認していない外部 ID の ABA 再利用を防ぐ。

直接構築した`Bus`はregistry identityを消費しないため、`close()`成功後に同じobjectへ`init()`して再利用できる。
master/slave切替等の再構成はこの形を使う。registryから`acquire()`したBusのcloseはconsuming operationであり、
成功後のhandleは空になるため再初期化しない。

- **総数キャップ** (全 kind 合計の固定上限 `BusRegistry::kCapacity`) があり、満杯の acquire は `OUT_OF_RESOURCE`。
- registry slotは`Empty / Constructing / Live / Closing / Quarantined`で管理する。missはgeneration付きslotを
  `Constructing`予約してからfactoryをregistry mutex外で実行し、同一keyの並行acquireは`BUSY`。
  factory失敗・例外はreservationをrollbackし、publish / closeはslot+generation+exact instanceを照合する。
- local factoryとBusは`LocalResourceContext`を生成時に一度だけbindする。転送hot pathではdomain探索、
  `shared_ptr` copy、registry/key比較を行わない。
- **直接構築も可**: `<kind>::Bus bus; bus.init(cfg);` も引き続き可能 (acquire はインターン共有が要るときの
  導線)。無印 `<kind>::Bus` はbuildで選ばれたproviderへのfacadeで、portable `BusConfig`を受ける。
  特定providerの固有初期化が必要な場合だけ`Bus_<variant>::init(...)`をadvanced escape hatchとして使う
  ([variants.md](variants.md) §facade kind / 各 kind の spec)。

### I2C / SPI: intent 駆動の HW 割当

I2C / SPI は上記に加え `acquire(LogicalBusConfig{pins, intent})` + 明示 `commitBuses()` で
HW コントローラ割当を遅延解決し、ロック下で backend を hot-swap (reassign) できる。
portable `acquire(cfg)` はbuildで選ばれたproviderを使う。native resourceを明示する取得は、対応providerの
`acquire(cfg, native::borrowed(...))` / `native::managed(...)`で表し、intent管理対象にはならない。
詳細・`AllocationIntent` / query API は [i2c.md](i2c.md) §intent 駆動の HW 割当、および
[spi.md](spi.md) §Bus の入手。

### UART / I2S / PDM: static-backend policy

UART / I2S / PDM も `BusView` の形は I2C / SPI と揃える (実装は共有 — §BusView の実装共有)。つまり
`commitBuses()` は呼べるが no-op、`hardwareInUse()` は 0 (`BusTraits::MANAGED_ALLOCATION = false`)、
`acquire(LogicalBusConfig)` は surface とintent validationだけを持つ。
logical request はpin値によらずbackend未提供なら`NOT_IMPLEMENTED`、`AllocationIntent` の
require/forbid衝突は `INVALID_ARGUMENT`。実際の bus 生成はportable `acquire(cfg)` が担い、providerは
buildのwinner bindingで固定される。UART/I2S/PDMへcontroller allocation policyを追加する場合も、利用者が覚える
top-level API名は変えない。

### BusView の実装共有 — `bus::BusViewCore<Traits>`

5 kind の `BusView` は `bus::BusViewCore<Traits>` を共有し、 各 kind の `BusView` は
**`createBusConfig()` のオーバーロードだけを持つ薄い派生**である。 kind 差は `BusTraits` が持つ。

I2C / SPIのmanaged `Bus`は`bus::ManagedBusFacade<Traits>`で
`transfer` / `waitTransfer` / `transferBusy`のbackend転送を共有する。SPIだけに存在する
CS制御も共通名`beginOperation` / `endOperation`のSPI実装として`spi::Bus`に残す。UART / I2Sの
`transfer`とPDMのRX streamはconfig型と同期完了の戻り値がmaster kindと異なるため、各static facadeが転送する。
allocation resolverのkind seamは能力・適格性とatomicなbackend commitだけを公開し、raw backend
factoryはlocal adapter内部に閉じる。これによりresolverやfake policyがbackend構築helperへ依存しない。

- `Traits::MANAGED_ALLOCATION` — `true` = managed policy (I2C / SPI、 `hardwareInUse()` は backend へ
  委譲)、 `false` = static-backend policy (UART / I2S / PDM、 `hardwareInUse()` は 0)
- `createBusConfig()` はピン型 (`Scl`/`Sda`、 `Clk`/`Mosi`/`Miso` 等) が kind 固有のため共有できず、
  各 kind の派生に残る

**なぜ共有するのか**: `BusView` は `IHalBackend` の共通フックを呼び出す唯一の場所である。kindごとに
コピーを持つと、新しいフックを追加した際に一部の呼び出し側だけが更新されず、エラーを出さずに
処理を省略する経路が生じる。実装を1本にすることで、全kindが同じhook列を必ず通る。

**フック呼び出しの契約**: `acquire(const LogicalBusConfig&)` は identity 導出の**前に**
`IHalBackend::completeLogicalRequest()` を必ず呼ぶ。 補完フックを持たない kind では既定の no-op が
返るだけで、 呼び出し自体は省略しない。

**なぜportable acquireを共通境界に置くのか**: `acquire(const BusConfig&)`ならlocal / remoteが同じ
型消去境界を実装でき、config型をprovider selectorとして利用者へ露出しない。native ownership policyは
resource型と所有責務をコンパイル時に区別する薄いoverloadであり、対応providerの`NativeProvider`へだけ
委譲する。同じ`acquire` prefixへ揃えることで寿命別method familyの増殖やopaque `void*` configを避ける。
logical経路とcommitも型消去された要求を扱うため、local / remoteの双方が実装できる。

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

## なぜ内部lock ownerをAccessor identityとするか

protectedな`acquireAccessLock` / `releaseAccessLock`は`IAccessor&`を受け、排他保有者を
Accessor identityで追跡する。これにより:

- 排他保有者がruntimeで確認可能
- 別Accessorによる取得・解放の誤用をidentity比較で検出可能
- non-null参照なので「保有者不明」の状態を作れない

## transfer とAccessの契約

### short transfer: `tx_len` / `rx_len` は上限、totals が真実

`transfer(desc, Source*, tx_len, Sink*, rx_len)` の長さ引数は**上限**であり、`Source` が
`tx_len` より先に EOF に達した場合・`Sink` が先に閉じた場合は**エラーではなく自然な転送終了**
として扱う。実際に授受した量は各`transfer`が返す`TransferTotals`と
`getLastTransferStatus()`に記録され、不足の検出は呼び手がtotalsと要求長を比較して行う。span版sugarは
長さが既知なのでこの規約の影響を受けない (write 系は full-or-fail — 成功時は要求長を返す)。

**Why**: `Source`/`Sink` は `peek`/`reserve` が要求より短く返せる設計であり、「長さは正確な契約」
という前提の方が層の設計と合わない。remote 経由の可変長ストリームとも整合する。

### sugar はactive Accessをborrowする

単発sugar (`write` / `read` / `readRegister`等) はinactiveなら一時Accessを開閉する。
**呼出時点で明示Accessがactiveならそれをborrow**し、SPIのCS、I2Cの占有、UART/I2Sのchannelを
閉じない。公式パターンは全kindで
`beginAccess(); write(); write(); endAccess();`。

sugarの戻り値 (`result_t<size_t>`) は当該呼出し分であり、`endAccess()`は累計を返さない。
descriptor由来のbyte (I2C prefix、SPI command/address/dummy) はcaller data totalsに数えない。
in-flight転送がある状態で次のI/Oまたは`endAccess`を呼ぶと、Accessorが完了を待つ。

## 排他制御の意味論 (Accessor内部)

`Bus`は[runtime.md](runtime.md)の`runtime::Mutex`を常時内蔵する。公開raw lock APIは持たず、
Accessorの`beginAccess` / `endAccess`だけがprotected seam
`acquireAccessLock` / `releaseAccessLock`を呼ぶ。

- `beginAccess(timeout_ms)`のlock段は保有者がいる間`timeout_ms`まで待つ。取得できなければ
  **`TIMEOUT_ERROR`** (`timeout_ms == 0` は即時 try-lock、 `types::TIMEOUT_FOREVER` = 既定 =
  取得まで無限に待つ)。 lock timeout は **呼び出しコンテキストの属性**であり config には
  置かない: 明示の待ち時間を渡すのが本筋で、 省略 (無限待ち) は「タイムアウト後の
  処理が面倒な場面」向けのシュガー。 sugar (readRegister 等) の内部 lock も無限待ち —
  単発呼び出しと `ScopedAccess` で挙動が揃う
- **非再帰・非nesting**: 同一Accessorの二重beginはlock前に`INVALID_STATE`。同一Bus上の別Accessorは
  timeoutまで待ち、取得できなければ`TIMEOUT_ERROR`
- **task-context only**: ISR から呼ばない (runtime kind の契約に従う)
- **timeout 粒度**: runtime variant 依存 (FreeRTOS 環境では tick = 既定 10 ms)
- `_lock_owner`の更新は取得・解放ともmutex保持下で行う。owner不一致の内部解放はmutexに触れず
  `INVALID_ARGUMENT`
- **ロックの所有は facade が握る**: accessor が競合するロックは `Bus` facade が所有し、 facade が
  選んだ backend 自身の mutex は休眠する (二重ロックにしない)。 これにより、 将来ロック下で backend を
  差し替えても (hot-swap) accessor を再束縛せずに済む — accessor は facade のロックだけを見ており、
  backend の差し替えは facade の内側で完結するため、 hot-swap が安全に成立する構造になっている

## 動詞規約

詳細は[../style/coding_style.md](../style/coding_style.md) §動詞規約を参照。
公開lifecycleは`beginAccess` / `endAccess`、Bus checked入口は`beginOperation` / `endOperation`、
派生hookは`beginOperationBackend` / `endOperationBackend`、
protocol frameはkind固有語 (`openWireFrame` / `closeWireFrame`) を使う。

## RAII 型

| 型 | 対象 | コンストラクタ引数 |
|---|---|---|
| `m5::hal::v2::bus::ScopedAccess` | Accessor の `beginAccess` / `endAccess` | `Accessor&` (+ timeout_ms) |

`ScopedAccess`は:
- ctor内で`beginAccess`を試行
- 成否は `ok()` (成功 = true) / 失敗詳細は `has_error()` / `error()` で確認 (`ok() == !has_error()`)。極性曖昧さ回避のため `operator bool` は意図的に持たない
- dtorは取得失敗時に`endAccess`しない
- `finish(timeout_ms)`で終了errorを明示的に取得できる

### guarded (解放エラーも観測する厳格イディオム)

RAII の destructor は解放 (`endAccess`) の失敗を報告
できない。解放エラーまで観測したい厳格な用途 (bring-up コード等) には
`bus::guarded(begin, body, end)` を使う:

```cpp
auto r = m5::hal::v2::bus::guarded(
    [&] { return dev.beginAccess(); },
    [&] { return dev.write(init_seq); },
    [&] { return dev.endAccess(); });
```

policy (全 accessor sugar が内部で従うものと同一):
- `begin` 失敗は即 return (解放するものがない)
- `body` のエラーは `end` のエラーより優先
- **body 成功 + end 失敗はendのerrorを返す**。`end`は`begin`成功時に必ず1回呼ばれる

## API 宣言と kind 固有契約

型・関数の完全な宣言は `src/m5_hal/hal/v2/bus/` と各 kind の公開ヘッダを正本とする。
本書はそれらに共通する所有、Access lifecycle、排他、checked facade の意味論だけを定め、
宣言一覧は再掲しない。

backend終了処理だけは共通lifecycleの中核なので、基底hookの形をここにも固定する。

```cpp
protected:
    virtual bus::CloseOutcome closeBackend();
```

kind 固有のconfig、転送、sugar、wire効果は各設計文書を参照する。

- I2C: [i2c.md](i2c.md) / [i2c_slave.md](i2c_slave.md)
- SPI: [spi.md](spi.md)
- UART: [uart.md](uart.md)
- I2S: [i2s.md](i2s.md)
- PDM: [pdm.md](pdm.md)
- TransferDesc: [transfer_desc.md](transfer_desc.md)
- Source / Sink: [data_io.md](data_io.md)
- v0からの移行: [../style/migration.md](../style/migration.md)
