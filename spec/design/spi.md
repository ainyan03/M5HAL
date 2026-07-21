# design/spi — SPI kind 固有設計

> **読者**: 実装者・レビュー向け（設計仕様）。

## kind contract

SPIはI2Cと同じく`Bus` / `Accessor` / `TransferDesc` / `Source` / `Sink`で扱い、
software / hardware backendに同じ呼び出し面を提供する。
SPI slaveのcaller-owned queue契約は[slave_queue.md](slave_queue.md)を参照する。

- `MasterAccessor` の `beginAccess` / `endAccess` が排他・設定・CS assert/deassert・完了待ちを包む。
- 送受信データは `data::MemorySource` / `data::MemorySink` 経由で
  `IBus::transfer(...)` へ渡す。
- command / address / dummy / data phase は 1 回の transfer にまとめ、CS を
  phase 間で解除しない。
- `IBus::transfer(context, ...)` はactive Contextを検査するnon-virtual入口で、基底
  `transferBackend` defaultは`UNSUPPORTED`を返す (concrete variant — Arduino / ESP-IDF / software —
  がprotected hookをoverrideして実体を供給する)。
- `IBus::beginOperation` / `endOperation` はAccessor-owned
  `OperationContext<MasterAccessConfig>&`だけを受けるchecked入口である。hardware / framework variantは
  protected `beginOperationBackend` / `endOperationBackend`でnative設定とCS制御を行う。

software bit-bang SPIは同期transferを提供し、生成domainのservice runner
(default Halでは`M5_Hal.Services`)に登録されてcooperative schedulingで駆動される
(§software SPI variantの実装方針)。ESP-IDF hardware SPIはpolling APIを使い、data phaseは
DMA-capableな二重bounce bufferとworker taskでchunkの準備・回収を進める。
interrupt-driven transferは提供しない。

## Bus の入手

共通機構は [bus_accessor.md](bus_accessor.md) §Bus の保持 を参照。本 kind 固有の差分のみ以下に示す。

- **portable acquireのidentity projection = `Pins` tagのCLK / MOSI / MISO**。DC / CS / quadピン
  (`d2..d7`)はidentity外で**first-config-wins** ([bus_accessor.md](bus_accessor.md) §Busの保持)。
- SPI は I2C と同じ **managed BusView policy** を使う。portable `acquire(cfg)`はbuildで選ばれた
  providerを使い、`acquire(LogicalBusConfig{pins, intent})` は配線と `AllocationIntent` だけを
  記録し、`commitBuses()` が HW controller を優先度順に割り当てる。commit の優先度、snapshot 駆動の
  atomicity 方針、query API は [i2c.md](i2c.md) §intent 駆動の HW 割当と同じ。SPI 固有の identity は
  CLK / MOSI / MISO で、MISO-less 構成は MISO を未接続 sentinel として同一ポリシーに載る。
- facade alias 機構は [variants.md](variants.md) §facade kind を参照。

## BusConfig の役割

共通`BusConfig`はpinなどHAL共通の情報だけを持つ。通常取得は
`acquire(cfg)`であり、Framework依存のnative resourceは対応providerのownership policyで指定する。

- Arduino variant: caller-owned `SPIClass`は`acquire(cfg, native::borrowed(SPIClass&))`で
  借用する。managed native取得は提供しない。非ESP Arduino coreのportable `SPIClass`は任意pinを設定する
  共通 API を持たないため、portable `BusConfig`でCLK / MOSI / MISOが一つでも
  指定されていれば、配線を無視せず `UNSUPPORTED` を返す。三 pin を未指定にした場合だけ、
  選択した `SPIClass` instance の board 既定配線で `begin()` する。
- ESP-IDF master variant: 現行の公開native acquisitionは提供しない。provider固有の
  `spi_host_device_t`操作が必要なら`Bus_espidf`のdirect APIをadvanced escape hatchとして使う。
- software variant: native ownership policyを提供せず、portable configだけを受ける。

これにより、Arduino / ESP-IDF / softwareのどれを選んでも通常利用者は同じportable configだけを読む。
任意 pin でsoftware fallbackを許す場合は
`acquire(LogicalBusConfig{Clk{18}, Mosi{23}, Miso{19}, automatic()})` のようなlogical acquireを
使い、bus確定時にbackendを選ぶ。`preferHardware()`もfallbackを許し、`software()`はsoftware固定、
`requireHardware()`はfallback禁止である。
transfer開始後の`UNSUPPORTED`を契機とした再実行は、CS区間・Source消費・wire副作用を
重複させ得るため行わない。

backendの機能要件はbus確定前の`AllocationIntent`へ宣言する。MISO-less half-duplex RXには
`requireMosiSharedRx(intent)`を使う。このhelperはSPI固有のrequired capabilityを既存intentへ合成し、
identityには影響しない。`requireMosiSharedRx(preferHardware())`では対応hardwareを優先し、非対応なら
この機能を提供するsoftware SPIへfallbackする。`requireMosiSharedRx(requireHardware())`ではfallbackを
禁止し、対応controllerがなければ`commitBuses()`がtransfer開始前に`OUT_OF_RESOURCE`を返す。同一identityを
この要件付きで再acquireした場合も、次のcommitで非対応hardwareからsoftwareへdemoteしてから使用する。
accessorの`spi_data_mode`をcommit後に見てbackendを切り替える方式は採らない。
remoteの`BusCreate` wire形式はintent/capabilityを搬送しないため、この要件を指定したremote logical acquireは
黙って無視せず`UNSUPPORTED`を返す。peer capability negotiationを追加するまでremote保証には含めない。

このcapabilityはMOSIを入力へ切り替えるRX能力だけを表す。CLK未配線でMOSIだけを波形出力に使う能力は
別要件であり、allocation capabilityとして宣言・広告しない。software SPIはCLKを必要とし、
ESP-IDFでCLKを`-1`にするtransferもportable contractに含めない。

コア配線ピンは **タグ型 ctor** で与えられる: `BusConfig{spi::Clk{18}, spi::Mosi{23}, spi::Miso{19}}`。
MISOを省略したbusは既定の`FullDuplex`ではwrite-onlyで、RX要求を`INVALID_STATE`としてwire操作前に拒否する。
ただし`spi_data_mode`を`HalfDuplex` / `HalfDuplexWithDcPin` / `HalfDuplexWithDcBit`のいずれかへ
明示設定すると、softwareとESP-IDF variantはMOSIを送受信兼用の単一データ線として扱い、TX phase後に
入力へ切り替えてRX phaseを実行する。ESP-IDFでは`SPI_DEVICE_3WIRE`に対応する。Arduino `SPIClass`
variantはportableな方向切替APIを持たないため、このMISO-less half-duplex RXを`UNSUPPORTED`で拒否する。
i2c (`Scl`/`Sda`) / uart (`Tx`/`Rx`) と同じく順序取り違えがコンパイルエラーになる。
QSPI データ線 (`pin_d2..d7`) と `pin_dc`、各 variant の native handle は構築後にフィールド代入する。
各 variant config は `using IBusConfig::IBusConfig;` でこのタグ ctor を継承する。

D/C pin は二段で解決する: バス共通の既定は `IBusConfig::pin_dc`、device 差が
あるときだけ `MasterAccessConfig::pin_dc` (非負) が transfer 単位でそれを上書き
する。単一 display の配線では bus 側だけ設定すればよく、D/C 配線の異なる
display 系 device を同一バスに 2 つぶら下げる構成は accessor 側で表現する。

CS区間はAccessそのものであり、`bus::ScopedAccess scope{dev};`でRAII化できる。
早期returnの多いdisplay initでも解放漏れを防げる。解放errorまで観測したい場合は
`ScopedAccess::finish(timeout)`または`bus::guarded`
([bus_accessor.md](bus_accessor.md) §guarded)。

display 系の定型設定は `MasterAccessConfig` の setup プリセットでまとめて入る:

- **`setupWithDCPin(pin_dc)`** — D/C を専用ピンで搬送 (データシート用語の
  4-wire / 4-line serial)。device の D/C pin・`HalfDuplexWithDcPin` モード・
  `writeCommand*` が前提にする 8-bit command phase を 1 呼び出しで設定する
- **`setupWithDCBit()`** — D/C をデータ先頭の 9 ビット目で搬送 (同 3-wire /
  3-line serial)。D/C pin は使わない (`HalfDuplexWithDcBit` モード)

命名は **D/C の搬送方法** に基づく: 3-wire / 4-wire という線数ベースの呼称は
LCD 文脈 (D/C ビット内蔵 9-bit) とセンサ文脈 (MOSI/MISO 共有半二重) で意味が
衝突する多義語のため採らない。メソッド名と選択される enum 値の語彙が一致する
ので、対応が見たまま分かる。戻り値は `*this` で、後続のフィールド代入に
チェーンできる。

## コントローラの占有

SPI masterの論理割当、standalone SPI slave、provider固有direct APIによる外部host利用は、同じ
ハードウェアコントローラのプールを共有する。プール外の利用者は `SPI::claimController(intent)` で
external claim を取得し、利用を止めてから `releaseClaimedController(controller)` で返す。
claim / release は同じkindのAccessを一つも保持していない箇所で呼ぶ。commitは
allocation lockを保持したままbus lockを待つため、window内からのclaim / releaseはlock順を逆転させる。

`SlaveBusConfig::controller` は claim が返した **0 起点の controller index** を受ける。ESP-IDF の
生の `spi_host_device_t` ではない。ESP-IDF backend は内部で `SPI2_HOST + controller` へ変換し、
flash / PSRAM 用の `SPI1_HOST` はプールにも slave にも含めない。

```cpp
auto claim = M5_Hal.SPI.claimController();  // 既定 = Auto
if (!claim.has_value()) { /* 全コントローラ使用中、または不適格 */ }

spi::SlaveBusConfig cfg;
// pins / mode ...
cfg.controller = claim.value();
auto init = slave_bus.init(cfg);
if (!init.has_value()) {
    M5_Hal.SPI.releaseClaimedController(claim.value());
}

// claim を先に返すと、まだ動いている slave と master 再配置が衝突する。
if (slave_bus.close().has_value()) {
    M5_Hal.SPI.releaseClaimedController(claim.value());
}
```

`controller = -1` (既定値) は standalone 互換の「backend 既定 host (`SPI2_HOST`) を台帳外で使う」
経路であり、同じ host の master との衝突を検出できない自己責任モードである。台帳を使う実運用では
claim 経由を推奨する。`-1` 以外の負値と、存在しない controller index は `init()` が
`INVALID_ARGUMENT` で拒否する。

外部で初期化するESP-IDF hostは、**`spi_bus_initialize()`より前**に対応controllerをclaimする。
現行のportable/native `acquire`はこのhost束縛を公開しない。必要な実装者は`Bus_espidf`のdirect APIを使い、
claimをexternal hostの全寿命にわたってcaller-ownedに保つ。終了順序はdirect busの`close()`、外部
`spi_bus_free()`、`SPI.releaseClaimedController(controller)`である。`close()`はM5HAL内部の
device / worker利用だけを止め、caller-owned hostをfreeせずclaimも返さない。`SPI1_HOST`、範囲外host、
hostとcontrollerの不一致はdirect初期化を`INVALID_ARGUMENT`で拒否する。

## SPI slave

SPI slave は address phase や clock stretch を持たず、master がclockを供給する前にslave側が送信予定dataを
用意する必要がある。正準APIはcaller-owned SPSC queueを持つ`SpiSlaveAccessor`である。`write()`または
`txFrames()`でTXをpreloadし、`beginAccess()`でCS受付を開始する。一つの長期Accessへ任意個のCS frameが到着し、
`read()`または`rxFrames()`で受信済みdataを取り出す。`endAccess(timeout)`は新規受付をfenceし、進行中frameを
期限内に完了またはabortしてbackend producerを停止してからBus lockを解放する。local queue I/OはAccess外でも
利用できるため、停止後のRX drainと次回Access用TX preloadが可能である。

`SlaveAccessConfig::transaction_bytes`はbackendへqueueする一回の最大CS frame長で、TX/RXの
`QueueMode::Byte` / `QueueMode::Frame`は方向ごとに固定する。TX不足分は`SlaveBusConfig::tx_fill_byte`で埋め、
RX容量不足は未読dataを上書きせずstatusへdrop/truncateを記録する。一つのCS区間は一つのcommon frameとして
publishされ、`FrameMetadata::wire_bytes`と`stored_bytes`で実clock量と保存量を区別する。event callbackはISRから
直接呼ばず、`dispatchEvents()`または`eventService()`でtask文脈へ配送する。queueとeventの共通契約は
[slave_queue.md](slave_queue.md)を参照する。

ESP-IDF backendはcallback受付と新規transactionを先に閉じてからworkerを有界停止する。安全なreset/freeが
可能なIDF capabilityはversion gateし、CS low中に安全なhard abortができない場合は`UNSUPPORTED`を返して
close後へcleanupをdeferする。この失敗後はBusをpersistent brokenとし、`close()`後の再initまで
後続`beginAccess()`を`IO_ERROR`で拒否する。終了APIが返った後にbackendがcaller-owned queueへ触れてはならない。
一記述子workerでは、完了descriptorを回収して次をqueueするまで次のCS frameを受けられない。連続frameを
送るmasterはbackend/boardに必要なCS high時間を確保する。必要なCS high時間はportable APIでは固定せず、
対象boardとbackendのverification fixtureが規定する。
開始直後のCS同期edgeが0-byte frameとして観測される場合も、backendは境界を偽装せずmetadata付きでpublishする。

## TransferDesc の役割

SPI は I2C よりも peripheral ごとの癖が多いため、共通性の高い
per-call 情報だけを `spi::TransferDesc` に置く。

- `dc_level_valid` / `dc_level`
  display 系で command/data を分ける DC pin 制御の意図を表す。
- `command` / `address` / `command_bytes` / `address_bytes`
  SPI flash や display controller のように command/address/data phase を持つ
  transfer を、 CS を維持した 1 回の transfer として表す。
- `dummy_cycles`
  data phase の直前に入れる dummy clock 数を表す。 byte 数ではなく clock 数。
  移植性の契約: bit 単位で clock を刻める path は任意の値を honor する
  (software bit-bang、 ESP-IDF `SPI_TRANS_VARIABLE_DUMMY`、 および ESP32 上の
  Arduino 変種 — 端数を `SPIClass::transferBits` でクロックする)。 ESP32 以外の
  素の byte 駆動 Arduino `SPIClass` は 8 の倍数のみ honor し、 端数は
  `INVALID_ARGUMENT` を返す。 全 backend で portable に書くなら 8 の倍数で指定する。

  > 設計メモ (端数 dummy の実装手段): Arduino 変種は SDK 抽象 (SPIClass) の層に
  > 留めるため、 端数クロックは SPIClass 拡張の `transferBits` で出す。 別解の
  > 「`ck_idle_edge` (CPOL) 二度反転でエッジ生成」 はレジスタ直叩きであり、
  > [variants.md](variants.md) の分類では **platform variant の技法**。register-level
  > 実装が必要な場合は `platforms/espressif/esp32` SPI の責務とし、framework variant
  > 側にチップレジスタを持ち込まない。

`TransferDesc` は最初から大きなdescriptorにせず、複数backendで意味を固定できる
具体的要求とtestが揃った語彙だけを追加する。

## Accessor sugar

`MasterAccessor` は I2C と同じ思想で thin sugar を提供する。

- `transfer(desc, src, dst)`
- `write(src)`
- `read(dst)`
- raw pointer overload の `write(ptr, len)` / `read(ptr, len)`
- `beginAccess()` / `endAccess()`
- `getLastTransferStatus()`
- `writeCommand(src)`
- `writeCommandData(src)`
- `writeCommandData(command, src)` / `readCommandData(command, dst)`
- `writeCommandAddressData(command, address, src)` /
  `readCommandAddressData(command, address, dst)`
- `sendDummyClock(count)`

command/address/data 系 API は `MasterAccessConfig::spi_command_length` /
`spi_address_length` を byte 数へ丸め上げ、 command/address/dummy/data phase を
1 回の Bus transfer にまとめる。 CS は phase 間で解除しない。 DC pin がある
software SPI variant では command phase を low、 address/data phase を high にする。
Write-family sugar (`write` / `writeCommand*` / `sendDummyClock`) は
full-or-fail で、成功時は caller data phase の送信量を `result_t<size_t>` で返す。
command/address/dummy phase は低レイヤの `TransferDesc` 側に載るため、この byte count
には含めない。

`beginAccess`はlock取得後に`IBus::beginOperation`を呼び、そこでCSをassertする。
`endAccess`はoutstanding transferを完了させ、`IBus::endOperation`でCSをdeassertしてから
lockを解放する。Accessはnon-nestableで、二重beginは`INVALID_STATE`。sugarはinactiveなら一時Accessを
自動開始・終了し、明示Access中なら既存Accessをborrowする。低レイヤのpublic `transfer()`はactive Accessを
必須とし、inactiveなら`INVALID_STATE`を返す。したがって一つの明示Access内の複数transferは一つのCS frameになる。

低レイヤの`IBus::transfer(...)`はCSを直接操作しない。これはAccessor coreとbackend間のseamであり、
通常利用者は`MasterAccessor::transfer`を、明示`beginAccess`後に呼ぶ。

dummy clock は data phase の直前の latency phase として扱う。 AccessConfig では
read/write の自然な違いを表現するため、 `spi_read_dummy_cycle` と
`spi_write_dummy_cycle` を分ける。 convenience API は read 系で read dummy、 write 系で
write dummy を `TransferDesc::dummy_cycles` に詰める。 特定 command だけ dummy 数が
違う場合は、 caller が `TransferDesc` を直接組んで `transfer()` を呼ぶ。

### Access中のI/O error

各`transfer`は同期して自身の`TransferTotals`またはerrorを返し、
`getLastTransferStatus()`もそのI/O単位で更新する。errorはAccess-wideにlatchしないため、
pre-flight拒否や回復可能なI/O errorの後でも、backendがbrokenでなければcallerは同じAccess内で
次のtransferを試せる。`endAccess`はoutstanding I/Oとbackend cleanupの結果だけを返し、過去の
I/O errorや累計totalsを再返却しない。

- backend (IBus 実装者) の義務: 同期エラーを返す場合は**ワイヤに触れる前に拒否する**。
  これは latch 意味論とは独立に、wire 状態の整合を保証するための契約。

## ワイヤタイミング不変条件

これらは全 backend が守るべきプロトコル正当性の契約である。

**dummy cycle の edge 順序**: ワイヤ上の dummy cycle は **データビットと同じ
first→second level の順**で刻む。各 dummy cycle が必ず sample edge への遷移を 1 回持ち、
設定数ぶんの clock がデバイス側で正確に数えられる。「!CPOL→CPOL」順は CPHA=0 で先頭
cycle が無遷移となり、ワイヤ上の dummy clock が 1 少なくなる off-by-one を生むため採らない。

**MOSI は launch edge の前に確定させる** (software master): 各 bit の MOSI 更新は
launch edge (sample edge の反対側エッジ) を打つ**前**にワイヤへ出す。hardware master の
MOSI 出力遅延は edge から数 ns だが、GPIO 書込み 2 回の順序が「CLK → MOSI」だと MOSI が
edge から書込みギャップ分 (数百 ns、割り込みでさらに伸びる) 遅れる。launch edge 近傍で
MOSI をサンプルする slave (ESP32 初代 slave の CPHA=1 で実測) はこの遅れで**1 つ前の bit**
を読み、受信ストリーム全体が 1 bit 遅れてずれる (先頭に 0 が挿入された形)。MOSI 先行なら
標準 slave の hold 条件 (直前 sample edge から half period) も崩れない。native
回帰 = test_software_spi_wire_order (launch/sample 両エッジ時点の MOSI 有効性)。

**CS deassert 前の SCK 復帰**: 転送の終端では CS deassert の前に SCK を idle level
(CPOL) へ復帰させ、さらに half period 分の settle を置く (CPHA=1 系は元々 idle で終わるため
復帰は無遷移)。CS が active clock のまま解除される波形を防ぐ。

**DC の遷移タイミング**: phase 境界 (command→address→data) の DC 変更は、直前 phase の
最終 sample edge から half period 置いてから行う。phase 遷移は最終 edge と同一 poll 内で
起きるため、即時に書くと DC hold が razor-thin になる (デバイスは edge で latch するので
実害は出にくいが、capture 方式の wire test では分解できない)。

**CS ウィンドウ内への構成 SCK 漏れ防止** (ESP-IDF backend): device の構成
(mode = CPOL/CPHA の確定) は **CS assert の前**に行い、構成を変更した場合は CS 非アクティブの
まま settle 用 dummy 1 clock を出して SCK を mode のアイドルレベルへ確定させる。構成を
transfer 内へ遅延させると、CPOL=1 系で CS assert 後に SCK のアイドル遷移エッジが 1 個混入する
(実機 wire test で検出)。CS 非アクティブ中のクロックはプロトコル不可視なので settle は無害。
device handle はトランザクションを跨いでキャッシュされ、構成 (freq / mode / order / duplex) が
変わったときだけ再生成 + settle が走る — 同一構成の連続トランザクションでは SCK は前回の終端で
既にアイドルに置かれている。

**full-duplex 最終 RX バイトの 0x00 上書き (ESP-IDF backend / ESP32 初代 master の既知の制約)**:
ESP32 (初代) の ESP-IDF hardware master による DMA 全二重転送では、受信バッファの最終
1 バイトが 0x00 に上書きされ得る。この制約はESP32-S3 hardware masterとsoftware masterには適用しない。
最終バイトまで意味を持つ全二重 read が必要な場合は software master を使うか、受信長を
1 バイト余分に確保して末尾を読み捨てる。

**ESP32 (初代) slave の mode 0/2 における MISO 1-bit 早送り (ESP-IDF backend の既知の制約)**:
ESP32 (初代) を SPI slave にすると、mode 0/2 では master の受信ストリーム全体が 1 bit
早くずれる (各受信バイトが「期待値を 1 bit 左シフトし、次バイトの MSB を下位に継いだ値」に
なる)。ESP-IDF driver が同 SoC の DMA シリコン問題を回避するため slave の clock phase を
変更しており、slave出力が最大半クロック早く現れる副作用である。受信(MOSI)は影響を受けない。
**ESP32 (初代) を slave にする場合は mode 1 または 3 を使う**。

**ESP32 (初代) slave の低速 SCLK における RX 末尾 word 欠落 (ESP-IDF backend の既知の制約)**:
ESP32 (初代) の DMA slave は、SCLK が遅いと受信データの**最終 word 域 (末尾 1〜4 バイト)**
が DMA へ書かれず 0x00 のまま残る。bit 数カウント (`trans_len`) は全長を報告するため、
戻り値からは検出できない。機構はシリコン内部(mode 0/2のDMA位相workaroundと同族)で外部からは
制御できない。**ESP32 (初代) をslaveにする場合、masterは実効SCLK >= 250kHz
(推奨400kHz以上)のhardware masterを使う**。
software master を使う必要がある場合は mode 1/3 に限る。

## software SPI variant の実装方針

`variants::frameworks::software` の SPI master は、 GPIO `Pin` を push-pull
で駆動する bit-bang 実装として扱う。`Bus_software` は `service::IService` を実装し
生成domainの`ServiceRunner`（default Halでは`M5_Hal.Services`）に登録され、command / address / dummy / data
phase を持つ小さな state machine として cooperative に駆動される。auto-run が
稼働していなければ `Bus::transfer` の呼び出し元が `runOnce()` で自力ポンプし、
同期 transfer のように完了まで回す (service.md の R2 契約どおり、runner 稼働中の
呼び出し元は `runOnce()` の try-lock に委ねる — 詳細は
[service.md](service.md) §API 契約)。

I2C と違い、SPI は clock stretch や open-drain rise wait がないため、状態機械は
比較的単純に保てる。byte 転送は `ByteTransferState` として bit/edge 単位に分解し、
GPIO 操作後は runner に戻せる構造にしている。

実装不変条件:

- 同期transfer pathのwire semanticとAPIを検証で固定する。
- phase 遷移、bit order、SPI mode、dummy clock、DC 区間は `TransferService`
  内の状態として明示する。
- CS 区間は `MasterAccessor::beginAccess/endAccess`と`IBus::beginOperation/endOperation`で扱い、byte/phase
  transfer 本体から分離する。
- byte 内部は bit/edge 単位の state に分解し、1 回の poll で出力する edge は
  **最大 1 つ**に制限する。poll が遅延して理想スケジュールに backlog ができた
  場合は、複数 edge をまとめて取り戻すのではなく `now` へ再アンカーして
  スケジュールを滑らせる (i2c と同方針 — backlog をまとめて出力すると runner
  速度で連続 edge となり、設定より大幅に速いクロック burst になるため。
  クロックは公称より遅れることはあっても速くならない)。
- `ByteTransferState` は enum による setup/leading/trailing state を持たず、
  `active` / `done` / `pending_second_edge` / `bit_index` で byte 内の進行だけを
  表す。phase 遷移は `TransferService` に閉じ、byte hot path は edge 消費に集中させる。
- bit hot path は `first clock level -> MOSI update -> second clock level -> optional MISO sample`
  のまとまりで処理する。 `first_level` / `second_level` は CPOL/CPHA から事前計算し、
  hot path 内の mode 分岐と、MOSI/MISO 操作が両 half-cycle に散ることを抑える。
- 表示デバイス等で多い write-only transfer では MISO sample 分岐を byte loop
  から外す。 MISO がある read/full-duplex path は別経路で維持し、MOSI-only の
  clock/MOSI hot path をできるだけ細く保つ。
- transfer 途中の `Source` / `Sink` chunk lifetime と CS assert 区間は、service
  runner 経由の cooperative 駆動下でも `bus_accessor.md` §Access契約の
  規約 (short transfer はper-I/O totalsが真実、sugarはactive Accessをborrowする)
  がそのまま適用される。

timing は I2C と同様に `fastTick()` / `fastTickFrequencyHz()` 由来の half period
で管理する。ポール間のスケジューリングは service private の仮想時計
(`ServiceContext::elapsed` の積算、[service.md](service.md) §時間契約) で行い、
エッジ刻み自体は呼び出し内で `ctx.local_tick` を起点に生 `fastTick()` へスピンする
二軸構成をとる — 絶対 tick がポール (= タスク/コア) を跨いで比較されることはない。
呼び出し内スピン中のコア移動だけは契約でも守れないため、**精密なビットバン波形を
要する producer タスクはコアへピン留めすることを推奨**する。
CPOL=1 では CS assert 前に SCLK を idle-high へ置く。これを怠ると、
CS active 直後に余分な active edge として観測されることがあるため、native test
と embedded wire self-test の両方で固定する。

## 検証境界

native検証はAccessor lifecycle、Source/Sink、TransferDescへの変換、software backendのedge順序を扱う。
embedded wire検証はCS active範囲、command/address/data/DC phase、dummy clock、bit order、SPI mode 0-3を扱う。
実効速度とrise timeはboard・配線依存でありportable contractには含めない。検証入口は
[verification.md](../verification.md)を参照する。

## 関連

- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離
- [transfer_desc.md](transfer_desc.md) — `TransferDesc` の位置付け
- [data_io.md](data_io.md) — Source / Sink + Limited 装飾
- [verification.md](../verification.md) — native / embedded / experiment の使い分け
