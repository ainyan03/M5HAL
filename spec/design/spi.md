# design/spi — SPI kind 固有設計

> **読者**: 実装者・レビュー向け（設計仕様）。

## 当面の目標

SPI は I2C と同じく `Bus` / `Accessor` / `TransferDesc` / `Source` / `Sink`
で扱う。 v2 初期の目標は、上位ライブラリが software / hardware backend を
差し替えても同じ呼び出し面を使えるようにすること。

- `MasterAccessor` が `beginAccess` / `endAccess` で排他制御を包む。
- SPI 固有の `beginTransaction` / `endTransaction` が CS assert/deassert を包む。
- 送受信データは `data::MemorySource` / `data::MemorySink` 経由で
  `IBus::transfer(...)` へ渡す。
- command / address / dummy / data phase は 1 回の transfer にまとめ、CS を
  phase 間で解除しない。
- `IBus::transfer` の基底 default は `NOT_IMPLEMENTED` を返す (concrete
  variant — Arduino / ESP-IDF / software — が override して実体を供給する)。
- `IBus::beginTransaction` / `endTransaction` は基底で no-op default を持つ。
  hardware / framework variant は必要に応じて native transaction 設定や CS 制御を
  override する。

この層を土台に、software bit-bang SPI は同期 transfer と低速実機 wire self-test
まで実装済み。software SPI は `M5_Hal.Services` の service runner に登録済みで、
cooperative スケジューリングで駆動される (§software SPI variant の実装方針)。
ESP-IDF hardware SPI は実機 wire 受入済み。ESP-IDF の polling API を使い、data phase は
DMA-capable な二重 bounce buffer と worker task で次 chunk の準備・回収を進める。
interrupt-driven path は未実装。

## Bus の入手

共通機構は [bus_accessor.md](bus_accessor.md) §Bus の保持 を参照。本 kind 固有の差分のみ以下に示す。

- **identity = CLK / MOSI / MISO のコア 3 線**。DC / CS / quad ピン (`d2..d7`) は identity 外で
  **first-config-wins** (理由は `bus/registry.hpp` の `IdentityKey` doc 参照)。
- SPI は I2C と同じ **managed BusView policy** を使う。`acquire<CfgT>(cfg)` は config 型で backend を
  明示固定する経路で、`acquire(LogicalBusConfig{pins, intent})` は配線と `AllocationIntent` だけを
  記録し、`commitBuses()` が HW controller を優先度順に割り当てる。commit の優先度、snapshot 駆動の
  atomicity 方針、query API は [i2c.md](i2c.md) §intent 駆動の HW 割当と同じ。SPI 固有の identity は
  CLK / MOSI / MISO で、MISO-less 構成は MISO を未接続 sentinel として同一ポリシーに載る。
- facade alias 機構は [variants.md](variants.md) §facade kind を参照。

## BusConfig の役割

共通 `IBusConfig` は pin など HAL 共通の情報だけを持つ。Framework 依存の
native handle / host は各 variant 固有の `BusConfig` に置く。各 variant は
`BusConfig` を必ず公開し、`init` はその型を直接受ける非 virtual メンバとして
宣言する (基底 `bus::IBus` に virtual `init` は無い。[variants.md](variants.md)
§offer 要件と [i2c.md](i2c.md) §設定型の説明を参照)。

- Arduino variant: `SPIClass* spi` を明示する。`init(BusConfig)` はその
  `SPIClass` に `begin` / `end` を行い、`attach(SPIClass&)` は caller-owned
  lifecycle として扱う。非 ESP Arduino core の portable `SPIClass` は任意 pin を設定する
  共通 API を持たないため、型指定された `BusConfig_arduino` で CLK / MOSI / MISO が一つでも
  指定されていれば、配線を無視せず `NOT_IMPLEMENTED` を返す。三 pin を未指定にした場合だけ、
  選択した `SPIClass` instance の board 既定配線で `begin()` する。
- ESP-IDF master variant: `spi_host_device_t host` を持つ。既定値は `SPI2_HOST`。
  これは framework 固有 config が native host を受ける欄であり、共通 slave config の
  `controller` (0 起点のプール index) とは単位が異なる。
- software variant: native handle を持たないため、共通 config を空派生した
  `BusConfig_software` を公開する (variant 固有型を受けることで sibling config の
  流入をコンパイルエラーにする — [variants.md](variants.md) §offer 要件)。

これにより、Arduino / ESP-IDF / software のどれを選んでも「共通configは共通情報、
variant configはnative実体」という同じ読み方になる。

型指定 acquire (`acquire<BusConfig_arduino>`) は Arduino backend を明示固定するため、失敗後に
software variantへ切り替えない。任意 pin でsoftware fallbackを許す場合は
`acquire(LogicalBusConfig{Clk{18}, Mosi{23}, Miso{19}, automatic()})` のようなlogical acquireを
使い、bus確定時にbackendを選ぶ。`preferHardware()`もfallbackを許し、`software()`はsoftware固定、
`requireHardware()`はfallback禁止である。
transfer開始後の`NOT_IMPLEMENTED`を契機とした再実行は、CS区間・Source消費・wire副作用を
重複させ得るため行わない。

backendの機能要件はbus確定前の`AllocationIntent`へ宣言する。MISO-less half-duplex RXには
`requireMosiSharedRx(intent)`を使う。このhelperはSPI固有のrequired capabilityを既存intentへ合成し、
identityには影響しない。`requireMosiSharedRx(preferHardware())`では対応hardwareを優先し、非対応なら
この機能を実装済みのsoftware SPIへfallbackする。`requireMosiSharedRx(requireHardware())`ではfallbackを
禁止し、対応controllerがなければ`commitBuses()`がtransfer開始前に`OUT_OF_RESOURCE`を返す。同一identityを
この要件付きで再acquireした場合も、次のcommitで非対応hardwareからsoftwareへdemoteしてから使用する。
accessorの`spi_data_mode`をcommit後に見てbackendを切り替える方式は採らない。
remoteの`BusCreate` wire形式はintent/capabilityを搬送しないため、この要件を指定したremote logical acquireは
黙って無視せず`NOT_IMPLEMENTED`を返す。peer capability negotiationを追加するまでremote保証には含めない。

このcapabilityはMOSIを入力へ切り替えるRX能力だけを表す。CLK未配線でMOSIだけを波形出力に使う能力は
別要件であり、現時点では宣言・広告しない。software SPIはCLKを必要とし、ESP-IDFでCLKを`-1`にした
transferの成立もまだ契約化していないためである。

コア配線ピンは **タグ型 ctor** で与えられる: `BusConfig{spi::Clk{18}, spi::Mosi{23}, spi::Miso{19}}`。
MISOを省略したbusは既定の`FullDuplex`ではwrite-onlyで、RX要求を`INVALID_STATE`としてwire操作前に拒否する。
ただし`spi_data_mode`を`HalfDuplex` / `HalfDuplexWithDcPin` / `HalfDuplexWithDcBit`のいずれかへ
明示設定すると、softwareとESP-IDF variantはMOSIを送受信兼用の単一データ線として扱い、TX phase後に
入力へ切り替えてRX phaseを実行する。ESP-IDFでは`SPI_DEVICE_3WIRE`に対応する。Arduino `SPIClass`
variantはportableな方向切替APIを持たないため、このMISO-less half-duplex RXを`NOT_IMPLEMENTED`で拒否する。
i2c (`Scl`/`Sda`) / uart (`Tx`/`Rx`) と同じく順序取り違えがコンパイルエラーになる。
QSPI データ線 (`pin_d2..d7`) と `pin_dc`、各 variant の native handle は構築後にフィールド代入する。
各 variant config は `using IBusConfig::IBusConfig;` でこのタグ ctor を継承する。

D/C pin は二段で解決する: バス共通の既定は `IBusConfig::pin_dc`、device 差が
あるときだけ `MasterAccessConfig::pin_dc` (非負) が transfer 単位でそれを上書き
する。単一 display の配線では bus 側だけ設定すればよく、D/C 配線の異なる
display 系 device を同一バスに 2 つぶら下げる構成は accessor 側で表現する。

CS 区間 (transaction) は RAII でも書ける: `spi::ScopedTransaction scope{dev};`
は `beginTransaction` / `endTransaction` を `bus::ScopedAccess` と同じ polarity
規約 (`has_error()`) で包み、早期 return の多い display init でも解放漏れが
起きない。解放エラーまで観測したい場合は `bus::guarded`
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

SPI master の論理割当、standalone SPI slave、外部初期化済み host への `attach()` は、同じ
ハードウェアコントローラのプールを共有する。プール外の利用者は `SPI::claimController(intent)` で
external claim を取得し、利用を止めてから `releaseClaimedController(controller)` で返す。
claim / release は同じkindのaccess/transaction windowを一つも保持していない箇所で呼ぶ。commitは
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
if (slave_bus.release().has_value()) {
    M5_Hal.SPI.releaseClaimedController(claim.value());
}
```

`controller = -1` (既定値) は standalone 互換の「backend 既定 host (`SPI2_HOST`) を台帳外で使う」
経路であり、同じ host の master との衝突を検出できない自己責任モードである。台帳を使う実運用では
claim 経由を推奨する。`-1` 以外の負値と、存在しない controller index は `init()` が
`INVALID_ARGUMENT` で拒否する。

外部で初期化する ESP-IDF host は、**`spi_bus_initialize()` より前**に対応 controller を claim する。
`Bus_espidf::attach(host, claimed_controller)` は生 host と claim の index が一致することを検証するが、
claim 自体の所有権は caller に残す。解放順序は `Bus_espidf::release()` → 外部
`spi_bus_free()` → `SPI.releaseClaimedController(claimed_controller)` である。`release()` は M5HAL
内部の device / worker 利用だけを止め、caller-owned host を free せず claim も返さない。

```cpp
auto claim = M5_Hal.SPI.claimController(spi::requireController(0));
if (!claim.has_value()) { /* SPI2 host は使用中、または不適格 */ }

auto host = static_cast<spi_host_device_t>(SPI2_HOST + claim.value());
// spi_bus_initialize(host, ...) は claim 成功後に行う
auto attached = bus.attach(host, claim.value());

// 使用終了時。各段が成功した場合だけ次の所有権を返す。
auto detached = bus.release();
if (detached.has_value()) {
    auto freed = spi_bus_free(host);
    if (freed == ESP_OK) {
        auto unclaimed = M5_Hal.SPI.releaseClaimedController(claim.value());
        // unclaim 失敗時は claim.value() を保持して再試行する
    }
}
```

attach 内で初めて claim すると、それ以前の外部 `spi_bus_initialize()` と master の衝突を防げない。
逆に `Bus_espidf::release()` が claim を返すと、外部 host がまだ初期化済みの窓で pool が再貸与できる。
そのため claim は外部 host の全寿命を包む caller-owned とする。`SPI1_HOST`、範囲外 host、host と
controller の不一致は attach を `INVALID_ARGUMENT` で拒否する。

## SPI slave

SPI slave は address phase や clock stretch を持たず、master が無条件に clock を供給する前に slave 側が
交換バッファを queue する必要がある。このため公開プリミティブは
`SpiSlaveAccessor::serve(data::Source* tx, data::Sink* rx, size_t len, uint32_t timeout_ms)` の 1 つで、
1 回の `serve()` が 1 CS 区間の full-duplex 交換を表す。I2C slave のような個別 read / write や
begin / end transaction seam は持たず、利用側は resident loop で取引ごとに `serve()` を呼ぶ。

`len` は両方向で共有する最大 clock byte 数。`tx` が短いまたは null なら残りの MISO を
`SlaveBusConfig::tx_fill_byte` で埋め、`rx` が null なら MOSI を破棄する。戻り値は master が実際に clock
した byte 数で、短い CS 区間では `len` 未満になり得る。待ち時間内に取引が始まらなかった場合は error
ではなく 0 を返し、queue 済み取引は次の `serve()` で回収する。

## TransferDesc の役割

SPI は I2C よりも peripheral ごとの癖が多い。初期段階では共通性の高い
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
  > [variants.md](variants.md) の分類では **platform variant の技法**。 将来
  > register-level の `platforms/espressif/esp32` SPI を起こす場合は、 そちらで
  > CPOL 反転や `MOSI_DLEN`/`MISO_DLEN` 直接設定を採る余地がある。 framework 変種
  > 側にチップレジスタを持ち込まない。

dual/quad/octal 幅、DMA hint などは今後の concrete variant 実装で必要性が
固まった時点で拡張する。最初から大きな descriptor にせず、実装とテストで
必要になった語彙だけを追加する。

## Accessor sugar

`MasterAccessor` は I2C と同じ思想で thin sugar を提供する。

- `transfer(desc, src, dst)`
- `write(src)`
- `read(dst)`
- raw pointer overload の `write(ptr, len)` / `read(ptr, len)`
- `beginTransaction()` / `endTransaction()`
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

CS assert/deassert は `beginAccess` / `endAccess` ではなく、SPI 固有の
`beginTransaction` / `endTransaction` に閉じる。 `beginAccess` は bus 共通の排他期間で、
1 つ以上の SPI transaction を含んでよい。通常の `transfer()` / sugar は内部で
transaction を自動開始・終了するため、単発用途の互換性は保つ。明示 transaction 中に
複数回 `transfer()` した場合は、transaction depth により CS を維持する。

低レイヤの `IBus::transfer(...)` は、CS を直接操作しない。Accessor sugar から
呼ばれる場合は `MasterAccessor` が transaction を包み、Bus を直接呼ぶ上級用途では
caller が `beginTransaction` / `endTransaction` または variant 固有の等価操作で
transaction 区間を管理する。

dummy clock は data phase の直前の latency phase として扱う。 AccessConfig では
read/write の自然な違いを表現するため、 `spi_read_dummy_cycle` と
`spi_write_dummy_cycle` を分ける。 convenience API は read 系で read dummy、 write 系で
write dummy を `TransferDesc::dummy_cycles` に詰める。 特定 command だけ dummy 数が
違う場合は、 caller が `TransferDesc` を直接組んで `transfer()` を呼ぶ。

### transaction 中のエラー

transaction (CS 保持区間) 内の segment 群は 1 個の論理操作を成す (command 送信 →
data 授受、のような依存チェーン)。したがって **segment の失敗は種別を問わず
transaction に latch される** — `IBus::transfer` の同期エラー (pre-flight 拒否) も、
`waitTransfer` で表面化する wire 失敗も同じ扱い。latch 後は同一 transaction 内の
後続 transfer が同じエラーで reject され、`endTransaction` も同じエラーを報告する。
復帰は新しい transaction の開始 (`beginTransaction` が latch をクリアする) =
チェーン先頭からのやり直し。契約の意味論は I2C と共通
([i2c.md](i2c.md) §transaction 中のエラー — 不採用案の要約もそちら)。

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
1 バイトが 0x00 に上書きされる。2 系統の配線・複数モード/周波数の実機 A/B で再現し、
スレーブの応答内容に依存しない (スレーブ不在で MISO がフロートし全バイト 0xFF を読む交換でも、
最終バイトだけが 0x00 になる)。ESP32-S3 の hardware master では同一条件 (複数 mode・
30 交換連続) で発生しないことを実測済み。software master variant でも発生しない。
最終バイトまで意味を持つ全二重 read が必要な場合は software master を使うか、受信長を
1 バイト余分に確保して末尾を読み捨てる。

**ESP32 (初代) slave の mode 0/2 における MISO 1-bit 早送り (ESP-IDF backend の既知の制約)**:
ESP32 (初代) を SPI slave にすると、mode 0/2 では master の受信ストリーム全体が 1 bit
早くずれる (各受信バイトが「期待値を 1 bit 左シフトし、次バイトの MSB を下位に継いだ値」に
なる)。ESP-IDF driver が同 SoC の DMA シリコン問題を回避するため slave の clock phase を
変更しており、slave 出力が最大半クロック早く現れる副作用 (driver ソース内コメントに明記)。
周波数 (100kHz/1MHz で実測不変) にも master 実装 (hardware/software) にも依存せず決定論的に
再現する。mode 1/3 では発生せず、転送長 (4 の倍数以外の 3/7/30 を含む)・partial (early CS
deassert) を含めて送受とも全バイト一致を実測済み。受信 (MOSI) はどの mode でも影響を受けない。
**ESP32 (初代) を slave にする場合は mode 1 または 3 を使う**。

**ESP32 (初代) slave の低速 SCLK における RX 末尾 word 欠落 (ESP-IDF backend の既知の制約)**:
ESP32 (初代) の DMA slave は、SCLK が遅いと受信データの**最終 word 域 (末尾 1〜4 バイト)**
が DMA へ書かれず 0x00 のまま残る。bit 数カウント (`trans_len`) は全長を報告するため、
戻り値からは検出できない。hardware master の一様なクロックでは 200kHz 以下で全 mode 決定論的に
再現し、250kHz 以上では発生しない (32 バイト転送で実測)。software (bit-bang) master は
poll 律速で実効クロックがこの帯域に入るため、設定周波数に関わらず mode 0/2 で常に発現する
(mode 1/3 は bit-bang の不均一 cadence でも実測上発現しない)。転送長には依存せず、常に最終
word 域だけが欠ける (長さが 4 の倍数でない場合は末尾の端数バイト)。機構はシリコン内部
(mode 0/2 の DMA 位相 workaround と同族) で外部からは制御できない。**ESP32 (初代) を slave
にする場合、master は実効 SCLK ≥ 250kHz (推奨 400kHz 以上) の hardware master を使う**。
software master を使う必要がある場合は mode 1/3 に限る。

## software SPI variant の実装方針

`variants::frameworks::software` の SPI master は、 GPIO `Pin` を push-pull
で駆動する bit-bang 実装として扱う。`Bus_software` は `service::IService` を実装し
`M5_Hal.Services`（service runner）に登録済みで、command / address / dummy / data
phase を持つ小さな state machine として cooperative に駆動される。auto-run が
稼働していなければ `Bus::transfer` の呼び出し元が `runOnce()` で自力ポンプし、
同期 transfer のように完了まで回す (service.md の R2 契約どおり、runner 稼働中の
呼び出し元は `runOnce()` の try-lock に委ねる — 詳細は
[service.md](service.md) §API 契約)。

I2C と違い、SPI は clock stretch や open-drain rise wait がないため、状態機械は
比較的単純に保てる。byte 転送は `ByteTransferState` として bit/edge 単位に分解し、
GPIO 操作後は runner に戻せる構造にしている。

段階移行の方針:

- まず現行の同期 transfer path を維持し、wire semantic と API をテストで固定する。
- phase 遷移、bit order、SPI mode、dummy clock、DC 区間は `TransferService`
  内の状態として明示する。
- CS 区間は `MasterAccessor` / `IBus` の transaction 層で扱い、byte/phase
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
  runner 経由の cooperative 駆動下でも `bus_accessor.md` §transaction 契約の
  規約 (short transfer は totals が真実、sugar は開いている transaction に参加)
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

## 将来拡張

- ESP-IDF hardware SPI の実機 wire 回帰範囲拡充 (§当面の目標)。
- **ESP-IDF SPI master variant の拡充**: 現行の polling API + DMA-capable 二重 bounce buffer +
  worker task を起点に、未実装の interrupt-driven path とバージョン差分吸収を段階的に追加する。
- **dual/quad/octal 幅・DMA hint**: concrete variant 実装で必要性が固まった時点で
  `TransferDesc` を拡張する。最初から大きな descriptor にせず、実装とテストで
  必要になった語彙だけを追加する。
- **platform variant (ESP32 レジスタ直叩き)**: `CPOL` 二度反転や `MOSI_DLEN`/`MISO_DLEN`
  直接設定が必要な場合は、`platforms/espressif/esp32` SPI として起こす。framework variant 側に
  チップレジスタを持ち込まない。

## テスト

API 層は `test/v2/native/bus/test_spi_api` で検証する。

- transfer sugar が lock と Source/Sink を介して Bus に届くこと。
- read/write の raw pointer overload が動くこと。
- access 中の `setConfig` が reject されること。
- 単発 transfer sugar が transaction を自動開始・終了すること。
- 明示 transaction 中の複数 transfer で CS が維持されること。
- command/data split が `TransferDesc` の DC 指示に反映されること。
- command/address/data が CS を維持した 1 回の transfer として Bus に届くこと。
- read/write dummy cycle がそれぞれ `TransferDesc::dummy_cycles` として Bus に届くこと。
- dummy clock が `TransferDesc::dummy_cycles` として Bus に届くこと。
- software SPI bus が CS/DC/CLK/MOSI を駆動し、MISO を読み取れること。
- CPOL=1 で CS assert 前に SCLK idle-high が適用されること。

wire 上の protocol semantic は `test/v2/embedded/bus/test_software_spi_wire/`
で検証する。既定は polling capture に余裕を持たせるため 2kHz とし、USB 接続だけで
smoke できる同一 GPIO capture と、物理ジャンパで別 GPIO に読む env を用意する。
対象は `writeCommandAddressData` の write dummy 4 clock と
`readCommandAddressData` の read dummy 8 clock、CS active 範囲、DC phase、MOSI bit
sequence、LSB first、SPI mode 0-3 の edge semantic とする。実効速度や rise time は
この test の対象外で、ロジアナ/オシロ確認に残す。

## 関連

- [bus_accessor.md](bus_accessor.md) — Bus / Accessor 責務分離
- [transfer_desc.md](transfer_desc.md) — `TransferDesc` の位置付け
- [data_io.md](data_io.md) — Source / Sink + Limited 装飾
- [verification.md](../verification.md) — native / embedded / experiment の使い分け
