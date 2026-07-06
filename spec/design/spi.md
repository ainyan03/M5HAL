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
まで実装済み。 ESP-IDF hardware SPI は polling master backend の初版を追加済みで、
今後は実機 smoke と、software SPI の cooperative service 化を段階的に進める。

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
  lifecycle として扱う。
- ESP-IDF variant: `spi_host_device_t host` を持つ。既定値は `SPI2_HOST`。
- software variant: native handle を持たないため、共通 config を空派生した
  `BusConfig_software` を公開する (variant 固有型を受けることで sibling config の
  流入をコンパイルエラーにする — [variants.md](variants.md) §offer 要件)。

これにより、Arduino / ESP-IDF / software のどれを選んでも「共通configは共通情報、
variant configはnative実体」という同じ読み方になる。

コア配線ピンは **タグ型 ctor** で与えられる: `BusConfig{spi::Clk{18}, spi::Mosi{23}, spi::Miso{19}}`
(MISO 省略で write-only)。i2c (`Scl`/`Sda`) / uart (`Tx`/`Rx`) と同じく順序取り違えがコンパイルエラーになる。
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

## ワイヤタイミング不変条件

これらは全 backend が守るべきプロトコル正当性の契約である。

**dummy cycle の edge 順序**: ワイヤ上の dummy cycle は **データビットと同じ
first→second level の順**で刻む。各 dummy cycle が必ず sample edge への遷移を 1 回持ち、
設定数ぶんの clock がデバイス側で正確に数えられる。「!CPOL→CPOL」順は CPHA=0 で先頭
cycle が無遷移となり、ワイヤ上の dummy clock が 1 少なくなる off-by-one を生むため採らない。

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

## software SPI variant の実装方針

`variants::frameworks::software` の SPI master は、 GPIO `Pin` を push-pull
で駆動する bit-bang 実装として扱う。現状は `TransferService` が command /
address / dummy / data phase を持つ小さな `IService` 互換 state machine として
存在し、通常の `Bus::transfer` では同期 runner 的に完了まで回す。

I2C と違い、SPI は clock stretch や open-drain rise wait がないため、同期実装は
比較的単純に保てる。一方で、将来 `M5_Hal.Services` の service runner へ載せる場合に備え、
byte 転送は `ByteTransferState` として bit/edge 単位に分解し、GPIO 操作後は
runner に戻せる構造へ寄せている。

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
- service runner に外部公開する場合は、transfer 途中の `Source` / `Sink` chunk
  lifetime と CS assert 区間を API 契約として追加で固定する。

timing は I2C と同様に `fastTick()` / `fastTickFrequencyHz()` 由来の half period
で管理する。CPOL=1 では CS assert 前に SCLK を idle-high へ置く。これを怠ると、
CS active 直後に余分な active edge として観測されることがあるため、native test
と embedded wire self-test の両方で固定する。

## 将来拡張

- **software SPI の service 化**: `TransferService` を `M5_Hal.Services` の service runner へ載せ、
  cooperative スケジューリングで動かせるようにする。現行の同期 transfer path と wire semantic は
  その前提として native / embedded test で固定する。
- **ESP-IDF SPI master variant の拡充**: polling master の初版を起点に、DMA 転送・
  interrupt driven path・バージョン差分吸収を段階的に追加する。
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
