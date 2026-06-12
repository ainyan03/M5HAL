# design/spi — SPI kind 固有設計

## 当面の目標

SPI は I2C と同じく `Bus` / `Accessor` / `TransferDesc` / `Source` / `Sink`
で扱う。 v1 初期の目標は、上位ライブラリが software / hardware backend を
差し替えても同じ呼び出し面を使えるようにすること。

- `SPIMasterAccessor` が `beginAccess` / `endAccess` で排他制御を包む。
- SPI 固有の `beginTransaction` / `endTransaction` が CS assert/deassert を包む。
- 送受信データは `data::MemorySource` / `data::MemorySink` 経由で
  `SPIBus::transfer(...)` へ渡す。
- command / address / dummy / data phase は 1 回の transfer にまとめ、CS を
  phase 間で解除しない。
- concrete variant が未実装の段階では `SPIBus::transfer` は `NOT_IMPLEMENTED`
  を返す。
- `SPIBus::beginTransaction` / `endTransaction` は基底で no-op default を持つ。
  hardware / framework variant は必要に応じて native transaction 設定や CS 制御を
  override する。

この層を土台に、software bit-bang SPI は同期 transfer と低速実機 wire self-test
まで実装済み。 ESP-IDF hardware SPI は polling master backend の初版を追加済みで、
今後は実機 smoke と、software SPI の cooperative service 化を段階的に進める。

## BusConfig の役割

共通 `SPIBusConfig` は pin など HAL 共通の情報だけを持つ。Framework 依存の
native handle / host は各 variant 固有の `BusConfig` に置く。各 variant は
`BusConfig` を必ず公開し、`init` はその型を直接受ける非 virtual メンバとして
宣言する (基底 `bus::Bus` に virtual `init` は無い。[variants.md](variants.md)
§offer 要件と [i2c.md](i2c.md) §BusConfig の誤用防止の説明を参照)。

- Arduino variant: `SPIClass* spi` を明示する。`init(BusConfig)` はその
  `SPIClass` に `begin` / `end` を行い、`attach(SPIClass&)` は caller-owned
  lifecycle として扱う。
- ESP-IDF variant: `spi_host_device_t host` を持つ。既定値は `SPI2_HOST`。
- software variant: native handle を持たないため、`using BusConfig = SPIBusConfig;`
  の alias を公開して共通 config をそのまま受ける。

これにより、Arduino / ESP-IDF / software のどれを選んでも「共通configは共通情報、
variant configはnative実体」という同じ読み方になる。

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

`SPIMasterAccessor` は I2C と同じ思想で thin sugar を提供する。

- `transfer(desc, tx, rx)`
- `write(tx)`
- `read(rx)`
- raw pointer overload の `write(ptr, len)` / `read(ptr, len)`
- `beginTransaction()` / `endTransaction()`
- `writeCommand(tx)`
- `writeCommandData(tx)`
- `writeCommandData(command, tx)` / `readCommandData(command, rx)`
- `writeCommandAddressData(command, address, tx)` /
  `readCommandAddressData(command, address, rx)`
- `sendDummyClock(count)`

command/address/data 系 API は `SPIMasterAccessConfig::spi_command_length` /
`spi_address_length` を byte 数へ丸め上げ、 command/address/dummy/data phase を
1 回の Bus transfer にまとめる。 CS は phase 間で解除しない。 DC pin がある
software SPI variant では command phase を low、 address/data phase を high にする。

CS assert/deassert は `beginAccess` / `endAccess` ではなく、SPI 固有の
`beginTransaction` / `endTransaction` に閉じる。 `beginAccess` は bus 共通の排他期間で、
1 つ以上の SPI transaction を含んでよい。通常の `transfer()` / sugar は内部で
transaction を自動開始・終了するため、単発用途の互換性は保つ。明示 transaction 中に
複数回 `transfer()` した場合は、transaction depth により CS を維持する。

低レイヤの `SPIBus::transfer(...)` は、CS を直接操作しない。Accessor sugar から
呼ばれる場合は `SPIMasterAccessor` が transaction を包み、Bus を直接呼ぶ上級用途では
caller が `beginTransaction` / `endTransaction` または variant 固有の等価操作で
transaction 区間を管理する。

dummy clock は data phase の直前の latency phase として扱う。 AccessConfig では
read/write の自然な違いを表現するため、 `spi_read_dummy_cycle` と
`spi_write_dummy_cycle` を分ける。 convenience API は read 系で read dummy、 write 系で
write dummy を `TransferDesc::dummy_cycles` に詰める。 特定 command だけ dummy 数が
違う場合は、 caller が `TransferDesc` を直接組んで `transfer()` を呼ぶ。

ワイヤ上の dummy cycle は **データビットと同じ first→second level の順**で刻む。
これにより各 dummy cycle が必ず sample edge への遷移を 1 回持ち、 設定数ぶんの
clock がデバイスから数えられる (旧実装の「!CPOL→CPOL」順は CPHA=0 で先頭 cycle が
無遷移になり、 ワイヤ上の dummy clock が 1 少なくなる off-by-one があった —
2026-06 の実機 wire test 初実走で検出)。

転送の終端では **CS deassert の前に SCK を idle level (CPOL) へ復帰**させ、 さらに
half period 分の settle を置く (CPHA=1 系は元々 idle で終わるため復帰は無遷移)。
CS が active clock のまま解除される波形を防ぐ。

**DC の遷移も half-period グリッドに載せる**: phase 境界 (command→address→data) の
DC 変更は、 直前 phase の最終 sample edge から half period 置いてから行う。 phase
遷移は最終 edge と同一 poll 内で起きるため、 即時に書くと DC hold が数 µs の
razor-thin になる (デバイスは edge で latch するので実害は出にくいが、 波形として
曖昧で、 capture 方式の wire test では分解できない)。

**CS ウィンドウ内に構成由来の SCK エッジを漏らさない** (ESP-IDF backend): device の
構成 (mode = CPOL/CPHA の確定) は **CS assert の前**に行い、 構成を変更した場合は
CS 非アクティブのまま settle 用 dummy 1 clock を出して SCK を mode のアイドル
レベルへ確定させる。 構成を transfer 内へ遅延させると、 CPOL=1 系で CS assert 後に
SCK のアイドル遷移エッジが 1 個混入する (実機 wire test で検出)。 CS 非アクティブ中の
クロックはプロトコル不可視なので settle は無害。 device handle はトランザクションを
跨いでキャッシュされ、 構成 (freq / mode / order / duplex) が変わったときだけ
再生成 + settle が走る — 同一構成の連続トランザクションでは SCK は前回の終端で
既にアイドルに置かれている。

## software SPI variant の実装方針

`variants::frameworks::software` の SPI master は、 GPIO `Pin` を push-pull
で駆動する bit-bang 実装として扱う。現状は `TransferService` が command /
address / dummy / data phase を持つ小さな `IService` 互換 state machine として
存在し、通常の `Bus::transfer` では同期 runner 的に完了まで回す。

I2C と違い、SPI は clock stretch や open-drain rise wait がないため、同期実装は
比較的単純に保てる。一方で、将来 M5HALCore の service runner へ載せる場合に備え、
byte 転送は `ByteTransferState` として bit/edge 単位に分解し、GPIO 操作後は
runner に戻せる構造へ寄せている。

段階移行の方針:

- まず現行の同期 transfer path を維持し、wire semantic と API をテストで固定する。
- phase 遷移、bit order、SPI mode、dummy clock、DC 区間は `TransferService`
  内の状態として明示する。
- CS 区間は `SPIMasterAccessor` / `SPIBus` の transaction 層で扱い、byte/phase
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

## 今後の実装順

1. software SPI bus の検証を厚くする。
   - GPIO 解決は I2C software variant と同じく `M5_Hal.Gpio.tryGetPin(num)`。
   - 現状は CLK/MOSI/MISO/DC/CS を使った同期 bit-bang transfer を実装済み。
   - mode/order/pin 有無は transfer 開始時に `TransferPlan` へまとめる。
     template dispatch で組み合わせごとに実体を増やさず、非テンプレートの byte
     transfer 実体を維持する。
   - `Source` / `Sink` は chunk 単位で処理する。
   - clock wait は相対 wait ではなく fastTick deadline 方式にし、処理時間を毎 half period に
     加算しない。
   - `dummy_cycles` は byte 数ではなく clock 数として扱う。
   - `spi_read_dummy_cycle` / `spi_write_dummy_cycle` は data phase 前の dummy clock
     として扱い、 address の有無には依存しない。
   - mode/cpol/cpha、bit order、full-duplex の native test を追加する。
   - 低速実機 self-test を `test/v1/embedded/` に置き、 command/address/dummy/data
     phase、CS/DC 区間、dummy clock 数、bit order、mode edge semantic をコード上で判定する。
   - `beginTransaction` / `endTransaction` により、複数 transfer 間で CS を維持できることを
     native test と wire self-test の両側で固定する。
2. software SPI の service 化を進める。
   - 現在の `TransferService` は `IService` 互換の phase state machine だが、
     dummy clock 内部は同期的に完了まで回す。
   - byte transfer 内部は `ByteTransferState` として分離済み。単一 edge でも、
     edge budget に応じた複数 edge でも進められるため、低速時は cooperative に、
     高速時は batch 的に動かせる。
   - 速度最適化は GPIO 汎用 API の構造を壊さず、SPI 側の hot path 分岐削減と
     service dispatch 粒度の調整を中心に行う。
3. ESP-IDF SPI master variant を実機確認する。
   - I2C と同じく framework variant 内部で ESP-IDF バージョン差分を吸収する。
   - Arduino-on-ESP-IDF と ESP-IDF components + Arduino の併存を前提に、
     `ARDUINO` の有無だけで ESP-IDF variant を無効化しない。
   - 初版は `driver/spi_master.h` の polling transfer に委譲する。 CS と D/C は
     M5HAL の transaction / phase 制御に残し、 Arduino / software variant と同じ
     accessor 意味論を保つ。
4. experiment sketch と embedded test を使い分けて実機で確認する。
   - Arduino IDE 向け examples は単独完結のサンプルに限定し、variant 差し替えや
     計測用コードは `experiments/` に置く。
   - ロジアナ/オシロで確認する速度・波形品質は `experiments/`、実機配線が必要でも
     PASS/FAIL を self-contained に判定できる protocol semantic は `test/v1/embedded/`
     に置く。

## テスト

API 層は `test/v1/native/bus/test_spi_api` で検証する。

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

wire 上の protocol semantic は `test/v1/embedded/bus/test_software_spi_wire/`
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
