# M5HAL experiments

`experiments/` は M5HAL 開発中の検証用 sketch を置く場所。

`examples/` は Arduino IDE で開いた利用者がそのまま読んで試せる、単独で完結した教材に限定する。 一方 `experiments/` は PlatformIO の `build_flags` で backend、周波数、runner 条件などを差し替える測定・検証コードを扱う。

実行後にコードだけで PASS/FAIL を判断できるものは `test/` に置く。 実機配線が必要でも、低速 self-test で protocol-level の不変条件を判定できるものは `test/v1/embedded/` に寄せる。 `experiments/` はロジアナ/オシロによる観測、速度上限の探索、測定条件を変えながら調査する sketch を担当する。

## GUI 入口

```bash
pio run -e v1_exp_menu_arduino
pio run -e v1_exp_bench_arduino
pio run -e v1_exp_remote_idf5      # リモートバスサーバ (device)
pio run -e v1_exp_remote_host      # リモートバス CLI メニュー (PC)
```

PlatformIO GUI に表示される experiment env は、目視選択しやすいように
`v1_exp_menu_*` / `v1_exp_bench_*` / `v1_exp_remote_*` の短い入口に絞る。 通常の M5Stack BASIC 実機確認は `menu` から
I2C / SPI / GPIO の小操作をボタンで選び、性能測定は `bench` から
timer / memory などの測定項目を選ぶ。 リモートバス機構の実機確認は `remote` の
device を焼き、PC 側の `remote_host` メニューから操作する。 `arduino` は Arduino framework、
`idf4` / `idf5` / `idf6` は純 ESP-IDF framework で世代差を見るための env。

利用者向け examples は公開入口として GUI に表示する。 高度な単機能測定 env、check/test env は `*.ini.cli` に退避している。
この file 群は通常の `platformio.ini` から読まれず、GUI にも出ない。
速度測定・ロジアナ観測・backend 差し替え検証・広い build/test が必要になった時だけ
一時的に有効化する。 手順は [`../pio_envs/README.md`](../pio_envs/README.md)。

## 配置方針

### GUI-facing

- `experiments/v1/BoardMenu`: M5Stack BASIC 上でボタン操作により GPIO / I2C / SPI の小操作を選んで実行する統合ハーネス。
- `experiments/v1/BenchmarkMenu`: M5Stack BASIC 上でボタン操作により timer / memory allocator などの性能測定を実行する統合ハーネス。
- `experiments/v1/RemoteMenu`: リモートバス機構 (spec/design/remote.md) の統合ハーネス。device 側サーバ + PC 側 stdin メニュー CLI。詳細は後述。

### Advanced / CLI-only (`*.ini.cli`, not GUI)

- `experiments/v1/I2CHotPathBenchmark`: software I2C の write/read hot path を synthetic line driver で測る。
- `experiments/v1/SoftwareSPILogicAnalyzer`: software SPI の SCLK/MOSI/CS/DC をロジアナで確認する。
- `experiments/v1/RemoteMenu` の WiFi/TCP transport 版 env (`v1_exp_remote_tcp_*`): WiFi 資格情報が前提のため GUI から外している (experiments_advanced.ini.cli)。

timer / cycle counter の呼び出しコストは `BenchmarkMenu` の Timer benchmark、
I2C backend 切り替え (Arduino / Software / ESP-IDF) の実機確認は `BoardMenu` の
`Use ... I2C` + scan / register read に統合した (旧 `MicrosBenchmark` /
`HowToUseI2CVariant` / `ESPIDFI2CSmoke` は廃止)。 ESP-IDF I2C の純 `app_main()`
smoke も `BoardMenu` の idf env (`v1_exp_menu_idf5` 等) でカバーする。

## M5Stack BASIC 操作メニュー

`BoardMenu` は M5Stack BASIC 固定 pin の実機ハーネス。 ArduinoIDE
向け教材ではなく、開発者が USB 接続した BASIC 上で複数の小さな操作を
ボタンから選び、 Serial とロジアナで状態を見るための入口として置く。

- BtnA: 前の項目
- BtnC: 次の項目
- BtnB: 選択中の操作を実行

現在の操作項目:

- Use Arduino I2C
- Use Software I2C
- Use ESP-IDF I2C
- I2C scan (`SDA=21`, `SCL=22`)
- I2C register read (`0x00`) for the first scanned device
- Use Arduino SPI
- Use Software SPI
- Use ESP-IDF SPI
- SPI write pattern (`CLK=18`, `MOSI=23`, `MISO=19`, `DC=2`, `CS=5`)
- SPI command/address/dummy pattern
- SPI dummy clocks
- LCD init on M5Stack BASIC built-in panel (`CLK=18`, `MOSI=23`, `DC=27`, `CS=14`, `RST=33`)
- LCD fill pattern through selected SPI bus
- fastTick snapshot
- backlight toggle (`GPIO32`)

`Use ...` 項目は現在の bus variant を切り替える。 `I2C scan` /
`I2C register read` / `SPI write pattern` / `LCD fill pattern` などの操作は、
選択中 variant の bus から実行時に Accessor を作って実行する。 Accessor 自体の
Bus 参照を後から差し替えるのではなく、操作ごとに選択中 Bus へ束縛する。
ESP-IDF I2C / SPI 項目は、対象 framework が対応する ESP-IDF driver を公開している
環境でだけ表示される。 I2C は gen5 (`driver/i2c_master.h`) と gen4 (`driver/i2c.h`)
のどちらかが使える場合に表示される。 Arduino-on-IDF では Arduino
variant と ESP-IDF variant が同時に見えるため、同じメニュー内で backend を切り替えて
比較できる。
LCD 操作は選択順に依存しないよう、panel reset 前に LCD の CS/DC を idle high
へ明示初期化する。
SPI dummy は clock(bit) 単位で扱う。 Software SPI / ESP-IDF SPI と、 ESP32 系の
Arduino SPI (`SPIClass::transferBits`) は sub-byte dummy に対応する。 非 ESP32 の
byte-oriented な Arduino `SPIClass` backend は 8 clock の倍数だけを portable に扱う。
`SPI dummy clocks` は 1/3/7/8/13/16/31/33 clock の pulse 数を、選択中 SPI variant
ごとにロジアナで比較するための操作。

```bash
pio run -e v1_exp_menu_arduino -t upload
pio device monitor -e v1_exp_menu_arduino
```

単機能の速度測定や波形確認は `I2CHotPathBenchmark` /
`SoftwareSPILogicAnalyzer` に残す。 `BoardMenu` は、実機に入れた
まま I2C / SPI / GPIO の代表操作を切り替える統合 smoke test 的な位置づけ。

`SoftwareSPILogicAnalyzer` で確認した command/address/dummy/data phase や read/write dummy の意味は、低速実機 self-test `test/v1/embedded/bus/test_software_spi_wire/` にも載せて回帰検出する。 ロジアナ確認は、速度・波形品質・測定器でしか見えない物理条件の確認に残す。

## M5Stack BASIC ベンチマークメニュー

`BenchmarkMenu` は Arduino framework / ESP32 target の GUI-facing 性能測定入口。
M5Stack BASIC のボタン操作は `BoardMenu` と同じ。

```bash
pio run -e v1_exp_bench_arduino -t upload
pio device monitor -e v1_exp_bench_arduino
```

現在の測定項目:

- Memory snapshot
- Timer benchmark
- Lock benchmark
- Memory size sweep
- TempBuffer benchmark
- malloc comparison
- reallocate benchmark
- Fragmentation
- Pool exhaustion

`Memory size sweep` / `TempBuffer benchmark` / `malloc comparison` は
`M5_Hal.Memory` の fixed-block temp pool と fallback path の呼び出しコストをサイズ別に見る。
`reallocate benchmark` は pool の in-place grow / pool 内 move / preserve size 0 /
pool から fallback への移動 / fallback realloc hook のコストを見る。
`Lock benchmark` は allocator とは独立して、no-op / portMUX critical / FreeRTOS mutex /
scheduler suspend / atomic_flag spin の lock/unlock 固定費を比較する。
`Fragmentation` は pool を 1 block 確保で埋めた後に一部を解放し、
`usedBlocks()` / `largestFreeRun()` の変化を見る。 `Pool exhaustion` は pool を満杯にした状態で
temp allocation が fallback に回る時のコストを見る。

## リモートバス統合ハーネス (RemoteMenu)

`RemoteMenu` はリモートバス機構 ([spec/design/remote.md](../spec/design/remote.md)、
decisions/022-024) の常設受入ハーネス。 旧 examples の Remote 系 4 sketch + host 2 本
(RemoteBus / RemoteBusTcp / RemoteI2S / RemoteI2STcp) をここに一本化した。

- **device** (`RemoteMenu/device/remote_menu_device.cpp`): 内蔵 I2C (`SDA=21/SCL=22`、bus_id 0) と
  allowlist 化した GPIO {39,38,37} を公開する。 Core2 V1.1 (AXP2101 を I2C probe で検出) では
  内蔵スピーカーの I2S TX (bus_id 1) も公開する。 AXP2101 が見つからない基板 (Core BASIC 等) では
  I2S を一切構成しない — `GPIO12/0/2` は M-BUS に露出しており 12 はブートストラップピンのため、
  誤駆動を避ける安全ゲート。
- **host** (`RemoteMenu/host/remote_menu_host.cpp`): stdin メニューの CLI。 caps / ping (RTT) /
  I2C scan / レジスタ read / GPIO スナップショット / gpio-write (allowlist 拒否の実演) /
  push イベント監視 (watch) / 正弦波ストリーミング (tone、`hz=0` で無音) /
  credit 会計ソーク (soak、終了時に avg vs 公称と credit 沈降の判定サマリ) を選択実行する。

```bash
# UART transport (espidf、3 Mbaud。Core2 V1.1 なら I2S も公開される)
pio run -e v1_exp_remote_idf5 -t upload
pio run -e v1_exp_remote_host
.pio/build/v1_exp_remote_host/program <port> 3000000

# UART transport (arduino、115200。I2S なし = arduino-esp32 2.x に IDF gen5 I2S が無い)
pio run -e v1_exp_remote_arduino -t upload
.pio/build/v1_exp_remote_host/program            # 自動探索 (115200)

# WiFi/TCP transport (資格情報は環境変数から define へ注入)
M5HAL_WIFI_SSID="myssid" M5HAL_WIFI_PASS="mypass" \
M5HAL_PIO_EXTRA_CONFIG="pio_envs/v1/experiments_advanced.ini.cli" \
  pio run -e v1_exp_remote_tcp_idf5 -t upload    # または v1_exp_remote_tcp_arduino
.pio/build/v1_exp_remote_host/program tcp:<board-ip>:5555
```

transport (UART/TCP) と framework (arduino/espidf) は同一ソースの build flag 差で切り替わる。
`v1_exp_remote_tcp_arduino` は「espidf variant の TcpListener/TcpStream (BSD ソケット、
`ESP_PLATFORM` ゲート) が arduino framework でそのまま使える」ことの build fence を兼ねる。

**WiFi ゼロコスト対照**: M5HAL 自体は `<WiFi.h>` / esp_wifi を一切 include しないため、WiFi を
使わない sketch のバイナリには WiFi スタックが入らない。 `v1_exp_remote_arduino` (WiFi include
なし) と `v1_exp_remote_tcp_arduino` (同一 TU + WiFi) の ELF を `nm` / `size` で比較すると実測
できる (flash 差 ~482 KB、WiFi 非使用側の wifi シンボルは arduino core 自前の wrapper ~73 B のみ。
decisions/024)。

## software SPI ロジアナ確認

`SoftwareSPILogicAnalyzer` は外部 SPI slave を必要としない wire activity smoke test。 software SPI bus を初期化し、次のパターンを 100ms ごとに繰り返す。

- `writeCommandData`: command `0x9F`、data `0xA5 0x5A 0x00 0xFF`
- `write`: `0x00 0xFF 0x55 0xAA`
- `writeCommandAddressData`: command `0x02`、address `0x001234`、dummy 4 clock、write `0xDE 0xAD`
- `readCommandAddressData`: command `0x0B`、address `0x001234`、dummy 8 clock、read 4 byte
- `sendDummyClock`: 16 clock

`writeCommandData` は command phase と data phase を 1 回の bus transfer にまとめる。 CS は command と data の間で解除せず、 D/C pin は command phase で low、 data phase で high に切り替える。
`readCommandAddressData` は command / address / dummy / read data を 1 回の bus transfer にまとめる。 `spi_read_dummy_cycle` / `spi_write_dummy_cycle` はそれぞれ read / write data phase の直前に入れる dummy clock 数で、最終的には per-transfer の `TransferDesc::dummy_cycles` として Bus に渡す。

通常の Accessor sugar は内部で `beginTransaction` / `endTransaction` を呼び、1 回の
transfer 前後で CS を assert/deassert する。command と data を別 transfer に分けつつ
CS を維持したい実験では、明示的に `SPIMasterAccessor::beginTransaction()` を呼んでから
複数の `write()` / `transfer()` を行い、最後に `endTransaction()` する。

既定 pin は ESP32 VSPI で見やすい `CLK=18 MOSI=23 MISO=19 DC=2 CS=5`、既定周波数は 1MHz、mode 0、MSB first。

```bash
pio run -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino -t upload
pio device monitor -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino
```

5MHz 設定の上限観測用 env も用意している。

```bash
pio run -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino_5mhz -t upload
pio device monitor -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino_5mhz
```

`M5HAL_ASSERT` の hot path 影響を切り分ける場合は `-DNDEBUG` 付き env を使う。

```bash
pio run -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino_5mhz_ndebug -t upload
pio device monitor -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino_5mhz_ndebug
```

pin / 周波数 / mode / bit order は build flags で差し替えられる。

```ini
-DM5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CLK=14
-DM5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MOSI=13
-DM5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_MISO=12
-DM5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_DC=27
-DM5HAL_EXPERIMENT_SOFTWARE_SPI_PIN_CS=15
-DM5HAL_EXPERIMENT_SOFTWARE_SPI_FREQ=4000000
-DM5HAL_EXPERIMENT_SOFTWARE_SPI_MODE=3
-DM5HAL_EXPERIMENT_SOFTWARE_SPI_ORDER=0
```

## 実機測定メモ

software I2C の wire 上の実測速度は code hot path だけで決まらない。 特に 400kHz 以上では pull-up 抵抗、配線長、接続 device 数、bus capacitance、ロジアナ閾値の影響が大きい。 周波数を比較するときは、設定周波数、SDA/SCL pin、pull-up 条件、接続 device、測定点を一緒に記録する。

`SoftwareSPILogicAnalyzer` は既定 1MHz / mode 0 / MSB first 設定で SCLK の wire activity を観測する。 software SPI の実装 (chunk 単位 transfer + fastTick deadline 方式) を変更した際は、同条件で再測定して比較する。

software SPI は GPIO 抽象を維持したまま、blocking `transfer()` の内部を `TransferService` に分離している。 `TransferService` は command / address / dummy / data / done phase と Source/Sink chunk cursor を持つ内部ステートマシンで、公開 API の挙動は同期 transfer。 CS は `TransferService` ではなく transaction 層で扱う。 runner service へ接続する場合は、この `poll()` 境界をより細かい edge budget / tick budget 駆動へ拡張する。

`sendDummyClock(N)` は N byte ではなく N clock を出力する。 `SoftwareSPILogicAnalyzer` の dummy phase は 16 clock。

`I2CHotPathBenchmark` の `write buffer timed` / `read buffer timed` は実機 timer を使った state machine の内部上限を見る。 `est kHz` は synthetic line driver 上の SCL rise count からの推定であり、wire 上の実測値ではない。
