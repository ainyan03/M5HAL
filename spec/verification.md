# verification — 検証コマンドと運用

M5HAL v1 開発で使う検証コマンドとその運用詳細を示す。

## 基本構成

> **注**: 素で動くのは `test_native` 系 (`pio test -e test_native[ -f ...]`、 `platformio.ini` 本体に定義)、GUI-facing の `v1_exp_menu_*` / `v1_exp_bench_*`、および公開 examples の `HowToUse_*`。 表中のそれ以外の `v0_check_*` / `v1_check_*` / `v1_experiment_*` / `v1_test_*` は `pio_envs/**/*.ini.cli` に退避しており、 既定の `platformio.ini` からは読まれない。 実行時に `M5HAL_PIO_EXTRA_CONFIG` 環境変数で該当 `*.ini.cli` (またはグロブ) を指すとロードされる (コピー不要。 手順は [`../pio_envs/README.md`](../pio_envs/README.md)、 CI も同じ env var を step `env:` で設定する)。

| カテゴリ | 用途 | コマンド |
|---|---|---|
| native 単体 | gtest による v1 core / GPIO / I2C / SPI / UART / Memory / Source/Sink / frame codec / bytecode の単体テスト | `pio test -e test_native` |
| SPI API 単体 | SPI Accessor API skeleton の狭い確認 | `pio test -e test_native -f v1/native/bus/test_spi_api` |
| クロスチェック v0 | v0 公開 entry の native + ESP32/S3/C3/C6 Arduino/ESP-IDF build fence | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v0/check.ini.cli pio run -e v0_check_native -e v0_check_esp32_arduino -e v0_check_esp32_espidf -e v0_check_esp32_espidf4 -e v0_check_esp32_espidf6 -e v0_check_esp32s3_arduino -e v0_check_esp32s3_espidf -e v0_check_esp32s3_espidf6 -e v0_check_esp32c3_arduino -e v0_check_esp32c3_espidf -e v0_check_esp32c6_espidf` |
| クロスチェック v1 | v1 公開 entry と主要 API surface の native + ESP32/S3/C3/C6 Arduino/ESP-IDF build fence | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/check.ini.cli pio run -e v1_check_native -e v1_check_esp32_arduino -e v1_check_esp32_espidf -e v1_check_esp32_espidf4 -e v1_check_esp32_espidf6 -e v1_check_esp32s3_arduino -e v1_check_esp32s3_espidf -e v1_check_esp32s3_espidf6 -e v1_check_esp32c3_arduino -e v1_check_esp32c3_espidf -e v1_check_esp32c6_espidf` |
| v1 inline flip fence | `m5::hal::*` が v1 に resolve される構成の確認 | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/check.ini.cli pio run -e v1_check_native_inline` |
| v0/v1 共存 fence (device) | 同一 TU で両エントリを include し、 include ガードの世代分離と platform checker の macro 名前空間分離 (v0 = 無印 / v1 = `M5HAL_V1_`) を device build で保証 (native 側は `test_coexist_include`) | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v0v1/check.ini.cli pio run -e v0v1_check_esp32_arduino -e v0v1_check_esp32s3_arduino -e v0v1_check_esp32_espidf` |
| examples build | `examples/v1/HowToUse/I2C` / `SPI` / `UART` / `UARTEcho` / `I2SAudio` の ESP32 / ESP32-S3 ビルド + `Bytecode` (Core BASIC 前提、ESP32 のみ) | `pio run -e HowToUse_I2C_esp32 -e HowToUse_I2C_esp32s3 -e HowToUse_SPI_esp32 -e HowToUse_SPI_esp32s3 -e HowToUse_UART_esp32 -e HowToUse_UART_esp32s3 -e HowToUse_UARTEcho_esp32 -e HowToUse_UARTEcho_esp32s3 -e HowToUse_I2SAudio_esp32 -e HowToUse_I2SAudio_esp32s3 -e HowToUse_Bytecode_esp32` |
| remote harness build | リモートバス統合ハーネス `experiments/v1/RemoteMenu` (device 4 構成 + PC 側 CLI メニュー)。arduino framework のソケット層 fence と WiFi ゼロコスト対照を兼ねる | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/experiments_advanced.ini.cli pio run -e v1_exp_remote_arduino -e v1_exp_remote_idf5 -e v1_exp_remote_host -e v1_exp_remote_tcp_arduino -e v1_exp_remote_tcp_idf5` |
| I2C variant 実機切替 | BoardMenu で Arduino / Software / ESP-IDF I2C backend を切り替えて scan / register read | `pio run -e v1_exp_menu_arduino -t upload` |
| software SPI logic analyzer | software SPI の SCLK/MOSI/CS/DC wire activity smoke test | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/experiments_advanced.ini.cli pio run -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino` |
| software SPI 5MHz / NDEBUG | software SPI の高周波設定と `M5HAL_ASSERT` 無効化の比較 | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/experiments_advanced.ini.cli pio run -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino_5mhz -e v1_experiment_SoftwareSPILogicAnalyzer_esp32_arduino_5mhz_ndebug` |
| software SPI wire self-test | ESP32 実機で低速 software SPI の command/address/dummy/data、bit order、mode edge semantic を GPIO capture で判定 | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/test.ini.cli pio test -e v1_test_esp32_arduino_software_spi_wire` |
| espidf SPI wire self-test | 同じ wire 契約を ESP-IDF hardware spi_master backend で判定 (capture rig は共通 `test/v1/embedded/bus/spi_wire_capture.hpp`) | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/test.ini.cli pio test -e v1_test_esp32_arduino_espidf_spi_wire` |
| ESP-IDF I2C smoke | BoardMenu の純 ESP-IDF env で ESP-IDF I2C backend を選び bus init → scan → register read | `pio run -e v1_exp_menu_idf5 -t upload` |
| ESP-IDF SPI build check | ESP-IDF framework variant の SPI master backend が public v1 header から見えることを確認 (wire semantic は上記 espidf SPI wire self-test で実機判定) | `pio run -e v1_check_esp32_espidf` |
| software I2C hot path | bit-bang I2C の timer / virtual GPIO / write buffer state machine の切り分け | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/experiments_advanced.ini.cli pio run -e v1_experiment_I2CHotPathBenchmark_esp32_arduino` |
| M5UU 破壊検出 | M5UnitUnified との互換性確認 | 下記参照 |
| clang-format | コードフォーマット検証 | 下記参照 |

> **env 命名規則**: check/test/experiment は `<v0|v1|v0v1>_<purpose>_<chip>[_<framework>][_<variant>]` (`v0v1` = 両世代共存 fence)。 公開 examples は GUI 左ペインでの識別性を優先し、 `HowToUse_<Kind>_<chip>` を使う。

## CI

push 時の build / test チェックは `.github/workflows/` の workflow に委ねる
(PlatformIO build matrix と、 公式 ESP-IDF コンポーネントビルドを別 workflow に分ける)。
各 job の env 構成・cache 設定は workflow file を参照。 ローカルでは上記の検証
コマンド表で同じ build / test を実行でき、 push 前のバックストップになる。

build 系 workflow は upstream repository と fork で同じ定義を使う。 lint /
clang-format は m5stack org では self-hosted runner、 fork では GitHub-hosted runner
へ切り替えるが、 PlatformIO build matrix と公式 ESP-IDF component build は
GitHub-hosted runner 固定にする。 これにより fork でも非 fork でも同じ workflow が
動作し、 ESP-IDF 世代別 probe を matrix 並列で実行できる。

CI の v1 compile fence は公開 examples を流用しない。 examples は Arduino IDE
利用者が読む単独完結コードとして保ち、compile fence は
`test/v1/build_check/build_check.hpp` に集約する。 この共通コードは
`test/v1/stub/build_check_{native,arduino,espidf}.cpp` (および inline flip fence 用
`build_check_v1inline.cpp`) から各 env でビルドされ、さらに
`test/v1/native/test_build_check/` の gtest から同じ関数を実行する。
これにより I2C / SPI / UART の accessor sugar、Source/Sink overload、Arduino /
ESP-IDF variant config の公開名が examples と独立して壊れていないことを確認する。

### 公式 ESP-IDF コンポーネントビルド (全 ESP32 family)

PlatformIO の `espressif32` platform は board file の都合で一部 chip (esp32-h2 /
esp32-p4 など) をビルドできないが、 **公式 ESP-IDF 本体は M5HAL の対象 chip
(`idf_component.yml` の `targets:` = esp32 / esp32s2 / esp32s3 / esp32c2 /
esp32c3 / esp32c5 / esp32c6 / esp32c61 / esp32h2 / esp32p4) をサポートする**。
そこで権威ある全機種検証として、 公式 `espressif/idf` toolchain で M5HAL を
**ESP-IDF コンポーネントとしてビルド**する lane を持つ。

`test/idf_component_build/` が consumer プロジェクト。 `idf.py set-target <chip> build`
で M5HAL と M5Utility を `EXTRA_COMPONENT_DIRS` から component 登録し
(`IDF_COMPONENT_MANAGER=0` で hermetic)、 `main` が前述の `build_check.hpp` の
v1 API surface を呼ぶ。 これによりコンポーネント依存 (`REQUIRES`) の宣言漏れも
検出できる (PlatformIO の espidf build は全 driver header を暗黙にパスへ載せるため
依存漏れを隠蔽する)。 ローカルでは公式 IDF Docker、 CI では
`.github/workflows/build-check-idf.yml` (公式 image を job container に使用) で実行する。
コミュニティ fork の pioarduino platform は h2/p4 をビルドできるが、 公式実装との
挙動一致が保証されないため**検証ゲートには使わない**。

## test / experiments の使い分け

`test/` は合否をコードで判定できる検証を置く。 native test は host 上で完結する単体・protocol semantic を対象にし、 embedded test は実機・実配線を使うが、実行後に PASS/FAIL をテストコード自身が判断できるものを対象にする。

`experiments/` は開発中の観測・測定・調査用 sketch を置く。 ロジアナやオシロで波形品質、実効クロック、rise time、jitter を見るもの、または PlatformIO の `build_flags` で backend / 周波数 / runner 条件を差し替えて探索するものは experiments に置く。

方針として、ロジアナ等で意図を確認した protocol-level の不変条件は、可能なら embedded test に落とす。 たとえば software SPI の command/address/dummy/data phase、CS/DC の区間、dummy clock 数、bit order は低速 self-test で検証できるため、`test/v1/embedded/bus/test_software_spi_wire/` の実機 wire test として管理する。

SPI の CS 区間は `beginAccess` / `endAccess` ではなく `beginTransaction` /
`endTransaction` の責務として検証する。native SPI API test では、単発 sugar が
transaction を自動で包むことと、明示 transaction 中の複数 transfer で CS 区間が
1 回にまとまることを固定する。embedded wire self-test では、CS active 範囲と
command/address/dummy/data phase の意味を低速 capture で確認する。

## software I2C 実機ベンチの読み方

検証用 sketch は `experiments/` に置く。 `examples/` は Arduino IDE で開いた利用者がそのまま読める単独完結コードに限定し、 PlatformIO の `build_flags` で backend や runner 条件を差し替えるものは `experiments/` に分離する。

`I2CHotPathBenchmark` は外部 I2C slave を必要とせず、 software I2C の hot path を分解して測るための sketch。 `fastTick loop` は timer 取得コスト、 `virtual line ops` は line driver の仮想呼び出しと read/write の上限、 `write buffer forced due` / `write buffer timed` は byte 列送信 state machine、 `read buffer forced due` / `read buffer timed` は byte 列受信 state machine の内部上限を見る。

software I2C の protocol-level native test は `test/v1/native/bus/test_software_i2c/` にある。 当初の `MockSlave` 案は、現在は `SlaveService` と `VirtualOpenDrainBus` の組み合わせとして実装済み。 probe ACK、write、read-only、write-then-read、address NACK、data NACK、clock stretch timeout、STOP 時 SDA stuck-low、read 最終 byte の master NACK 観測を固定している。

この sketch の `est kHz` は synthetic line driver 上の内部推定であり、 wire 上の実測周波数ではない。 実機の SCL/SDA は pull-up 抵抗、配線長、接続 device 数、bus capacitance、ロジアナ閾値の影響を受ける。 特に 400kHz を超える software I2C 検証では、SCL/SDA の pull-up が弱いと rise time が支配的になり、 code hot path より低い周波数で頭打ちになる。

software I2C の性能評価では、code hot path の内部推定 (`write buffer timed` 等) と bus 物理条件 (pull-up 強度・配線・接続 device 数) を分けて見ること。 SCL pull-up を強めると wire 実測周波数は上がる一方、内部推定はそれより高い値を示しうる。

設定周波数への追従性を見る場合は、pull-up 条件を明記する。 十分な pull-up なら設定周波数にほぼ追従し、弱い pull-up では低速化するが通信自体は成立する、という形を安全側の確認として扱う。

## software SPI 実機 wire self-test 方針

software SPI の速度や波形品質はロジアナ/オシロで確認する。 一方、波形の意味が正しいかだけを見るなら、数kHz から 100kHz 程度へ落とした embedded test で十分に検証できる。 既定 env は software polling capture に余裕を持たせるため 2kHz とする。

想定する test は、送信側 software SPI pin を別の capture GPIO に物理ジャンパし、capture 側が SCLK edge を基準に MOSI / CS / DC を読む方式にする。

```text
SCLK out -> SCLK capture in
MOSI out -> MOSI capture in
CS out   -> CS capture in
DC out   -> DC capture in
```

初期 env は USB 接続だけで smoke できるように、capture pin の既定値を output pin と同じ GPIO にして、GPIO input register から出力 pin の状態を読む。 より wire 寄りに確認したい場合は `M5HAL_TEST_SOFTWARE_SPI_CAPTURE_*` build flags で別 GPIO を指定し、上記の物理ジャンパを行う。

`v1_test_esp32_arduino_software_spi_wire_jumper` は ESP32 用の別 GPIO capture env。 既定では `CLK 18 -> 32`、`MOSI 23 -> 33`、`DC 2 -> 25`、`CS 5 -> 26` を想定する。 `v1_test_esp32s3_arduino_software_spi_wire` は ESP32-S3 用 env で、初期値は USB-only capture を想定する。

最初の対象は低速の protocol semantic。 `writeCommandAddressData(0x02, 0x001234, {0xDE, 0xAD})` では command 8 clock、address 24 clock、write dummy 4 clock、data 16 clock、 `readCommandAddressData(0x0B, 0x001234, rx[4])` では command 8 clock、address 24 clock、read dummy 8 clock、read data 32 clock を検証する。 CS は phase 全体で active、DC は command phase だけ low、address/dummy/data phase は high であることを固定する。 追加で LSB first の bit sequence と、SPI mode 0-3 の active edge polarity / edge alternation、 および pacing guard (連続 3 edge のスパン >= half period — 「設定より速くクロックしない」方針の恒久ガード) を検証する。

> **capture 分解能の限界**: capture task は GPIO ポーリング (1 サンプル ~数 µs) なので、 数 µs 以内に複数の遷移が起きると 1 サンプルに融合して個別 edge を分解できない。 このため本テストは **低速設定 (既定 2 kHz) 専用**で、 高速設定のワイヤ検証には外部ロジックアナライザが要る。 実装側も phase 境界のイベント (DC 遷移、 idle 復帰) を half-period グリッドに載せて、 capture が分解できる間隔を保証している (design/spi.md)。

明示 transaction の検証では、2 回以上の `transfer()` / `write()` を連続実行しても
CS assert/deassert が transaction の前後 1 回ずつに留まることを見る。これは
command と data を別 transfer に分ける display controller 風の使い方を想定したもの。

この self-test はロジアナの代替ではなく、ロジアナで確認した protocol semantic を守る回帰テストとして扱う。 実効速度、立ち上がり時間、overshoot、jitter は引き続き experiments + 外部測定器で見る。

software SPI の速度調整では、GPIO 汎用 API 自体を崩すより、SPI hot path 内の edge 配置・batch 粒度・read/write 分岐削減を優先して評価する。 bit hot path を `first clock level -> MOSI update -> second clock level` に寄せると、高速設定時の SCLK half-cycle 幅の偏りが改善する。

## M5UnitUnified 連携ビルドチェック

M5HAL の変更が主要利用者 M5UnitUnified の Adapter 層を壊していないことを確認する。 fence では M5UU `main` と M5HAL の v0 expose の組み合わせを対象にする。

### 前提条件

- M5HAL リポジトリと M5UnitUnified リポジトリがローカルに並列配置されている
- M5UnitUnified が `main` ブランチに checkout されている (= 安定リリース系列を fence の対象とする)
- `M5UnitUnified/lib/M5HAL` が M5HAL リポジトリへの symlink、 `M5UnitUnified/lib/M5Utility` が M5Utility リポジトリへの symlink になっている
- M5UU 側 `platformio.ini` の `[readme_base]` で `m5stack/M5HAL` と `m5stack/M5Utility` を `lib_deps` から外している (= symlink 解決を優先させる)

### コマンド (M5UnitUnified リポジトリのルートで実行)

```bash
pio test -e test_readme_Core --without-uploading --without-testing
```

### 注意点

- `pio run` ではなく `pio test` を使う (= `test_readme_Core` env はテスト系列に属するため)
- `--without-uploading --without-testing` を付ける (= ビルド確認のみで、 実機書き込み・実行はしない)

## clang-format 運用

```bash
clang-format-18 --style=file --dry-run --Werror \
  $(find src -type f \( -name "*.hpp" -o -name "*.cpp" -o -name "*.inl" \))
```

- 採用バージョンは **v18**
- CI 検査範囲は `src/` のみ

### `_offer.hpp` の注意

variant の `_offer.hpp` は bare path マクロを含むため、 clang-format による自動整形で壊れる。 `_offer.hpp` 全体は `// clang-format off` / `// clang-format on` で囲む。

## 関連

- [README.md](README.md)
- [style/migration.md](style/migration.md)
