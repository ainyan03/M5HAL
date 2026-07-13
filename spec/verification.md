# verification — 検証コマンドと運用

> **読者**: メンテナ向け（ビルド・運用・規約）。

M5HAL v2 開発で使う検証コマンドとその運用詳細を示す。

## 基本構成

> **注**: 素で動くのは `test_native` 系 (`pio test -e test_native[ -f ...]`、 `platformio.ini` 本体に定義)、GUI-facing の `RemoteServer_*` / `RemoteTest_host`、および公開 examples の `HowToUse_*`。 表中のそれ以外の `v0_check_*` / `v2_check_*` / `v2_experiment_*` / `v2_test_*` は `pio_envs/**/*.ini.cli` に退避しており、 既定の `platformio.ini` からは読まれない。 実行時に `M5HAL_PIO_EXTRA_CONFIG` 環境変数で該当 `*.ini.cli` (またはグロブ) を指すとロードされる (コピー不要。 手順は [`../pio_envs/README.md`](../pio_envs/README.md)、 CI も同じ env var を step `env:` で設定する)。

| カテゴリ | 用途 | コマンド |
|---|---|---|
| native 単体 | gtest による v2 core / GPIO / I2C / SPI / UART / Memory / Source/Sink / frame codec / bytecode の単体テスト。BusRegistry の exact-instance consuming release、co-owner `BUSY`、release/acquire 直列化、weak 復活時の lifecycle close も含む | `pio test -e test_native` |
| remote mux 単体 | `RemoteSession` / `RemoteServerHandler` / data stream / GPIO event の狭い回帰確認。fire-and-forget drain に加え、session 全経路の RPC 直列化、closed handle の旧 proxy `CLOSED`、明示・自然 BusRelease と ID 隔離を含む | `pio test -e test_native -f v2/native/remote/test_mux_remote` |
| remote TCP E2E | 実物の TCP connection/server 経路。Hal reconnect 後も旧 GPIO `Pin` storage が有効で、新 peer と分離され、最終 cache/no-op 契約になることを含む | `pio test -e test_native -f v2/native/remote/test_tcp_remote_server` |
| SPI API 単体 | SPI Accessor API skeleton の狭い確認 | `pio test -e test_native -f v2/native/bus/test_spi_api` |
| ThreadSanitizer | posix runtime の並行性契約 (`service::ServiceRunner` auto-run、[design/service.md](design/service.md) §R1-R8) と remote session/proxy/release の並行回帰を `-fsanitize=thread` つきで検証。`test_native` と同じスイートを別ビルドで走らせる (低頻度・時間がかかるため常用の CI には含めない) | `pio test -e test_native_tsan` |
| クロスチェック v0 | v0 公開 entry の native + ESP32/S3/C3/C6 Arduino/ESP-IDF build fence | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v0/check.ini.cli pio run -e v0_check_native -e v0_check_esp32_arduino -e v0_check_esp32_espidf -e v0_check_esp32_espidf4 -e v0_check_esp32_espidf6 -e v0_check_esp32s3_arduino -e v0_check_esp32s3_espidf -e v0_check_esp32s3_espidf6 -e v0_check_esp32c3_arduino -e v0_check_esp32c3_espidf -e v0_check_esp32c6_espidf` |
| クロスチェック v2 | v2 公開 entry と主要 API surface の native + ESP32/S3/C3/C6 Arduino/ESP-IDF build fence | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/check.ini.cli pio run -e v2_check_native -e v2_check_esp32_arduino -e v2_check_esp32_espidf -e v2_check_esp32_espidf4 -e v2_check_esp32_espidf6 -e v2_check_esp32s3_arduino -e v2_check_esp32s3_espidf -e v2_check_esp32s3_espidf6 -e v2_check_esp32c3_arduino -e v2_check_esp32c3_espidf -e v2_check_esp32c6_espidf` |
| v2 inline flip fence | `m5::hal::*` が v2 に resolve される構成の確認 | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/check.ini.cli pio run -e v2_check_native_inline` |
| v0/v2 共存 fence (device) | 同一 TU で両エントリを include し、 include ガードの世代分離と platform checker の macro 名前空間分離 (v0 = 無印 / v2 = `M5HAL_V2_`) を device build で保証 (native 側は `test_coexist_include`) | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v0v2/check.ini.cli pio run -e v0v2_check_esp32_arduino -e v0v2_check_esp32s3_arduino -e v0v2_check_esp32_espidf` |
| examples build | `pio_envs/v2/examples.ini` の全 env をビルド。 内訳: `HowToUse_{I2C,SPI,UART,UARTEcho,I2SAudio}_{esp32,esp32s3}` / `HowToUse_I2CRegistry_{esp32,esp32s3}` / `HowToUse_Bytecode_esp32` / `HowToUse_Remote_host` / `HowToUse_RemoteI2S_host` / `BuildTest_*` / `RemoteServer_{esp32,esp32s3,esp32_arduino}` / `RemoteServerTCP_{esp32,esp32_arduino,esp32s3_arduino}` / `RemoteTest_host` | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/examples.ini pio run -e HowToUse_I2C_esp32 -e HowToUse_I2C_esp32s3 -e HowToUse_SPI_esp32 -e HowToUse_SPI_esp32s3 -e HowToUse_UART_esp32 -e HowToUse_UART_esp32s3 -e HowToUse_UARTEcho_esp32 -e HowToUse_UARTEcho_esp32s3 -e HowToUse_I2CRegistry_esp32 -e HowToUse_I2CRegistry_esp32s3 -e HowToUse_I2SAudio_esp32s3 -e HowToUse_I2SAudio_esp32 -e HowToUse_Bytecode_esp32 -e HowToUse_Remote_host -e HowToUse_RemoteI2S_host -e BuildTest_esp32 -e BuildTest_esp32s3 -e BuildTest_host -e RemoteServer_esp32 -e RemoteServer_esp32s3 -e RemoteServer_esp32_arduino -e RemoteServerTCP_esp32 -e RemoteServerTCP_esp32_arduino -e RemoteServerTCP_esp32s3_arduino -e RemoteTest_host` |
| remote serial examples | リモートバス統合ハーネス `examples/v2/RemoteServer` (device) + `examples/v2/RemoteTest` (host) | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/examples.ini pio run -e RemoteServer_esp32_arduino -e RemoteServer_esp32 -e RemoteServer_esp32s3 -e RemoteTest_host` |
| remote TCP examples | TCP transport の device example と host-side remote I2S example | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/examples.ini pio run -e RemoteServerTCP_esp32 -e RemoteServerTCP_esp32_arduino -e RemoteServerTCP_esp32s3_arduino -e HowToUse_RemoteI2S_host` |
| software SPI wire self-test | ESP32 実機で低速 software SPI の command/address/dummy/data、bit order、mode edge semantic を GPIO capture で判定 | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/test.ini.cli pio test -e v2_test_esp32_arduino_software_spi_wire` |
| espidf SPI wire self-test | 同じ wire 契約を ESP-IDF hardware spi_master backend で判定 (capture rig は共通 `test/v2/embedded/bus/spi_wire_capture.hpp`) | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/test.ini.cli pio test -e v2_test_esp32_arduino_espidf_spi_wire` |
| ESP-IDF SPI build check | ESP-IDF framework variant の SPI master backend が public v2 header から見えることを確認 (wire semantic は上記 espidf SPI wire self-test で実機判定) | `M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/check.ini.cli pio run -e v2_check_esp32_espidf` |
| M5UU 破壊検出 | M5UnitUnified との互換性確認 | 下記参照 |
| clang-format | コードフォーマット検証 | 下記参照 |

> **env 命名規則**: check/test は `<v0|v2|v0v2>_<purpose>_<chip>[_<framework>][_<variant>]` (`v0v2` = 両世代共存 fence)。 公開 examples は GUI 左ペインでの識別性を優先し、 `HowToUse_<Kind>_<chip>` を使う。

## CI

push 時の build / test チェックは `.github/workflows/` の workflow に委ねる:
`build-check-pio.yml`、`build-check-idf.yml`、`clang-format-check.yml`、
`Arduino-Lint-Check.yml`。各 job の env 構成・cache 設定は workflow file を参照。 ローカルでは上記の検証
コマンド表で同じ build / test を実行でき、 push 前のバックストップになる。

build 系 workflow は upstream repository と fork で同じ定義を使う。 lint /
clang-format は m5stack org では self-hosted runner、 fork では GitHub-hosted runner
へ切り替えるが、 PlatformIO build matrix と公式 ESP-IDF component build は
GitHub-hosted runner 固定にする。 これにより fork でも非 fork でも同じ workflow が
動作し、 ESP-IDF 世代別 probe を matrix 並列で実行できる。

CI の v2 compile fence は公開 examples を流用しない。 examples は Arduino IDE
利用者が読む単独完結コードとして保ち、compile fence は
`test/v2/build_check/build_check.hpp` に集約する。 この共通コードは
`test/v2/stub/build_check_{native,arduino,espidf}.cpp` (および inline flip fence 用
`build_check_v2inline.cpp`) から各 env でビルドされ、さらに
`test/v2/native/test_build_check/` の gtest から同じ関数を実行する。
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
v2 API surface を呼ぶ。 これによりコンポーネント依存 (`REQUIRES`) の宣言漏れも
検出できる (PlatformIO の espidf build は全 driver header を暗黙にパスへ載せるため
依存漏れを隠蔽する)。 ローカルでは公式 IDF Docker、 CI では
`.github/workflows/build-check-idf.yml` (公式 image を job container に使用) で実行する。
コミュニティ fork の pioarduino platform は h2/p4 をビルドできるが、 公式実装との
挙動一致が保証されないため**検証ゲートには使わない**。

## test / experiments の使い分け

`test/` は合否をコードで判定できる検証を置く。 native test は host 上で完結する単体・protocol semantic を対象にし、 embedded test は実機・実配線を使うが、実行後に PASS/FAIL をテストコード自身が判断できるものを対象にする。

`experiments/` は開発中の観測・測定・調査用 sketch を置く。 ロジアナやオシロで波形品質、実効クロック、rise time、jitter を見るもの、または PlatformIO の `build_flags` で backend / 周波数 / runner 条件を差し替えて探索するものは experiments に置く。

方針として、ロジアナ等で意図を確認した protocol-level の不変条件は、可能なら embedded test に落とす。 たとえば software SPI の command/address/dummy/data phase、CS/DC の区間、dummy clock 数、bit order は低速 self-test で検証できるため、`test/v2/embedded/bus/test_software_spi_wire/` の実機 wire test として管理する。

SPI の CS 区間は `beginAccess` / `endAccess` ではなく `beginTransaction` /
`endTransaction` の責務として検証する。native SPI API test では、単発 sugar が
transaction を自動で包むことと、明示 transaction 中の複数 transfer で CS 区間が
1 回にまとまることを固定する。embedded wire self-test では、CS active 範囲と
command/address/dummy/data phase の意味を低速 capture で確認する。

## software I2C テストの読み方

software I2C の protocol-level native test は `test/v2/native/bus/test_software_i2c/` にある。 slave 側は `SlaveBus_software` + `SlaveStreamAccessor` (トランザクション窓モデル) と `VirtualOpenDrainBus` の組み合わせで検証する。 probe ACK、write、read-only、write-then-read、address NACK、data NACK、clock stretch timeout、STOP 時 SDA stuck-low、read 最終 byte の master NACK 観測に加え、 窓の分離・Tx 自動消滅・underrun fill を固定している。 slave モデルの詳細は [design/i2c.md](design/i2c.md) §I2C slave を参照。

`est kHz` の解釈やpull-up・bus capacitance条件による実測周波数の頭打ちについては、詳細は
[design/i2c.md](design/i2c.md) §timing と物理層の注意 を参照。要点: software I2C native testの
`est kHz`はsynthetic line driver上の内部推定でありwire実測値ではない。実機のwire品質
(pull-up強度・配線長・接続device数)を別途確認すること。

## software SPI 実機 wire self-test 方針

wire 上の protocol semantic の仕様（CS/DC phase 定義・dummy clock 数・SPI mode edge 規約・pacing guard）は [design/spi.md](design/spi.md) §テスト を参照。本 kind 固有の差分のみ以下に示す。

software SPI の速度や波形品質はロジアナ/オシロで確認する。 一方、波形の意味が正しいかだけを見るなら、数kHz から 100kHz 程度へ落とした embedded test で十分に検証できる。 既定 env は software polling capture に余裕を持たせるため 2kHz とする。

想定する test は、送信側 software SPI pin を別の capture GPIO に物理ジャンパし、capture 側が SCLK edge を基準に MOSI / CS / DC を読む方式にする。

```text
SCLK out -> SCLK capture in
MOSI out -> MOSI capture in
CS out   -> CS capture in
DC out   -> DC capture in
```

初期 env は USB 接続だけで smoke できるように、capture pin の既定値を output pin と同じ GPIO にして、GPIO input register から出力 pin の状態を読む。 より wire 寄りに確認したい場合は `M5HAL_TEST_SOFTWARE_SPI_CAPTURE_*` build flags で別 GPIO を指定し、上記の物理ジャンパを行う。

`v2_test_esp32_arduino_software_spi_wire_jumper` は ESP32 用の別 GPIO capture env。 既定では `CLK 18 -> 32`、`MOSI 23 -> 33`、`DC 2 -> 25`、`CS 5 -> 26` を想定する。 `v2_test_esp32s3_arduino_software_spi_wire` は ESP32-S3 用 env で、初期値は USB-only capture を想定する。

> **capture 分解能の限界**: capture task は GPIO ポーリング (1 サンプル ~数 µs) なので、 数 µs 以内に複数の遷移が起きると 1 サンプルに融合して個別 edge を分解できない。 このため本テストは **低速設定 (既定 2 kHz) 専用**で、 高速設定のワイヤ検証には外部ロジックアナライザが要る。

明示 transaction の検証では、2 回以上の `transfer()` / `write()` を連続実行しても CS assert/deassert が transaction の前後 1 回ずつに留まることを見る。 これは command と data を別 transfer に分ける display controller 風の使い方を想定したもの。

この self-test はロジアナの代替ではなく、ロジアナで確認した protocol semantic を守る回帰テストとして扱う。 実効速度、立ち上がり時間、overshoot、jitter は引き続き experiments + 外部測定器で見る。

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
