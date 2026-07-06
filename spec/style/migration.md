# style/migration — v0 API → v2 API 移行ガイド

> **読者**: 利用者向け。

v0 API 利用者が v2 API に移行する際の指針を示す。

## 基本方針

- 既存コードをそのまま使い続ける場合は `<M5HAL.hpp>` または `<M5HAL_v0.hpp>` を使う
- v2 API を使う場合は `<M5HAL_v2.hpp>` を使う
- v0 と v2 は同一ライブラリ内で共存し、 **同一 translation unit での両エントリ include も可能** (移行途中のファイル等。 [design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §エントリヘッダ)。 ただし可読性のため、 通常は TU ごとに使う世代を明示する
- v2 への移行は、 旧 API の置き換えではなく **新しい API 体系への移行** として扱う

## ヘッダ選択

詳細は [design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §エントリヘッダ を参照。

## 移行の考え方

各 API は次の 3 区分で考える。

| 区分 | 意味 |
|---|---|
| **保持** | 命名・役割をほぼ維持して使える |
| **再構成** | 概念は残るが、 使い方や責務が変わる |
| **廃止** | v2 では使わない |

## 維持される要素

| API | 配置 | 備考 |
|---|---|---|
| `error::error_t` / `error::isError` / `error::isOk` | `hal/error.hpp` | cross-cutting な型 |
| `types::PeripheralType` / `types::BusType` / `types::GpioMode` | `hal/types.hpp` | 命名維持 |
| `types::gpio_number_t` | `hal/types.hpp` | pin 指定の基本型 |
| `M5HAL_V2_TARGET_PLATFORM_*` / `M5HAL_FRAMEWORK_HAS_*` | 各 `_checker.hpp` | variant 機構で利用。 platform 系は世代分離のため `M5HAL_V2_` プレフィックス (無印は v0 が所有) |

## v0 → v2 読み替え一覧

| v0 の考え方 / API | v2 の考え方 / API | 備考 |
|---|---|---|
| バス操作は旧 chain API で行う | `transfer` を核に行う | Bus / BusConfig / AccessConfig は構造維持 |
| `beginAccess` / `endAccess` chain virtual | `beginAccess` / `endAccess` + sugar を中心に再構成 | Accessor 抽象 |
| `interface::io::Input` / `Output` 系 | `Source` / `Sink` | [design/data_io.md](../design/data_io.md) §向き (direction) の規約 参照 |
| 旧 I2C 操作 / software I2C singleton | `write` / `read` / `writeRegister` / `readRegister` / `probe` + software variant | [design/i2c.md](../design/i2c.md) 参照 |
| `Bus::beginAccess(AccessConfig&)` factory | 利用者が `Accessor` を直接構築 | [design/bus_accessor.md](../design/bus_accessor.md) §Bus の保持 参照 |
| 旧 `interface::gpio::*` 抽象 | `IGPIO` / `IPort` / `Pin` / `GPIOGroup` | [design/gpio.md](../design/gpio.md) 参照 |

## v2 で使わない要素

| API | 備考 |
|---|---|
| `interface::io::Input` / `Output` 系 | `Source` / `Sink` へ置換 |
| 旧 `bus::Accessor` chain virtual | `transfer` ベースへ置換 |
| `Bus::beginAccess(AccessConfig&)` factory | 利用者が `Accessor` を直接構築 |
| 旧 software I2C singleton 群 | software variant に置換 |
| 旧 `interface::gpio::*` 抽象 | v2 GPIO 抽象へ置換 |

## v2 専用 (v0 対応なし)

以下は v2 のみが持つ機能で、 v0 から移行する概念がない:

| 機能 | 説明 |
|---|---|
| リモートバス機構 | `Hal::connect(endpoint)` (endpoint = `"uart:<path>"` / `"tcp:<host>:<port>"`。 typed API `initUart(port)` / `initTcp("host:port")` も存続) で遠隔 M5HAL server に接続し、 同型 `hal.I2C.acquire()` / `SPI.acquire()` 等で proxy bus を取得する。 追加のビルドフラグは不要 (`M5HAL_CONFIG_REMOTE=1` は winner scan へ remote variant を参加させる別用途の opt-in)。 詳細は [design/remote.md](../design/remote.md) |

## 移行時の確認項目

1. include しているヘッダが v0 か v2 かを明示する (詳細は [design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §エントリヘッダ)
2. 同一 TU で v0 / v2 を併用する場合は [design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §同一 TU 安全性の保証 (coexist fence) の前提を満たす
3. 旧 I/O 抽象を `Source` / `Sink` に置き換える (詳細は [design/data_io.md](../design/data_io.md) §向き (direction) の規約)
4. 旧 I2C 操作を `transfer` または v2 sugar に置き換える (詳細は [design/i2c.md](../design/i2c.md))
5. 旧 GPIO 抽象を `Pin` / `IPort` / `GPIOGroup` ベースへ置き換える (詳細は [design/gpio.md](../design/gpio.md))

## 関連

- [../design/bus_accessor.md](../design/bus_accessor.md)
- [../design/data_io.md](../design/data_io.md)
- [../design/gpio.md](../design/gpio.md)
- [../design/i2c.md](../design/i2c.md)
- [../design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md)
