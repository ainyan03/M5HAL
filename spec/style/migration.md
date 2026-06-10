# style/migration — v0 API → v1 API 移行ガイド

v0 API 利用者が v1 API に移行する際の指針を示す。

## 基本方針

- 既存コードをそのまま使い続ける場合は `<M5HAL.hpp>` または `<M5HAL_v0.hpp>` を使う
- v1 API を使う場合は `<M5HAL_v1.hpp>` を使う
- v0 と v1 は同一ライブラリ内で共存し、 **同一 translation unit での両エントリ include も可能** (移行途中のファイル等。 [design/v0_v1_coexistence.md](../design/v0_v1_coexistence.md) §エントリヘッダ)。 ただし可読性のため、 通常は TU ごとに使う世代を明示する
- v1 への移行は、 旧 API の置き換えではなく **新しい API 体系への移行** として扱う

## ヘッダ選択

| ヘッダ | 用途 |
|---|---|
| `<M5HAL.hpp>` | 後方互換 shim |
| `<M5HAL_v0.hpp>` | 明示的に v0 を使う |
| `<M5HAL_v1.hpp>` | 明示的に v1 を使う |

## 移行の考え方

各 API は次の 3 区分で考える。

| 区分 | 意味 |
|---|---|
| **保持** | 命名・役割をほぼ維持して使える |
| **再構成** | 概念は残るが、 使い方や責務が変わる |
| **廃止** | v1 では使わない |

## 維持される要素

| API | 配置 | 備考 |
|---|---|---|
| `error::error_t` / `error::isError` / `error::isOk` | `hal/error.hpp` | cross-cutting な型 |
| `types::PeripheralType` / `types::BusType` / `types::GpioMode` | `hal/types.hpp` | 命名維持 |
| `types::gpio_number_t` | `hal/types.hpp` | pin 指定の基本型 |
| `M5HAL_V1_TARGET_PLATFORM_*` / `M5HAL_FRAMEWORK_HAS_*` | 各 `_checker.hpp` | variant 機構で利用。 platform 系は世代分離のため `M5HAL_V1_` プレフィックス (無印は v0 が所有) |

## 再構成される要素

| テーマ | v1 での扱い |
|---|---|
| Bus 抽象 | `Bus` / `BusConfig` / `AccessConfig` を維持しつつ、 `transfer` を核に再構成 |
| Accessor 抽象 | `beginAccess` / `endAccess` と sugar を中心に再構成 |
| GPIO 抽象 | `IGPIO` / `IPort` / `Pin` / `GPIOGroup` に整理 |
| I2C register access | `writeRegister` / `readRegister` / `probe` に整理 |

## v1 で使わない要素

| API | 備考 |
|---|---|
| `interface::io::Input` / `Output` 系 | `Source` / `Sink` へ置換 |
| 旧 `bus::Accessor` chain virtual | `transfer` ベースへ置換 |
| `Bus::beginAccess(AccessConfig&)` factory | 利用者が `Accessor` を直接構築 |
| 旧 software I2C singleton 群 | software variant に置換 |
| 旧 `interface::gpio::*` 抽象 | v1 GPIO 抽象へ置換 |

## 主要な読み替え

| v0 の考え方 | v1 の考え方 |
|---|---|
| バス操作は旧 chain API で行う | `transfer` を核に行う |
| I/O 抽象は `Input` / `Output` | `Source` / `Sink` |
| GPIO は旧 interface 系抽象 | `IGPIO` / `IPort` / `Pin` / `GPIOGroup` |
| 単発 I2C 操作は独自 sugar 群 | `write` / `read` / `writeRegister` / `readRegister` / `probe` |

## 移行時の確認項目

1. include しているヘッダが v0 か v1 かを明示する
2. 同一 TU で v0 / v1 を併用する場合は `design/v0_v1_coexistence.md` の前提を満たす
3. 旧 I/O 抽象を `Source` / `Sink` に置き換える
4. 旧 I2C 操作を `transfer` または v1 sugar に置き換える
5. 旧 GPIO 抽象を `Pin` / `IPort` / `GPIOGroup` ベースへ置き換える

## 関連

- [../design/bus_accessor.md](../design/bus_accessor.md)
- [../design/data_io.md](../design/data_io.md)
- [../design/gpio.md](../design/gpio.md)
- [../design/i2c.md](../design/i2c.md)
- [../design/v0_v1_coexistence.md](../design/v0_v1_coexistence.md)
