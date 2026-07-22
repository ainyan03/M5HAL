# style/migration — v0 API → v2 API 移行ガイド

> **読者**: 利用者向け。

v0 API 利用者が v2 API に移行する際の指針を示す。
旧v2 Bus取得API・`begin/endTransaction`から現行v2への移行は
[legacy_v2_migration.md](legacy_v2_migration.md)を参照する。

## 基本方針

- v0 対応 target で既存コードをそのまま使い続ける場合は `<M5HAL.hpp>` または `<M5HAL_v0.hpp>` を使う。非 ESP32 Arduino target では v0 entry は利用できないため、`<M5HAL_v2.hpp>` を使う
- v2 API を使う場合は `<M5HAL_v2.hpp>` を使う
- v0 対応 target では v0 と v2 が同一ライブラリ内で共存し、 **同一 translation unit での両エントリ include も可能** (移行途中のファイル等。 [design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §エントリヘッダ)。 ただし可読性のため、 通常は TU ごとに使う世代を明示する
- v2への移行では、対応表の「再構成」を単純なsymbol置換として扱わない

## ヘッダ選択

詳細は [design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §エントリヘッダ を参照。

## 移行分類と対応表

分類は旧symbolがv2に同名で残るかではなく、**利用者が行う移行作業**で決める。

| 区分 | 意味 |
|---|---|
| **保持** | 呼出し側の役割と基本契約を維持する。世代namespaceやmacro prefixの機械的変更は許容 |
| **再構成** | use caseには移行先があるが、所有権・責務・呼出し形を組み替える。drop-in置換ではない |
| **廃止** | v2に対応概念がなく、移行時に削除または上位設計へ吸収する |

この表をv0→v2の利用者作業の正本とする。設計契約は各design文書を参照する。

| 分類 | v0の考え方 / API | v2の考え方 / API | 移行上の要点 |
|---|---|---|---|
| 保持 | `error::error_t` / `isError` / `isOk` | 同名 (`m5::hal::v2::error`) | cross-cuttingな結果型・判定の役割を維持 |
| 保持 | `types::GpioMode` / `gpio_number_t` | 同名 (`m5::hal::v2::types`) | pin指定の値とmode名を維持 |
| 保持 | framework/platform検出macro | `M5HAL_FRAMEWORK_HAS_*` / `M5HAL_V2_DETECTED_PLATFORM_VARIANT_*` | platform出力だけ世代分離prefixを付ける (無印はv0が所有) |
| 再構成 | `Bus` / `BusConfig` / `AccessConfig`と旧chain操作 | 同じ役割の型 + `transfer`を核とする操作 | 型の役割は残るがcall chainは互換でない |
| 再構成 | `beginAccess` / `endAccess` chain virtual、`Bus::beginAccess()` factory | 利用者が`Accessor`を直接構築し、access window + sugarを使う | BusはAccessorを所有しない。[bus_accessor.md](../design/bus_accessor.md) §Busの保持 |
| 再構成 | `interface::io::Input` / `Output` | `data::Source` / `Sink` | directionとcursor契約を明示。[data_io.md](../design/data_io.md) §向きの規約 |
| 再構成 | 旧I2C操作 / software I2C singleton | `write` / `read` / register sugar / `probe` + software variant | instanceとbackend選択を分離。[i2c.md](../design/i2c.md) |
| 再構成 | `interface::gpio::*` | `IGPIO` / `IPort` / `Pin` / `GPIOGroup` | global pin番号とport/groupをv2所有モデルへ移す。[gpio.md](../design/gpio.md) |
| 再構成 | `types::BusType` (`bus_type_t`) | `types::BusKind` (`bus_kind_t`) | kind識別へ名称を統一。[glossary.md](glossary.md) |
| 廃止 | `types::PeripheralType` | 対応物なし | v0/platform固有概念。必要な資源選択は各BusConfig / controller policyへ吸収 |

## 移行時の確認項目

1. include しているヘッダが v0 か v2 かを明示する (詳細は [design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §エントリヘッダ)
2. 同一 TU で v0 / v2 を併用する場合は [design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md) §同一 TU 安全性の保証 (coexist fence) の前提を満たす
3. 旧 I/O 抽象を `Source` / `Sink` に置き換える (詳細は [design/data_io.md](../design/data_io.md) §向き (direction) の規約)
4. 旧 I2C 操作を `transfer` または v2 sugar に置き換える (詳細は [design/i2c.md](../design/i2c.md))
5. 旧 GPIO 抽象を `Pin` / `IPort` / `GPIOGroup` ベースへ置き換える (詳細は [design/gpio.md](../design/gpio.md))

## 関連

- [../design/bus_accessor.md](../design/bus_accessor.md)
- [../design/bus_capabilities.md](../design/bus_capabilities.md)
- [../design/data_io.md](../design/data_io.md)
- [../design/gpio.md](../design/gpio.md)
- [../design/i2c.md](../design/i2c.md)
- [../design/v0_v2_coexistence.md](../design/v0_v2_coexistence.md)
