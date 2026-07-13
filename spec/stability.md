# stability — API 安定度と変更不可の範囲

> **読者**: 利用者・メンテナ・移植者。

M5HAL の各 API がどこまで変わりうるかを宣言する。 **段は spec 文書ごとに付く** (`design/*.md` 1 本 =
1 段)。 名前空間やディレクトリの階層とは無関係で、 段は属性である。

## 三段階

| 段 | 意味 |
|---|---|
| `experimental` | 契約が大きく変わりうる。 予告なく非互換変更が入る |
| `unstable` | 契約は固まりつつあるが、 まだ確定していない。 非互換変更はリリースノートに記載する |
| `stable` | **契約は変更不可**。 非互換変更を行わない |

> **`unstable` の意味**: **API 契約の形がまだ確定していない**ことを表す。 実装の品質・動作の
> 安定性とは**無関係**である。 実装が完成し実機受入済みであっても `unstable` でありうる。

**既定は `unstable`。** 段の宣言が無い spec 文書は `unstable` とみなす。 `stable` は**明示的な
宣言行為**であり、 黙って約束が発生することはない。

## 昇格

段は API ごとに独立して昇格する。 `1.1.0` で `spi` が `stable`、 `1.2.0` で `i2c` が `stable`、
という進み方を正とする。 降格は行わない。

| 遷移 | ゲート |
|---|---|
| `experimental` → `unstable` | 同一 kind に**異なる実装機構で 2 実装以上**。 実装機構は HW ペリフェラル / software bit-bang / host OS / remote proxy で数える (arduino と espidf はどちらも同じ ESP32 HW ペリフェラルを使うため 1 系統と数える) |
| `unstable` → `stable` | 明示的判断。 実機検証と protocol テストが揃い、 かつ契約に現れる最適化に実測の裏付けがあること |

**`1.x` における `stable` 宣言の性質**: `1.x` は移行期メジャーで semver の主体は v0 である
([goals.md](goals.md) §呼称と版数)。 v2 API を壊しても semver 上は破壊的変更にならない。 したがって
`1.x` 途中の `stable` 宣言は **semver が強制しないのに自ら課す約束**である。 `stable` からの降格は
semver イベントではなく信頼の毀損にあたるため、 昇格は慎重に行う。

## 現在の段

| 段 | spec 文書 |
|---|---|
| `experimental` | [design/frame.md](design/frame.md) / [design/bytecode.md](design/bytecode.md) / [design/remote.md](design/remote.md) |
| `unstable` | [design/bus_accessor.md](design/bus_accessor.md) / [design/configuration.md](design/configuration.md) / [design/data_io.md](design/data_io.md) / [design/errors.md](design/errors.md) / [design/gpio.md](design/gpio.md) / [design/i2c.md](design/i2c.md) / [design/i2c_slave.md](design/i2c_slave.md) / [design/i2s.md](design/i2s.md) / [design/memory.md](design/memory.md) / [design/runtime.md](design/runtime.md) / [design/service.md](design/service.md) / [design/spi.md](design/spi.md) / [design/transfer_desc.md](design/transfer_desc.md) / [design/uart.md](design/uart.md) / [design/v0_v2_coexistence.md](design/v0_v2_coexistence.md) / [design/variants.md](design/variants.md) |
| `stable` | (なし) |

`stable` が空なのは出発点であるためで、 方針ではない。 昇格ゲートを満たした API から順に
`1.x` の各リリースで宣言する。

## 段と独立に変更不可であるもの

以下は**相手が同時に更新できない**ため、 段に関わらず公開後は変更不可である。

- **エラー値** — [design/errors.md](design/errors.md) §ABI 契約 (改番・再利用禁止)
- **variant ID の値** — [design/variants.md](design/variants.md) §変更不可規約 (改番禁止、 廃止は欠番)
- **公開済みワイヤ形式** — `frame` / `remote` が `experimental` を脱した後の wire レイアウト

段が `unstable` であっても、 これらの値そのものは変わらない。 段が支配するのは **API の形**であり、
別バイナリと握る**値**ではない。

## 関連

- 版数と世代の呼称 → [goals.md](goals.md) §呼称と版数
- 2.0.0 への昇格条件 → [goals.md](goals.md) §成功条件 / 2.0.0 ゲート
- 検証レーン → [verification.md](verification.md)
