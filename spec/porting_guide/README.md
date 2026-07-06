# porting_guide — M5HAL v2 variant ポーティングガイド

> **読者**: 新しい variant (framework / platform) を実装する実装者向け。

M5HAL に新しいバックエンドを追加する際の手順・契約・注意点を示す。
設計の背景は [../design/variants.md](../design/variants.md) を参照。

## 前提知識

このガイドを読む前に、以下を把握しておく:

- [../architecture.md](../architecture.md) — 層構成と設計原則
- [../design/bus_accessor.md](../design/bus_accessor.md) — Bus / Accessor の責務分離
- [../design/data_io.md](../design/data_io.md) — Source / Sink のデータ流通
- [../design/transfer_desc.md](../design/transfer_desc.md) — per-call メタ情報

## 目次

| セクション | 内容 |
|---|---|
| [framework.md](framework.md) | framework variant の追加手順 (SPI を例題に) |

将来追加予定:

- `platform.md` — platform variant の追加手順
- `i2c.md` — I2C variant 固有の契約 (probe path, restart, gen4/gen5 互換)
- `gpio.md` — GPIO variant 固有の契約 (Port / GPIOGroup / slot register)
- `runtime.md` — runtime variant 固有の契約 (early scan, using namespace 注入)
