# porting_guide — M5HAL v2 variant ポーティングガイド

> **読者**: 新しい variant、HAL kind、chip capability を実装する実装者向け。

M5HALへ実装単位を追加する手順とチェックリストを示す。選択・申告の規範契約は
[../design/variants.md](../design/variants.md)、配置とinclude先は
[../reference/directory-layout.md](../reference/directory-layout.md)を参照。

## 前提知識

このガイドを読む前に、以下を把握しておく:

- [../architecture.md](../architecture.md) — 層構成と設計原則
- [../design/bus_accessor.md](../design/bus_accessor.md) — Bus / Accessor の責務分離
- [../design/data_io.md](../design/data_io.md) — Source / Sink のデータ流通
- [../design/transfer_desc.md](../design/transfer_desc.md) — per-call メタ情報

## 目次

| セクション | 内容 |
|---|---|
| [framework.md](framework.md) | framework variantの追加手順 (SPIを例題に)、HAL kind / chip capabilityの追加手順 |
