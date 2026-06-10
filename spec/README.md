# spec — 公開前提の確定仕様

`spec/` は **M5HAL の確定仕様** を読むための文書ルート。
確定した設計仕様のみを置き、 作業中の進捗・計画・提案は含めない。

## 役割

| パス | 役割 |
|---|---|
| `spec/` | 公開前提で残す文書の正本 |
| `docs/` | Doxygen 入力専用 (将来作成、 現在は未作成) |

## 読み始める順

全部を上から読む必要はない。目的別に必要な入口だけ読む。

| 目的 | 読むもの |
|---|---|
| v1 API を使う | [../README.md](../README.md) の v1 入口 + [`examples/v1/`](../examples/v1/) |
| v0 から v1 への考え方を知る | [style/migration.md](style/migration.md), [design/v0_v1_coexistence.md](design/v0_v1_coexistence.md) |
| Bus / Accessor の基本設計を知る | [design/bus_accessor.md](design/bus_accessor.md), [design/data_io.md](design/data_io.md), [design/transfer_desc.md](design/transfer_desc.md) |
| ストリームのフレーム化を知る | [design/frame.md](design/frame.md), [design/data_io.md](design/data_io.md) §Stream アダプタ |
| HAL 操作の bytecode 化を知る | [design/bytecode.md](design/bytecode.md) |
| I2C / SPI / UART を実装・レビューする | [design/i2c.md](design/i2c.md), [design/spi.md](design/spi.md), [design/uart.md](design/uart.md), [verification.md](verification.md) |
| GPIO / variant を実装・レビューする | [design/gpio.md](design/gpio.md), [design/variants.md](design/variants.md), [reference/directory-layout.md](reference/directory-layout.md) |
| ビルド時の挙動を設定する (`M5HAL_CONFIG_*`) | [design/configuration.md](design/configuration.md) |
| プロジェクト全体の方針を確認する | [goals.md](goals.md), [architecture.md](architecture.md) |

初見で迷った場合は、まず [goals.md](goals.md) → [architecture.md](architecture.md) →
[design/bus_accessor.md](design/bus_accessor.md) の順で読むと全体像を掴みやすい。

## ファイルマップ

| パス | 内容 |
|---|---|
| [goals.md](goals.md) | 上位方針、スコープ、成功条件 |
| [architecture.md](architecture.md) | 全体構造、層構成、配置原則 |
| [design/](design/) | kind / 機構ごとの確定仕様 |
| [reference/](reference/) | 配置規約などの補助リファレンス |
| [style/](style/) | コーディング規約、移行ガイド |
| [verification.md](verification.md) | 検証コマンドと運用方針 |

## 運用ルール

- `spec/` には **現行仕様として読む必要がある内容だけ**置く
- 履歴、検討経緯、巻き戻し理由は `spec/` の本文に書かない
