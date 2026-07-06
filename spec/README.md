# spec — 公開前提の確定仕様

> **読者**: ナビゲーション。

> 使うだけなら spec/ を読む必要はありません — [../README.md](../README.md) と [examples/v2/](../examples/v2/) を参照。

`spec/` は **M5HAL の確定仕様** を読むための文書ルート。
確定した設計仕様のみを置き、 作業中の進捗・計画・提案は含めない。

## 目的別ルーター

全部を上から読む必要はない。目的に合った入口だけ読む。

| 目的 | 読むもの |
|---|---|
| v2 API を使う | [../README.md](../README.md) の v2 入口 + [`examples/v2/`](../examples/v2/) |
| v0 から v2 への考え方を知る | [style/migration.md](style/migration.md), [design/v0_v2_coexistence.md](design/v0_v2_coexistence.md) |
| Bus / Accessor の基本設計を知る | [design/bus_accessor.md](design/bus_accessor.md), [design/data_io.md](design/data_io.md), [design/transfer_desc.md](design/transfer_desc.md) |
| ストリームのフレーム化を知る | [design/frame.md](design/frame.md), [design/data_io.md](design/data_io.md) §Stream アダプタ |
| HAL 操作の bytecode 化を知る | [design/bytecode.md](design/bytecode.md) |
| リモートバス機構を知る | [design/remote.md](design/remote.md) (下層: frame / bytecode / data_io) |
| I2C / SPI / UART / I2S を実装・レビューする | [design/i2c.md](design/i2c.md), [design/spi.md](design/spi.md), [design/uart.md](design/uart.md), [design/i2s.md](design/i2s.md), [verification.md](verification.md) |
| 新しい variant を追加する (ポーティング) | [porting_guide/](porting_guide/README.md) (手順レシピ), [design/variants.md](design/variants.md) (設計仕様) |
| GPIO / variant を実装・レビューする | [design/gpio.md](design/gpio.md), [design/variants.md](design/variants.md), [reference/directory-layout.md](reference/directory-layout.md) |
| runtime 設備 (time / mutex) と Bus 排他の意味論を知る | [design/runtime.md](design/runtime.md), [design/bus_accessor.md](design/bus_accessor.md) §排他制御の意味論 |
| ビルド時の挙動を設定する (`M5HAL_CONFIG_*`) | [design/configuration.md](design/configuration.md) |
| エラーコードの意味と対処を調べる | [design/errors.md](design/errors.md) |
| ディレクトリ・名前空間の 1:1 規約を確認する | [reference/directory-layout.md](reference/directory-layout.md) |
| プロジェクト全体の方針を確認する | [goals.md](goals.md), [architecture.md](architecture.md) |

初見で迷った場合は、まず [goals.md](goals.md) → [architecture.md](architecture.md) →
[design/bus_accessor.md](design/bus_accessor.md) の順で読むと全体像を掴みやすい。

## 読者の枠 (利用者 / メンテナ)

文書は読者軸で 2 つの枠に分かれる。

- **利用者向け** — ライブラリを使う人が読む集合: [../README.md](../README.md) と
  [`examples/v2/`](../examples/v2/) を起点に、必要に応じて
  [design/configuration.md](design/configuration.md) / [design/errors.md](design/errors.md) /
  [style/migration.md](style/migration.md)
- **メンテナ・コントリビュータ向け** — 実装・レビュー・ポーティングをする人が読む集合:
  [goals.md](goals.md), [architecture.md](architecture.md), [design/](design/) の各設計文書,
  [porting_guide/](porting_guide/), [style/coding_style.md](style/coding_style.md),
  [verification.md](verification.md), [reference/directory-layout.md](reference/directory-layout.md)

各文書は冒頭の `> **読者**:` ラベルで枠を明示する。ラベルは 3 種 — 「利用者向け」/
「実装者・レビュー向け（設計仕様）」/「メンテナ向け（ビルド・運用・規約）」で、後 2 者が
メンテナ・コントリビュータ枠に属する。メンテナ枠の設計文書は「なぜこの形か・何を不採用に
したか」を **§設計判断** 節に現在形で持つ (→ 運用ルール)。

## ファイルマップ

| パス | 内容 |
|---|---|
| [goals.md](goals.md) | 上位方針、スコープ、成功条件 |
| [architecture.md](architecture.md) | 全体構造、層構成、配置原則 |
| [design/](design/) | kind / 機構ごとの確定仕様 |
| [reference/directory-layout.md](reference/directory-layout.md) | 配置規約などの補助リファレンス |
| [porting_guide/](porting_guide/) | variant 追加のレシピ (framework / platform) |
| [style/](style/) | コーディング規約、移行ガイド |
| [verification.md](verification.md) | 検証コマンドと運用方針 |
| (`docs/` は Doxygen 入力専用、将来作成予定) | |

## 運用ルール

- `spec/` には **現行仕様として読む必要がある内容だけ**置く
- **時系列・人名・検討プロセス** (いつ誰がどう決めたか、巻き戻しの顛末) は `spec/` の
  本文に書かない
- ただし**現在形で書ける設計判断の根拠** (なぜこの形か・何を不採用にしたか) は仕様の
  一部であり、各設計文書の **§設計判断** 節に数行で置く。判断が改定されたら節を
  in-place で書き換える (訂正の追記を積み増さない)
