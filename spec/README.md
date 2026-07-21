# spec — 公開前提の確定仕様

> **読者**: メンテナ向け（ナビゲーション）。

> 使うだけなら spec/ を読む必要はありません — [../README.md](../README.md) と [examples/v2/](../examples/v2/) を参照。

`spec/` は **M5HAL の確定仕様** を読むための文書ルート。
確定した設計仕様のみを置き、 作業中の進捗・計画・提案は含めない。

## 目的別ルーター

全部を上から読む必要はない。目的に合った入口だけ読む。

| 目的 | 読むもの |
|---|---|
| v2 API を使う | [../README.md](../README.md) の v2 入口 + [`examples/v2/`](../examples/v2/) |
| v0からv2、または旧v2 lifecycleから現行へ移行する | [style/migration.md](style/migration.md), [style/accessor_lifecycle_migration.md](style/accessor_lifecycle_migration.md), [design/v0_v2_coexistence.md](design/v0_v2_coexistence.md) |
| Bus / Accessor とinstance capabilityの基本設計を知る | [design/bus_accessor.md](design/bus_accessor.md), [design/bus_capabilities.md](design/bus_capabilities.md), [design/data_io.md](design/data_io.md), [design/transfer_desc.md](design/transfer_desc.md) |
| ストリームのフレーム化を知る | [design/frame.md](design/frame.md), [design/data_io.md](design/data_io.md) §Stream アダプタ |
| HAL 操作の bytecode 化を知る | [design/bytecode.md](design/bytecode.md) |
| リモートバス機構を知る | [design/remote.md](design/remote.md) (下層: frame / bytecode / data_io) |
| I2C / I2C slave / SPI / UART / I2S / PDM を実装・レビューする | [design/i2c.md](design/i2c.md), [design/i2c_slave.md](design/i2c_slave.md), [design/slave_queue.md](design/slave_queue.md), [design/spi.md](design/spi.md), [design/uart.md](design/uart.md), [design/i2s.md](design/i2s.md), [design/pdm.md](design/pdm.md), [verification.md](verification.md) |
| 新しい variant を追加する (ポーティング) | [porting_guide/](porting_guide/README.md) (手順レシピ), [design/variants.md](design/variants.md) (設計仕様) |
| GPIO / variant を実装・レビューする | [design/gpio.md](design/gpio.md), [design/variants.md](design/variants.md), [reference/directory-layout.md](reference/directory-layout.md) |
| runtime 設備 (time / mutex / task) と Bus 排他の意味論を知る | [design/runtime.md](design/runtime.md), [design/bus_accessor.md](design/bus_accessor.md) §排他制御の意味論 |
| バックグラウンド実行 (auto-run) の並行性契約を知る | [design/service.md](design/service.md) |
| ビルド時の挙動を設定する (`M5HAL_CONFIG_*`) | [design/configuration.md](design/configuration.md) |
| エラーコードの意味と対処を調べる | [design/errors.md](design/errors.md) |
| 一時メモリpoolとallocatorを実装・レビューする | [design/memory.md](design/memory.md) |
| ディレクトリ・名前空間の 1:1 規約を確認する | [reference/directory-layout.md](reference/directory-layout.md) |
| プロジェクト全体の方針を確認する | [goals.md](goals.md), [architecture.md](architecture.md) |

初見で迷った場合は、まず [goals.md](goals.md) → [architecture.md](architecture.md) →
[design/bus_accessor.md](design/bus_accessor.md) の順で読むと全体像を掴みやすい。

## 主な読者と文書の役割

冒頭の `> **読者**:` は、その文書を最初に読むべき主な読者を示す。複数の読者が参照する文書でも、
主役を一つに定め、別の役割の説明は正本へのリンクに留める。

- **利用者向け** — ライブラリを使う人が読む集合: [../README.md](../README.md) と
  [`examples/v2/`](../examples/v2/) を起点に、必要に応じて
  [design/configuration.md](design/configuration.md) / [design/errors.md](design/errors.md) /
  [style/migration.md](style/migration.md)
- **実装者・レビュー向け（設計仕様）** — 現行contract、層境界、非自明な設計理由を読む集合:
  [goals.md](goals.md), [architecture.md](architecture.md), [design/](design/) の各設計文書
- **メンテナ向け（手順・規約）** — 実装手順、配置・記述規約、検証入口を読む集合:
  [porting_guide/](porting_guide/), [style/coding_style.md](style/coding_style.md),
  [verification.md](verification.md), [reference/directory-layout.md](reference/directory-layout.md)

設計文書は「なぜこの形か・何を不採用にしたか」を**根拠節**
(`## 採用しない要素` / `## なぜ〜か`) に現在形で持つ (→ 運用ルール)。

## ファイルマップ

| パス | 内容 |
|---|---|
| [goals.md](goals.md) | 上位方針、スコープ、版数の契約 |
| [architecture.md](architecture.md) | 全体構造、層構成、配置原則 |
| [stability.md](stability.md) | API 安定度 (experimental / unstable / stable) と変更不可の範囲 |
| [design/](design/) | kind / 機構ごとの確定仕様 |
| [reference/directory-layout.md](reference/directory-layout.md) | 配置規約などの補助リファレンス |
| [porting_guide/](porting_guide/) | variant 追加のレシピ (framework / platform) |
| [style/](style/) | コーディング規約、移行ガイド |
| [verification.md](verification.md) | 検証コマンドと運用方針 |
| (`docs/` は Doxygen 入力用に予約、現行未作成) | |

## 運用ルール

- `spec/` には **現行仕様として読む必要がある内容だけ**置く
- 現在未対応であることが利用者・実装者の判断に必要なら、期限や採用を約束せず
  **現行スコープ外**として記す。未採用の機能候補、実装順、検証拡充予定は `spec/` に置かない
- 将来の実装でも破ってはならない互換性予約、層境界、拡張規約は現行仕様として残す。
  これは機能の採用予定ではなく、拡張時にも守る契約である
- [stability.md](stability.md) の API 安定度は、現行APIを変更できる範囲を表す。
  未実装機能の採用状況やroadmapとは別の軸として扱う
- **時系列・人名・検討プロセス** (いつ誰がどう決めたか、巻き戻しの顛末) は `spec/` の
  本文に書かない
- ただし**現在形で書ける設計判断の根拠** (なぜこの形か・何を不採用にしたか) は仕様の
  一部であり、各設計文書の**根拠節**に数行で置く。節名は内容に合わせる —
  不採用案の要約なら `## 採用しない要素`、形の理由なら `## なぜ〜か`。判断が改定されたら
  節を in-place で書き換える (訂正の追記を積み増さない)
