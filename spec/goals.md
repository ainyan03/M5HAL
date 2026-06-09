# goals — M5HAL v1 上位方針

この文書は M5HAL v1 の上位方針を示す。 具体的な構造は [architecture.md](architecture.md) を参照。

## 呼称と版数

| 呼称 | 意味 |
|---|---|
| **v0** | 旧 API 世代 (仕様系統) |
| **v1** | 新 API 世代 (仕様系統) |
| **v0.2** | v0 と v1 を同梱する移行期間のライブラリリリース系列 |

メジャー版数が `0` の間 (`v0.x.y`) は **v0 が既定動作**で、 既存利用者はコード変更なしで新リリースを受け取れる。 `v0.2` リリースからは、 明示的に opt-in した利用者が v1 API を利用できる。 将来メジャー版数が `1` (`v1.x.y`) に上がるタイミングで、 v1 が既定動作に切り替わる。

## v1 の目的

v1 は、 バス抽象 / エラーハンドリング / I/O モデル / プラットフォーム選択機構を本格設計の前提で再構成した API 世代である。

## 設計の柱

1. **マルチプラットフォーム** — Arduino / ESP-IDF どちらでも同じユーザーコードで動く。 コンパイル時のプラットフォーム選択機構を持つ ([design/variants.md](design/variants.md))
2. **対応チップの段階的拡張** — まず ESP32(1st) と ESP32-S3 を優先し、 後続で C6 / P4 / その他へ展開する
3. **バス抽象の本格化** — I2C / SPI / UART / GPIO を統一的に扱える Bus / Accessor モデルを提供する ([design/bus_accessor.md](design/bus_accessor.md))
4. **ゼロコピー指向の I/O モデル** — 大きなデータ転送で不要なバッファコピーを排除できる構造を持つ ([design/data_io.md](design/data_io.md))
5. **テスト可能性** — googletest によるユニットテストで契約を担保する
6. **M5Stack エコシステムとの整合** — M5UnitUnified などの上位ライブラリから利用しやすい API を保つ

## スコープ

| 区分 | 内容 |
|---|---|
| **含める (v1 初期)** | I2C / SPI / UART / GPIO のバス抽象、 Arduino / ESP-IDF の variant、 ESP32(1st) / S3 のプラットフォーム対応 |
| **将来含める** | ESP32 C6 / P4、 software bit-bang variant、 TCP / UDP / Wi-Fi 等の通信バス、 Zephyr framework variant |
| **含めない (少なくとも初期は)** | ディスプレイ描画、 音声/画像のドメインロジック、 リモートバス機構、 上位ライブラリ機能 |

## 成功条件

v1 が既定動作になる時点で、 以下が達成できていること:

- M5Stack BASIC (ESP32 1st) と CoreS3 (ESP32-S3) で I2C / SPI / GPIO の基本動作が確認できる
- M5UnitUnified が v1 API を経由して既存 Unit を動作させられる
- v0 API からの主要 API について v1 API への移行ガイドが整備されている
- googletest による native ビルドのテストが CI で通る
- Arduino / ESP-IDF 両方のビルドチェックが CI で通る

## 対応プラットフォーム

| カテゴリ | 内容 |
|---|---|
| **MCU** | ESP32 (1st gen) → ESP32-S3 → 後続で C6 / P4 |
| **Framework** | Arduino (Arduino-ESP32) と ESP-IDF |
| **Host (テスト用)** | native ビルド (macOS / Linux) |
