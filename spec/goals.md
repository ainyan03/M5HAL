# goals — M5HAL v1 上位方針

この文書は M5HAL v1 の上位方針を示す。 具体的な構造は [architecture.md](architecture.md) を参照。

## 呼称と版数

| 呼称 | 意味 |
|---|---|
| **v0** | 旧 API 世代 (仕様系統) |
| **v1** | 新 API 世代 (仕様系統) |
| **v0.2** | v0 と v1 を同梱する移行期間のライブラリリリース系列 |

メジャー版数が `0` の間 (`v0.x.y`) は **v0 が既定動作**で、 既存利用者はコード変更なしで新リリースを受け取れる。 `v0.2` リリースからは、 明示的に opt-in した利用者が v1 API を利用できる。 将来メジャー版数が `1` (`v1.x.y`) に上がるタイミングで、 v1 が既定動作に切り替わる。

### v1.0.0 リリース条件

`v1.0.0` は default の inline namespace を v0 から v1 へ反転するリリースである。候補化は、少なくとも以下が揃ってから判断する:

1. **上位ライブラリ移行** — M5UnitUnified の Adapter 層が v1 API を採用し、代表 Unit が既存動作を維持している
2. **実機検証** — M5Stack BASIC と CoreS3 で v1 API 経由の I2C / SPI / UART / GPIO 基本動作が確認済み
3. **protocol テスト** — I2C / SPI / UART の主要 protocol semantic と edge case が native test または embedded test で固定されている
4. **API 安定期間** — `m5::hal::v1::*` の公開 API に利用者影響のある breaking change が一定期間発生していない
5. **移行ガイド** — v0 から v1 への主要 API 対応と entry header の選び方が文書化されている
6. **下流互換性** — 主要下流ライブラリが v0 default のまま壊れず、v1 opt-in の代表 build も通る

`v1.x.y` 系列では v1 API を semver の主体とし、breaking change は次の major まで避ける。v0 API は deprecation 段階として `<M5HAL_v0.hpp>` 経由で維持し、削除する場合も別 major と十分な移行期間を置く。

## v1 の目的

v1 は、 バス抽象 / エラーハンドリング / I/O モデル / プラットフォーム選択機構を本格設計の前提で再構成した API 世代である。

## 設計の柱

1. **マルチプラットフォーム** — Arduino / ESP-IDF どちらでも同じユーザーコードで動く。 コンパイル時のプラットフォーム選択機構を持つ ([design/variants.md](design/variants.md))
2. **対応チップの段階的拡張** — ESP32 family を単一 platform variant として扱い、 公式 ESP-IDF component build で対象 chip を継続検証する
3. **バス抽象の本格化** — I2C / SPI / UART / GPIO を統一的に扱える Bus / Accessor モデルを提供する ([design/bus_accessor.md](design/bus_accessor.md))
4. **ゼロコピー指向の I/O モデル** — 大きなデータ転送で不要なバッファコピーを排除できる構造を持つ ([design/data_io.md](design/data_io.md))
5. **テスト可能性** — googletest によるユニットテストで契約を担保する
6. **M5Stack エコシステムとの整合** — M5UnitUnified などの上位ライブラリから利用しやすい API を保つ

## スコープ

| 区分 | 内容 |
|---|---|
| **含める (現行 v1)** | I2C / SPI / UART / I2S / GPIO のバス抽象、 Arduino / ESP-IDF / software / POSIX UART の framework variant、 ESP32 family platform variant、 リモートバス機構 ([design/remote.md](design/remote.md)、 push イベントまで導入済み)。 I2S ([design/i2s.md](design/i2s.md)) は段階導入: 初回はローカル再生 = master TX のみ |
| **将来含める** | TCP / UDP / Wi-Fi 等の通信バス、 Zephyr framework variant、 リモートバス機構の拡張 (flow control / マルチノード)、 I2S の RX (録音) |
| **含めない (少なくとも初期は)** | ディスプレイ描画、 音声/画像のドメインロジック (raw PCM の搬送はバス抽象として含む)、 上位ライブラリ機能 |

## 成功条件

v1 が既定動作になる時点で、 以下が達成できていること:

- M5Stack BASIC (ESP32 1st) と CoreS3 (ESP32-S3) で I2C / SPI / UART / GPIO の基本動作が確認できる
- M5UnitUnified が v1 API を経由して既存 Unit を動作させられる
- v0 API からの主要 API について v1 API への移行ガイドが整備されている
- googletest による native ビルドのテストが CI で通る
- Arduino / ESP-IDF 両方のビルドチェックが CI で通る

## 対応プラットフォーム

| カテゴリ | 内容 |
|---|---|
| **MCU** | ESP32 family (`esp32`, `esp32s2`, `esp32s3`, `esp32c2`, `esp32c3`, `esp32c5`, `esp32c6`, `esp32c61`, `esp32h2`, `esp32p4`) |
| **Framework** | Arduino (Arduino-ESP32)、 ESP-IDF、 software bit-bang、 POSIX UART |
| **Host (テスト用)** | native ビルド (macOS / Linux) |
