# goals — M5HAL v2 上位方針

> **読者**: 実装者・レビュー向け（設計仕様）。

この文書は M5HAL v2 の上位方針を示す。 具体的な構造は [architecture.md](architecture.md) を参照。

## 設計の目的 (不変)

組み込み向けの基礎ライブラリ (HAL) として、 **長期にわたり保守・拡張し続けられる**設計であること。
これは API 世代 (v0 / v2) をまたいで変わらない。

- リリース後に仕様が変わっても保守しやすい
- 新規コントリビュータが迷わず参入できる
- 実装に無駄がなく、 過度に複雑でない (性能・資源の要件は満たす)
- **特定のボード・製品に依存しない** — 市販ボードで試作したソフトウェアを、 自作基板へそのまま
  持ち込める

最後の項は「対応範囲の広さ」ではなく「依存の無さ」を指す。 サポート範囲は
§対応プラットフォーム が正本であり、 ボード非依存はそれを広げる約束ではない。

**安定性は「変更を止めること」ではなく「変更が安く安全であること」で買う。** 公開後に変更不可と
する対象は限定する — 相手が同時に更新できないもの (ワイヤ形式・公開 ID・エラー値) と、 `stable` と
宣言した API だけである ([stability.md](stability.md))。

この目的から導かれる決定則は [architecture.md](architecture.md) §設計原則 / §最適化の判断 が正本。

## v2 の目的

v2 は、 バス抽象 / エラーハンドリング / I/O モデル / プラットフォーム選択機構を本格設計の前提で再構成した API 世代である。

## 設計の柱

1. **マルチプラットフォーム** — Arduino / ESP-IDF どちらでも同じユーザーコードで動く。 コンパイル時のプラットフォーム選択機構を持つ ([design/variants.md](design/variants.md))
2. **チップ差の隔離** — ESP32 family を単一 platform variant として扱い、対象 chip ごとの差をvariant内へ閉じ込める
3. **バス抽象の本格化** — I2C / SPI / UART / GPIO を統一的に扱える Bus / Accessor モデルを提供する ([design/bus_accessor.md](design/bus_accessor.md))
4. **ゼロコピー指向の I/O モデル** — 大きなデータ転送で不要なバッファコピーを排除できる構造を持つ ([design/data_io.md](design/data_io.md))
5. **テスト可能性** — googletest によるユニットテストで契約を担保する
6. **M5Stack エコシステムとの整合** — M5UnitUnified などの上位ライブラリから利用しやすい API を保つ

## 呼称と版数

M5HAL は **API 世代** (spec の系統番号) と **リリース版数** (ライブラリの版数) を
分けて扱う。 **世代番号は、その世代が既定になるメジャー版数と一致させる**規則を
採る — v0 ⇔ `0.x`、 v2 ⇔ `2.x`。 `1.x` は両世代を同梱する移行期メジャーで、
既定は引き続き v0。 `v1` という API 世代は欠番である (世代番号とメジャー版数が
食い違う組み合わせを作らないよう、 意図的に飛ばしている)。

呼称と版管理の仕組みの詳細は [../README.md](../README.md) §API generations (v0 / v2) and release numbering と [design/v0_v2_coexistence.md](design/v0_v2_coexistence.md) §切替マクロ を参照。

## 成功条件 / 2.0.0 ゲート

`2.0.0` は default の inline namespace を v0 から v2 へ反転するリリースである。以下がすべて揃ってから
候補化を判断する:

1. **上位ライブラリ移行** — M5UnitUnified の Adapter 層が v2 API を採用し、代表 Unit が既存動作を維持している
2. **実機検証** — M5Stack BASIC (ESP32 1st) と CoreS3 (ESP32-S3) で v2 API 経由の I2C / SPI / UART / GPIO 基本動作が確認済み
3. **protocol テスト** — I2C / SPI / UART の主要 protocol semantic と edge case が native test または embedded test で固定されている
4. **API 安定度** — 既定にする v2 API の spec がすべて `stable` と宣言されている ([stability.md](stability.md))
5. **移行ガイド** — v0 から v2 への主要 API 対応と entry header の選び方が文書化されている
6. **下流互換性** — 主要下流ライブラリが v0 default のまま壊れず、v2 opt-in の代表 build も CI で通る

## 2.x の互換性契約

`2.0.0` 以降はdefaultのinline namespaceをv2とし、v2 APIをsemverの主体とする。
`2.x`ではv2 APIのbreaking changeを次のmajorまで行わない。`stable`宣言したcontractは、major更新でも
非互換変更しないという、より強い約束に従う。
v0 APIは`<M5HAL_v0.hpp>`経由で明示選択できる互換面として扱い、削除はmajor releaseでのみ行う。
API安定度の意味と宣言方法は[stability.md](stability.md)を参照する。

## スコープ

| 区分 | 内容 |
|---|---|
| **含める (v2)** | I2C / SPI / UART / I2S / PDM / GPIO のバス抽象、 Arduino / ESP-IDF / software / POSIX UART / remote framework variant、 ESP32 family platform variant、 リモートバス機構 ([design/remote.md](design/remote.md)、 UART/TCP transport、 push event / stream搬送)。I2S ([design/i2s.md](design/i2s.md)) はlocal TX (再生) / RX (録音) / full duplex、PDM ([design/pdm.md](design/pdm.md)) は対応SoC上の16-bit mono PCM RXを扱う |
| **現行スコープ外** | ディスプレイ描画、 音声/画像のドメインロジック (raw PCM の搬送はバス抽象として含む)、 上位ライブラリ機能 |

## 対応プラットフォーム

| カテゴリ | 内容 |
|---|---|
| **MCU platform variant** | ESP32 family (`esp32`, `esp32s2`, `esp32s3`, `esp32c2`, `esp32c3`, `esp32c5`, `esp32c6`, `esp32c61`, `esp32h2`, `esp32p4`) |
| **Framework** | allowlist対象のArduino core、 ESP-IDF、 software bit-bang、 POSIX UART、 remote proxy、 BSD socket TCP transport。Arduino coreと保証範囲は[design/variants.md](design/variants.md) §arduino variantの対応コアを正本とする |
| **Host (テスト用)** | native ビルド (macOS / Linux) |
