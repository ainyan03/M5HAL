# コンパイル時設定 (`M5HAL_CONFIG_*`)

> **読者**: 利用者向け。

M5HAL の挙動をビルド時に変えるユーザー設定ノブの一覧。
原則として project-wide な `-D` ビルドフラグで上書きし、M5HAL 自身の source TU と
consumer TU を含む関連する全 TU に同じ値を渡す。ヘッダ include より前の `#define` を使う場合も、
ライブラリの別 TU を含めた全関連 TU に同じ定義が見えるビルド構成に限る。

## ノブ一覧

| マクロ | 既定 | 値 | 効果 | 既定の定義位置 |
|---|---|---|---|---|
| `M5HAL_CONFIG_POSIX_UART` | `1` | `0`/`1` | `1`=POSIX host で termios serial を既定 UART provider として自動提供。 `0`=抑止 (host で UART kind を未提供へ戻す)。 **UART kind のみ** に作用し、 posix variant の runtime kind は影響を受けない ([runtime.md](runtime.md)) | `variants/frameworks/_checker.hpp` |
| `M5HAL_CONFIG_REMOTE_VARIANT` | `0` | `0`/`1` | `1` のとき remote variant を framework winner scan に参加させる (`M5HAL_FRAMEWORK_HAS_REMOTE`) | `variants/frameworks/_checker.hpp` |
| `M5HAL_CONFIG_VARIANT_RUNTIME` | `M5HAL_V2_VARIANT_ID_NONE` | 登録済み `M5HAL_V2_VARIANT_ID_*` | time providerを指定。`NONE`はoverrideなし。無効指定はcompile error | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_RUNTIME_MUTEX` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | runtime mutex providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_RUNTIME_TASK` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | runtime task providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_RUNTIME_EVENT` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | runtime event providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_GPIO` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | GPIO providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_I2C` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | I2C providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_SPI` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | SPI providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_I2S` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | I2S providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_PDM` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | PDM providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_VARIANT_UART` | `M5HAL_V2_VARIANT_ID_NONE` | 同上 | UART providerを指定。同上 | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_REMOTE_TCP_MAX_CONNECTIONS` | `2` | 整数 `>=1` | BSD TCP remote server が同時保持する connection slot 数と listen backlog。値を増やすと slot 配列の固定メモリも増える | `variants/frameworks/bsd/hal/remote/tcp_server.hpp` |
| `M5HAL_CONFIG_ESPIDF_I2C_MASTER_LEGACY_DRIVER` | `0` | `0`/`1` | ESP-IDF の I2C backend 選択。 `0`=新 bus-device driver (gen5)、 `1`=legacy command-link driver (gen4)。 legacy driver を既に使うプロジェクトは `1` で混在リンク abort を回避。legacy選択時はLP_I2Cをcontroller poolへ公開しない ([i2c.md](i2c.md) §ESP-IDF LP_I2C) | `variants/frameworks/espidf/detail/espidf_version.hpp` |
| `M5HAL_CONFIG_ESPIDF_I2C_SLAVE_IRAM_ISR` | `1` | `0`/`1` | `1`=ESP-IDF LL I2C slave ISR と到達コードを IRAM に配置し、flash cache 無効窓中も応答を維持。`0`=IRAM を回収するが、その窓中の slave 応答は保証しない ([i2c_slave.md](i2c_slave.md)) | `variants/frameworks/espidf/hal/i2c/slave.hpp` |
| `M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ` | ESP target: `1200000` / その他: `0` | `0` または `uint32_t` に収まる Hz の正整数 | I2C master SCL のフェールセーフ上限。超過要求をこの値へクランプし、`0` は上限無効 ([i2c.md](i2c.md)) | `hal/v2/i2c/master_clock_limit.hpp` |
| `M5HAL_CONFIG_ERROR_STRINGS` | `1` | `0`/`1` | `1`=`error::toString` がコード名の文字列テーブルを持つ。 `0`=テーブルを落とす (`toString` は常に `""`)。 容量が厳しいビルド向け ([errors.md](errors.md)) | `hal/v2/error.hpp` |
| `M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE_BYTES` | `256` | `>=4` かつ 4 の倍数 | 一時メモリプールの 1 block サイズ (byte) | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT` | `32` | `1`〜`32` | 一時メモリプールの block 数 (bitmap が `uint32_t` 1 個のため上限 32) | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_SERVICE_AUTORUN_CORE` | `TASK_CORE_ANY` (-1) | core id / `-1` (ANY) / `-2` (OPPOSITE) / `-3` (SAME) | auto-run タスクの core 配置 (`runtime::Task::start` の `core` 引数へ渡る)。 既定 = スケジューラ任せ — 相対 tick 契約 (コアドメインガード + gap-drop) がコア移動を安全にしており、 dual-core 実測でも SAME pin に退行しない ([service.md](service.md))。 runner を consumer と同一クロックドメインへ固定したい場合 (例: `ASSUME_PINNED` 併用) は core id か `-3` (SAME) を指定する。 単コア・host build には影響しない | `hal/v2/service/service.hpp` |
| `M5HAL_CONFIG_SERVICE_ASSUME_PINNED` | `0` | `0`/`1` | `1`=経過測定のコアドメインガード (パス毎の core-id 読み + 比較、数 cycles) を compile-time に外す。 **有効化してよい条件 = 「各 runner を default-clock 駆動する全タスクが同一 core で走る」または単コアビルド** (「全タスクがどこかに pin されている」だけでは不十分 — 別 core に pin された 2 駆動者が同一ドメイン扱いになる)。 既定 `0` (正しさ側、ガードは実測でスループットに現れない — [service.md](service.md)) | `hal/v2/service/service.hpp` |
| `M5HAL_CONFIG_DIAG` | `0` | `0`/`1` | `1`=`M5HAL_DIAG` イベントトレース有効。 `0`=呼び出しサイトは引数非評価の no-op | `hal/v2/diag.hpp` |

各ノブは値で読まれる (`0`/`1` フラグは値ベース)。
**`-D<マクロ>=0` は常に無効化として効く** (`#if defined(X)` 方式は `-D...=0` を黙って無視するため採用しない)。

variant selectorの`<KIND>`は `GPIO` / `I2C` / `SPI` / `I2S` / `PDM` / `UART` / `RUNTIME` /
`RUNTIME_MUTEX` / `RUNTIME_TASK` / `RUNTIME_EVENT` の10種。生の数値は使わず、
`variants/ids.hpp`の名前付きIDを指定する。各selectorは定義済みのC++整数定数式へ展開できなければならず、
名前付きIDの綴り誤りもcompile errorになる。remote providerを明示選択する場合、selectorはremote variantを
自動参加させないため、`M5HAL_CONFIG_REMOTE_VARIANT=1`も必要になる。逆にremoteをscanへ参加させるだけなら
selectorは不要で、overrideなしの走査順が適用される。これらは型aliasと別TU実装を一致させるため、M5HAL自身の
source TUを含む全関連TUへ同じ値を渡す。

例:

```bash
# host で POSIX UART を自動提供しない
-DM5HAL_CONFIG_POSIX_UART=0
# legacy driver を既に使う ESP-IDF プロジェクトで gen4 backend を選ぶ
-DM5HAL_CONFIG_ESPIDF_I2C_MASTER_LEGACY_DRIVER=1
# hostでremoteもscanしつつ、I2Cの既定providerだけsoftwareへ固定する
-DM5HAL_CONFIG_REMOTE_VARIANT=1
-DM5HAL_CONFIG_VARIANT_I2C=M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
# 一時プールを拡げる
-DM5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE_BYTES=512
```

### ESP-IDF component build での REQUIRES

IDF v5.3+ の I2C 依存として `esp_driver_i2c` と `driver` の両方を `REQUIRES` に含める。
既定 backend は `driver/i2c_master.h`、legacy backend は `driver/i2c.h` を公開
header から参照するため、`M5HAL_CONFIG_ESPIDF_I2C_MASTER_LEGACY_DRIVER` が compiler define として渡される場合でも
どちらの header も解決できる。C++ 側の feature selection は常に片方の backend だけを compile する。

## デバッグ診断 (`M5HAL_DEBUG_*`)

ライブラリ自体をデバッグするための診断入力。 **サポート対象外** (production 設定ではない)。
boolean switch は既定 off かつ値ベースで読み、marker pin のような dependent parameter は親 switch が
有効なときだけ参照する。通常はどのビルドでも有効化しない。

| マクロ | 既定 | 値 | 効果 |
|---|---|---|---|
| `M5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS` | `0` | `0`/`1` | `1`=ESP-IDF LL I2C slave の TX FIFO fill / RX FIFO drain / stretch ISR 処理中を GPIO パルスで示す。logic analyzer 相関用 |
| `M5HAL_DEBUG_ESPIDF_I2C_SLAVE_TX_FILL_MARKER_PIN` | `6` | GPIO 番号 | TX FIFO fill marker の出力 pin。`M5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS=1` のときのみ使用 |
| `M5HAL_DEBUG_ESPIDF_I2C_SLAVE_RX_DRAIN_MARKER_PIN` | `7` | GPIO 番号 | RX FIFO drain marker の出力 pin。`M5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS=1` のときのみ使用 |
| `M5HAL_DEBUG_ESPIDF_I2C_SLAVE_STRETCH_MARKER_PIN` | `13` | GPIO 番号 | stretch ISR marker の出力 pin。`M5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS=1` のときのみ使用 |
| `M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_TX_WATERMARK` | `0` | `0`/`1` | `1`=先回りの TX FIFO water-mark 補充を止め、TX_EMPTY への reactive 補充だけにする A/B 診断 |
| `M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_CONTROLLER_CLOCK` | `0` | `0`/`1` | `1`=I2C 機能 clock の有効化を省き、C6/H2 等の cold-boot failure を再現する A/B 診断 |

## バージョン/ABI切替

`M5HAL_V0_INLINE` / `M5HAL_V2_INLINE` は `m5::hal::vN` を `inline namespace` にするかを選ぶ
**バージョン共存/ABI** の切替であり、挙動configではない。詳細は
[v0_v2_coexistence.md](v0_v2_coexistence.md)。

新しいmacroを追加する実装者向けの分類・命名規約は
[coding_style.md](../style/coding_style.md) §マクロ を参照する。本ページは利用者が指定できる
`M5HAL_CONFIG_*`と、サポート対象外の`M5HAL_DEBUG_*`のカタログだけを正本とする。
