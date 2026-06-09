# コンパイル時設定 (`M5HAL_CONFIG_*`)

M5HAL の挙動をビルド時に変えるユーザー設定ノブの一覧と命名規約。
すべて `-D` ビルドフラグ、 または M5HAL のヘッダ include より前の `#define` で上書きできる。

## 命名規約

ライブラリの挙動を変えるユーザー設定マクロは `M5HAL_CONFIG_<領域>_<機能>` で命名する。
規約は次の4点。

1. **接頭辞 `M5HAL_CONFIG_` は「ユーザーが定義してよい入力」を意味する。**
   ライブラリが内部で算出する派生マクロ (`M5HAL_FRAMEWORK_HAS_*` / `M5HAL_VARIANT_*` /
   `M5HAL_ESPIDF_I2C_HAS_*` 等) はこの接頭辞を使わない。 名前を見れば入力か出力かが分かる。
2. **値ベース。 定義の有無では判定しない。** 各ノブは `#ifndef`＋既定値で定義され、
   **値** で読まれる (フラグは `0`/`1`)。 したがって `-D...=0` は常に無効化として効く。
   (`#if defined(X)` 方式は `=0` を黙って無視するため採用しない。)
3. **名前に極性動詞・否定を入れない** (`USE_` / `DISABLE_` / `NO_` を使わない)。
   正の名詞で表し、 既定値が極性を担う。
4. 既定値は **それを読むサブシステムの隣に co-locate** する (コードと既定の乖離防止)。
   横断的なものだけ `src/m5_hal_config.hpp` に置く。 本ページがその単一カタログ。

挙動に影響するマクロは3種に分かれ、 接頭辞で見分ける: **`M5HAL_CONFIG_*`** = サポートする
設定 (本ページ) / **`M5HAL_DEBUG_*`** = ライブラリのデバッグ用診断トグル (非サポート・既定 off・
下記「デバッグ診断」節) / **無印** = ライブラリが算出する内部派生 (ユーザーは定義しない)。

## ノブ一覧

| マクロ | 既定 | 値 | 効果 | 既定の定義位置 |
|---|---|---|---|---|
| `M5HAL_CONFIG_POSIX_UART` | `1` | `0`/`1` | `1`=POSIX host で termios serial を既定 UART provider として自動提供。 `0`=抑止 (host で UART を未提供へ戻す) | `variants/frameworks/_checker.hpp` |
| `M5HAL_CONFIG_IDF_I2C_LEGACY` | `0` | `0`/`1` | ESP-IDF の I2C backend 選択。 `0`=新 bus-device driver (gen5)、 `1`=legacy command-link driver (gen4)。 legacy driver を既に使うプロジェクトは `1` で混在リンク abort を回避 | `variants/frameworks/espidf/detail/espidf_version.hpp` |
| `M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE` | `256` | `>=4` かつ 4 の倍数 | 一時メモリプールの 1 block サイズ (byte) | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT` | `32` | `1`〜`32` | 一時メモリプールの block 数 (bitmap が `uint32_t` 1 個のため上限 32) | `src/m5_hal_config.hpp` |
| `M5HAL_CONFIG_SOFTWARE_I2C_YIELD_PROBE_SPINS` | `64` | `>=1` | software I2C 同期ランナーの通常 idle パスの調整。 yield 前の busy probe 回数。 *上級* | `variants/frameworks/software/hal/i2c/i2c.inl` |

例:

```bash
# host で POSIX UART を自動提供しない
-DM5HAL_CONFIG_POSIX_UART=0
# legacy driver を既に使う ESP-IDF プロジェクトで gen4 backend を選ぶ
-DM5HAL_CONFIG_IDF_I2C_LEGACY=1
# 一時プールを拡げる
-DM5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE=512
```

## デバッグ診断 (`M5HAL_DEBUG_*`)

ライブラリ自体をデバッグするための診断トグル。 **サポート対象外** (production 設定ではない)。
既定 off で、 通常はどのビルドでも有効化しない。 機構は `M5HAL_CONFIG_*` と同じ値ベース。

| マクロ | 既定 | 値 | 効果 |
|---|---|---|---|
| `M5HAL_DEBUG_SOFTWARE_I2C_NO_WAIT` | `0` | `0`/`1` | `1`=software I2C 同期ランナーの yield パスを丸ごと無効化し連続 spin。 host 協調 (同一スレッドの master+slave で peer に実行機会を譲る動作) を壊すため、 ランナーのタイミングを切り分ける調査時のみ使う |

## 規約の対象外: バージョン/ABI 切替

`M5HAL_V0_INLINE` / `M5HAL_V1_INLINE` は `m5::hal::vN` を `inline namespace` にするかを選ぶ
**バージョン共存/ABI** の切替であり、 挙動 config とは別カテゴリ。 歴史的経緯からこの名前のまま
据え置き、 `M5HAL_CONFIG_*` 規約の対象外とする。 詳細は
[v0_v1_coexistence.md](v0_v1_coexistence.md)。
