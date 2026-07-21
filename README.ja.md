# M5HAL

*English: [README.md](README.md)*

<!-- pair: overview -->
## 概要

M5HAL は M5 製品向けの HAL (ハードウェア抽象化レイヤ) です。安定した
**v0 API** は既存 ESP32 コード向けの既定動作を維持します。**v2 API** は
開発中の新設計 API であり、`<M5HAL_v2.hpp>` による明示的な opt-in で利用します。

<!-- pair: requirements -->
## 動作要件

- v0 は ESP32 系ボードを対象とします。v2 は allowlist 対象の非 ESP Arduino core も
  build-check していますが、runtime の保証範囲は core ごとに異なります
  ([詳細](spec/design/variants.md#arduino-variant-の対応コア-build-gate))。
  ESP-IDF component として直接使う場合は ESP-IDF 5.0 以降が必要です。
  Arduino-ESP32 は、内部 SDK が ESP-IDF 4.4 の core 2.x も対応範囲です。
- C++17 に対応したコンパイラ。
- [M5Utility](https://github.com/m5stack/M5Utility)。PlatformIO と ESP-IDF
  component manager は自動解決します。Arduino IDE では M5HAL と併せて
  インストールしてください。

<!-- pair: installation -->
## インストール

- **Arduino IDE:** Library Manager から「M5HAL」と「M5Utility」をインストール。
- **PlatformIO:** `platformio.ini` にライブラリを追加:

  ```ini
  lib_deps =
      m5stack/M5HAL
  ```

- **ESP-IDF component manager:** `idf_component.yml` に追加:

  ```yaml
  dependencies:
    m5stack/M5HAL: "*"
  ```

  ESP-IDF 5.0 同梱の component manager は現在の manifest schema より古いため、
  ESP-IDF 5.0 環境を有効化して
  `python -m pip install --upgrade idf-component-manager` で更新してください。

<!-- pair: quickstart -->
## v2 クイックスタート

M5HAL v2 は C++17 が必須です。Arduino core 2.x を使う PlatformIO
`espressif32@6.x` の既定は gnu++11 なので、
`build_flags = -std=gnu++17` と `build_unflags = -std=gnu++11` を追加します。
Arduino-ESP32 3.x と Arduino IDE は既定で C++17 です。

次の最小 Arduino I2C 例では portable な `BusConfig` に配線を指定し、選択済み
provider に backend の生成と初期化を任せます:

```cpp
#include <M5HAL_v2.hpp>

#include <memory>

namespace m5hal = m5::hal::v2;

std::shared_ptr<m5hal::i2c::IBus> i2c_bus;

void setup()
{
    m5hal::i2c::BusConfig bus_cfg{
        m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};

    auto acquired = m5hal::M5_Hal.I2C.acquire(bus_cfg);
    if (!acquired) return;
    i2c_bus = acquired.value();

    m5hal::i2c::MasterAccessConfig dev_cfg;
    dev_cfg.i2c_addr = 0x76;
    dev_cfg.freq = 100000;

    m5hal::i2c::MasterAccessor dev{i2c_bus, dev_cfg};
    auto value = dev.readRegister(0x00);
    if (!value) return;
    // value.value() を使う。
}

void loop() {}
```

`BusConfig` が保持するのは portable な bus semantics と配線であり、framework
handle ではありません。既存 native object を使う場合は caller が初期化したうえで、
別の ownership policy として渡します。完全な例は
[`I2C example`](examples/v2/HowToUse/I2C/)、ownership、locking、transfer、lifetime
の契約は [`Bus / Accessor 仕様`](spec/design/bus_accessor.md) を参照してください。

<!-- pair: generations -->
## API 世代とエントリヘッダ

API 世代とリリース番号は別の概念です。v0 は安定した旧世代として移行期間中も
既定を維持し、v2 は明示的な opt-in で利用できます。v1 API 世代は存在しません。

| ヘッダ | 公開 API | 用途 |
|---|---|---|
| `<M5HAL.hpp>` | 既定で v0 | 既存 source との互換性 |
| `<M5HAL_v0.hpp>` | `m5::hal::*` (v0) | v0 の明示選択 |
| `<M5HAL_v2.hpp>` | `m5::hal::v2::*` | v2 の明示選択 |

非 ESP Arduino target は v2 専用です。v0 対応 target では v0 と v2 の
エントリヘッダを同じ翻訳単位に含められますが、ファイルごとに一方を選ぶ方が
明確です。切替と namespace の完全な契約は
[`v0 / v2 共存`](spec/design/v0_v2_coexistence.md) にあります。

<!-- pair: navigation -->
## 次に読む文書

| 目的 | 文書 |
|---|---|
| example を実行する | [`v2 example index`](examples/v2/HowToUse/README.md) |
| 公開仕様の全体を把握する | [`spec/README.md`](spec/README.md) |
| build-time の挙動を設定する | [`configuration.md`](spec/design/configuration.md) |
| bus ownership と accessor を理解する | [`bus_accessor.md`](spec/design/bus_accessor.md) |
| backend を選択・移植する | [`variants.md`](spec/design/variants.md), [`porting guide`](spec/porting_guide/README.md) |
| remote transport を使う | [`remote.md`](spec/design/remote.md) |
