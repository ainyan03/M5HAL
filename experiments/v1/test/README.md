# experiments/test — `pio run` 方式のテスト

ここは **`pio test` ではなく `pio run` で動かすテスト系**の置き場。現状の主役は
**HIL（hardware-in-the-loop）**: native ホストプロセスと実機 firmware が実リンク（USB シリアル等）で
同時に動き、**ホスト側が実機と通信して結果を判定する**検証。

`test/` 配下（`pio test` の gtest スイート）とは役割が違う:

| 場所 | 主体 | 判定 | CI |
|---|---|---|---|
| `test/v1/native/` | host のみ | host gtest（`pio test`） | ✅ 自動・HW 不要 |
| `test/v1/embedded/` | device のみ | device 自己判定（Unity） | 手動・実機 |
| `experiments/v1/` | device のみ | 人間が目視 / ロジアナ | 手動・探索/デモ |
| **`experiments/v1/test/`（ここ）** | **device + host ペア** | **host が device と喋って判定** | 手動・実機＋ポート指定 |

`pio test` は探索先が `test_dir`（=`test/`）に固定されるため使えない。よって host ドライバは
**gtest バイナリを `pio run` でビルド → 直接実行**する。

## レイアウト

```
experiments/v1/test/
  common/hil_host.hpp        共有ホストハーネス（ポート open / sync / drain / readExact / env）
  hil-run.sh                 ランナー（flash → host ビルド → host 実行）
  <name>/
    README.md                配線・実行・期待結果
    device/<name>.cpp        実機 firmware（M5HAL ベース）
    host/<name>.cpp          host ドライバ（gtest、hil_host.hpp を使う）
```

env は `pio_envs/v1/hil.ini.cli`（GUI に出さない `.ini.cli`。`M5HAL_PIO_EXTRA_CONFIG` で
オンデマンドにロード、コピー不要）に `v1_hil_<name>_device_esp32` ＋ `v1_hil_<name>_host` の 2 本。

## 実行

一発（ポート自動検出 / baud 指定可）:

```sh
experiments/v1/test/hil-run.sh uart_echo                       # 既定 115200
experiments/v1/test/hil-run.sh uart_echo /dev/cu.usbserial-X 3000000
```

手動:

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/hil.ini.cli       # hil env をロード（コピー不要）
pio run -e v1_hil_uart_echo_device_esp32 -t upload          # 実機に焼く
pio run -e v1_hil_uart_echo_host                            # host をビルド
M5HAL_POSIX_UART_PORT=/dev/cu.usbserial-X \
  .pio/build/v1_hil_uart_echo_host/program                 # host を実行
```

`M5HAL_POSIX_UART_PORT` 未設定なら host は **skip**（HW 無しでもビルドは通る）。

## 新しい HIL テストの追加

1. `experiments/v1/test/<name>/device/<name>.cpp`（実機 firmware）と
   `experiments/v1/test/<name>/host/<name>.cpp`（host gtest、`#include "../../common/hil_host.hpp"`）を作る。
2. `pio_envs/v1/hil.ini.cli` に `v1_hil_<name>_device_esp32` と `v1_hil_<name>_host` を追加。
3. `experiments/v1/test/<name>/README.md` に配線・実行・期待結果を書く。
4. `experiments/v1/test/hil-run.sh <name>` で動く。

remote バス等の将来の HIL（host transport ↔ device server）も同じ枠に乗る。
