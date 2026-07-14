# test/v2/hil — `pio run` 方式の実機テスト

ここは **`pio test` ではなく `pio run` で動かすテスト系**の置き場。現状の主役は
**HIL（hardware-in-the-loop）**: native ホストプロセスと実機 firmware が実リンク（USB シリアル等）で
同時に動き、**ホスト側が実機と通信して結果を判定する**検証。

`pio test` で実行する native / embedded スイートとは役割が違う:

| 場所 | 主体 | 判定 | CI |
|---|---|---|---|
| `test/v2/native/` | host のみ | host gtest（`pio test`） | ✅ 自動・HW 不要 |
| `test/v2/embedded/` | device のみ | device 自己判定（Unity） | 手動・実機 |
| **`test/v2/hil/`（ここ）** | **device + host ペア** | **host が device と喋って判定** | 手動・実機＋ポート指定 |

device / host の専用 env が対象ソースを明示しているため、host ドライバは
**gtest バイナリを `pio run` でビルド → 直接実行**する。

## レイアウト

```
test/v2/hil/
  common/hil_host.hpp        共有ホストハーネス（ポート open / sync / drain / readExact / env）
  hil-run.sh                 ランナー（flash → host ビルド → host 実行）
  <name>/
    README.md                配線・実行・期待結果
    device/<name>.cpp        実機 firmware（M5HAL ベース）
    host/<name>.cpp          host ドライバ（gtest、hil_host.hpp を使う）
```

env は `pio_envs/v2/hil.ini.cli`（GUI に出さない `.ini.cli`。`M5HAL_PIO_EXTRA_CONFIG` で
オンデマンドにロード、コピー不要）に `v2_hil_<name>_device_esp32` ＋ `v2_hil_<name>_host` の 2 本。

## 実行

一発（ポート自動検出 / baud 指定可）:

```sh
test/v2/hil/hil-run.sh uart_echo                       # 既定 115200
test/v2/hil/hil-run.sh uart_echo /dev/cu.usbserial-X 3000000
```

手動:

```sh
export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli       # hil env をロード（コピー不要）
pio run -e v2_hil_uart_echo_device_esp32 -t upload          # 実機に焼く
pio run -e v2_hil_uart_echo_host                            # host をビルド
M5HAL_POSIX_UART_PORT=/dev/cu.usbserial-X \
  .pio/build/v2_hil_uart_echo_host/program                 # host を実行
```

`M5HAL_POSIX_UART_PORT` 未設定なら host は **skip**（HW 無しでもビルドは通る）。

## 新しい HIL テストの追加

1. `test/v2/hil/<name>/device/<name>.cpp`（実機 firmware）と
   `test/v2/hil/<name>/host/<name>.cpp`（host gtest、`#include "../../common/hil_host.hpp"`）を作る。
2. `pio_envs/v2/hil.ini.cli` に `v2_hil_<name>_device_esp32` と `v2_hil_<name>_host` を追加。
3. `test/v2/hil/<name>/README.md` に配線・実行・期待結果を書く。
4. `test/v2/hil/hil-run.sh <name>` で動く。

remote バス等の将来の HIL（host transport ↔ device server）も同じ枠に乗る。
