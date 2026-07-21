# Hardware-in-the-loop テスト

このディレクトリには、device、host、または独立masterを実リンクして動かす再利用可能なテストを置く。
HIL programは `pio run` でビルドし、PlatformIO unit testとしては実行しない。

## テストモデル

| 場所 | endpoint | 判定主体 | 実行入口 |
|---|---|---|---|
| `test/v2/native/` | hostのみ | gtest | `pio test` |
| `test/v2/embedded/` | device 1台 | device自己判定 | `pio test` |
| `test/v2/hil/` | 接続した複数endpoint | fixtureが指定するhost、master、またはdevice | `pio run`後に実行またはmonitor |

HIL configは [`../../../pio_envs/v2/hil.ini.cli`](../../../pio_envs/v2/hil.ini.cli) を正本とする。
各fixtureのREADMEは、そのfixture固有の配線、コマンド、同期点、合否出力の正本である。

## 配置と役割

```text
test/v2/hil/
  common/hil_host.hpp       serial open、同期、I/Oの共通helper
  hil-run.sh                flash、build、実行を行うone-shot helper
  <fixture>/
    README.md               fixture固有の配線、コマンド、合否
    device/                 device firmware
    host/ or master/        必要な場合の判定driver
```

envの配置と名前は [`../../../pio_envs/README.md`](../../../pio_envs/README.md) とHIL configだけに置く。
親READMEは共通modelとrunner規則を扱い、子READMEではディレクトリ分類や設計契約を再掲しない。

## One-shot runner

`hil-run.sh` は `v2_hil_<name>_device_esp32` と `v2_hil_<name>_host` の組を持つfixtureに対応する。
serial portを自動検出または引数で受け取り、deviceのflash、host programのbuild、実行を順に行う。

```sh
test/v2/hil/hil-run.sh uart_echo
test/v2/hil/hil-run.sh uart_echo /dev/cu.usbserial-X 3000000
```

複数device、phase切替、異なるenv命名を持つfixtureは、子READMEの手動コマンドを使う。host programは
port未指定時にruntime skipしてよいが、それはhost buildを可能にするだけでHIL合格を意味しない。

## Fixtureの追加

1. 一つのfixtureディレクトリにdeviceとhost/masterのsourceを追加する。
2. `pio_envs/v2/hil.ini.cli` に専用envを追加する。
3. 子READMEには必要なhardware、配線、コマンド、同期点、正確な合否出力だけを書く。
4. 通常の1 device/1 host lifecycleで足りる場合だけ `hil-run.sh` に登録する。

日付付き実績、local device path、常設rigの割当、復帰手順はここへ記録しない。電気的な探索や一時的な
fault injectionも、再利用可能な公開受入手順とは分離する。
