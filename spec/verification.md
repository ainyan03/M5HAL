# verification — 検証の構成と実行入口

> **読者**: メンテナ向け（検証範囲と公開実行入口）。

M5HAL の検証を層ごとに分け、ローカルで再現するための安定した入口を示す。個々の
PlatformIO env の配置と選択方法は [`../pio_envs/README.md`](../pio_envs/README.md)、実機fixtureの
配線・合否条件は [`../test/v2/hil/`](../test/v2/hil/) 以下を正本とする。

## 検証層

| 層 | 保証する範囲 | 保証しない範囲 | 主な正本 |
|---|---|---|---|
| 文書・静的検査 | Markdown構造とリンク、設定macro、公開API表記、廃止APIの混入 | compile/link、runtime | [`.github/scripts/`](../.github/scripts/) |
| native unit | host上で決定的に再現できるAPI意味論、状態遷移、protocol処理 | SDK実装、物理配線、電気特性 | [`test/v2/native/`](../test/v2/native/) |
| native fuzz/property | frame codecとreceive-only bytecodeの境界・任意入力耐性 (ASan/UBSan) | hardware actuation、coverage率保証 | [`test/v2/fuzz/`](../test/v2/fuzz/) |
| ESP-IDF fake | 実物backendをfake SDK/driver上で動かせる割込み順序、timeout、cleanup | 実peripheral、DMA、bus timing | [`test/v2/native_espidf/`](../test/v2/native_espidf/) |
| compile fence | 公開header/API surfaceが対象platform・framework・世代でcompile/linkすること | 実機runtime | [`pio_envs/`](../pio_envs/) と [CI workflow](../.github/workflows/) |
| embedded self-test | 1台のdeviceで自己判定できるwire semantic | host/device連携、測定器による波形品質 | [`test/v2/embedded/`](../test/v2/embedded/) |
| HIL | 実配線した複数endpoint間の転送とlifecycle | 自動CI、一般的な電気特性保証 | [`test/v2/hil/`](../test/v2/hil/) |

公開APIの正準compile fixtureは
[`examples/v2/BuildTest/BuildTest.cpp`](../examples/v2/BuildTest/BuildTest.cpp) に集約する。全public型・関数・overload・default引数を参照し、Arduino ESP32、official ESP-IDF、POSIX native、SPRESENSEのwrapperから同じfileをcompile/linkする。旧APIの再流入は別のobsolete/negative fenceで拒否する。設計契約そのものは [`design/`](design/) を正本とする。

## ローカル実行

コマンドはrepository rootで実行する。依存checkoutやtoolchainなど、各workflow固有の準備は対応する
workflowを参照する。

### 文書・公開表記

```sh
python3 .github/scripts/check-docs.py
python3 .github/scripts/check-config-macros.py
python3 .github/scripts/check-spec-api.py
python3 .github/scripts/check-obsolete-api.py
```

### native unitとESP-IDF fake

```sh
pio test -e test_native
pio test -e test_native_ndebug
pio test -e test_native_espidf_fake
pio test -e test_native_espidf_fake_be
pio test -e test_native_espidf_fake_spi_slave
```

対象を絞る場合はPlatformIOの `-f` を使う。通常のnative suiteと低頻度のThreadSanitizer envは
[`platformio.ini`](../platformio.ini) を正本とする。

`test_native_ndebug`はdebug assertionと対になるrelease error経路だけを絞って実行する。frame/bytecodeの固定seed property testは通常native suite、libFuzzerは次で実行する。

```sh
bash scripts/fuzz-test.sh
```

PR/pushはcommit済みcorpusから各target 2000 iteration、scheduled/manualは長いrunを使う。入力は4096 byteに制限し、bytecode runnerにはbus/GPIOを登録せずdelayもno-opにする。失敗artifactは最小化してCI artifactまたは`.generated/fuzz/artifacts/`へ保持し、coverage thresholdは設けない。

### compile fenceとexample

CLI専用envはconfigを明示してロードする。

```sh
M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/check.ini.cli \
  pio run -e v2_check_native
pio run -e HowToUse_I2C_esp32
```

対象世代・framework・boardに対応するenvは
[`../pio_envs/README.md`](../pio_envs/README.md) から選ぶ。全matrixの正確な対象は
[`build-check-pio.yml`](../.github/workflows/build-check-pio.yml)、公式ESP-IDF component buildは
[`build-check-idf.yml`](../.github/workflows/build-check-idf.yml)、SPRESENSEは
[`build-check-spresense.yml`](../.github/workflows/build-check-spresense.yml) を正本とする。

### embedded self-testとHIL

embedded self-testは `pio_envs/v2/test.ini.cli`、HILは `pio_envs/v2/hil.ini.cli` をロードする。
HILの共通modelとrunnerは [`../test/v2/hil/README.md`](../test/v2/hil/README.md)、fixture固有の配線、
flash、実行、合否は各fixtureのREADMEに従う。

## CIとの関係

CI job、matrix、cache、runnerの選択は [`.github/workflows/`](../.github/workflows/) が正本である。
この文書にはworkflow名やenvの完全な一覧を複製しない。ローカル実行は変更箇所に対応する層を選び、
CIは公開matrix全体のbackstopとして扱う。

実機を使わないbuild jobはHIL firmwareのcompile fenceであり、HILのruntime結果ではない。実機の
合否を主張するときは、fixture READMEに記載された配線、実行コマンド、合否文字列を使う。

## 公開testに置くもの

`test/` には、第三者が再現でき、コードまたは明示した出力で合否を判断できる検証を置く。一時的な
探索、日付付きの受入実績、特定benchの常設運用、ロジックアナライザやオシロによる観測記録は公開testの
手順に含めない。観測から得たprotocol不変条件を公開する場合は、再現可能なself-testまたはHIL fixtureへ
落とし込む。

## software I2C テストの読み方

software I2Cのprotocol-level native testは`test/v2/native/bus/test_software_i2c/`にある。
正準queue lifecycle (`SlaveAccessor::beginAccess/endAccess`) とlegacy opt-in wire-frame window
(`SlaveStreamAccessor::openWireFrame/closeWireFrame`) の双方を`VirtualOpenDrainBus`で検証し、
probe ACK、write、read-only、write-then-read、NACK、clock stretch、STOP、queue overflow、
受付fenceを固定する。詳細は[design/i2c_slave.md](design/i2c_slave.md)。実機のwire品質
(pull-up強度・配線長・接続device数) は[design/i2c.md](design/i2c.md) §timing と物理層の注意 を参照。

## M5UnitUnified 連携ビルドチェック

M5HALの変更が主要利用者M5UnitUnifiedの現行pinを壊していないことを確認する。現行M5UUは
`<M5HAL.hpp>`とunqualified `m5::hal`を使うv0 consumerであり、このgateはM5UUをv2へ移行するものではない。
固定pinのconsumer TUをArduino既定のC++11から変更せず、M5HALのC++11互換v0 surfaceとcompile/linkする
(M5HAL自身のsourceは`library.json`指定どおりC++17)。

前提: M5HAL、M5Utility、M5UnitUnifiedを同じ親directoryに置き、後2者を
`test/pio_m5uu_v0_consumer/pins.env`のexact commitでcleanにcheckoutしておく。

```bash
bash test/pio_m5uu_v0_consumer/check.sh
```

fixtureは3 repositoryをlocal sibling sourceとして解決し、build前後のexact pin/clean確認と
compiler dependency recordでlocal sourceが実際に消費されたことを証明する (`libdeps`配下の同名copy
混入も失敗)。CIでは`.github/workflows/m5uu-v0-consumer-check.yml`が同じfixtureとpinを使う。
pin更新は単なるbranch追従ではなく、M5UUが依然v0 consumerか、fixture契約とprovenance検査が妥当かを
同時にレビューして行う。M5UUのv2 adapter化は別のsource-breaking作業であり、この互換fenceへ混ぜない。
