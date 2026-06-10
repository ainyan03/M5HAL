# M5HAL

*English: [README.md](README.md)*

## 概要
M5 製品向けの HAL (ハードウェア抽象化レイヤ) です。

**v0 API は安定版**で既定動作のため、 既存コードはそのまま動作します。
**v1 API は開発中**で opt-in です — `<M5HAL_v1.hpp>` を明示的に include
して試せます。

## ドキュメント

- 確定した仕様文書は [`spec/`](spec/README.md) 配下にあります。

## API 世代 (v0 / v1) とリリース番号の関係

M5HAL は **API 世代** (仕様系統) と **リリース番号** (ライブラリ版数) を分けて扱います。

- **v0** — 旧 API 世代。 `v0.0.x` リリースで公開済の仕様。
- **v1** — 新 API 世代。 大規模な再設計を含む新仕様。
- **v0.2.x** — v0 と v1 を 1 ライブラリ内に同梱する **移行期間のリリース系列**。

メジャー版数が `0` の間 (`v0.x.y`) は **v0 が既定動作**で、 既存利用者は
コード変更なしで新リリースを受け取れます。 `v0.2` リリースからは、 明示的に
オプトインした利用者が v1 API を試せるようになります。 将来メジャー版数が
`1` (`v1.x.y`) に上がるタイミングで、 v1 が既定動作に切り替わります。

## v0 / v1 共存

M5HAL は、 既存の v0 利用者がコード変更なしで動かし続けられる形で
v1 API を 1 ライブラリ内に**共存**させる戦略を採用しています。 利用者は
用途に応じて以下のエントリヘッダを使い分けます:

| エントリヘッダ | 公開する namespace | 想定利用者 |
|---|---|---|
| `<M5HAL.hpp>` | (shim → 既定で v0) | 後方互換性用。 既に `<M5HAL.hpp>` を include している既存コードはそのまま動作。 新規コードは下の明示ヘッダを推奨。 |
| `<M5HAL_v0.hpp>` | `m5::hal::*` (= v0、 `inline namespace v0` 経由) | 明示的に v0 (legacy) API を選ぶコード |
| `<M5HAL_v1.hpp>` | `m5::hal::v1::*` | 明示的に v1 API を選ぶコード |

- **翻訳単位 (TU) ごとに一方を選択**。 ヘッダ群は共通マクロ
  (`M5HAL_FRAMEWORK_HAS_*` / `M5HAL_TARGET_PLATFORM_*`) を持つため、
  中間ライブラリは各 `.cpp` で v0 系エントリ (`<M5HAL.hpp>` shim か
  `<M5HAL_v0.hpp>` 直接) か `<M5HAL_v1.hpp>` のいずれか一方を include
  してください。 1 つの sketch 内で異なるヘッダを使うライブラリを
  組み合わせることは可能 (各 TU で 1 つだけに抑える限り)。
- **既定の切替**。 `M5HAL_V0_INLINE` マクロ (`src/m5_hal_config.hpp`、
  メジャー版数 `0` の間は既定 `1`) で v0 を `inline namespace v0` として
  expose するかを制御します。 将来メジャー版数が `1` に上がるタイミングで
  既定値が `0` に反転し、 代わりに v1 が `inline namespace v1` になります。
  この段階で旧コードは `m5::hal::v0::Foo` の明示参照が必要になります。
- **将来拡張に開いた配置**。 物理階層 (`hal/<vN>/` で実装、
  `examples/<vN>/` でサンプル、 `M5HAL_<vN>.hpp` でエントリ) は将来の
  v2 / v3 にも同パターンで拡張可能で、 既存利用者を巻き込まずに新版を
  並列追加できます。

## 読み始める場所

| 読者 | 最初に読む場所 |
|---|---|
| 既存 v0 利用者 | `<M5HAL.hpp>` または `<M5HAL_v0.hpp>` をそのまま使ってください。移行期間の意味を知りたい場合だけ [v0 / v1 共存](#v0--v1-共存) を読めば十分です。 |
| v1 を sketch で試す人 | [v1 API を試す](#v1-api-を試す) を読んでから [`examples/v1/HowToUse/I2C`](examples/v1/HowToUse/I2C/)、[`examples/v1/HowToUse/SPI`](examples/v1/HowToUse/SPI/)、[`examples/v1/HowToUse/UART`](examples/v1/HowToUse/UART/) のいずれかを開いてください。 |
| backend 実装者・内部レビュー担当 | [`spec/README.md`](spec/README.md) を地図として使ってください。中心になる設計文書は `bus_accessor`, `i2c`, `spi`, `gpio`, `variants` です。 |

## v1 API を試す

v1 は明示的に opt-in して使います。 `<M5HAL_v1.hpp>` を include し、
同じ翻訳単位で v0 系エントリヘッダを混ぜないでください。

現在の v1 バス API は次の要素を中心に組み立てます:

- **Bus** — 物理バスのインスタンス (`i2c::Bus`, `spi::Bus`, `uart::Bus`、
  または `spi::variant::software::Bus` のような明示 variant)
- **Accessor** — その bus 上の 1 つの通信相手。アドレス、CS pin、
  baud rate、周波数、タイムアウト、SPI mode などを保持
- **TransferDesc** — I2C register prefix や SPI command/address/dummy
  phase など、1 回の transfer に付くメタ情報。UART は transfer descriptor
  を必要としません
- **Source / Sink** — stream も見据えた入出力抽象。単純な buffer には
  span / raw pointer overload も使えます

I2C の最小形:

```cpp
#include <M5HAL_v1.hpp>
#include <Wire.h>

namespace m5hal = m5::hal::v1;

m5hal::i2c::Bus i2c_bus;

void setup()
{
    m5hal::i2c::BusConfig bus_cfg{&Wire, 22, 21};  // Wire, SCL, SDA
    i2c_bus.init(bus_cfg);

    m5hal::i2c::I2CMasterAccessConfig dev_cfg;
    dev_cfg.i2c_addr = 0x76;
    dev_cfg.freq = 100000;
    dev_cfg.timeout_ms = 100;
    // dev_cfg.register_address_bytes = 2;  // 2-byte register address の device だけ指定

    m5hal::i2c::I2CMasterAccessor dev{i2c_bus, dev_cfg};
    auto id = dev.readRegister(0x00);
}
```

Arduino sketch として試す場合は
[`examples/v1/HowToUse/I2C`](examples/v1/HowToUse/I2C/)
から始めてください。bus scan、最初に ACK を返した device への Accessor 作成、
register read、複数 transfer を 1 つの bus lock にまとめる `ScopedAccess`
の例を含みます。

backend を明示したい場合は variant alias namespace を使います。たとえば
software I2C backend は example 内の bus 型を次のように差し替えます:

```cpp
using ExampleI2CBus = m5::hal::v1::i2c::variant::software::Bus;
```

SPI も同じ Bus / Accessor の形で扱います。Arduino SPI、ESP-IDF SPI、
software SPI は、対応する framework support が見えている環境で v1 backend
として利用できます。CS を複数 transfer の間維持したい場合は、SPI 固有の
`beginTransaction()` / `endTransaction()` を使います。外部 SPI slave なしで
送信波形を確認する最初の sketch として
[`examples/v1/HowToUse/SPI`](examples/v1/HowToUse/SPI/) を用意しています。

UART も同じ Bus / Accessor の形で扱います。
[`examples/v1/HowToUse/UART`](examples/v1/HowToUse/UART/) は、USB Serial をログ用、
`Serial1` を M5HAL UART bus として使う Arduino sketch です。TX と RX を接続すると、
外部 UART device なしで loopback 受信を確認できます。
[`examples/v1/HowToUse/UARTEcho`](examples/v1/HowToUse/UARTEcho/) はその一歩先として、
受信したバイトを `StreamSink` アダプタ経由でそのまま送信側へ返す echo sketch です。
accessor を Source / Sink の stream モデルと組み合わせる方法を示します。

[`examples/v1/HowToUse/Bytecode`](examples/v1/HowToUse/Bytecode/) は、GPIO / I2C / SPI の
一連の操作を bytecode (byte 配列のまま sketch に記述) で表し、M5Stack Core BASIC の
ボタン操作で実行するデモです。「初期化シーケンスを const テーブル化して再生する」
使い方をそのまま示します。
