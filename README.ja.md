# M5HAL

*English: [README.md](README.md)*

## 概要
M5 製品向けの HAL (ハードウェア抽象化レイヤ) です。

**v0 API は安定版**で既定動作のため、 既存コードはそのまま動作します。
**v2 API は開発中**で opt-in です — `<M5HAL_v2.hpp>` を明示的に include
して試せます。

## 動作要件

- ESP32 系ボード。 公開パッケージは `espressif32` platform
  (Arduino-ESP32 または ESP-IDF >= 4.4) を対象としています。
- C++17 に対応したコンパイラ。
- [M5Utility](https://github.com/m5stack/M5Utility) — PlatformIO と
  ESP-IDF component manager は自動で取得します。 Arduino IDE では
  M5HAL と併せてインストールしてください。

## インストール

- **Arduino IDE**: ライブラリマネージャから「M5HAL」をインストールし、
  「M5Utility」も併せてインストールしてください。
- **PlatformIO**: `platformio.ini` に追加:
  ```ini
  lib_deps =
      m5stack/M5HAL
  ```
  M5Utility は依存関係として自動解決されます。
- **ESP-IDF component manager**: プロジェクトの `idf_component.yml` に追加:
  ```yaml
  dependencies:
    m5stack/M5HAL: "*"
  ```

## ドキュメント

- 確定した仕様文書は [`spec/`](spec/README.md) 配下にあります
  (リリースパッケージにも同梱されます)。
- ビルド時の挙動設定と診断スイッチは
  [`spec/design/configuration.md`](spec/design/configuration.md) に一覧化しています。

## 読み始める場所

| 読者 | 最初に読む場所 |
|---|---|
| 既存 ESP32 v0 利用者 | `<M5HAL.hpp>` または `<M5HAL_v0.hpp>` をそのまま使ってください。移行期間の意味を知りたい場合だけ [v0 / v2 共存](#v0--v2-共存) を読めば十分です。 |
| v2 を sketch で試す人 | [v2 API を試す](#v2-api-を試す) を読んでから [`examples/v2/HowToUse/I2C`](examples/v2/HowToUse/I2C/)、[`examples/v2/HowToUse/SPI`](examples/v2/HowToUse/SPI/)、[`examples/v2/HowToUse/UART`](examples/v2/HowToUse/UART/) のいずれかを開いてください。 |
| backend 実装者・内部レビュー担当 | [`spec/README.md`](spec/README.md) を地図として使ってください。中心になる設計文書は `bus_accessor`, `i2c`, `spi`, `gpio`, `variants` です。 |

## v2 API を試す

v2 は明示的に opt-in して使います。 `<M5HAL_v2.hpp>` を include して
ください。 v0 対応 target では、同じ翻訳単位に v0 系エントリヘッダを混ぜることも可能です
([v0 / v2 共存](#v0--v2-共存) 参照) が、 ファイルごとに一方の世代に
揃えたほうが読みやすくなります。

現在の v2 バス API は次の要素を中心に組み立てます:

- **Bus** — 物理バスのインスタンス (`i2c::Bus`, `spi::Bus`, `uart::Bus`,
  `i2s::Bus`、 または `spi::Bus_software` のような明示 variant)
- **Accessor** — その bus 上の 1 つの通信相手。アドレス、CS pin、
  baud rate、周波数、タイムアウト、SPI mode などを保持
- **TransferDesc** — I2C register prefix や SPI command/address/dummy
  phase など、1 回の transfer に付くメタ情報。UART は transfer descriptor
  を必要としません
- **Source / Sink** — stream も見据えた入出力抽象。単純な buffer には
  span / raw pointer overload も使えます

**配線ごとに一つの共有 bus を acquire します**: v2 は HAL の中に隠れた
singleton bus を持たず、`M5_Hal` 自身も bus の寿命を保持しません。kind
ごとの registry が保持するのは weak reference です。推奨経路は、配線を
指定して bus を acquire することです — `M5_Hal.I2C.acquire(cfg)` はピンで
intern された shared owner を返すので、
ボードサポート層と利用者コードが同じピンを指せば *同一* インスタンス
(一つの物理バス・一つのロック) を共有でき、ワイヤの奪い合いになりません。
その handle から accessor を作ると、accessor も bus を共有所有します。最後の
handle または共有所有 accessor が破棄されるまで bus は生存し、その後 backend
が解放されて weak registry entry が再利用可能になります。
`M5_Hal` は GPIO / bus registry と service runner を束ねています。

escape hatch: 自分で bus を所有したい場合は、直接構築
(`i2c::Bus bus; bus.init(cfg);`) して accessor に参照で渡せます —
registry 経由の acquire が推奨の既定経路です。バス所有モデルは
[`spec/design/bus_accessor.md`](spec/design/bus_accessor.md)、共有と
hardware allocation のデモは
[`examples/v2/HowToUse/I2CRegistry`](examples/v2/HowToUse/I2CRegistry/)
を参照してください。

I2C の最小形 (**Arduino 環境**):

> **PlatformIO で使う場合:** M5HAL v2 は C++17 が必須です。
> `espressif32@6.x` (Arduino core 2.x) の既定は gnu++11 なので、
> `platformio.ini` に `build_flags = -std=gnu++17` と
> `build_unflags = -std=gnu++11` を追加してください。arduino-esp32 3.x と
> Arduino IDE は既定で C++17 です。(指定を忘れた場合、大量の `constexpr`
> エラーではなくヘッダの1行 `#error` でビルドが止まります。)
>
> 以下の例は Arduino backend 用です。`BusConfig` のフィールド構成は
> **variant 依存**で、`bus_cfg.wire = &Wire` は Arduino I2C backend にのみ
> 存在します。ESP-IDF / software backend では config が持つフィールドが
> 異なります (例の後の注記を参照)。

```cpp
#include <M5HAL_v2.hpp>
#include <Wire.h>

#include <memory>

namespace m5hal = m5::hal::v2;

std::shared_ptr<m5hal::i2c::IBus> i2c_bus;  // 寿命を所有する shared handle

void setup()
{
    // タグ型ピン指定: どちらの順で書いても正しいフィールドに入る
    m5hal::i2c::BusConfig bus_cfg{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};
    bus_cfg.wire = &Wire;

    // intern された bus を acquire する。この shared_ptr が寿命を所有する。
    auto acquired = m5hal::M5_Hal.I2C.acquire(bus_cfg);
    if (!acquired) return;
    i2c_bus = acquired.value();

    // 全フィールドを accessor 構築の「前」に設定する: config は構築時に
    // 値コピーされ、その後は凍結される。(遅延設定したい場合は accessor を
    // 既定構築し、後から setConfig() を呼ぶ。)
    m5hal::i2c::AccessConfig dev_cfg;   // フィールド代入。タグ ctor は無い
    dev_cfg.i2c_addr        = 0x76;
    dev_cfg.freq            = 100000;
    dev_cfg.wire_timeout_ms = 100;
    // dev_cfg.register_address_bytes = 2;  // 2-byte register address の device だけ指定

    m5hal::i2c::MasterAccessor dev{i2c_bus, dev_cfg};  // acquire した bus を共有所有

    // 各 transfer は result_t<T> を返す。直接代入せず unwrap する。
    auto id = dev.readRegister(0x00);   // uint8_t ではなく result_t<uint8_t>
    if (!id) return;                    // 使う前に確認 (id.error() にコードが入る)
    if (id.value() == 0x60) {
        // ... 期待した WHO_AM_I に一致 ...
    }
}
```

Arduino sketch として試す場合は
[`examples/v2/HowToUse/I2C`](examples/v2/HowToUse/I2C/)
から始めてください。bus scan、最初に ACK を返した device への Accessor 作成、
register read、複数 transfer を 1 つの bus lock にまとめる `ScopedAccess`
の例を含みます。

特定の backend を使いたい場合は、suffix 付きの CONFIG 型を acquire に渡します。
たとえば同じピンを software (bit-bang) I2C backend で駆動するには:

```cpp
m5hal::i2c::BusConfig_software bus_cfg{m5hal::i2c::Scl{22}, m5hal::i2c::Sda{21}};
auto i2c_bus = m5hal::M5_Hal.I2C.acquire(bus_cfg).value();
```

`#include <Wire.h>` が必要なのは、Arduino 環境の既定 I2C backend
(`i2c::BusConfig_arduino`) が `TwoWire` ハンドルを保持しているためです
(`bus_cfg.wire = &Wire`)。software / ESP-IDF の config を渡せば不要になります。
無印の `i2c::BusConfig` という綴りは、ビルド環境で最初に申告した backend の
型 alias (`BusConfig_arduino` 等) です。そのフィールド構成 (`wire` の有無
など) も選択された variant のものになります。

### よくある間違い (他ライブラリの癖は通用しません)

以下の書き方 (人間でもコード生成 AI でも手が伸びがち) は M5HAL v2 では
意図的に別の形になっており、コンパイルできません:

- **位置引数のピン指定** — `BusConfig{22, 21}` はコンパイルエラー。ピンは
  タグ型で `BusConfig{Scl{22}, Sda{21}}` と書く (SCL/SDA の取り違えを型で防ぐ)。
- **型駆動の register 幅** — `readRegister<uint16_t>` や `readRegister16` は
  存在しない。register アドレス幅は config の `register_address_bytes`
  フィールドで決まり、テンプレート引数では決まらない。
- **`acquire()` を素の `shared_ptr` 扱いする** — `M5_Hal.I2C.acquire(cfg)` は
  `shared_ptr` そのものではなく `result_t<shared_ptr<IBus>>` を返す。
  `if (!acquired)` で確認してから `acquired.value()`。register accessor も
  同様に `result_t<T>` を返すので、エラー確認の後に `.value()` を使い、
  直接代入はしない。

SPI も同じ Bus / Accessor の形で扱います。Arduino SPI、ESP-IDF SPI、
software SPI は、対応する framework support が見えている環境で v2 backend
として利用できます。CS を複数 transfer の間維持したい場合は、SPI 固有の
`beginTransaction()` / `endTransaction()` を使います。外部 SPI slave なしで
送信波形を確認する最初の sketch として
[`examples/v2/HowToUse/SPI`](examples/v2/HowToUse/SPI/) を用意しています。

UART も同じ Bus / Accessor の形で扱います。**baud rate は bus 側ではなく
`uart::AccessConfig` (accessor 側) にあります** — 同じ物理ポートを相手ごとに
異なる設定で使う形のためです。アクセサは TX 専用 (`TxAccessor`) / RX 専用
(`RxAccessor`) / 両方向の facade (`Accessor`) の 3 つから選びます: 送信と
受信を別タスクが扱うなら split を、単純なコマンド応答なら facade を
(設計上の主 API は split 側。詳細は
[`spec/design/uart.md`](spec/design/uart.md))。
[`examples/v2/HowToUse/UART`](examples/v2/HowToUse/UART/) は、USB Serial をログ用、
`Serial1` を M5HAL UART bus として使う Arduino sketch です。TX と RX を接続すると、
外部 UART device なしで loopback 受信を確認できます。
[`examples/v2/HowToUse/UARTEcho`](examples/v2/HowToUse/UARTEcho/) はその一歩先として、
受信したバイトを `StreamSink` アダプタ経由でそのまま送信側へ返す echo sketch です。
accessor を Source / Sink の stream モデルと組み合わせる方法を示します。

I2S は連続ストリーム型の bus です。ローカルの ESP-IDF gen5 backend は
TX (再生) / RX (録音) / 全二重に対応します: DOUT を設定すると TX、DIN を
設定すると RX、両方を設定すると独立した TX/RX DMA 経路を持つ全二重 bus
になります。再生だけなら `i2s::TxAccessor`、録音だけなら `i2s::RxAccessor`、
両方向を 1 object にまとめるなら `i2s::Accessor` を使います。remote I2S
proxy は credit flow control 付きの TX streaming と、同期 RPC の RX
`read` / `readableBytes` を公開します。host 側の例は
[`examples/v2/HowToUse/RemoteI2S`](examples/v2/HowToUse/RemoteI2S/) を
参照してください。

[`examples/v2/HowToUse/I2SAudio`](examples/v2/HowToUse/I2SAudio/) は、`i2s::Bus` の
ローカル TX パスで内蔵スピーカーから正弦波を再生する sketch です (I2S はボード固有の
アンプ初期化が必要です。M5Stack Core2 V1.1 で動作確認済み、CoreS3 向けの配線も
含みますが未検証)。

remote example は serial と TCP の両 transport を用意しています:
[`examples/v2/HowToUse/Remote`](examples/v2/HowToUse/Remote/) は host facade の入口、
[`examples/v2/RemoteServerTCP`](examples/v2/RemoteServerTCP/) は device を TCP で公開する例、
[`examples/v2/RemoteTest`](examples/v2/RemoteTest/) は host 側 protocol test harness です。

remote の一つの `Hal` は、一つの connection session と一つの RPC 直列化 gate を
所有します。その connection の bus、GPIO proxy、互換 session view、backend 操作は
すべて同じ gate を通ります。再接続すると旧 session は close され、再接続前の bus
proxy は以後 `CLOSED` を返し、新しい peer へ付け替えられません。保持済み GPIO object
が memory-safe なのは所有元 `Hal` の生存中だけです。再接続後は最終 cache 値を読み、
write / mode change は無視します。明示的な `BusView::release(shared_ptr&)` は caller が
唯一の owner であることを要求します (先に accessor と alias を破棄する)。成功時は
handle を consume して空にし、失敗時は元の handle を維持します。完全な lifetime、
callback、quarantine 規約は [`spec/design/remote.md`](spec/design/remote.md) と
[`spec/design/bus_accessor.md`](spec/design/bus_accessor.md) を参照してください。

[`examples/v2/HowToUse/Bytecode`](examples/v2/HowToUse/Bytecode/) は、GPIO / I2C / SPI の
一連の操作を bytecode (byte 配列のまま sketch に記述) で表し、M5Stack Core BASIC の
ボタン操作で実行するデモです。「初期化シーケンスを const テーブル化して再生する」
使い方をそのまま示します。

全 example の一覧 (配線・期待される出力つき) は
[`examples/v2/HowToUse/README.md`](examples/v2/HowToUse/README.md) にあります。

## API 世代 (v0 / v2) とリリース番号の関係

M5HAL は **API 世代** (仕様系統) と **リリース番号** (ライブラリ版数) を分けて扱います。
世代番号は、その世代が既定になるメジャー版数と一致させています —
v0 ⇔ `0.x`、v2 ⇔ `2.x`。(`v1` という API 世代は存在しません — 世代番号と
メジャー版数が食い違う組み合わせが生まれないよう、番号を意図的に欠番に
しています。)

- **v0** — 旧 API 世代。 `v0.0.x` リリースで公開済の仕様。
- **v2** — 新 API 世代。 大規模な再設計を含む新仕様。
- **`1.x.x`** — v0 と v2 を 1 ライブラリ内に同梱する **移行期間のリリース系列**。

メジャー版数が `0` の間 (`v0.x.y`) は **v0 が既定動作**で、 既存利用者は
コード変更なしで新リリースを受け取れます。 `1.0` リリースからは、 明示的に
オプトインした利用者が v2 API を試せるようになります。 将来メジャー版数が
`2` (`v2.x.y`) に上がるタイミングで、 v2 が既定動作に切り替わります。

## v0 / v2 共存

M5HAL は、 既存の v0 利用者がコード変更なしで動かし続けられる形で
v2 API を 1 ライブラリ内に**共存**させる戦略を採用しています。 利用者は
用途に応じて以下のエントリヘッダを使い分けます:

| エントリヘッダ | 公開する namespace | 想定利用者 |
|---|---|---|
| `<M5HAL.hpp>` | (shim → 既定で v0) | 後方互換性用。 既に `<M5HAL.hpp>` を include している既存コードはそのまま動作。 新規コードは下の明示ヘッダを推奨。 |
| `<M5HAL_v0.hpp>` | `m5::hal::*` (= v0、 `inline namespace v0` 経由) | 明示的に v0 (legacy) API を選ぶコード |
| `<M5HAL_v2.hpp>` | `m5::hal::v2::*` | 明示的に v2 API を選ぶコード |

- **非 ESP32 Arduino target は v2 専用**。 `<M5HAL_v2.hpp>` を明示的に include
  してください。install 済み library に `M5HAL_v0.cpp` は残りますが、非 ESP32
  Arduino core では empty TU としてコンパイルされます。それらの target で
  `<M5HAL.hpp>` または `<M5HAL_v0.hpp>` を include すると、v2 を使うよう案内する
  error で早期に拒否されます。compatibility shim が target によって別世代を選ぶことは
  ありません。
- **v0 対応 target では、同一翻訳単位 (TU) での両エントリ include も可能**。 include ガードと
  platform 検出マクロは世代分離済みのため、 1 つの `.cpp` が v0 系
  エントリ (`<M5HAL.hpp>` shim か `<M5HAL_v0.hpp>` 直接) と
  `<M5HAL_v2.hpp>` を同時に include できます (ファイル単位で段階的に
  移行する場合など)。 ただし中間ライブラリでは、 可読性のため TU ごとに
  使う世代を明示することを推奨します。

inline namespace の既定切替 (`M5HAL_V0_INLINE`)、世代分離した platform
マクロ、 将来世代を既存利用者を巻き込まず追加できる `hal/<vN>/`
配置の詳細は
[`spec/design/v0_v2_coexistence.md`](spec/design/v0_v2_coexistence.md)
を参照してください。
