# architecture — M5HAL v2 全体構造

> **読者**: 実装者・レビュー向け（設計仕様）。

M5HAL v2 API の全体構造と設計原則を示す。 個別の設計は [design/](design/) 配下を参照。

## 設計原則

- **組み込みファースト** — RTTI / 例外に依存しない
- **シンプル優先** — 状態機械や補助状態は必要最小限にとどめる
- **宣言と実装の分離** — ヘッダは宣言を中心に置き、 重い実装は `.inl` 等に分離する
- **同期 API を正本とする** — 非同期は将来拡張として扱う
- **テストで契約を担保する** — 抽象基底の契約はユニットテストで検証する
- **ゼロコピー指向** — 大きなデータ転送で不要なコピーを避ける
- **先行整備をしすぎない** — 実需が見えていない抽象は導入しない

## 層構成

依存は上から下への一方向を原則とする。

```text
hal           外界との入出力資源 (I2C, SPI, UART, GPIO 等) と
              実行環境設備 (time, mutex = runtime kind)                  → m5::hal::*
core          ハードウェア非依存の基盤 (error, span, chunk, source, sink)  → m5::core::*         (予約層、 現行リリース未実装)
variants      層を横断するメタ機構                                        → m5::variants::*
```

- `core` は他層へ依存しない
- `hal` は `core` に依存してよい
- 抽象基底レベルでの相互依存は避ける

> **runtime 設備は hal 層の 1 kind**: time / mutex は独立 namespace (`m5::runtime::*`) ではなく
> **`m5::hal::v2::runtime`** の HAL kind (variant flat 注入) として提供する
> ([design/runtime.md](design/runtime.md))。 `m5::*` 直下の独立 namespace は他公式ライブラリとの
> 名前先取り問題があり、 v0/v2 世代分離の外に出てしまうため採らない。
>
> **`core` は予約層**: 現行リリースでは `src/m5_hal/core/` ディレクトリは存在せず、 namespace
> `m5::core::*` の実体もまだ持たない。 span/chunk 等の hal-agnostic 共通型を分離する際の予約と
> して、 名前と依存方向だけを先取りで定義している。 cross-cutting な型 (`error_t` 等) は現行では
> `m5::hal::v2::error::*` に置く ([design/v0_v2_coexistence.md](design/v0_v2_coexistence.md))。

## HAL object 層

**v2 caller の正本入口は `m5::hal::v2::M5_Hal` object** (`m5::hal::v2::Hal&` 型)。 `m5::hal::v2::Hal` はローカルとリモートで共通の public facade で、 sub-object (`Gpio` / `I2C` / `SPI` / `UART` / `I2S` / `Services` / `Memory`) を束ねる。

リモート `Hal` は接続確立後に同型 API を提供する: `Hal remote; remote.connect(endpoint)` で接続する (endpoint = `"uart:<path>"` / `"tcp:<host>:<port>"`。 typed API `remote.initUart(port)` / `remote.initTcp("host:port")` も存続)。 以降は同じ `remote.I2C.acquire(cfg)` / `remote.SPI.acquire(cfg)` の形で proxy bus を取得でき、 ローカル `Hal` と同一のコード面で使える。 詳細は [design/remote.md](design/remote.md) を参照。 `M5HALCore` はローカル backend を所有する内部シングルトン (caller は直接使わない)。 caller は `m5::hal::v2::M5_Hal.Gpio.*` のように各 sub-object にアクセスする。

`m5::hal::v2::<kind>::*` 配下の勝者バインドされた関数 (例: `gpio::getGPIO()`) は `M5HALCore` ctor が bootstrap seam として内部利用するため、 caller が直接呼ぶ必要はない (詳細は [design/gpio.md](design/gpio.md) §caller 向け唯一の entry point)。

namespace-scope initializer や他ライブラリの global ctor から触る場合は `getM5_Hal()` を使う (`Hal&` を返す。 `M5_Hal` alias は eager-init のため lazy-safe ではない、 lazy-safe accessor は `getM5_Hal()` のみ)。

## namespace と配置

### namespace 帰属

- HAL の範疇は `m5::hal::*` 配下に置く (runtime 設備も `m5::hal::v2::runtime` の 1 kind)
- core (予約層) は `m5::*` 直下の別 namespace に置く
- cross-cutting な型 (`error_t` 等) は `m5::hal::` 直下に置く
- variant 機構は `m5::variants::*` に置く

v0/v2 共存による物理 namespace (`m5::hal::v2::<kind>` 等) と inline 展開の詳細は
詳細は [design/v0_v2_coexistence.md](design/v0_v2_coexistence.md) §namespace 配置 を参照。

### ディレクトリ配置

- ライブラリ実装は `src/m5_hal/` 配下に置く
- `hal/` は `m5::hal::*`、 `variants/` は `m5::variants::*` に対応させる
- **例外 (variant 公開型)**: variant が提供する公開型 (`Bus_<variant>` / `BusConfig_<variant>` / gpio の `Port_<variant>` 等) は、 物理ファイルを `variants/` 配下に置いたまま **namespace は `m5::hal::v2::<kind>` 直下**に定義する ([design/variants.md](design/variants.md) §offer 要件)。 suffix が variant の出自を示し、 勝者選択は無印名への型 alias で行う。 variant 固有の内部構造 (service 群・レジスタ層・固有ユーティリティ) は従来どおり `m5::variants::*` に置く
- `m5::hal::v2::<kind>::*` 直下の variant 由来 free function (例: `gpio::getGPIO()`) は bootstrap 用の内部 seam として位置付ける。 v2 caller が直接呼ぶ主入口ではない
- `src/m5_hal/hal/v2/m5_hal.hpp` は HAL object 層を提供する
- 詳細な配置規約は [reference/directory-layout.md](reference/directory-layout.md) を参照

## 参照

- [design/bus_accessor.md](design/bus_accessor.md)
- [design/runtime.md](design/runtime.md)
- [design/data_io.md](design/data_io.md)
- [design/transfer_desc.md](design/transfer_desc.md)
- [design/variants.md](design/variants.md)
- [design/gpio.md](design/gpio.md)
- [design/i2c.md](design/i2c.md)
- [reference/directory-layout.md](reference/directory-layout.md)
