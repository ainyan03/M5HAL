# architecture — M5HAL v2 全体構造

> **読者**: 実装者・レビュー向け（設計仕様）。

M5HAL v2 API の全体構造と設計原則を示す。 個別の設計は [design/](design/) 配下を参照。

## 設計原則

- **組み込みファースト** — RTTI / 例外に依存しない
- **シンプル優先** — 状態機械や補助状態は必要最小限にとどめる
- **宣言と実装の分離** — ヘッダは宣言を中心に置き、 重い実装は `.inl` 等に分離する
- **同期 API を正本とする** — 非同期機構を追加する場合も同期契約を正本として維持する
- **テストで契約を担保する** — 抽象基底の契約はユニットテストで検証する
- **無駄を省く最適化は迷わず行う** — 不要なコピーや不要な状態を省けば、 実装は単純になり速度も上がる (§最適化の判断)
- **汎用性や単純さを犠牲にする最適化は、 要件を満たせないと実測で示せたときだけ** (§最適化の判断)
- **先行整備をしすぎない** — 実需が見えていない抽象は導入しない

## 最適化の判断

1. **無駄を省く** (不要なコピー・不要な状態) — 単純さと速さは両立する。 迷わず行う。
2. **汎用性や単純さを犠牲にして速くする** — 要件を満たせないことを実測で示せたときだけ行う。
   「その方が速いから」は理由にならない。
3. **どちらの場合も、 API の形に出る選択は理由をここか design/ に書く。** API の形に出た選択は、
   利用者のコードがそれに合わせて書かれるため、 あとから消せない。

例:

- `data::Source::peek` が借用 Span を返すのは **1**。 RAM / ROM に既にあるデータを無駄に複製せず、
  ストリームのデータも同じ型で扱える。 ただし借用の lifetime 規則は API の形に出るため **3** が要る
  ([design/data_io.md](design/data_io.md) §Source の契約)
- `PinBackup` の IO_MUX 配列 weak シンボル化は実装の内側に閉じる。 利用者のコードは変わらないので
  **3** は不要 — 明日ふつうの配列に戻しても誰も気づかない

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

**既定の v2 caller 入口は `m5::hal::v2::M5_Hal` object** (`m5::hal::v2::Hal&` 型)。
`Hal` はローカルとリモートで共通の public facade で、sub-object (`Gpio` / `I2C` / `SPI` /
`UART` / `I2S` / `Services` / `Memory`) を束ねる。ローカル資源の所有単位はcopy可能な
`ResourceDomain` facadeのco-owned stateであり、`BusRegistry`、`GPIOGroup`、`ServiceRunner`、
`Allocator`を一つのidentity namespaceに置く。`Hal()`は独立domainを作り、`Hal(domain)`は指定domainを
共有する。`init()` / `connect("local")` はそのHalのdomainへlocal backendとadapterを束縛する。
束縛経路での自 variant GPIO 登録 (`addGPIO`) の失敗は invariant break であり、戻り値を握り潰さず
fail-fast で顕在化させる (正しく束ねられた variant では必ず成功するため、失敗 = 構成バグの即時検出)。

同一domainの同一`ResourceKey`は同じBus/lockへinternされ、別domainではkey値が同じでも別Busになる。
`ResourceKey`のproduction ABIは全対応targetで32 B固定とし、target幅でidentity表現やregistry layoutを
分岐させない。
Busとfactoryはdomain stateをco-ownするため、取得済みBusはstack上の`Hal` / `ResourceDomain` facadeより
長生きできる。`M5_Hal`はこの一般形のdefault-domain便利入口であり、local Halをprocess内一個に限定しない。

リモート `Hal` は接続確立後に同型 API を提供する: `Hal remote; remote.connect(endpoint)` で接続する
(endpoint = `"uart:<path>"` / `"tcp:<host>:<port>"`。typed API `remote.initUart(port)` /
`remote.initTcp("host:port")` も存続)。以降は同じ `remote.I2C.acquire(cfg)` /
`remote.SPI.acquire(cfg)` の形で proxy bus を取得でき、ローカル `Hal` と同一のコード面で使える。
1 `Hal` は 1 接続先に束縛され、reconnect は旧 session を失効させる。旧 proxy は connection を
延命せず `CLOSED` を返し、同一 session の RPC は session gate で直列化される。明示closeは
exact-instance の唯一所有 handle を消費する。詳細は [design/remote.md](design/remote.md) と
[design/bus_accessor.md](design/bus_accessor.md) を参照。`M5HALCore` はdefault `M5_Hal` facadeを構築する
bootstrap singletonで、個々のlocal backendは各`Hal`のlocal connection stateが所有する。default利用では
従来どおり`m5::hal::v2::M5_Hal.Gpio.*`の形で各sub-objectへアクセスする。

取得済みBusは全kind共通の`capabilities()`で、現在のinstanceが実装するoperation、数値limit、
backend/session generationの固定長snapshotを返す。local/remoteのgeneric preflightはこのsnapshotを正本とする。
詳細は[design/bus_capabilities.md](design/bus_capabilities.md)。

`Hal` は copy / move ともに不可。ローカル singleton を移動させず、remote connection や proxy cache の
非自明な状態を複製・移送しないためである。関数へは `Hal&` で渡し、所有が必要なら
`std::unique_ptr<Hal>` を使う。

M5HAL は board ID や board preset catalog を持たない。board 固有の pin / bus preset は M5Unified や
BSP 等の上位層がデータとして保持し、既存の `BusConfig` へ供給する。`variant_id_t` は backend 実装を
識別する型であり、board 識別子を同じ番号空間へ混在させない。

`m5::hal::v2::<kind>::*` 配下の勝者バインドされた関数 (例: `gpio::getGPIO()`) はlocal connectionの
bootstrap seamとして内部利用するため、callerが直接呼ぶ必要はない (詳細は
[design/gpio.md](design/gpio.md) §caller 向けentry point)。

namespace-scope initializer や他ライブラリの global ctor から触る場合は `getM5_Hal()` を使う (`Hal&` を返す。 `M5_Hal` alias は eager-init のため lazy-safe ではない、 lazy-safe accessor は `getM5_Hal()` のみ)。

## namespace と配置

### namespace 帰属

- HAL の範疇は `m5::hal::*` 配下に置く (runtime 設備も `m5::hal::v2::runtime` の 1 kind)
- core (予約層) は `m5::*` 直下の別 namespace に置く
- cross-cuttingな型もAPI世代に属する間は`m5::hal::vN::*`配下に置く。現行`error_t`は
  `m5::hal::v2::error::*`で、世代非依存の`m5::hal::*`直下へはまだ置かない
- variant 機構は `m5::variants::*` に置く
- ライブラリは global な短縮 namespace alias `m5hal` を定義しない。例や利用コードで使う場合は
  `namespace m5hal = m5::hal::v2;` のように caller 自身のスコープで定義する。取り消しにくい global 名を
  公式 API として予約せず、世代の既定切替は inline namespace の仕組みに委ねる

v0/v2共存による物理namespace (`m5::hal::v2::<kind>`等) とinline展開の詳細は
[design/v0_v2_coexistence.md](design/v0_v2_coexistence.md) §namespace 配置 を参照。

### ディレクトリ配置

物理配置は namespace に対応させる。配置、include 形式、主要ファイルの検索先は
[reference/directory-layout.md](reference/directory-layout.md) を正本とする。variant が公開する
provider symbolだけは、物理ファイルを`variants/`配下に置いたまま`m5::hal::v2::<kind>`へ公開する。
この例外の意味論と公開symbolは[design/variants.md](design/variants.md)
§offer 要件 (facade bus kind)を参照。

## 参照

- [design/bus_accessor.md](design/bus_accessor.md)
- [design/bus_capabilities.md](design/bus_capabilities.md)
- [design/runtime.md](design/runtime.md)
- [design/data_io.md](design/data_io.md)
- [design/transfer_desc.md](design/transfer_desc.md)
- [design/variants.md](design/variants.md)
- [design/gpio.md](design/gpio.md)
- [design/i2c.md](design/i2c.md)
- [reference/directory-layout.md](reference/directory-layout.md)
