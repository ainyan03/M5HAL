# directory-layout — ディレクトリ階層と namespace 1:1 規約

> **読者**: メンテナ向け（ビルド・運用・規約）。

M5HALの物理tree、配置規約、include先の検索台帳を示す。variantの選択・申告契約は
[../design/variants.md](../design/variants.md)、追加手順は
[../porting_guide/framework.md](../porting_guide/framework.md)を参照。

## version 階層

`src/m5_hal/hal/` 配下は `v0/` と `v2/` の 2 sub directory を持つ。

- `src/m5_hal/hal/v0/` ⇔ `m5::hal::v0::*`
- `src/m5_hal/hal/v2/` ⇔ `m5::hal::v2::*`

`hal/v0/` は freeze 例外とし、 v2 側の配置規約は適用しない。 `variants/` 配下は v2 用のみとする。
`hal/v0/` 内に残る TODO コメントは、 取り込み元との差分を小さく保つため現行 v2 作業のタスク対象外として扱い、 コメント形式の整備目的だけでは書き換えない。

### 適用範囲

| 対象 | 版の持ち方 | 備考 |
|---|---|---|
| `src/m5_hal/hal/` | `hal/v0/`, `hal/v2/` | namespace と対応 |
| `src/m5_hal/variants/` | 版ディレクトリを持たない | variant 機構は v2 用 |
| `src/M5HAL_{v0,v2}.{hpp,cpp}` | ファイル名 suffix | entry を版別に分離 |
| `examples/` | `examples/v2/` | サンプルを版別管理 |
| `test/` | `test/v0/`, `test/v2/` | 検証を版別管理。 `native/` は host test、 `embedded/` は実機で PASS/FAIL を判定する test |
| `docs/` | Doxygen 用に予約 (現行未作成) | Markdown 仕様書の置き場には使わない |

## v2 構造と検索先

```text
src/
  M5HAL.hpp
  M5HAL_v0.{hpp,cpp}
  M5HAL_v2.{hpp,cpp}
  m5_hal_config.hpp
  m5_hal/
    _macro/                         variant勝者binding
      offer_all.inl
      offer_kind.inl
      offer_runtime_only.inl
    hal/
      v2/
        <kind>/<kind>.{hpp,inl}      kindの契約と共通実装
        bus/                        Bus共通基盤、registry、ownership
        data/                       Source / Sink具象
        remote/                     remote protocolとserver
        memory/                     allocator / pool
        service/                    background service
        resource_domain.hpp         local資源domain
        {types,error}.hpp           API世代に属する共通型
        m5_hal.hpp                  HAL object層
    variants/
      ids.hpp                       variant ID台帳
      frameworks/
        _checker.hpp                framework検出
        <name>/
          _offer.hpp                capability申告
          hal.{hpp,inl}             kind別実装hub
          hal/<kind>/<kind>.{hpp,inl}
      platforms/
        _checker.hpp                platform検出
        <vendor>/<chip-family>/
          _offer.hpp                capability申告
          hal.{hpp,inl}             kind別実装hub
          hal/<kind>/<kind>.{hpp,inl}
```

| 探すもの | 正本・入口 |
|---|---|
| public umbrellaとscan順 | `src/M5HAL_v2.hpp` / `src/M5HAL_v2.cpp` |
| build設定macro | `src/m5_hal_config.hpp` |
| variant ID | `src/m5_hal/variants/ids.hpp` |
| framework / platform検出 | `src/m5_hal/variants/{frameworks,platforms}/_checker.hpp` |
| capability申告 | 各variantの`_offer.hpp` |
| 勝者binding | `src/m5_hal/_macro/offer_all.inl`と`offer_kind.inl` |
| kind共通契約 | `src/m5_hal/hal/v2/<kind>/` |
| provider実装 | `src/m5_hal/variants/{frameworks,platforms}/.../hal/<kind>/` |

## 規約

1. `src/m5_hal/hal/` 配下は `m5::hal::*` と厳密に対応させる
2. `src/m5_hal/variants/` 配下は、下記のprovider公開symbolを除き`m5::variants::*`と対応させる
3. ライブラリルート `src/m5_hal/` は `m5` ルートに対応する例外とする
4. `_macro/`, `_checker.hpp`, `_offer.hpp` などのメタ要素は namespace 非対応の例外とする。
   variant 横断の共有実装 (FreeRTOS OS プリミティブ、 BSD socket TCP) は `variants/frameworks/`
   配下に独立 variant として配置する (`freertos/`, `bsd/`)
5. HAL の範疇外要素は `hal/` の外に置く
6. cross-cuttingな型の配置規則は [../architecture.md](../architecture.md) §namespace 帰属 を参照

## 公開パッケージの除外 (idf_component.yml / library.json)

レジストリへ公開するパッケージには「ライブラリ本体 + 利用者向けドキュメント」 (`src/` + 公開ヘッダ + `CMakeLists.txt` + `README*` + `LICENSE` + `spec/`) を含め、 開発専用物 (test / CI 設定など) は除外する。 2 つのエコシステムで **examples の扱いだけ非対称** にする。

| マニフェスト | エコシステム | examples | 除外する主なもの |
|---|---|---|---|
| `idf_component.yml` (`files.exclude`) | ESP-IDF Component Registry | **除外** | docs / examples / test / pio_envs / .github / boards / platformio.ini / library.json / library.properties |
| `library.json` (`export.exclude`) | PlatformIO Registry | **残す** | test / pio_envs / .github / platformio.ini / idf_component.yml |

**spec/ を残す理由**: レジストリ経由の利用者が README から仕様文書へ辿れるようにする (除外するとパッケージ内のリンクが行き止まりになる)。 `spec/` は確定仕様のみを置く公開文書ツリーであり、 開発専用物ではない。

**examples を非対称にする理由**: Arduino / PlatformIO では examples がライブラリ体験の一部 (Arduino IDE の File > Examples、 ライブラリマネージャ) なのでパッケージに残す。 ESP-IDF コンポーネントは純粋なビルド対象ソースで examples をビルド・消費しないため除外して lean に保つ (公式 m5stack/M5HAL の `idf_component.yml` も examples を除外している)。 `idf_component.yml` は公式の除外リストをベースに M5HAL 固有の開発ディレクトリ (`pio_envs` / `platformio.ini`) を足して発展させたもの。

`library.json` は **JSON のためファイル内にコメントを書けない**。 除外方針の根拠は本節を正本とする (`idf_component.yml` 側にも本節を指すコメントを置く)。

## `hal/<kind>` のパターン

| パターン | 用途 | 例 |
|---|---|---|
| `hal/<kind>.hpp` | 派生のない型定義 | `types.hpp`, `error.hpp` |
| `hal/<kind>.hpp` + `hal/<kind>/*.hpp` | 親抽象 + 派生具象 | `data.hpp`, `data/memory.hpp` |
| `hal/<kind>/<kind>.{hpp,inl}` | 抽象と具象が密結合 | `i2c/i2c.{hpp,inl}`, `bus/bus.{hpp,inl}` |

ファイル名 (拡張子除く) は namespace 名と一致させる。 別 namespace を与えたい派生はディレクトリで階層化する。

## namespace 宣言形式

詳細は [../style/coding_style.md](../style/coding_style.md) §namespace 宣言形式 を参照。

## variant 内の構造規約

variant の内部構造 (`_offer.hpp` + `hal.hpp` + `hal.inl` の hub 構成) は
[../design/variants.md](../design/variants.md) §variant 内部の構造 が正本。
ここでは配置・namespace 面の規約だけ補足する。

- provider公開symbolは物理的にはvariant配下に置き、`m5::hal::v2::<kind>`へ定義する。対象symbolと
  選択規則は[../design/variants.md](../design/variants.md) §offer 要件 (facade bus kind)が正本
- `m5::hal::v2::<kind>` 内 (公開型の定義場所) では `::m5::hal::v2::` を省略し相対名で書く (`result_t<T>` / `bus::IAccessor` 等)。 ただし `detail::` は sibling kind の同名 namespace と曖昧になるためフル修飾を維持する
- provider-privateな内部構造は`m5::variants::...`へ置く。このnamespaceでは`::m5::hal::v2::`が
  探索経路にないためフル修飾が必要。`using namespace ::m5::hal::v2;`で省略も可

## include 形式

- 同一ライブラリ内はダブルクォート + 相対パスを使う
- search path 前提の絶対パス的記述は避ける
- 公開エントリからは `./m5_hal/...` の明示パスを許容する

## 検出機構

詳細は [../design/variants.md](../design/variants.md) §走査順 (`M5HAL_v2.hpp` 内) を参照。
検索入口は以下の2ファイルとする。

- `src/m5_hal/variants/frameworks/_checker.hpp` — `M5HAL_FRAMEWORK_HAS_<NAME>` 系
- `src/m5_hal/variants/platforms/_checker.hpp` — `M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID` / `M5HAL_V2_DETECTED_PLATFORM_VARIANT_PATH` の検出 (識別番号の正本は `variants/ids.hpp`。 無印は変更不可の v0 が所有)

## 関連

- [../architecture.md](../architecture.md)
- [../design/variants.md](../design/variants.md)
- [../style/coding_style.md](../style/coding_style.md)
