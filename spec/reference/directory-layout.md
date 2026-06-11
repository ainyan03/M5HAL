# directory-layout — ディレクトリ階層と namespace 1:1 規約

M5HAL では、 ライブラリのディレクトリ階層と namespace 階層の 1:1 対応を以下の規約で取る。

## version 階層

`src/m5_hal/hal/` 配下は `v0/` と `v1/` の 2 sub directory を持つ。

- `src/m5_hal/hal/v0/` ⇔ `m5::hal::v0::*`
- `src/m5_hal/hal/v1/` ⇔ `m5::hal::v1::*`

`hal/v0/` は freeze 例外とし、 v1 側の配置規約は適用しない。 `variants/` 配下は v1 用のみとする。
`hal/v0/` 内に残る TODO コメントは、 取り込み元との差分を小さく保つため現行 v1 作業のタスク対象外として扱い、 コメント形式の整備目的だけでは書き換えない。

### 適用範囲

| 対象 | 版の持ち方 | 備考 |
|---|---|---|
| `src/m5_hal/hal/` | `hal/v0/`, `hal/v1/` | namespace と対応 |
| `src/m5_hal/variants/` | 版ディレクトリを持たない | variant 機構は v1 用 |
| `src/M5HAL_{v0,v1}.{hpp,cpp}` | ファイル名 suffix | entry を版別に分離 |
| `examples/` | `examples/v1/` | サンプルを版別管理 |
| `test/` | `test/v0/`, `test/v1/` | 検証を版別管理。 `native/` は host test、 `embedded/` は実機で PASS/FAIL を判定する test |
| `docs/` | Doxygen 用に別管理 (将来作成) | Markdown 仕様書の置き場には使わない |

## v1 構造

```text
src/
  M5HAL.hpp
  M5HAL_v0.{hpp,cpp}
  M5HAL_v1.{hpp,cpp}
  m5_hal/
    _macro/
      offer_all.inl
    hal/
      v1/
        bus/bus.{hpp,inl}
        i2c/i2c.{hpp,inl}
        i2c/slave.hpp
        i2s/i2s.{hpp,inl}
        spi/spi.{hpp,inl}
        uart/uart.{hpp,inl}
        gpio/gpio.hpp
        gpio/port.hpp
        gpio/group.hpp
        data.hpp
        data/memory.hpp
        data/limited.hpp
        data/stream.hpp
        frame/frame.{hpp,inl}
        bytecode/bytecode.{hpp,inl}
        remote/remote.{hpp,inl}
        memory/allocator.{hpp,inl}
        memory/pool.{hpp,inl}
        service/service.hpp
        types.hpp
        error.hpp
        m5_hal.hpp
    variants/
      frameworks/
        _checker.hpp
        <name>/
          _offer.hpp
          hal.hpp
          hal.inl
          hal/<kind>/<kind>.{hpp,inl}
      platforms/
        _checker.hpp
        <vendor>/<chip-family>/
          _offer.hpp
          hal.hpp
          hal.inl
          hal/<kind>/<kind>.{hpp,inl}
```

## 規約

1. `src/m5_hal/hal/` 配下は `m5::hal::*` と厳密に対応させる
2. `src/m5_hal/variants/` 配下は `m5::variants::*` と厳密に対応させる
3. ライブラリルート `src/m5_hal/` は `m5` ルートに対応する例外とする
4. `_macro/`, `_checker.hpp`, `_offer.hpp` などのメタ要素は namespace 非対応の例外とする
5. HAL の範疇外要素は `hal/` の外に置く
6. cross-cutting な型 (`error_t` 等) は `m5::hal::` 直下に置く

## `hal/<kind>` のパターン

| パターン | 用途 | 例 |
|---|---|---|
| `hal/<kind>.hpp` | 派生のない型定義 | `types.hpp`, `error.hpp` |
| `hal/<kind>.hpp` + `hal/<kind>/*.hpp` | 親抽象 + 派生具象 | `data.hpp`, `data/memory.hpp` |
| `hal/<kind>/<kind>.{hpp,inl}` | 抽象と具象が密結合 | `i2c/i2c.{hpp,inl}`, `bus/bus.{hpp,inl}` |

ファイル名 (拡張子除く) は namespace 名と一致させる。 別 namespace を与えたい派生はディレクトリで階層化する。

## namespace 宣言形式

C++17 nested namespace specifier を使う。

```cpp
namespace m5::variants::frameworks::arduino::hal::v1::i2c {
...
}  // namespace m5::variants::frameworks::arduino::hal::v1::i2c
```

例外として `_macro/offer_all.inl` はマクロ展開の都合で 1 行ネスト形式を許容する。

## variant 内の構造規約

variant の内部構造 (`_offer.hpp` + `hal.hpp` + `hal.inl` の hub 構成) は
[../design/variants.md](../design/variants.md) §variant 内部の構造 が正本。
ここでは配置・namespace 面の規約だけ補足する。

- variant 内の HAL 提供物は `namespace hal::v1::<kind> { ... }` に置く
- variant 内では必要に応じて `using namespace ::m5::hal::v1;` を置いて unqualified 参照を救済する
- `types::*` は `::m5::hal::v1::types::*` のように fully qualified で書く

## include 形式

- 同一ライブラリ内はダブルクォート + 相対パスを使う
- search path 前提の絶対パス的記述は避ける
- 公開エントリからは `./m5_hal/...` の明示パスを許容する

## 検出機構

- `src/m5_hal/variants/frameworks/_checker.hpp` — `M5HAL_FRAMEWORK_HAS_<NAME>` 系
- `src/m5_hal/variants/platforms/_checker.hpp` — `M5HAL_V1_PLATFORM_NUMBER_*`, `M5HAL_V1_TARGET_PLATFORM_*` 系 (無印は凍結 v0 が所有)

## 公開パッケージの除外 (idf_component.yml / library.json)

レジストリへ公開するパッケージには「ライブラリ本体」 (`src/` + 公開ヘッダ + `CMakeLists.txt` + `README*` + `LICENSE`) だけを含め、 開発専用物 (test / 内部仕様 / CI 設定など) は除外する。 2 つのエコシステムで **examples の扱いだけ非対称** にする。

| マニフェスト | エコシステム | examples | 除外する主なもの |
|---|---|---|---|
| `idf_component.yml` (`files.exclude`) | ESP-IDF Component Registry | **除外** | docs / examples / experiments / test / spec / pio_envs / .github / boards / platformio.ini / library.json / library.properties |
| `library.json` (`export.exclude`) | PlatformIO Registry | **残す** | experiments / test / spec / pio_envs / .github / platformio.ini / idf_component.yml |

**examples を非対称にする理由**: Arduino / PlatformIO では examples がライブラリ体験の一部 (Arduino IDE の File > Examples、 ライブラリマネージャ) なのでパッケージに残す。 ESP-IDF コンポーネントは純粋なビルド対象ソースで examples をビルド・消費しないため除外して lean に保つ (公式 m5stack/M5HAL の `idf_component.yml` も examples を除外している)。 `idf_component.yml` は公式の除外リストをベースに M5HAL 固有の開発ディレクトリ (experiments / spec / pio_envs / platformio.ini) を足して発展させたもの。

`library.json` は **JSON のためファイル内にコメントを書けない**。 除外方針の根拠は本節を正本とする (`idf_component.yml` 側にも本節を指すコメントを置く)。

## 関連

- [../architecture.md](../architecture.md)
- [../design/variants.md](../design/variants.md)
- [../style/coding_style.md](../style/coding_style.md)
