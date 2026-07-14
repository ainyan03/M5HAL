# directory-layout — ディレクトリ階層と namespace 1:1 規約

> **読者**: メンテナ向け（ビルド・運用・規約）。

M5HAL では、 ライブラリのディレクトリ階層と namespace 階層の 1:1 対応を以下の規約で取る。

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
| `docs/` | Doxygen 用に別管理 (将来作成) | Markdown 仕様書の置き場には使わない |

## v2 構造

```text
src/
  M5HAL.hpp
  M5HAL_v0.{hpp,cpp}
  M5HAL_v2.{hpp,cpp}
  m5_hal_config.hpp
  m5_hal/
    _macro/
      offer_all.inl
      offer_kind.inl
      offer_runtime_only.inl
    hal/
      v2/
        runtime/runtime.hpp
        bus/allocation_core.{hpp,inl}
        bus/bus.{hpp,inl}
        bus/bus_view.hpp
        bus/hal_backend.hpp
        bus/hw_pool.hpp
        bus/local_backend.{hpp,inl}
        bus/managed_bus.hpp
        bus/managed_facade.hpp
        bus/registry.hpp
        i2c/i2c.{hpp,inl}
        i2c/master_clock_limit.hpp
        i2c/slave.{hpp,inl}
        i2c/virtual_bus.hpp
        i2s/i2s.{hpp,inl}
        spi/slave.hpp
        spi/spi.{hpp,inl}
        uart/bus_console.{hpp,inl}
        uart/bus_streaming.{hpp,inl}
        uart/uart.{hpp,inl}
        gpio/gpio.hpp
        gpio/group.{hpp,inl}
        gpio/port.hpp
        data.hpp
        data/block.hpp
        data/memory.hpp
        data/limited.hpp
        data/mux.{hpp,inl}
        data/ring.{hpp,inl}
        data/stdio.hpp
        data/stream.{hpp,inl}
        data/tap.hpp
        frame/frame.{hpp,inl}
        bytecode/bytecode.{hpp,inl}
        remote/credit_notifier.hpp
        remote/remote.hpp
        remote/remote_connection.{hpp,inl}
        remote/session_handle.hpp
        remote/server.{hpp,inl}
        remote/server_adapter.{hpp,inl}
        remote/server_bus_pool.{hpp,inl}
        remote/server_connection_wiring.{hpp,inl}
        remote/server_handler.{hpp,inl}
        remote/wire_drain.hpp
        memory/allocator.{hpp,inl}
        memory/pool.{hpp,inl}
        service/completion_gate.hpp
        service/service.{hpp,inl}
        types.hpp
        error.hpp
        assert.hpp
        diag.hpp
        m5_hal.hpp
    variants/
      frameworks/
        _checker.hpp
        freertos/
          _offer.hpp
          hal/runtime/mutex.hpp
          hal/runtime/task.hpp
          hal/runtime/time.hpp
        bsd/
          hal/remote/tcp_server.{hpp,inl}
          hal/tcp/bsd_tcp.{hpp,inl}
        remote/
          _offer.hpp
          backend.{hpp,inl}
          bus_lease.hpp
          detail_helpers.hpp
          hal.hpp
          hal/gpio/gpio.{hpp,inl}
          hal/i2c/i2c.{hpp,inl}
          hal/i2s/i2s.{hpp,inl}
          hal/spi/spi.{hpp,inl}
          hal/uart/uart.{hpp,inl}
          remote_transfer.{hpp,inl}
          session.{hpp,inl}
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
4. `_macro/`, `_checker.hpp`, `_offer.hpp` などのメタ要素は namespace 非対応の例外とする。
   variant 横断の共有実装 (FreeRTOS OS プリミティブ、 BSD socket TCP) は `variants/frameworks/`
   配下に独立 variant として配置する (`freertos/`, `bsd/`)
5. HAL の範疇外要素は `hal/` の外に置く
6. cross-cutting な型 (`error_t` 等) は `m5::hal::` 直下に置く

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

- variant 内の HAL 提供物は `namespace hal::v2::<kind> { ... }` に置く
- `m5::hal::v2::<kind>` 内 (公開型の定義場所) では `::m5::hal::v2::` を省略し相対名で書く (`result_t<T>` / `bus::IAccessor` 等)。 ただし `detail::` は sibling kind の同名 namespace と曖昧になるためフル修飾を維持する
- `m5::variants::...` 内 (内部構造) では `::m5::hal::v2::` が探索経路にないためフル修飾が必要。 `using namespace ::m5::hal::v2;` で省略も可

## include 形式

- 同一ライブラリ内はダブルクォート + 相対パスを使う
- search path 前提の絶対パス的記述は避ける
- 公開エントリからは `./m5_hal/...` の明示パスを許容する

## 検出機構

詳細は [../design/variants.md](../design/variants.md) §走査順 (`M5HAL_v2.hpp` 内) を参照。本 kind 固有の差分のみ以下に示す。

- `src/m5_hal/variants/frameworks/_checker.hpp` — `M5HAL_FRAMEWORK_HAS_<NAME>` 系
- `src/m5_hal/variants/platforms/_checker.hpp` — `M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID` / `M5HAL_V2_DETECTED_PLATFORM_VARIANT_PATH` の検出 (識別番号の正本は `variants/ids.hpp`。 無印は変更不可の v0 が所有)

## 関連

- [../architecture.md](../architecture.md)
- [../design/variants.md](../design/variants.md)
- [../style/coding_style.md](../style/coding_style.md)
