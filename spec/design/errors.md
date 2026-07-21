# design/errors — エラーコードと対処の手引き

> **読者**: 利用者向け。

v2 HAL の**失敗し得るoperation**は `result_t<T>`
(`m5::stl::expected<T, error::error_t>` のpure alias、
[../style/coding_style.md](../style/coding_style.md) §型選択) で値かエラーコードを返す。
状態queryは`bool`または値型、契約上失敗しないcommandは`void`を使う。例えば
`runtime::Task::start()`と`runtime::Mutex::lock/unlock()`は`result_t<void>`だが、冪等な`join()`は
`void`、`joinable()`は`bool`である。
コードの定義は `hal/v2/error.hpp` の`m5::hal::v2::error::error_t`。

- **負値 = 失敗**、`OK` (0) と `ASYNC_RUNNING` (1) は非失敗

## エラー対処 hint 表

「このコードを見たらまず何を疑うか」の早見表。コードの**意味**は
error.hpp の doc コメントが正本で、ここは現場の確認手順を足す。

| コード | まず疑うこと |
|---|---|
| `TIMEOUT_ERROR` | **lock 競合**: 他タスク/別 accessor がバスを保持したまま (有限 timeout を渡した場合のみ — 既定は無限待ち)。**ワイヤ側**: I2C のクロックストレッチが `wire_timeout_ms` を超過、リモートの応答遅延 |
| `INVALID_ARGUMENT` | API 契約違反: unbound accessor の使用 (release ビルド)、範囲外の引数 (`register_address_bytes` 等)、owner 不一致の `unlock`。BusView の明示closeではnull、foreign bus、または同一identityの別実体 |
| `INVALID_STATE` | 操作自体は有効だが現在のオブジェクト状態が許可しない: アクセスウィンドウが開いたままの `setConfig`/`bind`、対応する `begin` のない `endAccess`、Remote/Bytecode の登録可能 bus slot / GPIOGroup が未登録 |
| `NOT_CONNECTED` | `Hal` が未束ね、未束ねBusViewでacquire / commit / closeした、または remote 専用操作に remote connection が無い。`connect()` / `init()` 後に再試行する |
| `NOT_IMPLEMENTED` | 選択した build / variant に、その API を実行する実装経路自体が存在しない。選択 variant の確認は `M5HAL_V2_SELECTED_VARIANT_<KIND>` ([variants.md](variants.md) §診断) |
| `UNSUPPORTED` | API と要求は有効だが、現在の instance / config / peer がその操作・mode・capability・limit を提供しない。`capabilities()` と構成を確認する |
| `I2C_NO_ACK` | アドレス違い (7bit 表記か確認)・プルアップ欠落・配線・デバイス未給電。`bus.probe(addr)` でのスキャンが切り分けの最短 |
| `I2C_BUS_ERROR` | アービトレーション喪失 (マルチマスタ)・SDA/SCL の張り付き (デバイスのリセットで解消することがある)・ノイズ |
| `BUSY` | (現行 v2 の lock 競合は `TIMEOUT_ERROR`。`BUSY` はそれ以外の「使用中」資源)。BusView の明示closeではAccessor/aliasがco-own中、同一identityがclose中、または自然終了を確認できずtombstone隔離中 |
| `IO_ERROR` | OS/ドライバ層の失敗: デバイスパス・権限 (posix)、ドライバの未初期化、USB シリアルの切断 |
| `CLOSED` | ストリーム/接続の終端。リモートバスでは transport 切断 (peer hang-up 含む) に加え、reconnect/local 復帰で旧 session handle が失効した proxy 操作、close成功後にweakから復活した旧proxy操作を表す。新しい`Hal` bindingからacquireし直す (`bsd_tcp.hpp:54`参照。UART transportでは抜線は`IO_ERROR`・無応答は`TIMEOUT_ERROR`として現れる) |
| `DISCONNECTED` | **将来予約** — 現状は `CLOSED` が transport 切断の役割を担う。`DISCONNECTED` は peer が明示的に切断を通知する高水準プロトコル層向けに確保 |
| `END_OF_STREAM` | ストリームの clean な終端。再データは来ない (`frame.inl:177` 等が実際に返す。ringbuffer Sink 等が返す EOF 表現) |
| `UNKNOWN_ERROR` | last-resort fallback。`data_io.md` の「可能な限り `UNKNOWN_ERROR` へ潰さず recoverable error の意味を保つ」指針の通り、原因が特定できない場合にのみ使う |
| `OUT_OF_RESOURCE` | 固定スロット (BusRunner の登録枠等)・ハンドル・メモリプールの枯渇 |
| `BUFFER_OVERFLOW` / `BUFFER_UNDERFLOW` | 供給と消費のサイズ不一致。フレーム長とバッファ長の突き合わせ |
| `CHECKSUM_ERROR` / `PROTOCOL_ERROR` | ワイヤ品質 (baud を下げて切り分け)・世代不一致の peer (フレームは正しいが意味が違う) |
| `REMOTE_FAULT` | リモート側が内部失敗を報告した。peer のログを確認する ([remote.md](remote.md)) |
| `DEVICE_MISMATCH` | 期待したチップではない: chip-ID レジスタの読み値と期待値、同一アドレスの別デバイス (互換品・リビジョン違い)。**HAL コアは返さない** — ドライバ層 (M5UU 等) が ID 照合の結果として使う予約コード |

## toString (ログ用の名前)

`error::toString(error_t)` は列挙子の綴り (`"I2C_NO_ACK"`) を返す
constexpr 関数。未知の値 (新しい世代の peer からワイヤ越しに届いた
コード等) は `"UNKNOWN"`。文字列は **grep できる安定識別子**であって
散文ではない — 説明は本ページの hint 表が担う。

```cpp
if (auto r = dev.readRegister(REG_ID); !r.has_value()) {
    Serial.printf("read failed: %s\n", m5::hal::v2::error::toString(r.error()));
}
```

文字列テーブルが惜しいビルドは `-DM5HAL_CONFIG_ERROR_STRINGS=0` で
テーブルを落とせる (`toString` は常に `""` を返す。
[configuration.md](configuration.md) に登録)。

## エラー伝播 cookbook

`result_t<T>` は `m5::stl::expected` の pure alias なので、その monadic
API (`and_then` / `or_else` / `map` 等) がそのまま使える。場面別の正解形:

**1. 素の早期 return** — 分岐ごとに別の対処をする場面の基本形:

```cpp
auto r = dev.readRegister(REG_STATUS);
if (!r.has_value()) {
    Serial.printf("status read failed: %s\n", m5hal::error::toString(r.error()));
    return;
}
uint8_t status = r.value();
```

**2. `and_then` 連鎖** — 「前段が成功したら次へ、失敗したらそのまま伝播」を
連ねる場面。init シーケンスのような直列ステップが 1 式になる:

```cpp
m5hal::result_t<void> initSensor(m5hal::i2c::MasterAccessor& dev)
{
    return dev.writeRegister(REG_RESET, uint8_t{0x01})
        .and_then([&](size_t) { return dev.writeRegister(REG_MODE, uint8_t{0x03}); })
        .and_then([&](size_t) -> m5hal::result_t<void> { return {}; });
}
```

**3. `result_t<size_t>` → `result_t<void>` 変換** — 「バイト数はもう用済み、
成否だけ上に返す」場面。連鎖の最後に空 lambda を 1 つ足す (上の例の最終段)。
単独なら:

```cpp
m5hal::result_t<void> writeAll(m5hal::uart::TxAccessor& tx, m5hal::data::ConstDataSpan span)
{
    auto r = tx.write(span);
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    return {};
}
```

**4. `or_else` でのフォールバック** — 失敗時に代替経路を試す場面:

```cpp
auto id = dev.readRegister(REG_ID_PRIMARY)
              .or_else([&](m5hal::error::error_t) { return dev.readRegister(REG_ID_ALT); });
```

TRY 風の伝播マクロは提供しない (式の中に return を隠すコストが、現状の
利用規模では節約を上回る — 実害が顕在化したら再訪)。begin/end の対で
囲む処理は [bus_accessor.md](bus_accessor.md) §guarded が cookbook の
続きにあたる。

## ABI 契約 — i8 範囲と値の変更不可

基底型 `int8_t` はワイヤ形式との契約を兼ねる。bytecode ワイヤが
エラーコードを 1 byte (i8) で運ぶ仕組みは
[bytecode.md](bytecode.md) §ワイヤフォーマット (M5HAL bytecode v1) を参照。
ここでは値域と安定性の規約のみを定める:

- **-128..127 の外に値を足さない** (i8 で表現できる範囲が上限)
- 値は公開リリースに載った時点で変更不可 (改番・再利用禁止)

## 関連

- [bus_accessor.md](bus_accessor.md) — 排他制御の意味論 (TIMEOUT_ERROR の由来)
- [bytecode.md](bytecode.md) — エラーコードのワイヤ搬送 (i8)
- [configuration.md](configuration.md) — `M5HAL_CONFIG_ERROR_STRINGS`
