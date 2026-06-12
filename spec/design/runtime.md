# design/runtime — runtime 設備 (time + mutex) の variant 注入

実行環境依存の基礎設備 (時刻取得・遅延・mutex) を **runtime kind** として variant が供給する。
bus kind (I2C / SPI / ...) と同じコンパイル時の勝者選択で、 選択 variant の具象 (free function /
具象クラス) が `m5::hal::v1::runtime` に注入される。 virtual 基底は持たない (ゼロコスト)。
bus kind の勝者バインドが型 alias なのに対し、 runtime は free function 群を運ぶ必要があるため
**この kind だけ `using namespace` 注入を維持する** ([variants.md](variants.md) §offer 要件の例外)。

設計目標は「現行ターゲットでの最小実装」ではなく **variant 注入による環境非依存 API の提供**
(M5HAL の移植性 API 面そのものが製品)。 time / mutex も gpio / i2c と同格の HAL 設備として扱う。

## API 契約 (`m5::hal::v1::runtime`)

```cpp
namespace m5::hal::v1::runtime {

// time (環境非依存。 実体は選択 variant の inline 委譲)
uint32_t millis(void);       // 起動からの経過 ms。 32bit wrap (約 49.7 日)
uint32_t micros(void);       // 起動からの経過 µs。 32bit wrap (約 71.6 分)
void     delayMs(uint32_t);  // 最低 ms の遅延。 タスクを譲る (RTOS では block)
void     delayUs(uint32_t);  // busy-wait 精度の短時間遅延

// mutex (選択 variant の具象型への alias / 注入。 virtual なし)
class Mutex;
//   bool lock(uint32_t timeout_ms);  // timeout_ms まで待つ。 0 = try-lock 即時、 TIMEOUT_FOREVER = 無限待ち。 true = 取得
//   void unlock(void);               // lock したタスクから呼ぶ
// コピー / ムーブ不可

}
```

### 意味論

- **`lock` は待ち合わせる**: 保有者がいる間 `timeout_ms` まで待ち、 取得できなければ `false`。
  `timeout_ms == 0` は即時 try-lock、 `types::TIMEOUT_FOREVER` は取得まで無限に待つ
  (stub フェイクのみ例外: 第 2 タスクが存在せず解放され得ないため、 無限待ちでも即 `false` を
  返してテストの決定性を保つ)。
- **非再帰**: 保有タスク自身の再 lock も timeout まで待って `false`。
- **task-context only**: runtime API を ISR から呼んではならない (`memory/pool` と同じ契約)。
  FreeRTOS の priority-inheritance mutex は ISR から take / give できない。
- **unlock は保有タスクから**: FreeRTOS mutex の要件。
- **timeout 粒度**: variant 依存。 FreeRTOS 環境では tick (既定 10 ms) が下限精度で、 非ゼロ
  timeout は最低 1 tick へ切り上げる。
- **delayUs は busy-wait**: 長時間の遅延には `delayMs` を使う。 `delayMs` は「最低 ms」 保証
  (espidf は tick 切り上げ +1 tick)。
- time の wrap は実クロックと同じ意味論 (`uint32_t` 減算で経過計測する)。

## variant 申告

| variant | time backend | Mutex backend |
|---|---|---|
| `frameworks/arduino` | `::millis` / `::micros` / `::delay` / `::delayMicroseconds` | FreeRTOS mutex (arduino-esp32 は FreeRTOS 環境) |
| `frameworks/espidf` | `esp_timer_get_time` (+ `vTaskDelay` / `esp_rom_delay_us`) | FreeRTOS mutex |
| `frameworks/posix` | `clock_gettime(CLOCK_MONOTONIC)` / `nanosleep` | `std::timed_mutex` |
| `frameworks/stub` | 単調フェイク (native テストで決定的) | シングルタスク owner ガード (競合 = 即 false) |
| `frameworks/software` | 申告しない (bus 実装 variant) | 同左 |

- FreeRTOS mutex は arduino / espidf 共有の `_detail/freertos_mutex.hpp`
  (`m5::hal::v1::detail::FreeRtosMutex`、 自己ゲート = `__has_include(<freertos/FreeRTOS.h>)`)。
- posix の `std::timed_mutex` は、 保有スレッド自身の `try_lock(_for)` を C++ 規格は未定義と
  するが、 配備されている両実装 (libstdc++ = NORMAL mutex の `pthread_mutex_timedlock`、
  libc++ = 自前 mutex + condvar) とも「timeout まで待って false」 に解決する (= 本契約どおり)。
- stub のフェイククロックは 0 起点で `delayMs` / `delayUs` によってのみ進む。 テスト分離用に
  `fakeReset()` を持つ。 mutex は待っても解放され得ない (他タスクがいない) ため、 競合は
  timeout 値に関わらず即 false — native テストの決定性を保つ。
- scan 順 / first-hit / `M5HAL_V1_SELECTED_VARIANT_RUNTIME` マーカは既存 kind と同一規約
  ([variants.md](variants.md))。 **NONE fallback だけは無い**: stub が常に申告するため選択は
  必ず成立し、 不成立は `hal/v1/runtime/runtime.hpp` の `#error` で止める (`bus::IBus` が型と
  して必要なので NONE を許容できない)。

## early scan (bus kind との違い)

`bus::IBus` は `runtime::Mutex` を **値で内蔵** するため、 runtime の勝者選択は
`hal/v1/bus/bus.hpp` より前に完了していなければならない。 そこで runtime kind だけは
`M5HAL_v1.hpp` 末尾の本 scan ではなく、 **`hal/v1/runtime/runtime.hpp` 内の early scan** で
解決する:

1. framework `_checker.hpp` で検出済みの variant を本 scan と同じ順 (arduino → espidf →
   posix → stub) に走査する
2. 各 pass は variant の runtime 実装ヘッダ + `_offer.hpp` を include し、
   `_macro/offer_runtime_only.inl` で **runtime 以外の HAS フラグをマスク** して共通エミッタ
   (`offer_all.inl`) に委譲する — ディスパッチブロック自体は他 kind と完全同形
3. 本 scan が同じ `_offer.hpp` を再 include した際、 runtime の first-hit マーカは既に焼かれて
   いるため variant alias の再発行のみが起こる (重複 using-directive は無害)

platform variant は現在 runtime を申告しない。 申告する platform が現れたら early scan の
**先頭** に platform pass を追加する (本 scan の platform 優先順をこの kind でも保つため)。

## Bus との統合

`bus::IBus` は `runtime::Mutex _mutex` を常時内蔵し、 `lock(owner, timeout_ms)` は mutex の
待ち合わせに従う。 競合は **`TIMEOUT_ERROR`** (詳細は [bus_accessor.md](bus_accessor.md)
§排他制御の意味論)。 UART はチャネル別に mutex を 2 本持つ ([uart.md](uart.md) §channel
semantics)。

`service::fastTick` 系 (perf counter) は service 層に残る — runtime::time とは役割が別
(wall-clock ms/µs と遅延 vs 高分解能位相維持)。

## posix opt-out との関係

`M5HAL_CONFIG_POSIX_UART=0` は **UART kind のみ** を抑止する (variant 全体を止めない)。
runtime は引き続き posix が供給するため、 host の Bus が stub フェイク mutex へ静かに退行する
ことはない ([configuration.md](configuration.md))。

## 関連

- [variants.md](variants.md) — offer 機構と kind 追加チェックリスト
- [bus_accessor.md](bus_accessor.md) — IBus::lock の意味論
- [../architecture.md](../architecture.md) — 層構成 (runtime 設備の置き場所)
