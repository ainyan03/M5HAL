# design/runtime — runtime 設備 (time + mutex + task + event) の variant 注入

> **読者**: 実装者・レビュー向け（設計仕様）。

実行環境依存の基礎設備 (時刻取得・遅延・mutex・task・event) を **runtime kind** として variant が
供給する。 runtime kind は 4 つの独立した **sub-kind** に分割されており、 sub-kind ごとに
異なる variant が勝者になれる:

| sub-kind | marker | 注入方式 |
|---|---|---|
| **RUNTIME** (time) | `M5HAL_V2_SELECTED_VARIANT_RUNTIME` | `using namespace` 注入 |
| **RUNTIME_MUTEX** | `M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX` | `using Mutex = ...;` 型 alias |
| **RUNTIME_TASK** | `M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK` | `using Task = ...;` 型 alias |
| **RUNTIME_EVENT** | `M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT` | `using Event = ...;` 型 alias |

bus kind (I2C / SPI / ...) と同じコンパイル時の勝者選択で、 選択 variant の具象 (free function /
具象クラス) が `m5::hal::v2::runtime` に注入される。 virtual 基底は持たない (ゼロコスト)。
time の free function は型 alias では運べないため **RUNTIME sub-kind だけ `using namespace`
注入を維持する** ([variants.md](variants.md) §offer 要件の例外)。 Mutex / Task / Event は型 alias。

設計目標は「現行ターゲットでの最小実装」ではなく **variant 注入による環境非依存 API の提供**
(M5HAL の移植性 API 面そのものが製品)。 time / mutex / task も gpio / i2c と同格の HAL 設備
として扱う。

## API 契約 (`m5::hal::v2::runtime`)

```cpp
namespace m5::hal::v2::runtime {

// time (環境非依存。 実体は選択 variant の inline 委譲)
uint32_t millis(void);       // 起動からの経過 ms。 32bit wrap (約 49.7 日)
uint32_t micros(void);       // 起動からの経過 µs。 32bit wrap (約 71.6 分)
void     delayMs(uint32_t);  // 最低 ms の遅延。 タスクを譲る (RTOS では block)
void     delayUs(uint32_t);  // busy-wait 精度の短時間遅延

// mutex (選択 variant の具象型への alias / 注入。 virtual なし)
class Mutex;
//   result_t<void> lock(uint32_t timeout_ms);  // 0 = try-lock、TIMEOUT_FOREVER = 無限待ち
//   result_t<void> unlock(void);               // lock に成功した同一タスクから呼ぶ
// コピー / ムーブ不可

// task (選択 variant の具象型への alias / 注入。 virtual なし)
class Task;
//   result_t<void> start(void (*fn)(void*), void* arg, const char* name = nullptr,
//                        size_t stack_size = 4096, int priority = 1,
//                        int core = types::TASK_CORE_ANY);
//   void join(void);       // start に成功したタスクの終了を待つ。 非 joinable なら即座に戻る
//   bool joinable(void) const;
// コピー / ムーブ不可。 dtor は join() を呼ぶ (生存タスクを刈り取ってから破棄する)

// event (選択 variant の具象型への alias / 注入。 virtual なし)
class Event;
//   result_t<void> wait(uint32_t timeout_ms);  // 通知を消費してsuccess、期限切れはTIMEOUT_ERROR。
//                                              // 0 = 非ブロッキング確認、TIMEOUT_FOREVER = 無限待ち
//   void notify(void);               // 通知を掲上する (latching)。 wait より先に打たれても失われない
// コピー / ムーブ不可

void* currentTaskId(void);  // 呼び出し元タスクを一意に識別する opaque ハンドル (§Task API 契約)

}
```

### 意味論

共通機構 (**Mutex lock/unlock の待ち合わせ・非再帰・task-context only・timeout 粒度**) は
[bus_accessor.md](bus_accessor.md) §排他制御の意味論 を参照。本 kind 固有の差分のみ以下に示す。

- **stub フェイク**: 第 2 タスクが存在せず解放され得ないため、 無限待ちでも即
  `TIMEOUT_ERROR` を返してテストの決定性を保つ。
- **delayUs は busy-wait**: 長時間の遅延には `delayMs` を使う。 `delayMs` は「最低 ms」 保証
  (espidf は tick 切り上げ +1 tick)。
- **time の wrap**: 実クロックと同じ意味論 (`uint32_t` 減算で経過計測する)。

### Mutex API 契約

- **呼出しごとの結果**: `lock()` は取得成功時に`{}`、待ち時間切れに`TIMEOUT_ERROR`を返す。
  POSIX backendが有効な前提条件下でOS失敗を`std::system_error`として報告した場合は`IO_ERROR`。
  `unlock()`は正常解放時に`{}`、backendが解放不能な状態を検出した場合は`INVALID_STATE`を返す。
- **所有規律**: non-recursive。取得中のMutexを同じタスクから再取得してはならない。
  `unlock()`は`lock()`に成功した同一タスクが一度だけ呼ぶ。再取得、wrong-owner unlock、所有して
  いないPOSIX Mutexのunlockは契約違反であり、回復可能なerrorとしての検出を保証しない。
  FreeRTOSの`xSemaphoreGive()`が`pdFALSE`を返した場合は`INVALID_STATE`へ写像するが、wrong-ownerを
  必ず検出できるという意味ではない。
- **timeout**: `0`は即時試行。有限値はvariantの粒度で待ち、FreeRTOSはtickへ切り上げる。
  `types::TIMEOUT_FOREVER`は取得まで待つ。stubだけは競合時に即`TIMEOUT_ERROR`を返す。
- **cleanup**: 明示的な解放APIは観測した`unlock()` errorを返す。cleanup前の本処理も失敗していた場合は
  本処理のerrorを返すが、`unlock()`失敗を黙って回復可能とは扱わない。再利用されるownerはlockに依存しない
  `Broken`状態へ、外部identityを持つlifecycleは`Quarantined`へ遷移し、後続利用を拒否する。本処理が成功して
  いた場合は`unlock()` errorを返す。複数Mutexを解放するcleanupは全解放を試し、最初のerrorを採用する。
  RAII destructorは返却先がないため結果自体はbest-effortに畳むが、ownerがその場で寿命終了しない場合の
  `Broken` / `Quarantined`遷移は省略しない。共有固定表やprovider内部guardのように、安全な
  lock-independent隔離状態も返却先も持たない内部invariantでは、unlock失敗後の再利用を許すより
  fail-loudする。
- コピー / ムーブ不可。task context限定でISRから呼ばない。待機中または保有中のMutexを破棄しない。

### Event API 契約

latching binary event。 「タスクを起こす通知」 の最小プリミティブで、 counting semaphore の
広い契約は持たない (計数用途には使わない)。 [service.md](service.md) の auto-run idle 起床が
第一の利用箇所。

- **latching**: `notify()` が `wait()` より先に打たれても失われず、 次の `wait()` が即座に
  消費してsuccessを返す。 「フラグ判定 → wait 突入」 の間に notify が滑り込んでも永眠しない
  ことをこの性質が保証する。
- **合体**: 未消費の通知が既にある状態での複数回の `notify()` は 1 回に合体してよい
  (wait 1 回で全部消費される)。 通知の回数を数える用途には使えない。
- **single waiter**: `wait()` の同時呼び出しは最大 1 タスク。 複数 waiter の起床順・分配は
  未定義。
- **可視性**: `notify()` より前の書込みは、 その通知を消費してsuccessを返した `wait()` の後から
  可視 (release/acquire 対)。pending済みへ合体した`notify()`もこの公開順序へ参加し、次の
  success `wait()`から、その合体通知より前の書込みが可視になる。
- **timeout 引数**: `0` = 非ブロッキング確認 (pending があれば消費してsuccess)。
  `types::TIMEOUT_FOREVER` = 無限待ち (stub は即`TIMEOUT_ERROR`の文書化例外 — Mutex と同型)。
  有限値の tick 変換は切上げ (Mutex と同じ流儀)。
- **呼出しごとの結果**: `wait()`は通知を消費したときsuccess、未通知のまま期限へ達したとき
  `TIMEOUT_ERROR`を返す。POSIX backendが待機中のOS失敗を`std::system_error`として報告した場合は
  `IO_ERROR`へ写像する。FreeRTOSのstatic handle不成立は`INVALID_STATE`だが、compliant portの通常経路
  では到達しない。`notify()`は満たされたprecondition下で失敗しないため`void`を維持する。binary
  semaphoreが既に掲上済みの重複notifyはerrorでなく、契約どおり1件へ合体したsuccessである。
- **コピー / ムーブ不可**。 破棄は 「waiter がおらず、 concurrent notify も起き得ない」 状態で
  のみ許す (Mutex / Task と同じ所有規律)。
- Mutex と同じく **task 文脈限定** (ISR 不可)。notify / wait ともISRから呼んではならない。
  将来ISR notifyを追加する場合も`wait()`はtask限定のままとし、RTOS固有のyield指定を公開APIへ漏らさない。

### Task API 契約

- **生成結果は呼出しごとに返す**: `start()`は成功時に`{}`、失敗時に次のerrorを返す。objectへ
  last statusは保存せず、失敗を`bool`へ畳まない。

  | 条件 | error |
  |---|---|
  | null entry、FreeRTOSで0/表現不能なstack、範囲外priority/core | `INVALID_ARGUMENT` |
  | 既にjoinable | `INVALID_STATE` |
  | RTOS task / host threadの生成資源不足 | `OUT_OF_RESOURCE` |
  | host thread生成のその他OS error | `IO_ERROR` |
  | threadを提供しないstub、または例外無効のhost providerで有効な生成要求 | `UNSUPPORTED` |

  引数検証を状態検証より先に行うため、null entryかつjoinableなら`INVALID_ARGUMENT`を返す。
- **start/join のペア性**: `start()` に成功したタスクは `join()` されるまで生存する前提で
  設計する。`join()` は非 joinable なら即座に戻る (冪等)。デストラクタは `join()` を呼ぶため、
  `Task` を破棄するだけで生存タスクを刈り取れる。通常の契約内では`join()`にcallerへ返す失敗がないため
  戻り値は`void`を維持する。自己joinと実行中task自身からの破棄は契約違反であり、回復可能なerror面にしない。
- **`core` 引数 (配置)**: 非負 = その core へ pin。`types::TASK_CORE_ANY` (-1) = 配置は
  スケジューラ任せ (affinity なし)。`types::TASK_CORE_SAME` (-3) = **呼び出し core と同じ
  core へ pin** — 「生成元と per-core クロックドメインを共有しなければならないワーカー」
  のための配置 (ESP32 系の CPU cycle counter は per-core かつ WFI で停止するため、生成元が
  書いた時刻マークは同一 core 上でのみ比較可能。auto-run の配置ノブ
  `M5HAL_CONFIG_SERVICE_AUTORUN_CORE` が明示 pin したい場合に使う — [service.md](service.md))。
  `types::TASK_CORE_OPPOSITE` (-2) = 呼び出し core の補集合へ pin。単コアターゲットでは
  SAME / OPPOSITE とも自 core となり実質無効果。posix / stub は API parity のため受け取って
  無視する (host はスケジューラが配置を所有)。`std::thread`の生成失敗は例外でしか報告されないため、
  例外無効host buildは生成を試みず`UNSUPPORTED`を返す。freertos は範囲外の core id を
  `INVALID_ARGUMENT`で拒否する。
- **`currentTaskId()` は同値性のみが契約**: 「同一タスクからの呼び出しは常に同じ値を返し、
  異なるタスクからの呼び出しは異なる値を返す」ことだけが契約であり、値そのものに意味はない
  (アドレスとして解釈・デリファレンスしてはならない、opaque ハンドル)。
  [service.md](service.md) の R1/R3/R6 (svc タスク自身かどうかの自己判定) はこの同値性のみに
  依存する。variant ごとの実体:

  | variant | 実体 |
  |---|---|
  | freertos | `xTaskGetCurrentTaskHandle()` |
  | posix | `thread_local` 変数のアドレス |
  | stub | 固定定数 (全呼び出し元が同一値を共有) |

- **thread-capable host stub の Task/Mutex は「同値性が壊れている」ことに注意**: 例外有効hostの
  stub `Task::start`はAPI 形こそposixと同一 (`std::thread`を生成する) だが、`currentTaskId()` は
  生成したスレッドの識別を反映せず常に固定値を返し、`Mutex` も実際の相互排他ではなく
  「非ブロッキングの単一所有者ガード」(競合は timeout 値に関わらず即座に失敗) である。
  したがって stub 上で `Task` を用いて実際に並行実行させると、単一ライタ判定・自己判定の
  前提が成立しなくなる。**stub 上でマルチタスク的な並行性を模倣するコードを書いてはならない**
  ([service.md](service.md) §stub が対象外である理由 — auto-run が posix のみに開放され
  stub を対象外とする根拠と同じ)。threadlessまたは例外無効stubはtaskを生成せず`UNSUPPORTED`を返す。

## variant 申告

| variant | RUNTIME (time) | RUNTIME_MUTEX | RUNTIME_TASK | RUNTIME_EVENT |
|---|---|---|---|---|
| `frameworks/freertos` | 申告しない | FreeRTOS mutex (`xSemaphoreCreateMutex`) | FreeRTOS task (`xTaskCreatePinnedToCore`) | binary semaphore (`xSemaphoreCreateBinary`) |
| `frameworks/arduino` | `::millis` / `::micros` / `::delay` / `::delayMicroseconds` | 申告しない | 申告しない | 申告しない |
| `frameworks/espidf` | `esp_timer_get_time` (+ `vTaskDelay` / `esp_rom_delay_us`) | 申告しない | 申告しない | 申告しない |
| `frameworks/posix` | `clock_gettime(CLOCK_MONOTONIC)` / `nanosleep` | `std::timed_mutex` | 例外有効時は`std::thread`、無効時は`UNSUPPORTED` | mutex + condvar + flag |
| `frameworks/stub` | 単調フェイク (native テストで決定的) | シングルタスク owner ガード | hostかつ例外有効時は`std::thread`、それ以外は`UNSUPPORTED` | flag のみフェイク (wait は即帰) |
| `frameworks/software` | 申告しない (bus 実装 variant) | 同左 | 同左 | 同左 |

- **FreeRTOS variant** (`variants/frameworks/freertos/`) は独立 framework variant として offer
  機構にフル参加する。 scan 順で arduino / espidf より前に配置されるため RUNTIME_MUTEX /
  RUNTIME_TASK / RUNTIME_EVENT を先に勝ち取る。 time functions は申告しない (framework 固有
  API に依存するため arduino / espidf に委ねる)。 現在の FreeRTOS 検出は ESP-IDF のインクルードレイアウト
  (`<freertos/FreeRTOS.h>`) のみ。 Task は Espressif 拡張 (`xTaskCreatePinnedToCore` /
  `tskNO_AFFINITY`) を使用するため、generic FreeRTOSは現行backendの対応範囲外。
- Mutex / Event は constructor を fallible にしない現 API の correctness を保つため、
  `configSUPPORT_STATIC_ALLOCATION=1` を必須とする。無効な FreeRTOS 構成は compile error とし、
  heap 枯渇を lock timeout や Event の idle として誤報しない。各 object は caller-owned の
  `StaticSemaphore_t` から handle を生成する。Mutex / Eventは`TIMEOUT_FOREVER`を真の無限待ちにするため
  `INCLUDE_vTaskSuspend=1`も必須とし、満たさない構成はcompile errorにする。
- posix の `std::timed_mutex` は、保有スレッド自身の `try_lock(_for)` とwrong-owner unlockが
  C++規格上未定義であるため実行しない。これらをtimeoutやportableなerrorへ変換する契約は持たない。
- posix の Event は condvar 単体では latching にならないため bool flag で自作し、 **predicate
  付き `wait_for` を必須**とする (predicate なしの単発 `wait_for` は spurious wakeup を
  timeout と誤報するため禁止 — 実装コメントにも明記)。 freertos の Event は binary semaphore
  が latching・合体・timeout を素で満たす。static allocation を必須とするため、生成失敗を
  timeout / no-op に縮退させる契約は持たない。
- stub のフェイククロックは 0 起点で `delayMs` / `delayUs` によってのみ進む。 テスト分離用に
  `fakeReset()` を持つ。 mutex は待っても解放され得ない (他タスクがいない) ため、 競合は
  timeout 値に関わらず即 `TIMEOUT_ERROR` — native テストの決定性を保つ。
- scan 順 / first-hit / `M5HAL_V2_SELECTED_VARIANT_RUNTIME{,_MUTEX,_TASK,_EVENT}` マーカは既存
  kind と同一規約 ([variants.md](variants.md))。入力はそれぞれ
  `M5HAL_CONFIG_VARIANT_RUNTIME{,_MUTEX,_TASK,_EVENT}`で明示指定できる。入力の`NONE`はoverrideなしを
  意味する一方、選択結果には **NONE fallback は無い**: stub が 4 sub-kind
  全てを常に申告するため選択は必ず成立し、 不成立は `hal/v2/runtime/runtime.hpp` の `#error` で
  止める (`bus::IBus` が型として必要なので NONE を許容できない)。

## early scan (bus kind との違い)

`bus::IBus` は `runtime::Mutex` を **値で内蔵** するため、 runtime の勝者選択 (4 sub-kind 全て)
は `hal/v2/bus/bus.hpp` より前に完了していなければならない。 そこで runtime kind だけは
`M5HAL_v2.hpp` 末尾の本 scan ではなく、 **`hal/v2/runtime/runtime.hpp` 内の early scan** で
解決する:

1. framework `_checker.hpp` で検出済みの variant を本 scan と同じ順 (**freertos → arduino →
   espidf → posix → stub**) に走査する。 freertos が最初に scan されるため RUNTIME_MUTEX /
   RUNTIME_TASK / RUNTIME_EVENT を先に勝ち取り、 arduino / espidf が RUNTIME (time) を勝つ
2. 各 pass は variant の runtime 実装ヘッダ + `_offer.hpp` を include し、
   `_macro/offer_runtime_only.inl` で **runtime 以外の HAS フラグをマスク** して共通エミッタ
   (`offer_all.inl`) に委譲する。kind別selectorもここで適用され、指定variantがscanに不参加または
   対象sub-kindをofferしなければcompile errorになる（例: ArduinoをRUNTIME_MUTEXへ指定できない）
3. 本 scan が同じ `_offer.hpp` を再 include した際、 runtime の first-hit マーカは既に焼かれて
   いるため variant alias の再発行のみが起こる (重複 using-directive は無害)

platform variant は現在 runtime を申告しない。 申告する platform が現れたら early scan の
**先頭** に platform pass を追加する (本 scan の platform 優先順をこの kind でも保つため)。

## Bus との統合

`bus::IBus` は `runtime::Mutex _mutex` を常時内蔵し、Accessor lifecycleから呼ぶprotected seam
`acquireAccessLock(owner, timeout_ms)` は mutex の
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
- [bus_accessor.md](bus_accessor.md) — `IBus::acquireAccessLock` の意味論
- [../architecture.md](../architecture.md) — 層構成 (runtime 設備の置き場所)
