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
//   bool lock(uint32_t timeout_ms);  // timeout_ms まで待つ。 0 = try-lock 即時、 TIMEOUT_FOREVER = 無限待ち。 true = 取得
//   void unlock(void);               // lock したタスクから呼ぶ
// コピー / ムーブ不可

// task (選択 variant の具象型への alias / 注入。 virtual なし)
class Task;
//   bool start(void (*fn)(void*), void* arg, const char* name, size_t stack_size, int priority,
//              int core = types::TASK_CORE_ANY);
//   void join(void);       // start に成功したタスクの終了を待つ。 非 joinable なら即座に戻る
//   bool joinable(void) const;
// コピー / ムーブ不可。 dtor は join() を呼ぶ (生存タスクを刈り取ってから破棄する)

// event (選択 variant の具象型への alias / 注入。 virtual なし)
class Event;
//   bool wait(uint32_t timeout_ms);  // 通知があるまで (または timeout まで) ブロックし、 消費して true。
//                                    // 0 = 非ブロッキング確認、 TIMEOUT_FOREVER = 無限待ち。 false = timeout
//   void notify(void);               // 通知を掲上する (latching)。 wait より先に打たれても失われない
// コピー / ムーブ不可

void* currentTaskId(void);  // 呼び出し元タスクを一意に識別する opaque ハンドル (§Task API 契約)

}
```

### 意味論

共通機構 (**Mutex lock/unlock の待ち合わせ・非再帰・task-context only・timeout 粒度**) は
[bus_accessor.md](bus_accessor.md) §排他制御の意味論 を参照。本 kind 固有の差分のみ以下に示す。

- **stub フェイク**: 第 2 タスクが存在せず解放され得ないため、 無限待ちでも即 `false` を
  返してテストの決定性を保つ。
- **delayUs は busy-wait**: 長時間の遅延には `delayMs` を使う。 `delayMs` は「最低 ms」 保証
  (espidf は tick 切り上げ +1 tick)。
- **time の wrap**: 実クロックと同じ意味論 (`uint32_t` 減算で経過計測する)。

### Event API 契約

latching binary event。 「タスクを起こす通知」 の最小プリミティブで、 counting semaphore の
広い契約は持たない (計数用途には使わない)。 [service.md](service.md) の auto-run idle 起床が
第一の利用箇所。

- **latching**: `notify()` が `wait()` より先に打たれても失われず、 次の `wait()` が即座に
  消費して true を返す。 「フラグ判定 → wait 突入」 の間に notify が滑り込んでも永眠しない
  ことをこの性質が保証する。
- **合体**: 未消費の通知が既にある状態での複数回の `notify()` は 1 回に合体してよい
  (wait 1 回で全部消費される)。 通知の回数を数える用途には使えない。
- **single waiter**: `wait()` の同時呼び出しは最大 1 タスク。 複数 waiter の起床順・分配は
  未定義。
- **可視性**: `notify()` より前の書込みは、 その通知を消費して true を返した `wait()` の後から
  可視 (release/acquire 対)。
- **timeout 引数**: `0` = 非ブロッキング確認 (pending があれば消費して true)。
  `types::TIMEOUT_FOREVER` = 無限待ち (stub は即 false の文書化例外 — Mutex と同型)。
  有限値の tick 変換は切上げ (Mutex と同じ流儀)。
- **コピー / ムーブ不可**。 破棄は 「waiter がおらず、 concurrent notify も起き得ない」 状態で
  のみ許す (Mutex / Task と同じ所有規律)。
- Mutex と同じく **task 文脈限定** (ISR 不可)。 将来 ISR からの通知が必要になったときの拡張形は
  `notifyFromISR()` のメソッド追加 (後方互換。 freertos = `xSemaphoreGiveFromISR` +
  `portYIELD_FROM_ISR` まで内部で完結させ、 RTOS 固有の後始末を呼び出し元へ漏らさない。
  posix / stub = `notify()` の別名)。 **wait 側の FromISR 版は作らない** (ISR でのブロックは
  原理的に不可 — 非対称な API であることを契約とする)。

### Task API 契約

- **start/join のペア性**: `start()` に成功したタスクは `join()` されるまで生存する前提で
  設計する。`join()` は非 joinable なら即座に戻る (冪等)。デストラクタは `join()` を呼ぶため、
  `Task` を破棄するだけで生存タスクを刈り取れる。
- **`core` 引数 (配置)**: 非負 = その core へ pin。`types::TASK_CORE_ANY` (-1) = 配置は
  スケジューラ任せ (affinity なし)。`types::TASK_CORE_SAME` (-3) = **呼び出し core と同じ
  core へ pin** — 「生成元と per-core クロックドメインを共有しなければならないワーカー」
  のための配置 (ESP32 系の CPU cycle counter は per-core かつ WFI で停止するため、生成元が
  書いた時刻マークは同一 core 上でのみ比較可能。auto-run の配置ノブ
  `M5HAL_CONFIG_SERVICE_AUTORUN_CORE` が明示 pin したい場合に使う — [service.md](service.md))。
  `types::TASK_CORE_OPPOSITE` (-2) = 呼び出し core の補集合へ pin。単コアターゲットでは
  SAME / OPPOSITE とも自 core となり実質無効果。posix / stub は API parity のため受け取って
  無視する (host はスケジューラが配置を所有)。freertos は範囲外の core id を `false` で
  拒否する。
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

- **stub の Task/Mutex は「同値性が壊れている」ことに注意**: stub の `Task::start` は
  API 形こそ posix と同一 (実際に `std::thread` を生成する) だが、`currentTaskId()` は
  生成したスレッドの識別を反映せず常に固定値を返し、`Mutex` も実際の相互排他ではなく
  「非ブロッキングの単一所有者ガード」(競合は timeout 値に関わらず即座に失敗) である。
  したがって stub 上で `Task` を用いて実際に並行実行させると、単一ライタ判定・自己判定の
  前提が成立しなくなる。**stub 上でマルチタスク的な並行性を模倣するコードを書いてはならない**
  ([service.md](service.md) §stub が対象外である理由 — auto-run が posix のみに開放され
  stub を対象外とする根拠と同じ)。

## variant 申告

| variant | RUNTIME (time) | RUNTIME_MUTEX | RUNTIME_TASK | RUNTIME_EVENT |
|---|---|---|---|---|
| `frameworks/freertos` | 申告しない | FreeRTOS mutex (`xSemaphoreCreateMutex`) | FreeRTOS task (`xTaskCreatePinnedToCore`) | binary semaphore (`xSemaphoreCreateBinary`) |
| `frameworks/arduino` | `::millis` / `::micros` / `::delay` / `::delayMicroseconds` | 申告しない | 申告しない | 申告しない |
| `frameworks/espidf` | `esp_timer_get_time` (+ `vTaskDelay` / `esp_rom_delay_us`) | 申告しない | 申告しない | 申告しない |
| `frameworks/posix` | `clock_gettime(CLOCK_MONOTONIC)` / `nanosleep` | `std::timed_mutex` | `std::thread` wrapper | mutex + condvar + flag |
| `frameworks/stub` | 単調フェイク (native テストで決定的) | シングルタスク owner ガード | no-op Task | flag のみフェイク (wait は即帰) |
| `frameworks/software` | 申告しない (bus 実装 variant) | 同左 | 同左 | 同左 |

- **FreeRTOS variant** (`variants/frameworks/freertos/`) は独立 framework variant として offer
  機構にフル参加する。 scan 順で arduino / espidf より前に配置されるため RUNTIME_MUTEX /
  RUNTIME_TASK / RUNTIME_EVENT を先に勝ち取る。 time functions は申告しない (framework 固有
  API に依存するため arduino / espidf に委ねる)。 現在の FreeRTOS 検出は ESP-IDF のインクルードレイアウト
  (`<freertos/FreeRTOS.h>`) のみ。 Task は Espressif 拡張 (`xTaskCreatePinnedToCore` /
  `tskNO_AFFINITY`) を使用するため、 generic FreeRTOS 向けは将来の拡張候補。
- posix の `std::timed_mutex` は、 保有スレッド自身の `try_lock(_for)` を C++ 規格は未定義と
  するが、 配備されている両実装 (libstdc++ = NORMAL mutex の `pthread_mutex_timedlock`、
  libc++ = 自前 mutex + condvar) とも「timeout まで待って false」 に解決する (= 本契約どおり)。
- posix の Event は condvar 単体では latching にならないため bool flag で自作し、 **predicate
  付き `wait_for` を必須**とする (predicate なしの単発 `wait_for` は spurious wakeup を
  timeout と誤報するため禁止 — 実装コメントにも明記)。 freertos の Event は binary semaphore
  が latching・合体・timeout を素で満たす。 dynamic allocation 構成で生成に失敗した場合は
  null handle を許し wait=false / notify=no-op に縮退する (Mutex の null 方針と同じ)。
- stub のフェイククロックは 0 起点で `delayMs` / `delayUs` によってのみ進む。 テスト分離用に
  `fakeReset()` を持つ。 mutex は待っても解放され得ない (他タスクがいない) ため、 競合は
  timeout 値に関わらず即 false — native テストの決定性を保つ。
- scan 順 / first-hit / `M5HAL_V2_SELECTED_VARIANT_RUNTIME{,_MUTEX,_TASK,_EVENT}` マーカは既存
  kind と同一規約 ([variants.md](variants.md))。 **NONE fallback は無い**: stub が 4 sub-kind
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
