# design/service — ServiceRunner の協調実行モデルと並行性契約

> **読者**: 実装者・レビュー向け（設計仕様）。

`m5::hal::v2::service` は、単発の I/O 呼び出しだけでは表現できない**継続的なバックグラウンド作業**
(ソフトウェア I2C/SPI の状態機械駆動、UART の再設定監視、GPIO watch のポーリング、リモートバスの
frame mux 駆動 等) を、単一のタスク上で協調的にポーリングする最小の実行機構を提供する。仮想基底を
持たず (`IService::serviceImpl` のみ virtual)、対応する RTOS が無い環境 (posix host / stub) でも
同じ API 面で動く。

## API 契約 (`m5::hal::v2::service`)

```cpp
namespace m5::hal::v2::service {

enum class ServiceResult : uint8_t { Idle, Progress, Done, Error };

struct ServiceContext {
    fast_tick_t elapsed    = 0;  // このサービスの前回ポールからの経過 (mod-2^32 の duration)
    fast_tick_t local_tick = 0;  // 呼び出し内スピンの起点アンカー専用の生ティック

    ServiceContext() = default;                              // 集成体ではない (下記 why)
    ServiceContext(fast_tick_t elapsed, fast_tick_t local);  // {elapsed, local_tick}
};

struct ServicePoll {
    ServiceResult result = ServiceResult::Idle;
    fast_tick_t next_due_delta = 0;  // 「最短でもこれだけ先」の相対値。0 = ヒントなし
};

class IService {
protected:
    virtual ServicePoll serviceImpl(const ServiceContext& ctx) = 0;
};

class ServiceRunner {
public:
    static constexpr size_t kMaxServices = 16;

    result_t<void> add(IService& service);
    result_t<void> remove(IService& service);   // SYNCHRONOUS (後述 R3)
    result_t<void> clear();                     // SYNCHRONOUS like remove()

    static ServicePoll run(IService& service, const ServiceContext& ctx);
    result_t<bool> runOnce();                           // default-clock 駆動 (fastTick 測定)
    result_t<bool> runOnce(const ServiceContext& ctx);  // 明示 ctx 駆動

    size_t size() const;      // 近似値 (後述)
    size_t capacity() const;

    result_t<void> startAutoRun();
    result_t<void> stopAutoRun();
    bool autoRunActive() const;  // lock-free の助言値 (後述)
};

}  // namespace m5::hal::v2::service
```

### 時間契約: なぜ相対値か (`elapsed` / `next_due_delta`)

`fastTick()` は ESP32 系で **per-core の CPU cycle counter** (WFI で停止、コア間で無関係な値)
であり、絶対 tick は「同じコアで読まれた 2 値」しか比較できない。旧契約 (`ctx.now_tick` 絶対値)
では、producer が保存した絶対 tick を別コアで走る runner の tick と比較する事故が構造的に
可能だった (auto-run のコア移動で software bit-bang が ~15 倍劣化した実測が発端)。現契約は
**絶対 tick を契約から排除**し、時間は次の 3 形でのみ流通する:

- **`ctx.elapsed`** — 「このサービスへの前回ポールからの経過」。測定責任は**ポールストリーム**
  (runner / sole-pumper ループ / 手動駆動元) にあり、ストリームは (前回値, 読み取りドメイン)
  の組で測る。ドメイン (= コア) が変わって連続性を保証できないときは **0 (gap-drop)** を渡す。
  誤差方向は常に「遅れる側」で、open-drain I2C・SPI クロックには安全側。サービス側は private の
  仮想時計 (`_svc_now += elapsed`) を進め、既存の wrap-safe 絶対値計算をその上で行う。
- **`ctx.local_tick`** — 呼び出し時点の生 tick。**呼び出し内スピンの起点アンカー専用**。保存・
  呼び出しを跨ぐ比較・`elapsed` との混用は禁止。呼び出し内で実カウンタと比較して待つ service
  (software SPI のエッジ刻み) へは、明示 ctx 駆動でも**実 `fastTick()` 由来の値**を渡すこと
  (純仮想値は挙動未定義)。スピンを持たない service (software I2C 等) は純仮想値でよい。
- **転送全体デッドライン** — 仮想時計は gap-drop で実時間より遅れるため、wire timeout には
  使わない。コア間共有の単調クロック `sharedNowUs()` (ESP32 = `esp_timer_get_time`) で
  差分比較し、読みコスト (~100+ cycles) はポール回数で償却する (詳細は [i2c.md](i2c.md))。

`ServiceContext` を集成体にしないのは移行安全のため: 旧契約の `ServiceContext{絶対tick}` が
新レイアウトの `elapsed` として無警告に解釈される事故を、1 引数の positional 初期化を
不成立にして塞ぐ。

**駆動モードの混在**: 同一 runner を default-clock 駆動 (`runOnce()` / auto-run) と明示 ctx
駆動で交互に駆動した場合、切り替え直後の default-clock パスは gap-drop (elapsed=0) から始まり、
以後は実時間で続行する。状態は腐敗しないが、**タイミング保証はない** — テストで明示駆動する
場合はblocking完了API (`endAccess`等。内部のdefault-clock fallbackが混在する入口)を
呼ぶ前に転送を完走させること。

- **`add`/`remove` は登録済みサービスの管理**。`add` は通信開始ではなく、継続poll対象をrunnerへ
  登録するライフサイクルcommandである。成功は「要求をpendingへ置いた」だけでなく、テーブルへの適用が
  確定したことを意味する。auto-run中はrunner taskが適用結果を確定するまで同期して待つ。テーブル容量は
  `kMaxServices = 16` 固定。外部callerのactive `add` は結果確認まで制御mutexで直列化されるため、
  pending-add容量は公開failure modeではない。active `add` の待機中にrunnerが既存serviceのcallbackを
  完了できるよう、callerは登録済み `serviceImpl` が取得し得るlockを保持してはならない。
- **`runOnce`** は登録済み全サービスを 1 パス分ポーリングする手動駆動。呼び出し元が明示的に
  周期的に呼ぶ (native テストや、auto-run を使わない環境向け)。
- **`startAutoRun`/`stopAutoRun`** は専用バックグラウンドタスク (`runtime::Task`) で
  `runOnce` 相当を回し続ける自動駆動。**対応 variant は ESP-IDF/Arduino (FreeRTOS task) と
  例外有効posix (`std::thread`) のみ** — stubと例外無効hostは対象外 (§stub が対象外である理由、
  [runtime.md](runtime.md) §Task API 契約)。例外無効posixの`startAutoRun()`はTaskの
  `UNSUPPORTED`をそのまま返す。
  ESP-IDF/Arduino では最初の `add()` が暗黙に auto-run を起動する (usability 判断、利用者は
  明示的な `startAutoRun()` を書かなくてよい)。**posix では `add()` からの暗黙起動は行わない**
  (native テストの決定性維持) — posix で auto-run を使うには `startAutoRun()` を明示的に呼ぶ。

### command結果と状態不変条件

| 操作・条件 | 結果 | 状態契約 |
|---|---|---|
| `add` 新規・空きあり | success | 戻るまでに登録を適用。embeddedの暗黙Task起動も完了 |
| `add` duplicate | `INVALID_STATE` | table/pending/timing不変 |
| `add` table容量不足 | `OUT_OF_RESOURCE` | table/pending/timing不変 |
| `add` 暗黙Task起動失敗 | `runtime::Task::start`のexact error | 今回の登録をrollbackし、auto-run inactive |
| `remove` present / absent | success | presentは同期解除。absentはpostcondition済みの冪等no-op |
| `clear` empty | success | reset commandとしてdefault-clock streamをinvalidate |
| `runOnce` 正常pass | success + `bool` | `true`はProgress/Doneあり、`false`は正常なno-progress |
| `runOnce` mutex競合、auto-run中、callback再入 | `BUSY` | stream/virtual/table/pending不変 |
| `startAutoRun` already running | success | 冪等no-op |
| `stopAutoRun` already stopped | success | wake latchを含めた冪等no-op |
| `startAutoRun` Task失敗 | Taskのexact error | Task non-joinable、auto-run inactive、登録表不変 |
| manual callback内の`startAutoRun` | `INVALID_STATE` | 状態不変 |
| callback内の`clear`/`stopAutoRun` | `INVALID_STATE` | stop flagを含め状態不変 |
| 制御mutexの`unlock`失敗後 | `INVALID_STATE` | runnerをBrokenとして後続commandを拒否 |

`result_t<bool>` のtruth値はpayloadでなく「errorがない」ことを表す。従って `runOnce()` の利用側は
`if (!r)` でerrorを処理した後、`r.value()` でprogressを読む。`!runOnce()` のように両者を一つの
bool式へ畳んではならない。

- **runner の idle 起床は通知駆動** (`runtime::Event`、latching — [runtime.md](runtime.md)
  §Event API 契約)。テーブルが空の間 runner は wake event でブロックし、`add()` (pending 投入) /
  `remove()` (pending 投入) / `stopAutoRun()` (stop flag) が状態 store 後・制御 mutex 解放前に
  notify で起こす。tick 量子化されたポーリング眠り (旧 `delayMs(1)` = FreeRTOS で実質 10–20ms)
  を排し、空テーブルへの `add()` から最初のポーリングまでの遅延をスケジューラ粒度にする。
  wait の timeout (100ms) は通知漏れバグ時の liveness backstop であって遅延保証ではない —
  `TIMEOUT_ERROR`だけを通知漏れの兆候としてDIAGカウンタに数える。その他のEvent backend errorは
  別カウンタ/診断へ分け、短いbackoff後にrunnerを継続する。taskを終了して`autoRunActive()`だけtrueに
  残したり、backend errorをtimeoutへ偽装したりしない。サービスが登録されている間の
  ポーリング周期 (yield ループ) はこの機構と独立で、変更していない。
- **runner タスクの core 配置は既定でスケジューラ任せ** (`TASK_CORE_ANY`、
  `M5HAL_CONFIG_SERVICE_AUTORUN_CORE` で上書き可 — [configuration.md](configuration.md))。
  一時期の SAME pin 既定は絶対 tick 契約時代のコア間比較事故 (software bit-bang 取引が
  ~15 倍劣化、dual-core ESP32 実測) への対症で、その構造的原因は上記の相対 tick 契約が排除した
  (コアが移動したパスは gap-drop になり、壊れた比較は起きない)。撤去は dual-core 実機 A/B で
  確認済み: ANY は software bit-bang read で退行なし (むしろ僅かに改善 = コア移動の
  スケジューリング/キャッシュ費用は実測に現れない)。加えて SAME pin には、consumer が
  M5HAL の待ち経路 (SpinBackoff、yield/delay で譲る) を通らず busy-poll で完了を待つと同居
  runner を飢えさせる副作用があり、ANY 既定はこれも解消する。単コアでは実質無効果。
  runner を consumer と同一クロックドメインへ固定したいビルド (例:
  `M5HAL_CONFIG_SERVICE_ASSUME_PINNED=1` でコアガード = パス毎の core-id 読み+比較、
  数 cycles を compile-time に外す場合) は core id か SAME を明示指定する (既定は正しさ側)。

## なぜこの規約が要るか

ESP/Arduino では最初の `add()` が専用タスクを自動起動し、以後 `serviceImpl()` はそのタスク上で
走る。この時点で「svc タスクと他スレッドの間で何をどう共有してよいか」という並行性契約が生まれる。
本 doc の R1-R8 がその契約の正文であり、`service::ServiceRunner` 自身の実装が従うだけでなく、
`IService` を実装する側 (software I2C/SPI の状態機械、UART の再設定監視、GPIOGroup の watcher、
リモートバスの frame mux 等) も等しく従う必要がある。

## 規約本体 (R1-R8)

- **R1 (単一ライタ)**: サービステーブル (登録済みサービスの配列とその due 時刻) の書き手は
  「auto-run 中は svc タスクのみ、非稼働時は制御 mutex 保持者のみ」に限られる。svc タスク以外
  からの `add`/`remove` は常に pending キュー経由で行われ、svc タスクがパス先頭でキューを
  消費してから直接テーブルへ適用する。**例外**: svc タスク自身が自分の `serviceImpl()` 内から
  `add`/`remove` を呼ぶ場合 (自己変更、または同じパス内の他サービスへの変更) は pending を
  経由せず直接適用する — 単一ライタの内側からの自己変更であり、走査カーソルの補正がその場で
  効くため安全 (pending 経由だと同一パス内でまだ対象が poll されてしまい得る)。
- **R2 (制御プレーンの排他)**: `add`/`remove`/`clear`/`startAutoRun`/`stopAutoRun`/`runOnce`
  は制御 mutex で直列化する。auto-run の稼働判定もすべて制御 mutex 保持下で行う。**auto-run
  タスクの poll パス (`serviceImpl` の呼び出し列) は制御 mutex を取らない** (ホットパスは
  無ロックのまま維持)。手動 `runOnce()` はパス全体を制御 mutex 保持で実行し、try-lock失敗、
  auto-run中、callback再入は`BUSY`を返す。成功時のno-progressは`result_t<bool>{false}`であり
  errorではない。
  公開クエリ `autoRunActive()`/`size()` は lock-free の近似値であり、「タスクが実在して進行
  可能」の証拠として扱ってはならない — `startAutoRun` は**タスク生成後に**フラグを公開する
  (生成前に公開すると、高優先度のスピナーがまだ存在しないタスクの完了を待ち続け、生成スレッド
  自身を飢餓させ得る)。runner の進行を待つスピンは有限回の yield の後、blocking な delay へ
  フォールバックすること (FreeRTOS の `taskYIELD` は同優先度のタスクにしか譲らないため、より
  高い優先度で待つ呼び出し元は無限に spin し得る)。
  制御mutexの`unlock()`失敗は排他機構の健全性喪失であり、lockに依存しないBroken flagを設定する。
  以後のcommandはmutexを再利用せず`INVALID_STATE`を返す。本処理とunlockが共に失敗した場合は本処理の
  errorを返し、Broken flagがcleanup失敗を保持する。本処理が成功していた場合はunlock errorを返す。
- **R3 (remove の意味論)**: 成功した`remove()`/`clear()` は「戻った時点で以後 `serviceImpl` は呼ばれ
  ない」ことを保証する (svc タスク自身からの呼び出しは R1 の直接適用で同じ保証を得る)。
  呼び出し側の義務: ①対象サービスの `serviceImpl` が取得しうるロックを保持したまま
  `remove`/`clear` を呼ばない (svc タスクがそのロックを待っている間に完了を待ち続けデッドロック
  し得る) ②同一サービスへの `add` と `remove` を異なるスレッドから並行に呼ばない (結果は未定義)。
  未登録対象の`remove()`はこのpostconditionが既に成立しているためsuccess no-op。この保証があるため、
  呼び出し側は成功した`remove()`の直後にサービスオブジェクトを破棄してよい。
- **R4 (serviceImpl のコンテキスト契約)**: `serviceImpl` はタスクコンテキスト専用 (ISR からの
  呼び出し禁止)、ブロッキング禁止 (待ちは戻り値の `next_due` によるスケジューリングヒントで
  表現する)。`add`/`remove`はwriterが直接適用するためcallback内から許可する。`clear()`/
  `stopAutoRun()`は自己joinを始めず`INVALID_STATE`を返す。手動`runOnce()`のcallback内からinactive
  runnerを`startAutoRun()`する操作も`INVALID_STATE`。auto-run callback内の`startAutoRun()`だけは
  already-runningの冪等successである。ある呼び出しスレッドは「auto-run タスク」か「`runOnce`
  呼び出しスレッド」のどちらか一方としてのみ `serviceImpl` を駆動できる。
- **R5 (データ受け渡しの標準形)**: svc タスクと他スレッドが共有する状態は、①完了通知型 = 共通
  Gate プリミティブ (release/acquire、ペイロードは通知 store より前に書き終え、読み手は acquire
  成立後にのみ読む)、②状態表型 = `runtime::Mutex`、のいずれかで保護する。素の非 atomic 共有・
  volatile 頼みの同期は禁止する。Gate は「完了の可視性」を保証するものであって「生存期間」を
  保証するものではない — 状態のリセット・破棄は R3 の remove 保証を得た後にのみ行う。
- **R6 (コールバック契約)**: サービス発のユーザーコールバックは svc タスク上・当該モジュールの
  ロック外で呼ぶ。コールバック内から同モジュールの API への再入 (watch/unwatch 等) を許す。
  コールバックはブロッキング禁止。unwatch 系の解除 API は in-flight のコールバックの完了を
  待ってから返る (コールバック内からの自己解除は待たない — R3 と同じ自己判定)。
  **remote push event は明示的な例外**で、frame decode と応答状態を一体に保つため canonical
  session lease 内で同期 dispatch する。同じ session の transaction / `Hal::pumpRemote()` /
  reconnect へは再入不可 ([remote.md](remote.md) §Hal facade)。
- **R7 (ロック階層)**: bus channel lock → bus state mutex (最内) の一方向のみ許可する。
  ServiceRunner の制御 mutex・GPIOGroup の mutex は「保持中に他のいかなるロックも取らない」
  意味での leaf lock として扱う。モジュール内から ServiceRunner を呼ぶ操作 (watch サービスの
  遅延登録等) は自モジュールの mutex を放してから行う。外部実装 (`IGPIO::read` 等) の呼び出しも
  自モジュールの mutex 保持中に行ってはならない。
- **R8 (タスク識別)**: `runtime::currentTaskId()` (opaque `void*`) を R1/R3/R6 の自己判定に
  用いる。実体・同値性契約は [runtime.md](runtime.md) §Task API 契約を参照。

## stub が対象外である理由

`runtime::Task` は posix・stub の双方に実装があり、API 形だけを見れば stub でも auto-run を
動かせるように見える。しかし stub の `runtime::Mutex` は実際の相互排他ではなく「非ブロッキングの
単一所有者ガード」(競合は timeout 値に関わらず即座に失敗) であり、`currentTaskId()` も呼び出し元
の実スレッドに関わらず固定値を返す ([runtime.md](runtime.md) §Task API 契約)。これは「stub は
単一の論理タスクしか存在しない」という前提の上で初めて正しく、その前提を stub 上で実際に複数
スレッドを並行させて破ると R1 (単一ライタ判定) や R8 (自己判定) が成立しなくなる。したがって
auto-run のマルチスレッド化は posix のみを対象とし、stub は意図的に除外する
(`ESP_PLATFORM`/`ARDUINO`/posix 判定の `#if` に stub は含まれない)。stub の決定性はテストの
資産であり、崩さない。

## 検証可能性: TSan native レーン

例外有効native (posix runtime) ビルドでは `runtime::Task`/`runtime::Mutex` が実体 (`std::thread`/
`std::timed_mutex`) を持つため、上記 R1-R8 の並行性契約を ThreadSanitizer (`-fsanitize=thread`)
つきの gtest から機械検証できる。`test/v2/native/core/test_service_runner/test_service_runner.cpp`
の `ServiceRunnerAutoRunTest` フィクスチャが該当する (2 スレッド `startAutoRun()` の冪等性、
複数callerの同時`add`によるtable容量結果、異なるserviceへの同時`add`/`remove`と返却後poll停止の
R1/R2/R3/R8検証)。実行は `pio test -e test_native_tsan`
(詳細は [verification.md](../verification.md))。

## 各 kind との関係

- **software I2C/SPI**: 状態機械の駆動を `IService` として登録する。svc タスクとコンシューマ
  (ユーザースレッド) 間の完了通知は R5 の Gate プリミティブ (`service::CompletionGate`) に
  従う。詳細は [i2c.md](i2c.md) / [spi.md](spi.md)。
- **GPIO watch**: `GPIOGroup` が watch サービスを `ServiceRunner` に登録してポーリングを駆動する
  (`bindServiceRunner`)。コールバック契約は R6、mutex は R7 の leaf lock。詳細は
  [gpio.md](gpio.md) §watcher API。
- **リモートバス**: `RemoteWireService` が frame mux の wire I/O を service poll ループの中で
  駆動する。host の標準 `RemoteSession` は 1 in-flight で、complete RPC と push callback を
  canonical session gate 内で直列化する。proxy 経路のロック順序は bus/channel → session。
  callback の R6 例外と再入禁止を含む詳細は [remote.md](remote.md)。
- **UART**: 現状 auto-run 上のサービスとしては駆動しない (再設定の静止条件は
  [uart.md](uart.md) §state mutex と再設定を参照)。R1-R8 のうち R5 (Gate) はチャネル状態の
  受け渡しに準用する。

## 関連

- [runtime.md](runtime.md) — `runtime::Mutex`/`runtime::Task`/`currentTaskId()` の variant 実装・
  Task API 契約
- [bus_accessor.md](bus_accessor.md) §排他制御の意味論 — Bus 層の mutex 契約 (ServiceRunner の
  制御 mutex とは別レイヤ)
- [gpio.md](gpio.md) §watcher API — GPIO watch サービスの kind 固有差分
- [i2c.md](i2c.md) / [spi.md](spi.md) — software backend の service 駆動と `CompletionGate`
- [remote.md](remote.md) — `RemoteWireService`
