# design/memory — temporary memory allocator

M5HAL v1 は短寿命の作業領域を扱うための memory allocator を持つ。主用途は I2C / SPI / service / future stream helper の一時バッファであり、長寿命 object graph や汎用 C++ allocator の置き換えは目的にしない。

## 目的

- 頻繁に発生する小〜中サイズの一時確保を heap fragmentation から切り離す
- `malloc` を直接呼ばずに、M5HAL 内部で共通の一時バッファ取得経路を持つ
- pool 枯渇時や大きすぎる要求では fallback できる
- 将来 ESP-IDF の `heap_caps_malloc` / PSRAM / DMA-capable allocation に差し替えられる入口を残す

## 非目的

- STL allocator 互換 interface は初期スコープ外
- variable-size heap allocator の完全実装は初期スコープ外
- ISR-safe allocation は提供しない
- v0 側への導入は行わない

## 公開面

配置は `m5::hal::v1::memory` とする。

```cpp
namespace m5::hal::v1::memory {

enum class usage_t : uint8_t {
    temp,
    persistent,
    persistent_slow,
};

class Allocator {
public:
    using malloc_fn_t = void* (*)(size_t, usage_t);
    using realloc_fn_t = void* (*)(void*, size_t preserve_size, size_t new_size, usage_t);
    using free_fn_t   = void (*)(void*);

    void* allocate(size_t size, usage_t usage = usage_t::temp);
    void* reallocate(void* ptr, size_t preserve_size, size_t new_size, usage_t usage = usage_t::temp);
    void deallocate(void* ptr);

    void setFallback(malloc_fn_t malloc_fn, free_fn_t free_fn);
    void setFallback(malloc_fn_t malloc_fn, realloc_fn_t realloc_fn, free_fn_t free_fn);
};

class TempBuffer {
public:
    TempBuffer() = default;
    TempBuffer(Allocator& alloc, size_t size, usage_t usage = usage_t::temp);
    ~TempBuffer();

    TempBuffer(TempBuffer&& other) noexcept;
    TempBuffer& operator=(TempBuffer&& other) noexcept;
    TempBuffer(const TempBuffer&)            = delete;
    TempBuffer& operator=(const TempBuffer&) = delete;

    void* data() const;
    size_t size() const;
    explicit operator bool() const;

    bool reallocate(size_t size);
    void reset();
    void* release();
};

Allocator& defaultAllocator();

}  // namespace m5::hal::v1::memory
```

`M5HALCore` は allocator instance を member として所有する。

```cpp
m5::hal::v1::M5_Hal.Memory.allocate(128);
m5::hal::v1::memory::TempBuffer tmp{m5::hal::v1::M5_Hal.Memory, 128};
```

`defaultAllocator()` は `getM5_Hal().Memory` を返す。namespace-scope initializer から使う場合は `M5_Hal` ではなく `getM5_Hal()` 経由になる点は既存の `M5HALCore` 規約と同じ。

### API の位置づけ

通常の一時 buffer 所有には `TempBuffer` を使う。`TempBuffer` は現在の要求サイズを保持しているため、再確保時に caller が旧サイズを管理する必要がない。

`Allocator::allocate()` / `reallocate()` / `deallocate()` は低レベル API であり、driver helper や allocator wrapper が直接使う入口とする。特に `Allocator::reallocate(ptr, preserve_size, new_size, usage)` の `preserve_size` は caller が保持したい byte 数を渡す契約である。実際の確保済み容量ではなく「旧 buffer から copy したい data size」を渡すことを想定し、M5HAL は `min(preserve_size, new_size)` bytes を保持する。

`preserve_size` は意図的に旧確保サイズや旧 data size より小さくできる。`0` を渡せば旧内容を保持せず、容量変更だけを要求できる。これにより、呼び出し側は不要な copy / memmove を抑制できる。一方で、`preserve_size` が実際に読み取り可能な旧 buffer 範囲より大きい場合、fallback への copy で未定義の領域を読みうる。一般利用では `TempBuffer::reallocate()` を使い、低レベル API を直接使う場合だけこの責務を caller が負う。

## 初期実装

初期実装は単一の 256 byte fixed-block pool とする。まずは小分け対応を入れず、256 byte x 32 blocks の基礎性能を測る。

```text
default block size  : 256 bytes
default block count : 32 blocks
default pool size   : 8192 bytes
```

任意サイズの確保要求は block size 単位へ切り上げ、連続 block を貸す。

```text
   1..256 bytes  -> 1 block
 257..512 bytes  -> 2 blocks
1024 bytes       -> 4 blocks
8192 bytes       -> 32 blocks
```

この段階では tiny allocation の内部断片化は大きいが、探索・mark・unmark 対象が最大 32 blocks に収まる。512 byte〜4 KiB 付近の実用的な一時 buffer サイズで、管理コストを把握することを優先する。

次段では、各 256 byte block を 32 byte x 8 small blocks として小分けする hybrid pool を検討する。

## 設定

block count は 32 とし、bitmap を `uint32_t` ひとつで持つ。初期値は以下を想定する。

```cpp
#ifndef M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE
#define M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE 256
#endif

#ifndef M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT
#define M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT 32
#endif
```

制約:

- `M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE >= 4`
- `M5HAL_CONFIG_MEMORY_TEMP_BLOCK_SIZE` は 4 の倍数
- `M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT >= 1`
- `M5HAL_CONFIG_MEMORY_TEMP_BLOCK_COUNT <= 32`

`block_counts_` は確保開始 block にだけ連続 block 数を保持するため、`uint8_t` を使う。

## 内部データ構造

```cpp
template <size_t BlockSize, size_t BlockCount>
class FixedBlockPool {
    alignas(16) uint8_t storage_[BlockSize * BlockCount]{};
    uint32_t bitmap_;
    uint8_t block_counts_[BlockCount]{};
};
```

`bitmap_` は 1 bit = 1 block。`block_counts_[head]` は、その head block から何 blocks を確保したかを持つ。head 以外の `block_counts_` は 0 のまま。

解放時は pointer が pool 範囲内かつ block boundary 上にあることを確認し、`block_counts_[index]` の値から該当 bit 範囲を clear する。pool 外 pointer は `false` を返し、`Allocator` 側が fallback free へ回す。

## 再確保

`Allocator::reallocate(ptr, preserve_size, new_size, usage)` は、既存 buffer の内容を `min(preserve_size, new_size)` bytes まで保持しながら容量を変更する。`ptr == nullptr` は `allocate(new_size, usage)` と同じ、`new_size == 0` は `deallocate(ptr)` と同じ扱いにする。

この API は pointer を変更しうる。成功時は必ず戻り値を新しい所有 pointer として扱い、古い pointer は使わない。失敗時は `nullptr` を返し、旧 pointer の所有権と内容は維持される。

`usage_t::temp` かつ `ptr` が temp pool 所有の場合は、まず pool 内で再確保を試す。

- 旧 run の bit を外した `released_bitmap` を作る
- `released_bitmap` 上で旧 index に `new_size` 分の run が置ける場合はそこを優先する
- 旧 index に置けない場合は `released_bitmap` から新しい run を探す
- 新旧 run が同じ位置なら pointer は変えず、違う位置なら `preserve_size` を上限に data を移動する
- pool 内に十分な run がない、または pool サイズを超える場合は、Allocator 側で新規確保・copy・旧 buffer 解放へ fallback する

`FixedBlockPool::reallocate()` は metadata update と pool 内 copy を pool lock 内で完了させる。これは copy 中に旧 run が別 task へ再利用されることを避けるためである。pool block 数は最大 32 であり、temp buffer の pool 内移動は短い処理に留まる前提とする。

fallback pointer の再確保は `setFallback(malloc, realloc, free)` で登録された `realloc_fn_t` を優先する。fallback `realloc_fn_t` も `preserve_size` を copy 上限として扱う。`realloc_fn_t` が未登録の場合は、従来通り fallback `malloc` / copy / fallback `free` で動作する。pool pointer から fallback へ移る場合は fallback `realloc_fn_t` に pool pointer を渡さず、新規 fallback allocation に copy してから pool を解放する。

`TempBuffer::reallocate(size)` はこの `reallocate()` を使う RAII wrapper とする。呼び出し側は I2C / SPI などで必要容量を block size 単位に丸めて `reallocate()` すれば、pool 側で in-place grow 可能なケースでは追加 copy を避けられる。pointer は変わりうるため、呼び出し側は再確保後に `data()` を取り直す。

## 探索

`allocate(size)` は `needed = ceil(size / BlockSize)` を計算し、`needed` 個の連続 0 bit を探す。

初期実装は front/back first-fit とする。理由:

- pool は一時用途であり、default 32 blocks なら最悪走査も小さい
- 実装が単純で native test しやすい
- 将来最適化する場合も public API を変えずに済む

単純 first-fit は低番地側に小確保が偏るため、各探索で front 側と back 側の候補を固定順で見る。前回探索方向のような追加状態は持たず、読みやすさと低コストを優先する。

探索仕様:

- `needed > BlockCount` は pool 失敗
- pool 失敗時は fallback allocation を試す
- `size == 0` は `nullptr`

## fallback

`usage_t::temp` は pool を優先し、失敗時に fallback する。

`usage_t::persistent` / `persistent_slow` は pool を使わず fallback へ直接回す。初期実装では `std::malloc` / `std::free` を default fallback とする。

ESP-IDF / Arduino / native の環境差は `setFallback()` で吸収する。`ESP_PLATFORM` が有効な環境では `M5HALCore` 初期化時に `heap_caps_malloc` / `heap_caps_realloc` / `heap_caps_free` を fallback として登録する。ArduinoESP32 は ESP-IDF の上にあるため、この fallback は Arduino variant と ESP-IDF variant が同居する構成でも有効になる。

ESP-IDF fallback の usage mapping:

- `usage_t::temp`: temp pool 失敗時に `MALLOC_CAP_DEFAULT`
- `usage_t::persistent`: `MALLOC_CAP_DEFAULT`
- `usage_t::persistent_slow`: `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT` を優先し、失敗時は `MALLOC_CAP_DEFAULT`

native や ESP-IDF 以外の環境では、fallback hook 未設定時に `std::malloc` / `std::free` を使う。Allocator 本体の public API は framework-specific fallback に依存しない。

`setFallback()` は初期化時に呼ぶ設定 API とし、他 task が `allocate()` / `reallocate()` / `deallocate()` を実行している間に呼んではいけない。fallback allocator 自体のスレッドセーフ性は fallback 実装側に委ね、M5HAL は fallback 関数ポインタの動的差し替えを同期しない。

## lock / ISR

pool metadata の更新は pool 内部の `std::atomic_flag` による短時間の spin lock で保護する。`Allocator` は temp pool 優先 / fallback への経路制御だけを担当し、pool の `bitmap_` / `block_counts_` は `FixedBlockPool` 自身が守る。pool 所有範囲の pointer 判定は不変な storage 範囲を見るだけなので、metadata lock の対象にはしない。

`M5HALCore` が process-wide singleton であるため、複数 task から `M5_Hal.Memory` が使われる可能性を最初から前提にする。一方で、temp pool の更新範囲は非常に短く、ESP32 実機ベンチでは `portMUX` / FreeRTOS mutex より `std::atomic_flag` spin lock の固定費が大幅に小さいことを確認した。このため lock 方式は設定可能にせず、spin lock 固定とする。

採用しない方式:

- `portMUX_TYPE`: 割り込みを含めた保護は強いが、短い pool 操作には固定費が重い
- FreeRTOS mutex / `std::mutex`: 非競合時でも fixed-block temp pool には重い
- no-op lock: benchmark 用としては有用だが、通常仕様にはしない

`M5_Hal.Memory.allocate()` / `deallocate()` / `TempBuffer` は task context 専用であり、ISR から呼んではいけない。ESP32 では ISR から呼ぶ関数は `IRAM_ATTR` で IRAM に置く必要があり、allocator 本体だけでなく fallback や関連 helper まで ISR-safe に設計する必要がある。M5HAL の通常 temp allocator はその責務を負わない。

ISR で buffer が必要な場合は、task context で事前確保した storage を ISR に渡す。将来どうしても ISR 用の一時領域が必要になった場合は、通常 allocator とは別 API として設計する。

## 断片化

この pool は可変長要求を連続 fixed blocks で扱うため、外部断片化は起きる。例として、1 block 確保が飛び石で残ると大きな連続確保は失敗しうる。

M5HAL では以下の理由で許容する。

- 主用途は短寿命の一時バッファ
- `TempBuffer` で scope exit 解放を促す
- pool 失敗時は fallback がある
- default 256B x 32 blocks は 512 byte〜4 KiB 程度の要求を軽い管理コストで扱える

断片化の観測用に、初期 public API として以下を持つ。

```cpp
size_t usedBlocks() const;
size_t largestFreeRun() const;
```

## data::Source / Sink との関係

`TempBuffer` は raw byte storage の所有者であり、`data::MemorySource` / `MemorySink` は非所有 view である。組み合わせる場合は `TempBuffer` が view より長生きする必要がある。

```cpp
memory::TempBuffer tmp{M5_Hal.Memory, 128};
data::MemorySink sink{data::DataSpan{static_cast<uint8_t*>(tmp.data()), tmp.size()}};
```

この lifetime は caller / driver 側の責務とする。`TempBufferSource` / `TempBufferSink` のような所有 view は初期スコープ外。

## 現在の導入箇所

ESP-IDF I2C master backend は、IDF driver API に渡す write payload として `TransferDesc::prefix` と `data::Source` をひとつの連続 buffer にまとめる必要がある。この用途では、従来の `std::vector<uint8_t>` の代わりに `TempBuffer` を使う。

`data::Source` は残り総サイズを問い合わせる API を持たないため、完全な事前見積もりはできない。一方で memory-backed Source の典型ケースでは `peek(SIZE_MAX)` が残り全体を返すため、ESP-IDF I2C 用の helper は `prefix_len + first peek size` で初期容量を見積もり、容量は temp pool block size 単位へ切り上げる。後続 chunk が追加で現れた場合だけ block size 単位で `TempBuffer::reallocate()` する。

これにより、典型的な register write / write-read の一時連結 buffer は 256B x 32 blocks の temp pool から供給され、pool に収まらない大きな転送だけ fallback へ回る。拡張時に隣接 block が空いていれば pool 内で in-place grow でき、隣接 block が埋まっていても別 run があれば pool 内移動で継続できるため、I2C helper 側は再確保 copy の詳細を持たない。Source を直接 driver に渡せる backend では無理に `TempBuffer` へコピーしない。

## 実装順

1. `spec/design/memory.md` で本方針を固定
2. `src/m5_hal/hal/v1/memory/pool.hpp` / `.inl` に `FixedBlockPool` を追加
3. `src/m5_hal/hal/v1/memory/allocator.hpp` / `.inl` に `Allocator` / `TempBuffer` を追加
4. `M5HALCore` に `Memory` member を追加
5. `M5HAL_v1.cpp` に memory implementation include を追加
6. native test を追加
7. ESP-IDF I2C backend の一時 `std::vector` を `TempBuffer` へ置換する
8. 必要になった段階で他の I2C / SPI backend の一時 buffer を `TempBuffer` へ置換する

## native test

最低限の test:

- zero size returns nullptr
- 1 byte allocation consumes one block and succeeds
- block boundary allocation succeeds
- multi-block allocation succeeds
- full pool allocation succeeds
- pool oversize falls back
- pool exhaustion falls back
- deallocate nullptr is safe
- deallocate fallback pointer calls fallback free
- `TempBuffer` releases on destruction
- `TempBuffer` move ctor / move assignment transfers ownership
- `release()` gives up ownership
- `reallocate()` grows / shrinks pool allocation and preserves data
- `TempBuffer::reallocate()` preserves data
- fallback pointer reallocate uses fallback realloc when registered
- fragmented pool can still allocate from another free run
- block boundary 以外の pointer は pool deallocate に成功しない

## 採用判断

M5HAL の初期 allocator は、単一の 256 byte fixed-block pool とする。

理由:

- 「事前 pool から任意サイズを一時貸しする」という説明に素直
- 256B x 32 blocks の単純管理で基礎性能を測りやすい
- 512 byte〜4 KiB 付近で探索・mark・unmark の対象 block 数を抑えられる
- 実装が可変長 heap allocator より小さく、失敗時 fallback も単純
- public API は allocator として抽象化するため、将来 32 byte small block 対応 / arena / buddy allocator に差し替え可能
