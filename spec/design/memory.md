# design/memory — temporary memory allocator

> **読者**: 実装者・レビュー向け（設計仕様）。

M5HAL v2 は短寿命の作業領域を扱うための memory allocator を持つ。主用途は I2C / SPI / service / stream helper の一時バッファであり、長寿命 object graph や汎用 C++ allocator の置き換えは目的にしない。

## 目的

- 頻繁に発生する小〜中サイズの一時確保を heap fragmentation から切り離す
- `malloc` を直接呼ばずに、M5HAL 内部で共通の一時バッファ取得経路を持つ
- pool 枯渇時や大きすぎる要求では fallback できる
- ESP-IDFの`heap_caps_malloc` / PSRAM / DMA-capable allocation等へ差し替え可能な入口を持つ

## 非目的

- STL allocator 互換 interface は提供しない
- variable-size heap allocator は提供しない
- ISR-safe allocation は提供しない
- v0 側への導入は行わない

## 公開面

配置は`m5::hal::v2::memory`とする。`usage_t`が用途、`Allocator`がpoolとfallbackの選択、
`TempBuffer`が一時bufferのRAII所有を表す。宣言と既定引数の正本は
[allocator.hpp](../../src/m5_hal/hal/v2/memory/allocator.hpp)とし、この文書では所有権と再確保の意味論を定める。

```cpp
enum class usage_t : uint8_t {
    Temp,
    Persistent,
    PersistentSlow,
};
```

`Allocator`は`ResourceDomain` stateが所有し、`Hal::Memory`はそのdomain allocatorへの参照である。
domain copyと`Hal(domain)`は同じAllocatorを共有し、別domainは独立する。ライフサイクル・初期化規約は
[architecture.md](../architecture.md) §HAL object 層 を参照。

```cpp
m5::hal::v2::M5_Hal.Memory.allocate(128);
m5::hal::v2::memory::TempBuffer tmp{m5::hal::v2::M5_Hal.Memory, 128};
```

`defaultAllocator()` はcompatibility / bootstrap入口として`getM5_Hal().Memory`を返す。注入可能なlocal
backendは生成時に`LocalResourceContext.memory`を受け取る。namespace-scope initializerからは`M5_Hal`ではなく
`getM5_Hal()`経由で使う。

### API の位置づけ

通常の一時 buffer 所有には `TempBuffer` を使う。`TempBuffer` は現在の要求サイズを保持しているため、再確保時に caller が旧サイズを管理する必要がない。

`Allocator::allocate()` / `reallocate()` / `deallocate()` は低レベル API であり、driver helper や allocator wrapper が直接使う入口とする。特に `Allocator::reallocate(ptr, preserve_size, new_size, usage)` の `preserve_size` は caller が保持したい byte 数を渡す契約である。実際の確保済み容量ではなく「旧 buffer から copy したい data size」を渡すことを想定し、M5HAL は `min(preserve_size, new_size)` bytes を保持する。

`Allocator::isTempPoolAllocation(const void* ptr) const` は、`ptr` が現在有効な temp pool allocation の先頭 pointer である場合だけ `true` を返す。pool 内の interior pointer、解放済み pointer、fallback allocation は `false` とする。これは `usedBlocks()` の前後差から個別 allocation の帰属を推定できない共有 allocator の利用箇所で、取得した pointer 自体の帰属を確認するための低レベル API である。判定時点の snapshot であり、pointer の lifetime や所有権を延長しない。

`preserve_size` は意図的に旧確保サイズや旧 data size より小さくできる。`0` を渡せば旧内容を保持せず、容量変更だけを要求できる。これにより、呼び出し側は不要な copy / memmove を抑制できる。一方で、`preserve_size` が実際に読み取り可能な旧 buffer 範囲より大きい場合、fallback への copy で未定義の領域を読みうる。一般利用では `TempBuffer::reallocate()` を使い、低レベル API を直接使う場合だけこの責務を caller が負う。

## pool model

一時領域は単一のfixed-block poolから確保する。既定は256 byte x 32 blocksで、block内をさらに
小分けしない。

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

tiny allocationの内部断片化を許容する代わりに、探索・mark・unmark対象を最大32 blocksへ限定する。
512 byte〜4 KiB付近の一時bufferを小さいmetadataで扱うことを優先する。

## 設定

既定値と入力制約の正本は[configuration.md](configuration.md) §ノブ一覧とする。
block countは最大32なので、bitmapは`uint32_t`一つで持つ。

`block_counts_` は確保開始 block にだけ連続 block 数を保持するため、`uint8_t` を使う。

## 内部データ構造

実際の型定義 (`FixedBlockPool<BlockSize, BlockCount>`) は [pool.hpp](../../src/m5_hal/hal/v2/memory/pool.hpp) を正本とする。
`bitmap_` は 1 bit = 1 block。`block_counts_[head]` は、その head block から何 blocks を確保したかを持つ。head 以外の `block_counts_` は 0 のまま。

解放時は pointer が pool 範囲内かつ block boundary 上にあることを確認し、`block_counts_[index]` の値から該当 bit 範囲を clear する。pool 外 pointer は `false` を返し、`Allocator` 側が fallback free へ回す。

## 再確保

`Allocator::reallocate(ptr, preserve_size, new_size, usage)` は、既存 buffer の内容を `min(preserve_size, new_size)` bytes まで保持しながら容量を変更する。`ptr == nullptr` は `allocate(new_size, usage)` と同じ、`new_size == 0` は `deallocate(ptr)` と同じ扱いにする。

この API は pointer を変更しうる。成功時は必ず戻り値を新しい所有 pointer として扱い、古い pointer は使わない。失敗時は `nullptr` を返し、旧 pointer の所有権と内容は維持される。

`usage_t::Temp` かつ `ptr` が temp pool 所有の場合は、まず pool 内で再確保を試す。

- 旧 run の bit を外した `released_bitmap` を作る
- `released_bitmap` 上で旧 index に `new_size` 分の run が置ける場合はそこを優先する
- 旧 index に置けない場合は `released_bitmap` から新しい run を探す
- 新旧 run が同じ位置なら pointer は変えず、違う位置なら `preserve_size` を上限に data を移動する
- pool 内に十分な run がない、または pool サイズを超える場合は、Allocator 側で新規確保・copy・旧 buffer 解放へ fallback する

`FixedBlockPool::reallocate()` は metadata update と pool 内 copy を pool lock 内で完了させる。これは copy 中に旧 run が別 task へ再利用されることを避けるためである。pool block 数は最大 32 であり、temp buffer の pool 内移動は短い処理に留まる前提とする。

fallback pointer の再確保は構築時の `FallbackOps` に `realloc_fn_t` があればそれを優先する。fallback `realloc_fn_t` も `preserve_size` を copy 上限として扱う。省略時は fallback `malloc` / copy / fallback `free` で動作する。pool pointer から fallback へ移る場合は fallback `realloc_fn_t` に pool pointer を渡さず、新規 fallback allocation に copy してから pool を解放する。

`TempBuffer::reallocate(size)` はこの `reallocate()` を使う RAII wrapper とする。呼び出し側は I2C / SPI などで必要容量を block size 単位に丸めて `reallocate()` すれば、pool 側で in-place grow 可能なケースでは追加 copy を避けられる。pointer は変わりうるため、呼び出し側は再確保後に `data()` を取り直す。

## 探索

`allocate(size)` は `needed = ceil(size / BlockSize)` を計算し、`needed` 個の連続 0 bit を探す。

探索はfront/back first-fitとする。理由:

- pool は一時用途であり、default 32 blocks なら最悪走査も小さい
- 実装が単純で native test しやすい
- allocatorのpublic APIを探索方式から分離できる

単純 first-fit は低番地側に小確保が偏るため、各探索で front 側と back 側の候補を固定順で見る。前回探索方向のような追加状態は持たず、読みやすさと低コストを優先する。

探索仕様:

- `needed > BlockCount` は pool 失敗
- pool 失敗時は fallback allocation を試す
- `size == 0` は `nullptr`

## fallback

`usage_t::Temp` は pool を優先し、失敗時に fallback する。

`usage_t::Persistent` / `usage_t::PersistentSlow` は pool を使わず fallback へ直接回す。default fallbackは `std::malloc` / `std::free` とする。

fallback family は `Allocator` の構築時に `FallbackOps` として値渡しし、object lifetime中は固定する。全関数がnullなら標準 `malloc` / `realloc` / `free` family、custom familyはmalloc/freeの組を必須、reallocだけ任意とする。組が不正ならdebugでassertし、releaseでは混成familyを使わず標準familyへ退避する。確保後の差し替えを許さないため、確保元と解放元が分離しない。

`ESP_PLATFORM` が有効なdefault HALでは、`M5HALCore` がHAL初期化前に `heap_caps_malloc` / `heap_caps_realloc` / `heap_caps_free` を持つ `ResourceDomain` を構築する。ArduinoESP32 は ESP-IDF の上にあるため、この family は Arduino variant と ESP-IDF variant が同居する構成でも有効になる。独立default構築した `ResourceDomain` は他環境と同じ標準familyを使う。

ESP-IDF fallback の usage mapping:

- `usage_t::Temp`: temp pool 失敗時に `MALLOC_CAP_DEFAULT`
- `usage_t::Persistent`: `MALLOC_CAP_DEFAULT`
- `usage_t::PersistentSlow`: `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT` を優先し、失敗時は `MALLOC_CAP_DEFAULT`

native や ESP-IDF 以外の環境では、fallback hook 未設定時に `std::malloc` / `std::free` を使う。Allocator 本体の public API は framework-specific fallback に依存しない。

custom fallback実装自体のスレッドセーフ性は提供側に委ねる。M5HAL側の関数組は構築後不変なので、動的更新の同期やlive allocation追跡は持たない。custom allocatorを使う `ResourceDomain` / Mux / BlockSource は参照先allocatorより短く生存させる。

## lock / ISR

pool metadata の更新は pool 内部の `std::atomic_flag` による短時間の spin lock で保護する。`Allocator` は temp pool 優先 / fallback への経路制御だけを担当し、pool の `bitmap_` / `block_counts_` は `FixedBlockPool` 自身が守る。pool storage 範囲の pointer 判定は不変な範囲を見るだけなので lock 不要だが、有効な allocation 先頭かを判定する `isTempPoolAllocation()` は `block_counts_` を pool lock 下で参照する。

同一domainのAllocatorは複数taskから共有され得ることを前提にする。一方で、temp poolの更新範囲は非常に短く、
ESP32実機ベンチでは`portMUX` / FreeRTOS mutexより`std::atomic_flag` spin lockの固定費が大幅に小さいことを
確認した。このためlock方式は設定可能にせず、spin lock固定とする。

採用しない方式:

- `portMUX_TYPE`: 割り込みを含めた保護は強いが、短い pool 操作には固定費が重い
- FreeRTOS mutex / `std::mutex`: 非競合時でも fixed-block temp pool には重い
- no-op lock: benchmark 用としては有用だが、通常仕様にはしない

`M5_Hal.Memory.allocate()` / `deallocate()` / `TempBuffer` は task context 専用であり、ISR から呼んではいけない。ESP32 では ISR から呼ぶ関数は `IRAM_ATTR` で IRAM に置く必要があり、allocator 本体だけでなく fallback や関連 helper まで ISR-safe に設計する必要がある。M5HAL の通常 temp allocator はその責務を負わない。

ISR で buffer が必要な場合は、task context で事前確保した storage を ISR に渡す。ISR用の一時領域を追加する場合も、通常allocatorとは別APIとし、通常allocatorへISR-safe責務を混在させない。

## 断片化

この pool は可変長要求を連続 fixed blocks で扱うため、外部断片化は起きる。例として、1 block 確保が飛び石で残ると大きな連続確保は失敗しうる。

M5HAL では以下の理由で許容する。

- 主用途は短寿命の一時バッファ
- `TempBuffer` で scope exit 解放を促す
- pool 失敗時は fallback がある
- default 256B x 32 blocks は 512 byte〜4 KiB 程度の要求を軽い管理コストで扱える

断片化の観測用に、以下のpublic APIを持つ。

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

この lifetime は caller / driver 側の責務とする。`TempBufferSource` / `TempBufferSink` のような所有viewは提供しない。

## ESP-IDF I2Cとの統合

ESP-IDF I2C master backendは、IDF driver APIに渡すwrite payloadとして`TransferDesc::prefix`と
`data::Source`を一つの連続bufferにまとめる必要がある。この用途では`TempBuffer`を使う。

`data::Source` は残り総サイズを問い合わせる API を持たないため、完全な事前見積もりはできない。一方で memory-backed Source の典型ケースでは `peek(SIZE_MAX)` が残り全体を返すため、ESP-IDF I2C 用の helper は `prefix_len + first peek size` で初期容量を見積もり、容量は temp pool block size 単位へ切り上げる。後続 chunk が追加で現れた場合だけ block size 単位で `TempBuffer::reallocate()` する。

これにより、典型的な register write / write-read の一時連結 buffer は 256B x 32 blocks の temp pool から供給され、pool に収まらない大きな転送だけ fallback へ回る。拡張時に隣接 block が空いていれば pool 内で in-place grow でき、隣接 block が埋まっていても別 run があれば pool 内移動で継続できるため、I2C helper 側は再確保 copy の詳細を持たない。Source を直接 driver に渡せる backend では無理に `TempBuffer` へコピーしない。

## 検証契約

テストで担保する契約: zero-size / 1B / block boundary / multi-block / pool full の各確保が正しく動き、pool 枯渇・oversize では fallback へ降りる。`nullptr` deallocate / fallback pointer の deallocate は安全。`TempBuffer` は RAII・move・release・reallocate でデータを保全し、断片化した pool でも別 run から確保できる。block boundary 以外の pointer は pool deallocate に成功しない。

検証入口は[verification.md](../verification.md)を参照する。

## 採用判断

M5HALのtemporary allocatorは、単一の256 byte fixed-block poolとする。

理由:

- 「事前 pool から任意サイズを一時貸しする」という説明に素直
- 256B x 32 blocks の単純なmetadataで挙動を検証しやすい
- 512 byte〜4 KiB 付近で探索・mark・unmark の対象 block 数を抑えられる
- 実装が可変長 heap allocator より小さく、失敗時 fallback も単純
- public APIをallocatorとして抽象化し、pool内部の探索・配置方式を公開契約から分離できる
