# design/bytecode — HAL 命令ストリーム (encoder + runner)

**bytecode** は v1 HAL の操作 (バス転送・GPIO・遅延) を表す、コンパクトで自己記述的な命令列。namespace は `m5::hal::v1::bytecode` (`src/m5_hal/hal/v1/bytecode/`)。

第一級ユースケースは **ローカルの byte 配列からの直接実行** — デバイス初期化シーケンス等を const テーブルとして持ち、`runner.run(ConstDataSpan{table, len})` で再生する。同じスクリプトはそのまま [frame codec](frame.md) (kind=`data`) に載せてリモート実行へ運べる。

由来: LovyanAPI lxyz `extensions/bytecode` (HAL 形 dispatch) と lapi `extensions/bytecode` (自己記述命令・store スロット・対称パイプライン) のハイブリッド移植。wire はいずれとも非互換の新規定義で、本フォーマットを **M5HAL bytecode v1** と呼ぶ。

## ワイヤフォーマット (M5HAL bytecode v1)

```
命令     : [LenVar size][opcode:1][payload:size-1]    (size は opcode を含む)
終端     : LenVar 0 (1 byte の 0x00)。入力の終わりも正常な終端
```

- マルチバイト整数は **little endian** 統一
- **LenVar (LE)**: `0x00-0xFC` = その値 1 byte / `0xFD` + u16 LE / `0xFE` + u32 LE / `0xFF` 予約 (出現したら `PROTOCOL_ERROR`)
- **前方互換**: すべての命令が長さ前置なので、未知 opcode は size 分スキップして続行できる
- **critical フラグ (opcode bit7)**: 「黙って無視されては困る」将来命令のための区分。未知 opcode は bit7=0 ならスキップ (件数は `unknownSkipped()` で観測可)、bit7=1 なら `PROTOCOL_ERROR` で停止

### opcode 一覧

| opcode | 値 | payload | 意味 |
|---|---|---|---|
| `delay_ms` | 0x01 | `[ms:u32]` | ミリ秒遅延 |
| `bus_configure` | 0x10 | `[kind:1][bus_id:1][cfg...]` | 登録済み accessor の AccessConfig を更新 |
| `bus_transfer` | 0x11 | `[kind:1][bus_id:1][store_id:1][rx_len:LenVar][meta_size:1][meta][tx...]` | バス転送。tx 長は命令 size から自己記述 |
| (予約) | 0x12 / 0x13 | — | bus_init / bus_deinit (バス生成は当面ホストアプリの責務) |
| `gpio_set_mode` | 0x20 | `[mode:1]([gpio_num:u16])*` | ピン列のモード設定 |
| `gpio_write_high` | 0x21 | `([gpio_num:u16])*` | ピン列を High |
| `gpio_write_low` | 0x22 | `([gpio_num:u16])*` | ピン列を Low |
| `gpio_read` | 0x23 | `[store_id:1]([gpio_num:u16])*` | ピン列を読み、LSB 詰めのビット列をスロットへ |
| `gpio_subscribe` | 0x24 | `([gpio_num:u16])*` | ピン列を変化通知の購読に追加 (実行環境が購読機構を持たない場合 UNSUPPORTED。意味論は [remote.md](remote.md) §push イベント) |
| `gpio_unsubscribe` | 0x25 | `([gpio_num:u16])*` (空 = 全解除) | 購読解除 |
| `store_data` | 0x40 | `[store_id:1][data...]` | (応答) データをスロットへ |
| `report_error` | 0x41 | `[error:i8][offset:LenVar]` | (応答) エラーと発生位置 |
| `report_complete` | 0x42 | `[status:i8]` | (応答) 完了通知 |
| `evt_gpio_state` | 0x60 | `([gpio_num:u16][level:u8])*` | (イベント) 変化したピンと新レベル。受信側 runner はハンドラ未登録なら黙って無視 |
| `evt_stream_credit` | 0x61 | `[kind:1][bus_id:1][free:u32][submitted:u32]` | (イベント) stream 系バスの credit スナップショット。意味論は [remote.md](remote.md) §stream credit |
| `bus_write_stream` | 0xB0 (critical) | `[kind:1][bus_id:1][data...]` | stream 系バス (現在 I2S) への書き込み。data 長は命令 size から自己記述。受理しきれない分は `BUFFER_OVERFLOW` |
| `bus_stream_status` | 0xB1 (critical) | `[kind:1][bus_id:1][store_id:1]` | `[free:u32][submitted:u32]` (LE) をスロットへ。`submitted` は binding ごとの累積受理バイト数 (mod 2^32、runner が保持) |

- `kind` は `types::bus_kind_t` の値 (I2C/SPI/UART/I2S)。`gpio_num` は統合 `gpio_number_t` 空間 (スロット込み) の u16 表現
- 0xB0/0xB1 が critical (bit7) なのは、これらを知らない実行環境による黙殺 = 「応答 OK なのに無音」を `PROTOCOL_ERROR` で即検出するため ([remote.md](remote.md) §stream credit)
- `store_id` は応答データのラベル (任意値)。`0xFF` は「読み捨て」
- GPIO 命令がピン**列**を取るのは M5HAL の GPIO モデル (GPIOGroup → 個別 Pin) に合わせた形。port 一括の value/mask 形式は採らない

### bus_configure の cfg payload

accessor の現在の config を起点に、既知フィールドだけ上書きする**寛容 decode** (payload が短ければ前半のみ適用、長ければ余剰をスキップ — config 拡張の前方互換)。

| kind | layout (LE) | 計 |
|---|---|---|
| I2C | `freq:u32, wire_timeout_ms:u32, i2c_addr:u16, flags:u8 (bit0=10bit, bit1=use_restart), register_address_bytes:u8` | 12 B |
| SPI | `pin_cs:i16, pin_dc:i16, freq:u32, data_mode:u8, mode:u8 (bit0-1=spi_mode, bit2=order), cmd_len:u8, addr_len:u8, read_dummy:u8, write_dummy:u8` | 14 B |
| UART | `baud:u32, first_byte:u32, inter_byte:u32, write_timeout:u32, data_bits:u8, stop_bits:u8, parity:u8, invert:u8` | 20 B |
| I2S | `sample_rate:u32, write_timeout:u32, bits_per_sample:u8, channels:u8` | 10 B |

### bus_transfer の meta

| kind | layout | 計 |
|---|---|---|
| I2C | `prefix_len:u8, prefix bytes (≤8)` → `i2c::TransferDesc` の prefix | 1-9 B |
| SPI | `flags:u8 (bit0=dc_valid, bit1=dc_level), cmd_dc:i8, addr_dc:i8, data_dc:i8, command:u32, address:u32, command_bytes:u8, address_bytes:u8, dummy_cycles:u8` | 15 B |
| UART | なし (`meta_size=0`)。tx を write し、`rx_len` byte を read | 0 B |

## BytecodeRunner

```cpp
m5::hal::v1::bytecode::BytecodeRunner runner;       // allocator は省略時 defaultAllocator()
runner.registerI2C(0, i2c_accessor);                // bus_id 0..3
runner.setGPIOGroup(M5_Hal.Gpio);                   // 統合 gpio_number_t 空間

static const uint8_t init_table[] = { /* ... */ };
auto consumed = runner.run(data::ConstDataSpan{init_table, sizeof init_table});
auto chip_id  = runner.storedData(7);               // gpio_read / bus_transfer の結果
```

- **実行パスは `data::Source` 1 本**: `run(ConstDataSpan)` は内部で `MemorySource` を被せる薄い overload。`StreamSource` (UART 直結) やファイル再生も同じ実装で動く
- **dispatch 先は事前登録制**: `registerI2C/SPI/UART(bus_id, accessor&)` 各 4 枠 + `setGPIOGroup`。未登録のターゲットを指す命令は `INVALID_ARGUMENT`
- **store スロット**: ラベル付き 8 枠 (`memory::TempBuffer` 所有)。`run` 開始時に全クリア。同一 `store_id` への再書き込みは上書き、9 個目の異なるラベルは `OUT_OF_RESOURCE`
- **エラー方針**: 最初に失敗した命令で停止しエラーを返す。`lastOffset()` がその命令の byte offset。応答化 (report) は呼び出し側が `writeResponse` で行う
- **実行は同期**: `delay_ms` を含む全命令が呼び出しスレッドをブロックする。長い delay 入りスクリプトは runner (remote server なら poll ループ) を専有する。remote server 側の実行時間の制限規約は [remote.md](remote.md) §server の実行モデルが定める
- **ストリーミング実行の前提**: 命令は Source が 1 回の `peek` で貸せるサイズであること (`StreamSource` なら scratch ≥ 最大命令長)。命令の続きが timeout 期間を丸ごと待っても届かない場合は `BUFFER_UNDERFLOW` で停止する (途中再開は不可。`StreamSource` の peek は不足分を timeout までブロックして待つため、バイト間ギャップ程度では失敗しない)

## 対称パイプライン (応答も bytecode)

read 結果の往復に専用フォーマットを持たない。device 側の `writeResponse(sink, status)` がスロットを `store_data` 命令列 + `report_*` + 終端として書き出し、host 側は **同じ Runner** でそれを実行する — `store_data` がスロットを埋め、`report_*` が `statusReported()` / `reportedStatus()` / `reportedOffset()` に記録される。

```
host                                device
BytecodeEncoder → script  ─frame→  BytecodeRunner::run (HAL dispatch)
BytecodeRunner::run       ←frame─  BytecodeRunner::writeResponse
  → storedData(id) / reportedStatus()
```

`data::Sink` を取るため、`MemorySink` (ローカル組み立て) にも `StreamSink` / [`FrameWriter`](frame.md) (送信直結) にも書ける。

## BytecodeEncoder

`data::Sink&` へ命令単位 (reserve → 構築 → commit) で書く。`MemorySink` で byte 配列を組み立て、`StreamSink` なら encode 即送信。append 系は `expected<void, error_t>` を返し、Sink が命令全長を貸せない場合は `CLOSED` / `BUFFER_OVERFLOW`。

主なメソッド: `delayMs` / `configure(bus_id, <kind>AccessConfig)` / `transfer(bus_id, <kind>TransferDesc, tx, rx_len, store_id)` / `uartTransfer` / `gpioSetMode` / `gpioWriteHigh` / `gpioWriteLow` / `gpioRead` / `storeData` / `reportError` / `reportComplete` / `end`。

## 互換性と版管理

- 本フォーマット (M5HAL bytecode v1) は**実験段階**であり、公開リリースノートで凍結を宣言するまでは非互換変更があり得る
- 凍結後の拡張は原則として**新 opcode の追加**で行う (size 前置 + critical フラグにより旧 runner とも共存できる)。命令レイアウト自体の非互換変更が必要になった場合は **v2 として別フォーマット名**を与える
- 予約 opcode の値は将来の版でも再利用しない。`report_error` が運ぶ error code は **i8 (-128..127)** であり、`error_t` の値域はこの範囲に収める ([error.hpp] 側にも明記)
- `report_error` / `report_complete` の payload は**前方互換的に拡張され得る** (例: 下層ドライバのネイティブエラーコード等の診断 detail を末尾に追加)。受信側は既知の prefix のみ読み、余剰バイトは無視すること (命令 size が境界を自己記述するため安全にスキップできる)

## 採用しない要素

| 要素 | 不採用理由 |
|---|---|
| 長さ前置なしの命令形式 | 未知 opcode でストリーム全体が解釈不能になり前方互換を持てない |
| 応答の Sink 直行ストリーミング | ローカル実行者が応答スクリプトの自前パースを強いられる。スロット + `storedData(id)` が簡潔 |
| Device 層 dispatch | v1 の dispatch 先は Accessor + TransferDesc。Device 抽象は HAL スコープ外 |
| bus 生成 (init/deinit) | バス生成ファクトリが必要。リモートバス機構 ([remote.md](remote.md)) でも bus は server 側静的登録で足りると判定し、init/deinit は引き続き値のみ予約 |
| Simple/Compound の 2 クラス命令 | 全命令が size 前置で自己記述なら不要。「無視されては困る」は critical フラグで表現 |

## 関連

- 実例: [`examples/v1/HowToUse/Bytecode`](../../examples/v1/HowToUse/Bytecode/) — byte 配列で記述した GPIO/I2C/SPI スクリプトをボタン操作で実行する sketch (M5Stack Core BASIC)
- フレーム化と remote 搬送: [frame.md](frame.md) (kind=`data` の kind_body にスクリプトを載せる)、メッセージ層は [remote.md](remote.md)
- Source/Sink 契約: [data_io.md](data_io.md)
- 検証: [../verification.md](../verification.md) (native gtest `test_bytecode` / posix UART ミニ remote 往復)
