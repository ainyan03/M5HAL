# I2S バス設計 (v1)

I2S (オーディオ用シリアルバス) の v1 抽象。当面のスコープは **master TX (再生)・
Philips standard・16bit・mono/stereo** で、raw PCM の搬送までを役割とする。
WAV 等のコンテナ解釈・デコード・ミキシングは上位 (アプリケーション / example) の責務
([goals.md](../goals.md) の「音声のドメインロジックは含めない」を維持する)。

## API 形 — uart 同型の非トランザクショナル bus

I2S は DMA 駆動の連続ストリームであり、I2C / SPI のような「開始 → 転送 → 終了」の
トランザクション境界を持たない。このため transfer 形ではなく
[uart](uart.md) と同型の write 系 API とする:

```cpp
struct I2SBus : bus::Bus {
    virtual expected<size_t, error_t> write(Accessor* owner, const I2SAccessConfig& cfg,
                                            data::Source* tx, size_t len);
    virtual expected<size_t, error_t> writableBytes(Accessor* owner, const I2SAccessConfig& cfg);
};
struct I2STxAccessor : bus::Accessor, data::StreamWriter;  // 再生 = StreamWriter としても振る舞う
```

- `I2STxAccessor` は [data_io.md](data_io.md) の `StreamWriter` を実装する。汎用ストリーム
  コードがそのまま音声出力先として使える。
- RX (録音) は将来拡張。`I2SBusConfig::pin_din` のみ予約済みで、accessor / read API は
  必要になった時点で uart 同型 (`I2SRxAccessor` + read / readableBytes) を追加する。

## 設定の分担

| 構造体 | 内容 | 所有 |
|---|---|---|
| `I2SBusConfig` | pins (bclk / ws / dout / din / mclk)、`tx_buffer_size` (DMA 総量の目安) | bus を生成・登録する側 (device) |
| `I2SAccessConfig` | `sample_rate_hz` / `bits_per_sample` (当面 16) / `channels` (1=mono, 2=stereo) / `timeout_ms` (bus lock) / `write_timeout_ms` | アクセスする側 |

他バスと同じく AccessConfig は呼び出しごとに渡され、backend は前回設定と異なる場合のみ
再構成する (明示的な start / stop API は置かない。再生開始 = 最初の write、停止 = write を
やめる)。

## write の意味論

- 戻り値は **受理できたバイト数** (DMA バッファへコピーできた量)。`write_timeout_ms` 内に
  全量を受理できなければ短い戻りになるが、これは**正常** (uart の short read と同じ思想)。
  `write_timeout_ms = 0` は non-blocking (いま入る分だけ受理)。
- **underrun はエラーにしない**: DMA が枯れたら無音を出力し、次の write から再開する。
  エラー扱いにしない理由は、連続再生では「途切れたら静かに継続」が常に正しい縮退であり、
  呼び出し側に回復処理を強いる価値がないため。
- `writableBytes()` は「いま write してもブロックしない量」。送信側のフロー制御の源泉で、
  リモートバス搬送 (将来: [remote.md](remote.md) の flow control) は backend の DMA 消費
  追跡をそのまま credit に使う。
- フレーム整列は呼び出し側の責務 (16bit stereo なら 4 byte 単位で渡す)。backend は
  端数バイトを受理しても良いが、サンプル境界をまたぐ分割で音声が壊れることはない
  (バイト列としてそのまま DMA へ並ぶ)。

## variant 規定

- **espidf**: ESP-IDF gen5 ドライバ (`driver/i2s_std.h`) を backend とする。legacy (gen4)
  ドライバは対応しない。ヘッダの有無で variant ごと有効化する (I2S 非搭載 SoC では IDF が
  ヘッダを公開しないことに依拠)。I2S ポートは自動割当 (`I2S_NUM_AUTO`。BusConfig に
  ポート番号フィールドは置かない — 必要になった時点で追加する)。`writableBytes` の源泉は
  **in-flight (送信済み未消費) バイト数の直接追跡**: write で加算し、送信完了コールバックで
  **0 でクランプしながら減算**する。クランプが本質 — underrun 中の無音バッファも送信完了
  コールバックを発火させるため、累積 submitted/consumed のカウンタ対では無音分がドリフトし
  free が容量に張り付く (実機で確認)。underrun 時は無音クリアを有効にする。
- **arduino**: 専用 variant は置かない。arduino-esp32 3.x は IDF 5.x を内包するため
  espidf variant を直接使う (2.x = IDF 4.4 は legacy ドライバのため対象外。ヘッダ検出に
  より自動的に無効となる)。

## リモートバス搬送

リモートバス機構による搬送は [remote.md](remote.md) §stream credit が正本。`writableBytes`
(= backend の DMA 消費追跡) がそのまま device 側の credit 報告の源泉になる。

## 将来拡張

- RX (録音 / マイク入力)
- 8 / 24 / 32 bit、PCM short / MSB 等のスロット形式
