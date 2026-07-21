# PDM bus設計 (v2)

> **読者**: 実装者・レビュー向け（設計仕様）。

PDMはstandard I2Sとは独立した`pdm` namespaceと`BusKind::PDM`を持つ。scopeは、1本のdata inputから
hardware PDM-to-PCM filterで得る16-bit mono PCM RXである。WAV、codec、gain、mixing等の音声処理は上位層の
責務とする。

## 公開APIとscope

- `pdm::IBusConfig`: 必須の`pin_clk` / `pin_din`と`rx_buffer_size`。タグ型ctorは
  `pdm::BusConfig{pdm::Clk{clk}, pdm::Din{din}}`。
- `pdm::AccessConfig`: `sample_rate_hz`、`read_timeout_ms`、`bits_per_sample`、`channels`。
  現行backendは`bits_per_sample=16`かつ`channels=1`だけを受理する。
- `pdm::RxAccessor`: `data::StreamReader`として`read()` / `readableBytes()`を提供する。公開ライフサイクルは
  非ネストの`beginAccess()` / `endAccess()`で、開始時にRX lockを取得して`Bus::beginOperation`を1回呼び、
  終了時に`Bus::endOperation`を1回呼んでから解放する。これは物理フレーム境界でなく連続DMA streamの
  RX排他・設定適用区間である。read sugarは既存Accessを借用し、無ければ一時Accessを開閉する。
- Busの`beginOperation/endOperation/read/readableBytes`はAccessor-owned ContextのBus、Accessor、active、
  generation、live lock ownerを検査するnon-virtual入口である。providerはprotected
  `beginOperationBackend/endOperationBackend/readBackend/readableBytesBackend`だけをoverrideする。
- 各`read()`は個別の`result_t<size_t>`を返す。`getLastTransferStatus()`は直近read 1件のtransfer id、
  bytes、complete/partial/aborted、errorを保持し、Access全体のbyte totalsは集計しない。
- `M5_Hal.PDM.acquire(cfg)`はCLK/DINを`Pins` tagのidentityへ射影してbusをinternする。buffer sizeが異なる同一identityの
  再acquireは`INVALID_STATE`。

現行scopeにPDM TX、raw PDM byte stream、複数DIN lineは含めない。これらをPCM RXのmode fieldや無効pinとして
先行公開せず、SoC間で共通の意味と利用要求が確定した時点で方向別APIを追加する。standard I2SのBCLK/WS要件は
このPDM APIの追加によって変わらない。

## ESP-IDF backendとcapability

ESP-IDF gen5 `driver/i2s_pdm.h`を用いる。backendを提供するのは、headerに加えて
`SOC_I2S_SUPPORTS_PDM_RX`と`SOC_I2S_SUPPORTS_PDM2PCM`の両方が真のSoCだけである。PDM RXはclock master、
slot formatは16-bit mono PCM固定。対応しないSoCでraw modeへ暗黙縮退しない。

`read()`は受理できた偶数byte数を返し、timeoutは正常なshort readとして扱う。`readableBytes()`はDMA callbackで
取込済み未排出byteを追跡し、buffer capacityでclampする。overrunは古いsampleの欠落として継続する。
`closeBackend()`はchannelを削除し、CLK/DINを`gpio_reset_pin`でhigh-Zへ戻す。
要求`AccessConfig`はaccessor内の`OperationContext`が値として保持し、ESP-IDF backendは
`beginOperationBackend`でchannelを生成・再構成する。同じAccess内の個々の`readBackend`では再構成しない。
`endAccess`はDMAの物理停止を意味せず、停止は`close`または再構成時に行う。

受入範囲は対応SoCのbuild fenceとnativeでのAPI・remote protocol検証までであり、物理PDM microphoneからの
PCM captureは未受入である。sample品質、board固有の配線、実clock精度はportable APIの保証に含めない。
検証層と実行入口は[verification.md](../verification.md)を参照する。

## standard I2Sとのcontroller排他

PDMとstandard I2Sは公開kindを分けるが、ESP-IDFの同じI2S controller familyを消費する。両backendは共通leaseを
使い、同じcontrollerを同時取得しない。PDMは現行SoCでPDM対応するI2S0をclaimする。standard I2Sは番号の高い
controllerからclaimし、複数controller SoCではI2S0をPDM用に残す。単一controller SoCでは両kindは相互排他で、
空きがなければ`OUT_OF_RESOURCE`を返す。M5HAL外で先に取得されたcontrollerはIDFのchannel生成失敗で検出し、
standard I2Sは次のcontrollerを試す。

## remote wire

PDMはwire上も独立`BusKind`値9を使う。既存値0〜8は変更しない。

- `BusCreate` pin payload: `[clk:i16][din:i16][rx_buf_kb:u8]` (5 byte)
- `BusConfigure` payload: `[sample_rate:u32][read_timeout:u32][bits:u8][channels:u8]` (10 byte)
- sample data: 既存`BusStreamTransfer` + `Data` frameのRX stream

remote backendは`beginOperationBackend`で`BusConfigure`を送る。ただし現行protocolの各stream RPCは短く完結し、
server側Accessを複数RPCにまたがって保持する意味はまだ持たない。

serverはPDM専用bindingへ`pdm::RxAccessor`を登録する。旧peerは未知kindとして安全に拒否し、I2S payloadとして
解釈しない。
