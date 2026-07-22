# design/bus_capabilities — Bus instance capability

> **読者**: 利用者向け / 実装者・レビュー向け（設計仕様）。

`bus::IBus::capabilities()`は、取得済みの一つのBus instanceについて、利用できるoperationと数値上限を
`bus::BusCapabilities`として返す。I2C / SPI / UART / I2S / PDM、master / slave、local / remoteで
同じqueryを使う。

```cpp
const auto caps = acquired_bus->capabilities();
if (!caps.supports(m5::hal::v2::bus::BusFeature::MasterTransfer)) {
    return;
}
auto max_tx = caps.limit(m5::hal::v2::bus::BusLimit::MaxAtomicTxBytes);
if (max_tx.has_value()) {
    // Size the next atomic write phase to max_tx.value() or less.
}
```

これはpreflight用のsnapshotであり、operationの成功予約ではない。状態はquery後にも変化し得るため、実際の
`read` / `write` / `transfer`も結果を返す。非対応operationはpreflightでfalseまたは`UNSUPPORTED`、実行時も
`UNSUPPORTED`とする。

## 値と寿命

`BusCapabilities`は28 byteの固定長valueで、pointer、参照、shared ownership、heap allocationを持たない。
copyしたsnapshotはBusのclose、local backend hot-swap、remote disconnect後も安全に読め、その内容は変化しない。

- `supports(BusFeature)`は対応時true、未知のfeature IDを含め非対応時false
- `limit(BusLimit)`は提供時`result_t<uint32_t>`の値、未提供・未知IDは`UNSUPPORTED`
- `generation()`はsnapshotが属するBus backend/session世代を表すopaqueな`uint32_t`。
  [bus_accessor.md](bus_accessor.md) §OperationContext capabilityとchecked facade のOperationContext slot
  generation (Access単位の登録トークン) とは別の軸である

limit値0を「不明」のsentinelとして使わない。不明ならlimit自体を提供しない。既存scalar
`maxFrequency()`の0（ceiling未申告）とはこの点が異なる。既存scalar queryは互換のため残るが、genericな
operation選択の正本は`capabilities()`である。

## BusFeature

enumの数値はremote schemaのstable IDである。既存値を改番せず、新値は末尾へ追加する。

| 値 | ID | 意味 |
|---|---:|---|
| `ManagedAllocation` | 0 | facade型がmanaged allocation / hot-swapへ参加できる（現在のacquire経路や割当状態ではない） |
| `HardwareBackend` | 1 | 現在のinstanceがhardware/native backendを使う |
| `LowPowerBackend` | 2 | 現在のinstanceがlow-power controllerを使う |
| `MasterTransfer` | 3 | masterのatomic transferを提供する |
| `Transmit` | 4 | 送信方向を提供する |
| `Receive` | 5 | 受信方向を提供する |
| `FullDuplex` | 6 | 一つのoperationで送受信を同時に進められる |
| `MosiSharedRx` | 7 | SPI half-duplexでMOSIを受信線として共用できる |
| `SlaveByteTx` | 8 | slave Byte queueの送信方向を提供する |
| `SlaveByteRx` | 9 | slave Byte queueの受信方向を提供する |
| `SlaveFrameTx` | 10 | slave Frame queueの送信方向を提供する |
| `SlaveFrameRx` | 11 | slave Frame queueの受信方向を提供する |
| `SlaveLegacyWireFrame` | 12 | legacy wire-frame windowを提供する |
| `ClockStretch` | 13 | slave TX underrun時のclock stretchを提供する |
| `IsrRegMap` | 14 | slave ISR register-map flavorを提供する |

`FullDuplex`は`Transmit` / `Receive`の代用ではない。方向別判定は各方向のfeatureを使う。slave queueも粗い
`SlaveQueued`へまとめず、flavorと方向を個別に判定する。

## BusLimit

| 値 | ID | 単位と意味 |
|---|---:|---|
| `MaxFrequencyHz` | 0 | backendが申告する最大clock / sample frequency (Hz) |
| `MaxAtomicTxBytes` | 1 | 一つのwire write phaseに含められる最大byte数 |
| `MaxAtomicRxBytes` | 2 | 一つのwire read phaseで受け取れる最大byte数 |
| `MaxSlaveTransactionBytes` | 3 | slave backendが一transactionとして受理できる最大byte数 |

I2Cの`MaxAtomicTxBytes`はprefixを含むwire write phase全体を数える。Source payloadの上限は、この値から
`TransferDesc::prefix_len`を引いて求める。TX/RXは非対称になり得るため一つの
`AtomicTransferBytes`へ統合しない。callerが提供するqueue容量はBusのhard limitではないためここへ載せない。

## local snapshot

concrete providerは、実装済みのoperation、現在のconfig、controller、backend hard ceilingだけを申告する。
基底`IBus`は`backendKind()`がHardwareなら`HardwareBackend`、非0の`maxFrequency()`を
`MaxFrequencyHz`へ射影する。kind基底は、未実装の派生providerを過大申告しないためdata-path featureを
一律には追加しない。

runtime facadeはprovider snapshotへ、型の構造的能力として`ManagedAllocation`を合成する。direct / typed acquire
直後でもtrueになり得るため、現在resolver管理下かを表す状態bitとして使わない。backendの初期化、close、再初期化、
hot-swap時は全fieldを一つのcommitとしてmirrorし、generationを増やす。queryはbefore/after sequenceが一致するまで再読するため、異なる
世代のfeature/limitが混ざらない。取得済みsnapshotは不変で、次回queryだけが新世代を返す。

代表的な射影は次のとおり。

| instance | feature / limit |
|---|---|
| software I2C master | `MasterTransfer`, `Transmit`, `Receive` |
| software SPI master | `MasterTransfer` + 配線済み方向。MOSI/MISO両方なら`FullDuplex`、MISOなしMOSIありなら`MosiSharedRx` |
| Arduino I2C/SPI master | hardware + 実装するmaster/方向/full-duplex feature。I2Cはcoreから安全に得たRX buffer長を`MaxAtomicRxBytes`へ射影 |
| ESP-IDF I2C/SPI master | hardware + master + 配線済み方向。I2C LPは`LowPowerBackend`、SPIの`FullDuplex`/`MosiSharedRx`は配線で決まる |
| UART streaming backend | `Transmit`, `Receive`, `FullDuplex`; native peripheral providerはhardware |
| ESP-IDF I2S | configのDOUT/DINに応じた`Transmit` / `Receive` / `FullDuplex` |
| ESP-IDF PDM | hardware + `Receive` |
| I2C slave | 選択したByte/legacy flavor、方向、stretch、ISR regmapを個別申告 |
| ESP-IDF SPI slave | Byte/Frame TX/RX + `MaxSlaveTransactionBytes` |

## remote snapshotと互換性

remoteはserverのinstance snapshotを固定長recordへencodeし、hostで同じ`BusCapabilities`へdecodeする。
record自体のschemaは次のlength-bounded列である。

```text
[schema_version:u8 = 1]
[generation:u32 LE]
[feature_count:u8][feature_id:u8] * feature_count
[limit_count:u8]([limit_id:u8][value:u32 LE]) * limit_count
```

既知feature/limitだけの最大recordは42 byte。未知feature/limit IDはskipする。未知schema versionは、そのrecordを
保守的な空snapshotとして扱える`UNSUPPORTED`、truncate、count不整合、矛盾する既知limitの重複、余剰byteは
`PROTOCOL_ERROR`である。encode/decodeはcallerの固定bufferを使いheap allocationしない。

静的登録BusはHello extension、動的Busは`BusCreate`の`StoreData`でrecordを交換する。wire layoutと旧新互換は
[remote.md](remote.md)と[bytecode.md](bytecode.md)を参照する。旧serverまたは未知recordから能力を得られない
場合、hostはfeature/limitを推測せず空snapshotを使う。ただしhost自身が確定できるremote transportの
`MaxAtomicTxBytes`は追加する。RXはserver recordで実効上限が申告された場合だけwire上限との小さい方を採る。
serverはprovider上限に
`Server::Config::max_transfer_rx`とwire上限を重ねた実効RX上限を申告する。旧serverがrecordを返さない場合、
新hostはserver固有のRX予算を推測できないため`MaxAtomicRxBytes`を未提供のままにする。

静的・動的登録とも、物理Bus全体ではなくwireから到達できるAccessor bindingを射影する。例えばI2Sの
`TxAccessor`だけを登録したentryは`Transmit`のみを申告し、物理BusがRX対応でも`Receive` / `FullDuplex`や
RX limitを申告しない。

remote proxyのgenerationはserver内部generationをそのまま公開せず、hostのsession generationへ束縛する。
reconnect後に取得したBusは新generationと新能力を持つ。旧Busのoperationは`CLOSED`、以前取得したsnapshotは
不変値として残る。remote proxy上の`HardwareBackend`はhost側transport実装ではなく、server側でそのBusを
実行するconcrete providerがhardware/nativeであることを表す。

## connection capabilityとの区別

`Hal::capabilities()`が返す`remote::Capabilities`は、Helloで得た接続全体のGPIO公開、動的Bus生成、静的Bus一覧を
表す。`IBus::capabilities()`が返す`bus::BusCapabilities`は、取得済みの一Bus instanceのoperation能力とlimitで
ある。同名でも対象範囲が異なり、前者をper-operation preflightには使わない。

## なぜ固定長snapshotか

feature別virtual queryを増やす形は、generic codeとremote schemaの拡張点を分散させる。可変containerや
shared objectはqueryへallocation・寿命・同期問題を持ち込む。固定長valueなら全kindを同じコードで扱え、
hot-swap中も一貫したcopyを返し、wireでは既知IDだけをlength付きで交換できる。
