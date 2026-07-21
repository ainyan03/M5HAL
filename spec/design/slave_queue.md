# design/slave_queue — slave queueと非同期受付の共通契約

> **読者**: 実装者・レビュー向け（設計仕様）。

I2C/SPI slaveは、外部masterがいつ通信を開始するかをapplication taskから予測できない。
正準APIはcaller-owned queueへTX予定dataを事前投入し、既に受信されたRX dataを後から取り出す。
`beginAccess` / `endAccess`はslave engineの受付期間を表し、local queue I/Oの可否を表さない。

## 所有とstorage

`slave::SlaveQueue`はheap容量を暗黙確保しない。callerがbyte領域とframe descriptor配列を渡す。

```cpp
template<class Descriptor = slave::FrameDescriptor>
struct QueueStorage {
    data::DataSpan bytes;
    Descriptor* descriptors;
    size_t descriptor_count;
};

slave::StaticSlaveQueueStorage<TxBytes, RxBytes, TxFrames, RxFrames> storage;
```

一方向queueにつきproducer一つ、consumer一つのSPSC契約である。TXはapplicationがproducer、backendが
consumer。RXはbackendがproducer、applicationがconsumer。payload/descriptorを先に書き、releaseで
indexをpublishし、consumerがacquireで観測する。descriptor自体へatomicやvptrを入れない。

Accessor、queue、storage、event callbackの寿命はbackend producerより長くなければならない。
`endAccess`はproducerを停止・退去させてからcaller pointerをdetachする。期限内に停止できないbackendは
hard abortまたは明示errorを返し、caller storageへ後から触れてはならない。

## Byte / Frame mode

TXとRXはそれぞれ`QueueMode::Byte`または`QueueMode::Frame`へ固定する。未消費dataまたは有効reservationが
ある間はmode変更を`INVALID_STATE`で拒否する。暗黙mode選択や同一directionのbyte/frame混在は行わない。

byte API:

- `write(ConstDataSpan)`はenqueueしたbyte数を返す
- `read(DataSpan)`はdequeueしたbyte数を返す
- 空queueへのnon-zero read、満杯queueへのnon-zero writeは`WOULD_BLOCK`
- short successはerrorではない。未読RXを上書きしない

frame API:

- consumerは最大2 spanの`FrameView`を`peekFrame()`し、`popFrame()`で一体消費する
- producerは`reserveFrame` → payload書込み → `commitFrame`、または`writeFrame`を使う
- reservationはmove-only tokenで識別し、stale/wrong-owner/double commitを拒否する
- zero-length frameもdescriptor一個を消費する
- Access境界を越える未commit reservationは禁止。`endAccess`はcancel後もbackend終了とunlockを続行し、
  より強いcleanup errorがなければ`INVALID_STATE`を返す

## metadataとkind固有detail

共通`FrameMetadata`は`frame_id`、`wire_bytes`、`stored_bytes`、`dropped_bytes`、`FrameFlags`、
`error`、`segment_count`を持つ。payload offset/lengthとcounterは32-bitで、飽和またはbind時rejectにより
silent truncateしない。

protocol固有detailを共通descriptorへ埋め込まない。I2Cは別storageの`I2cFrameSegment`をcommon frameと
atomicにpublish/popし、START/repeated START/direction/offset/lengthを表す。detail capacityだけが不足した
場合もcommon frameを保持し、metadataでdetail欠落を観測可能にする。SPIは一つのCS区間を一つのcommon
frameとし、初期APIにSPI固有segment配列を持たない。

## overflow、underrun、status

RX容量不足では未読dataを上書きせず、新着の保存できないsuffixをdropする。TX不足はAccessConfigの
backend capabilityに従いfill/stretch/NACK等でwireを進めるが、存在しない実dataを消費済みにしない。
`QueueStatus`はdropped byte/frame、underrun回数、sticky eventを32-bit counterで保持する。
status clearはsnapshotを返し、queue dataを消去しない。queue dataの`clearTx/clearRx`はinactive時だけ許可する。

## event

callbackはfunction pointer + `void*`で登録し、ISRからuser callbackを直接呼ばない。backendはlevel eventを
publishし、task文脈の`dispatchEvents()`または`eventService()`が配送する。同一generation・同一pending bitは
ownerの`acknowledgeEvents()`まで再配送しない。acknowledgeはclear後にlevelを再確認し、競合更新を失わない。

主なeventはRX available、TX space、frame completed、overflow、underrun、bus brokenである。eventは通知で
あってqueue dataの所有権移動ではないため、consumerは配送後にqueueを再確認する。

## Access lifecycle

`beginAccess`はlock取得、OperationContext runtime初期化、backend `beginOperation`、外部受付開始を行う。
`endAccess`は新規受付をfenceし、進行中wire frameを期限内に完了またはabortし、backend `endOperation`、
lock解放を行う。Accessはnon-nestable。一つのAccessに任意個の外部master frameが到着し得る。

`beginOperation/endOperation`はAccessor-owned Contextを検査するnon-virtual入口で、providerの派生点は
`beginOperationBackend/endOperationBackend`である。Bus・Accessor・generation・live lock owner不一致は
`INVALID_STATE`。backend endの成否にかかわらず、worker/ISRがquiesceしてhookから戻った後にContext slotを失効する。

TX preloadとinactive後のRX drainは許可するため、local `read/write`はAccess外でも使える。Config変更、callback
交換、queue clear、mode変更はinactiveかつ該当queueが安全な状態でだけ許可する。
legacy I2C wire-frame read/writeも別契約であり、checked queue lifecycleへ混ぜない。

backendが要求mode、frame境界、abort、lifetime fenceを正確に実装できない場合、近似せず
`UNSUPPORTED`を返す。silent fallbackでByteをFrameとして申告したり、複数wire frameを一つへ併合しない。

## 関連

- [bus_accessor.md](bus_accessor.md)
- [i2c_slave.md](i2c_slave.md)
- [spi.md](spi.md)
- [errors.md](errors.md)
