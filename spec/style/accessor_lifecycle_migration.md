# style/accessor_lifecycle_migration — 旧v2 lifecycle移行

> **読者**: 利用者向け。

旧v2のlock-only Accessとkind別transaction APIから、統一Accessor lifecycleへ移行するための
対応表。旧名aliasは提供しない。

## API対応

| 旧v2 | 現行v2 | 移行上の意味 |
|---|---|---|
| Accessor `beginTransaction()` / `endTransaction()` | `beginAccess(timeout)` / `endAccess(timeout)` | lock、設定、backend開始終了、最後のI/O完了を一つのlifecycleへ統合 |
| lock-only `beginAccess()` / `endAccess()` + nested depth | non-nestable Access | 二重beginは`INVALID_STATE`。sugarはactive Accessをborrowする |
| `endTransaction() -> TransferTotals` | 各`transfer`の戻り値 + `getLastTransferStatus()` | `endAccess`はlifecycle結果だけを返す |
| Bus `beginTransaction(owner, cfg)` / public virtual `beginOperation(owner, context)` | non-virtual `beginOperation(context)` / `endOperation(context)` + protected `*Backend` hook | ContextはAccessor-ownedで外部構築・copy不能。raw ownerを渡さず、Config copyを各I/Oで作らない |
| public `Bus::lock/unlock`, `ScopedLock` | Accessor内部lock seam | 利用者はAccessor lifecycleだけを使う |
| `spi::ScopedTransaction` | `bus::ScopedAccess` | SPI Accessが一つのCS frame |
| I2C slave `beginTransaction/endTransaction` | legacy `openWireFrame/closeWireFrame` | 外部masterが作ったframeのclaim/release。Access lifecycleではない |
| I2C slave `transactionComplete()` | `wireFrameComplete()` | master STOP観測query |
| `legacy_transaction_window` | `legacy_wire_frame_window` | 旧blocking stream受付を明示opt-in |

Arduino `SPIClass::beginTransaction/endTransaction`はframework native APIなので置換しない。
bytecodeのwire互換名は[bytecode.md](../design/bytecode.md)、Accessor/Bus間の契約は
[bus_accessor.md](../design/bus_accessor.md)を参照する。

## 呼出し形

旧SPI:

```cpp
dev.beginTransaction();
dev.write(command);
dev.write(payload);
auto totals = dev.endTransaction();
```

現行SPI:

```cpp
auto begun = dev.beginAccess(100);
if (!begun) return begun.error();
auto command_result = dev.write(command);
if (!command_result) {
    (void)dev.endAccess(1000);
    return command_result.error();
}
auto payload_result = dev.write(payload);
auto ended = dev.endAccess(1000);  // I/O errorを再返却しない
```

通常はRAIIを使う。

```cpp
bus::ScopedAccess access{dev, 100};
if (!access.ok()) return access.error();
auto sent = dev.write(payload);
if (!sent) return sent.error();
return access.finish(1000);
```

I2C masterでは同じAccess内の各`transfer`がそれぞれSTART〜STOPを持つ。SPIでは同じAccess内の複数
`transfer`が一つのCS frameを共有する。共通API名からwire境界を推測せず、kind仕様を確認する。

## slaveの移行

新規実装はcaller-owned queueの`i2c::SlaveAccessor` / `spi::SpiSlaveAccessor`を使う。

```cpp
slave::StaticSlaveQueueStorage<256, 256, 8, 8> storage;
i2c::StaticI2cSegmentStorage<16> segments;
i2c::SlaveAccessor endpoint{bus, storage.tx(), storage.rx(), segments.storage()};

endpoint.write(preloaded_reply);  // Access前TX preload
endpoint.beginAccess();           // slave受付開始
// callbackまたはtask loopでendpoint.read()/write()
endpoint.endAccess();             // 受付停止、producer退去
```

旧`SlaveStreamAccessor::serve()`を一時的に使う場合だけBusConfigの
`legacy_wire_frame_window = true`を指定する。queue lifecycleとlegacy windowは排他的であり、同じbackend
instanceで混在させない。

## porting checklist

1. Accessor旧`begin/endTransaction`を全削除し、明示scopeを`begin/endAccess`へ変更する
2. nested beginに依存するhelperを、active Accessのborrowまたは上位一回の`ScopedAccess`へ組み替える
3. Access-wide totals/sticky error依存を、各I/O resultと`TransferStatus`へ移す

provider実装のchecked入口、rollback、Context lifetimeは
[bus_accessor.md](../design/bus_accessor.md) §OperationContext capabilityとchecked facadeを参照する。slave workerの停止契約は
[slave_queue.md](../design/slave_queue.md)を参照する。

## 関連

- [../design/bus_accessor.md](../design/bus_accessor.md)
- [../design/slave_queue.md](../design/slave_queue.md)
- [../design/i2c_slave.md](../design/i2c_slave.md)
- [../design/spi.md](../design/spi.md)
