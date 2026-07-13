// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_I2C_SLAVE_INL_
#define M5_HAL_HAL_V2_I2C_SLAVE_INL_

#include "slave.hpp"

namespace m5::hal::v2::i2c {

result_t<void> SlaveStreamAccessor::beginTransaction(uint32_t timeout_ms)
{
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const uint32_t start_ms = runtime::millis();
    for (;;) {
        auto r = getBus().beginTransaction(this, 0);
        if (r.has_value() || r.error() != error::error_t::TIMEOUT_ERROR || timeout_ms == 0) {
            return r;
        }
        if (timeout_ms != types::TIMEOUT_FOREVER && runtime::millis() - start_ms >= timeout_ms) {
            return r;
        }
        // Event-driven open: wake on the first RX/STOP ISR activity (with a short
        // safety timeout) so a sub-millisecond write is opened and drained before
        // its STOP. A fixed poll here opened too late for a fast small write, so the
        // RX ring stayed full at STOP and its FIFO tail was dropped.
        getBus().waitForActivity(this, 4);
    }
}

result_t<void> SlaveStreamAccessor::endTransaction(void)
{
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return getBus().endTransaction(this);
}

result_t<size_t> SlaveStreamAccessor::read(data::DataSpan dst)
{
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (dst.data == nullptr && dst.size != 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const uint32_t timeout_ms = getBus().getConfig().timeout_ms;
    const uint32_t start_ms   = runtime::millis();
    for (;;) {
        auto readable = getBus().readableBytes(this);
        if (!readable.has_value()) {
            return m5::stl::make_unexpected(readable.error());
        }
        if (readable.value() != 0 || dst.size == 0) {
            return getBus().read(this, dst);
        }

        auto complete = getBus().transactionComplete(this);
        if (!complete.has_value()) {
            return m5::stl::make_unexpected(complete.error());
        }
        if (complete.value()) {
            // TOCTOU guard (same race as the serve() loops): the STOP ISR can
            // deliver the final FIFO bytes into the ring between the
            // readableBytes()==0 above and this complete check. Re-check before
            // reporting end-of-transaction, so a caller treating 0 as EOF does
            // not discard the tail.
            auto readable_after = getBus().readableBytes(this);
            if (!readable_after.has_value()) {
                return m5::stl::make_unexpected(readable_after.error());
            }
            if (readable_after.value() != 0) {
                return getBus().read(this, dst);
            }
            return size_t{0};
        }
        if (timeout_ms == 0 || (timeout_ms != types::TIMEOUT_FOREVER && runtime::millis() - start_ms >= timeout_ms)) {
            return size_t{0};
        }
        runtime::delayMs(1);
    }
}

result_t<size_t> SlaveStreamAccessor::write(data::ConstDataSpan src)
{
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return getBus().write(this, src);
}

result_t<size_t> SlaveStreamAccessor::readableBytes(void)
{
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return getBus().readableBytes(this);
}

result_t<bool> SlaveStreamAccessor::transactionComplete(void)
{
    if (!isBound()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return getBus().transactionComplete(this);
}

result_t<size_t> SlaveStreamAccessor::serve(data::Source *src, data::Sink *dst, uint32_t timeout_ms)
{
    auto begin = beginTransaction(timeout_ms);
    if (!begin.has_value()) {
        return m5::stl::make_unexpected(begin.error());
    }

    size_t rx_total    = 0;  // bytes received into the Sink (write side)
    size_t tx_total    = 0;  // bytes queued from the Source (read side)
    error::error_t err = error::error_t::OK;
    bool failed        = false;

    // Stall escape (finite timeout only): treat timeout_ms as the max time to
    // block WITHOUT progress. If a bounded/null Sink (or lagging Source) makes no
    // progress for that long, stop holding the master -- drain the rest of the
    // write into `discard` so the RX_FULL stretch lifts and the master finishes,
    // then return TIMEOUT_ERROR. TIMEOUT_FOREVER keeps the zero-loss hold.
    const bool has_deadline   = (timeout_ms != types::TIMEOUT_FOREVER);
    uint32_t last_progress_ms = runtime::millis();
    bool escaping             = false;
    uint8_t discard[64];

    for (;;) {
        bool progressed = false;

        // RX: drain received bytes straight into the Sink's reserved span. A
        // zero-length reserve (Sink full/closed) just stops draining -- the ring
        // backs up and the backend stretches the master until space frees here.
        // Once escaping, drain into `discard` instead (works for a null rx too) so
        // the stretch lifts and the master can finish the abandoned transaction.
        if (dst != nullptr || escaping) {
            auto readable = readableBytes();
            if (!readable.has_value()) {
                err    = readable.error();
                failed = true;
                break;
            }
            if (readable.value() > 0) {
                if (escaping) {
                    const size_t want = std::min(readable.value(), sizeof(discard));
                    auto got          = read(data::DataSpan{discard, want});
                    if (!got.has_value()) {
                        err    = got.error();
                        failed = true;
                        break;
                    }
                    if (got.value() > 0) {
                        progressed = true;  // progress, but discarded (tail dropped)
                    }
                } else {
                    auto rsv = dst->reserve(readable.value());
                    if (!rsv.has_value()) {
                        err    = rsv.error();
                        failed = true;
                        break;
                    }
                    if (rsv.value().size > 0) {
                        const size_t want = std::min(rsv.value().size, readable.value());
                        auto got          = read(data::DataSpan{rsv.value().data, want});
                        if (!got.has_value()) {
                            err    = got.error();
                            failed = true;
                            break;
                        }
                        if (got.value() > 0) {
                            auto com = dst->commit(got.value());
                            if (!com.has_value()) {
                                err    = com.error();
                                failed = true;
                                break;
                            }
                            rx_total += got.value();
                            progressed = true;
                        }
                    } else if (dst->closed()) {
                        // The Sink reserved 0 AND reports closed: it is permanently done
                        // (a fixed MemorySink that filled, or a LimitedSink that hit its
                        // cap) while the master still has bytes to write. Escape NOW --
                        // without waiting out the stall deadline, and regardless of
                        // TIMEOUT_FOREVER -- so a closed Sink can never wedge the bus.
                        // (A full-but-open ring Sink keeps closed()==false and still uses
                        // the deadline below.) The master's remaining tail drains to
                        // `discard`; the escape returns TIMEOUT_ERROR like any escape
                        // (the write did not fully fit; rx_total bytes landed in the Sink).
                        escaping = true;
                    }
                }
            }
        }

        // TX: fill the reply from the Source -- only while nothing has been
        // received, so a write transaction never wastes effort composing a reply
        // the master will not read (mirrors the validated echo gate). write()
        // returns short when the TX ring is full; the master draining it opens
        // room on a later pass (the backend holds it under TX underrun stretch).
        // Suspended once escaping: the transaction is being abandoned.
        if (!escaping && src != nullptr && rx_total == 0 && !src->eof()) {
            auto peeked = src->peek(SIZE_MAX);
            if (!peeked.has_value()) {
                err    = peeked.error();
                failed = true;
                break;
            }
            if (peeked.value().size > 0) {
                auto wrote = write(data::ConstDataSpan{peeked.value().data, peeked.value().size});
                if (!wrote.has_value()) {
                    err    = wrote.error();
                    failed = true;
                    break;
                }
                if (wrote.value() > 0) {
                    auto adv = src->advance(wrote.value());
                    if (!adv.has_value()) {
                        err    = adv.error();
                        failed = true;
                        break;
                    }
                    tx_total += wrote.value();
                    progressed = true;
                }
            }
        }

        // Completion: only after a no-progress pass. The final write bytes arrive
        // with the STOP interrupt, becoming readable exactly when complete turns
        // true -- so drain everything before declaring the transaction done.
        if (progressed) {
            last_progress_ms = runtime::millis();
        } else {
            auto complete = transactionComplete();
            if (!complete.has_value()) {
                err    = complete.error();
                failed = true;
                break;
            }
            if (complete.value()) {
                // TOCTOU guard: the STOP ISR can land between this pass's
                // readableBytes()==0 (the no-progress verdict above) and this
                // complete check, delivering the write's final FIFO bytes into
                // the ring together with the flag. Breaking here would return
                // with those bytes unread (a silent short count -- seen on HW as
                // a 65-byte write surfacing as 64 at 400 kHz, no overflow, with
                // everything downstream desyncing). Re-check before declaring done.
                // (Only when a drain target exists: with a null Sink and no escape
                // the top of the loop cannot consume the tail anyway.)
                bool tail_pending = false;
                if (dst != nullptr || escaping) {
                    auto readable_after = readableBytes();
                    if (!readable_after.has_value()) {
                        err    = readable_after.error();
                        failed = true;
                        break;
                    }
                    tail_pending = readable_after.value() > 0;
                }
                if (!tail_pending) {
                    break;
                }
                // Tail present. Do NOT `continue` here: a full-but-open Sink
                // (reserve()==0, closed()==false) also lands on this branch, and
                // an unconditional retry would spin hot forever, unreachable by
                // the finite-timeout stall escape below. Fall THROUGH to the
                // shared stall/deadline/wait logic instead: an expired finite
                // deadline escapes to `discard` (preserving the documented stall
                // contract), otherwise the activity wait paces the retry and the
                // next pass drains the tail normally.
            }
            // Stall escape: a finite timeout_ms bounds how long we hold the master
            // with no progress (a full / null Sink). Past that deadline, abandon the
            // transaction -- flip to discard mode so the next passes drain the ring,
            // the backend lifts the stretch, and the master finishes; we then return
            // TIMEOUT_ERROR. TIMEOUT_FOREVER never sets has_deadline, so it holds.
            if (has_deadline && (runtime::millis() - last_progress_ms) >= timeout_ms) {
                if (escaping) {
                    // Second no-progress deadline WHILE escaping: the discard drain
                    // assumes the master finishes once the stretch lifts, but nothing
                    // arrived for another full deadline -- the master itself went
                    // inactive (died / aborted with no visible STOP). Waiting on
                    // transactionComplete() would hang forever; abandon outright.
                    // Bus-safe: no-progress means the ring is empty, so no RX_FULL
                    // hold is pending, and a TX-side hold self-heals via the
                    // responder task's stretch budget (fill-byte fallback).
                    break;
                }
                escaping = true;
                // Fresh deadline for the escape drain: without this reset the very
                // next no-progress pass would see the stale timer already expired
                // and abandon almost immediately -- the documented contract grants
                // the drain one more full deadline to reach the master's STOP.
                last_progress_ms = runtime::millis();
                continue;
            }
            // Event-driven wait: the backend wakes us on the next RX/TX/STOP ISR
            // (with a short safety timeout in case a wake is missed). This is what
            // keeps the consumer draining promptly enough that the RX ring is not
            // full at the master's STOP -- a fixed delay here lagged a fast master
            // and dropped the final FIFO load (the >64B-at-speed tail truncation).
            getBus().waitForActivity(this, 4);
        }
    }

    auto ended = endTransaction();
    if (failed) {
        return m5::stl::make_unexpected(err);
    }
    if (escaping) {
        // Transaction abandoned -- the Sink could not take the whole write (it hit a
        // finite stall deadline, or it reported closed() while full). The tail was
        // discarded (also counted by rxOverflowCount); the rx_total bytes that fit
        // already landed in the Sink. A closed() Sink takes this path immediately,
        // even under TIMEOUT_FOREVER, so it can never wedge the bus.
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    // Return the RECEIVED amount: bytes the master wrote into the Sink this
    // transaction. tx_total (bytes queued to the master's read) is tracked for
    // the TX gate logic above but is not part of the return contract; a serve()
    // is "received N" regardless of how large a reply it composed.
    (void)tx_total;
    return rx_total;
}

result_t<void> SlaveRegMapAccessor::serve(uint32_t timeout_ms)
{
    if (_isr_bound) {
        // Fast path: the backend's own ISR already ingested writes and composed
        // reads directly against reg_file/onRead/onWrite (see
        // ISlaveBus::bindIsrRegMap). Nothing here needs beginExchange/ingest/
        // composeReply -- just open the transaction and wait for the backend to
        // report it complete (the master's STOP).
        auto begin = _stream.beginTransaction(timeout_ms);
        if (!begin.has_value()) {
            return begin;
        }

        // No-progress deadline, the fast-path counterpart of the task-context
        // stall escape below: the ISR is what progresses this transaction, so an
        // expired finite deadline can only mean the master went inactive without
        // a STOP. There is nothing to drain/discard here (the ISR never handed
        // anything to this task) -- just abandon the wait and close the
        // transaction, like the task-context path's stall abandon. Matches the
        // documented contract (an in-transaction NO-PROGRESS stall, not a
        // wall-clock total): every CONFIRMED waitForActivity() wake (an RX/TX/
        // STOP ISR pass) refreshes the deadline, so a master that keeps the ISR
        // busy past timeout_ms is not mistaken for a stalled one.
        const bool has_deadline   = (timeout_ms != types::TIMEOUT_FOREVER);
        uint32_t last_progress_ms = runtime::millis();
        bool stalled              = false;
        for (;;) {
            auto complete = _stream.transactionComplete();
            if (!complete.has_value()) {
                (void)_stream.endTransaction();
                return m5::stl::make_unexpected(complete.error());
            }
            if (complete.value()) {
                break;
            }
            if (has_deadline && (runtime::millis() - last_progress_ms) >= timeout_ms) {
                stalled = true;
                break;
            }
            auto activity = _stream.waitForActivity(4);
            if (activity.has_value() && activity.value()) {
                last_progress_ms = runtime::millis();
            }
        }
        auto ended = _stream.endTransaction();
        if (stalled) {
            return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
        }
        return ended;
    }

    auto begin = _stream.beginTransaction(timeout_ms);
    if (!begin.has_value()) {
        return begin;
    }

    beginExchange();

    error::error_t err = error::error_t::OK;
    bool failed        = false;

    // Stall escape (finite timeout only), the regmap counterpart of
    // SlaveStreamAccessor::serve()'s no-progress deadline (same documented
    // contract; not a wall-clock total). The regmap consumer always progresses
    // while the wire moves -- RX drains straight into the register file and the
    // reply pump is bounded by the tx ring -- so an expired deadline can only
    // mean the MASTER went inactive mid-transaction (died / aborted with no
    // visible STOP), never local back-pressure. There is thus no discard-drain
    // phase here (the stream escape's job): abandon the exchange outright.
    // Register writes ingested before the stall stay applied, like a real
    // register device cut off mid-write. Bus-safe for the same reason as the
    // stream abandon: an empty ring means no RX_FULL hold is pending, and a
    // TX-side hold self-heals via the responder task's stretch budget.
    const bool has_deadline   = (timeout_ms != types::TIMEOUT_FOREVER);
    uint32_t last_progress_ms = runtime::millis();
    bool stalled              = false;

    // 1. Non-blocking read of whatever has arrived: the register byte plus
    //    any early write data. A pure read (SPLIT's 2nd transaction) has
    //    nothing here, so gate on readableBytes() -- a blocking read would
    //    stall the whole timeout.
    uint8_t buf[kReadChunkBytes];
    size_t n0     = 0;
    auto readable = _stream.readableBytes();
    if (!readable.has_value()) {
        err    = readable.error();
        failed = true;
    } else if (readable.value() > 0) {
        auto got = _stream.read(data::DataSpan{buf, sizeof(buf)});
        if (!got.has_value()) {
            err    = got.error();
            failed = true;
        } else {
            n0 = got.value();
        }
    }

    if (!failed) {
        // The first byte of the exchange sets the pointer; feed it BEFORE
        // composing so the reply window reflects the pre-write register state
        // (matches the validated reference). Data bytes are applied after.
        if (n0 >= 1) {
            ingest(data::ConstDataSpan{buf, 1});
        }

        // 2. Compose the read reply from the register pointer and queue it.
        //    Writing the first chunk lets the backend release a held read
        //    stretch promptly; for a write-only transaction the master never
        //    reads it (harmless, it expires at STOP). The reply STREAMS: when
        //    the master reads past a chunk the tx ring drains, the pump in the
        //    loop below composes the next chunk from the advancing offset
        //    (8-bit wrap), and the backend's TX_EMPTY refill keeps the read
        //    going -- a single read is not capped at one window (matches the
        //    ESP32_I2C_slave_example reference, which refills from the
        //    register file in-ISR). Composed-but-unaccepted bytes are retried
        //    verbatim, never re-composed, so onRead fires at most once per
        //    streamed byte (at most one chunk ahead of the wire).
        uint8_t tx[kReplyWindowBytes];
        size_t tx_have  = composeReply(data::DataSpan{tx, sizeof(tx)});
        size_t tx_sent  = 0;
        size_t resp_off = tx_have;  // next register offset to compose
        {
            auto wrote = _stream.write(data::ConstDataSpan{tx, tx_have});
            if (!wrote.has_value()) {
                err    = wrote.error();
                failed = true;
            } else {
                tx_sent = wrote.value();
            }
        }

        // Apply the remaining bytes of the initial read as write data.
        if (!failed && n0 >= 2) {
            ingest(data::ConstDataSpan{buf + 1, n0 - 1});
        }

        // 3. Drain the rest of the write phase until the master STOPs, and keep
        //    the read reply pumped. Drain everything available BEFORE checking
        //    complete: the final write bytes are pushed into the rx queue by
        //    the STOP interrupt, so they become readable at the same moment
        //    transactionComplete() turns true. Checking complete first would
        //    leave that tail unread.
        while (!failed) {
            auto more = _stream.readableBytes();
            if (!more.has_value()) {
                err    = more.error();
                failed = true;
                break;
            }
            if (more.value() > 0) {
                auto got = _stream.read(data::DataSpan{buf, sizeof(buf)});
                if (!got.has_value()) {
                    err    = got.error();
                    failed = true;
                    break;
                }
                if (got.value() > 0) {
                    ingest(data::ConstDataSpan{buf, got.value()});
                    last_progress_ms = runtime::millis();
                    continue;
                }
            }
            auto complete = _stream.transactionComplete();
            if (!complete.has_value()) {
                err    = complete.error();
                failed = true;
                break;
            }
            if (complete.value()) {
                // TOCTOU guard (same race as SlaveStreamAccessor::serve): the STOP
                // ISR can deliver the final write bytes into the ring between the
                // readableBytes()==0 above and this complete check; breaking now
                // would drop that tail. Re-check and drain it first.
                auto readable_after = _stream.readableBytes();
                if (!readable_after.has_value()) {
                    err    = readable_after.error();
                    failed = true;
                    break;
                }
                if (readable_after.value() > 0) {
                    continue;
                }
                break;
            }
            // Reply pump: top the tx ring back up while it has room (the ring
            // frees as the master clocks the reply out). write() is
            // partial-accept, so this composes at most one chunk beyond what
            // the ring can hold and then stands down until the next wake.
            while (!failed) {
                if (tx_sent == tx_have) {
                    tx_have = composeReply(data::DataSpan{tx, sizeof(tx)}, resp_off);
                    tx_sent = 0;
                    resp_off += tx_have;
                }
                auto wrote = _stream.write(data::ConstDataSpan{tx + tx_sent, tx_have - tx_sent});
                if (!wrote.has_value()) {
                    err    = wrote.error();
                    failed = true;
                    break;
                }
                if (wrote.value() > 0) {
                    // The ring accepted bytes, which means the master clocked some
                    // of the reply out since the last pass -- wire progress.
                    last_progress_ms = runtime::millis();
                }
                tx_sent += wrote.value();
                if (tx_sent < tx_have) {
                    break;  // ring full for now; retry after the next activity
                }
            }
            if (failed) {
                break;
            }
            // No-progress deadline (see the header comment above): the master went
            // inactive mid-transaction. Abandon the exchange and report the stall.
            if (has_deadline && (runtime::millis() - last_progress_ms) >= timeout_ms) {
                stalled = true;
                break;
            }
            // Event-driven drain: wake on the next RX/STOP ISR (short safety
            // timeout) rather than a fixed poll. With the espidf RX ring sized to
            // the HW FIFO, a fixed delay here let a mid-size register write
            // (ring < k <= ring+FIFO, which does not trip back-pressure) finish
            // before this loop drained it, dropping the ring's FIFO tail at STOP.
            _stream.waitForActivity(4);
        }
    }

    // Always close the transaction (even on a mid-exchange error) so the bus
    // is not left open. Report the first hard error if there was one.
    auto ended = _stream.endTransaction();
    if (failed) {
        return m5::stl::make_unexpected(err);
    }
    if (stalled) {
        // Report the stall even if the close also failed: like `failed` above,
        // the policy is "first abnormality wins" (the stall was detected before
        // the close was attempted). Matches SlaveStreamAccessor::serve(), whose
        // escape return likewise takes priority over the endTransaction result.
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    return ended;
}

void SlaveRegMapAccessor::beginExchange(void)
{
    _pointer_set  = false;
    _write_offset = 0;
}

void SlaveRegMapAccessor::ingest(data::ConstDataSpan src)
{
    if (src.data == nullptr) {
        return;
    }
    for (size_t i = 0; i < src.size; ++i) {
        const uint8_t b = src.data[i];
        if (!_pointer_set) {
            _pointer     = b;
            _pointer_set = true;
        } else {
            writeByte(static_cast<uint8_t>(_pointer + _write_offset), b);
            ++_write_offset;
        }
    }
}

size_t SlaveRegMapAccessor::composeReply(data::DataSpan dst, size_t offset)
{
    if (dst.data == nullptr) {
        return 0;
    }
    for (size_t i = 0; i < dst.size; ++i) {
        dst.data[i] = readByte(static_cast<uint8_t>(_pointer + offset + i));
    }
    return dst.size;
}

SlaveStreamAccessor &SlaveRegMapAccessor::stream(void)
{
    return _stream;
}

uint8_t SlaveRegMapAccessor::readByte(uint8_t reg) const
{
    return regMapReadByte(_reg_file, reg, _on_read, _on_read_ctx);
}

void SlaveRegMapAccessor::writeByte(uint8_t reg, uint8_t value)
{
    regMapWriteByte(_reg_file, reg, value, _on_write, _on_write_ctx);
}

result_t<void> ScopedSlaveServiceRegistration::registerTo(service::ServiceRunner &runner, ISlaveBus &driver)
{
    release();
    auto *svc = driver.service();
    if (svc == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (!runner.add(*svc)) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    _runner  = &runner;
    _service = svc;
    return {};
}

void ScopedSlaveServiceRegistration::release()
{
    if (_runner != nullptr && _service != nullptr) {
        (void)_runner->remove(*_service);
    }
    _runner  = nullptr;
    _service = nullptr;
}

bool ScopedSlaveServiceRegistration::registered() const
{
    return _runner != nullptr && _service != nullptr;
}

result_t<void> SlaveBus_software::init(SlaveLineDriver &lines, const SlaveBusConfig &config)
{
    if (config.address_is_10bit || config.address > 0x7F) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _lines  = &lines;
    _config = config;
    resetProtocol();
    return {};
}

result_t<void> SlaveBus_software::init(const SlaveBusConfig &cfg)
{
    (void)cfg;
    return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
}

result_t<void> SlaveBus_software::release(void)
{
    if (_lines != nullptr) {
        _lines->pullSdaLow(false);
        _lines->pullSclLow(false);
    }
    _lines = nullptr;
    _open  = nullptr;
    return {};
}

service::IService *SlaveBus_software::service()
{
    return this;
}

result_t<void> SlaveBus_software::beginTransaction(bus::IAccessor *owner, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (owner == nullptr || _open != nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    Transaction *txn = oldestOpenableTransaction();
    if (txn == nullptr) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    txn->opened = true;
    _open       = txn;
    _open_owner = owner;
    resumeStretchIfReady();
    return {};
}

result_t<void> SlaveBus_software::endTransaction(bus::IAccessor *owner)
{
    if (owner == nullptr || _open == nullptr || _open_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    discardTransaction(*_open);
    _open       = nullptr;
    _open_owner = nullptr;
    return {};
}

result_t<size_t> SlaveBus_software::read(bus::IAccessor *owner, data::DataSpan dst)
{
    if (!isOpenOwner(owner) || (dst.data == nullptr && dst.size != 0)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const size_t readable = readableBytesOf(*_open);
    const size_t take     = std::min(dst.size, readable);
    if (take > 0) {
        // rx[] is a power-of-two ring; the readable span may wrap past the end.
        const size_t start = _open->rx_read & (kRxCapacity - 1);
        const size_t first = std::min(take, kRxCapacity - start);
        ::memcpy(dst.data, _open->rx + start, first);
        if (take > first) {
            ::memcpy(static_cast<uint8_t *>(dst.data) + first, _open->rx, take - first);
        }
        _open->rx_read += take;
    }
    return take;
}

result_t<size_t> SlaveBus_software::write(bus::IAccessor *owner, data::ConstDataSpan src)
{
    if (!isOpenOwner(owner) || (src.data == nullptr && src.size != 0)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    // tx[] is a power-of-two ring: free space is bounded by the UNSENT occupancy
    // (tx_size - tx_read), not the monotonic tx_size. As the master consumes the
    // reply (advancing tx_read) room opens here, so the app can stream a reply far
    // larger than kTxCapacity across one read by writing more as space frees up.
    const size_t occupancy = _open->tx_size - _open->tx_read;
    const size_t free      = (occupancy < kTxCapacity) ? (kTxCapacity - occupancy) : 0;
    const size_t take      = std::min(src.size, free);
    if (take > 0) {
        // The write span may wrap past the end of the ring (up to 2 copies).
        const size_t start = _open->tx_size & (kTxCapacity - 1);
        const size_t first = std::min(take, kTxCapacity - start);
        ::memcpy(_open->tx + start, src.data, first);
        if (take > first) {
            ::memcpy(_open->tx, static_cast<const uint8_t *>(src.data) + first, take - first);
        }
        _open->tx_size += take;
    }
    resumeStretchIfReady();
    return take;
}

result_t<size_t> SlaveBus_software::readableBytes(bus::IAccessor *owner)
{
    if (!isOpenOwner(owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return readableBytesOf(*_open);
}

result_t<bool> SlaveBus_software::transactionComplete(bus::IAccessor *owner)
{
    if (!isOpenOwner(owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return _open->complete;
}

void SlaveBus_software::setMaxAckedWriteBytes(size_t count)
{
    _max_acked_write_bytes = count;
}

size_t SlaveBus_software::maxAckedWriteBytes() const
{
    return _max_acked_write_bytes;
}

size_t SlaveBus_software::masterAckCount() const
{
    return _master_ack_count;
}

bool SlaveBus_software::masterAckAt(size_t index) const
{
    return (index < _master_ack_count) ? _master_acks[index] : false;
}

size_t SlaveBus_software::stopCount() const
{
    return _stop_count;
}

size_t SlaveBus_software::rxOverflowCount() const
{
    return _rx_overflow_count;
}

service::ServicePoll SlaveBus_software::serviceImpl(const service::ServiceContext &ctx)
{
    if (_lines == nullptr) {
        return service::ServiceResult::Idle;
    }
    // Private virtual timeline for the stretch budget; gap-drop only makes
    // the stretch last longer (slave keeps waiting), which is the safe side.
    _svc_now += ctx.elapsed;

    const bool scl = _lines->readScl();
    const bool sda = _lines->readSda();

    auto result = service::ServiceResult::Idle;
    if (_prev_scl && _prev_sda && !sda) {
        startCondition();
        result = service::ServiceResult::Progress;
    }

    if (_state == State::WaitTx) {
        if (txAvailableForCurrent() || stretchExpired(_svc_now)) {
            _lines->pullSclLow(false);
            _state     = State::Transmit;
            _bit_count = 0;
            driveTxBit();
            result = service::ServiceResult::Progress;
        }
    }

    if (!_prev_scl && scl && _state == State::Receive) {
        _byte = static_cast<uint8_t>((_byte << 1) | (sda ? 1 : 0));
        ++_bit_count;
        if (_bit_count == 8) {
            if (_is_address) {
                const uint8_t addr = _byte >> 1;
                _matched           = (addr == _config.address);
                _read_phase        = (_byte & 1) != 0;
                if (_matched && _current == nullptr) {
                    _current = allocateTransaction();
                }
                _drive_ack = _matched;
            } else {
                const bool stored = storeReceivedByte(_byte);
                _drive_ack        = _matched && !_read_phase && stored && (currentRxSize() <= _max_acked_write_bytes);
            }
            _state = State::AckSetup;
        }
        result = service::ServiceResult::Progress;
    }

    if (!_prev_scl && scl && _state == State::ReadMasterAck) {
        _master_ack = !sda;
        storeMasterAck(_master_ack);
        result = service::ServiceResult::Progress;
    }

    if (_prev_scl && !scl && _state == State::Transmit) {
        if (++_bit_count < 8) {
            driveTxBit();
        } else {
            _lines->pullSdaLow(false);
            _state = State::ReadMasterAck;
        }
        result = service::ServiceResult::Progress;
    } else if (_prev_scl && !scl && _state == State::ReadMasterAck) {
        if (_master_ack) {
            _bit_count = 0;
            if (_config.tx_underrun == TxUnderrun::Stretch && !txAvailableForCurrent()) {
                beginStretch(_svc_now);
            } else {
                _state = State::Transmit;
                driveTxBit();
            }
        } else {
            _state = State::Ignore;
            _lines->pullSdaLow(false);
        }
        result = service::ServiceResult::Progress;
    } else if (_prev_scl && !scl && _state == State::AckSetup) {
        _lines->pullSdaLow(_drive_ack);
        _state = State::Ack;
        result = service::ServiceResult::Progress;
    } else if (_prev_scl && !scl && _state == State::Ack) {
        _lines->pullSdaLow(false);
        _byte      = 0;
        _bit_count = 0;
        if (_matched && _read_phase) {
            if (_config.tx_underrun == TxUnderrun::Stretch && !txAvailableForCurrent()) {
                beginStretch(_svc_now);
            } else {
                _state = State::Transmit;
                driveTxBit();
            }
        } else {
            _is_address = false;
            _state      = _matched && !_read_phase ? State::Receive : State::Ignore;
        }
        result = service::ServiceResult::Progress;
    }

    if (_prev_scl && !_prev_sda && sda) {
        stopCondition();
        result = service::ServiceResult::Progress;
    }

    _prev_scl = scl;
    _prev_sda = sda;
    return result;
}

void SlaveBus_software::resetProtocol()
{
    _state             = State::Idle;
    _prev_scl          = true;
    _prev_sda          = true;
    _is_address        = true;
    _matched           = false;
    _read_phase        = false;
    _drive_ack         = false;
    _master_ack        = false;
    _byte              = 0;
    _bit_count         = 0;
    _current           = nullptr;
    _open              = nullptr;
    _open_owner        = nullptr;
    _stop_count        = 0;
    _rx_overflow_count = 0;
    _master_ack_count  = 0;
    _next_seq          = 1;
    _stretching        = false;
    for (auto &txn : _transactions) {
        txn = Transaction{};
    }
    for (size_t i = 0; i < kMaxObservedMasterAcks; ++i) {
        _master_acks[i] = false;
    }
    if (_lines != nullptr) {
        _lines->pullSdaLow(false);
        _lines->pullSclLow(false);
    }
}

void SlaveBus_software::startCondition()
{
    _state      = State::Receive;
    _byte       = 0;
    _bit_count  = 0;
    _is_address = true;
    _matched    = false;
    _read_phase = false;
    _drive_ack  = false;
    _lines->pullSdaLow(false);
}

void SlaveBus_software::stopCondition()
{
    if (_current != nullptr) {
        _current->complete = true;
        _current->tx_size  = _current->tx_read;
        _current           = nullptr;
    }
    _state = State::Idle;
    ++_stop_count;
    _stretching = false;
    _lines->pullSdaLow(false);
    _lines->pullSclLow(false);
}

SlaveBus_software::Transaction *SlaveBus_software::allocateTransaction()
{
    for (auto &txn : _transactions) {
        if (!txn.in_use) {
            txn        = Transaction{};
            txn.in_use = true;
            txn.seq    = _next_seq++;
            return &txn;
        }
    }
    Transaction *oldest = nullptr;
    for (auto &txn : _transactions) {
        if (&txn == _open) {
            continue;
        }
        if (oldest == nullptr || txn.seq < oldest->seq) {
            oldest = &txn;
        }
    }
    if (oldest != nullptr) {
        *oldest        = Transaction{};
        oldest->in_use = true;
        oldest->seq    = _next_seq++;
    }
    return oldest;
}

SlaveBus_software::Transaction *SlaveBus_software::oldestOpenableTransaction()
{
    Transaction *best = nullptr;
    for (auto &txn : _transactions) {
        if (!txn.in_use || txn.opened) {
            continue;
        }
        if (best == nullptr || txn.seq < best->seq) {
            best = &txn;
        }
    }
    return best;
}

void SlaveBus_software::discardTransaction(Transaction &txn)
{
    if (&txn == _current) {
        _current = nullptr;
    }
    txn = Transaction{};
}

bool SlaveBus_software::isOpenOwner(bus::IAccessor *owner) const
{
    return owner != nullptr && _open != nullptr && _open_owner == owner;
}

size_t SlaveBus_software::readableBytesOf(const Transaction &txn)
{
    return (txn.rx_read < txn.rx_size) ? (txn.rx_size - txn.rx_read) : 0;
}

size_t SlaveBus_software::currentRxSize() const
{
    return (_current != nullptr) ? _current->rx_size : 0;
}

bool SlaveBus_software::storeReceivedByte(uint8_t value)
{
    if (_current == nullptr) {
        return false;
    }
    // rx[] is a power-of-two ring: occupancy is (rx_size - rx_read), not the
    // monotonic rx_size. As long as the consumer drains via read(), a single
    // transaction can receive far more than kRxCapacity bytes total. The cap
    // bounds only the UNREAD backlog -- if the consumer falls behind and the
    // backlog reaches kRxCapacity, writing here would overwrite the oldest
    // unread byte (silent corruption), so drop instead and surface it via
    // rxOverflowCount().
    if ((_current->rx_size - _current->rx_read) >= kRxCapacity) {
        ++_rx_overflow_count;
        return false;
    }
    _current->rx[(_current->rx_size++) & (kRxCapacity - 1)] = value;
    return true;
}

void SlaveBus_software::storeMasterAck(bool ack)
{
    if (_master_ack_count < kMaxObservedMasterAcks) {
        _master_acks[_master_ack_count++] = ack;
    }
}

bool SlaveBus_software::txAvailableForCurrent() const
{
    return _current != nullptr && _current->opened && _current->tx_read < _current->tx_size;
}

uint8_t SlaveBus_software::nextTxByte()
{
    if (txAvailableForCurrent()) {
        // tx[] is a power-of-two ring; consume at the wrapped read cursor.
        return _current->tx[(_current->tx_read++) & (kTxCapacity - 1)];
    }
    return _config.tx_fill_byte;
}

void SlaveBus_software::driveTxBit()
{
    if (_lines == nullptr) {
        return;
    }
    if (_bit_count == 0) {
        _tx_byte = nextTxByte();
    }
    const bool bit = (_tx_byte & (0x80u >> _bit_count)) != 0;
    _lines->pullSdaLow(!bit);
}

void SlaveBus_software::beginStretch(service::fast_tick_t now_tick)
{
    _state         = State::WaitTx;
    _stretching    = true;
    _stretch_start = now_tick;
    // stretch_timeout_ms is a duration in milliseconds; fastTick() ticks are
    // CPU cycles on ESP32 (not microseconds), so convert through the
    // frequency-aware helper instead of assuming a 1 MHz tick rate.
    const uint64_t nsec    = static_cast<uint64_t>(_config.stretch_timeout_ms) * uint64_t{1000000};
    const uint32_t clamped = nsec > static_cast<uint64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(nsec);
    _stretch_budget =
        service::nsecToFastTickCeil(static_cast<service::tick_nsec_t>(clamped), service::fastTickFrequencyHz());
    _lines->pullSclLow(true);
    _lines->pullSdaLow(false);
}

bool SlaveBus_software::stretchExpired(service::fast_tick_t now_tick) const
{
    return _stretching && service::elapsedTicks(now_tick, _stretch_start) >= _stretch_budget;
}

void SlaveBus_software::resumeStretchIfReady()
{
    if (_stretching && txAvailableForCurrent() && _lines != nullptr) {
        _stretching = false;
        _lines->pullSclLow(false);
    }
}

}  // namespace m5::hal::v2::i2c

#endif  // M5_HAL_HAL_V2_I2C_SLAVE_INL_
