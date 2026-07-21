// SPDX-License-Identifier: MIT
#ifndef M5_HAL_PDM_PDM_INL_
#define M5_HAL_PDM_PDM_INL_

#include "pdm.hpp"

namespace m5::hal::v2::pdm {

RxAccessor::RxAccessor(IBus& bus, const AccessConfig& access_config)
    : bus::IAccessor{bus}, _context{makeOperationContext(access_config)}
{
}

RxAccessor::RxAccessor(std::shared_ptr<IBus> bus, const AccessConfig& access_config)
    : bus::IAccessor{std::move(bus)}, _context{makeOperationContext(access_config)}
{
}

IBus& RxAccessor::getBus() const
{
    return static_cast<IBus&>(bus::IAccessor::getBus());
}

result_t<void> RxAccessor::setConfig(const AccessConfig& cfg)
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _context.config = cfg;
    return {};
}

result_t<void> RxAccessor::beginAccess(uint32_t timeout_ms)
{
    return _beginOperationAccess(_context, timeout_ms, bus::OperationMode::Rx,
                                 [&](auto& context) { return getBus().beginOperation(context); });
}

result_t<void> RxAccessor::endAccess(uint32_t timeout_ms)
{
    return _endOperationAccess(_context, timeout_ms, [&](auto& context) { return getBus().endOperation(context); });
}

result_t<bus::TransferStatus> RxAccessor::getLastTransferStatus() const
{
    if (_last_transfer_status.transfer_id == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return _last_transfer_status;
}

void RxAccessor::recordTransferResult(const result_t<size_t>& result, size_t requested)
{
    if (++_next_transfer_id == 0) {
        ++_next_transfer_id;
    }
    _last_transfer_status             = {};
    _last_transfer_status.transfer_id = _next_transfer_id;
    if (!result.has_value()) {
        _last_transfer_status.error      = result.error();
        _last_transfer_status.completion = bus::CompletionLevel::Aborted;
        return;
    }
    _last_transfer_status.totals.rx = result.value();
    _last_transfer_status.completion =
        result.value() == requested ? bus::CompletionLevel::Complete : bus::CompletionLevel::Partial;
}

result_t<size_t> RxAccessor::read(data::DataSpan dst_bytes)
{
    if (dst_bytes.size != 0 && dst_bytes.data == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    data::MemorySink sink{dst_bytes};
    return read(sink, dst_bytes.size);
}

result_t<size_t> RxAccessor::read(data::Sink& dst, size_t len)
{
    const bool borrowed = inAccess();
    if (!borrowed) {
        auto begun = beginAccess();
        if (!begun.has_value()) {
            return m5::stl::make_unexpected(begun.error());
        }
    }
    auto read_result = readInCurrentAccess(dst, len);
    result_t<void> ended{};
    if (!borrowed) {
        ended = endAccess();
    }
    if (!read_result.has_value()) {
        return m5::stl::make_unexpected(read_result.error());
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    return read_result.value();
}

result_t<size_t> RxAccessor::readInCurrentAccess(data::Sink& dst, size_t len)
{
    if (!inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto read_result = getBus().read(_context, &dst, len);
    recordTransferResult(read_result, len);
    return read_result;
}

result_t<size_t> RxAccessor::read(uint8_t* dst, size_t len)
{
    return read(data::DataSpan{dst, len});
}

result_t<size_t> RxAccessor::readableBytes()
{
    const bool borrowed = inAccess();
    if (!borrowed) {
        auto begun = beginAccess();
        if (!begun.has_value()) {
            return m5::stl::make_unexpected(begun.error());
        }
    }
    auto readable = getBus().readableBytes(_context);
    result_t<void> ended{};
    if (!borrowed) {
        ended = endAccess();
    }
    if (!readable.has_value()) {
        return m5::stl::make_unexpected(readable.error());
    }
    if (!ended.has_value()) {
        return m5::stl::make_unexpected(ended.error());
    }
    return readable.value();
}

result_t<void> IBus::beginOperation(bus::OperationContext<AccessConfig>& context)
{
    auto registered = _operation_slot.registerContext(context, this, _lock_owner);
    if (!registered.has_value()) {
        return registered;
    }
    auto begun = beginOperationBackend(context);
    if (!begun.has_value()) {
        _operation_slot.invalidate(context);
    }
    return begun;
}

result_t<void> IBus::endOperation(bus::OperationContext<AccessConfig>& context)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        if (_operation_slot.registered(context, this)) {
            if (_operation_slot.restoreRegisteredRuntime(context, this, _lock_owner)) {
                (void)endOperationBackend(context);
            }
            _operation_slot.invalidate(context);
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto ended = endOperationBackend(context);
    _operation_slot.invalidate(context);
    return ended;
}

result_t<size_t> IBus::read(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return readBackend(context, dst, len);
}

result_t<size_t> IBus::readableBytes(bus::OperationContext<AccessConfig>& context)
{
    if (!_operation_slot.valid(context, this, _lock_owner)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return readableBytesBackend(context);
}

result_t<void> IBus::beginOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    return {};
}

result_t<void> IBus::endOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    return {};
}

result_t<size_t> IBus::readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len)
{
    (void)context;
    (void)dst;
    (void)len;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

result_t<size_t> IBus::readableBytesBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

}  // namespace m5::hal::v2::pdm

#endif  // M5_HAL_PDM_PDM_INL_
