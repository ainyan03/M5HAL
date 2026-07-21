// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_UART_UART_INL

#include "uart.hpp"

#if M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#include <algorithm>
#include <cstdlib>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <new>
#include <string.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#if defined(__APPLE__)
// IOSSIOSPEED: set an arbitrary (non-B*) baud on macOS, whose termios only has
// B* constants up to B230400. Lets the variant reach 0.5/1/2/3 Mbaud etc.
#include <IOKit/serial/ioss.h>
#endif

#include "../../../../../hal/v2/diag.hpp"
#include "../../../../../hal/v2/resource_domain.hpp"

namespace m5::hal::v2::uart {

namespace {
namespace impl_posix {

// Map a numeric baud rate to a termios speed_t, returning false if this libc
// has no B* constant for it. Rates above the POSIX-standard set are #ifdef
// -guarded so the file compiles everywhere. Linux glibc/musl define
// B460800..B4000000, so those high rates take this path there. macOS termios
// stops at B230400; higher rates return false here and applyConfig() sets them
// via the IOSSIOSPEED ioctl instead (validated to 3 Mbaud against real hardware
// in LovyanAPI). Single source of truth for the table; Bus_posix::baudToSpeed() is a
// thin public wrapper over it for unit tests.
bool baudConstant(uint32_t baud, speed_t& out)
{
    switch (baud) {
        case 1200:
            out = B1200;
            return true;
        case 2400:
            out = B2400;
            return true;
        case 4800:
            out = B4800;
            return true;
        case 9600:
            out = B9600;
            return true;
        case 19200:
            out = B19200;
            return true;
        case 38400:
            out = B38400;
            return true;
        case 57600:
            out = B57600;
            return true;
        case 115200:
            out = B115200;
            return true;
        case 230400:
            out = B230400;
            return true;
#ifdef B460800
        case 460800:
            out = B460800;
            return true;
#endif
#ifdef B500000
        case 500000:
            out = B500000;
            return true;
#endif
#ifdef B576000
        case 576000:
            out = B576000;
            return true;
#endif
#ifdef B921600
        case 921600:
            out = B921600;
            return true;
#endif
#ifdef B1000000
        case 1000000:
            out = B1000000;
            return true;
#endif
#ifdef B1152000
        case 1152000:
            out = B1152000;
            return true;
#endif
#ifdef B1500000
        case 1500000:
            out = B1500000;
            return true;
#endif
#ifdef B2000000
        case 2000000:
            out = B2000000;
            return true;
#endif
#ifdef B2500000
        case 2500000:
            out = B2500000;
            return true;
#endif
#ifdef B3000000
        case 3000000:
            out = B3000000;
            return true;
#endif
#ifdef B3500000
        case 3500000:
            out = B3500000;
            return true;
#endif
#ifdef B4000000
        case 4000000:
            out = B4000000;
            return true;
#endif
        default:
            return false;
    }
}

bool sameConfig(const uart::AccessConfig& lhs, const uart::AccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

// Block until `fd` is readable / writable for up to `timeout_ms`.
// Returns >0 when ready, 0 on timeout, <0 on error.
int waitFd(int fd, bool for_write, uint32_t timeout_ms)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv;
    tv.tv_sec  = static_cast<time_t>(timeout_ms / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000) * 1000);
    int r;
    do {
        r = ::select(fd + 1, for_write ? nullptr : &fds, for_write ? &fds : nullptr, nullptr, &tv);
    } while (r < 0 && errno == EINTR);
    return r;
}

error::error_t posixIOError()
{
    return error::error_t::IO_ERROR;
}

constexpr uint16_t kNativeProvider = 0x0501;

bool portableConfigValid(const IBusConfig& cfg)
{
    return cfg.pin_tx == -1 && cfg.pin_rx == -1 && cfg.pin_rts == -1 && cfg.pin_cts == -1 &&
           cfg.rx_buffer_size <= UINT32_MAX && cfg.tx_buffer_size <= UINT32_MAX;
}

bus::BindingDescriptor makeBinding(const IBusConfig& cfg, bus::Ownership ownership, bus::NativeToken token,
                                   const NativeOptions& options)
{
    bus::BindingDescriptor binding;
    binding.provider         = kNativeProvider;
    binding.ownership        = ownership;
    binding.native_kind      = bus::NativeBindingKind::Native;
    binding.native           = token;
    binding.config_primary   = static_cast<uint32_t>(cfg.rx_buffer_size);
    binding.config_secondary = static_cast<uint32_t>(cfg.tx_buffer_size);
    binding.native_options   = static_cast<uint32_t>(options.tx_coalesce_bytes);
    return binding;
}

result_t<bus::NativeIdentity> identityForFd(int fd)
{
    struct stat st {};
    if (fd < 0 || ::fstat(fd, &st) != 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return bus::NativeIdentity::make(
        bus::NativeIdentityKind::PosixDevice,
        {static_cast<uint64_t>(st.st_dev), static_cast<uint64_t>(st.st_ino), static_cast<uint64_t>(st.st_rdev)});
}

int duplicateFd(int fd)
{
#ifdef F_DUPFD_CLOEXEC
    int duplicate = ::fcntl(fd, F_DUPFD_CLOEXEC, 0);
    if (duplicate >= 0 || errno != EINVAL) {
        return duplicate;
    }
#endif
    return ::dup(fd);
}

struct PendingNative {
    int fd                                                                               = -1;
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>* interner = nullptr;
    bus::NativeToken token{};

    ~PendingNative()
    {
        if (fd >= 0) {
            ::close(fd);
        }
        if (interner != nullptr && token.valid()) {
            (void)interner->release(token);
        }
    }

    void dismiss()
    {
        fd       = -1;
        interner = nullptr;
        token    = {};
    }
};

result_t<std::shared_ptr<IBus>> acquireBoundFd(bus::IHalBackend& hal_backend, const IBusConfig& cfg, int owned_fd,
                                               bus::Ownership ownership, const NativeOptions& options)
{
    PendingNative pending;
    pending.fd = owned_fd;
    if (!portableConfigValid(cfg) || options.tx_coalesce_bytes > NativeOptions::kMaxTxCoalesceBytes) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const auto* domain = hal_backend.localResourceDomain();
    if (domain == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    auto identity = identityForFd(owned_fd);
    if (!identity.has_value()) {
        return m5::stl::make_unexpected(identity.error());
    }
    auto& interner = domain->nativeInterner();
    auto token     = interner.intern(identity.value());
    if (!token.has_value()) {
        return m5::stl::make_unexpected(token.error());
    }
    pending.interner = &interner;
    pending.token    = token.value();

    auto key = bus::ResourceKey::makeToken(types::bus_kind_t::UART, bus::ResourceTag::Native, token.value(),
                                           static_cast<uint32_t>(bus::NativeIdentityKind::PosixDevice));
    if (!key.has_value()) {
        return m5::stl::make_unexpected(key.error());
    }
    const auto binding = makeBinding(cfg, ownership, token.value(), options);
    auto acquired      = hal_backend.busRegistry().acquireOrFind(
        key.value(), binding,
        [&cfg](const std::shared_ptr<bus::IBus>& existing) -> result_t<void> {
            if (!BusTraits::configCompatible(static_cast<const IBusConfig&>(existing->getConfig()), cfg)) {
                return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
            }
            return {};
        },
        [&]() -> result_t<std::shared_ptr<bus::IBus>> {
            std::unique_ptr<Bus_posix> concrete{new (std::nothrow) Bus_posix()};
            if (!concrete) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            concrete->bindLocalResources(hal_backend.localResources());
            auto adopted_native =
                concrete->adoptOwnedNative(pending.fd, cfg, options.tx_coalesce_bytes, interner, token.value());
            if (!adopted_native.has_value()) {
                return m5::stl::make_unexpected(adopted_native.error());
            }
            pending.dismiss();

            std::shared_ptr<Bus> facade{new (std::nothrow) Bus()};
            if (!facade) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            facade->bindLocalResources(hal_backend.localResources());
            std::unique_ptr<IBus> backend{concrete.release()};
            auto adopted = facade->adoptPortableBackend(std::move(backend), cfg);
            if (!adopted.has_value()) {
                return m5::stl::make_unexpected(adopted.error());
            }
            return std::shared_ptr<bus::IBus>{std::move(facade)};
        });
    if (!acquired.has_value()) {
        return m5::stl::make_unexpected(acquired.error());
    }
    return std::static_pointer_cast<IBus>(acquired.value());
}

}  // namespace impl_posix
}  // namespace

bool Bus_posix::baudToSpeed(uint32_t baud, uint32_t& out_speed)
{
    speed_t s = 0;
    if (!impl_posix::baudConstant(baud, s)) {
        return false;
    }
    out_speed = static_cast<uint32_t>(s);
    return true;
}

Bus_posix::~Bus_posix()
{
    (void)teardownBackend();
}

result_t<void> Bus_posix::init(const IBusConfig& config)
{
    (void)config;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
}

result_t<void> Bus_posix::initOwnedDirect(int fd, const IBusConfig& config, const NativeOptions& options)
{
    if (fd < 0 || options.tx_coalesce_bytes > kCoalesceCapacity) {
        if (fd >= 0) {
            (void)::close(fd);
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto reset = resetForInitialization();
    if (!reset.has_value()) {
        (void)::close(fd);
        return reset;
    }
    _config      = config;
    _fd          = fd;
    _owns_fd     = true;
    _begun       = false;
    _tx_coalesce = options.tx_coalesce_bytes;

    uart::AccessConfig access_config;
    auto applied = applyConfig(nullptr, Channel::None, access_config);
    if (!applied.has_value()) {
        (void)resetForInitialization();
        return applied;
    }
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)resetForInitialization();
    }
    return initialized;
}

result_t<void> Bus_posix::init(const IBusConfig& config, native::Borrowed<NativeFd> policy)
{
    return initOwnedDirect(impl_posix::duplicateFd(policy.resource().value), config, NativeOptions{});
}

result_t<void> Bus_posix::init(const IBusConfig& config, native::Managed<NativePath> policy)
{
    auto arguments = std::move(policy).arguments();
    return init(config, native::managed(std::get<0>(arguments), NativeOptions{}));
}

result_t<void> Bus_posix::init(const IBusConfig& config, native::Managed<NativePath, NativeOptions> policy)
{
    auto arguments          = std::move(policy).arguments();
    const NativePath path   = std::get<0>(arguments);
    const NativeOptions opt = std::get<1>(arguments);
    if (path.value.empty()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const int fd = ::open(path.value.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return m5::stl::make_unexpected(impl_posix::posixIOError());
    }
    return initOwnedDirect(fd, config, opt);
}

bus::CloseOutcome Bus_posix::closeBackend(void)
{
    return teardownBackend();
}

bus::CloseOutcome Bus_posix::teardownBackend(void)
{
    auto locked = _state_mutex.lock(types::TIMEOUT_FOREVER);
    if (!locked.has_value()) {
        return bus::CloseOutcome::noMutation(locked.error());
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};

    bool mutated = false;
    if (_original_termios_valid && _fd >= 0) {
        if (::tcsetattr(_fd, TCSANOW, &_original_termios) != 0) {
            return bus::CloseOutcome::noMutation(impl_posix::posixIOError());
        }
        _original_termios_valid = false;
        mutated                 = true;
    }
    if (_owns_fd && _fd >= 0) {
        if (::close(_fd) != 0) {
            // POSIX leaves descriptor state unspecified for some close
            // failures (notably EINTR). Never retry close on a number the OS
            // may already have reused; retain the remaining token for retry.
            _fd      = -1;
            _owns_fd = false;
            _begun   = false;
            _co_used = 0;
            return bus::CloseOutcome::partialOrUnknown(impl_posix::posixIOError());
        }
        mutated = true;
    } else if (_fd >= 0) {
        // A legacy attached descriptor remains caller-owned, but detaching it
        // is still a teardown step before a native token can be released.
        mutated = true;
    }
    _fd      = -1;
    _owns_fd = false;
    _begun   = false;
    _co_used = 0;
    if (_native_interner != nullptr && _native_token.valid()) {
        auto token_released = _native_interner->release(_native_token);
        if (!token_released.has_value()) {
            if (mutated) {
                return bus::CloseOutcome::partialOrUnknown(token_released.error());
            }
            return bus::CloseOutcome::noMutation(token_released.error());
        }
        mutated = true;
    }
    _native_interner = nullptr;
    _native_token    = {};
    // Preserve the init-time coalescing policy across attach(fd), as before.
    return bus::CloseOutcome::success();
}

result_t<void> Bus_posix::resetForInitialization(void)
{
    auto outcome = teardownBackend();
    if (outcome.disposition == bus::CloseDisposition::Success) {
        return {};
    }
    if (outcome.disposition == bus::CloseDisposition::PartialOrUnknown) {
        quarantineLifecycleAfterPartialTeardown();
    }
    return m5::stl::make_unexpected(outcome.error_code);
}

result_t<void> Bus_posix::adoptOwnedNative(
    int fd, const IBusConfig& config, size_t tx_coalesce_bytes,
    bus::FixedNativeInterner<bus::NativeIdentity, bus::BusRegistry::kCapacity>& interner, bus::NativeToken token)
{
    if (fd < 0 || !token.valid() || tx_coalesce_bytes > kCoalesceCapacity) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto reset = resetForInitialization();
    if (!reset.has_value()) {
        return reset;
    }
    _fd              = fd;
    _owns_fd         = true;
    _begun           = false;
    _config          = config;
    _tx_coalesce     = tx_coalesce_bytes;
    _native_interner = &interner;
    _native_token    = token;
    return {};
}

result_t<std::shared_ptr<IBus>> NativeProvider_posix<native::Borrowed<NativeFd>>::acquire(
    bus::IHalBackend& backend, const IBusConfig& cfg, native::Borrowed<NativeFd> policy)
{
    if (backend.localResourceDomain() == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    const int duplicate = impl_posix::duplicateFd(policy.resource().value);
    if (duplicate < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return impl_posix::acquireBoundFd(backend, cfg, duplicate, bus::Ownership::Borrowed, NativeOptions{});
}

result_t<std::shared_ptr<IBus>> NativeProvider_posix<native::Managed<NativePath>>::acquire(
    bus::IHalBackend& backend, const IBusConfig& cfg, native::Managed<NativePath> policy)
{
    auto arguments = std::move(policy).arguments();
    return NativeProvider_posix<native::Managed<NativePath, NativeOptions>>::acquire(
        backend, cfg, native::managed(std::get<0>(arguments), NativeOptions{}));
}

result_t<std::shared_ptr<IBus>> NativeProvider_posix<native::Managed<NativePath, NativeOptions>>::acquire(
    bus::IHalBackend& backend, const IBusConfig& cfg, native::Managed<NativePath, NativeOptions> policy)
{
    if (backend.localResourceDomain() == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    auto arguments          = std::move(policy).arguments();
    const NativePath path   = std::get<0>(arguments);
    const NativeOptions opt = std::get<1>(arguments);
    if (path.value.empty()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    char canonical[PATH_MAX];
    if (::realpath(path.value.c_str(), canonical) == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const int fd = ::open(canonical, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return m5::stl::make_unexpected(impl_posix::posixIOError());
    }
    return impl_posix::acquireBoundFd(backend, cfg, fd, bus::Ownership::Managed, opt);
}

uint32_t Bus_posix::reconfigSkips()
{
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER).has_value()) {
        return 0;
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};
    return _reconfig_skips;
}

result_t<void> Bus_posix::beginOperationBackend(bus::OperationContext<uart::AccessConfig>& context)
{
    const Channel entered = context.runtime.mode == bus::OperationMode::Tx ? Channel::Tx : Channel::Rx;
    return applyConfig(operationOwner(context), entered, context.config,
                       bus::remainingTimeout(context.runtime, runtime::millis()));
}

result_t<void> Bus_posix::endOperationBackend(bus::OperationContext<uart::AccessConfig>& context)
{
    if (context.runtime.mode != bus::OperationMode::Tx) {
        return {};
    }
    return flushCoalesced(bus::remainingTimeout(context.runtime, runtime::millis()));
}

result_t<void> Bus_posix::applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg,
                                      uint32_t timeout_ms)
{
    if (cfg.baud_rate == 0 || cfg.data_bits != 8 || (cfg.stop_bits != 1 && cfg.stop_bits != 2)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    auto locked = _state_mutex.lock(timeout_ms);
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};

    if (_fd < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    if (!_begun) {
        // First apply on this fd: no other owner can be mid-transfer yet
        // (spec/design/uart.md), so no quiescence gate is needed.
        return applyConfigLocked(cfg);
    }
    if (impl_posix::sameConfig(_applied_cfg, cfg)) {
        return {};
    }

    if (owner == nullptr) {
        // A reconfigure without an accessor identity cannot prove quiescence.
        ++_reconfig_skips;
        M5HAL_DIAG("uart reconfig rejected: no accessor identity (skips=%u)", static_cast<unsigned>(_reconfig_skips));
        return {};
    }

    // Reconfigure: apply only when the opposite channel is quiescent for
    // `owner` (spec/design/uart.md; IBus::tryAcquireOppositeChannel is the
    // sanctioned exception to the channel-lock -> state-mutex ordering).
    auto& ibus = static_cast<IBus&>(owner->getBus());
    auto grant = ibus.tryAcquireOppositeChannel(owner, entered);
    if (!grant.granted) {
        ++_reconfig_skips;
        M5HAL_DIAG("uart reconfig rejected: opposite channel busy (skips=%u)", static_cast<unsigned>(_reconfig_skips));
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    auto applied = applyConfigLocked(cfg);
    ibus.releaseOppositeChannel(owner, grant);
    return applied;
}

result_t<void> Bus_posix::applyConfigLocked(const uart::AccessConfig& cfg)
{
    speed_t speed          = 0;
    const bool have_bconst = impl_posix::baudConstant(cfg.baud_rate, speed);
#if !defined(__APPLE__)
    // Linux/other: only rates that have a termios B* constant are supported.
    // glibc/musl provide B460800..B4000000, covering the standard high rates.
    if (!have_bconst) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#endif

    struct termios tio;
    if (::tcgetattr(_fd, &tio) != 0) {
        return m5::stl::make_unexpected(impl_posix::posixIOError());
    }
    if (!_original_termios_valid) {
        _original_termios       = tio;
        _original_termios_valid = true;
    }
    ::cfmakeraw(&tio);
    // A rate without a B* constant (macOS high baud) gets a placeholder here; the
    // real integer speed is applied via IOSSIOSPEED after tcsetattr() below.
    const speed_t cfspeed = have_bconst ? speed : static_cast<speed_t>(B9600);
    ::cfsetispeed(&tio, cfspeed);
    ::cfsetospeed(&tio, cfspeed);

    tio.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
    tio.c_cflag |= static_cast<tcflag_t>(CS8);
    if (cfg.stop_bits == 2) {
        tio.c_cflag |= static_cast<tcflag_t>(CSTOPB);
    } else {
        tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
    }
    switch (cfg.parity) {
        case uart::parity_t::None:
            tio.c_cflag &= ~static_cast<tcflag_t>(PARENB);
            break;
        case uart::parity_t::Even:
            tio.c_cflag |= static_cast<tcflag_t>(PARENB);
            tio.c_cflag &= ~static_cast<tcflag_t>(PARODD);
            break;
        case uart::parity_t::Odd:
            tio.c_cflag |= static_cast<tcflag_t>(PARENB);
            tio.c_cflag |= static_cast<tcflag_t>(PARODD);
            break;
        default:
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    // Non-blocking semantics; timeouts are enforced by impl_posix::waitFd()/select().
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (::tcsetattr(_fd, TCSANOW, &tio) != 0) {
        return m5::stl::make_unexpected(impl_posix::posixIOError());
    }
#if defined(__APPLE__)
    // Set a baud that termios has no B* constant for (e.g. 0.5/1/2/3 Mbaud).
    // Must follow tcsetattr(), which would otherwise reset the line speed.
    if (!have_bconst) {
        speed_t real = static_cast<speed_t>(cfg.baud_rate);
        if (::ioctl(_fd, IOSSIOSPEED, &real) != 0) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
    }
#endif
    // Ensure the descriptor is non-blocking even when it was adopted via attach().
    int flags = ::fcntl(_fd, F_GETFL, 0);
    if (flags >= 0) {
        (void)::fcntl(_fd, F_SETFL, flags | O_NONBLOCK);
    }
    // Deliberately no tcflush() here: discarding pending RX would drop bytes a
    // peer already sent before this (re)configure. Stale-input draining, if
    // ever wanted, belongs to the caller.

    _applied_cfg = cfg;
    _begun       = true;
    return {};
}

result_t<size_t> Bus_posix::rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    size_t done = 0;
    while (done < len) {
        ssize_t n = ::write(_fd, data + done, len - done);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (impl_posix::waitFd(_fd, true, timeout_ms) > 0) {
                    continue;
                }
                break;  // write timeout
            }
            return m5::stl::make_unexpected(impl_posix::posixIOError());
        }
        if (n == 0) {
            break;
        }
        done += static_cast<size_t>(n);
    }
    return done;
}

result_t<void> Bus_posix::flushCoalescedLocked(uint32_t timeout_ms)
{
    if (_co_used == 0) {
        return {};
    }
    const size_t pending = _co_used;
    _co_used             = 0;  // reset first: a failed flush must not replay stale bytes
    // B12: rawWrite runs while _state_mutex is still held (the accepted
    // trade-off — see spec/design/uart.md — a concurrent RX-side flush call
    // waits up to write_timeout_ms behind this write instead of racing it).
    auto w = rawWrite(_co_buf, pending, timeout_ms);
    if (!w.has_value()) {
        return m5::stl::make_unexpected(w.error());
    }
    if (w.value() != pending) {
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    return {};
}

result_t<void> Bus_posix::flushCoalesced(uint32_t timeout_ms)
{
    auto locked = _state_mutex.lock(timeout_ms);
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock state_unlock{_state_mutex};
    return flushCoalescedLocked(timeout_ms);
}

result_t<size_t> Bus_posix::writeBackend(bus::OperationContext<uart::AccessConfig>& context, data::Source* src,
                                         size_t len)
{
    const auto& cfg  = context.config;
    const size_t cap = _tx_coalesce == 0 ? 0 : std::min(_tx_coalesce, kCoalesceCapacity);
    size_t done      = 0;
    while (src != nullptr && !src->eof() && done < len) {
        auto span = src->peek(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        size_t n = 0;
        if (cap == 0 || span.value().size >= cap) {
            // Write-through (coalescing off, or the span alone fills a batch).
            // flushCoalesced() (self-locking) drains any stale coalesced
            // bytes first; this rawWrite of the NEW span is deliberately NOT
            // under _state_mutex (B12: only _co_buf/_co_used need it).
            auto f = flushCoalesced(cfg.write_timeout_ms);
            if (!f.has_value()) {
                return m5::stl::make_unexpected(f.error());
            }
            auto w = rawWrite(span.value().data, span.value().size, cfg.write_timeout_ms);
            if (!w.has_value()) {
                return m5::stl::make_unexpected(w.error());
            }
            n = w.value();
        } else {
            // Coalesce-buffer append (B12): capacity check + optional flush +
            // memcpy + size update must be one atomic step against a
            // concurrent RX-side flushCoalesced() call.
            auto locked = _state_mutex.lock(types::TIMEOUT_FOREVER);
            if (!locked.has_value()) {
                return m5::stl::make_unexpected(locked.error());
            }
            runtime::ScopedUnlock state_unlock{_state_mutex};
            if (_co_used + span.value().size > cap) {
                auto f = flushCoalescedLocked(cfg.write_timeout_ms);
                if (!f.has_value()) {
                    return m5::stl::make_unexpected(f.error());
                }
            }
            ::memcpy(_co_buf + _co_used, span.value().data, span.value().size);
            _co_used += span.value().size;
            n = span.value().size;
        }
        if (n == 0) {
            break;
        }
        auto advanced = src->advance(n);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        done += n;
        if (n < span.value().size) {
            break;  // write timeout: report the short write
        }
    }
    // No tcdrain(): the bytes are handed to the OS write buffer here, and
    // draining the line discipline can block indefinitely on a pty (the
    // loopback used by tests). Flushing the physical UART FIFO is the OS
    // driver's job; callers that need a hard drain can add one out of band.
    return done;
}

result_t<size_t> Bus_posix::rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    int ready = impl_posix::waitFd(_fd, false, timeout_ms);
    if (ready < 0) {
        return m5::stl::make_unexpected(impl_posix::posixIOError());
    }
    if (ready == 0) {
        return static_cast<size_t>(0);
    }
    ssize_t n = ::read(_fd, buf, len);
    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            return static_cast<size_t>(0);
        }
        return m5::stl::make_unexpected(impl_posix::posixIOError());
    }
    return static_cast<size_t>(n);
}

result_t<size_t> Bus_posix::rawReadableBytes()
{
    int avail = 0;
    if (::ioctl(_fd, FIONREAD, &avail) != 0 || avail < 0) {
        return m5::stl::make_unexpected(impl_posix::posixIOError());
    }
    return static_cast<size_t>(avail);
}

result_t<size_t> Bus_posix::readBackend(bus::OperationContext<uart::AccessConfig>& context, data::Sink* dst, size_t len)
{
    const auto& cfg = context.config;
    auto flushed    = flushCoalesced(cfg.write_timeout_ms);
    if (!flushed.has_value()) {
        return m5::stl::make_unexpected(flushed.error());
    }
    return Bus_streaming::readBackend(context, dst, len);
}

result_t<size_t> Bus_posix::readableBytesBackend(bus::OperationContext<uart::AccessConfig>& context)
{
    const auto& cfg = context.config;
    auto flushed    = flushCoalesced(cfg.write_timeout_ms);
    if (!flushed.has_value()) {
        return m5::stl::make_unexpected(flushed.error());
    }
    return Bus_streaming::readableBytesBackend(context);
}

}  // namespace m5::hal::v2::uart

#endif  // M5HAL_FRAMEWORK_HAS_POSIX && M5HAL_CONFIG_POSIX_UART

#endif
