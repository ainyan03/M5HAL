// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_DETAIL_SLAVE_TX_LEDGER_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_DETAIL_SLAVE_TX_LEDGER_HPP_

#include "../../../../../../hal/v2/error.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace m5::hal::v2::i2c::detail {

enum class SlaveTxProvenance : uint8_t { Real, Fill };
enum class SlaveTxBoundaryEvidence : uint8_t { ShifterAmbiguous, ShifterDrained };

struct SlaveTxTotals {
    size_t real = 0;
    size_t fill = 0;
};

struct SlaveTxBoundaryResult {
    SlaveTxTotals confirmed{};
    SlaveTxTotals unclocked{};
};

/*!
  @brief Fixed-capacity provenance ledger for an I2C slave TX FIFO.

  `observeFifoOccupancy()` confirms only departures older than the one byte
  that may already have moved from the FIFO into the hardware shifter. A wire
  A STOP/abort boundary does not prove that the final prefetched byte outside
  the FIFO reached the wire: after a multi-byte read, an early master NACK may
  leave one byte in the hardware shifter. A one-byte read has no prior ACK that
  could trigger such a prefetch. `confirmBoundaryAndDiscard()` therefore
  preserves one byte only when there are multiple departures, unless the
  caller has stronger `ShifterDrained` evidence. The caller may dequeue only
  confirmed `Real` bytes and account confirmed `Fill` bytes as underruns.
 */
template <size_t FifoCapacity>
class SlaveTxLedger {
public:
    static_assert(FifoCapacity != 0, "SlaveTxLedger requires a non-zero FIFO capacity");
    static constexpr size_t kCapacity = FifoCapacity + 1;

    result_t<void> load(SlaveTxProvenance provenance, size_t count = 1)
    {
        if (!valid(provenance)) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        if (count > FifoCapacity - fifo_occupancy_ || count > kCapacity - size_) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        for (size_t i = 0; i < count; ++i) {
            entries_[(head_ + size_) % kCapacity] = provenance;
            ++size_;
        }
        fifo_occupancy_ += count;
        return {};
    }

    result_t<SlaveTxTotals> observeFifoOccupancy(size_t occupancy)
    {
        auto valid_observation = validateObservation(occupancy);
        if (!valid_observation.has_value()) {
            return m5::stl::make_unexpected(valid_observation.error());
        }
        const size_t departed  = size_ - occupancy;
        const size_t confirmed = departed == 0 ? 0 : departed - 1;
        auto result            = confirmFront(confirmed);
        if (!result.has_value()) {
            return m5::stl::make_unexpected(result.error());
        }
        fifo_occupancy_ = occupancy;
        return *result;
    }

    result_t<SlaveTxBoundaryResult> confirmBoundaryAndDiscard(size_t occupancy, SlaveTxBoundaryEvidence evidence)
    {
        auto valid_observation = validateObservation(occupancy);
        if (!valid_observation.has_value()) {
            return m5::stl::make_unexpected(valid_observation.error());
        }
        const size_t departed = size_ - occupancy;
        const size_t confirmed_count =
            evidence == SlaveTxBoundaryEvidence::ShifterDrained || departed <= 1 ? departed : departed - 1;
        auto confirmed = confirmFront(confirmed_count);
        if (!confirmed.has_value()) {
            return m5::stl::make_unexpected(confirmed.error());
        }

        SlaveTxBoundaryResult result;
        result.confirmed = *confirmed;
        for (size_t i = 0; i < size_; ++i) {
            increment(result.unclocked, entries_[(head_ + i) % kCapacity]);
        }
        head_           = 0;
        size_           = 0;
        fifo_occupancy_ = 0;
        return result;
    }

    void reset()
    {
        head_             = 0;
        size_             = 0;
        fifo_occupancy_   = 0;
        confirmed_totals_ = {};
    }

    size_t outstanding() const
    {
        return size_;
    }

    size_t fifoOccupancy() const
    {
        return fifo_occupancy_;
    }

    SlaveTxTotals confirmedTotals() const
    {
        return confirmed_totals_;
    }

private:
    static bool valid(SlaveTxProvenance provenance)
    {
        return provenance == SlaveTxProvenance::Real || provenance == SlaveTxProvenance::Fill;
    }

    result_t<void> validateObservation(size_t occupancy) const
    {
        if (occupancy > FifoCapacity) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        if (occupancy > fifo_occupancy_ || occupancy > size_) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        return {};
    }

    static void increment(SlaveTxTotals& totals, SlaveTxProvenance provenance)
    {
        if (provenance == SlaveTxProvenance::Real) {
            ++totals.real;
        } else {
            ++totals.fill;
        }
    }

    result_t<SlaveTxTotals> confirmFront(size_t count)
    {
        if (count > size_) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        SlaveTxTotals delta;
        for (size_t i = 0; i < count; ++i) {
            increment(delta, entries_[(head_ + i) % kCapacity]);
        }
        if (delta.real > std::numeric_limits<size_t>::max() - confirmed_totals_.real ||
            delta.fill > std::numeric_limits<size_t>::max() - confirmed_totals_.fill) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        confirmed_totals_.real += delta.real;
        confirmed_totals_.fill += delta.fill;
        head_ = (head_ + count) % kCapacity;
        size_ -= count;
        return delta;
    }

    std::array<SlaveTxProvenance, kCapacity> entries_{};
    size_t head_           = 0;
    size_t size_           = 0;
    size_t fifo_occupancy_ = 0;
    SlaveTxTotals confirmed_totals_{};
};

}  // namespace m5::hal::v2::i2c::detail

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_DETAIL_SLAVE_TX_LEDGER_HPP_
