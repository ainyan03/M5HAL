// SPDX-License-Identifier: MIT
//
// HIL host driver — host POSIX UART variant <-> ESP32 echo interop.
//
// Verifies that bytes sent from the host through the M5HAL posix UART variant
// reach a real ESP32 running the paired echo firmware
// (experiments/v1/test/uart_echo/device/uart_echo.cpp) and come back byte-exact.
//
// This is a gtest binary built and run via `pio run` (not `pio test`): see
// experiments/v1/test/README.md. The serial port comes from M5HAL_POSIX_UART_PORT
// (and optional M5HAL_POSIX_UART_BAUD, default 115200); the test SKIPs when the
// port is unset, so building without hardware is harmless.
//
// Run (or use experiments/v1/test/hil-run.sh uart_echo [port] [baud]):
//   export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v1/hil.ini.cli
//   pio run -e v1_hil_uart_echo_device_esp32 -t upload      # flash the device
//   pio run -e v1_hil_uart_echo_host                        # build the host
//   M5HAL_POSIX_UART_PORT=/dev/cu.usbserial-XXXX \
//     .pio/build/v1_hil_uart_echo_host/program

#include <M5HAL_v1.hpp>
#include <gtest/gtest.h>

#include <csignal>

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <cstring>
#include <vector>

#include "../../common/hil_host.hpp"

namespace {

namespace uart = ::m5::hal::v1::uart;
namespace data = ::m5::hal::v1::data;

class UartEcho : public ::testing::Test {
protected:
    void SetUp() override
    {
        const char* port = hil::portEnv();
        if (port == nullptr) {
            GTEST_SKIP() << "set M5HAL_POSIX_UART_PORT=/dev/cu.usbserial-... to run this HIL test";
        }
        baud_ = hil::baudEnv();
        ASSERT_TRUE(hil::openSynced(bus_, port, baud_))
            << "no echo from " << port << " — is the uart_echo device firmware flashed at this baud?";
    }

    uart::Bus bus_;
    uint32_t baud_ = 115200;
};

TEST_F(UartEcho, EchoesSmallPayload)
{
    auto cfg = hil::ioConfig(baud_);
    uart::TxAccessor tx_dev{bus_, cfg};
    uart::RxAccessor rx_dev{bus_, cfg};
    const uint8_t tx[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    ASSERT_TRUE(tx_dev.write(data::ConstDataSpan{tx, sizeof(tx)}).has_value());

    uint8_t rx[sizeof(tx)] = {};
    EXPECT_EQ(hil::readExact(rx_dev, rx, sizeof(tx), 10), sizeof(tx));
    EXPECT_EQ(::memcmp(rx, tx, sizeof(tx)), 0);
}

TEST_F(UartEcho, EchoesBinaryIncludingNulAndNewline)
{
    auto cfg = hil::ioConfig(baud_);
    uart::TxAccessor tx_dev{bus_, cfg};
    uart::RxAccessor rx_dev{bus_, cfg};
    // Includes NUL, LF, CR, high bytes — must survive raw (no line discipline).
    const uint8_t tx[] = {0x00, 0x0A, 0x0D, 0xFF, 0x7F, 0x80, 0x00, 0x55};
    ASSERT_TRUE(tx_dev.write(data::ConstDataSpan{tx, sizeof(tx)}).has_value());

    uint8_t rx[sizeof(tx)] = {};
    EXPECT_EQ(hil::readExact(rx_dev, rx, sizeof(tx), 16), sizeof(tx));
    EXPECT_EQ(::memcmp(rx, tx, sizeof(tx)), 0);
}

TEST_F(UartEcho, EchoesLargePayloadInOrder)
{
    auto cfg = hil::ioConfig(baud_);
    uart::TxAccessor tx_dev{bus_, cfg};
    uart::RxAccessor rx_dev{bus_, cfg};
    std::vector<uint8_t> tx(512);
    for (size_t i = 0; i < tx.size(); ++i) {
        tx[i] = static_cast<uint8_t>(i * 7 + 1);
    }
    ASSERT_TRUE(tx_dev.write(data::ConstDataSpan{tx.data(), tx.size()}).has_value());

    std::vector<uint8_t> rx(tx.size());
    EXPECT_EQ(hil::readExact(rx_dev, rx.data(), rx.size(), 200), tx.size());
    EXPECT_EQ(::memcmp(rx.data(), tx.data(), tx.size()), 0);
}

}  // namespace

#else  // M5HAL_FRAMEWORK_HAS_POSIX

TEST(UartEcho, SkippedWhenPosixDisabled)
{
    SUCCEED() << "POSIX UART variant disabled (M5HAL_CONFIG_POSIX_UART=0 or non-POSIX host)";
}

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

int main(int argc, char** argv)
{
    // Closing a pty/serial end can raise SIGHUP; ignore it so teardown does not
    // kill the run.
    ::signal(SIGHUP, SIG_IGN);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
