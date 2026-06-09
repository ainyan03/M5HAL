// =============================================================================
// M5HAL — HowToUseI2C
//
// I2C Bus / Accessor の使い方を最小限に示す sketch。 特定 device を想定せず、
// 「bus scan で見つけた最初の device」 に対して probe / readRegister /
// ScopedAccess を順に試す。 利用者の手元に I2C device が 1 つでも繋がって
// いれば動作する (M5Stack Core 付属の AXP192 / MPU6886、 M5Env III、
// I2C EEPROM、 など何でも可)。
//
// 流れ:
//   setup()
//     1. Bus を init() で作る
//     2. scan で 0x08〜0x77 を probe()、 最初に ACK を返した address を捕捉
//     3. その address に readRegister(REG_PROBE) で 1 byte を読む
//     4. burst read で 4 byte を読む
//     5. ScopedAccess で連続 transfer を atomic 範囲に囲む例
//
// note: writeRegister / readRegister は register address を指定する糖衣 API。
// `readRegister(0x00)` のような直値は 1-byte register address として扱われる。
// 2-byte register address のデバイスでは `acc_cfg.register_address_bytes = 2` を
// 指定すると、直値 `0x1234` が big-endian で wire に送出される。
// register address 幅を型で固定したい場合は `static constexpr uint8_t REG_X` や
// `static constexpr uint16_t REG_PAGE` のような型付き定数を使う。
// =============================================================================

#include <Arduino.h>
#include <M5HAL_v1.hpp>  // v1 API entry header (see examples/v1/ で v1 系を明示)
#include <Wire.h>

namespace m5hal = m5::hal::v1;

// M5Stack Core (Basic / Gray / Fire) の I2C pin。 他デバイスを使う場合は
// ピン番号を書き換える。 attach 路線 (caller-owned Wire) を使うなら
// `Wire.begin(SDA, SCL)` → `bus.attach(Wire)` も可。
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// device 固有の register address は型付き定数にしておくと幅が明示的になる。
// この sketch では、直値糖衣の例も見せるため demoReadRegister だけ `0x00`
// を直接渡している。
static constexpr uint8_t REG_PROBE_R  = 0x00;  // probe 用 (register 0x00 は多くの device で何かを返す)
static constexpr uint8_t REG_PROBE_R2 = 0x01;

#ifndef M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ
#define M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ 100000
#endif

// グローバル所有 (Arduino sketch の典型的な書き方に合わせる)
// I2C backend を切り替える場合は、下の using のどちらか一方を有効にする。
using ExampleI2CBus = m5hal::i2c::Bus;
// using ExampleI2CBus = m5::variants::frameworks::software::hal::v1::i2c::Bus;
ExampleI2CBus i2c_bus;

// -------------------------------------------------------------------------
// I2C バススキャン: 0x08〜0x77 を probe() で順に叩き、 ACK を返した address
// を返す。 見つからなければ 0xFFFF。

static uint16_t scanFirst(m5hal::i2c::I2CBus& bus)
{
    Serial.println("I2C scan:");
    uint16_t found = 0xFFFF;
    // I2CBus::probe(addr) sugar (spec_polish A2) を使うと、 caller は Accessor を
    // 作らず 1 行で probe できる。 default 引数 (freq=100000Hz / timeout=50ms) は
    // scan 用途で実用的な値が選ばれているのでそのまま使う。 内部では Bus が
    // stack-allocated な sentinel Accessor を作って lock 取得 → 0-byte transfer →
    // unlock を回す。 caller の Accessor 構築回数は 0 になる。
    for (uint16_t addr = 0x08; addr < 0x78; ++addr) {
        if (bus.probe(addr).has_value()) {
            Serial.printf("  found device at 0x%02X\n", addr);
            if (found == 0xFFFF) {
                found = addr;
            }
        }
    }
    return found;
}

// -------------------------------------------------------------------------
// 1 byte register read の最小デモ。 register 0x00 はほぼ全ての I2C device
// で何らかの値を返す (typically chip ID / status)。

static void demoReadRegister(m5hal::i2c::I2CMasterAccessor& dev)
{
    auto v = dev.readRegister(0x00);  // 直値は default で 1-byte register address
    if (!v.has_value()) {
        Serial.printf("readRegister(0x00) failed: %d\n", (int)v.error());
        return;
    }
    Serial.printf("register 0x00 = 0x%02X\n", v.value());
}

// -------------------------------------------------------------------------
// burst read の最小デモ。 register 0x00 から 4 byte を一括で読む。
// 多くの I2C device は連続 register の auto-increment を実装している。

static void demoBurstRead(m5hal::i2c::I2CMasterAccessor& dev)
{
    // raw `uint8_t* + size_t` overload (spec_polish A3): C 配列をそのまま渡せる。
    // Span を組む必要なし。 Span 版 (`m5hal::data::DataSpan{buf, sizeof(buf)}`)
    // も併存しているので、 caller は好みで選べる。
    uint8_t buf[4] = {};
    auto r         = dev.readRegister(REG_PROBE_R, buf, sizeof(buf));
    if (!r.has_value()) {
        Serial.printf("burst read failed: %d\n", (int)r.error());
        return;
    }
    Serial.printf("registers 0x00..0x03 = %02X %02X %02X %02X\n", buf[0], buf[1], buf[2], buf[3]);
}

// -------------------------------------------------------------------------
// ScopedAccess の使い方。 複数の transfer を 1 つの bus lock 期間に囲み、
// 他の Accessor が割り込まないことを保証する。 個々の transfer sugar 内部
// でも beginAccess/endAccess は呼ばれるが、 depth counter により外側
// ScopedAccess がある場合は実際の lock 取得・解放は 1 回で済む。

static void demoScopedAccess(m5hal::i2c::I2CMasterAccessor& dev)
{
    m5hal::bus::ScopedAccess scope{dev};
    if (scope.has_error()) {
        Serial.printf("ScopedAccess failed: %d\n", (int)scope.error());
        return;
    }
    // ここでの 2 connections は他 Accessor に lock を奪われない
    auto a = dev.readRegister(REG_PROBE_R);
    auto b = dev.readRegister(REG_PROBE_R2);
    if (a.has_value() && b.has_value()) {
        Serial.printf("atomic read: 0x%02X 0x%02X\n", a.value(), b.value());
    }
}  // scope の dtor で bus を unlock

// -------------------------------------------------------------------------
// setup: Bus を初期化して device 1 つに対して各 sugar を順に試す

void setup()
{
    Serial.begin(115200);
    delay(500);

    // --- Bus 初期化 ---
    //
    // 2 経路あり:
    //   (a) init(BusConfig): bus が Wire を所有して begin/end する (本 sketch)
    //   (b) attach(Wire):    caller が Wire の lifecycle を持つ
    //
    // pin 指定は BusConfig::pin_scl / pin_sda に gpio_number_t (= int16_t、
    // default = -1 invalid) を入れる単一 path:
    //   (1) 番号だけ知っている典型ケース: 2-引数 ctor で 1 行
    //         m5hal::i2c::BusConfig bus_cfg{&Wire, PIN_SCL, PIN_SDA};
    //   (2) 個別代入:
    //         bus_cfg.wire = &Wire;
    //         bus_cfg.pin_scl = PIN_SCL;
    //         bus_cfg.pin_sda = PIN_SDA;
    //
    // expander pin を SCL/SDA に使う escape hatch は `M5_Hal.Gpio` (S9a 採用、
    // singleton GPIOGroup) 経由:
    //   constexpr m5hal::types::gpio_slot_t EXPANDER_SLOT = 1;  // slot 0 は MCU 予約
    //   m5hal::M5_Hal.Gpio.addGPIO(&expander_gpio, EXPANDER_SLOT);
    //   m5hal::i2c::BusConfig bus_cfg{
    //       &Wire,
    //       m5hal::types::makeGpioNumber(EXPANDER_SLOT, 0),     // expander local pin 0 = SCL
    //       m5hal::types::makeGpioNumber(EXPANDER_SLOT, 1)};    // expander local pin 1 = SDA
    m5hal::i2c::BusConfig bus_cfg{&Wire, PIN_SCL, PIN_SDA};

    if (auto r = i2c_bus.init(bus_cfg); !r) {
        Serial.printf("Bus init failed: %d\n", (int)r.error());
        return;
    }

    // --- バススキャン ---
    auto addr = scanFirst(i2c_bus);
    if (addr == 0xFFFF) {
        Serial.println("No I2C device found, abort.");
        return;
    }
    Serial.printf("Using device at 0x%02X for demo.\n", addr);

    // --- Accessor を作って各 sugar を順に試す ---
    m5hal::i2c::I2CMasterAccessConfig acc_cfg;
    acc_cfg.i2c_addr   = addr;
    acc_cfg.freq       = M5HAL_EXAMPLE_HOWTOUSEI2C_FREQ;
    acc_cfg.timeout_ms = 100;
    m5hal::i2c::I2CMasterAccessor dev{i2c_bus, acc_cfg};

    demoReadRegister(dev);
    demoBurstRead(dev);
    demoScopedAccess(dev);

    Serial.println("HowToUseI2C done.");
}

void loop()
{
    delay(1000);
}
