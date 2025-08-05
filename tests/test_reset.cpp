#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "qca7000.hpp"
#include <string>

extern uint16_t mock_signature;
extern uint16_t mock_wrbuf;
extern uint16_t mock_intr_cause;

TEST(Qca7000ResetSimple, ReturnsTrue) {
    mock_signature = 0xAA55;
    mock_wrbuf = 0x0C5B;
    mock_intr_cause = SPI_INT_CPU_ON;
    ASSERT_TRUE(qca7000ResetAndCheck());
}

TEST(Qca7000ResetSimple, MissingCpuOnHardReset) {
    mock_signature = 0xAA55;
    mock_wrbuf = 0x0C5B;
    mock_intr_cause = 0;
    testing::internal::CaptureStderr();
    EXPECT_FALSE(qca7000ResetAndCheck());
    std::string log = testing::internal::GetCapturedStderr();
    EXPECT_NE(std::string::npos, log.find("CPU_ON"));
    mock_intr_cause = SPI_INT_CPU_ON;
}

TEST(Qca7000ResetSimple, MissingCpuOnSoftReset) {
    mock_intr_cause = 0;
    testing::internal::CaptureStderr();
    EXPECT_FALSE(qca7000SoftReset());
    std::string log = testing::internal::GetCapturedStderr();
    EXPECT_NE(std::string::npos, log.find("CPU_ON"));
    mock_intr_cause = SPI_INT_CPU_ON;
}
