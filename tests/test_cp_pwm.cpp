#include <gtest/gtest.h>
#define ARDUINO
#include "arduino_stubs.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"

// Global variables used by stubs
gpio_mode_t g_last_gpio_mode;
gpio_mode_t g_gpio_mode_at_ledc_set_duty;
uint32_t g_ledc_last_duty;

// Override configuration to disable idle drive high
#define CP_IDLE_DRIVE_HIGH 0
#include "../examples/platformio_complete/src/cp_pwm.cpp"

TEST(CpPwm, ResumesAfterStopWhenIdleDriveLow) {
    // Initialize and start PWM
    cpPwmInit();
    cpPwmStart(CP_PWM_DUTY_5PCT, true);
    cpPwmStop();

    // Clear tracking variables
    g_last_gpio_mode = static_cast<gpio_mode_t>(-1);
    g_gpio_mode_at_ledc_set_duty = static_cast<gpio_mode_t>(-1);
    g_ledc_last_duty = 0;

    // Restart PWM
    cpPwmStart(CP_PWM_DUTY_5PCT, true);

    EXPECT_TRUE(cpPwmIsRunning());
    EXPECT_EQ(g_last_gpio_mode, GPIO_MODE_OUTPUT);
    EXPECT_EQ(g_gpio_mode_at_ledc_set_duty, GPIO_MODE_OUTPUT);
    EXPECT_EQ(g_ledc_last_duty, CP_PWM_DUTY_5PCT);
}

TEST(CpPwm, AcceptsDutyOutsideFivePercent) {
    cpPwmInit();

    // Set duty below the 5% handshake window without clamping
    uint16_t duty_low = CP_PWM_DUTY_5PCT / 2;
    cpPwmSetDuty(duty_low, false);
    EXPECT_EQ(g_ledc_last_duty, duty_low);
    EXPECT_EQ(cpGetLastPwmDuty(), duty_low);

    // Set duty above the handshake window without clamping
    uint16_t duty_high = CP_PWM_DUTY_5PCT * 2;
    cpPwmSetDuty(duty_high, false);
    EXPECT_EQ(g_ledc_last_duty, duty_high);
    EXPECT_EQ(cpGetLastPwmDuty(), duty_high);
}
