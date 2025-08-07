#include "cp_pwm.h"
#include "cp_monitor.h"
#include <esp_pm.h>
#include <driver/ledc.h>
#include <driver/gpio.h>

static bool pwmRunning = false;
static bool pinWasInput = false;
static constexpr uint8_t PWM_CHANNEL = 0;
static esp_pm_lock_handle_t cp_pm_lock = nullptr;

static uint16_t clamp_5pct(uint16_t duty_raw) {
    const uint16_t scale = (1u << CP_PWM_RES_BITS);
    const uint16_t lo = (scale * 9) / 200;  // 4.5 %
    const uint16_t hi = (scale * 11) / 200; // 5.5 %
    if (duty_raw < lo)
        return lo;
    if (duty_raw > hi)
        return hi;
    return duty_raw;
}

bool cpPwmIsRunning() {
    return pwmRunning;
}

void cpPwmInit() {
    if (!cp_pm_lock)
        esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "cp_pwm", &cp_pm_lock);
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = static_cast<ledc_timer_bit_t>(CP_PWM_RES_BITS),
        .timer_num = LEDC_TIMER_0,
        .freq_hz = CP_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .gpio_num = CP_PWM_OUT_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = static_cast<ledc_channel_t>(PWM_CHANNEL),
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ch_cfg);
    cpPwmStop();
}

void cpPwmStart(uint16_t duty_raw, bool clamp) {
#ifdef ESP_PLATFORM
    if (cp_pm_lock)
        esp_pm_lock_acquire(cp_pm_lock);
#endif
    const uint16_t max_duty = (1u << CP_PWM_RES_BITS) - 1;
    if (duty_raw > max_duty)
        duty_raw = max_duty;
    if (clamp)
        duty_raw = clamp_5pct(duty_raw);
    if (pinWasInput) {
        gpio_set_direction(static_cast<gpio_num_t>(CP_PWM_OUT_PIN), GPIO_MODE_OUTPUT);
        pinWasInput = false;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), duty_raw);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));
    cpSetLastPwmDuty(duty_raw);
    pwmRunning = true;
}

void cpPwmSetDuty(uint16_t duty_raw, bool clamp) {
    if (!pwmRunning)
        cpPwmStart(duty_raw, clamp);
    else {
        const uint16_t max_duty = (1u << CP_PWM_RES_BITS) - 1;
        if (duty_raw > max_duty)
            duty_raw = max_duty;
        if (clamp)
            duty_raw = clamp_5pct(duty_raw);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), duty_raw);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));
        cpSetLastPwmDuty(duty_raw);
    }
}

void cpPwmStop() {
    if (CP_IDLE_DRIVE_HIGH) {
        constexpr uint16_t DUTY_FULL = (1u << CP_PWM_RES_BITS) - 1;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), DUTY_FULL);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));
        cpSetLastPwmDuty(DUTY_FULL);
        pinWasInput = false;
    } else {
        ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), 0);
        cpSetLastPwmDuty(0);
        gpio_set_direction(static_cast<gpio_num_t>(CP_PWM_OUT_PIN), GPIO_MODE_INPUT);
        pinWasInput = true;
    }
    pwmRunning = false;
#ifdef ESP_PLATFORM
    if (cp_pm_lock)
        esp_pm_lock_release(cp_pm_lock);
#endif
}
