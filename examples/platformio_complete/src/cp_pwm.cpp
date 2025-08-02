#include "cp_pwm.h"
#include "cp_monitor.h"
#include <esp_pm.h>
#include <soc/ledc_struct.h>

static bool pwmRunning = false;
static constexpr uint8_t PWM_CHANNEL = 0;
static esp_pm_lock_handle_t cp_pm_lock = nullptr;

bool cpPwmIsRunning() {
    return pwmRunning;
}

void cpPwmInit() {
    if (!cp_pm_lock)
        esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "cp_pwm", &cp_pm_lock);
    ledcSetup(PWM_CHANNEL, CP_PWM_FREQ_HZ, CP_PWM_RES_BITS);
    ledcAttachPin(CP_PWM_OUT_PIN, PWM_CHANNEL);
    cpPwmStop();
}

void cpPwmStart(uint16_t duty_raw) {
#ifdef ESP_PLATFORM
    if (cp_pm_lock)
        esp_pm_lock_acquire(cp_pm_lock);
#endif
#if !CP_IDLE_DRIVE_HIGH
    ledcAttachPin(CP_PWM_OUT_PIN, PWM_CHANNEL);
#endif
    const uint16_t max_duty = (1u << CP_PWM_RES_BITS) - 1;
    if (duty_raw > max_duty)
        duty_raw = max_duty;
    ledcWrite(PWM_CHANNEL, duty_raw);
    cpSetLastPwmDuty(duty_raw);
    pwmRunning = true;
}

void cpPwmSetDuty(uint16_t duty_raw) {
    if (!pwmRunning)
        cpPwmStart(duty_raw);
    else {
        const uint16_t max_duty = (1u << CP_PWM_RES_BITS) - 1;
        if (duty_raw > max_duty)
            duty_raw = max_duty;
        ledcWrite(PWM_CHANNEL, duty_raw);
        cpSetLastPwmDuty(duty_raw);
    }
}

void cpPwmStop() {
    if (CP_IDLE_DRIVE_HIGH) {
        constexpr uint16_t DUTY_FULL = (1u << CP_PWM_RES_BITS) - 1;
        ledcWrite(PWM_CHANNEL, DUTY_FULL);
        cpSetLastPwmDuty(DUTY_FULL);
    } else {
        ledcWrite(PWM_CHANNEL, 0);
        cpSetLastPwmDuty(0);
        ledcDetachPin(CP_PWM_OUT_PIN);
        pinMode(CP_PWM_OUT_PIN, INPUT);
    }
    pwmRunning = false;
#ifdef ESP_PLATFORM
    if (cp_pm_lock)
        esp_pm_lock_release(cp_pm_lock);
#endif
}
