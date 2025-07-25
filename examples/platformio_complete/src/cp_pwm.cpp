#include "cp_pwm.h"
#include "cp_monitor.h"
#include <atomic>

static std::atomic<bool> pwmRunning{false};
static constexpr uint8_t PWM_CHANNEL = 0;

void cpPwmInit() {
    ledcSetup(PWM_CHANNEL, CP_PWM_FREQ_HZ, CP_PWM_RES_BITS);
    ledcAttachPin(CP_PWM_OUT_PIN, PWM_CHANNEL);
    cpPwmStop();
}

void cpPwmStart(uint16_t duty_raw) {
    ledcWrite(PWM_CHANNEL, duty_raw);
    cpSetLastPwmDuty(duty_raw);
    pwmRunning.store(true, std::memory_order_relaxed);
}

void cpPwmSetDuty(uint16_t duty_raw) {
    if (!pwmRunning.load(std::memory_order_relaxed))
        cpPwmStart(duty_raw);
    else {
        ledcWrite(PWM_CHANNEL, duty_raw);
        cpSetLastPwmDuty(duty_raw);
    }
}

void cpPwmStop() {
    ledcWrite(PWM_CHANNEL, 0);
    cpSetLastPwmDuty(0);
    pwmRunning.store(false, std::memory_order_relaxed);
}
