#include "cp_pwm.h"
#include "cp_monitor.h"
#include <soc/ledc_struct.h>

static bool pwmRunning = false;
static constexpr uint8_t PWM_CHANNEL = 0;

void cpPwmInit() {
    ledcSetup(PWM_CHANNEL, CP_PWM_FREQ_HZ, CP_PWM_RES_BITS);
    ledcAttachPin(CP_PWM_OUT_PIN, PWM_CHANNEL);
    LEDC.int_ena.lstimer0_ovf = 1;
    cpPwmStop();
}

void cpPwmStart(uint16_t duty_raw) {
#if CP_IDLE_RELEASE
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

void cpPwmStop()
{
    constexpr uint16_t DUTY_FULL = (1u << CP_PWM_RES_BITS) - 1; // 100% duty
    ledcWrite(PWM_CHANNEL, DUTY_FULL);   // drive CP to +12V rail
    cpSetLastPwmDuty(DUTY_FULL);         // reflect actual pin level
    pwmRunning = false;
}
