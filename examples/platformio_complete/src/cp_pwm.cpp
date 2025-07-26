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
#if CP_IDLE_RELEASE
    /* Hi-Z – rely on pull-up resistor to keep CP at +12 V */
    ledcDetachPin(CP_PWM_OUT_PIN);
    pinMode(CP_PWM_OUT_PIN, INPUT);
#else
    /* Keep the output driven HIGH, but mark that no PWM is active */
    constexpr uint16_t DUTY_FULL = (1u << CP_PWM_RES_BITS) - 1; // 100% duty
    ledcWrite(PWM_CHANNEL, DUTY_FULL);
#endif
    /* Tell the monitor that the PWM is off */
    cpSetLastPwmDuty(0);
    pwmRunning = false;
}
