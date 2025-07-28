#include "cp_pwm.h"
#include "cp_monitor.h"
#include <soc/ledc_struct.h>

static bool pwmRunning = false;
static constexpr uint8_t PWM_CHANNEL = 0;

bool cpPwmIsRunning() { return pwmRunning; }

void cpPwmInit() {
    ledcSetup(PWM_CHANNEL, CP_PWM_FREQ_HZ, CP_PWM_RES_BITS);
    ledcAttachPin(CP_PWM_OUT_PIN, PWM_CHANNEL);
    cpPwmStop();
}

void cpPwmStart(uint16_t duty_raw) {
#if !CP_IDLE_DRIVE_HIGH
    ledcAttachPin(CP_PWM_OUT_PIN, PWM_CHANNEL);
#endif
    const uint16_t max_duty = (1u << CP_PWM_RES_BITS) - 1;
    if (duty_raw > max_duty)
        duty_raw = max_duty;
    ledcWrite(PWM_CHANNEL, duty_raw);
    cpSetLastPwmDuty(duty_raw);
    pwmRunning = true;
#if CP_USE_DMA_ADC
    cpDmaStart();
#else
    cpFastSampleStart();
#endif
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
#if CP_IDLE_DRIVE_HIGH
    constexpr uint16_t DUTY_FULL = (1u << CP_PWM_RES_BITS) - 1;
    ledcWrite(PWM_CHANNEL, DUTY_FULL);
    cpSetLastPwmDuty(DUTY_FULL);
#else
    ledcWrite(PWM_CHANNEL, 0);
    cpSetLastPwmDuty(0);
    ledcDetachPin(CP_PWM_OUT_PIN);
    pinMode(CP_PWM_OUT_PIN, INPUT);
#endif
#if CP_USE_DMA_ADC
    cpDmaStop();
#else
    cpFastSampleStop();
#endif
    pwmRunning = false;
}
