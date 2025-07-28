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
    LEDC.int_ena.lstimer0_ovf = 1;
#if CP_USE_DMA_ADC
    cpDmaStart();
#else
    cpFastSampleStart();
#endif
}

void cpPwmSetDuty(uint16_t duty_raw) {
    const uint16_t max_duty = (1u << CP_PWM_RES_BITS) - 1;
    if (duty_raw > max_duty)
        duty_raw = max_duty;

    if (!pwmRunning) {
        cpPwmStart(duty_raw);
        return;
    }

    uint16_t current = cpGetLastPwmDuty();
    const uint16_t step = (1u << CP_PWM_RES_BITS) / 20; // 5%
    while (current != duty_raw) {
        if (duty_raw > current) {
            uint16_t next = current + step;
            if (next > duty_raw)
                next = duty_raw;
            current = next;
        } else {
            uint16_t next = (current > step) ? current - step : 0;
            if (next < duty_raw)
                next = duty_raw;
            current = next;
        }
        ledcWrite(PWM_CHANNEL, current);
        cpSetLastPwmDuty(current);
        delay(1);
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
    pwmRunning = false;
#if CP_USE_DMA_ADC
    cpDmaStop();
#else
    cpFastSampleStop();
#endif
    LEDC.int_ena.lstimer0_ovf = 0;
}
