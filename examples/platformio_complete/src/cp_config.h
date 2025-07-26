#pragma once
#include <stdint.h>

#define CP_PWM_OUT_PIN      38
#define CP_READ_ADC_PIN     1
#define LOCK_FB_PIN         10
#define CONTACTOR_FB_PIN    13
#define VOUT_MON_ADC_PIN    9
#define ISOLATION_OK_PIN    14

#define CP_THR_12V      2350
#define CP_THR_9V       1950
#define CP_THR_6V       1650
#define CP_THR_3V       1400
#define CP_THR_1V       1100

#define CP_PWM_FREQ_HZ      1000
#define CP_PWM_RES_BITS     12
#define CP_PWM_DUTY_5PCT    ((1 << CP_PWM_RES_BITS) * 5 / 100)
#define CP_IDLE_RELEASE   0   // actively drive CP high when PWM is idle

#define T_PLC_INIT_MS       700
#define T_HLC_EST_MS        2000
#define T_ISO_CPLT_MS       5000
#define T_PC_DONE_MS        10000
#define T_STOP_MAX_MS       5000
#define T_SAFE_MAX_MS       2000
#define T_CP_B1_FAIL_MS     2000

inline uint32_t ms2ticks(uint32_t ms) { return ms / portTICK_PERIOD_MS; }
