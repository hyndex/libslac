#pragma once
#include <stdint.h>

#define CP_PWM_OUT_PIN      38
#define CP_READ_ADC_PIN     1
#define LOCK_FB_PIN         10
#define CONTACTOR_FB_PIN    13
#define VOUT_MON_ADC_PIN    9
#define ISOLATION_OK_PIN    14

#define CP_THR_12V_MV   2350
#define CP_THR_9V_MV    1950
#define CP_THR_6V_MV    1650
#define CP_THR_3V_MV    1350
#define CP_THR_1V_MV    200          // ≈ 1 V after divider

// threshold for the negative plateau (-12 V -> state E/F)
#define CP_THR_NEG12      900
// hysteresis for negative detection (in mV)
#define CP_THR_NEG12_HYS  30
#define CP_THR_NEG12_LOW  (CP_THR_NEG12 - CP_THR_NEG12_HYS)
#define CP_THR_NEG12_HIGH (CP_THR_NEG12 + CP_THR_NEG12_HYS)

#define CP_PWM_FREQ_HZ      1000
#define CP_PWM_RES_BITS     12
#define CP_PWM_DUTY_5PCT    ((1u << CP_PWM_RES_BITS) / 20)
#define CP_IDLE_DRIVE_HIGH  1   // 1=keep CP driven high when idle, 0=release
#define CP_SAMPLE_OFFSET_US \
    ((1000000 / CP_PWM_FREQ_HZ) * CP_PWM_DUTY_5PCT / (1u << CP_PWM_RES_BITS) / 2)

#define T_PLC_INIT_MS       700
#define T_HLC_EST_MS        2000
#define T_ISO_CPLT_MS       5000
#define T_PC_DONE_MS        10000
#define T_STOP_MAX_MS       5000
#define T_SAFE_MAX_MS       2000
#define T_CP_B1_FAIL_MS     2000

inline uint32_t ms2ticks(uint32_t ms) { return ms / portTICK_PERIOD_MS; }
