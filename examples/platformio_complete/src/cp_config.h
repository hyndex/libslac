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
#define CP_THR_3V_MV    1400
#define CP_THR_1V_MV    1100

// additional thresholds for detecting negative plateaus
#define CP_THR_NEG_E_MV   500
#define CP_THR_NEG_F_MV   300

#define CP_PWM_FREQ_HZ      1000
#define CP_PWM_RES_BITS     12
#define CP_PWM_DUTY_5PCT    (((1 << CP_PWM_RES_BITS) * 5 + 50) / 100) // round correctly
#define CP_IDLE_RELEASE     1   // 0=keep pin attached and drive high when idle, 1=release
#define CP_SAMPLE_OFFSET_US 25

#define T_PLC_INIT_MS       700
#define T_HLC_EST_MS        2000
#define T_ISO_CPLT_MS       5000
#define T_PC_DONE_MS        10000
#define T_STOP_MAX_MS       5000
#define T_SAFE_MAX_MS       2000
#define T_CP_B1_FAIL_MS     2000

inline uint32_t ms2ticks(uint32_t ms) { return ms / portTICK_PERIOD_MS; }
