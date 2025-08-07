#include <gtest/gtest.h>
#define ARDUINO
#include "Arduino.h"
#include "../examples/platformio_complete/src/cp_monitor.h"

// Wrappers exposed for testing
CpSubState mv2stateTest(uint16_t mv_max, uint16_t mv_min);
void cpSetMeasuredDuty(uint16_t duty);
void cpSetPrevState(CpSubState s);

TEST(Mv2State, MapsVoltagesToStates) {
    // CP_A when voltage > 12V
    cpSetPrevState(CP_B1);
    cpSetMeasuredDuty(0);
    EXPECT_EQ(mv2stateTest(CP_THR_12V_MV + 1, 0), CP_A);

    // CP_B1 when >9V and duty 0
    cpSetPrevState(CP_A);
    cpSetMeasuredDuty(0);
    EXPECT_EQ(mv2stateTest(CP_THR_9V_MV + 1, 0), CP_B1);

    // CP_B3 when >9V and duty >0
    cpSetMeasuredDuty(1);
    EXPECT_EQ(mv2stateTest(CP_THR_9V_MV + 1, 0), CP_B3);

    // CP_B2 for ~5%% duty between 6V and 9V
    uint16_t duty5pct = static_cast<uint16_t>((5u << CP_PWM_RES_BITS) / 100u);
    cpSetMeasuredDuty(duty5pct);
    EXPECT_EQ(mv2stateTest(CP_THR_6V_MV + 1, 0), CP_B2);

    // CP_C for other duty in same voltage range
    cpSetMeasuredDuty(0);
    EXPECT_EQ(mv2stateTest(CP_THR_6V_MV + 1, 0), CP_C);

    // CP_D for 3V-6V
    EXPECT_EQ(mv2stateTest(CP_THR_3V_MV + 1, 0), CP_D);

    // CP_E when below 3V but above negative threshold
    cpSetPrevState(CP_A);
    EXPECT_EQ(mv2stateTest(CP_THR_3V_MV - 1, CP_THR_NEG12_HIGH + 1), CP_E);

    // Hysteresis: stay in CP_E with relaxed threshold
    cpSetPrevState(CP_E);
    EXPECT_EQ(mv2stateTest(CP_THR_3V_MV - 1, CP_THR_NEG12_LOW + 1), CP_E);

    // CP_F when <1V
    EXPECT_EQ(mv2stateTest(CP_THR_1V_MV - 1, 0), CP_F);
}
