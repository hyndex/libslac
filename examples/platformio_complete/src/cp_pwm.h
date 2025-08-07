#pragma once
#include <stdint.h>
#include "cp_config.h"

void cpPwmInit();
// Start CP PWM output. If `clamp_5pct` is true the duty cycle will be
// constrained to the 4.5–5.5%% window used during the PWM handshake.
// Otherwise the raw duty value is used allowing the full 0–100%% range.
void cpPwmStart(uint16_t duty_raw = CP_PWM_DUTY_5PCT, bool clamp_5pct = false);
void cpPwmStop();
// Update the CP PWM duty cycle. Behaviour is identical to cpPwmStart with
// respect to the `clamp_5pct` flag.
void cpPwmSetDuty(uint16_t duty_raw, bool clamp_5pct = false);
bool cpPwmIsRunning();
