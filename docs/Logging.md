# Logging and Control Pilot Monitoring

This document describes how the example firmware reports Control Pilot (CP) metrics such as PWM duty cycle, voltage state, and SLAC status. It outlines how data flows from the hardware sampling tasks to the periodic log output and covers normal operation as well as edge cases.

## Overview

The example firmware starts three key subsystems:

* **CP PWM driver** – drives the control pilot output using the ESP32 LEDC peripheral.
* **CP monitor** – samples the CP and output voltages using the ADC in DMA mode.
* **Logging task** – prints a status line once per second with CP, EVSE, and SLAC information.

These components communicate via atomic variables so the logging task can read the latest measurements without blocking the time‑critical ADC loop.

## Periodic Log Output

A dedicated FreeRTOS task `logTask` invokes `logStatus()` every second:

```cpp
static void logTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000);
    while (true) {
        logStatus();
        vTaskDelay(period);
    }
}
```

`logStatus()` gathers live measurements and prints a formatted line using `ESP_LOGI`:

```cpp
void logStatus() {
    uint32_t cp_mv = cpGetVoltageMv();
    uint16_t duty_raw = cpGetLastPwmDuty();
    float duty_pct =
        100.0f * static_cast<float>(duty_raw) /
        static_cast<float>((1u << CP_PWM_RES_BITS) - 1u);
    uint32_t vout_mv = voutGetVoltageMv();
    uint16_t vout_raw = voutGetVoltageRaw();
    const uint8_t* ev_mac = qca7000GetMatchedMac();
    ESP_LOGI(TAG,
             "[STAT] CP=%c %lu.%03lu V Duty=%.1f%% Vout=%lu.%03lu V Raw=%u Stage=%s SLAC=%u EV=%02X:%02X:%02X:%02X:%02X:%02X",
             cpGetStateLetter(),
             static_cast<unsigned long>(cp_mv / 1000),
             static_cast<unsigned long>(cp_mv % 1000),
             duty_pct,
             static_cast<unsigned long>(vout_mv / 1000),
             static_cast<unsigned long>(vout_mv % 1000),
             static_cast<unsigned>(vout_raw),
             evseStageName(evseGetStage()),
             static_cast<unsigned>(g_slac_state.load(std::memory_order_relaxed)),
             ev_mac[0], ev_mac[1], ev_mac[2], ev_mac[3], ev_mac[4], ev_mac[5]);
}
```

The unit test `LogTask.PrintsDutyAndVout` confirms that this output includes the PWM duty cycle and output voltage in volts:

```cpp
TEST(LogTask, PrintsDutyAndVout) {
    g_slac_state_main.store(SlacState::Matched, std::memory_order_relaxed);
    testing::internal::CaptureStdout();
    logStatus();
    std::string out = testing::internal::GetCapturedStdout();
    EXPECT_NE(out.find("Duty=50.0%"), std::string::npos);
    EXPECT_NE(out.find("Vout=5.000 V"), std::string::npos);
}
```

## Control Pilot PWM

The CP PWM driver configures the LEDC timer and channel during `cpPwmInit()` and keeps track of the last duty cycle. When starting or updating the PWM, the duty value is clamped to the hardware resolution and optionally forced into the 4.5–5.5 % handshake window:

```cpp
void cpPwmStart(uint16_t duty_raw, bool clamp) {
    const uint16_t max_duty = (1u << CP_PWM_RES_BITS) - 1;
    if (duty_raw > max_duty)
        duty_raw = max_duty;
    if (clamp)
        duty_raw = clamp_5pct(duty_raw);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), duty_raw);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));
    cpSetLastPwmDuty(duty_raw);
    pwmRunning = true;
}
```

Stopping the PWM either drives the line high (when `CP_IDLE_DRIVE_HIGH` is set) or releases it as an input:

```cpp
void cpPwmStop() {
    if (CP_IDLE_DRIVE_HIGH) {
        constexpr uint16_t DUTY_FULL = (1u << CP_PWM_RES_BITS) - 1;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), DUTY_FULL);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL));
        cpSetLastPwmDuty(DUTY_FULL);
    } else {
        ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(PWM_CHANNEL), 0);
        cpSetLastPwmDuty(0);
        gpio_set_direction(static_cast<gpio_num_t>(CP_PWM_OUT_PIN), GPIO_MODE_INPUT);
    }
    pwmRunning = false;
}
```

The current duty value is stored in an atomic variable so `logStatus()` can report it without interfering with PWM operation:

```cpp
void cpSetLastPwmDuty(uint16_t duty) {
    cp_duty.store(duty, std::memory_order_relaxed);
}
uint16_t cpGetLastPwmDuty() {
    return cp_duty.load(std::memory_order_relaxed);
}
```

## Control Pilot Monitoring

The monitoring task continuously captures ADC samples for the CP and vehicle output voltages. It computes peak, minimum, and duty cycle values for each PWM period. If the ADC times out, the driver restarts the DMA engine:

```cpp
static void restart_adc() {
    if (!adc_handle)
        return;
    adc_continuous_stop(adc_handle);
    adc_continuous_start(adc_handle);
}

static void process_samples() {
    int rc = adc_continuous_read(adc_handle, buf, sizeof(buf), &len, 1000);
    if (rc == ESP_ERR_TIMEOUT) {
        restart_adc();
        return;
    }
    ...
    cp_mv.store(mv_max, std::memory_order_relaxed);
    cp_mv_min.store(mv_min, std::memory_order_relaxed);
    if (cp_tot_cnt > 0) {
        uint16_t duty = static_cast<uint16_t>((cp_low_cnt << CP_PWM_RES_BITS) / cp_tot_cnt);
        cp_meas_duty.store(duty, std::memory_order_relaxed);
    }
    CpSubState ns = mv2state(mv_max, mv_min);
    ...
}
```

The monitor can be halted cleanly via `cpMonitorStop()`, which stops and deinitializes the ADC and terminates the background task:

```cpp
void cpMonitorStop() {
    if (adc_handle) {
        adc_continuous_stop(adc_handle);
        adc_continuous_deinit(adc_handle);
        adc_handle = nullptr;
    }
}
```

## Edge Cases and Behaviour

* **ADC timeouts** – `process_samples()` restarts the DMA engine when `adc_continuous_read` returns `ESP_ERR_TIMEOUT`, ensuring that temporary stalls do not block logging.
* **Duty cycle limits** – `cpPwmStart()` and `cpPwmSetDuty()` clamp the requested duty to the valid range. When `clamp` is true, the value is further constrained to 4.5–5.5 % for the ISO15118 handshake.
* **Stopping PWM** – `cpPwmStop()` either drives the line high or releases it as input, depending on `CP_IDLE_DRIVE_HIGH`. The last duty value is updated so logs reflect the new state.
* **Log task lifetime** – `logTask` runs indefinitely. It relies on `logStatus()` to fetch the latest values; if subsystems are disabled, the log will show default or stale readings but the task continues to execute.

## Summary

Every second the firmware prints a consolidated status line containing CP voltage, measured duty cycle, output voltage, EVSE stage, SLAC state, and the matched EV MAC address. The CP monitor and PWM driver maintain shared atomic state so the logger can operate asynchronously. Robust handling of ADC timeouts and duty‑cycle clamping ensures meaningful logs even under error conditions.

