#include <cstring>
#include "driver/timer.h"
#include "esp_intr_alloc.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "arduino_stubs.hpp"

extern "C" {

// ----- Timer stubs -----
struct hw_timer_s {
    void (*isr)();
};
static hw_timer_s timer_storage[8];
void (*g_last_isr)(void) = nullptr;
uint64_t g_last_alarm_value = 0;

hw_timer_t* timerBegin(int timer, uint16_t, bool) {
    if (timer < 0 || timer >= 8) return nullptr;
    return &timer_storage[timer];
}
void timerAttachInterrupt(hw_timer_t* t, void (*fn)(), bool) {
    if (t) t->isr = fn;
    g_last_isr = fn;
}
void timerAlarmWrite(hw_timer_t*, uint64_t val, bool) { g_last_alarm_value = val; }
void timerAlarmEnable(hw_timer_t*) {}
void timerAlarmDisable(hw_timer_t*) {}
void timerWrite(hw_timer_t*, uint64_t) {}

// ----- LEDC ISR stub -----
ledc_dev_t LEDC{};
static intr_handle_t ledc_handle = nullptr;
static void (*ledc_cb)(void*) = nullptr;

esp_err_t ledc_isr_register(void (*fn)(void*), void* arg, int, intr_handle_t* handle) {
    ledc_cb = fn;
    ledc_handle = reinterpret_cast<intr_handle_t>(0x1);
    if (handle) *handle = ledc_handle;
    (void)arg;
    return ESP_OK;
}

esp_err_t esp_intr_free(intr_handle_t) { return ESP_OK; }

// ----- ADC one-shot stubs -----
int g_adc_raw_value = 0;

int adc_oneshot_io_to_channel(int, adc_unit_t* unit, adc_channel_t* chan) {
    if (unit) *unit = ADC_UNIT_1; if (chan) *chan = 0; return 0; }
int adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t*, adc_oneshot_unit_handle_t* out) { if (out) *out = (void*)1; return 0; }
int adc_oneshot_config_channel(adc_oneshot_unit_handle_t, adc_channel_t, const adc_oneshot_chan_cfg_t*) { return 0; }
int adc_oneshot_read(adc_oneshot_unit_handle_t, adc_channel_t, int* out) { if (out) *out = g_adc_raw_value; return 0; }

// ----- ADC continuous stubs -----
uint8_t* g_dma_read_data = nullptr;
uint32_t g_dma_read_len = 0;

int adc_continuous_new_handle(const adc_continuous_handle_cfg_t*, adc_continuous_handle_t* out) { if (out) *out = (void*)1; return 0; }
int adc_continuous_config(adc_continuous_handle_t, const adc_continuous_config_t*) { return 0; }
int adc_continuous_start(adc_continuous_handle_t) { return 0; }
int adc_continuous_stop(adc_continuous_handle_t) { return 0; }
int adc_continuous_deinit(adc_continuous_handle_t) { return 0; }
int adc_continuous_read(adc_continuous_handle_t, uint8_t* buf, uint32_t len, uint32_t* outlen, int) {
    if (!g_dma_read_data) { if (outlen) *outlen = 0; return 0; }
    uint32_t copy = g_dma_read_len;
    if (copy > len) copy = len;
    memcpy(buf, g_dma_read_data, copy);
    if (outlen) *outlen = copy;
    return 0;
}

} // extern "C"

int g_mock_adc_mv = 0;

