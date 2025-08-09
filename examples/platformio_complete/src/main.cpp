#include <slac/channel.hpp>
#include <slac/config.hpp>
#include <slac/slac.hpp>
#include <slac/slac_states.hpp>
#include <slac/iso15118_consts.hpp>
#include <esp32s3/qca7000_link.hpp>
#include <atomic>
#include <cstdio>
#include <esp_log.h>
#include <esp_timer.h>
#ifndef LIBSLAC_TESTING
#include <esp_system.h>
#include <esp_random.h>
#include <driver/gpio.h>
#ifdef USE_UART_CONSOLE
#include <driver/uart.h>
#endif
#include <driver/spi_master.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "plc_irq.hpp"
#include "cp_config.h"

#ifndef LIBSLAC_TESTING
// Placeholder stubs for control pilot and EVSE state machine logic that would
// normally interact with hardware. These are kept minimal so the example can
// be built without the full hardware drivers.
inline void cpPwmInit() {}
inline void cpMonitorInit() {}
inline bool cpPwmIsRunning() { return false; }
inline bool cpDigitalCommRequested() { return false; }
inline uint32_t cpGetVoltageMv() { return 0; }
inline uint16_t cpGetLastPwmDuty() { return 0; }
inline uint16_t voutGetVoltageMv() { return 0; }
inline uint16_t voutGetVoltageRaw() { return 0; }
inline char cpGetStateLetter() { return 'A'; }
inline void evseStateMachineInit() {}
inline void evseStateMachineTask(void*) {}
inline const char* evseStageName(int) { return ""; }
inline int evseGetStage() { return 0; }
#endif

static const char* TAG = "MAIN";
std::atomic<SlacState> g_slac_state{SlacState::Idle};
static TaskHandle_t qca7000_irq_task_handle = nullptr;

#ifndef LIBSLAC_TESTING
// qca7000ProcessSlice is declared in qca7000.hpp with a default timeout.
// The extern declaration here must not specify the default again to avoid
// multiple default argument definitions.
extern void qca7000ProcessSlice(uint32_t max_us);

// Default MAC address for the modem. Adjust as required.
uint8_t g_mac_addr[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
bool g_use_random_mac = false;

static inline uint32_t get_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// Global pointer used by the polling loop
static slac::Channel* g_channel = nullptr;
// Timestamp for SLAC restart logic
std::atomic<uint32_t> g_slac_ts{0};
std::atomic<uint32_t> g_slac_init_ts{0};
std::atomic<bool> g_waiting_for_parm_req{false};
static bool hlc_running = false;
static bool session_active = false;

static void hlc_start() {
    ESP_LOGI(TAG, "[HLC] start");
    hlc_running = true;
}

static void hlc_stop() {
    if (hlc_running) {
        ESP_LOGI(TAG, "[HLC] stop");
        hlc_running = false;
    }
}

static void generate_random_mac() {
#ifdef RANDOM_MAC
    g_use_random_mac = true;
#endif
    if (!g_use_random_mac)
        return;
    g_mac_addr[0] = 0x02; // locally administered, unicast
    for (int i = 1; i < ETH_ALEN; ++i)
        g_mac_addr[i] = static_cast<uint8_t>(esp_random() & 0xFF);
}
#ifdef USE_UART_CONSOLE
static void check_serial_flag() {
    printf("Press 'R' for random MAC\n");
    int64_t start = esp_timer_get_time();
    uint8_t data;
    while ((esp_timer_get_time() - start) < 3000000) {
        int len = uart_read_bytes(UART_NUM_0, &data, 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (data == 'R' || data == 'r')
                g_use_random_mac = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    generate_random_mac();
}
#endif
#endif

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

static void logTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000);
    while (true) {
        logStatus();
        vTaskDelay(period);
    }
}

static void qca7000_irq_task(void*) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        do {
            qca7000ProcessSlice(500);
        } while (qca7000ReadInternalReg(SPI_REG_RDBUF_BYTE_AVA) > 0);
    }
}

#ifndef LIBSLAC_TESTING
extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(4000));
    ESP_LOGI(TAG, "Starting SLAC modem...");
#ifdef USE_UART_CONSOLE
    check_serial_flag();
#else
    generate_random_mac();
#endif

    slac::set_spi_fast_hz(QCA7000_SPI_FAST_HZ);
    slac::set_spi_slow_hz(PLC_SPI_SLOW_HZ);
    slac::set_spi_burst_len(QCA7000_SPI_BURST_LEN);
    slac::set_hardreset_low_ms(QCA7000_HARDRESET_LOW_MS);
    slac::set_hardreset_high_ms(QCA7000_HARDRESET_HIGH_MS);
    slac::set_cpuon_timeout_ms(QCA7000_CPUON_TIMEOUT_MS);
    slac::set_max_retries(QCA7000_MAX_RETRIES);



    spi_bus_config_t buscfg{};
    buscfg.mosi_io_num = PLC_SPI_MOSI_PIN;
    buscfg.miso_io_num = PLC_SPI_MISO_PIN;
    buscfg.sclk_io_num = PLC_SPI_SCK_PIN;
    buscfg.max_transfer_sz = 1600;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg{};
    devcfg.clock_speed_hz = PLC_SPI_SLOW_HZ;
    devcfg.mode = 3;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 3;
    spi_device_handle_t spi_handle;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));

    qca7000_config cfg{spi_handle, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN,
                       PLC_INT_PIN, PLC_PWR_EN_PIN, g_mac_addr};
    static slac::port::Qca7000Link link(cfg);
    link.set_error_callback([](Qca7000ErrorStatus, void*) {
        ESP_LOGI(TAG, "[PLC] Fatal error - driver auto-reset");
    }, nullptr);
    static slac::Channel channel(&link);
    g_channel = &channel;
    ESP_LOGI(TAG, "Starting SLAC channel");
    if (!channel.open()) {
        ESP_LOGE(TAG, "Failed to open SLAC channel, aborting");
        g_channel = nullptr;
        while (true)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (PLC_INT_PIN >= 0) {
        xTaskCreatePinnedToCore(qca7000_irq_task,
                                "qca7000_irq",
                                4096,
                                nullptr,
                                configMAX_PRIORITIES - 1,
                                &qca7000_irq_task_handle,
                                1);
        plc_irq_setup(qca7000_irq_task_handle);
    } else {
        ESP_LOGE(TAG, "Invalid PLC_INT_PIN %d; skipping IRQ setup", PLC_INT_PIN);
    }
    // Start with the default NMK to match the PEV
    qca7000SetNmk(nullptr);

    cpPwmInit();
    cpMonitorInit();
    evseStateMachineInit();
    xTaskCreatePinnedToCore(evseStateMachineTask, "evseSM", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(logTask, "log", 4096, nullptr, 1, nullptr, 1);

    while (true) {
        slac::messages::HomeplugMessage msg;
        if (g_channel && g_channel->poll(msg)) {
            switch (msg.get_mmtype()) {
            case slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_REQ:
                qca7000HandleSlacParmReq(msg);
                g_waiting_for_parm_req.store(false, std::memory_order_relaxed);
                break;
            case slac::defs::MMTYPE_CM_SLAC_PARAM | slac::defs::MMTYPE_MODE_CNF:
                qca7000HandleSlacParmCnf(msg);
                break;
            case slac::defs::MMTYPE_CM_START_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND:
                qca7000HandleStartAttenCharInd(msg);
                break;
            case slac::defs::MMTYPE_CM_ATTEN_PROFILE | slac::defs::MMTYPE_MODE_IND:
                qca7000HandleAttenProfileInd(msg);
                break;
            case slac::defs::MMTYPE_CM_ATTEN_CHAR | slac::defs::MMTYPE_MODE_IND:
                qca7000HandleAttenCharInd(msg);
                break;
            case slac::defs::MMTYPE_CM_SET_KEY | slac::defs::MMTYPE_MODE_REQ:
                qca7000HandleSetKeyReq(msg);
                break;
            case slac::defs::MMTYPE_CM_VALIDATE | slac::defs::MMTYPE_MODE_REQ:
                qca7000HandleValidateReq(msg);
                break;
            case slac::defs::MMTYPE_CM_SLAC_MATCH | slac::defs::MMTYPE_MODE_REQ:
                qca7000HandleSlacMatchReq(msg);
                break;
            default:
                ESP_LOGW(TAG, "[PLC] Unhandled mmtype 0x%04X", msg.get_mmtype());
                break;
            }
            g_slac_state.store(qca7000getSlacResult(), std::memory_order_relaxed);
        }

        if (cpPwmIsRunning() &&
            cpDigitalCommRequested() &&
            !qca7000CheckAlive()) {
            ESP_LOGI(TAG, "[PLC] link lost");
            hlc_stop();
            qca7000LeaveAvln();
            if (g_use_random_mac)
                qca7000SetMac(g_mac_addr);
            if (qca7000startSlac()) {
                auto now = get_ms();
                g_slac_ts.store(now, std::memory_order_relaxed);
                g_slac_init_ts.store(now, std::memory_order_relaxed);
                g_waiting_for_parm_req.store(true, std::memory_order_relaxed);
            }
        }

        g_slac_state.store(qca7000getSlacResult(), std::memory_order_relaxed);
        if (!session_active &&
            g_slac_state.load(std::memory_order_relaxed) == SlacState::Matched) {
            session_active = true;
        }
        if (session_active && cpGetStateLetter() == 'A') {
            qca7000SetNmk(nullptr);
            session_active = false;
        }
        if (!hlc_running &&
            g_slac_state.load(std::memory_order_relaxed) == SlacState::Matched)
            hlc_start();
        if (hlc_running &&
            g_slac_state.load(std::memory_order_relaxed) != SlacState::Matched)
            hlc_stop();
        if (g_slac_state.load(std::memory_order_relaxed) != SlacState::Idle &&
            g_slac_state.load(std::memory_order_relaxed) != SlacState::WaitMatch) {
            if (get_ms() - g_slac_ts.load(std::memory_order_relaxed) > 60000) {
                ESP_LOGI(TAG, "Restarting SLAC handshake");
                if (g_use_random_mac)
                    qca7000SetMac(g_mac_addr);
                if (!qca7000startSlac())
                    ESP_LOGI(TAG, "startSlac failed");
                auto now = get_ms();
                g_slac_ts.store(now, std::memory_order_relaxed);
                g_slac_init_ts.store(now, std::memory_order_relaxed);
                g_waiting_for_parm_req.store(true, std::memory_order_relaxed);
            }
        } else {
            g_slac_ts.store(get_ms(), std::memory_order_relaxed);
        }

        if (g_waiting_for_parm_req.load(std::memory_order_relaxed) &&
            get_ms() - g_slac_init_ts.load(std::memory_order_relaxed) >
                slac::defs::TT_EVSE_SLAC_INIT_MS) {
            ESP_LOGI(TAG, "Timeout waiting for CM_SLAC_PARM.REQ");
            hlc_stop();
            qca7000LeaveAvln();
            g_slac_state.store(SlacState::Idle, std::memory_order_relaxed);
            g_waiting_for_parm_req.store(false, std::memory_order_relaxed);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif
