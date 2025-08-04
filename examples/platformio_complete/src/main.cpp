#include <port/esp32s3/port_config.hpp>
#include <slac/channel.hpp>
#include <slac/slac.hpp>
#include <port/esp32s3/qca7000_link.hpp>
#include <port/esp32s3/qca7000.hpp>
#include <atomic>
#include <cstdio>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "cp_pwm.h"
#include "cp_monitor.h"
#include "cp_state_machine.h"

// qca7000ProcessSlice is declared in qca7000.hpp with a default timeout.
// The extern declaration here must not specify the default again to avoid
// multiple default argument definitions.
extern void qca7000ProcessSlice(uint32_t max_us);

// Default MAC address for the modem. Adjust as required.
uint8_t g_mac_addr[ETH_ALEN] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
bool g_use_random_mac = false;
static const char* TAG = "MAIN";
static const uint8_t EVSE_NMK[slac::defs::NMK_LEN] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

static inline uint32_t get_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// Global pointer used by the polling loop
static slac::Channel* g_channel = nullptr;
volatile bool plc_irq = false;
std::atomic<uint8_t> g_slac_state{0};

void IRAM_ATTR plc_isr(void*) { plc_irq = true; }
// Timestamp for SLAC restart logic
std::atomic<uint32_t> g_slac_ts{0};
static bool hlc_running = false;

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

static void logTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000);
    while (true) {
        uint32_t mv = cpGetVoltageMv();
        printf("[STAT] CP=%c %lu.%03lu V Stage=%s SLAC=%u\n",
               cpGetStateLetter(),
               static_cast<unsigned long>(mv / 1000),
               static_cast<unsigned long>(mv % 1000),
               evseStageName(evseGetStage()),
               g_slac_state.load(std::memory_order_relaxed));
        vTaskDelay(period);
    }
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(4000));
    ESP_LOGI(TAG, "Starting SLAC modem...");
    check_serial_flag();

    gpio_config_t in_cfg{};
    in_cfg.mode = GPIO_MODE_INPUT;
    in_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    in_cfg.pin_bit_mask = (1ULL << LOCK_FB_PIN) |
                          (1ULL << CONTACTOR_FB_PIN) |
                          (1ULL << ISOLATION_OK_PIN);
    gpio_config(&in_cfg);

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
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    spi_device_handle_t spi_handle;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));

    qca7000_config cfg{spi_handle, PLC_SPI_CS_PIN, PLC_SPI_RST_PIN, g_mac_addr};
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

    gpio_config_t int_cfg{};
    int_cfg.pin_bit_mask = 1ULL << PLC_INT_PIN;
    int_cfg.mode = GPIO_MODE_INPUT;
    int_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    int_cfg.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&int_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(static_cast<gpio_num_t>(PLC_INT_PIN), plc_isr, nullptr);
    qca7000SetNmk(EVSE_NMK);

    cpPwmInit();
    cpMonitorInit();
    evseStateMachineInit();
    xTaskCreatePinnedToCore(evseStateMachineTask, "evseSM", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(logTask, "log", 4096, nullptr, 1, nullptr, 1);

    while (true) {
        if (plc_irq) {
            plc_irq = false;
            qca7000ProcessSlice(500);
        }

        slac::messages::HomeplugMessage msg;
        if (g_channel && g_channel->poll(msg)) {
            // Handle incoming SLAC messages here
        }

        if (cpPwmIsRunning() &&
            cpDigitalCommRequested() &&
            !qca7000CheckAlive()) {
            ESP_LOGI(TAG, "[PLC] link lost");
            hlc_stop();
            qca7000LeaveAvln();
            if (g_use_random_mac)
                qca7000SetMac(g_mac_addr);
            if (qca7000startSlac())
                g_slac_ts.store(get_ms(), std::memory_order_relaxed);
        }

        g_slac_state.store(qca7000getSlacResult(), std::memory_order_relaxed);
        if (!hlc_running && g_slac_state.load(std::memory_order_relaxed) == 6)
            hlc_start();
        if (hlc_running && g_slac_state.load(std::memory_order_relaxed) != 6)
            hlc_stop();
        if (g_slac_state.load(std::memory_order_relaxed) != 0 &&
            g_slac_state.load(std::memory_order_relaxed) != 5) {
            if (get_ms() - g_slac_ts.load(std::memory_order_relaxed) > 60000) {
                ESP_LOGI(TAG, "Restarting SLAC handshake");
                if (g_use_random_mac)
                    qca7000SetMac(g_mac_addr);
                if (!qca7000startSlac())
                    ESP_LOGI(TAG, "startSlac failed");
                g_slac_ts.store(get_ms(), std::memory_order_relaxed);
            }
        } else {
            g_slac_ts.store(get_ms(), std::memory_order_relaxed);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
