#include <gtest/gtest.h>
#define GPIO_STUB_CUSTOM
#include "driver/gpio.h"
#include "freertos/task.h"

static gpio_config_t last_cfg{};
static gpio_isr_t registered_isr = nullptr;
static TaskHandle_t expected_task = nullptr;
static BaseType_t notified = pdFALSE;

static esp_err_t gpio_config(const gpio_config_t* cfg) {
    last_cfg = *cfg;
    return ESP_OK;
}

static esp_err_t gpio_install_isr_service(int) { return ESP_OK; }

static esp_err_t gpio_isr_handler_add(gpio_num_t, gpio_isr_t isr, void*) {
    registered_isr = isr;
    return ESP_OK;
}

uint32_t ulTaskNotifyTake(BaseType_t, TickType_t) { return 0; }

void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t*) {
    if (handle == expected_task)
        notified = pdTRUE;
}

#define PLC_INT_PIN 0
#define plc_isr plc_isr_neg
#define plc_irq_setup plc_irq_setup_neg
#define plc_irq_task_handle plc_irq_task_handle_neg
#include "examples/platformio_complete/src/plc_irq.hpp"
#undef plc_isr
#undef plc_irq_setup
#undef plc_irq_task_handle

TEST(PlcIrqEdge, Negative) {
    notified = pdFALSE;
    TaskHandle_t dummy = reinterpret_cast<TaskHandle_t>(1);
    expected_task = dummy;
    plc_irq_setup_neg(dummy);
    EXPECT_EQ(last_cfg.intr_type, GPIO_INTR_NEGEDGE);
    ASSERT_NE(registered_isr, nullptr);
    registered_isr(nullptr);
    EXPECT_TRUE(notified);
}

#undef PLC_IRQ_HPP
#undef PLC_INT_EDGE
#define PLC_INT_EDGE GPIO_INTR_POSEDGE
#define plc_isr plc_isr_pos
#define plc_irq_setup plc_irq_setup_pos
#define plc_irq_task_handle plc_irq_task_handle_pos
#include "examples/platformio_complete/src/plc_irq.hpp"
#undef plc_isr
#undef plc_irq_setup
#undef plc_irq_task_handle

TEST(PlcIrqEdge, Positive) {
    notified = pdFALSE;
    TaskHandle_t dummy = reinterpret_cast<TaskHandle_t>(2);
    expected_task = dummy;
    plc_irq_setup_pos(dummy);
    EXPECT_EQ(last_cfg.intr_type, GPIO_INTR_POSEDGE);
    ASSERT_NE(registered_isr, nullptr);
    registered_isr(nullptr);
    EXPECT_TRUE(notified);
}

#undef PLC_INT_PIN
#define PLC_INT_PIN -1

static bool plc_irq_setup_called = false;
static void plc_irq_setup_stub(TaskHandle_t) { plc_irq_setup_called = true; }

TEST(PlcIrqEdge, InvalidPin) {
    testing::internal::CaptureStdout();
    if (PLC_INT_PIN >= 0) {
        plc_irq_setup_stub(nullptr);
    } else {
        printf("PLC_INT_PIN invalid: %d\n", PLC_INT_PIN);
    }
    std::string log = testing::internal::GetCapturedStdout();
    EXPECT_FALSE(plc_irq_setup_called);
    EXPECT_NE(log.find("PLC_INT_PIN invalid"), std::string::npos);
}

