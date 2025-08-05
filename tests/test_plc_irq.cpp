#include <gtest/gtest.h>
#define GPIO_STUB_CUSTOM
#include "driver/gpio.h"

static gpio_config_t last_cfg{};
static gpio_isr_t registered_isr = nullptr;

static esp_err_t gpio_config(const gpio_config_t* cfg) {
    last_cfg = *cfg;
    return ESP_OK;
}

static esp_err_t gpio_install_isr_service(int) { return ESP_OK; }

static esp_err_t gpio_isr_handler_add(gpio_num_t, gpio_isr_t isr, void*) {
    registered_isr = isr;
    return ESP_OK;
}

#define PLC_INT_PIN 0
#define plc_irq plc_irq_neg
#define plc_isr plc_isr_neg
#define plc_irq_setup plc_irq_setup_neg
#include "examples/platformio_complete/src/plc_irq.hpp"
#undef plc_irq
#undef plc_isr
#undef plc_irq_setup

TEST(PlcIrqEdge, Negative) {
    plc_irq_neg = false;
    plc_irq_setup_neg();
    EXPECT_EQ(last_cfg.intr_type, GPIO_INTR_NEGEDGE);
    ASSERT_NE(registered_isr, nullptr);
    registered_isr(nullptr);
    EXPECT_TRUE(plc_irq_neg);
}

#undef PLC_IRQ_HPP
#undef PLC_INT_EDGE
#define PLC_INT_EDGE GPIO_INTR_POSEDGE
#define plc_irq plc_irq_pos
#define plc_isr plc_isr_pos
#define plc_irq_setup plc_irq_setup_pos
#include "examples/platformio_complete/src/plc_irq.hpp"
#undef plc_irq
#undef plc_isr
#undef plc_irq_setup

TEST(PlcIrqEdge, Positive) {
    plc_irq_pos = false;
    plc_irq_setup_pos();
    EXPECT_EQ(last_cfg.intr_type, GPIO_INTR_POSEDGE);
    ASSERT_NE(registered_isr, nullptr);
    registered_isr(nullptr);
    EXPECT_TRUE(plc_irq_pos);
}
