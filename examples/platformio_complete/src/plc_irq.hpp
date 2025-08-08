#ifndef PLC_IRQ_HPP
#define PLC_IRQ_HPP

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

inline TaskHandle_t plc_irq_task_handle = nullptr;
static inline void IRAM_ATTR plc_isr(void*) {
    BaseType_t higher_woken = pdFALSE;
    if (plc_irq_task_handle)
        vTaskNotifyGiveFromISR(plc_irq_task_handle, &higher_woken);
    if (higher_woken == pdTRUE)
        portYIELD_FROM_ISR();
}

#ifndef PLC_INT_EDGE
#define PLC_INT_EDGE GPIO_INTR_NEGEDGE
#endif

inline void plc_irq_setup(TaskHandle_t task_to_notify) {
    plc_irq_task_handle = task_to_notify;
    gpio_config_t int_cfg{};
    int_cfg.pin_bit_mask = 1ULL << PLC_INT_PIN;
    int_cfg.mode = GPIO_MODE_INPUT;
    int_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    int_cfg.intr_type = PLC_INT_EDGE;
    gpio_config(&int_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(static_cast<gpio_num_t>(PLC_INT_PIN), plc_isr, nullptr);
}

#endif // PLC_IRQ_HPP

