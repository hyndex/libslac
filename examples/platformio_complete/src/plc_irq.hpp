#ifndef PLC_IRQ_HPP
#define PLC_IRQ_HPP

#include <driver/gpio.h>

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

inline volatile bool plc_irq = false;
static inline void IRAM_ATTR plc_isr(void*) { plc_irq = true; }

#ifndef PLC_INT_EDGE
#define PLC_INT_EDGE GPIO_INTR_NEGEDGE
#endif

inline void plc_irq_setup() {
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

