#ifndef SLAC_LOGGING_COMPAT_HPP
#define SLAC_LOGGING_COMPAT_HPP

#ifdef ESP_PLATFORM
#include <esp_log.h>
#else
#ifndef ESP_LOGE
#define ESP_LOGE(tag, fmt, ...)
#endif
#ifndef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...)
#endif
#ifndef ESP_LOGW
#define ESP_LOGW(tag, fmt, ...)
#endif
#endif

#endif // SLAC_LOGGING_COMPAT_HPP
