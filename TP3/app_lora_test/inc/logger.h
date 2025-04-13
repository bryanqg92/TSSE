#ifndef LOGGER_H
#define LOGGER_H

typedef enum {
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
} log_level_t;

void logger_init(log_level_t level);
void logger_log(log_level_t level, const char* format, ...);

// Helpers
#define STM_LOGE(TAG, fmt, ...) logger_log(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define STM_LOGW(TAG, fmt, ...)  logger_log(LOG_LEVEL_WARN,  fmt, ##__VA_ARGS__)
#define STM_LOGI(TAG, fmt, ...)  logger_log(LOG_LEVEL_INFO,  fmt, ##__VA_ARGS__)
#define STM_LOGD(TAG, fmt, ...) logger_log(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)

#endif // LOGGER_H
