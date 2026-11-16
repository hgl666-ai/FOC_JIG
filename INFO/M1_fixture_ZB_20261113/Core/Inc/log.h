
#ifndef __LOG_H__
#define __LOG_H__

#include "stm32f1xx_hal.h"

#define LOG_ENABLE   // 打开日志输出

#ifdef LOG_ENABLE
typedef enum {
    LOG_LEVEL_NONE = 0,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
} log_level_t;

/**
 * @brief 日志输出函数
 * 
 * @param level 日志级别，参考log_level_t
 * @param fmt 格式化字符串，目前只支持 %d, %s, %x, %X, %f
 * @param ... 可变参数
 */
extern void logging(log_level_t level, const char *fmt, ...);

extern log_level_t gLogLevel;

static inline void set_log_level(log_level_t level) {
    gLogLevel = level;
}

#define LOGD(fmt, ...)     do { if (gLogLevel>=LOG_LEVEL_DEBUG) {logging(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__);}} while(0)
#define LOGI(fmt, ...)     do { if (gLogLevel>=LOG_LEVEL_INFO) {logging(LOG_LEVEL_INFO, fmt, ##__VA_ARGS__);}} while(0)
#define LOGW(fmt, ...)     do { if (gLogLevel>=LOG_LEVEL_WARNING) {logging(LOG_LEVEL_WARNING, fmt, ##__VA_ARGS__);}} while(0)
#define LOGE(fmt, ...)     do { if (gLogLevel>=LOG_LEVEL_ERROR) {logging(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__);}} while(0)
#define LOG(fmt, ...)      logging(LOG_LEVEL_NONE, fmt, ##__VA_ARGS__)

#define LOG_ERROR_AND_EXIT(fmt, ...)   do { \
                                            LOGE(fmt, ##__VA_ARGS__); \
                                            LOGE("Exiting..."); \
                                            __disable_irq(); \
                                            while (1) {} \
                                        } while (0)
#else
#define LOGD(fmt, ...)
#define LOGI(fmt, ...)
#define LOGW(fmt, ...)
#define LOGE(fmt, ...)
#define LOG(fmt, ...)
#define LOG_ERROR_AND_EXIT(fmt, ...)  do { \
                                            __disable_irq(); \
                                            while (1) {} \
                                        } while (0)
#endif

#endif
