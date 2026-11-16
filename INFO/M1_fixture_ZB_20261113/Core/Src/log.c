
#include "log.h"
#include "m_macros.h"

#ifdef LOG_ENABLE

#include <stdarg.h>

#include "usart_m.h"

#if UART1_ENABLE
    #if ENABLED(UART_TX_DMA_ENABLE)
        #define LOG_TX_U8(u8data)              UART1_DMA_TX_U8(u8data, 0)
        #define LOG_TX_U8_START(u8data)        UART1_DMA_TX_U8(u8data, 1)
        #define LOG_TX_WAIT_DMA()              WAIT_UART1_DMA_READY()
    #else
        #define LOG_TX_U8(u8data)              UART1_TX_U8(u8data)
        #define LOG_TX_U8_START(u8data)        UART1_TX_U8(u8data)
        #define LOG_TX_WAIT_DMA()
    #endif
#elif UART2_ENABLE
    #if ENABLED(UART_TX_DMA)
        #define LOG_TX_U8(u8data)              UART2_DMA_TX_U8(u8data, 0)
        #define LOG_TX_U8_START(u8data)        UART2_DMA_TX_U8(u8data, 1)
        #define LOG_TX_WAIT_DMA()              WAIT_UART2_DMA_READY()
    #else
        #define LOG_TX_U8(u8data)              UART2_TX_U8(u8data)
        #define LOG_TX_U8_START(u8data)        UART2_TX_U8(u8data)
        #define LOG_TX_WAIT_DMA()
    #endif
#else
    static char log_buf[128]  = {0};
    static int log_i = 0;
    #define LOG_TX_U8(u8data)              do { log_buf[log_i++] = u8data; } while (0)
    #define LOG_TX_U8_START(u8data)        do { \
                                                log_buf[log_i++] = u8data; \
                                                HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, log_i, 10); \
                                                log_i = 0; \
                                            } while (0)
    #define LOG_TX_WAIT_DMA()              do { log_i = 0; } while (0)
    // #error "Can not config log: UART1_ENABLE or UART2_ENABLE must be defined"
#endif

log_level_t gLogLevel = LOG_LEVEL_INFO;

static void send_number(int num) {
    if (num == 0) {
        LOG_TX_U8('0');
        return;
    }

    // 处理-2147483648的特殊情况
    if (num == -2147483648) {
        const char *str = "-2147483648";
        while (*str) {
            LOG_TX_U8(*str++);
        }
        return;
    }

    char buffer[12];
    int i = 0;
    int is_negative = 0;

    if (num < 0) {
        is_negative = 1;
        num = -num;
    }

    while (num > 0) {
        buffer[i++] = '0' + (num % 10);
        num /= 10;
    }

    if (is_negative) {
        buffer[i++] = '-';
    }

    // 反转并发送字符
    while (i > 0) {
        LOG_TX_U8(buffer[--i]);
    }
}

static void send_hex_number(unsigned int num) {
    if (num == 0) {
        LOG_TX_U8('0');
        return;
    }

    char buffer[9];
    int i = 0;
    
    while (num > 0) {
        int digit = num % 16;
        if (digit < 10) {
            buffer[i++] = '0' + digit;
        } else {
            buffer[i++] = 'A' + (digit - 10);
        }
        num /= 16;
    }

    // 反转并发送字符
    while (i > 0) {
        LOG_TX_U8(buffer[--i]);
    }
}

void logging(log_level_t level, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    const char *p = fmt;

    LOG_TX_WAIT_DMA();
    if (LOG_LEVEL_NONE != level) {
        LOG_TX_U8('(');
        switch (level) {
            case LOG_LEVEL_DEBUG:
                LOG_TX_U8('D');
                break;
            case LOG_LEVEL_INFO:
                LOG_TX_U8('I');
                break;
            case LOG_LEVEL_WARNING:
                LOG_TX_U8('W');
                break;
            case LOG_LEVEL_ERROR:
                LOG_TX_U8('E');
                break;
            default:
                LOG_TX_U8('I');
                break;
        }
        LOG_TX_U8(')');
        LOG_TX_U8(' ');
    }

    while (*p != '\0') {
        if (*p == '%') {
            if (*(p + 1) == 'd') {
                int num = va_arg(args, int);
                send_number(num);
                p += 2; // 跳过%d
            } else if (*(p + 1) == 'f') {
                double numf = va_arg(args, double);
                int num = (int)numf;
                int decimal = (int)((numf - num) * 1000);
                if (decimal < 0) { decimal = -decimal; }
                if (num == 0 && numf < 0) {
                    LOG_TX_U8('-');
                }
                send_number(num);
                LOG_TX_U8('.');
                send_number(decimal);
                p += 2; // 跳过%X
            } else if ((*(p + 1) == 'X') || (*(p + 1) == 'x')) {
                unsigned int num = va_arg(args, unsigned int);
                send_hex_number(num);
                p += 2; // 跳过%X
            } else if (*(p + 1) == 's') {
                const char *str = va_arg(args, const char *);
                while (*str) {
                    LOG_TX_U8(*str);
                    str++;
                }
                p += 2; // 跳过%s
            } else {
                LOG_TX_U8('%');
                if (*(p + 1) == '\0') {
                    break;
                } else {
                    LOG_TX_U8(*(p + 1)); // 发送非格式化的字符
                    p += 2;
                }
            }
        } else {
            LOG_TX_U8(*p);
            p++;
        }
    }
    LOG_TX_U8_START('\n');

    va_end(args);
}

#endif
