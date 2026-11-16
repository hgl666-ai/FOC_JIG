/**
  ******************************************************************************
  * @file    fx_bus.c
  * @brief   副芯片 USART1 协议 + DUT UART5 命令通讯
  ******************************************************************************
  */
#include "fx_bus.h"

#include <string.h>

#include "usart.h"
#include "usart_m.h"
#include "log.h"

/*==============================================================================
 * 一、副芯片 USART1 协议
 *============================================================================*/
#define FXB_HDR0            0xA5u
#define FXB_HDR1            0x5Cu

#define FXB_CMD_PING        0x01u
#define FXB_CMD_IO_READ     0x10u
#define FXB_CMD_VOLT_READ   0x11u
#define FXB_CMD_ADC_READ    0x12u

#define FXB_DATA_MAX        16u
#define FXB_TIMEOUT_MS      60u
#define FXB_RETRY           2u

static uint16_t fxb_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  j;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (j = 0u; j < 8u; j++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

/*------------------------------------------------------------------------------
 * 应答帧解析：状态机 + CRC 校验。日志文本（ASCII）会被自然忽略。
 *---------------------------------------------------------------------------*/
static uint8_t  s_rx_st;
static uint8_t  s_rx_n;
static uint8_t  s_rx_need;
static uint8_t  s_rx[24];

static void fxb_rx_reset(void)
{
    s_rx_st   = 0u;
    s_rx_n    = 0u;
    s_rx_need = 0u;
}

/* @retval 0=还没收全；非 0=收到一帧合法帧，返回 CMD */
static uint8_t fxb_rx_feed(uint8_t b, uint8_t *out, uint8_t *out_len)
{
    if (s_rx_st == 0u) {
        if (b == FXB_HDR0) {
            s_rx[0] = b;
            s_rx_n  = 1u;
            s_rx_st = 1u;
        }
        return 0u;
    }

    if (s_rx_st == 1u) {
        if (b == FXB_HDR1) {
            s_rx[1] = b;
            s_rx_n  = 2u;
            s_rx_st = 2u;
        } else if (b == FXB_HDR0) {
            s_rx[0] = b;
            s_rx_n  = 1u;
        } else {
            fxb_rx_reset();
        }
        return 0u;
    }

    if (s_rx_n >= (uint8_t)sizeof(s_rx)) {
        fxb_rx_reset();
        return 0u;
    }

    s_rx[s_rx_n++] = b;

    if (s_rx_n == 4u) {
        if (s_rx[3] > (uint8_t)FXB_DATA_MAX) {
            fxb_rx_reset();
            return 0u;
        }
        s_rx_need = (uint8_t)(4u + s_rx[3] + 2u);
    }

    if ((s_rx_n >= 4u) && (s_rx_n == s_rx_need)) {
        uint16_t crc  = fxb_crc16(s_rx, (uint16_t)(s_rx_n - 2u));
        uint16_t recv = (uint16_t)(((uint16_t)s_rx[s_rx_n - 2u] << 8) |
                                   (uint16_t)s_rx[s_rx_n - 1u]);
        uint8_t  cmd  = s_rx[2];
        uint8_t  len  = s_rx[3];

        if (crc == recv) {
            if ((out != 0) && (out_len != 0)) {
                if (len > 0u) {
                    memcpy(out, &s_rx[4], len);
                }
                *out_len = len;
            }
            fxb_rx_reset();
            return cmd;
        }
        fxb_rx_reset();
    }

    return 0u;
}

static void fxb_flush_rx(void)
{
    uint8_t *p;

    while (uart_rx_queue_get_head(&gUart1RxQueue, &p) > 0u) {
        (void)p;
        uart_rx_queue_out(&gUart1RxQueue);
    }
}

static uint8_t fxb_build(uint8_t *buf, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint16_t n = 0u;
    uint16_t crc;

    buf[n++] = FXB_HDR0;
    buf[n++] = FXB_HDR1;
    buf[n++] = cmd;
    buf[n++] = len;
    if ((len > 0u) && (data != 0)) {
        memcpy(&buf[n], data, len);
        n = (uint16_t)(n + len);
    }
    crc = fxb_crc16(buf, n);
    buf[n++] = (uint8_t)(crc >> 8);
    buf[n++] = (uint8_t)(crc);

    return (uint8_t)n;
}

/** 发一条请求并等待同 CMD 的应答。成功返回 1，data 中为 LEN 字节。 */
static uint8_t fxb_transact(uint8_t cmd, uint8_t *data, uint8_t expect_len)
{
    uint8_t  req[8];
    uint8_t  req_len;
    uint8_t  attempt;

    req_len = fxb_build(req, cmd, 0, 0u);

    for (attempt = 0u; attempt < FXB_RETRY; attempt++) {
        uint32_t start;
        uint8_t  got = 0u;

        fxb_flush_rx();
        fxb_rx_reset();

        /* 先把上一行日志的 DMA 发完，避免请求字节和日志字节交叉。
         * 正式上线日志关闭后，这两步几乎不耗时。 */
        LOG("");
#ifdef UART_TX_DMA_ENABLE
        WAIT_UART1_DMA_READY();
#endif

        if (HAL_UART_Transmit(&huart1, req, req_len, 20u) != HAL_OK) {
            continue;
        }

        start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start) < FXB_TIMEOUT_MS) {
            uint8_t *p;
            uint8_t  n = uart_rx_queue_get_head(&gUart1RxQueue, &p);

            if (n > 0u) {
                uint8_t i;
                for (i = 0u; i < n; i++) {
                    uint8_t tmp[FXB_DATA_MAX];
                    uint8_t tmp_len = 0u;
                    uint8_t c = fxb_rx_feed(p[i], tmp, &tmp_len);

                    if ((c == cmd) && (tmp_len == expect_len)) {
                        memcpy(data, tmp, tmp_len);
                        got = 1u;
                        break;
                    }
                }
                uart_rx_queue_out(&gUart1RxQueue);
            }
            if (got != 0u) {
                return 1u;
            }
        }
    }

    return 0u;
}

uint8_t fx_bus_subchip_ping(void)
{
    uint8_t ver = 0u;
    return fxb_transact(FXB_CMD_PING, &ver, 1u);
}

uint8_t fx_bus_subchip_read_io(uint8_t *bitmap)
{
    if (bitmap == 0) {
        return 0u;
    }
    return fxb_transact(FXB_CMD_IO_READ, bitmap, 1u);
}

uint8_t fx_bus_subchip_read_volt(uint16_t mv[FX_SUB_VOLT_QTY])
{
    uint8_t  d[6];
    uint8_t  i;

    if (fxb_transact(FXB_CMD_VOLT_READ, d, 6u) == 0u) {
        return 0u;
    }
    for (i = 0u; i < (uint8_t)FX_SUB_VOLT_QTY; i++) {
        mv[i] = (uint16_t)(((uint16_t)d[i * 2u] << 8) | (uint16_t)d[i * 2u + 1u]);
    }
    return 1u;
}

uint8_t fx_bus_subchip_read_adc(uint16_t mv[FX_SUB_ADC_QTY], uint8_t *mer_level)
{
    uint8_t d[7];
    uint8_t i;

    if (fxb_transact(FXB_CMD_ADC_READ, d, 7u) == 0u) {
        return 0u;
    }
    for (i = 0u; i < (uint8_t)FX_SUB_ADC_QTY; i++) {
        mv[i] = (uint16_t)(((uint16_t)d[i * 2u] << 8) | (uint16_t)d[i * 2u + 1u]);
    }
    if (mer_level != 0) {
        *mer_level = (d[6] != 0u) ? 1u : 0u;
    }
    return 1u;
}

/*==============================================================================
 * 二、DUT UART5 命令通讯（DUT USART1，应用层 256000 8N1）
 *============================================================================*/
#define FX_DUT_BAUD         256000u
#define FX_DUT_TIMEOUT_MS   150u
#define FX_DUT_RETRY        2u
#define FX_DUT_LINE_MAX     192u

/** 解析 "0:1, 1:0, ..." 这种索引:值 形式。
 *  found_mask 要求所有需要的索引都出现，避免把别的日志当应答。 */
static uint8_t fx_dut_parse(const char *s, uint16_t *vals, uint8_t qty, uint16_t need_mask)
{
    const char *p = s;
    uint16_t found = 0u;

    while (*p != '\0') {
        if ((*p >= '0') && (*p <= '9')) {
            uint32_t idx = 0u;

            while ((*p >= '0') && (*p <= '9')) {
                idx = idx * 10u + (uint32_t)(*p - '0');
                if (idx > 255u) {
                    idx = 255u;
                }
                p++;
            }

            if (*p == ':') {
                int32_t  value = 0;
                uint8_t  neg = 0u;

                p++;
                while ((*p == ' ') || (*p == '\t')) {
                    p++;
                }
                if (*p == '-') {
                    neg = 1u;
                    p++;
                }
                while ((*p >= '0') && (*p <= '9')) {
                    value = value * 10 + (*p - '0');
                    if (value > 100000) {
                        value = 100000;
                    }
                    p++;
                }

                if (idx < (uint32_t)qty) {
                    vals[idx] = (uint16_t)((neg != 0u) ? -value : value);
                    found |= (uint16_t)(1u << idx);
                }
                continue;
            }
            continue;
        }
        p++;
    }

    return ((found & need_mask) == need_mask) ? 1u : 0u;
}

static uint8_t fx_dut_query(const char *cmd, uint16_t *vals, uint8_t qty, uint16_t need_mask)
{
    char     line[FX_DUT_LINE_MAX];
    uint16_t n;
    uint8_t  attempt;
    uint8_t  i;

    for (i = 0u; i < qty; i++) {
        vals[i] = 0u;
    }

    /* DUT 应用层固定 256000 8N1；若之前在烧录模式，这里切回默认模式 */
    UART5_SetConfig(FX_DUT_BAUD, UART_PARITY_NONE);
    UART5_SetMode(UART5_MODE_DEFAULT);

    for (attempt = 0u; attempt < FX_DUT_RETRY; attempt++) {
        uint32_t start;

        UART5_FlushRx();
        n = 0u;

        if (UART5_Transmit((const uint8_t *)cmd, (uint16_t)strlen(cmd), 50u) != HAL_OK) {
            continue;
        }

        start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - start) < FX_DUT_TIMEOUT_MS) {
            uint8_t b;

            while (UART5_ReadByte(&b)) {
                if ((b == '\n') || (b == '\r')) {
                    if (n > 0u) {
                        line[n] = '\0';
                        if (fx_dut_parse(line, vals, qty, need_mask) != 0u) {
                            return 1u;
                        }
                        n = 0u;
                    }
                } else if (n < (uint16_t)(sizeof(line) - 1u)) {
                    line[n++] = (char)b;
                }
            }
        }

        if (n > 0u) {
            line[n] = '\0';
            if (fx_dut_parse(line, vals, qty, need_mask) != 0u) {
                return 1u;
            }
        }
    }

    return 0u;
}

uint8_t fx_bus_dut_read_io(uint8_t levels[FX_DUT_IO_QTY])
{
    uint16_t v[FX_DUT_IO_QTY];
    uint8_t  i;

    if (levels == 0) {
        return 0u;
    }
    if (fx_dut_query("M119 A\n", v, (uint8_t)FX_DUT_IO_QTY, 0x00FFu) == 0u) {
        return 0u;
    }

    /* DUT v1.3 的触发极性（DUT gpio.c sensor_init）：
     *   KEY0..KEY4 : trigger_level = SET   -> 电平 = triggered
     *   TBS/TEST-INT/MMS : trigger_level = RESET -> 电平 = !triggered
     * M119 A 打印的是 triggered，不是原始引脚电平。 */
    for (i = 0u; i < (uint8_t)FX_DUT_IO_KEY4 + 1u; i++) {
        if (v[i] > 1u) {
            return 0u;
        }
        levels[i] = (uint8_t)((v[i] != 0u) ? 1u : 0u);
    }
    for (i = (uint8_t)FX_DUT_IO_TBS; i < (uint8_t)FX_DUT_IO_QTY; i++) {
        if (v[i] > 1u) {
            return 0u;
        }
        levels[i] = (uint8_t)((v[i] != 0u) ? 0u : 1u);
    }

    return 1u;
}

uint8_t fx_bus_dut_read_adc(uint16_t mv[FX_DUT_ADC_QTY])
{
    uint16_t v[FX_DUT_ADC_QTY];
    uint8_t  i;

    if (mv == 0) {
        return 0u;
    }
    if (fx_dut_query("M105 V\n", v, (uint8_t)FX_DUT_ADC_QTY, 0x0007u) == 0u) {
        return 0u;
    }
    for (i = 0u; i < (uint8_t)FX_DUT_ADC_QTY; i++) {
        if (v[i] > 4000u) {
            return 0u;                 /* 明显不是 mV，判为解析失败 */
        }
        mv[i] = v[i];
    }
    return 1u;
}

/*==============================================================================
 * 三、对外初始化/轮询
 *============================================================================*/
void fx_bus_init(void)
{
    fxb_rx_reset();
    fxb_flush_rx();
}

void fx_bus_poll(void)
{
    /* 副芯片与 DUT 都是"命令-应答"事务，阻塞在调用点完成；
     * 这里预留给以后的心跳/在线检测。 */
}
