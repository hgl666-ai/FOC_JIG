/**
  ******************************************************************************
  * @file    pc_link.c
  * @brief   PC 通讯口的传输层：USB CDC 或 USART1，由 jig_mode.h §2.1 选择
  *
  * 两个分支互斥编译，**任何时刻只有一套在固件里**：
  *   JIG_PC_LINK = JIG_PC_LINK_USB  → CDC_Transmit_FS / hUsbDeviceFS.TxState
  *   JIG_PC_LINK = JIG_PC_LINK_UART → uart_port 的 USART1 端口（PA9/PA10）
  *
  * 共同的不变量（两条路都必须满足，否则 PC 侧解析器会卡死）：
  *   I1. **整帧要么全发，要么不发** —— 缓冲放不下就整帧丢弃，绝不截断；
  *   I2. **上一帧发完之前不组下一帧** —— 由 PcLink_TxIdle() 表达，
  *       调用方拿不到空闲就放弃本帧（让 PC 超时重发），而不覆盖在飞的缓冲。
  ******************************************************************************
  */
#include "main.h"
#include "pc_link.h"
#include "jig_mode.h"
#include "flash_map.h"      /* APP_TX_BUSY_TIMEOUT_MS */

#if (JIG_PC_LINK == JIG_PC_LINK_UART)
#include "uart_cfg.h"
#include "uart_port.h"
#include "app_frame.h"
#include "app.h"          /* APP_PcInput：ASCII 控制台与帧解析的统一入口 */
#else
#include "usbd_cdc_if.h"
#include "usb_device.h"
#endif

/*==============================================================================
 * 分支 A：USB CDC（默认，出厂固件）
 *============================================================================*/
#if (JIG_PC_LINK != JIG_PC_LINK_UART)

uint8_t PcLink_TxIdle(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc == 0) {
        return 0u;              /* 还没枚举出 CDC 接口 → 视为不可用，本帧放弃 */
    }
    return (hcdc->TxState == 0u) ? 1u : 0u;
}

void PcLink_Send(const uint8_t *buf, uint16_t len)
{
    uint32_t tick = HAL_GetTick();

    /* 原样搬过来的忙等循环：USB 卡住就放弃这一帧，让 PC 超时重试 */
    while (CDC_Transmit_FS((uint8_t *)buf, len) == USBD_BUSY) {
        if ((HAL_GetTick() - tick) > APP_TX_BUSY_TIMEOUT_MS) {
            return;
        }
    }
}

void PcLink_Poll(void)
{
    /* 空：USB 侧的字节在 CDC_Receive_FS → APP_USB_Receive() 里已经喂进解析器了，
     * 这里不能再喂一遍（会重复解析）。 */
}

/*==============================================================================
 * 分支 B：串口 USART1（特殊测试 / C6T6 32KB 构建）
 *============================================================================*/
#else

/** 发送队列还能塞多少字节（uart_port 的环形缓冲故意留一格，容量是 SIZE-1） */
static uint16_t pc_tx_free(void)
{
    uart_port_t *p = UartPort_Get(UART_CFG_PORT_PC);
    uint16_t     used;

    if (p == 0) {
        return 0u;
    }
    used = (p->tx_head >= p->tx_tail)
           ? (uint16_t)(p->tx_head - p->tx_tail)
           : (uint16_t)((UART_PORT_TX_BUF_SIZE - p->tx_tail) + p->tx_head);
    if ((uint16_t)(used + 1u) >= UART_PORT_TX_BUF_SIZE) {
        return 0u;
    }
    return (uint16_t)(UART_PORT_TX_BUF_SIZE - used - 1u);
}

uint8_t PcLink_TxIdle(void)
{
    uart_port_t *p = UartPort_Get(UART_CFG_PORT_PC);

    if (p == 0) {
        return 0u;
    }
    if (p->tx_busy != 0u) {
        return 0u;                              /* 还有一段正在线上飞 */
    }
    return (p->tx_head == p->tx_tail) ? 1u : 0u;  /* 队列空 = 上一帧已发完 */
}

void PcLink_Send(const uint8_t *buf, uint16_t len)
{
    if ((buf == 0) || (len == 0u)) {
        return;
    }
    /* I1: 整帧进队。放不下就整帧放弃 —— 半截帧会让 PC 侧解析器丢掉后面所有字节 */
    if (pc_tx_free() < len) {
        return;
    }
    (void)UartPort_Write(UART_CFG_PORT_PC, buf, len);
}

void PcLink_Poll(void)
{
    uint8_t  chunk[32];
    uint16_t n;

    /* 只搬字节、不解析：解析统一在 APP_Frame_Poll() 里做，
     * 和 USB 那条路完全一致。⚠ 必须走 APP_PcInput() 而不是 APP_Frame_Feed()：
     * 那个入口里还有"这一串是不是 ASCII 命令行"的判断（串口助手手敲 AS:Bb 用），
     * 绕过它的话，USB 和串口两条路的行为就会不一样。 */
    while ((n = UartPort_Read(UART_CFG_PORT_PC, chunk, (uint16_t)sizeof(chunk))) > 0u) {
        uint16_t i;

        for (i = 0u; i < n; i++) {
            APP_PcInput(chunk[i]);
        }
    }
}

#endif /* JIG_PC_LINK */
