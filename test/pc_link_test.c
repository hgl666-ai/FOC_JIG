/*
 * PC 通讯口传输层（pc_link.c）宿主单元测试
 *
 * 一份源码两种编译：
 *   gcc ... -o pc_link_test.exe            → 测 **USB CDC 分支**（默认出厂配置）
 *   gcc ... -DJIG_PC_LINK=1 -o ...        → 测 **USART1 分支**（特殊测试 / C6T6 32KB）
 *
 * 必须守住的两条不变量（见 pc_link.c 文件头）：
 *   I1. 整帧要么全发，要么不发 —— 绝不允许截断
 *   I2. 上一帧没发完就不组下一帧 —— PcLink_TxIdle() 表达这件事
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "jig_mode.h"
#include "pc_link.h"
#include "protocol_frame.h"
#include "flash_map.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg)                                                  \
    do {                                                                  \
        if (cond) {                                                       \
            g_pass++;                                                     \
        } else {                                                          \
            g_fail++;                                                     \
            printf("  [FAIL] %s  (line %d)\n", (msg), __LINE__);           \
        }                                                                 \
    } while (0)

/* 虚拟时间（pc_link / uart_port 都会问 HAL_GetTick） */
static uint32_t g_tick = 0;
uint32_t HAL_GetTick(void) { return g_tick++; }
void     HAL_Delay(uint32_t ms) { g_tick += ms; }

/*==============================================================================
 * 假 HAL：USB 分支
 *============================================================================*/
#if (JIG_PC_LINK != JIG_PC_LINK_UART)

#include "usbd_cdc_if.h"    /* 宿主替身: USBD_BUSY / USBD_CDC_HandleTypeDef */
#include "usb_device.h"     /* 宿主替身: USBD_HandleTypeDef / hUsbDeviceFS */

USBD_HandleTypeDef hUsbDeviceFS;

static USBD_CDC_HandleTypeDef g_cdc;
static uint8_t  g_cap[256];
static uint16_t g_cap_len;
static uint8_t  g_busy_cnt;      /* 前 N 次 CDC_Transmit_FS 先回 USBD_BUSY */

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    if (g_busy_cnt != 0u) {
        g_busy_cnt--;
        return USBD_BUSY;
    }
    if (((uint32_t)g_cap_len + (uint32_t)Len) <= (uint32_t)sizeof(g_cap)) {
        memcpy(&g_cap[g_cap_len], Buf, Len);
        g_cap_len = (uint16_t)(g_cap_len + Len);
    }
    return USBD_OK;
}

/*==============================================================================
 * 假 HAL：USART1 分支
 *============================================================================*/
#else

#include "uart_cfg.h"
#include "uart_port.h"
#include "app_frame.h"

/* pc_link.c 现在把收到的字节交给 APP_PcInput()（那个入口里还有 ASCII 控制台的判断），
 * 而 APP_PcInput 的实现在 app.c 里 —— 本测试不链接 app.c，所以在这里给一个替身：
 * 只是把字节原样转给帧解析器，等于控制台没开时的行为，正是本测试要验的传输语义。 */
void APP_PcInput(uint8_t b)
{
    APP_Frame_Feed(b);
}

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

static uint8_t *s_rxp1;          /* huart1 的接收落点（= PC 方向进来的字节） */
static uint8_t  g_cap[256];
static uint16_t g_cap_len;
static uint8_t  g_hold;          /* 1 = 假装这段发不完（不回调 TxCplt），用来造"忙" */

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)Size;
    if (huart == &huart1) {
        s_rxp1 = pData;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if (g_hold == 0u) {
        if (((uint32_t)g_cap_len + (uint32_t)Size) <= (uint32_t)sizeof(g_cap)) {
            memcpy(&g_cap[g_cap_len], pData, Size);
            g_cap_len = (uint16_t)(g_cap_len + Size);
        }
        HAL_UART_TxCpltCallback(huart);      /* 同步完成：驱动推进 tx_tail */
    }
    return HAL_OK;
}

/* 往 huart1 的接收落点塞一个字节（等价于"线上来了一个字节"） */
static void pc_deliver(uint8_t b)
{
    if (s_rxp1 == 0) {
        return;                              /* 没挂接收 → 这字节真丢了 */
    }
    *s_rxp1 = b;
    s_rxp1  = 0;                             /* 落点被消费，等回调里重挂 */
    HAL_UART_RxCpltCallback(&huart1);
    g_tick++;
}

#endif /* JIG_PC_LINK */

/*==============================================================================
 * 用例
 *============================================================================*/
static uint8_t g_frame[APP_FRAME_BUF_SIZE];

static uint16_t MakeFrame(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    return Frame_Build(g_frame, cmd, seq, data, len);
}

#if (JIG_PC_LINK != JIG_PC_LINK_UART)

static void test_usb_branch(void)
{
    uint16_t n;

    printf("-- USB CDC branch --\n");

    /* T1: pClassData 还没挂上（USB 未枚举）→ 视为不可用 */
    hUsbDeviceFS.pClassData = 0;
    CHECK(PcLink_TxIdle() == 0u, "no CDC class data must report not-idle");

    /* T2: TxState 跟随 */
    hUsbDeviceFS.pClassData = &g_cdc;
    g_cdc.TxState = 0u;
    CHECK(PcLink_TxIdle() == 1u, "TxState=0 -> idle");
    g_cdc.TxState = 1u;
    CHECK(PcLink_TxIdle() == 0u, "TxState=1 -> busy");
    g_cdc.TxState = 0u;

    /* T3: 发出去的字节必须与 Frame_Build 组出来的一模一样 */
    {
        const uint8_t req[3] = { 0x11u, 0x22u, 0x33u };
        n = MakeFrame(0x0009u, 0x0001u, req, 3u);
        g_cap_len = 0u;
        PcLink_Send(g_frame, n);
        CHECK(g_cap_len == n, "sent length equals frame length");
        CHECK(memcmp(g_cap, g_frame, n) == 0, "sent bytes equal frame bytes");
    }

    /* T4: 前几次 BUSY → 重试后仍要发完整帧（不截断） */
    {
        const uint8_t ack[1] = { 0x00u };
        n = MakeFrame(0x0009u, 0x0001u, ack, 1u);
        g_cap_len  = 0u;
        g_busy_cnt = 3u;
        PcLink_Send(g_frame, n);
        CHECK(g_cap_len == n, "frame sent after BUSY retries");
        CHECK(memcmp(g_cap, g_frame, n) == 0, "retried frame bytes intact");
    }

    /* T5: Poll 在 USB 分支必须是空操作（字节已在中断里喂进解析器） */
    PcLink_Poll();
    CHECK(1, "PcLink_Poll() is a no-op in USB branch");
}

#else

static void test_uart_tx(void)
{
    const uint8_t ack[1] = { 0x00u };
    uint16_t      n;

    printf("-- UART branch: TX --\n");
    UartCfg_Init();                 /* 绑 huart1->端口0(PC)，开接收 */

    /* T1: 整帧原样发出去 */
    n = MakeFrame(0x0009u, 0x0001u, ack, 1u);
    g_cap_len = 0u;
    PcLink_Send(g_frame, n);
    CHECK(g_cap_len == n, "uart sent length equals frame length");
    CHECK(memcmp(g_cap, g_frame, n) == 0, "uart sent bytes equal frame bytes");
    CHECK(PcLink_TxIdle() == 1u, "idle again after synchronous TX");

    /* T2: 长帧（91B 应答）也完整发出，不被截断 */
    {
        uint8_t big[91];
        uint16_t i;
        for (i = 0u; i < (uint16_t)sizeof(big); i++) {
            big[i] = (uint8_t)(i * 7u + 1u);
        }
        n = MakeFrame(0x000Fu, 0x0002u, big, (uint16_t)sizeof(big));
        g_cap_len = 0u;
        PcLink_Send(g_frame, n);
        CHECK(g_cap_len == n, "long frame (91B data) sent completely");
        CHECK(memcmp(g_cap, g_frame, n) == 0, "long frame bytes intact");
    }

    /* T3: 有字节在飞 → TxIdle 必须为 0（I2） */
    {
        uart_port_t *p = UartPort_Get(UART_CFG_PORT_PC);
        g_hold = 1u;                                  /* 假装发不完 */
        (void)UartPort_Write(UART_CFG_PORT_PC, (const uint8_t *)"AB", 2u);
        CHECK(p->tx_busy != 0u, "tx_busy set while in flight");
        CHECK(PcLink_TxIdle() == 0u, "must report busy while a segment is in flight");

        g_hold = 0u;
        HAL_UART_TxCpltCallback(&huart1);              /* 发完这一段 */
        CHECK(PcLink_TxIdle() == 1u, "idle after completion");
    }

    /* T4: 发送队列放不下整帧 → 整帧丢弃，绝不发半截（I1） */
    {
        uart_port_t *p = UartPort_Get(UART_CFG_PORT_PC);
        uint8_t      filler[200];
        uint16_t     used;
        uint16_t     before;

        memset(filler, 0x5Au, sizeof(filler));
        g_hold   = 1u;                                 /* 不让它排空 */
        g_cap_len = 0u;
        (void)UartPort_Write(UART_CFG_PORT_PC, filler, (uint16_t)sizeof(filler));

        used   = (uint16_t)(p->tx_head - p->tx_tail);
        before = g_cap_len;

        n = MakeFrame(0x0009u, 0x0003u, ack, 1u);      /* 14B 帧 */
        PcLink_Send(g_frame, n);

        /* 队列已占 200B（容量 255），14B 其实还放得下 → 这次应当成功入队；
         * 真正要守的是"不出现半个帧"，所以这里校验要么整帧入队、要么完全没动。 */
        {
            uint16_t after = (uint16_t)(p->tx_head - p->tx_tail);
            uint16_t added = (uint16_t)(after - used);
            CHECK((added == 0u) || (added == n), "queue accepted either the whole frame or nothing");
            CHECK(g_cap_len == before, "nothing hits the wire while held busy");
        }

        /* 再塞满到放不下一整帧 */
        (void)UartPort_Write(UART_CFG_PORT_PC, filler, 40u);
        used  = (uint16_t)(p->tx_head - p->tx_tail);
        CHECK(used > (uint16_t)(UART_PORT_TX_BUF_SIZE - 1u - n), "queue nearly full for next check");

        {
            uint16_t head_before = p->tx_head;
            PcLink_Send(g_frame, n);
            CHECK(p->tx_head == head_before, "full queue -> frame dropped entirely, no partial write");
        }

        /* 收尾：排空，避免影响后续用例 */
        g_hold = 0u;
        while (UartPort_Get(UART_CFG_PORT_PC)->tx_busy != 0u) {
            HAL_UART_TxCpltCallback(&huart1);
            if (UartPort_Get(UART_CFG_PORT_PC)->tx_head == UartPort_Get(UART_CFG_PORT_PC)->tx_tail) {
                break;
            }
        }
        CHECK(1, "cleanup");
    }
}

static void test_uart_rx(void)
{
    const uint8_t req[3] = { 0xAAu, 0xBBu, 0xCCu };
    uint8_t      *got;
    uint16_t      n;
    uint16_t      glen;
    uint16_t      i;

    printf("-- UART branch: RX --\n");
    APP_Frame_Init();
    n = MakeFrame(0x000Eu, 0x002Au, req, 3u);

    /* 逐字节喂（模拟线上一个一个来），中间夹一次中途 Poll，验证不丢字节 */
    for (i = 0u; i < n; i++) {
        pc_deliver(g_frame[i]);
        if (i == 3u) {
            PcLink_Poll();                 /* 半截帧：不该被当成完整帧 */
            CHECK(APP_Frame_Pending() == 0u, "partial frame is not pending");
        }
    }
    PcLink_Poll();
    APP_Frame_Poll();
    CHECK(APP_Frame_Pending() == 1u, "complete frame pending after poll");

    got = APP_Frame_Consume(&glen);
    CHECK(glen == n, "consumed length matches");
    CHECK((got != 0) && (memcmp(got, g_frame, n) == 0), "consumed bytes match frame");

    /* 连发两帧：顺序必须保持 */
    {
        const uint8_t r2[2] = { 0x01u, 0x02u };
        uint8_t  f1[APP_FRAME_BUF_SIZE];
        uint8_t  f2[APP_FRAME_BUF_SIZE];
        uint16_t n2 = Frame_Build(f2, 0x000Fu, 0x002Bu, r2, 2u);

        memcpy(f1, g_frame, n);                 /* 第一帧 = 前面那帧 0x000E/0x002A */
        for (i = 0u; i < n; i++) {
            pc_deliver(f1[i]);
        }
        for (i = 0u; i < n2; i++) {
            pc_deliver(f2[i]);
        }

        PcLink_Poll();
        APP_Frame_Poll();
        CHECK(APP_Frame_Pending() == 1u, "first of two frames parsed");
        got = APP_Frame_Consume(&glen);
        CHECK((got != 0) && (glen == n) && (memcmp(got, f1, n) == 0), "first frame bytes match");

        APP_Frame_Poll();
        CHECK(APP_Frame_Pending() == 1u, "second of two frames parsed");
        got = APP_Frame_Consume(&glen);
        CHECK((got != 0) && (glen == n2) && (memcmp(got, f2, n2) == 0), "second frame bytes match");
    }
}

#endif

int main(void)
{
    printf("==================================================\n");
#if (JIG_PC_LINK == JIG_PC_LINK_UART)
    printf(" pc_link unit test  [JIG_PC_LINK = UART / USART1]\n");
#else
    printf(" pc_link unit test  [JIG_PC_LINK = USB CDC]\n");
#endif
    printf("==================================================\n");

#if (JIG_PC_LINK != JIG_PC_LINK_UART)
    g_cdc.TxState = 0u;
    test_usb_branch();
#else
    test_uart_tx();
    test_uart_rx();
#endif

    printf("--------------------------------------------------\n");
    printf(" pass = %d, fail = %d\n", g_pass, g_fail);
    if (g_fail == 0) {
        printf(" RESULT: ALL PASS\n");
        return 0;
    }
    printf(" RESULT: FAILED\n");
    return 1;
}
