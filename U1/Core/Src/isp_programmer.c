/**
  ******************************************************************************
  * @file    isp_programmer.c
  * @brief   FOC 板 ISP 烧录 — AN3155 (STM32 系统 bootloader) 实现
  *
  * 流程:
  *   CONNECT(0x7F 握手) → MASS_ERASE(0x44 全片擦除)
  *     → WRITE_BOOT(逐 256B 块写 Slot0 镜像到 DUT_BOOT_ADDR)
  *     → WRITE_APP (逐 256B 块写 Slot1 镜像到 DUT_APP_ADDR)
  *     → GO(0x21 跳转 DUT_GO_ADDR) → 完成
  *
  * 每一步都在 ISP_Step() 里推进一小步, 主循环不被长时间独占。
  * @warning ISP 串口与被控引脚见 isp_config.h (当前为占位值, 待硬件确认)。
  ******************************************************************************
  */
#include "isp_programmer.h"
#include "isp_config.h"
#include "flash_map.h"
#include "bsp_uart.h"
#include "gd25q128.h"
#include "main.h"
#include <string.h>

/*===== AN3155 常量 =====*/
#define ISP_ACK             0x79u
#define ISP_NACK            0x1Fu
#define ISP_CMD_SYNC        0x7Fu
#define ISP_CMD_GET         0x00u
#define ISP_CMD_GETID       0x02u
#define ISP_CMD_WRITE       0x31u
#define ISP_CMD_ERASE_STD   0x43u   /* Erase Memory          (bootloader v2.x, F1)   */
#define ISP_CMD_ERASE_EXT   0x44u   /* Extended Erase Memory (bootloader v3.0+)      */
#define ISP_CMD_GO          0x21u
#define ISP_CMD_GET_VER     0x01u   /* Get Version                                   */
#define ISP_CMD_READ        0x11u   /* Read Memory (仅 ISP_VERIFY_ENABLED 时使用)     */

/*===== 状态机 =====*/
typedef enum {
    ISP_ST_IDLE = 0,
    ISP_ST_CONNECT,
    ISP_ST_ERASE,
    ISP_ST_WRITE_BOOT,
    ISP_ST_VERIFY_BOOT,
    ISP_ST_WRITE_APP,
    ISP_ST_VERIFY_APP,
    ISP_ST_GO,
    ISP_ST_OK,
    ISP_ST_FAIL
} ISP_State_t;

static ISP_State_t s_state = ISP_ST_IDLE;
static uint8_t     s_result = APP_STATUS_ERR_PROG;
static uint8_t     s_bl_version = 0x22u;  /* 目标 bootloader 版本; 读不到时按 F1 v2.2 */
static uint32_t    s_img_len_boot = 0;  /* Bootloader 镜像长度 (擦除前已校验) */
static uint32_t    s_img_len_app  = 0;  /* App 镜像长度        (擦除前已校验) */
static uint32_t    s_slot_base = 0;     /* 当前镜像在 SPI Flash 中的槽基址 */
static uint32_t    s_img_len   = 0;     /* 当前镜像长度 (不含头部)          */
static uint32_t    s_offset    = 0;     /* 已写偏移                        */
static uint32_t    s_dst_addr  = 0;     /* 目标绝对地址                    */
static uint8_t     s_buf[APP_PAGE_SIZE];
#if (ISP_VERIFY_ENABLED)
static uint8_t     s_ref[APP_PAGE_SIZE];    /* 回读比对用的源镜像缓冲 */
#endif

/*==============================================================================
 * 底层: 复位时序
 *============================================================================*/
static void ISP_Boot0Set(uint8_t high)
{
#if (ISP_CTRL_PINS_CONFIRMED)
    HAL_GPIO_WritePin(ISP_BOOT0_PORT, ISP_BOOT0_PIN,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)high;
#endif
}

static void ISP_ResetSet(uint8_t high)
{
#if (ISP_CTRL_PINS_CONFIRMED)
    HAL_GPIO_WritePin(ISP_RESET_PORT, ISP_RESET_PIN,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)high;
#endif
}

/** 初始化 BOOT0/RESET 引脚为推挽输出 */
void ISP_GpioInit(void)
{
#if (ISP_CTRL_PINS_CONFIRMED)
    GPIO_InitTypeDef io = {0};

    /* ⚠ 必须先开对应 GPIO 端口的时钟!
     * CubeMX 生成的 MX_GPIO_Init() 只开了 GPIOD/GPIOA/GPIOB (板上用到 A/B),
     * **没有开 GPIOC** —— 而本工程 RESET 占位在 PC13。
     * STM32F1 上对"时钟未使能"的 GPIO 端口写 BSRR/BRR 是**静默无效**的,
     * 表现就是 RESET 永远不翻转、被烧板永远进不了 bootloader, 极难排查。
     * 这里把 A/B/C 三个端口都显式打开 (已开的重复使能无害),
     * 这样以后把 BOOT0/RESET 改到任意 A/B/C 引脚都不会再踩这个坑。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;

    io.Pin = ISP_BOOT0_PIN;
    HAL_GPIO_Init(ISP_BOOT0_PORT, &io);
    io.Pin = ISP_RESET_PIN;
    HAL_GPIO_Init(ISP_RESET_PORT, &io);

    ISP_Boot0Set(0);
    ISP_ResetSet(1);
#else
    /* 引脚未确认: 只把本体拉高, 不做任何外部时序 (等待用户飞线确认后打开) */
#endif
}

/*==============================================================================
 * 底层: 收发
 *============================================================================*/
static void ISP_SendCmd(uint8_t cmd)
{
    uint8_t pkt[2];
    pkt[0] = cmd;
    pkt[1] = (uint8_t)(cmd ^ 0xFFu);
    BSP_ISP_UART_FlushRx();
    BSP_ISP_UART_Send(pkt, 2);
}

/** @retval 1 = 收到 ACK(0x79); 0 = NACK 或超时 */
static uint8_t ISP_WaitAck(uint32_t timeout_ms)
{
    uint32_t tick = HAL_GetTick();
    uint8_t  b;

    while ((HAL_GetTick() - tick) < timeout_ms) {
        if (BSP_ISP_UART_RecvByte(&b, 50u)) {
            if (b == ISP_ACK)  return 1u;
            if (b == ISP_NACK) return 0u;
        }
    }
    return 0u;
}

static void ISP_SendAddr(uint32_t addr)
{
    uint8_t pkt[5];
    pkt[0] = (uint8_t)(addr >> 24);
    pkt[1] = (uint8_t)(addr >> 16);
    pkt[2] = (uint8_t)(addr >> 8);
    pkt[3] = (uint8_t)(addr);
    pkt[4] = (uint8_t)(pkt[0] ^ pkt[1] ^ pkt[2] ^ pkt[3]);
    BSP_ISP_UART_Send(pkt, 5);
}

/*==============================================================================
 * 底层: 单块写入 (最大 256 字节, 自动补齐到 ISP_WRITE_ALIGN)
 * @note  数据一律取自模块内的 s_buf —— 调用方必须先把待写数据放进 s_buf。
 *        (原实现带一个 data 形参却从不使用, 改成无参以免误用。)
 * @note  补齐到 8 字节: 同时满足 F1(4) 与 G0(8) 的对齐要求, 见 isp_config.h §3。
 *============================================================================*/
static uint8_t ISP_WriteChunk(uint32_t dst_addr, uint16_t len)
{
    uint8_t  len_byte;
    uint8_t  checksum;
    uint16_t padded = len;

    if (len == 0u || len > 256u) return 0u;
    if ((len % ISP_WRITE_ALIGN) != 0u) {
        padded = (uint16_t)(((len / ISP_WRITE_ALIGN) + 1u) * ISP_WRITE_ALIGN);
        if (padded > 256u) padded = 256u;
        if (padded > len) memset(&s_buf[len], 0xFF, (size_t)(padded - len));
    }

    ISP_SendCmd(ISP_CMD_WRITE);
    if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) return 0u;

    BSP_ISP_UART_FlushRx();
    ISP_SendAddr(dst_addr);
    if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) return 0u;

    len_byte = (uint8_t)((padded - 1u) & 0xFFu);
    checksum = len_byte;
    for (uint16_t i = 0; i < padded; i++) checksum ^= s_buf[i];

    BSP_ISP_UART_FlushRx();
    BSP_ISP_UART_SendByte(len_byte);
    BSP_ISP_UART_Send(s_buf, padded);
    BSP_ISP_UART_SendByte(checksum);

    return ISP_WaitAck(ISP_TIMEOUT_WRITE_MS);
}

/*==============================================================================
 * 底层: 单块回读 (AN3155 Read Memory 0x11), 结果放进 s_buf
 *   时序: 0x11+0xEE → ACK → addr(4)+XOR → ACK
 *         → (N-1) + (N-1)^0xFF → ACK → N 字节数据 → [可选尾随 ACK]
 * @note  长度字节后必须跟 (N-1)^0xFF 校验字节 (AN3155); 缺了会被 bootloader
 *        当成校验而回 NACK。尾随 ACK 有的器件发有的不发, 超时按正常处理,
 *        但收到 NACK 视为失败。
 *============================================================================*/
#if (ISP_VERIFY_ENABLED)

static uint8_t ISP_ReadChunk(uint32_t src_addr, uint16_t len)
{
    if (len == 0u || len > 256u) return 0u;

    ISP_SendCmd(ISP_CMD_READ);
    if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) return 0u;

    BSP_ISP_UART_FlushRx();
    ISP_SendAddr(src_addr);
    if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) return 0u;

    {
        uint8_t n1 = (uint8_t)((len - 1u) & 0xFFu);
        BSP_ISP_UART_FlushRx();
        BSP_ISP_UART_SendByte(n1);
        BSP_ISP_UART_SendByte((uint8_t)(n1 ^ 0xFFu));
    }
    if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) return 0u;

    {
        uint32_t tick = HAL_GetTick();
        uint16_t got  = 0u;
        while (got < len) {
            if (BSP_ISP_UART_RecvByte(&s_buf[got], 50u)) {
                got++;
            } else if ((HAL_GetTick() - tick) > ISP_TIMEOUT_READ_MS) {
                return 0u;
            }
        }
    }

    {
        uint8_t  b;
        uint32_t tick = HAL_GetTick();
        while ((HAL_GetTick() - tick) < 20u) {
            if (BSP_ISP_UART_RecvByte(&b, 5u)) {
                if (b == ISP_NACK) return 0u;
                if (b == ISP_ACK)  break;
            }
        }
    }
    return 1u;
}

/* 回读一块并与 SPI Flash 中的源镜像比对; 返回 1 = 一致 */
static uint8_t ISP_VerifyChunk(uint32_t dst_addr, uint16_t len)
{
    if (!ISP_ReadChunk(dst_addr, len)) {
        BSP_DBG_Send("[ISP] verify: read FAIL\r\n");
        return 0u;
    }
    GD25Q_ReadBuffer(s_ref, s_slot_base + FW_HEAD_SIZE + s_offset, len);
    if (memcmp(s_ref, s_buf, (size_t)len) != 0) {
        BSP_DBG_Send("[ISP] verify: DATA MISMATCH\r\n");
        return 0u;
    }
    return 1u;
}

#endif /* ISP_VERIFY_ENABLED */

/*==============================================================================
 * 底层: 读目标 bootloader 版本 (AN3155 0x01)
 *   响应: ACK | N (=后续字节数-1) | N+1 字节 | ACK
 *   返回的 buf[0] 即 bootloader 版本 (F1 典型 0x22, G0 典型 0x31/0x32)
 *============================================================================*/
static uint8_t ISP_ReadVersion(uint8_t *ver_out)
{
    uint8_t n = 0u;
    uint8_t buf[4];

    ISP_SendCmd(ISP_CMD_GET_VER);
    if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) return 0u;
    if (!BSP_ISP_UART_RecvByte(&n, ISP_TIMEOUT_ACK_MS)) return 0u;
    if ((uint16_t)n + 1u > (uint16_t)sizeof(buf)) return 0u;

    for (uint16_t i = 0; i < (uint16_t)n + 1u; i++) {
        if (!BSP_ISP_UART_RecvByte(&buf[i], ISP_TIMEOUT_ACK_MS)) return 0u;
    }
    if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) return 0u;

    *ver_out = buf[0];
    return 1u;
}

/*==============================================================================
 * 底层: 全片擦除 —— 两种命令都试
 *
 * ⚠ 关键差异 (这是从 G0 参考工程照搬过来会踩的坑):
 *   - 0x44 Extended Erase 是 bootloader **v3.0 以后**才有的命令 (G0/G4/L4 等)。
 *     STM32F103 的 bootloader 是 v2.2, **只认 0x43**, 发 0x44 会被 NACK。
 *   - 0x43 与 0x44 的"全片擦除"载荷都是 `0xFF 0xFF` + 校验 `0x00`
 *     (0xFFFF 作为"页数-1"表示擦全部, 此时不需要再逐页下发页号)。
 *   因此先读版本决定先试哪个, 另一个作为兜底, 两种目标都能烧。
 *============================================================================*/
static uint8_t ISP_EraseSend(uint8_t cmd, uint32_t timeout_ms)
{
    static const uint8_t mass[3] = { 0xFFu, 0xFFu, 0x00u };

    ISP_SendCmd(cmd);
    if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) return 0u;   /* 命令不被支持 → 这里就失败 */

    BSP_ISP_UART_FlushRx();
    BSP_ISP_UART_Send(mass, 3u);
    return ISP_WaitAck(timeout_ms);
}

static uint8_t ISP_EraseAll(void)
{
    /* bootloader >= v3.0 → 优先扩展擦除; 否则优先标准擦除 (F1) */
    if (s_bl_version >= 0x30u) {
        if (ISP_EraseSend(ISP_CMD_ERASE_EXT, ISP_TIMEOUT_ERASE_MS)) return 1u;
        return ISP_EraseSend(ISP_CMD_ERASE_STD, ISP_TIMEOUT_ERASE_MS);
    }
    if (ISP_EraseSend(ISP_CMD_ERASE_STD, ISP_TIMEOUT_ERASE_MS)) return 1u;
    return ISP_EraseSend(ISP_CMD_ERASE_EXT, ISP_TIMEOUT_ERASE_MS);
}

/*==============================================================================
 * 状态机步进
 *============================================================================*/
static void ISP_SetFail(void)
{
    /* ⚠ 失败路径也必须把"治具对被烧板 + 串口"恢复到空闲态, 否则会影响后续操作:
     *   - BOOT0 拉回低电平: 否则治具会一直把被烧板的 BOOT0 拉在高电平,
     *     被烧板停在 bootloader, 且相当于被外部持续驱动一个启动配置脚。
     *   - ISP 串口从 8E1 切回 8N1: 否则后面任何应用运行态(8N1)的收发都会因
     *     校验位不匹配而失败 —— 表现为"第一次烧录失败后, 之后全都不对"。
     * 刻意 **不** 额外复位被烧板: 让它在 bootloader 里等下一次 0x0112, 重试更直接。
     * (测径治具的 ISP_Connect 失败时不做恢复, 这里是刻意改进。) */
    ISP_Boot0Set(0);
    BSP_ISP_UART_SetParity(0u);

    s_state  = ISP_ST_FAIL;
    s_result = APP_STATUS_ERR_PROG;
}

/* 读取当前槽头部并校验。
 * @param max_len 该槽对应的目标区分区上限 (Bootloader 用 DUT_BOOT_SIZE,
 *                App 用 DUT_APP_SIZE) —— 防止超大镜像越过分区边界写坏另一段。
 * @retval 1 = 可用
 */
static uint8_t ISP_LoadImageHead(uint32_t slot_base, uint32_t max_len, uint32_t *p_len)
{
    uint8_t hdr[FW_HEAD_SIZE];
    GD25Q_ReadBuffer(hdr, slot_base, FW_HEAD_SIZE);

    uint32_t size = (uint32_t)hdr[FW_HEAD_OFF_SIZE]
                  | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 1u] << 8)
                  | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 2u] << 16)
                  | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 3u] << 24);
    uint8_t flag = hdr[FW_HEAD_OFF_FLAG];

    if (size == 0u || size > FW_MAX_IMAGE_SIZE || flag != FW_FLAG_COMPLETE) return 0u;
    if (size > max_len) return 0u;      /* 超出目标分区 → 拒绝, 不写 */

    *p_len = size;
    return 1u;
}

void ISP_Start(void)
{
    s_result   = APP_STATUS_ERR_PROG;
    s_offset   = 0;
    s_slot_base = FW_SLOT0_BASE;
    s_img_len  = 0;
    s_img_len_boot = 0;
    s_img_len_app  = 0;
    s_dst_addr = DUT_BOOT_ADDR;
    s_state    = ISP_ST_CONNECT;

    ISP_GpioInit();
    BSP_ISP_UART_SetParity(1u);     /* bootloader 是 8E1 */
}

uint8_t ISP_Busy(void)
{
    return (s_state != ISP_ST_IDLE && s_state != ISP_ST_OK && s_state != ISP_ST_FAIL) ? 1u : 0u;
}

uint8_t ISP_Result(void)
{
    return (s_state == ISP_ST_OK) ? APP_STATUS_OK : s_result;
}

void ISP_Step(void)
{
    switch (s_state) {

    /*── 1. 握手: BOOT0 拉高 + 复位 → 发 0x7F 等 ACK ──*/
    case ISP_ST_CONNECT: {
        ISP_Boot0Set(1);
        ISP_ResetSet(0);
        HAL_Delay(ISP_RESET_LOW_MS);
        ISP_ResetSet(1);
        HAL_Delay(ISP_BOOT_SETTLE_MS);

        BSP_ISP_UART_FlushRx();
        BSP_ISP_UART_SendByte(ISP_CMD_SYNC);
        if (!ISP_WaitAck(ISP_TIMEOUT_SYNC_MS)) {
            BSP_DBG_Send("[ISP] sync FAIL\r\n");
            ISP_SetFail(); break;
        }

        /* 读目标 bootloader 版本, 决定用 0x43 还是 0x44 擦除 */
        {
            uint8_t v = 0u;
            if (ISP_ReadVersion(&v)) s_bl_version = v;
            BSP_DBG_Send("[ISP] target BL ver=0x");
            BSP_DBG_SendHexByte(s_bl_version);
            BSP_DBG_Send("\r\n");
        }

        /* ⚠ 顺序很关键: **先把两段镜像都校验通过, 再擦被烧板**。
         * 若先擦后校验, 一旦镜像不合格 (大小超分区/完整标志缺失/槽为空),
         * 被烧板已经被擦空却什么都没写进去 —— 板子直接报废。
         * 本模块不依赖调用方做过前置检查, 自己先自证。 */
        if (!ISP_LoadImageHead(FW_SLOT0_BASE, DUT_BOOT_SIZE, &s_img_len_boot) ||
            !ISP_LoadImageHead(FW_SLOT1_BASE, DUT_APP_SIZE,  &s_img_len_app)) {
            BSP_DBG_Send("[ISP] image head invalid, ABORT before erase\r\n");
            ISP_SetFail(); break;
        }

        /* 全片擦除 (0x43 / 0x44 自适应, 见 ISP_EraseAll 注释) */
        if (!ISP_EraseAll()) {
            BSP_DBG_Send("[ISP] erase FAIL\r\n");
            ISP_SetFail(); break;
        }

        /* 进入写 Bootloader */
        s_img_len   = s_img_len_boot;
        s_slot_base = FW_SLOT0_BASE;
        s_dst_addr  = DUT_BOOT_ADDR;
        s_offset    = 0;
        s_state     = ISP_ST_WRITE_BOOT;
        break;
    }

    /*── 2. 逐块写 Bootloader ──*/
    case ISP_ST_WRITE_BOOT: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;
        if (remain == 0u) {
            /* Bootloader 写完 → (可选)回读校验 → 切到 App */
#if (ISP_VERIFY_ENABLED)
            s_offset = 0;
            s_state  = ISP_ST_VERIFY_BOOT;
#else
            s_img_len   = s_img_len_app;
            s_slot_base = FW_SLOT1_BASE;
            s_dst_addr  = DUT_APP_ADDR;
            s_offset    = 0;
            s_state     = ISP_ST_WRITE_APP;
#endif
            break;
        }
        uint16_t chunk = (remain > APP_PAGE_SIZE) ? (uint16_t)APP_PAGE_SIZE : (uint16_t)remain;
        GD25Q_ReadBuffer(s_buf, s_slot_base + FW_HEAD_SIZE + s_offset, chunk);
        if (!ISP_WriteChunk(s_dst_addr + s_offset, chunk)) { ISP_SetFail(); break; }
        s_offset += chunk;
        break;
    }

    /*── 3. 逐块写 App ──*/
    case ISP_ST_WRITE_APP: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;
        if (remain == 0u) {
#if (ISP_VERIFY_ENABLED)
            s_offset = 0;
            s_state  = ISP_ST_VERIFY_APP;
#else
            s_state = ISP_ST_GO;
#endif
            break;
        }

        uint16_t chunk = (remain > APP_PAGE_SIZE) ? (uint16_t)APP_PAGE_SIZE : (uint16_t)remain;
        GD25Q_ReadBuffer(s_buf, s_slot_base + FW_HEAD_SIZE + s_offset, chunk);
        if (!ISP_WriteChunk(s_dst_addr + s_offset, chunk)) { ISP_SetFail(); break; }
        s_offset += chunk;
        break;
    }

#if (ISP_VERIFY_ENABLED)
    /*── 2b/3b. 逐块回读校验 (仅 ISP_VERIFY_ENABLED=1) ──*/
    case ISP_ST_VERIFY_BOOT:
    case ISP_ST_VERIFY_APP: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;

        if (remain > 0u) {
            uint16_t chunk = (remain > APP_PAGE_SIZE) ? (uint16_t)APP_PAGE_SIZE : (uint16_t)remain;
            if (!ISP_VerifyChunk(s_dst_addr + s_offset, chunk)) { ISP_SetFail(); break; }
            s_offset += chunk;
            break;
        }

        /* 本段校验完 → 进入下一段 */
        if (s_state == ISP_ST_VERIFY_BOOT) {
            s_img_len   = s_img_len_app;
            s_slot_base = FW_SLOT1_BASE;
            s_dst_addr  = DUT_APP_ADDR;
            s_offset    = 0;
            s_state     = ISP_ST_WRITE_APP;
        } else {
            s_state = ISP_ST_GO;
        }
        break;
    }
#endif /* ISP_VERIFY_ENABLED */

    /*── 4. 跳转运行 ──*/
    case ISP_ST_GO: {
        ISP_SendCmd(ISP_CMD_GO);
        if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) { ISP_SetFail(); break; }
        BSP_ISP_UART_FlushRx();
        ISP_SendAddr(DUT_GO_ADDR);
        if (!ISP_WaitAck(ISP_TIMEOUT_ACK_MS)) { ISP_SetFail(); break; }

        ISP_Boot0Set(0);
        BSP_ISP_UART_SetParity(0u);     /* 应用运行态回到 8N1 */
        s_state  = ISP_ST_OK;
        s_result = APP_STATUS_OK;
        break;
    }

    default:
        break;
    }
}

/*==============================================================================
 * 0x000D 查询被烧板运行态固件版本
 *============================================================================*/
uint8_t ISP_QueryTargetVersion(uint8_t ver[3])
{
    if (ver == 0) return APP_STATUS_ERR_PARAM;

    ver[0] = 0x00u; ver[1] = 0xFFu; ver[2] = 0xFFu;    /* 失败默认值 (协议规定) */

#if (FOC_APP_VER_QUERY_ENABLED)
    /* TODO: 拿到被烧板应用层版本查询协议后在此实现:
     *   1) BSP_ISP_UART_SetParity(0);   // 应用运行态 8N1
     *   2) 按被烧板协议组帧发送
     *   3) 解析应答, 填入 ver[0..2]
     *   4) 返回 APP_STATUS_OK
     */
    return APP_STATUS_ERR_TIMEOUT;
#else
    /* 未配置被烧板版本查询协议 → 按协议返回失败码 0x0C, 版本号 00 FF FF */
    return APP_STATUS_ERR_TIMEOUT;
#endif
}
