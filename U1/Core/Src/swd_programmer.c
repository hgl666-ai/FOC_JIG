/**
  ******************************************************************************
  * @file    swd_programmer.c
  * @brief   FOC 板 SWD 烧录实现（SWCLK + SWDIO 两线，不用 NRST）
  *
  * 设计要点：
  *  - **非阻塞**：SWDP_Step() 每次只推进一步（一次握手 / 擦一个扇区 /
  *    写 128 字节 / 校验一块 / 复位运行），主循环里的 USB 与协议处理不受影响；
  *    F4 的 128KB 扇区擦除要 1.3s，所以擦除必须按扇区拆开，不能一次擦完。
  *  - **先校验后破坏**：两份镜像头在**擦除之前**读出来并校验，
  *    镜像缺失/不完整时一个扇区都不擦（沿用 ISP 版踩过的坑 7.6）。
  *  - **分区边界**：目标 F405 的扇区是 16/16/16/16/64/128/128/128 KB，
  *    分区必须落在扇区边界上，越界一律拒绝。
  ******************************************************************************
  */
#include <string.h>

#include "swd_programmer.h"
#include "swd_config.h"
#include "swd_phy.h"
#include "swd_dp.h"
#include "swd_memap.h"
#include "cortex_m.h"
#include "stm32f4_flash.h"
#include "flash_map.h"
#include "gd25q128.h"
#include "bsp_uart.h"

/* 一步写/校验多少字节（128B = 32 字；1MHz SWCLK 下约 10ms，够短） */
#define SWDP_CHUNK_BYTES    128u
#define SWDP_CHUNK_WORDS    (SWDP_CHUNK_BYTES / 4u)

typedef enum {
    SWDP_ST_IDLE = 0,
    SWDP_ST_CONNECT,      /* 线复位 + JTAG→SWD 切换 + IDCODE + 上电 + 选 AP */
    SWDP_ST_HALT,         /* 停内核 + 冻结看门狗 + 认型号 + 校验镜像头 */
    SWDP_ST_ERASE,        /* 按扇区擦除（一次一个） */
    SWDP_ST_PROGRAM,      /* 按块写（一次 128B） */
    SWDP_ST_VERIFY,       /* 按块回读校验 */
    SWDP_ST_GO,           /* 清 PG + SYSRESETREQ 复位运行 */
    SWDP_ST_OK,
    SWDP_ST_FAIL
} SWDP_State_t;

static SWDP_State_t s_state = SWDP_ST_IDLE;
static uint8_t  s_result = APP_STATUS_ERR_PROG;

/* 镜像信息 */
static uint32_t s_img_len_boot = 0u;
static uint32_t s_img_len_app  = 0u;

/* 当前处理的镜像 */
static uint8_t  s_phase      = 0u;       /* 0 = Bootloader, 1 = App */
static uint32_t s_slot_base  = 0u;       /* 该镜像在治具 SPI Flash 里的槽基址 */
static uint32_t s_dst_addr   = 0u;       /* 该镜像要写到被烧板的绝对地址 */
static uint32_t s_img_len    = 0u;
static uint32_t s_offset     = 0u;

/* 擦除进度 */
static uint32_t s_sector      = 0u;

/* 诊断 */
static uint32_t s_idcode     = 0u;
static uint16_t s_dev_id     = 0u;
static uint8_t  s_last_err   = 0u;

static uint8_t  s_buf[SWDP_CHUNK_BYTES];

/* ------------------------------------------------------------------ */
/* 小工具                                                             */
/* ------------------------------------------------------------------ */
static uint32_t swdp_min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static void swdp_fail(uint8_t status, uint8_t raw_err)
{
    s_last_err = raw_err;
    s_result   = status;
    s_state    = SWDP_ST_FAIL;
    BSP_DBG_Send("[SWD] FAIL step=");
    BSP_DBG_SendHexByte(s_last_err);
    BSP_DBG_Send("\r\n");
}

/* 读槽头并校验：大小非 0、不超分区上限、完整标志为 0xA5 */
static uint8_t swdp_load_image_head(uint32_t slot_base, uint32_t max_len, uint32_t *p_len)
{
    uint8_t  hdr[FW_HEAD_SIZE];
    uint32_t size;

    GD25Q_ReadBuffer(hdr, slot_base, (uint16_t)FW_HEAD_SIZE);

    size = (uint32_t)hdr[FW_HEAD_OFF_SIZE]
         | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 1u] << 8)
         | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 2u] << 16)
         | ((uint32_t)hdr[FW_HEAD_OFF_SIZE + 3u] << 24);

    if (hdr[FW_HEAD_OFF_FLAG] != FW_FLAG_COMPLETE) {
        return 0u;                        /* 不完整 */
    }
    if (size == 0u || size > max_len) {
        return 0u;                        /* 空或越过分区上限 */
    }
    *p_len = size;
    return 1u;
}

/* 切换当前处理的镜像（写与校验两阶段共用） */
static void swdp_select_image(uint8_t which)
{
    s_phase = which;
    if (which == 0u) {
        s_slot_base = FW_SLOT0_BASE;
        s_dst_addr  = DUT_BOOT_ADDR;
        s_img_len   = s_img_len_boot;
    } else {
        s_slot_base = FW_SLOT1_BASE;
        s_dst_addr  = DUT_APP_ADDR;
        s_img_len   = s_img_len_app;
    }
    s_offset = 0u;
}

/* ------------------------------------------------------------------ */
/* 对外 API                                                           */
/* ------------------------------------------------------------------ */
void SWDP_Init(void)
{
    SWD_PhyInit();
    s_state  = SWDP_ST_IDLE;
    s_result = APP_STATUS_ERR_PROG;
}

void SWDP_Start(void)
{
    s_result       = APP_STATUS_ERR_PROG;
    s_last_err     = 0u;
    s_idcode       = 0u;
    s_dev_id       = 0u;
    s_img_len_boot = 0u;
    s_img_len_app  = 0u;
    s_offset       = 0u;
    s_phase        = 0u;
    s_sector       = 0u;
    s_state        = SWDP_ST_CONNECT;
}

uint8_t SWDP_Busy(void)
{
    return ((s_state != SWDP_ST_IDLE) &&
            (s_state != SWDP_ST_OK) &&
            (s_state != SWDP_ST_FAIL)) ? 1u : 0u;
}

uint8_t SWDP_Result(void)
{
    return (s_state == SWDP_ST_OK) ? APP_STATUS_OK : s_result;
}

uint32_t SWDP_LastIdcode(void)  { return s_idcode; }
uint16_t SWDP_LastDevId(void)   { return s_dev_id; }
uint32_t SWDP_LastOffset(void)  { return s_offset; }
uint8_t  SWDP_LastSwdError(void){ return s_last_err; }

void SWDP_Step(void)
{
    int r;

    switch (s_state) {

    /* ── 1. 连接：线复位 + 0xE79E 切换 + 读 IDCODE + 上电 + 选 AP ── */
    case SWDP_ST_CONNECT:
        r = SWD_Connect(&s_idcode);
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        r = SWD_PowerUp();
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        r = SWD_MemInit();
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        BSP_DBG_Send("[SWD] idcode=");
        BSP_DBG_SendHexByte((uint8_t)(s_idcode >> 24));
        BSP_DBG_SendHexByte((uint8_t)(s_idcode >> 16));
        BSP_DBG_SendHexByte((uint8_t)(s_idcode >> 8));
        BSP_DBG_SendHexByte((uint8_t)s_idcode);
        BSP_DBG_Send("\r\n");
        s_state = SWDP_ST_HALT;
        break;

    /* ── 2. 停内核 + 冻结看门狗 + 认型号 + 校验镜像头（擦除之前！） ── */
    case SWDP_ST_HALT:
        r = CM_Halt();
        if (r != SWD_OK) {
            /* 没 NRST 时最常见的失败：被烧板程序把 SWD 脚占了或进了低功耗 */
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        (void)CM_FreezeWatchdogs();       /* 冻结 IWDG/WWDG，防止擦写期间被复位 */
        (void)CM_ReadDevId(&s_dev_id);

        if (swdp_load_image_head(FW_SLOT0_BASE, DUT_BOOT_SIZE, &s_img_len_boot) == 0u) {
            swdp_fail(APP_STATUS_ERR_INCOMPLETE, 0u);
            break;
        }
        if (swdp_load_image_head(FW_SLOT1_BASE, DUT_APP_SIZE, &s_img_len_app) == 0u) {
            swdp_fail(APP_STATUS_ERR_INCOMPLETE, 0u);
            break;
        }

        /* 擦除计划：Boot 与 App 覆盖到的扇区并集（两个分区连续，见 flash_map.h 的静态断言） */
        r = F4_SectorOf(DUT_BOOT_ADDR, &s_sector);
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        s_state = SWDP_ST_ERASE;
        break;

    /* ── 3. 擦除：一步一个扇区（128KB 扇区要约 1.3s，不能一次全擦） ── */
    case SWDP_ST_ERASE: {
        uint32_t last_sector;
        uint32_t app_end = DUT_APP_ADDR + s_img_len_app - 1u;

        if (F4_SectorOf(app_end, &last_sector) != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PARAM, 0u);
            break;
        }
        r = F4_EraseSector(s_sector);
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        if (s_sector >= last_sector) {
            r = F4_ProgramBegin();
            if (r != SWD_OK) {
                swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
                break;
            }
            swdp_select_image(0u);
            s_state = SWDP_ST_PROGRAM;
        } else {
            s_sector++;
        }
        break;
    }

    /* ── 4. 写：一步 128 字节 ── */
    case SWDP_ST_PROGRAM: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;

        if (remain == 0u) {
            if (s_phase == 0u) {
                swdp_select_image(1u);            /* 接着写 App */
            } else {
                (void)F4_ProgramEnd();            /* 写完清 PG */
                swdp_select_image(0u);
                s_state = SWDP_ST_VERIFY;
            }
            break;
        }
        {
            uint16_t chunk = (uint16_t)swdp_min_u32(remain, (uint32_t)SWDP_CHUNK_BYTES);
            GD25Q_ReadBuffer(s_buf, s_slot_base + FW_HEAD_SIZE + s_offset, chunk);
            r = F4_ProgramWords(s_dst_addr + s_offset, s_buf, (uint32_t)chunk);
            if (r != SWD_OK) {
                swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
                break;
            }
            s_offset += chunk;
        }
        break;
    }

    /* ── 5. 回读校验：SWD 的读回是目标 AHB 上的真实内容，可信 ── */
    case SWDP_ST_VERIFY: {
        uint32_t remain = (s_offset < s_img_len) ? (s_img_len - s_offset) : 0u;

        if (remain == 0u) {
            if (s_phase == 0u) {
                swdp_select_image(1u);
            } else {
                s_state = SWDP_ST_GO;
            }
            break;
        }
        {
            uint16_t chunk = (uint16_t)swdp_min_u32(remain, (uint32_t)SWDP_CHUNK_BYTES);
            GD25Q_ReadBuffer(s_buf, s_slot_base + FW_HEAD_SIZE + s_offset, chunk);
            r = F4_VerifyImage(s_dst_addr + s_offset, s_buf, (uint32_t)chunk);
            if (r != SWD_OK) {
                swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
                break;
            }
            s_offset += chunk;
        }
        break;
    }

    /* ── 6. 复位运行（不需要 NRST：用 AIRCR 的 SYSRESETREQ） ── */
    case SWDP_ST_GO:
        r = CM_ResetAndRun();
        if (r != SWD_OK) {
            swdp_fail(APP_STATUS_ERR_PROG, (uint8_t)(-r));
            break;
        }
        BSP_DBG_Send("[SWD] done, target running\r\n");
        s_result = APP_STATUS_OK;
        s_state  = SWDP_ST_OK;
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* 0x000D：读被烧板里的固件版本号                                      */
/* ------------------------------------------------------------------ */
uint8_t SWDP_QueryTargetVersion(uint8_t ver[3])
{
    uint32_t id  = 0u;
    uint32_t w   = 0u;

    if (ver == 0) {
        return APP_STATUS_ERR_PARAM;
    }
    if (SWD_Connect(&id) != SWD_OK) {
        return APP_STATUS_ERR_TIMEOUT;
    }
    if (SWD_PowerUp() != SWD_OK) {
        return APP_STATUS_ERR_TIMEOUT;
    }
    if (SWD_MemInit() != SWD_OK) {
        return APP_STATUS_ERR_TIMEOUT;
    }
    /* 版本号放在被烧板 App 分区的哪个偏移由 FOC 工程决定（见 flash_map.h 的 DUT_VER_OFFSET） */
    if (SWD_MemRead32(DUT_APP_ADDR + DUT_VER_OFFSET, &w) != SWD_OK) {
        return APP_STATUS_ERR_TIMEOUT;
    }
    if (w == 0xFFFFFFFFu) {
        /* 读得到但全 0xFF：被烧板 App 区是空的（没烧过或被擦过） */
        ver[0] = 0x00u;
        ver[1] = 0xFFu;
        ver[2] = 0xFFu;
        return APP_STATUS_ERR_NO_FW;
    }
    ver[0] = (uint8_t)(w & 0xFFu);
    ver[1] = (uint8_t)((w >> 8) & 0xFFu);
    ver[2] = (uint8_t)((w >> 16) & 0xFFu);
    return APP_STATUS_OK;
}
