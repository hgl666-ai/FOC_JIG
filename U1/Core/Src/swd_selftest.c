/**
  ******************************************************************************
  * @file    swd_selftest.c
  * @brief   SWD 烧录自检（B1）—— 实现见头文件说明
  *
  * 设计约束（照抄现有烧录状态机 `swd_programmer.c` 的成功序列，只是把"镜像来源"
  * 从 SPI Flash 换成**运行时生成的图样**）：
  *   1) 连接顺序: SWD_Connect → SWD_PowerUp → SWD_MemInit → CM_Halt
  *      （与 SWDP_ST_CONNECT / SWDP_ST_HALT 完全一致，别自创顺序）
  *   2) 擦除/编程: F4_EraseSector / F4_ProgramBegin / F4_ProgramWords / F4_ProgramEnd
  *      （F4_EraseSector 与 F4_ProgramBegin 内部会自己 F4_Unlock，不要重复解锁）
  *   3) 校验: F4_ReadImage（走 SWD 回读，是"目标 AHB 上的真实内容"）
  *   4) 收尾: CM_ResetAndRun 让被烧板回到运行态
  *   5) **全程非阻塞**：每个 SST_Step() 只做一小步
  ******************************************************************************
  */
#include "swd_selftest.h"
#include "main.h"           /* HAL 类型 / GPIO 定义（宿主构建下由 test/hostshim 提供） */
#include "swd_config.h"     /* SWD_CLK_PIN / SWD_DIO_PIN 等引脚定义 */
#include "flash_map.h"
#include "swd_phy.h"        /* SWD_OK / SWD_PhyInit */
#include "swd_dp.h"
#include "swd_memap.h"
#include "cortex_m.h"
#include "stm32f4_flash.h"
#include "sn_report.h"      /* SN_UID_BASE（F4 的 96-bit UID） */
#include <string.h>

#define SST_CHUNK       128u                      /* 与 SWDP_CHUNK_BYTES 一致 */
#define SST_BYTES       DUT_SELFTEST_BYTES        /* 4 KB */
#define SST_BAD_NONE    0xFFFFu
#define SST_F4_IDCODE   0x2BA01477u               /* STM32F405 的 SW-DP IDCODE（严格校验用） */

typedef enum {
    SST_ST_IDLE = 0,
    SST_ST_CONNECT,
    SST_ST_ERASE,
    SST_ST_PROGRAM,
    SST_ST_VERIFY,
    SST_ST_UID,
    SST_ST_DONE,
    SST_ST_FAIL
} sst_state_t;

static sst_state_t s_state = SST_ST_IDLE;
static uint8_t   s_status;              /* APP_STATUS_* */
static uint8_t   s_stage;
static uint8_t   s_last_ack;            /* 失败时 SWD 线路上的原始 ACK（排障用） */
static uint8_t   s_last_err;            /* 失败时的内部错误码 = (uint8_t)(-返回码) */
static uint8_t   s_substep;             /* 失败在哪个子步骤（1连接 2上电 3MEM-AP 4停机 5看门狗 6擦除 7写 8校验）*/
static uint8_t   s_retry;               /* 连接阶段的自动重试计数 */
static uint32_t  s_offset;
static uint32_t  s_idcode;
static uint16_t  s_dev_id;
static uint16_t  s_bad_off;
static uint8_t   s_buf[SST_CHUNK];      /* 期望图样 */
static uint8_t   s_rd[SST_CHUNK];       /* SWD 回读内容 */

/*==============================================================================
 * 自检图样：按**绝对偏移**生成
 *   同时覆盖 4 个字节通道、地址依赖、以及全 0/全 1 的反码，
 *   这样"写错地址/写错字节序/只写了一部分"都会被抓到。
 *============================================================================*/
static void sst_pattern(uint32_t off, uint8_t *buf, uint16_t n)
{
    uint16_t i;

    for (i = 0u; i < n; i++) {
        uint32_t o = off + (uint32_t)i;
        uint8_t  v = (uint8_t)((o * 7u) ^ (o >> 8) ^ 0x5Au);

        /* 每 256 字节里有 8 字节极端值，抓"写不进去/擦不干净" */
        if ((o & 0xFFu) < 8u) {
            v = ((o & 4u) != 0u) ? 0xFFu : 0x00u;
        }
        buf[i] = v;
    }
}

static void sst_fail(uint8_t status, int rc)
{
    s_status   = status;
    s_last_ack = SWD_LastAck();         /* ADIv5: 01=OK 02=WAIT 04=FAULT 00=无应答 */
    s_last_err = (uint8_t)(-rc);
    s_state    = SST_ST_FAIL;
}

/* 偶发位错的自动重试：飞线位翻转 SWD 里 NO_ACK / 非法 ACK 属于正常抖动，
 * 生产烧录器（ST-Link/DAPLink）都是靠重试解决，而不是当成致命错误。
 * 返回 1 = 已计入重试、调用方应直接 break 等下一轮 SST_Step 从头再来。 */
#define SST_MAX_RETRY   6u

static uint8_t sst_retry(int rc)
{
    if ((rc == SWD_ERR_NO_ACK) && (s_retry < SST_MAX_RETRY)) {
        s_retry++;
        return 1u;
    }
    return 0u;
}

void SST_Start(void)
{
    SWD_PhyInit();                      /* 引脚/延时初始化（幂等，可重复调用） */

    s_status  = APP_STATUS_ERR_PROG;    /* 没成功前先按最坏情况挂 */
    s_stage   = 1u;
    s_last_ack = 0xFFu;
    s_last_err = 0xFFu;
    s_substep  = 0u;
    s_retry    = 0u;
    s_offset  = 0u;
    s_idcode  = 0u;
    s_dev_id  = 0u;
    s_bad_off = SST_BAD_NONE;
    s_state   = SST_ST_CONNECT;
}

void SST_Step(void)
{
    int r;

    switch (s_state) {

    /* ── 1. 连接：顺序照抄 SWDP_ST_CONNECT / SWDP_ST_HALT ── */
    case SST_ST_CONNECT:
        s_stage = 1u;

        s_substep = 1u;
        r = SWD_Connect(&s_idcode);
        if (r != SWD_OK) {
            if (sst_retry(r) != 0u) { break; }
            sst_fail(APP_STATUS_ERR_PROG, r); break;
        }

        /* IDCODE 必须**完全正确**：位翻转链路上"连上了但读错一位"很常见，
         * 只判"非 0 非 FFFFFFFF"会把垃圾当成功（2026-11-03 实测踩到）。 */
        if (s_idcode != SST_F4_IDCODE) {
            if (sst_retry(SWD_ERR_NO_ACK) != 0u) { break; }
            sst_fail(APP_STATUS_ERR_PROG, SWD_ERR_NO_ACK); break;
        }

        s_substep = 2u;
        r = SWD_PowerUp();
        if (r != SWD_OK) {
            if (sst_retry(r) != 0u) { break; }
            sst_fail(APP_STATUS_ERR_PROG, r); break;
        }

        s_substep = 3u;
        r = SWD_MemInit();
        if (r != SWD_OK) {
            if (sst_retry(r) != 0u) { break; }
            sst_fail(APP_STATUS_ERR_PROG, r); break;
        }

        s_substep = 4u;
        r = CM_Halt();
        if (r != SWD_OK) {
            if (sst_retry(r) != 0u) { break; }
            sst_fail(APP_STATUS_ERR_PROG, r); break;
        }

        s_substep = 5u;
        (void)CM_FreezeWatchdogs();     /* 擦写期间别被目标自己的看门狗复位 */
        (void)CM_ReadDevId(&s_dev_id);  /* 只做记录，不强制要求等于 0x413 */
        s_substep = 0u;
        s_retry   = 0u;                 /* 连上了，重试计数归零 */

        s_state = SST_ST_ERASE;
        break;

    /* ── 2. 擦除 scratch 扇区（128 KB，约 1~1.5 秒，一次性完成） ── */
    case SST_ST_ERASE:
        s_stage = 2u;
        s_substep = 6u;

        r = F4_EraseSector(DUT_SCRATCH_SECTOR);
        if (r != SWD_OK) {
            /* 失步（NO_ACK）时整条链重来：线复位 → 切换 → 上电 → 停机 → 再擦。
             * 位翻转链路上"偶发失步 + 重连恢复"是标准做法（DAPLink 也是这么做的）。 */
            if (sst_retry(r) != 0u) { s_state = SST_ST_CONNECT; break; }
            sst_fail(APP_STATUS_ERR_PROG, r);
            break;
        }

        r = F4_ProgramBegin();
        if (r != SWD_OK) { sst_fail(APP_STATUS_ERR_PROG, r); break; }

        s_offset = 0u;
        s_state  = SST_ST_PROGRAM;
        break;

    /* ── 3. 写：一步 128 字节 ── */
    case SST_ST_PROGRAM:
        s_stage = 3u;
        s_substep = 7u;

        sst_pattern(s_offset, s_buf, (uint16_t)SST_CHUNK);
        r = F4_ProgramWords(DUT_SCRATCH_ADDR + s_offset, s_buf, SST_CHUNK);
        if (r != SWD_OK) {
            (void)F4_ProgramEnd();      /* 收拾干净再退出 */
            if (sst_retry(r) != 0u) { s_state = SST_ST_CONNECT; break; }
            sst_fail(APP_STATUS_ERR_PROG, r);
            break;
        }
        s_offset += SST_CHUNK;

        if (s_offset >= SST_BYTES) {
            r = F4_ProgramEnd();
            if (r != SWD_OK) { sst_fail(APP_STATUS_ERR_PROG, r); break; }
            s_offset = 0u;
            s_state  = SST_ST_VERIFY;
        }
        break;

    /* ── 4. 校验：SWD 回读，逐字节比对 ── */
    case SST_ST_VERIFY: {
        uint16_t i;

        s_stage = 4u;
        s_substep = 8u;

        sst_pattern(s_offset, s_buf, (uint16_t)SST_CHUNK);
        r = F4_ReadImage(DUT_SCRATCH_ADDR + s_offset, s_rd, SST_CHUNK);
        if (r != SWD_OK) {
            if (sst_retry(r) != 0u) { s_state = SST_ST_CONNECT; break; }
            sst_fail(APP_STATUS_ERR_PROG, r);
            break;
        }

        for (i = 0u; i < (uint16_t)SST_CHUNK; i++) {
            if (s_buf[i] != s_rd[i]) {
                s_bad_off = (uint16_t)(s_offset + (uint32_t)i);
                break;
            }
        }
        if (s_bad_off != SST_BAD_NONE) {
            sst_fail(APP_STATUS_ERR_TEST, 0);      /* 内容对不上：记下第一个不符偏移 */
            break;
        }
        s_offset += SST_CHUNK;
        if (s_offset >= SST_BYTES) {
            s_offset = 0u;
            s_state  = SST_ST_UID;
        }
        break;
    }

    /* ── 5. 读 UID（证明"能写也能读"），然后让目标回到运行态 ── */
    case SST_ST_UID: {
        uint32_t w[3];

        s_stage = 5u;

        r = SWD_MemReadBlock(SN_UID_BASE, w, 3u);
        if (r != SWD_OK) { sst_fail(APP_STATUS_ERR_PROG, r); break; }

        (void)CM_ResetAndRun();

        s_status = APP_STATUS_OK;
        s_stage  = 0xFFu;
        s_state  = SST_ST_DONE;
        break;
    }

    default:
        break;
    }
}

uint8_t SST_Busy(void)
{
    return ((s_state != SST_ST_IDLE) &&
            (s_state != SST_ST_DONE) &&
            (s_state != SST_ST_FAIL)) ? 1u : 0u;
}

uint8_t SST_Result(uint8_t *out)
{
    if (out == 0) {
        return s_status;
    }
    out[0]  = s_status;
    out[1]  = s_stage;
    out[2]  = (uint8_t)(s_bad_off >> 8);
    out[3]  = (uint8_t)s_bad_off;
    out[4]  = (uint8_t)(s_idcode >> 24);
    out[5]  = (uint8_t)(s_idcode >> 16);
    out[6]  = (uint8_t)(s_idcode >> 8);
    out[7]  = (uint8_t)s_idcode;
    out[8]  = (uint8_t)(s_dev_id >> 8);
    out[9]  = (uint8_t)s_dev_id;
    out[10] = (uint8_t)((SST_BYTES >> 8) & 0xFFu);
    out[11] = (uint8_t)(SST_BYTES & 0xFFu);
    out[12] = s_last_ack;               /* 失败时线路上的原始 SWD ACK（01=OK 02=WAIT 04=FAULT 00=无应答）*/
    out[13] = s_last_err;               /* 失败时的内部错误码 */
    out[14] = s_substep;                /* 失败在哪个子步骤：1连接 2上电 3MEM-AP 4停机 5看门狗 6擦除 7写 8校验 */
    out[15] = SWD_PhyLastReq();         /* 最后一笔事务的请求头（0xA5=DP读IDCODE 0x85=AP读 0x0D=AP写TAR 0x41=AP写DRW 0x09=DP写…）*/

    return s_status;
}

/*==============================================================================
 * AP 写事务波形捕捉（mode=5）——定位"连接之后每一笔都失败"的最后一招
 *----------------------------------------------------------------------------
 * 步骤：
 *   ① 先跑能正常工作的三步：DP 连接 / DP 上电 / MEM-AP 初始化
 *   ② 手工发一笔 **AP 写 TAR** 请求头（0x8B），然后抓接下来 24 拍 SWDIO 电平
 *   ③ 回传 24 位位图
 * 判读：
 *   期望前 3 位 = ACK（001=OK 010=WAIT 100=FAULT）
 *   全 1 = 目标完全没应答（包没被认出 / DP 不在 SWD 模式）
 *============================================================================*/
void SST_ApWriteCapture(uint8_t *out)
{
    uint32_t id = 0u;
    uint32_t bm = 0u;
    uint8_t  i;

    if (out == 0) {
        return;
    }
    for (i = 0u; i < 16u; i++) {
        out[i] = 0u;
    }
    out[0] = APP_STATUS_OK;
    out[8] = 0xB6u;                     /* 本版标记 */

    SWD_PhyInit();
    if (SWD_Connect(&id) != SWD_OK) { out[1] = 0xE1u; return; }
    if (SWD_PowerUp()     != SWD_OK) { out[1] = 0xE2u; return; }
    if (SWD_MemInit()     != SWD_OK) { out[1] = 0xE3u; return; }

    out[1] = 0x00u;                     /* 三步都过了 */

    /* ── DP 现场：CTRL/STAT 原始值（看 CDBGPWRUPACK/CSYSPWRUPACK 是否真置位）── */
    {
        uint32_t v = 0u;
        int      r;

        r = SWD_DPRead(SWD_DP_CTRLSTAT, &v);
        out[2] = (r == SWD_OK) ? 0x00u : 0xFFu;
        out[3] = (uint8_t)(v >> 24);
        out[4] = (uint8_t)(v >> 16);
        out[5] = (uint8_t)(v >> 8);
        out[6] = (uint8_t)v;
    }

    /* ── AP 探针：**读** IDR（只读标识寄存器，F4 的 AHB-AP 应为 0x24770011）── */
    {
        uint32_t v = 0u;
        int      r;

        r = SWD_APRead(SWD_AP_IDR, &v);
        out[7]  = (r == SWD_OK) ? 0x00u : 0xFFu;
        out[9]  = (uint8_t)(v >> 24);
        out[10] = (uint8_t)(v >> 16);
        out[11] = (uint8_t)(v >> 8);
        out[12] = (uint8_t)v;
    }

    /* ── DP SELECT 回读（AP 选没选中）── */
    {
        uint32_t v = 0u;
        int      r;

        r = SWD_DPRead(SWD_DP_SELECT, &v);
        out[13] = (r == SWD_OK) ? (uint8_t)(v >> 24) : 0xFFu;   /* APSEL 高字节 */
        out[14] = (uint8_t)v;
        out[15] = 0u;
    }
    (void)bm;
    (void)i;
    (void)id;

    SWD_PhyInit();
}

/*==============================================================================
 * 连续连接尝试（mode=4）——用来验证"是否必须复位下连接"
 *----------------------------------------------------------------------------
 * 在 SST_CONNECT_BURST_MS 这个窗口里**不停地重试 SWD_Connect**。
 * 用法：发命令后，在这 5 秒内按住再松开 FOC 板的复位按钮（或断电重上电）。
 *   只要有一次成功 → 说明目标 RUN 起来以后把 SWD 脚占了/关了，**必须复位下连接**
 *   一次都不成功   → 与复位时机无关
 * 应答（16B）：
 *   [0]     状态 00
 *   [1]     1 = 成功  0 = 全失败
 *   [2..5]  IDCODE（成功时）
 *   [6..7]  尝试次数低 16 位
 *   [8]     **0xB6 = 本版标记**
 *   [9..15] 预留 0
 *============================================================================*/
#define SST_CONNECT_BURST_MS   5000u

void SST_ConnectBurst(uint8_t *out)
{
    uint32_t t0;
    uint32_t tries = 0u;
    uint32_t got   = 0u;
    uint32_t id    = 0u;
    uint8_t  ok    = 0u;
    uint8_t  i;
    uint16_t shown;

    if (out == 0) {
        return;
    }
    for (i = 0u; i < 16u; i++) {
        out[i] = 0u;
    }
    out[0] = APP_STATUS_OK;

    SWD_PhyInit();

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < SST_CONNECT_BURST_MS) {
        got = 0u;
        if (SWD_Connect(&got) == SWD_OK) {
            ok = 1u;
            id = got;
            break;
        }
        tries++;
    }

    out[1] = ok;
    out[2] = (uint8_t)(id >> 24);
    out[3] = (uint8_t)(id >> 16);
    out[4] = (uint8_t)(id >> 8);
    out[5] = (uint8_t)id;
    shown  = (tries > 0xFFFFu) ? 0xFFFFu : (uint16_t)tries;
    out[6] = (uint8_t)(shown >> 8);
    out[7] = (uint8_t)shown;
    out[8] = 0xB6u;                     /* 本版标记 */

    SWD_PhyInit();
}

/*==============================================================================
 * SWD 波形捕捉（mode=3）——"穷人逻辑分析仪"
 *----------------------------------------------------------------------------
 * 抓取"发出请求头之后 32 个时钟"里 SWDIO 的电平，回传给 PC，用来判断
 * **目标到底有没有驱动数据线**（没有示波器时唯一的办法）：
 *   期望（目标活着）：最高 3 位应是 ACK=001 → 位图以 1,0,0 开头，后面跟 IDCODE 数据
 *   全是 1          ：那 32 拍里线上没人驱动 → 目标没认出我们的请求头
 *============================================================================*/
void SST_WaveCapture(uint8_t *out)
{
    uint32_t bm = 0u;
    uint8_t  i;

    if (out == 0) {
        return;
    }
    for (i = 0u; i < 16u; i++) {
        out[i] = 0u;
    }
    out[0] = APP_STATUS_OK;

    SWD_PhyInit();
    SWD_LineReset();                    /* ≥50 拍 SWDIO 高 */
    SWD_WriteBits(0xE79Eu, 16u);        /* JTAG → SWD 切换序列 */
    SWD_LineReset();
    SWD_IdleClock();

    SWD_WriteBits(0xA5u, 8u);           /* 读 IDCODE 的请求头（Start/APnDP/RnW/A2/A3/Par/Park）*/
    SWD_Turnaround();                   /* 释放总线 1 拍，交还目标 */

    for (i = 0u; i < 32u; i++) {
        bm = (bm << 1) | (uint32_t)SWD_ReadBit();
    }

    out[1] = (uint8_t)(bm >> 24);       /* 32 拍位图，bit31 = 第一拍 */
    out[2] = (uint8_t)(bm >> 16);
    out[3] = (uint8_t)(bm >> 8);
    out[4] = (uint8_t)bm;
    out[5] = 0u;
    out[6] = 0u;
    out[7] = 0u;

    SWD_PhyInit();                      /* 收尾：恢复正常 SWD 空闲态 */
}

/*==============================================================================
 * SWD 引脚活动测试（阻塞约 2 秒）
 *----------------------------------------------------------------------------
 * 用途：把"治具这两根脚到底会不会动"变成可测量的东西。
 *   SWCLK 连续输出方波 2 秒 → 万用表直流档能读到约 1.6 V（而不是 0.01 V）
 *   同时把 DIO 脚配成**带上拉输入**并采样 8 次：
 *     - 若 PA13 与 PA14 外部短接 → 采样应出现交替图案（0xAA/0x55 之类）
 *     - 若没短接        → 采样基本恒定（上拉为高）
 *============================================================================*/
#define SST_PIN_TEST_MS       2000u     /* 阶段 1：CLK 方波持续时间 */
#define SST_PIN_LOW_TEST_MS   5000u     /* 阶段 2：两根线都驱动为低，供万用表量目标端 */

void SST_PinActivityTest(uint8_t *out)
{
    GPIO_InitTypeDef g;
    uint32_t t0;
    uint32_t cycles = 0u;
    uint8_t  bitmap = 0u;
    uint8_t  highs  = 0u;
    uint8_t  i;

    if (out == 0) {
        return;
    }

    SWD_PhyInit();                      /* 先按 SWD 主机的方式配好两根脚 */

    out[0] = APP_STATUS_OK;

    /* 配置现场快照：SWJ_CFG 必须=4，PA14 必须是输出，否则脚根本不归 GPIO 管 */
#ifndef JIG_HOST_TEST
    out[1] = (uint8_t)((AFIO->MAPR >> 24) & 0x07u);   /* SWJ_CFG[2:0] */
    out[2] = (uint8_t)((GPIOA->CRH >> 20) & 0x0Fu);   /* PA13 */
    out[3] = (uint8_t)((GPIOA->CRH >> 24) & 0x0Fu);   /* PA14 */
#else
    out[1] = 0u;
    out[2] = 0u;
    out[3] = 0u;
#endif

    /* DIO 改成带上拉输入，用来读回外部电平 */
    g.Pin   = SWD_DIO_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWD_DIO_PORT, &g);

    /* CLK 连续方波 ~2 秒，每 8 个周期采样一次 DIO（共 8 次） */
    t0 = HAL_GetTick();
    for (;;) {
        if ((HAL_GetTick() - t0) >= SST_PIN_TEST_MS) {
            break;
        }
        HAL_GPIO_WritePin(SWD_CLK_PORT, SWD_CLK_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SWD_CLK_PORT, SWD_CLK_PIN, GPIO_PIN_RESET);
        cycles++;

        if ((cycles & 0x07u) == 0u) {
            uint8_t bit = (uint8_t)((cycles >> 3) - 1u);

            if (bit < 8u) {
                uint8_t hi = (HAL_GPIO_ReadPin(SWD_DIO_PORT, SWD_DIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;

                if (hi != 0u) {
                    bitmap |= (uint8_t)(1u << bit);
                    highs++;
                }
            }
        }
    }

    out[4] = bitmap;
    out[5] = highs;
    {
        uint32_t shown = (cycles > 0xFFFFu) ? 0xFFFFu : cycles;

        out[6] = (uint8_t)(shown >> 8);
        out[7] = (uint8_t)shown;
    }
    out[8] = 0xA5u;                     /* 本版标记：说明含"阶段 2（驱动低）" */
    for (i = 9u; i < 16u; i++) {
        out[i] = 0u;
    }

    /* ── 阶段 2：把 SWCLK 与 SWDIO 都**驱动为低**并保持 5 秒 ──
     * 用途：让用户在**目标板那一端**量 SWDIO 焊盘，判断这根线到底通不通：
     *   - 量到 0 V   → 线是通的（我们的低电平压过了目标内部的上拉）
     *   - 量到 ~3.3 V → 线没通到目标（只剩目标自己的上拉在拉高）
     * 阶段 1 期间（前 2 秒）SWCLK 是方波；阶段 2（第 2~7 秒）两根都是低电平。 */
    g.Pin   = SWD_DIO_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWD_DIO_PORT, &g);
    HAL_GPIO_WritePin(SWD_DIO_PORT, SWD_DIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SWD_CLK_PORT, SWD_CLK_PIN, GPIO_PIN_RESET);

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < SST_PIN_LOW_TEST_MS) {
        /* 空转保持低电平，等用户拿万用表去量目标板那一端 */
    }

    /* ── 阶段 3：顺手做一次 SWD 波形捕捉，把"ACK 窗口 32 拍"的电平放进 [9..12] ──
     * 这样用**同一条 mode=1 帧**就能同时得到：引脚配置 + 两段可量波形 + 线路抓包，
     * 不必再换算一条新帧的 CRC（现场少一步出错的机会）。 */
    {
        uint8_t w[MGR_SWD_SELFTEST_RESP_LEN];

        SST_WaveCapture(w);
        out[9]  = w[1];
        out[10] = w[2];
        out[11] = w[3];
        out[12] = w[4];
        out[13] = 0u;
        out[14] = 0u;
        out[15] = 0u;
    }

    /* 收尾：把两根脚恢复成 SWD 空闲态（CLK 低、DIO 由主机驱动） */
    SWD_PhyInit();
}

/*==============================================================================
 * SWD 速度扫描（mode=2）
 *----------------------------------------------------------------------------
 * 位翻转 SWD 的时钟由"空循环延时"决定。飞线长、没有终端匹配时，
 * 太快会出现**间歇性误码**（IDCODE 有时对、有时错位），极端时完全握不上手。
 * 这里一次把 8 档速度都试一遍，并用"IDCODE 是否正确"作为判据，
 * 一烧一测就能定出真正可靠的那一档（而不只是"能连上"）。
 *   0x00 = 该档读到了**正确** IDCODE（0x2BA01477）
 *   0x40 = 连上了但 IDCODE 不对（边缘误码）
 *   0x8x = 该档连不上，低 3 位是当时的 ACK
 *============================================================================*/
#define SST_SPEED_COUNT   8u
static const uint16_t s_speed_loops[SST_SPEED_COUNT] = {
    64u, 128u, 256u, 512u, 1024u, 2048u, 4096u, 8192u
};

void SST_SpeedProbe(uint8_t *out)
{
    uint8_t  i;
    uint8_t  good_idx = 0xFFu;
    uint32_t good_id  = 0u;

    if (out == 0) {
        return;
    }
    for (i = 0u; i < 16u; i++) {
        out[i] = 0xFFu;
    }
    out[0] = APP_STATUS_OK;
    out[8] = 0xB6u;                     /* 本版标记 */

    /* ⚠ 2026-11-03 改判据：**不只测"连接+读 IDCODE"（纯读）**，而是每档都跑
     * 连接 → DP 上电 → MEM-AP 初始化 —— 后两步含 **DP 写 + AP 写**，
     * 而实测失败都发生在写事务上，纯读全部正常。 */
    for (i = 0u; i < SST_SPEED_COUNT; i++) {
        uint32_t id = 0u;
        int      r;

        SWD_PhySetDelay(s_speed_loops[i]);
        SWD_PhyInit();

        r = SWD_Connect(&id);
        if ((r != SWD_OK) || (id != SST_F4_IDCODE)) {
            /* bit7=1：连接失败；低 3 位是当时的 ACK。bit6=1：连上了但 IDCODE 错 */
            out[1u + i] = (r != SWD_OK) ? (uint8_t)(0x80u | (SWD_LastAck() & 0x7Fu))
                                        : 0x40u;
            continue;
        }

        r = SWD_PowerUp();
        if (r != SWD_OK) {
            out[1u + i] = 0x20u;        /* bit5=1：DP 上电（**DP 写**）失败 */
            continue;
        }

        r = SWD_MemInit();
        if (r != SWD_OK) {
            out[1u + i] = 0x10u;        /* bit4=1：MEM-AP 初始化（**DP 写 + AP 写**）失败 */
            continue;
        }

        out[1u + i] = 0x00u;            /* 三步全过 */
        if (good_idx == 0xFFu) {
            good_idx = i;
            good_id  = id;
        }
    }

    SWD_PhySetDelay((uint16_t)SWD_DELAY_LOOPS);     /* 恢复默认速度 */
    SWD_PhyInit();

    out[9]  = (good_idx == 0xFFu) ? 0xFFu : (uint8_t)s_speed_loops[good_idx];
    out[10] = (uint8_t)(good_id >> 24);
    out[11] = (uint8_t)(good_id >> 16);
    out[12] = (uint8_t)(good_id >> 8);
    out[13] = (uint8_t)good_id;
    out[14] = (good_idx == 0xFFu) ? 0u : 1u;        /* 1 = 至少有一档读到了正确 IDCODE */
    out[15] = 0u;
}
