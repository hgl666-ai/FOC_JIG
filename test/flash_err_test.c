/*
 * SPI Flash "总线不通" 缺陷回归测试 (gd25q128.c)
 *
 * 缺陷背景 (2026-10-28 实测)：
 *   原实现的 SPI_Swap 用 1000ms 超时且**不看返回值**。SPI 总线不通时，
 *   每个字节白等 1 秒，一次 0x000C 查询读 2 槽 × 12 字节 = 24 次 → 卡 24 秒，
 *   读回全 0 还被当成"空槽"，回了一个误导性的 06 ERR_NO_FW。
 *
 * 本测试把 HAL_SPI_TransmitReceive 造成**永远超时**，然后验证：
 *   T1 失败被计数 (GD25Q_ErrCount 增量 = 传输次数)
 *   T2 GD25Q_ReadBuffer 返回 0 (上层才能报 ERR_FLASH)
 *   T3 GD25Q_ReadID 返回 0、GD25Q_Test 判失败
 *   T4 GD25Q_WaitForWriteEnd **不空转**: SPI 调用次数必须是常数级(≤3)，不是几百次
 *   T5 GD25Q_EraseSector 快速返回失败
 *   T6 对照组: 总线正常时一切照旧 (确保这次改动没伤到正常路径)
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "gd25q128.h"
#include "spi.h"

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

/*==============================================================================
 * 假 HAL
 *============================================================================*/
SPI_HandleTypeDef hspi2;

static uint32_t g_tick      = 0;
static uint32_t g_spi_calls = 0;      /* 本次操作里发生了多少次 SPI 传输 */
static int      g_bus_dead  = 1;      /* 1 = 模拟"总线不通"，0 = 正常 */

uint32_t HAL_GetTick(void) { return g_tick++; }

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) { (void)port; (void)init; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    (void)port; (void)pin; (void)state;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData, uint8_t *pRxData,
                                          uint16_t Size, uint32_t Timeout)
{
    (void)hspi; (void)Timeout; (void)pTxData;

    g_spi_calls++;
    if (g_bus_dead) {
        pRxData[0] = 0xFF;                 /* 悬空 MISO 常见读数 */
        return HAL_TIMEOUT;                /* ← 每个字节都超时 */
    }
    for (uint16_t i = 0; i < Size; i++) {
        pRxData[i] = 0x5Au;                /* 正常总线: 固定返回, 便于断言 */
    }
    return HAL_OK;
}

int main(void)
{
    uint8_t  hdr[8];
    uint32_t e0;

    printf("==================================================\n");
    printf(" gd25q128: dead-bus fast-fail regression\n");
    printf("==================================================\n");

    /* ── T1/T2: 读 8 字节 → 12 次传输全失败 → 返回 0, 计数 +12 ── */
    g_bus_dead = 1;
    memset(hdr, 0, sizeof(hdr));
    e0 = GD25Q_ErrCount();
    CHECK(GD25Q_ReadBuffer(hdr, 0u, 8u) == 0u, "ReadBuffer reports failure on dead bus");
    CHECK((GD25Q_ErrCount() - e0) == 12u, "error count grows by 12 (cmd+3 addr+8 data)");
    CHECK(hdr[0] == 0xFFu, "rx data reflects floating bus (documented behaviour)");

    /* ── T3: ID / 自检 ── */
    e0 = GD25Q_ErrCount();
    CHECK(GD25Q_ReadID() == 0u, "ReadID returns 0 on dead bus");
    CHECK((GD25Q_ErrCount() - e0) == 4u, "ReadID error delta = 4 (1 cmd + 3 bytes)");
    CHECK(GD25Q_Test() == 0u, "GD25Q_Test fails on dead bus");

    /* ── T4: 等 WIP 不空转（核心：原来会在 3s 里转几百次） ── */
    g_spi_calls = 0u;
    CHECK(GD25Q_WaitForWriteEnd() == 0u, "WaitForWriteEnd reports failure");
    CHECK(g_spi_calls <= 3u, "WaitForWriteEnd exits immediately (no 3s spin)");

    /* ── T5: 擦除快速失败 ── */
    g_spi_calls = 0u;
    CHECK(GD25Q_EraseSector(0u) == 0u, "EraseSector fails on dead bus");
    CHECK(g_spi_calls <= 6u, "EraseSector gives up immediately");

    /* ── T6: 对照组 —— 总线正常时行为不变 ── */
    g_bus_dead  = 0;
    g_spi_calls = 0u;
    e0 = GD25Q_ErrCount();
    CHECK(GD25Q_ReadBuffer(hdr, 0u, 8u) == 1u, "ReadBuffer returns 1 on healthy bus");
    CHECK(GD25Q_ErrCount() == e0, "no error counted on healthy bus");
    CHECK(hdr[0] == 0x5Au && hdr[7] == 0x5Au, "healthy bus returns the data it was given");

    /* ── T7: 诊断命令的数据结构（总线不通时的结论必须是 1 = 超时） ── */
    {
        uint8_t d[64];

        g_bus_dead = 1;
        memset(d, 0xEE, sizeof(d));
        CHECK(GD25Q_Diag(d) == 1u, "Diag reports 1 (bus timeout) on dead bus");
        CHECK(d[0] == 1u, "Diag result byte matches return value");
        CHECK(d[1] != 0u, "Diag error delta is non-zero");
        CHECK(d[2] == 0u && d[3] == 0u && d[4] == 0u && d[5] == 0u,
              "Diag JEDEC ID is 0 when transfers time out");

        /* 总线通、但读不到合法 ID → 结论 2（例如 0xFFFFFF = 没有芯片应答） */
        g_bus_dead = 0;
        memset(d, 0xEE, sizeof(d));
        CHECK(GD25Q_Diag(d) == 2u, "Diag reports 2 (bus ok, wrong ID) on healthy bus");
        CHECK(d[1] == 0u, "Diag error delta is zero on healthy bus");
        CHECK(d[3] == 0x5Au && d[4] == 0x5Au && d[5] == 0x5Au, "Diag reports read-back ID bytes");

        /* 宿主编译下寄存器区（24..47）必须被显式清零，不能留栈上的 0xEE */
        CHECK(d[24] == 0u && d[25] == 0u && d[26] == 0u && d[27] == 0u &&
              d[28] == 0u && d[29] == 0u && d[30] == 0u && d[31] == 0u &&
              d[32] == 0u && d[33] == 0u && d[34] == 0u && d[35] == 0u &&
              d[36] == 0u && d[37] == 0u && d[38] == 0u && d[39] == 0u &&
              d[40] == 0u && d[41] == 0u && d[42] == 0u && d[47] == 0u &&
              d[48] == 0u && d[49] == 0u && d[50] == 0u && d[51] == 0u &&
              d[52] == 0u && d[53] == 0u && d[54] == 0u && d[55] == 0u &&
              d[56] == 0u && d[57] == 0u && d[58] == 0u && d[59] == 0u &&
              d[60] == 0u && d[61] == 0u && d[62] == 0u && d[63] == 0u,
              "Diag register fields are zeroed on host build (JIG_HOST_TEST)");
    }

    printf("--------------------------------------------------\n");
    printf(" pass = %d, fail = %d\n", g_pass, g_fail);
    if (g_fail == 0) {
        printf(" RESULT: ALL PASS\n");
        return 0;
    }
    printf(" RESULT: FAILED\n");
    return 1;
}
