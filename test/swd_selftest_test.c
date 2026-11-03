/*
 * SWD 烧录自检（B1 / 命令 0x001E）宿主测试
 *
 * 用 test/swdsim 的**线级 SWD 目标模型**跑完整自检，然后直接查仿真 F4 Flash 里的内容，
 * 这样能在没有硬件、也没有 SPI Flash 的情况下证明：
 *   连接 → 擦 scratch 扇区 → 写 4KB 图样 → SWD 回读比对 → 读 UID → 复位运行
 * 这一整套是真的能跑通的。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "swd_selftest.h"
#include "flash_map.h"
#include "swd_config.h"     /* SWD_TA_BEFORE_READ_DATA */
#include "sim_target.h"

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
 * HAL 替身（SWD 位翻转只用 GPIO；时间只用于超时）
 *============================================================================*/
static uint32_t g_tick = 0;

uint32_t HAL_GetTick(void) { return g_tick++; }
void     HAL_Delay(uint32_t ms) { g_tick += ms; }

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    sim_gpio_dir(port, (uint16_t)init->Pin,
                 (init->Mode == GPIO_MODE_INPUT) ? 0u : 1u);
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    sim_gpio_write(port, pin, (state == GPIO_PIN_SET) ? 1u : 0u);
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return sim_gpio_read(port, pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/*==============================================================================
 * 跑一次自检
 *============================================================================*/
static uint8_t RunSelftest(uint8_t out[MGR_SWD_SELFTEST_RESP_LEN])
{
    uint32_t guard = 0u;

    SST_Start();
    while (SST_Busy() != 0u) {
        SST_Step();
        if (++guard > 100000u) {
            break;                      /* 防死循环：正常约 70 步 */
        }
    }
    return SST_Result(out);
}

int main(void)
{
    uint8_t  out[MGR_SWD_SELFTEST_RESP_LEN];
    uint8_t  st;
    uint8_t *flash;
    uint32_t base;

    printf("==================================================\n");
    printf(" SWD selftest (0x001E) on line-level target model\n");
    printf("==================================================\n");

    /* ── T1: 目标不在（线悬空）→ 必须失败并给出 ERR_PROG ── */
    sim_target_reset();
    sim_target_attach(0u);
    sim_target_set_read_ta((uint8_t)SWD_TA_BEFORE_READ_DATA);
    st = RunSelftest(out);
    CHECK(st == APP_STATUS_ERR_PROG, "target absent -> ERR_PROG");
    CHECK(out[1] == 1u, "失败阶段 = 1 (连接)");
    CHECK(SST_Busy() == 0u, "失败后状态机已停止");

    /* ── T2: 正常自检，全流程必须通过 ── */
    sim_target_reset();
    sim_target_attach(1u);
    sim_target_set_read_ta((uint8_t)SWD_TA_BEFORE_READ_DATA);

    st = RunSelftest(out);
    CHECK(st == APP_STATUS_OK,       "正常自检通过 (status=00)");
    CHECK(out[0] == APP_STATUS_OK,   "应答[0] = 状态码");
    CHECK(out[1] == 0xFFu,           "应答[1] = 0xFF (全部完成)");
    CHECK(out[2] == 0xFFu && out[3] == 0xFFu, "应答[2..3] = 0xFFFF (无不符字节)");
    CHECK(((uint32_t)out[4] << 24 | (uint32_t)out[5] << 16 |
           (uint32_t)out[6] << 8  | (uint32_t)out[7]) == 0x2BA01477u,
          "IDCODE = 0x2BA01477");
    CHECK(((uint16_t)out[8] << 8 | (uint16_t)out[9]) == 0x0413u,
          "DEV_ID = 0x413 (F405)");
    CHECK(((uint16_t)out[10] << 8 | (uint16_t)out[11]) == (uint16_t)DUT_SELFTEST_BYTES,
          "已写字节数 = 4096");

    /* ── T3: 目标侧动作确实发生了 ── */
    CHECK(sim_target_erases() >= 1u,      "至少擦除了 1 个扇区");
    CHECK(sim_target_programs() > 0u,     "发生了页编程");
    CHECK(sim_target_contention() == 0u,  "SWDIO 全程零总线冲突");
    CHECK(sim_target_parity_errors() == 0u, "零奇偶校验错");
    CHECK(sim_target_locked_reject() == 0u, "no write-before-unlock attempt");

    /* ── T4: 内容真的落到了 scratch 区（手工可核对的几个字节） ── */
    flash = sim_target_flash();
    base  = DUT_SCRATCH_ADDR - 0x08000000u;     /* 0x080E0000 → 0xE0000 */

    CHECK(flash[base + 0u] == 0x00u && flash[base + 1u] == 0x00u &&
          flash[base + 2u] == 0x00u && flash[base + 3u] == 0x00u,
          "scratch[0..3] = 00 00 00 00（图样每 256B 的低 4 字节）");
    CHECK(flash[base + 4u] == 0xFFu && flash[base + 5u] == 0xFFu &&
          flash[base + 6u] == 0xFFu && flash[base + 7u] == 0xFFu,
          "scratch[4..7] = FF FF FF FF（高 4 字节）");
    CHECK(flash[base + 8u] == (uint8_t)((8u * 7u) ^ 0x5Au),
          "scratch[8] = 0x32（(8*7) ^ 0x5A）");
    CHECK(flash[base + DUT_SELFTEST_BYTES - 1u] != 0xFFu ||
          flash[base + DUT_SELFTEST_BYTES - 2u] != 0xFFu,
          "4KB 末尾不是空白（确实写满了）");

    /* ── T5: 自检不碰 Boot/App 分区 ── */
    CHECK(DUT_SCRATCH_ADDR >= (DUT_APP_ADDR + DUT_APP_SIZE),
          "scratch 区在 App 分区之外（静态断言之外的运行期复核）");

    printf("--------------------------------------------------\n");
    printf(" 擦除 %lu 次  编程 %lu 次  SWD 事务 %lu  冲突 %lu\n",
           (unsigned long)sim_target_erases(),
           (unsigned long)sim_target_programs(),
           (unsigned long)sim_target_transactions(),
           (unsigned long)sim_target_contention());
    printf(" pass = %d, fail = %d\n", g_pass, g_fail);
    if (g_fail == 0) {
        printf(" RESULT: ALL PASS\n");
        return 0;
    }
    printf(" RESULT: FAILED\n");
    return 1;
}
