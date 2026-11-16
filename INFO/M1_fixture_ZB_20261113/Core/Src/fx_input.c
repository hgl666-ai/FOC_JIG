/**
  ******************************************************************************
  * @file    fx_input.c
  * @brief   8 路 IO 输入检测
  ******************************************************************************
  */
#include "fx_input.h"

#include "main.h"
#include "fx_bus.h"
#include "log.h"

#define FX_IN_SETTLE_MS     10u

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       default_level;    /* 输入模式下的期望默认电平 */
    uint8_t       drivable;         /* 1 = 可以配成输出驱动 */
} fx_in_pin_t;

/* 顺序必须与 fx_in_id_t / 0x0017 ACK 一致 */
static const fx_in_pin_t s_pins[FX_IN_QTY] = {
    { GPIOC, GPIO_PIN_4,  0u, 1u },     /* KEY3     */
    { GPIOC, GPIO_PIN_5,  0u, 1u },     /* KEY4     */
    { GPIOB, GPIO_PIN_0,  0u, 1u },     /* KEY1     */
    { GPIOB, GPIO_PIN_1,  0u, 1u },     /* KEY2     */
    { GPIOB, GPIO_PIN_10, 0u, 1u },     /* TBS      */
    { GPIOB, GPIO_PIN_11, 0u, 1u },     /* KEY0     */
    { GPIOC, GPIO_PIN_8,  0u, 0u },     /* TEST-INT */
    { GPIOA, GPIO_PIN_15, 1u, 1u },     /* MMS      */
};

/* 治具 idx -> DUT M119 A 的传感器下标 */
static const uint8_t s_dut_idx[FX_IN_QTY] = {
    FX_DUT_IO_KEY3,
    FX_DUT_IO_KEY4,
    FX_DUT_IO_KEY1,
    FX_DUT_IO_KEY2,
    FX_DUT_IO_TBS,
    FX_DUT_IO_KEY0,
    FX_DUT_IO_TEST_INT,
    FX_DUT_IO_MMS,
};

static void fx_pin_input(const fx_in_pin_t *p)
{
    GPIO_InitTypeDef gi = {0};

    gi.Pin   = p->pin;
    gi.Mode  = GPIO_MODE_INPUT;
    gi.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(p->port, &gi);
}

static void fx_pin_output(const fx_in_pin_t *p, uint8_t level)
{
    GPIO_InitTypeDef gi = {0};

    /* 先写 ODR 再切输出，避免切换瞬间产生低脉冲 */
    HAL_GPIO_WritePin(p->port, p->pin, (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    gi.Pin   = p->pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(p->port, &gi);
}

static uint8_t fx_pin_read(const fx_in_pin_t *p)
{
    return (HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_SET) ? 1u : 0u;
}

static uint8_t fx_sub_bit(uint8_t bitmap, uint8_t idx)
{
    return (uint8_t)((bitmap >> idx) & 0x01u);
}

void fx_input_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PA15 是 JTDI、PB3/PB4 是 JTDO/NJTRST；只关 JTAG、保留 SWD，
     * 才能把 PA15 当 MMS 用（PB3/PB4 在 CubeMX 里也配成了 GPIO 输入）。 */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    fx_input_all_input();
}

void fx_input_all_input(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        fx_pin_input(&s_pins[i]);
    }
}

void fx_input_poll(void)
{
    /* IO 检测由 0x0017 命令触发，无周期性工作 */
}

uint8_t fx_input_read(fx_in_id_t id)
{
    if ((uint8_t)id >= (uint8_t)FX_IN_QTY) {
        return 0u;
    }
    return fx_pin_read(&s_pins[id]);
}

/** 打一个阶段的"治具 / 副芯片 / DUT"三方电平。
 *  ok=0 表示该方这一轮没有有效数据，数值仅供对照，不参与判定。 */
static void fx_log_phase(const char *tag, const uint8_t *lv,
                         uint8_t sub_bm, uint8_t sub_ok,
                         const uint8_t *dut_lv, uint8_t dut_ok)
{
    LOGI("ioin %s: fix %d %d %d %d %d %d %d %d", tag,
         (int)lv[0], (int)lv[1], (int)lv[2], (int)lv[3],
         (int)lv[4], (int)lv[5], (int)lv[6], (int)lv[7]);

    LOGI("ioin %s: sub %d %d %d %d %d %d %d %d ok=%d", tag,
         (int)fx_sub_bit(sub_bm, 0u), (int)fx_sub_bit(sub_bm, 1u),
         (int)fx_sub_bit(sub_bm, 2u), (int)fx_sub_bit(sub_bm, 3u),
         (int)fx_sub_bit(sub_bm, 4u), (int)fx_sub_bit(sub_bm, 5u),
         (int)fx_sub_bit(sub_bm, 6u), (int)fx_sub_bit(sub_bm, 7u),
         (int)sub_ok);

    LOGI("ioin %s: dut %d %d %d %d %d %d %d %d ok=%d", tag,
         (int)dut_lv[s_dut_idx[0]], (int)dut_lv[s_dut_idx[1]],
         (int)dut_lv[s_dut_idx[2]], (int)dut_lv[s_dut_idx[3]],
         (int)dut_lv[s_dut_idx[4]], (int)dut_lv[s_dut_idx[5]],
         (int)dut_lv[s_dut_idx[6]], (int)dut_lv[s_dut_idx[7]],
         (int)dut_ok);
}

uint8_t fx_input_test(uint8_t flags[FX_IN_QTY])
{
    uint8_t fail[FX_IN_QTY]    = {0};
    uint8_t missing[FX_IN_QTY] = {0};
    uint8_t lv[FX_IN_QTY]      = {0};
    uint8_t sub_bm = 0u;
    uint8_t sub_ok;
    uint8_t dut_lv[FX_DUT_IO_QTY] = {0};
    uint8_t dut_ok;
    uint8_t i;
    uint8_t any_fail = 0u;
    uint8_t all_missing = 1u;

    /*--------------------------------------------------------------------------
     * 第一步：所有脚配成输入，检查默认电平
     *------------------------------------------------------------------------*/
    fx_input_all_input();
    HAL_Delay(FX_IN_SETTLE_MS);

    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        lv[i] = fx_pin_read(&s_pins[i]);
    }

    sub_ok = fx_bus_subchip_read_io(&sub_bm);
    dut_ok = fx_bus_dut_read_io(dut_lv);

    LOGI("ioin order: KEY3 KEY4 KEY1 KEY2 TBS KEY0 TEST-INT MMS");
    LOGI("ioin expect: fix 0 0 0 0 0 0 - 1 (TEST-INT no fixed, MMS high)");
    fx_log_phase("in", lv, sub_bm, sub_ok, dut_lv, dut_ok);

    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        if (i == (uint8_t)FX_IN_TEST_INT) {
            /* TEST-INT 在 DUT 上电平稳定，只要求三方相同 */
            if (dut_ok == 0u) {
                missing[i] = 1u;
            } else if (dut_lv[s_dut_idx[i]] != lv[i]) {
                fail[i] = 1u;
            }
            if ((sub_ok != 0u) && (fx_sub_bit(sub_bm, i) != lv[i])) {
                fail[i] = 1u;
            }
        } else {
            uint8_t expect = s_pins[i].default_level;

            if (lv[i] != expect) {
                fail[i] = 1u;
            }
            if ((sub_ok != 0u) && (fx_sub_bit(sub_bm, i) != expect)) {
                fail[i] = 1u;
            }
            if (dut_ok == 0u) {
                missing[i] = 1u;
            } else if (dut_lv[s_dut_idx[i]] != expect) {
                fail[i] = 1u;
            }
        }
    }

    /*--------------------------------------------------------------------------
     * 第二步：治具输出全高，三方读到的都必须是高
     *------------------------------------------------------------------------*/
    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        if (s_pins[i].drivable != 0u) {
            fx_pin_output(&s_pins[i], 1u);
        }
    }
    HAL_Delay(FX_IN_SETTLE_MS);

    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        lv[i] = fx_pin_read(&s_pins[i]);
    }
    if (sub_ok != 0u) {
        sub_ok = fx_bus_subchip_read_io(&sub_bm);
    }
    if (dut_ok != 0u) {
        dut_ok = fx_bus_dut_read_io(dut_lv);
    }

    fx_log_phase("hi", lv, sub_bm, sub_ok, dut_lv, dut_ok);

    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        if (s_pins[i].drivable == 0u) {
            continue;
        }
        if (lv[i] != 1u) {
            fail[i] = 1u;
        }
        if ((sub_ok != 0u) && (fx_sub_bit(sub_bm, i) != 1u)) {
            fail[i] = 1u;
        }
        if (dut_ok == 0u) {
            missing[i] = 1u;
        } else if (dut_lv[s_dut_idx[i]] != 1u) {
            fail[i] = 1u;
        }
    }

    /*--------------------------------------------------------------------------
     * 第三步：治具输出全低，三方读到的都必须是低
     *------------------------------------------------------------------------*/
    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        if (s_pins[i].drivable != 0u) {
            fx_pin_output(&s_pins[i], 0u);
        }
    }
    HAL_Delay(FX_IN_SETTLE_MS);

    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        lv[i] = fx_pin_read(&s_pins[i]);
    }
    if (sub_ok != 0u) {
        sub_ok = fx_bus_subchip_read_io(&sub_bm);
    }
    if (dut_ok != 0u) {
        dut_ok = fx_bus_dut_read_io(dut_lv);
    }

    fx_log_phase("lo", lv, sub_bm, sub_ok, dut_lv, dut_ok);

    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        if (s_pins[i].drivable == 0u) {
            continue;
        }
        if (lv[i] != 0u) {
            fail[i] = 1u;
        }
        if ((sub_ok != 0u) && (fx_sub_bit(sub_bm, i) != 0u)) {
            fail[i] = 1u;
        }
        if (dut_ok == 0u) {
            missing[i] = 1u;
        } else if (dut_lv[s_dut_idx[i]] != 0u) {
            fail[i] = 1u;
        }
    }

    /* 测试结束恢复高阻，避免治具继续驱动 DUT */
    fx_input_all_input();

    /*--------------------------------------------------------------------------
     * 汇总 flags
     *------------------------------------------------------------------------*/
    for (i = 0u; i < (uint8_t)FX_IN_QTY; i++) {
        if (fail[i] != 0u) {
            flags[i] = FX_FLAG_OUT_RANGE;
            any_fail = 1u;
        } else if (missing[i] != 0u) {
            flags[i] = FX_FLAG_NO_DATA;
        } else {
            flags[i] = FX_FLAG_PASS;
        }

        if (flags[i] != FX_FLAG_NO_DATA) {
            all_missing = 0u;
        }
    }

    return (any_fail != 0u) ? FX_TEST_FAIL : ((all_missing != 0u) ? FX_TEST_FAIL : FX_TEST_OK);
}
