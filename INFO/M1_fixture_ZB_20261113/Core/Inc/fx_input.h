/**
  ******************************************************************************
  * @file    fx_input.h
  * @brief   M1 治具 —— 8 路 IO 输入检测
  *
  * 引脚对应《PCB引脚功能对照表》E 列"IO输入检测"，顺序与 0x0017 ACK 一致：
  *
  *   idx0 KEY3 (PC4)   idx1 KEY4 (PC5)   idx2 KEY1 (PB0)   idx3 KEY2 (PB1)
  *   idx4 TBS  (PB10)  idx5 KEY0 (PB11)  idx6 TEST-INT(PC8) idx7 MMS (PA15)
  *
  * 检测分两步，两步都通过才算通过：
  *   1) 治具侧全部配成输入，读默认电平：
  *        KEY0..KEY4 / TBS = 低（DUT 内部下拉 / 外部下拉）
  *        MMS               = 高（DUT 外部上拉）
  *        TEST-INT          = 不做固定电平要求，只要求三方一致
  *   2) 治具侧把 KEYx / TBS / MMS 配成推挽输出，先全高再全低：
  *        治具自身、副芯片、DUT 三方读到的电平都必须等于输出电平。
  *
  * TEST-INT 不驱动，只做"治具芯片、副芯片、DUT 三方同电平"判定。
  * 副芯片通讯不上时，其采集值按通过处理；DUT 通讯不上则该路标 0x02。
  ******************************************************************************
  */
#ifndef __FX_INPUT_H
#define __FX_INPUT_H

#include "fx_common.h"

typedef enum {
    FX_IN_KEY3 = 0,     /* PC4  */
    FX_IN_KEY4,         /* PC5  */
    FX_IN_KEY1,         /* PB0  */
    FX_IN_KEY2,         /* PB1  */
    FX_IN_TBS,          /* PB10 */
    FX_IN_KEY0,         /* PB11 */
    FX_IN_TEST_INT,     /* PC8  */
    FX_IN_MMS,          /* PA15 */
    FX_IN_QTY
} fx_in_id_t;

void fx_input_init(void);

/** 主循环 1ms 任务（IO 检测是命令触发的，这里预留） */
void fx_input_poll(void);

/** 执行一次完整 IO 输入检测。
 *  @param flags 输出 8 字节，顺序同 fx_in_id_t，值见 FX_FLAG_xxx
 *  @retval FX_TEST_OK / FX_TEST_FAIL
 */
uint8_t fx_input_test(uint8_t flags[FX_IN_QTY]);

/** 读某一路当前电平（0/1），主要给调试用 */
uint8_t fx_input_read(fx_in_id_t id);

/** 把所有可驱动脚恢复为输入（高阻），避免测试结束后继续驱动 DUT */
void fx_input_all_input(void);

#endif /* __FX_INPUT_H */
