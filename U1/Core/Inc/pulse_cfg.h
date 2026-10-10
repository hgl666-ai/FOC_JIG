/**
  ******************************************************************************
  * @file    pulse_cfg.h
  * @brief   U1 的输入捕获绑定：三相**输出端** → TIM4
  *
  *   A 相 → TIM4_CH3 = PB8     ← MA0_OUT（经 3kΩ + SMF3.3A）
  *   B 相 → TIM4_CH2 = PB7     ← MB0_OUT
  *   C 相 → TIM4_CH1 = PB6     ← MC0_OUT
  *
  * 与 U2 的差别只有这张表（U2 用 TIM3_CH2/CH1 + TIM1_CH1 测主控驱动信号），
  * 算法在 pulse_cap.c（两片逐字节一致），本文件只做"寄存器 ↔ 算法"的桥。
  *
  * 捕获方式：**单通道 + 每个边沿翻转极性**（pulse_cap.c 文件头有约定说明）。
  * 为什么不用硬件 PWM 输入模式：那个模式每个信号要占 2 个捕获通道，
  * TIM4 只有 4 个通道，3 相要 6 个 → 放不下。
  ******************************************************************************
  */
#ifndef __PULSE_CFG_H
#define __PULSE_CFG_H

#include <stdint.h>
#include "pulse_cap.h"

/** 配好捕获并让定时器自由运行（上电初始化时调一次，此时**不采**） */
void    PULSE_CfgInit(void);

/** 开一个测量窗口：清时间戳、复位计数器、打开捕获中断 */
void    PULSE_CfgStart(uint16_t window_ms);

/** 关捕获（窗口未到也可强制停） */
void    PULSE_CfgStop(void);

/** 主循环推进：窗口到时结算并自动关捕获（内部调 PULSE_Poll） */
uint8_t PULSE_CfgPoll(void);

/** 诊断：丢边沿（中断里发现标志被覆盖）的次数 */
uint32_t PULSE_CfgLostEdges(void);

#endif /* __PULSE_CFG_H */
