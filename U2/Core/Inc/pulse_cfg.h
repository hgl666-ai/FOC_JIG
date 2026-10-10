/**
  ******************************************************************************
  * @file    pulse_cfg.h
  * @brief   U2 的输入捕获绑定：三相**主控驱动信号** → TIM3 + TIM1
  *
  *   A 相 → TIM3_CH2 = PB5     ← MA_IN
  *   B 相 → TIM3_CH1 = PB4     ← MB_IN
  *   C 相 → TIM1_CH1 = PA8     ← MC_IN
  *
  * ⚠ 三相分散在两个定时器上（原理图就这么接的），所以两个时间基**互相独立**：
  *   每个相位自己的周期/占空比都只用自己那一路的时间戳算，不需要跨定时器对齐。
  *   代价是 C 相要多一个溢出中断（TIM1_UP），比 U1 多一点点中断负载，可忽略。
  ******************************************************************************
  */
#ifndef __PULSE_CFG_H
#define __PULSE_CFG_H

#include <stdint.h>
#include "pulse_cap.h"

/** 配好捕获并让两个定时器自由运行（上电初始化时调一次，此时**不采**） */
void    PULSE_CfgInit(void);

/** 开一个测量窗口：清时间戳、复位两个计数器、打开捕获中断 */
void    PULSE_CfgStart(uint16_t window_ms);

/** 关捕获（窗口未到也可强制停） */
void    PULSE_CfgStop(void);

/** 主循环推进：窗口到时结算并自动关捕获（内部调 PULSE_Poll） */
uint8_t PULSE_CfgPoll(void);

/** 诊断：两个定时器的丢边沿计数之和 */
uint32_t PULSE_CfgLostEdges(void);

#endif /* __PULSE_CFG_H */
