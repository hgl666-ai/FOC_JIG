/**
  ******************************************************************************
  * @file    enc_gen.h
  * @brief   ⑤ 编码器脉冲发生（**U1 专用**）：用 TIM4 在 CN2 的 A/B/Z 上发定量脉冲串
  *
  * 硬件与接线见 `enc_plan.h` 的文件头（含"同一时刻只能有一片驱动"的约束）。
  * 纯算术在 `enc_plan.c`（有宿主单测）；本文件只管寄存器与引脚。
  *
  * 引脚 / 通道绑定：**按 5 脚编码器座的常规顺序推断**
  *   CN2 pin2 = A → TIM4_CH3 = PB8
  *   CN2 pin3 = B → TIM4_CH2 = PB7
  *   CN2 pin4 = Z → TIM4_CH1 = PB6
  * ⚠ 这个推断**没有被实测证实**（原理图上这三根线只标了 TIM4_CHx，没标 A/B/Z）。
  *   上板时用示波器看一眼：若发现 A/B 反了，**只改下面这三个宏**，其余代码不用动。
  *
  * 工作方式
  * --------
  * TIM4 上计数、ARR = 0xFFFF、PSC 由 `EncPlan_PscFor` 按最低那一路频率算；
  * 三个通道各自配成**输出比较翻转**（OCxM = 011），CCR 各填各的 →
  * 三路可以**各走各的频率**，每翻转一次中断计一次数，数够 2×脉冲数就单独停那一路。
  *
  * ⚠ 中断里显式清 CCxIF：**输出比较的标志必须写 0 清**，
  *   不像输入捕获那样"读 CCR 就清"（那套写法直接搬过来会死循环在中断里）。
  *
  * ⚠ 收尾一定把 PB6/PB7/PB8 **收回输入态**：那三根线 U2 也挂着，
  *   治具发完不收手，U2 再想采、或者被测板自带编码器想驱动，就是对打。
  ******************************************************************************
  */
#ifndef __ENC_GEN_H
#define __ENC_GEN_H

#include <stdint.h>
#include "enc_plan.h"

/* 完成掩码（与 enc_plan 的 A/B/Z 顺序一致） */
#define ENC_GEN_DONE_A      0x01u
#define ENC_GEN_DONE_B      0x02u
#define ENC_GEN_DONE_Z      0x04u
#define ENC_GEN_DONE_ALL    0x07u

/** 上电初始化：把 PB6/PB7/PB8 收回输入态（**不发任何东西**） */
void    EncGen_Init(void);

/**
  * @brief  按规划发一串脉冲
  * @param  p  规划（会先过一遍 `EncPlan_Check`）
  * @retval 1 = 已启动；0 = 参数非法或上一次还没结束
  * @note   **收到即返回**，不阻塞；发完由 `EncGen_Poll` 在主循环里收尾。
  */
uint8_t EncGen_Start(const enc_plan_t *p);

/** 主循环推进：全部发完或超时就自动收尾（收针 + 关中断） */
void    EncGen_Poll(void);

/** 立即停止并把引脚收回输入态（安全命令 / 出错时用） */
void    EncGen_Stop(void);

/** 1 = 正在发 */
uint8_t EncGen_Busy(void);

/** 哪些通道已经把该发的都发完了（ENC_GEN_DONE_*） */
uint8_t EncGen_DoneMask(void);

/**
  * @brief  本次是否**超时强停**的（1 = 异常）
  * @note   正常发完是 0；超时说明"数够了但没停下来"或"中断没来"，
  *         上板看到 1 就要查（多半是引脚没配成 AF、或 CCxIF 没清干净）。
  */
uint8_t EncGen_Aborted(void);

/** 诊断：某一路上次实际翻转了多少次（调用方与 2×脉冲数 对比） */
uint16_t EncGen_Flips(uint8_t ph);      /* ph: 0=A 1=B 2=Z */

/** 本次实际用的 PSC / 时钟（诊断用；0 表示当前没有在发） */
uint16_t EncGen_LastPsc(void);
uint32_t EncGen_LastClkHz(void);

#endif /* __ENC_GEN_H */
