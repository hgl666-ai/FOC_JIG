/**
  ******************************************************************************
  * @file    pulse_cap.h
  * @brief   TIM 输入捕获：测三相 PWM 的周期 / 高电平时间 / 占空比
  *
  * 用途（docs/12_TIM脉冲捕获设计.md，2026-10 按原理图更正过）：
  *   U1 和 U2 都测**主控驱动信号** MA_IN / MB_IN / MC_IN
  *     （TIM3_CH2=PB5 / TIM3_CH1=PB4 / TIM1_CH1=PA8，两片挂在同一组线上）
  *     → 两片测同一组信号，做**真冗余互比**（和 9 路 ADC 的交叉校验同构）。
  *   ⚠ 旧版这里写的是"U1 测电机输出端 MA0_OUT（TIM4）、两片测同一相的两端"，
  *     那个前提是**错的**：`PB6/PB7/PB8` 是编码器座 CN2 的 A/B/Z（留给 ⑤），
  *     `MA0_OUT` 组在硬件上**只有 ADC 通路**、没有定时器通路。详见 docs/00 §3.5。
  *
  * 分层：
  *   本文件（两片逐字节副本）：只有算法，**不碰任何寄存器**
  *     - 收到"某相来了一个边沿，时间戳是多少"(tick) 就存进环形缓冲
  *     - 主循环在窗口结束时算周期/占空比
  *   各片的 pulse_cfg.c：真正的输入捕获中断（读 SR/CCR、溢出扩展、翻转边沿极性）
  *     + 相位→(定时器,通道) 的绑定表
  *
  * 为什么用软件算而不是硬件 PWM 输入模式：
  *   PWM 输入模式每个信号要占 2 个捕获通道，而 U1 的 TIM4 只有 4 个通道，
  *   3 相要 6 个 → 放不下。所以统一用"单通道双边沿捕获 + 软件算"，两片同一套代码。
  *
  * 时间单位：定时器 tick。本工程 PSC=0 → 72 MHz → 1 tick = 125/9 ns ≈ 13.888 ns。
  *   时间戳是 32 位（16 位计数器 + 中断里扩展溢出），tick 分辨率不变。
  ******************************************************************************
  */
#ifndef __PULSE_CAP_H
#define __PULSE_CAP_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* 相位（与 measure.h 的 PHA/PHB/PHC 是同一相，只是测的是"时序"）        */
/* ------------------------------------------------------------------ */
typedef enum {
    PULSE_PH_A = 0,
    PULSE_PH_B,
    PULSE_PH_C,
    PULSE_PH_COUNT
} pulse_ph_t;

/* ------------------------------------------------------------------ */
/* 参数                                                                */
/* ------------------------------------------------------------------ */
/* 定时器时钟与 tick 换算：ns = tick * 125 / 9 */
#define PULSE_TICK_HZ        72000000u
#define PULSE_NS_NUM         125u      /* ns = tick * NUM / DEN */
#define PULSE_NS_DEN         9u

/* 一次测量窗口（ms）。下限由"窗口内至少要 2 个周期"决定：
 *   200 ms → 最低约 10 Hz。窗口越长越慢的信号也能测，但产线节拍变慢。 */
#ifndef PULSE_WINDOW_MS
#define PULSE_WINDOW_MS      200u
#endif

/* 最少边沿数：2 个完整周期。少于这个数当作"没信号"，而不是硬算出个离谱值 */
#define PULSE_MIN_EDGES      4u

/* 每相保留最近多少个边沿时间戳（用于平均）。
 * 20 kHz 时 32 个边沿 = 16 个周期，足够把抖动平均掉。 */
#define PULSE_RING_LEN       32u

/* 周期上下限（tick）：低于 200 tick(≈2.8us, 357kHz) 或高于 1 秒判"超范围" */
#define PULSE_MIN_PERIOD_TICK 200u
#define PULSE_MAX_PERIOD_TICK (PULSE_TICK_HZ)      /* 1 秒 */

/* ------------------------------------------------------------------ */
/* 结果                                                                */
/* ------------------------------------------------------------------ */
#define PULSE_F_VALID     (1u << 0)   /* 这一相测到了有效波形 */
#define PULSE_F_NOSIGNAL  (1u << 1)   /* 边沿太少 = 没信号 */
#define PULSE_F_RANGE     (1u << 2)   /* 周期超出可测范围 */
#define PULSE_F_DIRTY     (1u << 3)   /* 有异常间隔被丢弃(丢边沿/毛刺) */
#define PULSE_F_PEER_DIFF (1u << 4)   /* (U1 侧填) 与对侧频率明显不一致 */

typedef struct {
    uint32_t period_ns;      /* 周期（纳秒），0 = 没测到 */
    uint32_t high_ns;        /* 高电平时间（纳秒） */
    uint16_t duty_permille;  /* 占空比 ×1000（500 = 50.0%） */
    uint16_t edges;          /* 窗口内捕获到的边沿总数（饱和到 65535） */
    uint8_t  flags;          /* PULSE_F_* */
} pulse_result_t;

/* 一组三相结果（"快照"：某一窗口的整体情况） */
typedef struct {
    uint8_t        phase_count;     /* = PULSE_PH_COUNT */
    uint8_t        flags;           /* bit0 数据有效 / bit1 至少一相没信号 */
    uint16_t       window_ms;       /* 本次窗口长度 */
    pulse_result_t ph[PULSE_PH_COUNT];
} pulse_snapshot_t;

/* 快照整体标志 */
#define PULSE_SNAP_F_VALID    (1u << 0)
#define PULSE_SNAP_F_NOSIGNAL (1u << 1)   /* 至少一相没测到信号 */

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */
/** 清状态（采集层的定时器配置由各片 pulse_cfg.c 负责） */
void        PULSE_Init(void);

/**
  * @brief  开一个测量窗口：清缓冲、开始收集边沿
  * @note   ⚠ **必须在"捕获中断已关、且 NVIC 的 pending 已清"的临界区里调用**，
  *         否则中断可能把边沿追加进"清了一半"的环（s_ph[] 的 head/count 不成对
  *         → 奇偶判错 → 周期/占空比错值）。这就是 docs/03 §4 的 D5。
  *         固件里**唯一**的调用者是 pulse_cfg.c 的 `PULSE_CfgStart()`，它负责
  *         整个临界区；应用层不要再直接调本函数。
  */
void        PULSE_Start(uint16_t window_ms);

/** 关掉捕获（窗口未到也可强制停） */
void        PULSE_Stop(void);

/**
  * @brief  主循环推进：窗口时间到时结算三相结果
  * @retval 0 = 还在采, 1 = 已完成（可以取快照）, 2 = 当前没在采
  */
uint8_t     PULSE_Poll(void);

uint8_t     PULSE_IsRunning(void);

/** 取最近一次结算的结果 */
void        PULSE_Snapshot(pulse_snapshot_t *out);

/**
  * @brief  采集层（各片的捕获中断）每来一个边沿调一次
  * @param  ph   哪一相
  * @param  tick 边沿时刻（32 位 tick，已由采集层做过溢出扩展）
  */
void        PULSE_OnEdge(pulse_ph_t ph, uint32_t tick);


#endif /* __PULSE_CAP_H */
