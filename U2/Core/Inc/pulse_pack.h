/**
  ******************************************************************************
  * @file    pulse_pack.h
  * @brief   PWM 时序结果的**定长线格式**（0x000E 应答的脉冲段 / U1↔U2 交换）
  *
  * 与 meas_pack 同样的规矩（用户拍板）：**不用 TLV**，一条命令一套定长布局，
  * 全大端，逐字节读写，长度带编译期断言；两片各一份逐字节副本。
  *
  * 线格式（45 字节）：
  *   [0]      ver       = 1
  *   [1]      flags     PULSE_SNAP_F_*
  *   [2..3]   window_ms 本次测量窗口（u16）
  *   [4]      phase_count = 3
  *   [5]      rsvd      = 0
  *   [6..]    3 相 × 13 字节，顺序 = A,B,C（与 measure 的信号序号一致）：
  *              period_ns u32 | high_ns u32 | duty_permille u16 | edges u16 | flags u8
  ******************************************************************************
  */
#ifndef __PULSE_PACK_H
#define __PULSE_PACK_H

#include <stdint.h>
#include "pulse_cap.h"
#include "jig_mode.h"

/* 头部 6 + 3 相 × 13 = 45 */
#define PULSE_PACK_LEN       45u

/* 0x000F 脉冲捕获的应答长度 = 状态1 + 本片45 + 对侧45 = 91
 * （ADC 测量走另一条命令 0x000E，应答 77B，两条命令各自独立不糅在一起） */
#define PULSE_PACK_RESP_LEN  91u

/* 装得进链路单帧（LINK_MAX_PAYLOAD=64）与 PC 单帧 DATA（256） */
#ifndef PULSE_PACK_LINK_MAX_PAYLOAD
#define PULSE_PACK_LINK_MAX_PAYLOAD  64u
#endif
#ifndef PULSE_PACK_FRAME_MAX_DATA
#define PULSE_PACK_FRAME_MAX_DATA    256u
#endif

/**
  * @brief  打包一组三相结果
  * @param  s 可为 0（此时按"无数据"打包：全 0 + 无信号标志）
  * @retval 写入字节数（= PULSE_PACK_LEN）；容量不足返回 0
  */
uint16_t PulsePack_Pack(uint8_t *out, uint16_t max, const pulse_snapshot_t *s);

/**
  * @brief  解包
  * @retval 1 = 成功, 0 = 长度/参数不对
  */
uint8_t  PulsePack_Unpack(const uint8_t *in, uint16_t len, pulse_snapshot_t *s);

/**
  * @brief  比较两片同一相的结果（U1 的"输出端" vs U2 的"输入端"）
  * @param  diff [out, 可空] 每相置 1 = 该相两端对不上
  * @retval 置了差异位的相数
  * @note   判据（详见 docs/12）：
  *           - 一端有信号、另一端没有 → 差异（驱动级/接线问题）
  *           - 两端都有 → 比**频率**：周期差 > PULSE_PEER_TOL_PPM 算差异
  *           - 占空比**只记录不判**（死区/驱动延时会让两边占空比天然不同）
  */
uint8_t  PulsePack_Compare(const pulse_snapshot_t *out_side,
                           const pulse_snapshot_t *in_side,
                           uint8_t *diff);

/* ④ 的判据开关：见 jig_mode.h §3.2（随 JIG_MODE 联动；可单独覆盖） */

/* 两端周期允许的相对偏差（ppm）：10000 = 1% */
#ifndef PULSE_PEER_TOL_PPM
#define PULSE_PEER_TOL_PPM   10000u
#endif

#endif /* __PULSE_PACK_H */
