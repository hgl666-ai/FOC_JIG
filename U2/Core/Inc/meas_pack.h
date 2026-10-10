/**
  ******************************************************************************
  * @file    meas_pack.h
  * @brief   测量数据的**定长线格式**编解码（PC 上报 与 U1↔U2 链路共用）
  *
  * 设计原则（用户 2026-10 拍板）：
  *   - **不搞 TLV**：一条命令一套固定布局，PC 侧按下标取值，不做嵌套解析；
  *   - 布局版本就用外层帧的 LEN 判定：LEN 不认识 = 固件/上位机版本不匹配；
  *   - 全部大端，逐字节读写（不用 C struct 直接 memcpy，避开对齐/端序差异）；
  *   - 编解码只有这一份代码，两片各一份**逐字节副本**（回归脚本 fc /b 比对）。
  *
  * 三个长度都是编译期常量，并且带静态断言钉死（改布局必然要动这里）。
  *
  * 内存表示 meas_snapshot_t 与线格式无关：结构体带填充，线格式是紧凑的。
  ******************************************************************************
  */
#ifndef __MEAS_PACK_H
#define __MEAS_PACK_H

#include <stdint.h>
#include "measure.h"

/* ------------------------------------------------------------------ */
/* 长度（改这些常量 = 改协议，必须同步 docs 与上位机）                  */
/* ------------------------------------------------------------------ */
/* 快照：flags(1) ch_count(1) round(2) dt_ms(2) vdda_mv(2) cfg_crc(2) = 10
 *       + 9 通道 × (raw 2 + mv 2 + flags 1) = 45                        */
#define MEAS_PACK_SNAP_LEN      55u
/* 交叉校验块：cc_flags(1) + mask(2) + 9 路对侧 mv(2×9=18) = 21          */
#define MEAS_PACK_CC_LEN        21u
/* 0x000E 应答 DATA：命令状态(1) + 快照(55) + 交叉校验块(21) = 77        */
#define MEAS_PACK_RESP_LEN      77u

/* 快照 55B 必须装得进链路单帧（LINK_MAX_PAYLOAD 见 uart_link.h） */
#ifndef MEAS_PACK_LINK_MAX_PAYLOAD
#define MEAS_PACK_LINK_MAX_PAYLOAD  64u
#endif
/* 77B 应答必须装得进 PC 侧的单帧 DATA（FRAME_MAX_DATA 见 flash_map.h） */
#ifndef MEAS_PACK_FRAME_MAX_DATA
#define MEAS_PACK_FRAME_MAX_DATA    256u
#endif

/* ------------------------------------------------------------------ */
/* 编解码                                                              */
/* ------------------------------------------------------------------ */
/**
  * @brief  把快照编成 55B 线格式
  * @param  out 输出缓冲；max 必须 >= MEAS_PACK_SNAP_LEN
  * @retval 写入字节数（= MEAS_PACK_SNAP_LEN）；参数或容量不对返回 0
  */
uint16_t MeasPack_Snapshot(uint8_t *out, uint16_t max, const meas_snapshot_t *s);

/**
  * @brief  解 55B 线格式为快照
  * @retval 1 = 成功, 0 = 长度/参数不对
  */
uint8_t  MeasPack_UnpackSnapshot(const uint8_t *in, uint16_t len, meas_snapshot_t *s);

/**
  * @brief  编码交叉校验块（含对侧 9 路 mv）
  * @param  cc_flags MEAS_CC_F_*
  * @param  mask     超差掩码（MEAS_Compare 的返回值）
  * @param  peer     对侧快照（取其 9 路 mv；可传 0 = 只发标志与掩码，mv 填 0）
  * @retval 写入字节数（= MEAS_PACK_CC_LEN）；容量不足返回 0
  */
uint16_t MeasPack_CcBlock(uint8_t *out, uint16_t max,
                          uint8_t cc_flags, uint16_t mask,
                          const meas_snapshot_t *peer);

/**
  * @brief  解交叉校验块
  * @param  peer [out] 对侧 9 路 mv 会填进 peer->ch[i].mv（其余字段不动）
  * @retval 1 = 成功, 0 = 长度/参数不对
  */
uint8_t  MeasPack_UnpackCcBlock(const uint8_t *in, uint16_t len,
                                uint8_t *cc_flags, uint16_t *mask,
                                meas_snapshot_t *peer);

#endif /* __MEAS_PACK_H */
