/**
  ******************************************************************************
  * @file    protocol_frame.h
  * @brief   PC ↔ 治具 U1 帧层 (A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA | CRC16(2))
  * @note    全部字段大端序; CRC = MODBUS/CRC16, 范围 帧头..数据末字节
  ******************************************************************************
  */
#ifndef __PROTOCOL_FRAME_H
#define __PROTOCOL_FRAME_H

#include <stdint.h>

#define FRAME_HDR0          (0xA5u)
#define FRAME_HDR1          (0x5Bu)
#define FRAME_OVERHEAD      (10u)   /* 2 帧头 + 2 命令 + 2 序号 + 2 长度 + 2 CRC */

/**
  * @brief  MODBUS/CRC16 (多项式 0xA001 反射, 初值 0xFFFF)
  * @param  data 数据首地址
  * @param  len  参与计算的字节数
  * @retval 16 位 CRC (主机序)
  */
uint16_t Frame_CRC16(const uint8_t *data, uint16_t len);

/**
  * @brief  组帧 (不含 CRC 校验, 自动追加)
  * @param  buf  输出缓冲, 至少 8+len+2 字节
  * @param  cmd  命令码
  * @param  seq  帧序号
  * @param  data 数据区指针 (len=0 时可传 NULL)
  * @param  len  数据字节数
  * @retval 整帧字节数
  */
uint16_t Frame_Build(uint8_t *buf, uint16_t cmd, uint16_t seq,
                     const uint8_t *data, uint16_t len);

/**
  * @brief  解帧 + CRC 校验
  * @param  raw     原始帧首地址
  * @param  raw_len 原始帧字节数
  * @param  cmd     [out] 命令码
  * @param  seq     [out] 帧序号
  * @param  data    [out] 数据区拷贝目标 (可为 NULL)
  * @param  len     [out] 数据字节数
  * @retval 1=解析成功且 CRC 正确, 0=失败
  */
uint8_t Frame_Parse(const uint8_t *raw, uint16_t raw_len,
                    uint16_t *cmd, uint16_t *seq,
                    uint8_t *data, uint16_t *len);

#endif /* __PROTOCOL_FRAME_H */
