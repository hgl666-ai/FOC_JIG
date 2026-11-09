/**
  ******************************************************************************
  * @file    app_frame.h
  * @brief   PC→U1 字节流帧接收状态机
  *
  * 分工:
  *   APP_Frame_Feed()  中断上下文 (USB CDC 回调) — 只把字节压进环形缓冲, 极快
  *   APP_Frame_Poll()  主循环上下文 — 从环形缓冲取字节跑解析器
  *   这样解耦后, 一帧解析完成但尚未被取走时, 后续字节不会冲垮帧缓冲。
  ******************************************************************************
  */
#ifndef __APP_FRAME_H
#define __APP_FRAME_H

#include <stdint.h>

void     APP_Frame_Init(void);

/** @brief 中断上下文调用: 把 1 字节压入接收环形缓冲 (满则丢弃) */
void     APP_Frame_Feed(uint8_t byte);

/** @brief 主循环上下文调用: 从环形缓冲取字节解析, 攒满一帧即停 (等上层取走) */
void     APP_Frame_Poll(void);

/** @retval 1 = 有完整帧待处理 */
uint8_t  APP_Frame_Pending(void);

/**
  * @brief  解析器当前攒了多少字节（0 = 空闲，没有半截帧）
  * @note   给"半截帧超时"用（app.c 的 APP_FrameStallCheck）：
  *         一旦因为丢字节把 LEN 读成了大数（例如 0x00A5），解析器会一直等
  *         175 个永远不会来的字节 —— 表现为"发命令永远没应答，但字节确实收到了"。
  *         上层看到这个数长时间不涨，就调 APP_Frame_Init() 重新对齐。
  */
uint16_t APP_Frame_Buffered(void);

/** @brief 取出整帧并复位解析器 */
uint8_t *APP_Frame_Consume(uint16_t *len);

#endif /* __APP_FRAME_H */
