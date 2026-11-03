/**
  ******************************************************************************
  * @file    pc_link.h
  * @brief   PC 通讯口的**传输层开关**：USB CDC  ↔  USART1（只有 U1 有本模块）
  *
  * 为什么要有这一层：
  *   app.c 原来在 APP_SendFrame() 里直接摸 USB 的东西（hUsbDeviceFS.pClassData
  *   的 TxState、CDC_Transmit_FS），PC 口一换成串口，这些调用必须彻底消失
  *   —— 否则 HAL PCD + USBD + 描述符那 ~9.4 KB 仍会被链接进来，32 KB 的
  *   STM32F103C6T6 就装不下（见 jig_mode.h §2.1）。
  *   把"这一根管子"关进本模块后，app.c 只认 3 个函数，命令逻辑一行不用改。
  *
  * 用法（app.c 里就这三处）：
  *   PcLink_TxIdle()        1 = 上一帧已经发完，可以组下一帧（替代查 USB TxState）
  *   PcLink_Send(buf, len)  把一整帧交给 PC 口（替代 CDC_Transmit_FS 的忙等循环）
  *   PcLink_Poll()          主循环：串口模式下把收到的字节喂给帧解析器
  *
  * 收的方向两条路都一样地"只搬字节不解析"：
  *   USB  : CDC_Receive_FS → APP_USB_Receive() → APP_Frame_Feed()   （中断里）
  *   串口 : USART1 中断 → uart_port 环形缓冲 → PcLink_Poll() → APP_Frame_Feed()
  *   两条最后都进同一个 APP_Frame_* 状态机，所以上层看到的字节流完全一致。
  ******************************************************************************
  */
#ifndef __PC_LINK_H
#define __PC_LINK_H

#include <stdint.h>

/**
  * @brief  当前是否可以开始发下一帧
  * @retval 1 = 空闲（上一帧已经全部送到线上）；0 = 还忙 / PC 口不可用
  *
  * 语义与原来的"查 USB TxState"一致：调用方拿不到空闲就**放弃这一帧**
  * （宁可让 PC 超时重试，也不往线上交错半截帧）。
  */
uint8_t PcLink_TxIdle(void);

/**
  * @brief  发一整帧（帧头/CRC 都已经组好，本函数只负责"搬出去"）
  * @param  buf  完整帧（A5 5B ... CRC16）
  * @param  len  帧总长，**发不完整就不发**（绝不截断）
  *
  * USB 分支：等价于原来的 "CDC_Transmit_FS 忙等循环"；
  * 串口分支：整帧进 uart_port 的发送队列（队首空间不够就整帧丢弃）。
  */
void PcLink_Send(const uint8_t *buf, uint16_t len);

/**
  * @brief  主循环里调一次：把 PC 口收到的字节喂进帧解析器
  *
  * USB 分支：空函数（字节在 USB 中断里已经通过 APP_USB_Receive 喂进去了）；
  * 串口分支：从 uart_port 环形缓冲取字节喂 APP_Frame_Feed()。
  */
void PcLink_Poll(void);

#endif /* __PC_LINK_H */
