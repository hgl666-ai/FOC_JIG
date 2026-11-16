/**
  ******************************************************************************
  * @file    flash_mode.h
  * @brief   M1 治具 —— 目标板烧录方式选择（全局编译期宏）
  *
  * 两套方案完全独立，同一时刻只编译一套：
  *
  *   FLASH_MODE_UART5 = 0：
  *     使用 SWD_FLASH.c（PD2=SWCLK/PC12=SWDIO，或 PA13/PA14）。
  *     现有已量产验证的 SWD 直写方案，不要改它的内部代码。
  *
  *   FLASH_MODE_UART5 = 1：
  *     使用 UART_FLASH.c：治具 UART5(PD2/PC12) 接 DUT USART1(PA9/PA10)，
  *     把 DUT 的 BOOT0 拉高并复位，进 ST 系统存储器 bootloader，
  *     按 AN3155 协议擦除/写入（速度约 8 秒/70KB @115200，远快于 SWD 直写）。
  *
  * 切换方式：只改下面这一个宏即可。SWD_FLASH.c 整体被 #if 排除，
  *           不会和 UART_FLASH.c 的代码/全局变量混在一起。
  ******************************************************************************
  */
#ifndef __FLASH_MODE_H
#define __FLASH_MODE_H

/* 1 = UART5 → DUT USART1 系统 bootloader（当前默认）
 * 0 = 原 SWD 位翻转烧录 */
#ifndef FLASH_MODE_UART5
#define FLASH_MODE_UART5        1
#endif

#endif /* __FLASH_MODE_H */
