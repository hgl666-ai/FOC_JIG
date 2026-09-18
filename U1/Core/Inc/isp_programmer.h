/**
  ******************************************************************************
  * @file    isp_programmer.h
  * @brief   FOC 板 ISP 烧录 (AN3155 系统 bootloader 协议), 非阻塞状态机
  ******************************************************************************
  */
#ifndef __ISP_PROGRAMMER_H
#define __ISP_PROGRAMMER_H

#include <stdint.h>

/**
  * @brief 初始化 BOOT0 / RESET 控制引脚 (isp_config.h 中配置)
  * @note  仅当 ISP_CTRL_PINS_CONFIRMED=1 时才真正配置外部引脚
  */
void    ISP_GpioInit(void);

/**
  * @brief 启动 ISP 烧录流程 (调用方需已完成前置检查)
  */
void    ISP_Start(void);

/**
  * @brief 主循环步进 — 每次只推进一步 (一次握手 / 擦除 / 一个 256B 块 / 跳转)
  * @note  必须在 APP_Run 里每轮调用
  */
void    ISP_Step(void);

/** @retval 1 = 仍在烧录中, 0 = 已结束(成功或失败) */
uint8_t ISP_Busy(void);

/** @retval 结束时返回 APP_STATUS_OK 或 APP_STATUS_ERR_PROG */
uint8_t ISP_Result(void);

/**
  * @brief 查询被烧板运行态固件版本 (0x000D)
  * @param  ver [out] 3 字节版本号
  * @retval APP_STATUS_OK 或 APP_STATUS_ERR_TIMEOUT
  */
uint8_t ISP_QueryTargetVersion(uint8_t ver[3]);

#endif /* __ISP_PROGRAMMER_H */
