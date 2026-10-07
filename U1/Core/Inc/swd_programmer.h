/**
  ******************************************************************************
  * @file    swd_programmer.h
  * @brief   FOC 板 SWD 烧录（两线：SWCLK + SWDIO，不用 NRST），非阻塞状态机
  *
  * 流程：连接(SWD-DP 握手/上电/选 AP) → 停内核+冻结看门狗+认型号
  *      → 校验两份镜像头（擦除之前！）→ 按扇区擦除 → 逐块写 → 回读校验
  *      → SYSRESETREQ 复位运行
  *
  * 与旧 UART ISP 方案的区别：
  *   - 不依赖被烧板的系统 bootloader，也不依赖 BOOT0/RESET 跳线；
  *   - 可以直接回读校验（读回的是目标 AHB 总线上的真实内容）；
  *   - 可以顺手读被烧板里的固件版本（0x000D 不再需要被烧板配合）。
  ******************************************************************************
  */
#ifndef __SWD_PROGRAMMER_H
#define __SWD_PROGRAMMER_H

#include <stdint.h>

/** 初始化 SWD 引脚（swd_config.h 中配置；当前为占位引脚） */
void    SWDP_Init(void);

/** 启动烧录流程（调用方需已完成前置检查：镜像存在、大小合法） */
void    SWDP_Start(void);

/** 主循环步进：一次只推进一步（一次握手 / 擦一个扇区 / 写 128 字节 / 校验一块 / 复位运行） */
void    SWDP_Step(void);

/** @retval 1 = 仍在烧录中, 0 = 已结束（成功或失败） */
uint8_t SWDP_Busy(void);

/** @retval 结束时返回 APP_STATUS_OK 或具体错误码 */
uint8_t SWDP_Result(void);

/**
  * @brief  读被烧板 App 分区里的固件版本号（0x000D）
  * @param  ver [out] 3 字节版本号
  * @retval APP_STATUS_OK 或 APP_STATUS_ERR_TIMEOUT（连不上/读不到）
  * @note   版本号在 App 分区中的偏移由 FOC 工程决定，见 DUT_VER_OFFSET（占位）。
  */
uint8_t SWDP_QueryTargetVersion(uint8_t ver[3]);

/* ---- 诊断（失败时用来定位到底卡在哪一步） ---- */
uint32_t SWDP_LastIdcode(void);
uint16_t SWDP_LastDevId(void);
uint32_t SWDP_LastOffset(void);      /* 已写到哪个偏移 */
uint8_t  SWDP_LastSwdError(void);    /* 原始的 swd_status_t 取负 */

#endif /* __SWD_PROGRAMMER_H */
