/**
  ******************************************************************************
  * @file    fx_common.h
  * @brief   M1 治具检测功能公共状态码 / flags
  ******************************************************************************
  */
#ifndef __FX_COMMON_H
#define __FX_COMMON_H

#include <stdint.h>

/* 0x0015 / 0x0016 / 0x0017 ACK 的第 1 字节 */
#define FX_TEST_OK          0x00u   /* 合格 */
#define FX_TEST_FAIL        0x05u   /* 不合格 */

/* 0x0015 / 0x0017 ACK 里每通道的 flags */
#define FX_FLAG_PASS        0x01u   /* 通过 */
#define FX_FLAG_NO_DATA     0x02u   /* 数据不全不判 */
#define FX_FLAG_OUT_RANGE   0x08u   /* 超差 / 不通过 */

#endif /* __FX_COMMON_H */
