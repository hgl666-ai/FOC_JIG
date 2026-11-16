/**
  ******************************************************************************
  * @file    fx_bus.h
  * @brief   M1 治具 —— 副芯片通讯（USART1）与 DUT 命令通讯（UART5）
  *
  * 一、治具副芯片（USART1，115200 8N1）
  * ---------------------------------------------------------------------------
  * 正式上线时 USART1 的日志关闭，这条线只跑副芯片协议。
  * 帧格式（字段大端，CRC-16/MODBUS，计算范围 = A5 .. DATA 末字节）：
  *
  *     A5 5C | CMD(1) | LEN(1) | DATA(LEN) | CRC16(2)
  *
  *   请求（治具 -> 副芯片）：
  *     0x01 PING      LEN=0
  *     0x10 IO_READ   LEN=0
  *     0x11 VOLT_READ LEN=0
  *     0x12 ADC_READ  LEN=0
  *
  *   应答（副芯片 -> 治具，CMD 与请求相同）：
  *     0x01 PING      LEN=1  [0] 协议版本 = 0x01
  *     0x10 IO_READ   LEN=1  [0] IO 电平位图：
  *                              bit0 KEY3   bit1 KEY4   bit2 KEY1
  *                              bit3 KEY2   bit4 TBS    bit5 KEY0
  *                              bit6 TEST-INT            bit7 MMS
  *     0x11 VOLT_READ LEN=6  [0..1] U1-3V3 mV
  *                           [2..3] U1-5V  mV
  *                           [4..5] U1-24V mV
  *     0x12 ADC_READ  LEN=7  [0..1] U1-NHT mV
  *                           [2..3] U1-NTC mV
  *                           [4..5] MER mV（没有 ADC，固定 0）
  *                           [6]    MER 数字电平（0/1）
  *
  *   副芯片通讯不上时，所有采集值按"通过"处理，见各检测函数。
  *
  * 二、DUT 命令（UART5 <-> DUT USART1，256000 8N1）
  * ---------------------------------------------------------------------------
  *   "M119 A\n"  返回 0..7 的传感器触发状态
  *   "M105 V\n"  返回 0..2 的 ADC 电压（mV）
  * 解析结果中的极性换算、通道顺序都在本模块完成，调用者拿到的就是
  * 逻辑电平 / 明确的 NHT、NTC、MER 电压。
  ******************************************************************************
  */
#ifndef __FX_BUS_H
#define __FX_BUS_H

#include <stdint.h>

/*------------------------------------------------------------------------------
 * DUT M119 A 的传感器顺序（与 DUT gcode.c 中打印顺序一致）
 *---------------------------------------------------------------------------*/
typedef enum {
    FX_DUT_IO_KEY0 = 0,
    FX_DUT_IO_KEY1,
    FX_DUT_IO_KEY2,
    FX_DUT_IO_KEY3,
    FX_DUT_IO_KEY4,
    FX_DUT_IO_TBS,
    FX_DUT_IO_TEST_INT,
    FX_DUT_IO_MMS,
    FX_DUT_IO_QTY
} fx_dut_io_t;

/* DUT M105 V 的 ADC 顺序（DUT 当前固件：0=NTC, 1=MER, 2=NHT） */
typedef enum {
    FX_DUT_ADC_NTC = 0,
    FX_DUT_ADC_MER,
    FX_DUT_ADC_NHT,
    FX_DUT_ADC_QTY
} fx_dut_adc_t;

/* 副芯片电压通道顺序 */
typedef enum {
    FX_SUB_VOLT_3V3 = 0,
    FX_SUB_VOLT_5V,
    FX_SUB_VOLT_24V,
    FX_SUB_VOLT_QTY
} fx_sub_volt_t;

/* 副芯片 ADC 通道顺序 */
typedef enum {
    FX_SUB_ADC_NHT = 0,
    FX_SUB_ADC_NTC,
    FX_SUB_ADC_MER,
    FX_SUB_ADC_QTY
} fx_sub_adc_t;

void fx_bus_init(void);

/** 主循环 1ms 任务。当前副芯片事务都是"命令-应答"阻塞式，
 *  这里只做预留（例如以后要加心跳时放这里）。 */
void fx_bus_poll(void);

/** PING 副芯片，1=在线 */
uint8_t fx_bus_subchip_ping(void);

/** 读副芯片 8 路 IO 电平位图，1=成功（位定义见文件头） */
uint8_t fx_bus_subchip_read_io(uint8_t *bitmap);

/** 读副芯片 3 路电压，1=成功；mv 顺序为 3V3, 5V, 24V */
uint8_t fx_bus_subchip_read_volt(uint16_t mv[FX_SUB_VOLT_QTY]);

/** 读副芯片 3 路 ADC，1=成功；mv 顺序为 NHT, NTC, MER；
 *  mer_level 可为 NULL。副芯片不通时返回 0。 */
uint8_t fx_bus_subchip_read_adc(uint16_t mv[FX_SUB_ADC_QTY], uint8_t *mer_level);

/** 读 DUT 的 8 路传感器逻辑电平，1=成功；顺序见 fx_dut_io_t。
 *  内部已把 "triggered" 按 DUT 的触发极性换算成实际引脚电平。 */
uint8_t fx_bus_dut_read_io(uint8_t levels[FX_DUT_IO_QTY]);

/** 读 DUT 的 3 路 ADC 电压（mV），1=成功；顺序见 fx_dut_adc_t。 */
uint8_t fx_bus_dut_read_adc(uint16_t mv[FX_DUT_ADC_QTY]);

#endif /* __FX_BUS_H */
