/**
  ******************************************************************************
  * @file    app.h
  * @brief   治具主控 U1 应用层总入口
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#include "main.h"
#include <stdint.h>
#include "uart_link.h"      /* link_status_t (链路状态访问接口用) */

/*===== 顶层系统状态 =====*/
typedef enum {
    APP_STATE_INIT = 0,
    APP_STATE_IDLE,
    APP_STATE_UPLOADING,     /* 固件上传中 (只接受 0x000A / 0x000B) */
    APP_STATE_PROGRAMMING,   /* SWD 烧录中  (只接受 0x0112, 回 BUSY) */
    APP_STATE_TESTING,       /* 测量测试中  (0x000E mode=1, 跑完判定才回 ACK) */
    APP_STATE_DUT,           /* DUT 透传中等应答 (0x0011; 需要 JIG_DUT_LINK=1) */
    APP_STATE_ENC,           /* 编码器脉冲串发送中 (0x0012, 约 200ms 后回 ACK) */
    APP_STATE_ENC_PEER,      /* 向 U2 要编码器计数中 (0x0013) */
    APP_STATE_CUR,           /* ⑥ 相电流检测流程中 (0x0014; 需要 JIG_DUT_LINK=1) */
    APP_STATE_VOL,           /* ② 电压检测流程中 (0x0015; 需要 JIG_DUT_LINK=1) */
    APP_STATE_RUN            /* 0x0010 一键全流程编排中 (需要 JIG_DUT_LINK=1) */,
    APP_STATE_DUT_RAW        /* 0x001F 给 DUT 发原始字节、等它回字的这段窗口 (appended 2026-11-06) */,
    APP_STATE_ASCII          /* PC 串口助手里手敲 AS:xxx 的这条命令行 (appended 2026-11-09) */,
    APP_STATE_FLOW6          /* 0x0022 六步测试流程进行中 (appended 2026-11-09) */
} APP_State_t;

extern APP_State_t app_state;
extern uint16_t    pc_seq;      /* 最近一帧的帧序号, ACK 回显用 */

/*===== API =====*/
void APP_Init(void);
void APP_Run(void);
/**
  * @brief  PC 口收到一个字节：**先判 ASCII 命令行，不是才进帧解析**
  * @note   USB 中断（APP_USB_Receive）与串口（PcLink_Poll）两条路都走这里，
  *         所以"什么算命令行"只有一个出口，不会两条路行为不一致。
  */
void APP_PcInput(uint8_t b);

/*── FOC 板回字的文本解析（纯函数，见 docs/24 §7）────────────────────────────
 * 都在 `#if (JIG_DUT_LINK != 0)` 里实现；调用前不需要任何状态。以上三个
 * 只做"认不认得出"，不碰状态机、不发字节。 */

/**
  * @brief  在字节流里找编码器上报 `AS:A<十进制>B<十进制>C<十进制>`
  * @param  buf/len  收到的原始字节（可以带 `\r\n`、可以前面有别的日志）
  * @param  out_a/out_b/out_c  [out] 三个组的数值（可为 0 表示不要）
  * @retval 1 = 找到了；0 = 没有 / 格式不对
  * @note   必须 A、B、C 三组齐全才算；`AS:A0` 这种单组的不算（那是自检状态码）
  */
uint8_t APP_AsParseEnc(const uint8_t *buf, uint16_t len,
                       uint16_t *out_a, uint16_t *out_b, uint16_t *out_c);

/**
  * @brief  解析 FOC 的编码器上报 `AS:D,A=<十进制>,B=<十进制>,Z=<十进制>`
  * @retval 1 = 三个键都找到了；0 = 没有/格式不对
  * @note   不要求键的顺序，但要求出现 `AS:D` 前缀；数值按 u16 截断保护。
  */
uint8_t APP_AsParseEncD(const uint8_t *buf, uint16_t len,
                        uint16_t *out_a, uint16_t *out_b, uint16_t *out_z);

/**
  * @brief  在字节流里找前缀（大小写不敏感）
  * @retval 1 = 找到；0 = 没有
  */
uint8_t APP_AsHasPrefix(const uint8_t *buf, uint16_t len, const char *prefix);

/**
  * @brief  找自检状态码 `AS:A0`~`AS:A3`
  * @retval 0~3 = A0~A3；0xFF = 没找到
  * @note   要求状态码后面是 `\r`/`\n`/空格/0/串尾，避免把 `AS:A0B0C0` 认成 A0
  */
uint8_t APP_AsFindCode(const uint8_t *buf, uint16_t len);

void APP_USB_Receive(uint8_t *data, uint32_t len);   /* 由 CDC_Receive_FS 回调 */

/*===== 帧发送 =====*/
void APP_SendAck(uint16_t cmd, const uint8_t *data, uint16_t len);   /* SEQ = pc_seq */
void APP_SendEvent(uint16_t cmd, const uint8_t *data, uint16_t len); /* SEQ = 0      */

/*===== SPI Flash 槽头部操作 =====*/
/**
  * @brief  读某一槽的固件头（大小/版本/完整标志）
  * @retval 1 = 该槽有完整固件；0 = 空槽或不完整；
  *         2 = **SPI 总线读失败**（调用方必须报 03 ERR_FLASH，不能当空槽）
  * @note   "读不到"和"没上传过"是两件事：前者是硬件故障，后者才回 06 ERR_NO_FW。
  */
uint8_t Flash_ReadHead(uint8_t slot, uint32_t *p_size, uint8_t ver[3], uint8_t *p_flag);
void    Flash_WriteCompleteFlag(uint8_t slot, uint8_t flag);

/*===== 到 U2 的内部链路 (串口) =====*/
/** U2 是否还活着 (最近一次探测有应答)。诊断/自检用, 不影响协议流程 */
uint8_t APP_U2Alive(void);
/** 探测计数: [out] ok=成功次数, fail=超时次数 (任一可传 0) */
void    APP_U2ProbeStats(uint16_t *ok, uint16_t *fail);
/**
  * @brief  两片的链路代码版本是否一致 (1 = 一致)
  * @note   两片各有一份 uart_link.c/uart_port.c 副本。若改了副本却只重烧一片,
  *         链路字段布局可能对不上 → 这里会变成 0。还没收到过 STATUS 时返回 1。
  */
uint8_t APP_U2VerOk(void);
/** 最近一次 U2 上报的状态 (③ 测量交叉校验会用): 1 = 有数据, 0 = 还没收到过 */
uint8_t APP_U2Status(link_status_t *out);

#endif /* __APP_H */
