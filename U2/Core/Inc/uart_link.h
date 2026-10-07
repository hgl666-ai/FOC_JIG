/**
  ******************************************************************************
  * @file    uart_link.h
  * @brief   治具内部链路 (U1 <-> U2) — 在两片之间跑"请求/应答"帧
  *
  * 为什么需要它：
  *   U1 (主控) 需要读 U2 (测量/校验片) 的测量结果做交叉校验（治具的
  *   "一个检测、一个验证"就是靠这条链路落地的）。
  *
  * 帧格式：与 PC 协议完全一致（A5 5B | CMD | SEQ | LEN | DATA | CRC16），
  *   直接复用 protocol_frame.c 的 Frame_Build / Frame_Parse，不另造一套。
  *   链路命令码在 LINK_CMD_* 里，与 PC 的 MGR_CMD_* 不冲突。
  *
  * 角色：
  *   **主机 (U1)**：Link_Request() 发请求并等应答（有界超时）；
  *   **从机 (U2)**：Link_Poll() 收到请求后调应用层处理函数并应答。
  *   两个角色可以同时存在于一个进程里（测试就是这么做的）——所以本模块
  *   **不用全局单例**，状态放在调用方自己声明的 link_t 里。
  *
  * ⚠ 本文件在两片各有一份副本（U1/Core/Inc 与 U2/Core/Inc），必须**逐字节一致**；
 *   uart_port.* / protocol_frame.* 同理。代价是"改一边忘一边"——所以回归脚本
 *   第 7 阶段用 fc /b 机械比对这 3 对文件，不一致直接判回归失败。
 *
 * 线程/上下文：Link_Poll / Link_Request 只在主循环上下文调用；
  *   中断只往 uart_port 的接收环里塞字节（见 uart_port.c 的并发约定）。
  ******************************************************************************
  */
#ifndef __UART_LINK_H
#define __UART_LINK_H

#include <stdint.h>
#include "uart_port.h"

/*==============================================================================
 * 一、链路私有命令码 (只走 U1<->U2 串口，不出现在 PC 协议 xlsx 里)
 *============================================================================*/
#define LINK_CMD_PING       0x0100u   /* 回环自检: U2 把 payload 原样回显        */
#define LINK_CMD_STATUS     0x0101u   /* U2 状态: 版本 / 存活计数 / 自身串口统计  */
#define LINK_CMD_MEAS_GET   0x0102u   /* [③ 预留] 读 U2 的 9 路测量值             */
#define LINK_CMD_MEAS_XCHK  0x0103u   /* [③ 预留] 送 U1 的测量值给 U2 交叉校验     */

/* U2 固件版本 (LINK_CMD_STATUS 上报) */
#define LINK_U2_VER_MAJOR   0x00u
#define LINK_U2_VER_MINOR   0x01u

/*==============================================================================
 * 两片代码版本（链路自检用）—— 这一份 uart_link.c 在两片里各有一份副本
 *----------------------------------------------------------------------------
 * 两片是各编各的固件，所以"改了链路代码却只重烧了一片"必然会发生，
 * 后果是静默的字段错位（STATUS 解出垃圾）或者一直超时，很难看出是烧录不同步。
 *
 * 规则：**任何会改变链路对外行为（帧格式 / 字段布局 / 时序）的改动，
 *       必须把这个值 +1**（两份副本一起改），两片一起重编重烧。
 * 从机在 STATUS 里回这个值，主机比对；不一致时 U1 会把 APP_U2VerOk() 置 0。
 *============================================================================*/
#define LINK_CODE_VER     0x0101u   /* v1.1: STATUS 增加代码版本字段([2..3]) */

/* 链路级状态码: 放在应答数据第 1 字节 (PING 除外, 它的数据区是回环内容) */
#define LINK_ST_OK          0x00u   /* 正常      */
#define LINK_ST_NIY         0xFFu   /* 该命令还没实现 */

/* LINK_CMD_STATUS 应答数据区布局 (22 字节; 除版本/存活外都是大端 u32, 饱和不回卷)
 *   [0]      固件主版本     [1]      固件次版本
 *   [2..3]   链路代码版本 ( = LINK_CODE_VER, u16 大端)
 *   [4..5]   存活计数 (每应答一次 +1, u16 大端)
 *   [6..9]   链路统计 rx_frames
 *   [10..13] 链路统计 rx_crc_err
 *   [14..17] 端口统计 rx_bytes
 *   [18..21] 端口统计 rx_drop
 * 打包/解包都用下面的 Link_PackStatus / Link_UnpackStatus, 两片共用一份, 不会跑偏。 */
#define LINK_STATUS_LEN     22u

/* 单帧 DATA 上限：比 PC 侧 256B 小很多——链路上只跑小控制帧，
 * 缓冲按 .bss 静态分配，不吃栈（见项目栈深约定）。 */
#define LINK_MAX_PAYLOAD    64u
#define LINK_RAW_MAX        (8u + LINK_MAX_PAYLOAD + 2u)

/*==============================================================================
 * 二、从机应用层处理函数
 *----------------------------------------------------------------------------
 * @param  cmd    收到的命令码
 * @param  in     请求数据区 (inlen 字节)
 * @param  inlen  请求数据长度
 * @param  out    应答数据区 (容量 LINK_MAX_PAYLOAD)
 * @param  outlen [out] 应答数据长度
 * @retval 1 = 正常应答, 0 = 不对答 (命令不认识/是广播, 静默丢弃)
 *============================================================================*/
typedef uint8_t (*link_slave_fn_t)(uint16_t cmd, const uint8_t *in, uint16_t inlen,
                                   uint8_t *out, uint16_t *outlen);

/*==============================================================================
 * 三、LINK_CMD_STATUS 的数据结构 (编解码共用)
 *============================================================================*/
typedef struct {
    uint8_t  ver_major;
    uint8_t  ver_minor;
    uint16_t code_ver;    /* 对端的链路代码版本, 与本片 LINK_CODE_VER 比对 */
    uint16_t alive;
    uint32_t rx_frames;
    uint32_t rx_crc_err;
    uint32_t rx_bytes;
    uint32_t rx_drop;
} link_status_t;

/*==============================================================================
 * 四、统计 (诊断用，单调计数)
 *============================================================================*/
typedef struct {
    uint32_t tx_frames;     /* (主机) 发出的请求帧数                          */
    uint32_t rx_frames;     /* 收到的合法帧数 (含不匹配的陈旧帧)               */
    uint32_t tx_timeout;    /* 请求等应答超时次数                             */
    uint32_t tx_busy;       /* 上一次请求还没结束就再次发起 → 被拒次数         */
    uint32_t tx_nospace;    /* 请求帧没能全部塞进发送队列 → 放弃次数           */
    uint32_t rx_crc_err;    /* CRC 校验失败的帧数                             */
    uint32_t rx_len_err;    /* 长度字段非法 (> LINK_MAX_PAYLOAD) 的帧数        */
    uint32_t rx_resync;     /* 丢同步后丢弃的字节数                            */
    uint32_t rx_stale;      /* SEQ 与当前等待的请求不匹配 → 丢弃的应答帧数     */
    uint32_t slave_reply;   /* (从机) 已应答的帧数                            */
    uint32_t slave_drop;    /* (从机) 处理函数返回"不应答"的帧数               */
} link_stats_t;

/*==============================================================================
 * 五、链路对象
 *============================================================================*/
typedef struct {
    uint8_t         inited;
    uint8_t         is_master;
    uart_id_t       port;
    link_slave_fn_t slave_fn;

    /* 收帧装配 (逐字节状态机) */
    uint8_t         rx_raw[LINK_RAW_MAX];
    uint8_t         rx_data[LINK_MAX_PAYLOAD];   /* 解出的请求数据区 */
    uint16_t        rx_cnt;         /* 已收字节 */
    uint16_t        rx_need;        /* 整帧长度; 0 = 还不知道 */

    /* 主机请求状态 */
    uint16_t        req_seq;        /* 递增序号(从 1 开始, 0 保留) */
    uint16_t        wait_seq;       /* 当前等待的应答序号 */
    uint32_t        req_t0;         /* 请求发出时刻 (HAL_GetTick) */
    uint8_t         waiting;
    uint8_t         resp_ready;
    uint16_t        resp_cmd;
    uint16_t        resp_len;
    uint8_t         resp[LINK_MAX_PAYLOAD];

    /* 从机应答缓冲 */
    uint8_t         tx_raw[LINK_RAW_MAX];

    link_stats_t    st;
} link_t;

/*==============================================================================
 * 六、API
 *============================================================================*/
/** 初始化链路对象 (只绑定端口/角色，不动硬件；串口本身由 UartCfg_Init 起) */
void        Link_Init(link_t *l, uart_id_t port, uint8_t is_master);

/** 从机注册应用层处理函数 (主机侧可不注册) */
void        Link_SetSlaveHandler(link_t *l, link_slave_fn_t fn);

/** 主循环轮询：收字节 → 组帧 → (主机)存应答 / (从机)调处理函数并应答 */
void        Link_Poll(link_t *l);

/**
  * @brief  主机发一次请求并等应答 (调用期间会自己轮询接收)
  * @param  timeout_ms 等待上限; 0 = 只走一遍非阻塞轮询, 立刻返回
  * @param  out        [out, 可空] 应答数据, 缓冲必须 >= LINK_MAX_PAYLOAD
  * @param  outlen     [out, 可空] 应答长度
  * @retval 1 = 收到匹配应答, 0 = 超时/参数错/上一次请求未完成
  * @note   本函数最多阻塞 timeout_ms (内部只做 UartPort 收发, 不喂 USB 侧),
  *         因此调用方给的时间要短 (交叉校验用 20~50ms 级)。
  *         主循环里想要"完全不阻塞"就用下面的 Start/Poll 两个函数。
  */
uint8_t     Link_Request(link_t *l, uint16_t cmd,
                         const uint8_t *in, uint16_t inlen,
                         uint8_t *out, uint16_t *outlen, uint32_t timeout_ms);

/**
  * @brief  非阻塞: 组帧发出请求后立刻返回 (不等应答)
  * @retval 1 = 已发出, 0 = 失败 (角色不对/上一次还没结束/发送队列满/长度越界)
  */
uint8_t     Link_RequestStart(link_t *l, uint16_t cmd,
                              const uint8_t *in, uint16_t inlen);

/**
  * @brief  非阻塞: 推进一次请求 (每次主循环调一次即可)
  * @param  timeout_ms 从发出算起的等待上限
  * @param  out        [out, 可空] 应答数据, 缓冲必须 >= LINK_MAX_PAYLOAD
  * @param  outlen     [out, 可空] 应答长度
  * @retval 0 = 还在等, 1 = 收到应答(out/outlen 有效), 2 = 超时/无进行中的请求
  */
uint8_t     Link_RequestPoll(link_t *l, uint32_t timeout_ms,
                             uint8_t *out, uint16_t *outlen);

/** 是否有请求正在等待应答 */
uint8_t     Link_Busy(const link_t *l);

/** 复位链路状态 (丢弃半截帧/未完成请求)。用于错误恢复或换波特率后 */
void        Link_Reset(link_t *l);

/** 统计 (只读) */
const link_stats_t *Link_Stats(const link_t *l);

/*------------------------------------------------------------------------------
 * LINK_CMD_STATUS 编解码 (从机打包 / 主机解包, 同一份代码保证两边一致)
 *--------------------------------------------------------------------------*/
/**
  * @brief  打包状态应答数据区
  * @param  out       输出缓冲 (需 >= LINK_STATUS_LEN)
  * @param  max       输出缓冲容量
  * @param  ver       版本号 [主, 次]
  * @param  alive     存活计数 (从机自己维护)
  * @param  ls        [可空] 链路统计
  * @param  us        [可空] 该链路端口的串口统计
  * @retval 写入字节数; 容量不足返回 0
  */
uint16_t Link_PackStatus(uint8_t *out, uint16_t max,
                         const uint8_t ver[2], uint16_t alive,
                         const link_stats_t *ls, const uart_stats_t *us);

/**
  * @brief  解包状态应答数据区
  * @retval 1 = 长度与内容合法, 0 = 长度不对
  */
uint8_t  Link_UnpackStatus(const uint8_t *in, uint16_t len, link_status_t *out);

/**
  * @brief  对端的链路代码版本是否与本片一致 (1 = 一致)
  * @note   不一致 = 两片里有一片没跟着重烧, 链路字段可能对不上。
  *         U1 用它把"烧录不同步"变成一个可见的状态, 而不是等着解析出垃圾。
  */
uint8_t  Link_CheckVersion(const link_status_t *st);

/** 最近一次被采纳的应答里的命令码 (诊断用; 没收到过应答时为 0) */
uint16_t Link_LastRespCmd(const link_t *l);

#endif /* __UART_LINK_H */
