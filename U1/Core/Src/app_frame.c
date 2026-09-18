/**
  ******************************************************************************
  * @file    app_frame.c
  * @brief   PC→U1 帧接收状态机 (环形缓冲 + 严格头校验 + 失步重同步)
  *
  * 修复的两个问题 (由 test/protocol_test.c 暴露):
  *   1) 原实现把解析器直接放在中断里跑, 一帧收完(frame_ready=1)后若上层还没取走,
  *      后续字节会继续写 frame_buf 并把 frame_pos 一直加下去 → 缓冲区越界。
  *      现在改成 环形缓冲 + 主循环解析, 帧满即停止出队。
  *   2) 头部只认 A5 5B 就无条件吃满 8 字节, 遇到数据里出现的伪帧头会长时间失步。
  *      现在 LEN 非法时用 frame_resync() 在已缓存字节里找下一个 A5 5B 重新对齐。
  *      伪帧头最终由 CRC 兜底拒绝。
  ******************************************************************************
  */
#include "app_frame.h"
#include "flash_map.h"
#include "protocol_frame.h"
#include <string.h>

/*===== 接收环形缓冲 (ISR 写 / 主循环读) =====*/
#define RING_SIZE   (512u)                       /* 必须是 2 的幂 */
#define RING_MASK   (RING_SIZE - 1u)

static volatile uint8_t  ring[RING_SIZE];
static volatile uint16_t ring_head = 0u;         /* 写指针 (ISR)      */
static volatile uint16_t ring_tail = 0u;         /* 读指针 (主循环)   */

/*===== 解析器状态 =====*/
typedef enum { ST_HDR0 = 0, ST_HDR1, ST_HEAD, ST_BODY } FrameSt_t;

static uint8_t   frame_buf[APP_FRAME_BUF_SIZE];
static uint16_t  frame_pos   = 0u;
static uint16_t  frame_dlen  = 0u;
static uint8_t   frame_ready = 0u;
static FrameSt_t frame_st    = ST_HDR0;

/*--------------------------------------------------------------------------
 * 失步重同步: 在当前已缓存字节里寻找下一个 A5 5B, 把解析器对齐过去
 *------------------------------------------------------------------------*/
static void frame_resync(void)
{
    uint16_t i;

    for (i = 1u; (uint16_t)(i + 1u) < frame_pos; i++) {
        if (frame_buf[i] == FRAME_HDR0 && frame_buf[i + 1u] == FRAME_HDR1) break;
    }

    if ((uint16_t)(i + 1u) < frame_pos) {
        uint16_t n = (uint16_t)(frame_pos - i);
        memmove(frame_buf, &frame_buf[i], n);
        frame_pos = n;                  /* >= 2, 头两字节已就绪 */
        frame_st  = ST_HEAD;
    } else if (frame_pos > 0u && frame_buf[frame_pos - 1u] == FRAME_HDR0) {
        frame_buf[0] = FRAME_HDR0;      /* 末尾可能是新帧头的第一个字节 */
        frame_pos = 1u;
        frame_st  = ST_HDR1;
    } else {
        frame_pos = 0u;
        frame_st  = ST_HDR0;
    }
}

/*--------------------------------------------------------------------------
 * 解析 1 字节 (主循环上下文)
 *------------------------------------------------------------------------*/
static void frame_feed(uint8_t b)
{
    if (frame_ready) return;            /* 上一帧未被取走: 不该发生, 防御性丢弃 */

    switch (frame_st) {

    case ST_HDR0:
        if (b == FRAME_HDR0) {
            frame_buf[0] = b;
            frame_pos = 1u;
            frame_st  = ST_HDR1;
        }
        break;

    case ST_HDR1:
        if (b == FRAME_HDR1) {
            frame_buf[1] = b;
            frame_pos = 2u;
            frame_st  = ST_HEAD;
        } else if (b == FRAME_HDR0) {
            frame_buf[0] = b;           /* 连续 A5: 停在 HDR1 等下一个 5B */
            frame_pos = 1u;
        } else {
            frame_pos = 0u;
            frame_st  = ST_HDR0;
        }
        break;

    case ST_HEAD:                       /* CMD(2) SEQ(2) LEN(2) */
        frame_buf[frame_pos++] = b;
        if (frame_pos == 8u) {
            frame_dlen = (uint16_t)(((uint16_t)frame_buf[6] << 8) | frame_buf[7]);
            if (frame_dlen > FRAME_MAX_DATA) {
                frame_resync();         /* LEN 非法 → 立即重新对齐 */
                break;
            }
            frame_st = ST_BODY;
        }
        break;

    case ST_BODY:                       /* DATA(L) + CRC(2) */
        frame_buf[frame_pos++] = b;
        if (frame_pos == (uint16_t)(8u + frame_dlen + 2u)) {
            frame_ready = 1u;
            frame_st    = ST_HDR0;
        }
        break;

    default:
        frame_st = ST_HDR0;
        break;
    }
}

/*==============================================================================
 * 公开 API
 *============================================================================*/
void APP_Frame_Init(void)
{
    ring_head   = 0u;
    ring_tail   = 0u;
    frame_pos   = 0u;
    frame_dlen  = 0u;
    frame_ready = 0u;
    frame_st    = ST_HDR0;
}

void APP_Frame_Feed(uint8_t byte)
{
    uint16_t next = (uint16_t)((ring_head + 1u) & RING_MASK);
    if (next == ring_tail) return;      /* 环形缓冲满: 丢弃 (USB 链路上不该出现) */
    ring[ring_head] = byte;
    ring_head = next;
}

void APP_Frame_Poll(void)
{
    while (ring_tail != ring_head) {
        uint8_t b = ring[ring_tail];
        ring_tail = (uint16_t)((ring_tail + 1u) & RING_MASK);

        frame_feed(b);

        /* 攒满一帧就先交给上层处理, 剩余字节留在环形缓冲里, 下轮再取 */
        if (frame_ready) break;
    }
}

uint8_t APP_Frame_Pending(void)
{
    return frame_ready;
}

uint8_t *APP_Frame_Consume(uint16_t *len)
{
    if (len) *len = frame_pos;
    frame_ready = 0u;
    frame_pos   = 0u;
    frame_st    = ST_HDR0;
    return frame_buf;
}
