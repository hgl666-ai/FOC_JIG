/*
 * 协议层宿主侧单元测试 (gcc, 非目标机)
 * 直接编译固件(两片共用)的 protocol_frame.c / app_frame.c 真源码, 验证:
 *   1) CRC16-MODBUS 与标准校验向量一致 ("123456789" -> 0x4B37)
 *   2) 逐位实现 vs 独立表驱动实现 结果一致
 *   3) Frame_Build -> Frame_Parse 全命令往返 + 打印协议实际字节
 *   4) CRC 错误 / 长度错误 / 帧头错误 被拒绝
 *   5) 字节流状态机: 逐字节喂 / 粘连两帧 / 前置垃圾后重同步 / 超长 LEN 丢弃 /
 *      LEN=256 边界 / 连续 8 帧压力
 *
 * 建模方式与固件一致:
 *   feed(...)        == 中断上下文 (APP_Frame_Feed, 只压环形缓冲)
 *   recv_frame(...)  == 主循环上下文 (APP_Frame_Poll + Pending + Consume)
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "../U1/Core/Inc/protocol_frame.h"
#include "../U1/Core/Inc/app_frame.h"
#include "../U1/Core/Inc/flash_map.h"

static int g_checks = 0;
static int g_fails  = 0;

#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) { printf("  FAIL: %s  (line %d)\n", msg, __LINE__); g_fails++; } \
    } while (0)

/*===== 独立的表驱动 CRC16/MODBUS 实现, 用于交叉验证 =====*/
static uint16_t crc_tbl[256];
static void crc_tbl_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint16_t c = (uint16_t)i;
        for (int j = 0; j < 8; j++)
            c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
        crc_tbl[i] = c;
    }
}
static uint16_t crc16_tabled(const uint8_t *d, int n)
{
    uint16_t c = 0xFFFFu;
    for (int i = 0; i < n; i++) c = (uint16_t)((c >> 8) ^ crc_tbl[(c ^ d[i]) & 0xFFu]);
    return c;
}

static void dump(const char *tag, const uint8_t *b, uint16_t n)
{
    printf("  %-24s", tag);
    for (uint16_t i = 0; i < n; i++) printf("%02X ", b[i]);
    printf("\n");
}

/*===== 中断侧: 只压环形缓冲 =====*/
static void feed(const uint8_t *b, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) APP_Frame_Feed(b[i]);
}

/*===== 主循环侧: 取一整帧, 没有则返回 NULL =====*/
static uint8_t *recv_frame(uint16_t *len)
{
    APP_Frame_Poll();
    if (!APP_Frame_Pending()) { *len = 0; return NULL; }
    return APP_Frame_Consume(len);
}

/*====================================================================*/
static void test_crc(void)
{
    printf("[1] CRC16-MODBUS\n");

    const uint8_t vec[] = "123456789";
    uint16_t c = Frame_CRC16(vec, 9);
    printf("  CRC(\"123456789\") = 0x%04X  (标准值 0x4B37)\n", c);
    CHECK(c == 0x4B37u, "CRC 标准校验向量");
    CHECK(crc16_tabled(vec, 9) == c, "逐位/查表 两种实现一致");

    uint8_t buf[64];
    int mismatch = 0;
    for (int n = 0; n < 64; n++) {
        for (int i = 0; i < n; i++) buf[i] = (uint8_t)(i * 37 + n * 11 + 5);
        if (Frame_CRC16(buf, (uint16_t)n) != crc16_tabled(buf, n)) mismatch = 1;
    }
    CHECK(mismatch == 0, "长度 0..63 两种实现全部一致");
}

static void test_roundtrip(void)
{
    printf("[2] 组帧/解帧 往返 + 协议命令实际字节\n");

    struct { uint16_t cmd; const char *name; uint8_t data[16]; uint16_t len; } cases[] = {
        { MGR_CMD_FW_START,        "0x0009 固件起始帧", {0x00,0x02, 0x00,0x00,0x10,0x00, 0x01,0x02,0x03}, 9 },
        { MGR_CMD_FW_DATA,         "0x000A 固件数据帧", {0xDE,0xAD,0xBE,0xEF},                            4 },
        { MGR_CMD_FW_END,          "0x000B 固件结束帧", {0},                                              0 },
        { MGR_CMD_QUERY_FLASH_VER, "0x000C 查Flash版本", {0},                                             0 },
        { MGR_CMD_QUERY_FOC_VER,   "0x000D 查FOC板版本", {0},                                             0 },
        { MGR_CMD_PROGRAM,         "0x0112 烧录FOC板",  {0},                                              0 },
    };

    for (unsigned k = 0; k < sizeof(cases)/sizeof(cases[0]); k++) {
        uint8_t  raw[64];
        uint16_t seq   = 0x1234;
        uint16_t total = Frame_Build(raw, cases[k].cmd, seq, cases[k].data, cases[k].len);

        CHECK(total == (uint16_t)(10u + cases[k].len), "整帧长度 = 10 + LEN");
        CHECK(raw[0] == 0xA5 && raw[1] == 0x5B, "帧头 A5 5B");
        CHECK(raw[2] == (uint8_t)(cases[k].cmd >> 8) && raw[3] == (uint8_t)cases[k].cmd, "CMD 大端");
        CHECK(raw[4] == 0x12 && raw[5] == 0x34, "SEQ 大端");

        uint16_t cmd = 0, sq = 0, dl = 0;
        uint8_t  payload[256];
        CHECK(Frame_Parse(raw, total, &cmd, &sq, payload, &dl) == 1, "解帧成功");
        CHECK(cmd == cases[k].cmd && sq == seq && dl == cases[k].len, "CMD/SEQ/LEN 一致");
        CHECK(dl == 0 || memcmp(payload, cases[k].data, dl) == 0, "DATA 一致");

        char tag[48];
        snprintf(tag, sizeof(tag), "%s:", cases[k].name);
        dump(tag, raw, total);
    }

    {
        uint8_t  raw[16];
        uint8_t  ok = APP_STATUS_OK;
        uint16_t n  = Frame_Build(raw, MGR_CMD_FW_START, 0x0001, &ok, 1);
        dump("ACK 0x0009 成功:", raw, n);

        n = Frame_Build(raw, MGR_CMD_QUERY_FLASH_VER, 0x0001,
                        (const uint8_t *)"\x00\x01\x00\x00", 4);
        dump("ACK 0x000C v1.0.0:", raw, n);

        n = Frame_Build(raw, MGR_CMD_QUERY_FOC_VER, 0x0005,
                        (const uint8_t *)"\x0C\x00\xFF\xFF", 4);
        dump("ACK 0x000D 失败:", raw, n);
    }
}

static void test_reject(void)
{
    printf("[3] 非法帧拒绝\n");

    uint8_t  raw[32];
    uint16_t n = Frame_Build(raw, MGR_CMD_FW_END, 1, 0, 0);

    uint8_t bad[32];
    memcpy(bad, raw, n);
    bad[n - 1] ^= 0xFF;
    CHECK(Frame_Parse(bad, n, 0, 0, 0, 0) == 0, "CRC 错误被拒");
    CHECK(Frame_Parse(raw, (uint16_t)(n - 1), 0, 0, 0, 0) == 0, "长度不足被拒");
    CHECK(Frame_Parse(raw, (uint16_t)(n + 1), 0, 0, 0, 0) == 0, "长度多余被拒");

    bad[0] = 0xA4;
    CHECK(Frame_Parse(bad, n, 0, 0, 0, 0) == 0, "帧头错误被拒");
}

static void test_stream(void)
{
    uint16_t len;
    uint8_t *raw;

    printf("[4] 字节流状态机\n");

    uint8_t  f1[32], f2[32];
    uint8_t  p1[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint16_t n1 = Frame_Build(f1, MGR_CMD_FW_DATA, 0x0001, p1, 4);
    uint16_t n2 = Frame_Build(f2, MGR_CMD_PROGRAM, 0x0002, 0, 0);

    /* 4.1 逐字节喂一帧 */
    APP_Frame_Init();
    feed(f1, n1);
    raw = recv_frame(&len);
    CHECK(raw != NULL, "4.1 单帧接收完成");
    CHECK(raw && len == n1 && memcmp(raw, f1, n1) == 0, "4.1 帧长度/内容正确");
    raw = recv_frame(&len);
    CHECK(raw == NULL, "4.1 取走后无残留帧");

    /* 4.2 粘连两帧: 必须能连续取出两帧, 第二帧不被挤坏/越界 */
    APP_Frame_Init();
    feed(f1, n1);
    feed(f2, n2);
    raw = recv_frame(&len);
    CHECK(raw && len == n1 && memcmp(raw, f1, n1) == 0, "4.2 粘连第 1 帧正确");
    raw = recv_frame(&len);
    CHECK(raw && len == n2 && memcmp(raw, f2, n2) == 0, "4.2 粘连第 2 帧正确");
    raw = recv_frame(&len);
    CHECK(raw == NULL, "4.2 两帧取完无残留");

    /* 4.3 前置垃圾(含伪帧头与半截帧头)后必须重同步 */
    APP_Frame_Init();
    {
        uint8_t junk[7] = {0x00, 0xA5, 0x11, 0x22, 0xA5, 0x5B, 0x99};
        feed(junk, 7);
        feed(f1, n1);
    }
    raw = recv_frame(&len);
    CHECK(raw && len == n1 && memcmp(raw, f1, n1) == 0, "4.3 垃圾后重同步正确");

    /* 4.4 超长 LEN(>256) 必须被丢弃, 随后仍能重同步 */
    APP_Frame_Init();
    {
        uint8_t evil[8] = {0xA5, 0x5B, 0x00, 0x0A, 0x00, 0x01, 0xFF, 0xFF};
        feed(evil, 8);
        feed(f2, n2);
    }
    raw = recv_frame(&len);
    CHECK(raw && len == n2 && memcmp(raw, f2, n2) == 0, "4.4 超长帧后重同步正确");

    /* 4.5 最大合法数据帧 LEN=256 不越界 */
    APP_Frame_Init();
    {
        static uint8_t big[256];
        static uint8_t fbig[8 + 256 + 2 + 4];
        for (int i = 0; i < 256; i++) big[i] = (uint8_t)i;
        uint16_t nb = Frame_Build(fbig, MGR_CMD_FW_DATA, 0x0003, big, 256);
        CHECK(nb == 266, "4.5 LEN=256 帧总长 266");
        feed(fbig, nb);
        raw = recv_frame(&len);
        CHECK(raw && len == 266 && memcmp(raw, fbig, 266) == 0, "4.5 LEN=256 帧完整");
    }

    /* 4.6 环形缓冲压力: 连续 8 帧一次性灌入, 逐帧取出必须全部正确且有序 */
    APP_Frame_Init();
    {
        static uint8_t many[8][16];
        int ok = 1;
        for (int k = 0; k < 8; k++) {
            uint8_t  pl[2] = { (uint8_t)k, (uint8_t)(k ^ 0x5A) };
            uint16_t nn = Frame_Build(many[k], MGR_CMD_FW_DATA, (uint16_t)(0x10 + k), pl, 2);
            feed(many[k], nn);
        }
        for (int k = 0; k < 8; k++) {
            raw = recv_frame(&len);
            if (!raw || len != 12u || memcmp(raw, many[k], 12u) != 0) ok = 0;
        }
        CHECK(ok == 1, "4.6 连续 8 帧全部正确且有序");
        raw = recv_frame(&len);
        CHECK(raw == NULL, "4.6 无多余帧");
    }
}

int main(void)
{
    crc_tbl_init();
    printf("=== FOC JIG U1 协议层宿主单元测试 ===\n\n");
    test_crc();
    test_roundtrip();
    test_reject();
    test_stream();
    printf("\n=== 共 %d 项检查, 失败 %d 项 ===\n", g_checks, g_fails);
    return g_fails == 0 ? 0 : 1;
}
