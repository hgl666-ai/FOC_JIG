/*
 * DUT 通讯协议打包/解包层宿主单测 (dut_proto.c)
 *
 * 被测的是真源码：所有打包/解包/校验函数走真实路径。
 * 期望值有两类来源：
 *   1) docs/15_DUT通讯协议规格.md 里逐字节写死的布局（**规范是权威**）；
 *   2) 规范 §12 的示例帧 —— 本测试直接把示例帧的 DATA 段喂进来，
 *      验证"文档里写的帧"和"代码打出来的帧"完全一致（防止文档与代码漂移）。
 *
 * 纯函数，不需要任何 HAL 替身。
 */
#include <stdio.h>
#include <string.h>

#include "dut_proto.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *name)
{
    if (cond) {
        g_pass++;
        printf("  [ok]   %s\n", name);
    } else {
        g_fail++;
        printf("  [FAIL] %s\n", name);
    }
}

/* 比较字节数组 */
static void check_bytes(const uint8_t *got, const uint8_t *want, int n, const char *name)
{
    int i;
    for (i = 0; i < n; i++) {
        if (got[i] != want[i]) {
            g_fail++;
            printf("  [FAIL] %s (第 %d 字节: got %02X want %02X)\n", name, i, got[i], want[i]);
            return;
        }
    }
    g_pass++;
    printf("  [ok]   %s\n", name);
}

/* 把 "A5 5B ..." 这种十六进制串解析成字节；返回字节数 */
static int hex2bin(const char *s, uint8_t *out, int max)
{
    int n = 0;
    unsigned v;
    while (*s && n < max) {
        while (*s == ' ') {
            s++;
        }
        if (!*s) {
            break;
        }
        if (sscanf(s, "%2x", &v) != 1) {
            break;
        }
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}

int main(void)
{
    uint8_t  buf[DUT_LINK_MAX_PAYLOAD + 8];
    uint8_t  want[64];
    uint16_t n;
    int      len;

    printf("===== dut_proto 单测 =====\n");

    /* ---------------- 1. 长度常量 ---------------- */
    printf("\n-- 1. 长度常量（规范 §2/§7）--\n");
    check(DUT_INFO_RESP_LEN   == 16u, "INFO 应答 = 16B");
    check(DUT_TESTMODE_REQ_LEN == 3u, "TEST_MODE 请求 = 3B");
    check(DUT_PWM_REQ_LEN     == 10u, "PWM 请求 = 10B");
    check(DUT_LEVEL_REQ_LEN    == 2u, "LEVEL 请求 = 2B");
    check(DUT_CURRENT_REQ_LEN  == 8u, "CURRENT 请求 = 8B");
    check(DUT_GETADC_REQ_LEN   == 2u, "GET_ADC 请求 = 2B（掩码 9 路必须 2 字节）");
    check(DUT_GETENC_RESP_LEN == 14u, "GET_ENC 应答 = 14B");
    check((1u + DUT_GETADC_MAX_CH * DUT_GETADC_ITEM_LEN) <= DUT_LINK_MAX_PAYLOAD,
          "GET_ADC 满通道应答装得进 64B 单帧");

    /* ---------------- 2. TEST_MODE ---------------- */
    printf("\n-- 2. TEST_MODE 打包 --\n");
    n = DutPack_TestMode(buf, sizeof(buf), DUT_TESTMODE_ENTER, 1000u);
    check(n == 3u, "返回长度 3");
    want[0] = 0x01; want[1] = 0x03; want[2] = 0xE8;
    check_bytes(buf, want, 3, "进入 + 1000ms = 01 03 E8（大端）");
    n = DutPack_TestMode(buf, 2u, DUT_TESTMODE_ENTER, 1000u);
    check(n == 0u, "缓冲不够 → 返回 0");

    printf("\n-- 2b. TEST_MODE 参数校验（规范 §7.3 / §9.2）--\n");
    check(DutCheck_TestMode(DUT_TESTMODE_ENTER, 200u)  == 1u, "进入 + 最小 200ms 合法");
    check(DutCheck_TestMode(DUT_TESTMODE_ENTER, 10000u) == 1u, "进入 + 最大 10000ms 合法");
    check(DutCheck_TestMode(DUT_TESTMODE_ENTER, 199u)  == 0u, "进入 + 199ms 非法");
    check(DutCheck_TestMode(DUT_TESTMODE_ENTER, 10001u) == 0u, "进入 + 10001ms 非法");
    check(DutCheck_TestMode(DUT_TESTMODE_ENTER, 0u)    == 0u, "进入 + 0ms 非法（不许关掉看门狗）");
    check(DutCheck_TestMode(DUT_TESTMODE_EXIT, 0u)     == 1u, "退出时不带看门狗合法");
    check(DutCheck_TestMode(2u, 1000u)                 == 0u, "mode=2 非法");

    /* ---------------- 3. PWM ---------------- */
    printf("\n-- 3. PWM 打包（规范 §7.5）--\n");
    {
        dut_pwm_t p;
        p.freq_hz = 1000u; p.duty_a = 500u; p.duty_b = 500u; p.duty_c = 500u; p.count = 300u;
        n = DutPack_Pwm(buf, sizeof(buf), &p);
        check(n == 10u, "返回长度 10");
        want[0]=0x03; want[1]=0xE8; want[2]=0x01; want[3]=0xF4;
        want[4]=0x01; want[5]=0xF4; want[6]=0x01; want[7]=0xF4;
        want[8]=0x01; want[9]=0x2C;
        check_bytes(buf, want, 10, "1kHz / 50%×3 / 300 个 = 03E8 01F4 01F4 01F4 012C");
        check(DutPack_Pwm(buf, sizeof(buf), 0) == 0u, "空指针 → 0");
    }
    printf("\n-- 3b. PWM 参数校验 --\n");
    {
        dut_pwm_t p;
        p.freq_hz = 1000u; p.duty_a = 500u; p.duty_b = 500u; p.duty_c = 500u; p.count = 300u;
        check(DutCheck_Pwm(&p, 100u, 20000u) == 1u, "范围内合法");
        check(DutCheck_Pwm(&p, 0u, 0u)       == 1u, "不检查范围时合法");
        check(DutCheck_Pwm(&p, 2000u, 20000u) == 0u, "低于下限 → 非法");
        p.freq_hz = 30000u;
        check(DutCheck_Pwm(&p, 100u, 20000u) == 0u, "高于上限 → 非法");
        p.freq_hz = 1000u; p.duty_a = 1001u;
        check(DutCheck_Pwm(&p, 0u, 0u) == 0u, "占空比 1001 → 非法");
        p.duty_a = 1000u;
        check(DutCheck_Pwm(&p, 0u, 0u) == 1u, "占空比 1000（100%）合法");
        p.count = 0u;
        check(DutCheck_Pwm(&p, 0u, 0u) == 1u, "count=0（连续输出）合法");
        p.freq_hz = 0u;
        check(DutCheck_Pwm(&p, 0u, 0u) == 0u, "频率 0 → 非法");
    }

    /* ---------------- 4. LEVEL ---------------- */
    printf("\n-- 4. LEVEL（规范 §7.6）--\n");
    n = DutPack_Level(buf, sizeof(buf), 0x21u);
    check(n == 2u, "返回长度 2");
    want[0] = 0x21; want[1] = 0x00;
    check_bytes(buf, want, 2, "位图 0x21 + rsvd 0");
    check(DutLevel_Pack(DUT_LVL_HIGH, DUT_LVL_LOW, DUT_LVL_HIZ) == 0x21u,
          "A=高 B=低 C=高阻 → 0x21");
    check(DutLevel_Pack(DUT_LVL_LOW, DUT_LVL_LOW, DUT_LVL_LOW) == 0x00u, "全低 → 0x00");
    check(DutLevel_Pack(DUT_LVL_HIGH, DUT_LVL_HIGH, DUT_LVL_HIGH) == 0x15u, "全高 → 0x15");
    check(DutLevel_Pack(DUT_LVL_HIZ, DUT_LVL_HIZ, DUT_LVL_HIZ) == 0x2Au, "全高阻 → 0x2A");
    check(DutLevel_Pack(4u, 0u, 0u) == 0xFFu, "非法电平值 → 0xFF");
    check(DutLevel_Get(0x21u, DUT_PHASE_A) == DUT_LVL_HIGH, "取 A 相 = 高");
    check(DutLevel_Get(0x21u, DUT_PHASE_B) == DUT_LVL_LOW,  "取 B 相 = 低");
    check(DutLevel_Get(0x21u, DUT_PHASE_C) == DUT_LVL_HIZ,  "取 C 相 = 高阻");
    check(DutLevel_Get(0x21u, 0x08u) == 0xFFu, "非法相 → 0xFF");
    check(DutCheck_Level(0x21u) == 1u, "0x21 合法");
    check(DutCheck_Level(0x00u) == 1u, "全低合法");
    check(DutCheck_Level(0x15u) == 1u, "全高合法（互锁由 DUT 侧按母线风险再判）");
    check(DutCheck_Level(0x3Fu) == 0u, "某相取值 11 → 非法");
    check(DutCheck_Level(0x40u) == 0u, "保留位 bit6 非 0 → 非法");
    check(DutCheck_Level(0x80u) == 0u, "保留位 bit7 非 0 → 非法");

    /* ---------------- 5. CURRENT ---------------- */
    printf("\n-- 5. CURRENT（规范 §7.7）--\n");
    {
        dut_current_t c;
        c.mode = DUT_CUR_MODE_STATIC; c.hi_mask = DUT_PHASE_A; c.lo_mask = DUT_PHASE_B;
        c.duty_permille = 1000u; c.duration_ms = 200u;
        n = DutPack_Current(buf, sizeof(buf), &c);
        check(n == 8u, "返回长度 8");
        want[0]=0x00; want[1]=0x01; want[2]=0x02; want[3]=0x03; want[4]=0xE8;
        want[5]=0x00; want[6]=0xC8; want[7]=0x00;
        check_bytes(buf, want, 8, "静态通流 A→B 100% 200ms = 00 01 02 03E8 00C8 00");
        check(DutCheck_Current(&c) == 1u, "A→B 合法");

        c.hi_mask = DUT_PHASE_A; c.lo_mask = DUT_PHASE_A;
        check(DutCheck_Current(&c) == 0u, "同相（同桥臂直通）→ 非法");
        c.lo_mask = DUT_PHASE_B; c.hi_mask = 0x03u;
        check(DutCheck_Current(&c) == 0u, "上桥选了两相 → 非法");
        c.hi_mask = DUT_PHASE_A; c.lo_mask = 0x08u;
        check(DutCheck_Current(&c) == 0u, "掩码超出 A/B/C → 非法");
        c.lo_mask = DUT_PHASE_B; c.mode = 2u;
        check(DutCheck_Current(&c) == 0u, "mode=2 → 非法");
        c.mode = DUT_CUR_MODE_PWM;
        check(DutCheck_Current(&c) == 1u, "mode=1 PWM 通流合法");
        c.duty_permille = 1001u;
        check(DutCheck_Current(&c) == 0u, "占空比 1001 → 非法");
        c.duty_permille = 500u; c.duration_ms = 0u;
        check(DutCheck_Current(&c) == 1u, "duration=0（保持到 STOP）合法");
        check(DutCheck_Current(0) == 0u, "空指针 → 非法");
    }

    /* ---------------- 6. GET_ADC ---------------- */
    printf("\n-- 6. GET_ADC（规范 §7.8）--\n");
    n = DutPack_GetAdc(buf, sizeof(buf), (uint16_t)((1u << DUT_ADC_CUR1) | (1u << DUT_ADC_CUR2)));
    check(n == 2u, "返回长度 2");
    want[0] = 0x01; want[1] = 0x80;      /* bit7|bit8 = 0x0180 */
    check_bytes(buf, want, 2, "要 ch7+ch8 = 01 80（第 8 路要得到，这是改 2 字节的原因）");

    {
        /* 规范 §12 的 GET_ADC 应答示例帧 DATA：00 | 00 081A 012C | 01 0830 0138 */
        uint8_t resp[11] = {0x00, 0x00,0x08,0x1A,0x01,0x2C, 0x01,0x08,0x30,0x01,0x38};
        dut_adc_t a;
        check(DutUnpack_Adc(resp, 11u, 0x0003u, &a) == 1u, "解示例帧 OK");
        check(a.status == DUT_ST_OK, "状态 = 00");
        check(a.n == 2u, "收到 2 路");
        check(a.item[0].ch == 0u && a.item[0].raw == 0x081Au && a.item[0].mv == 300u,
              "ch0 raw=0x081A mv=300");
        check(a.item[1].ch == 1u && a.item[1].raw == 0x0830u && a.item[1].mv == 312u,
              "ch1 raw=0x0830 mv=312");
        check(DutUnpack_Adc(resp, 11u, 0x0007u, &a) == 0u, "少给一路（要 3 给 2）→ 失败");
        check(DutUnpack_Adc(resp, 10u, 0x0003u, &a) == 0u, "长度不是 1+5n → 失败");
        check(DutUnpack_Adc(resp, 1u,  0x0000u, &a) == 1u, "只要状态、0 路 → OK");
        check(DutUnpack_Adc(0, 11u, 0x0003u, &a) == 0u, "空指针 → 失败");
    }
    check(DutMask_Count(0x0000u) == 0u, "Mask_Count(0) = 0");
    check(DutMask_Count(0x0003u) == 2u, "Mask_Count(0x0003) = 2");
    check(DutMask_Count(0x01FFu) == 9u, "Mask_Count(9 路全要) = 9");

    /* ---------------- 7. INFO ---------------- */
    printf("\n-- 7. INFO（规范 §7.2）--\n");
    {
        /* 规范 §12 的 INFO 应答示例帧 */
        uint8_t resp[16] = {0x00, 0x01,0x01, 0x01,0x00,0x00, 0x03,0xFF,
                            0x00,0x64, 0x4E,0x20, 0x00,0x03, 0x00,0x00};
        dut_info_t f;
        check(DutUnpack_Info(resp, 16u, &f) == 1u, "解示例帧 OK");
        check(f.status == DUT_ST_OK, "状态 = 00");
        check(f.dut_ver == DUT_CODE_VER, "dut_ver = 0x0101（与本文件常量一致）");
        check(f.fw_ver[0]==1u && f.fw_ver[1]==0u && f.fw_ver[2]==0u, "固件版本 1.0.0");
        check(f.caps == 0x03FFu, "caps = 0x03FF");
        check(f.pwm_min_hz == 100u && f.pwm_max_hz == 20000u, "PWM 范围 100~20000 Hz");
        check(f.adc_mask == 0x0003u, "adc_mask = ch0/ch1");
        check(DutUnpack_Info(resp, 15u, &f) == 0u, "长度不足 → 失败");
    }

    /* ---------------- 8. GET_ENC ---------------- */
    printf("\n-- 8. GET_ENC（规范 §7.9）--\n");
    {
        uint8_t resp[14] = {0x00,
                            0x00,0x00,0x03,0xE8,     /* A = 1000 */
                            0x00,0x00,0x07,0xD0,     /* B = 2000 */
                            0x00,0x00,0x00,0x01,     /* Z = 1    */
                            0x01};
        dut_enc_t e;
        check(DutUnpack_Enc(resp, 14u, &e) == 1u, "解示例帧 OK");
        check(e.cnt_a == 1000u, "A = 1000");
        check(e.cnt_b == 2000u, "B = 2000");
        check(e.cnt_z == 1u,    "Z = 1");
        check(e.dir == 1,       "dir = +1");
        e.dir = 0;
        resp[13] = 0xFF;
        check(DutUnpack_Enc(resp, 14u, &e) == 1u && e.dir == -1, "dir = 0xFF 解析为 -1");
        check(DutUnpack_Enc(resp, 13u, &e) == 0u, "长度不足 → 失败");
        /* u32 大端：0x12345678 逐字节 */
        resp[1]=0x12; resp[2]=0x34; resp[3]=0x56; resp[4]=0x78;
        check(DutUnpack_Enc(resp, 14u, &e) == 1u && e.cnt_a == 0x12345678u, "A = 0x12345678（大端）");
    }

    /* ---------------- 9. 状态应答 ---------------- */
    printf("\n-- 9. 状态应答 --\n");
    {
        uint8_t st = 0xAAu;
        uint8_t one = 0x00u;
        check(DutUnpack_Status(&one, 1u, &st) == 1u && st == DUT_ST_OK, "解 00 OK");
        one = DUT_ST_NIY;
        check(DutUnpack_Status(&one, 1u, &st) == 1u && st == DUT_ST_NIY, "解 FF NIY");
        check(DutUnpack_Status(&one, 0u, &st) == 0u, "长度 0 → 失败");
        check(DutUnpack_Status(0, 1u, &st) == 0u, "空指针 → 失败");
    }

    /* ---------------- 10. PING ---------------- */
    printf("\n-- 10. PING 回显 --\n");
    {
        uint8_t pay[4] = {0xDE, 0xAD, 0xBE, 0xEF};
        n = DutPack_Ping(buf, sizeof(buf), pay, 4u);
        check(n == 4u, "返回长度 4");
        check_bytes(buf, pay, 4, "原样拷贝 DE AD BE EF");
        check(DutPack_Ping(buf, sizeof(buf), pay, 0u) == 0u, "长度 0 返回 0（空 DATA）");
        check(DutPack_Ping(buf, 3u, pay, 4u) == 0u, "缓冲不够 → 0");
        check(DutPack_Ping(buf, sizeof(buf), 0, 4u) == 0u, "空指针 → 0");
    }

    /* ---------------- 11. caps ---------------- */
    printf("\n-- 11. 能力位图（规范 §8）--\n");
    check(DutCaps_Has(DUT_CAP_ALL, DUT_CMD_PING)     == 1u, "caps 含 PING");
    check(DutCaps_Has(DUT_CAP_ALL, DUT_CMD_GET_ENC)  == 1u, "caps 含 GET_ENC");
    check(DutCaps_Has(DUT_CAP_ALL, 0x0209u)          == 0u,
          "caps 全集不含 0x0209（它只覆盖已定义的 9 条 0x0200..0x0208）");
    check(DutCaps_Has(0xFFFFu, 0x020Fu)              == 1u, "全 1 caps 含 0x020F（bit15，能声明的最后一条）");
    check(DutCaps_Has(0xFFFFu, 0x0210u)              == 0u, "0x0210 超出 16 位 caps → 不算支持");
    check(DutCaps_Has(0xFFFFu, DUT_CMD_HI)           == 0u, "0x02FF 超出 16 位 caps → 不算支持");
    check(DutCaps_Has(0u, DUT_CMD_PWM)               == 0u, "caps=0 不含 PWM");
    check(DutCaps_Has(DUT_CAP_PWM, DUT_CMD_LEVEL)    == 0u, "只有 PWM 位时不含 LEVEL");
    check(DutCaps_Has(DUT_CAP_ALL, 0x0100u)          == 0u, "命令码超出范围 → 0");
    check(DutCaps_Has(DUT_CAP_ALL, 0x0300u)          == 0u, "命令码超出范围（高）→ 0");
    check((DUT_CAP_MANDATORY & ~DUT_CAP_ALL) == 0u, "必做项都在 caps 全集中");
    check((DUT_CAP_GET_ENC >> (DUT_CMD_GET_ENC - DUT_CMD_LO)) == 1u,
          "GET_ENC 的位号 = 命令码偏移（caps 与命令码对得上）");

    /* ---------------- 12. 用规范 §12 的示例帧端到端核对 ---------------- */
    printf("\n-- 12. 用 docs/15 §12 的示例帧核对 DATA 段（防文档与代码漂移）--\n");
    {
        /* PWM 示例帧 DATA */
        dut_pwm_t p;
        uint8_t   doc[64];
        p.freq_hz = 1000u; p.duty_a = 500u; p.duty_b = 500u; p.duty_c = 500u; p.count = 300u;
        len = hex2bin("03 E8 01 F4 01 F4 01 F4 01 2C", doc, (int)sizeof(doc));
        n = DutPack_Pwm(buf, sizeof(buf), &p);
        check(len == (int)n, "PWM：文档 DATA 长度 = 代码打出长度");
        check_bytes(buf, doc, len, "PWM：文档 DATA 与代码逐字节一致");

        /* CURRENT 示例帧 DATA */
        {
            dut_current_t c;
            c.mode = DUT_CUR_MODE_STATIC; c.hi_mask = DUT_PHASE_A; c.lo_mask = DUT_PHASE_B;
            c.duty_permille = 1000u; c.duration_ms = 200u;
            len = hex2bin("00 01 02 03 E8 00 C8 00", doc, (int)sizeof(doc));
            n = DutPack_Current(buf, sizeof(buf), &c);
            check(len == (int)n, "CURRENT：文档 DATA 长度 = 代码打出长度");
            check_bytes(buf, doc, len, "CURRENT：文档 DATA 与代码逐字节一致");
        }
        /* GET_ADC 示例帧 DATA */
        len = hex2bin("00 03", doc, (int)sizeof(doc));
        n = DutPack_GetAdc(buf, sizeof(buf), 0x0003u);
        check(len == (int)n, "GET_ADC：文档 DATA 长度 = 代码打出长度");
        check_bytes(buf, doc, len, "GET_ADC：文档 DATA 与代码逐字节一致");
        /* GET_ENC 示例帧 DATA */
        len = hex2bin("00 00 00 03 E8 00 00 07 D0 00 00 00 01 01", doc, (int)sizeof(doc));
        check(len == (int)DUT_GETENC_RESP_LEN, "GET_ENC：文档 DATA 长度 = 常量");
        /* TEST_MODE 示例帧 DATA */
        len = hex2bin("01 03 E8", doc, (int)sizeof(doc));
        n = DutPack_TestMode(buf, sizeof(buf), DUT_TESTMODE_ENTER, 1000u);
        check_bytes(buf, doc, len, "TEST_MODE：文档 DATA 与代码逐字节一致");
        check((int)n == len, "TEST_MODE：长度一致");
        /* LEVEL 示例帧 DATA */
        len = hex2bin("21 00", doc, (int)sizeof(doc));
        n = DutPack_Level(buf, sizeof(buf), DutLevel_Pack(DUT_LVL_HIGH, DUT_LVL_LOW, DUT_LVL_HIZ));
        check((int)n == len, "LEVEL：长度一致");
        check_bytes(buf, doc, len, "LEVEL：文档 DATA 与代码逐字节一致");
    }

    printf("\n===== dut_proto: %d 项通过, %d 项失败 =====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
