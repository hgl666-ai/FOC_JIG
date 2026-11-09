#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
foc_jig_tool.py 的**应答解码**离线自检（不需要硬件、不需要串口）

为什么单独有这一份：主机回归里跑的 `foc_jig_tool.py selftest` 只验"帧的组装/解析"，
而 0x0014~0x001B 的应答全是**定长多字段**布局，偏移量写错一个字节就会把
"A 相的占空比"念成"B 相的期望值"，而且**没有任何报错** —— 只有到台架上看
数字不对劲才发现。所以这里按 flash_map.h 的布局**手工造一份合法应答**，
把解码器跑一遍，断言它念出来的关键数字对得上。

用法：python test/tool_decode_test.py     （返回 0 = 全过）
"""
import os
import sys
import io
import contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))

import foc_jig_tool as T          # noqa: E402


def be16(v):
    return bytes([(v >> 8) & 0xFF, v & 0xFF])


def be32(v):
    return bytes([(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


class FakeLink(object):
    """只实现 request()：按命令号返回预先造好的应答 DATA"""

    def __init__(self, table):
        self.table = table
        self.seen = []

    def request(self, cmd, data=b"", timeout=3.0, seq=None):
        self.seen.append((cmd, bytes(data)))
        if cmd not in self.table:
            return None
        return (1, self.table[cmd])


g_checks = 0
g_fails = 0


def check(cond, msg):
    global g_checks, g_fails
    g_checks += 1
    if not cond:
        g_fails += 1
        print("  FAIL: %s" % msg)


def run(fn, *a, **kw):
    """跑一个解码器并把它的打印吃掉，返回文本"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        fn(*a, **kw)
    return buf.getvalue()


# ─────────────────────────── 造应答 ───────────────────────────
def mk_vol_v2():
    """0x0015 应答 47B：状态 + 被测板状态 + 9×(治具mV, 被测板mV, 旗标)"""
    d = bytes([0x00, 0x00])
    vals = [(24000, 24100), (10000, 10050), (5000, 4980), (3300, 3310),
            (0, 0), (0, 0), (0, 0), (1650, 1650), (1655, 1655)]
    for i, (jm, dm) in enumerate(vals):
        f = 0x01 if i < 4 else 0x02          # 前 4 路判了，后 5 路"不判"
        d += be16(jm) + be16(dm) + bytes([f])
    assert len(d) == 47, len(d)
    return d


def mk_pulse():
    """0x0019 应答 23B：状态 + 对侧结论 + 3×(周期ns4, 占空比‰2, 旗标1)"""
    d = bytes([0x00, 0x00])                   # 状态 00 / 对侧 0（没有对侧数据）
    for _ in range(3):
        d += be32(1000000) + be16(500) + bytes([0x01])   # 1kHz / 50% / 有信号
    assert len(d) == 23, len(d)
    return d


def mk_phase():
    """0x001A 应答 16B：状态 + 3×(实测mV, 期望mV, 旗标)"""
    d = bytes([0x00])
    for i in range(3):
        d += be16(12000 + 120 - i * 60) + be16(12000) + bytes([0x01])
    assert len(d) == 16, len(d)
    return d


def mk_enc():
    """0x001B 应答 12B：状态 + 方向 + 对侧结论 + 3×(边沿数2, 旗标1)"""
    d = bytes([0x00, 0x01, 0x01])             # 状态 00 / 方向 +1 / 对侧一致
    for dut in (2000, 3998, 2):
        d += be16(dut) + bytes([0x01])
    assert len(d) == 12, len(d)
    return d


def mk_cur():
    """0x0014 应答 12B：状态 + 被测板状态 + 2×(治具变化量s16, 被测板变化量s16, 旗标1)"""
    d = bytes([0x00, 0x00])
    for dj, dd in ((62, 60), (63, 62)):
        d += be16(dj & 0xFFFF) + be16(dd & 0xFFFF) + bytes([0x01])
    assert len(d) == 12, len(d)
    return d


def mk_status(dut_link=True):
    """0x0017 应答 32B"""
    fl = 0x08 if dut_link else 0x00
    d = bytes([0x00, 0x01, fl])
    d += be16(0x0101) + be16(0x003F) + be16(0)          # 3..8
    d += be16(7) + be16(0)                              # 9..12 探测成功/失败
    d += be32(1234) + be32(0) + be16(0) + be16(0)       # 13..24
    d += be16(0) + be16(0) + be16(0) + bytes([0])       # 25..31
    assert len(d) == 32, len(d)
    return d


def mk_dutsend(sent=7, body=b""):
    """0x001F 应答：状态(1)+发出字节数(2)+收到字节数(2)+字节"""
    return bytes([0x00]) + be16(sent) + be16(len(body)) + body


def mk_dutrecv(body=b""):
    """0x0020 应答：状态(1)+字节数(2)+字节"""
    return bytes([0x00]) + be16(len(body)) + body


def mk_runall():
    """0x0010 应答 29B：状态 1 + 步数 1 + 跑到第几步 1 + 首个失败 1 + 12 × 2"""
    d = bytes([0x00, 12, 12, 0])
    for i in range(12):
        d += bytes([1, 0x00])
    d += b"\x00"                        # MGR_RUN_ALL_RESP_LEN = 4 + 12×2 + 1 的保留字节
    assert len(d) == 29, len(d)
    return d


# ─────────────────────────── 主流程 ───────────────────────────
def main():
    print("=== foc_jig_tool 应答解码自检（不需要硬件）===\n")

    # ① 0x0015 电压
    link = FakeLink({T.CMD_VOL_TEST: mk_vol_v2()})
    out = run(T.do_vol, link)
    check("24.000 V" in out, "电压：ch0 治具读数 24.000 V")
    check("24.100 V" in out, "电压：ch0 被测板读数 24.100 V")
    check("3.300 V" in out, "电压：ch3 = 3.300 V")
    check("不判(数据缺)" in out, "电压：ch7/ch8 显示“不判”")
    check("差值那一列是工具算的" in out, "电压：说明差值由工具算（固件不再发）")
    check(link.seen[0][0] == T.CMD_VOL_TEST and link.seen[0][1] == b"",
          "电压：请求是 LEN=0 的 0x0015")

    # ② 0x0019 脉冲
    link = FakeLink({T.CMD_PULSE_TEST: mk_pulse()})
    out = run(T.do_pulse, link)
    check("1.0 kHz" in out or "1000.0 Hz" in out, "脉冲：周期 1e6 ns 念成 1 kHz")
    check("50.0%" in out, "脉冲：占空比 50.0%（500‰）")
    check(out.count("对侧") >= 1, "脉冲：对侧那块也解了")

    # ③ 0x001A 三相
    link = FakeLink({T.CMD_PHASE_TEST: mk_phase()})
    out = run(T.do_phase, link)
    check("12.000 V" in out, "三相：期望值 12.000 V")
    check("+120" in out and "+60" in out, "三相：实测与期望的差 120/60（工具算的）")
    check("12.120 V" in out, "三相：实测值 12.120 V")

    # ④ 0x001B 编码器
    link = FakeLink({T.CMD_ENC_TEST: mk_enc()})
    out = run(T.do_enc, link)
    check("2000" in out and "3998" in out, "编码器：期望 2000/4000，被测板 2000/3998")
    check("+1" in out, "编码器：方向 +1")

    # ⑤ 0x0014 相电流
    link = FakeLink({T.CMD_CUR_TEST: mk_cur()})
    out = run(T.do_cur, link, hi=0x01, lo=0x02, duty=20, ms=100, mode=1)
    check("+62 mV" in out, "电流：治具变化量 +62 mV")
    check("+60 mV" in out, "电流：被测板变化量 +60 mV")
    sent = link.seen[0][1]
    check(len(sent) == 9, "电流：请求 9 字节（含 mode）")
    check(sent[0] == 0x01 and sent[1] == 0x02, "电流：hi=A lo=B")
    check(sent[2:4] == be16(20), "电流：占空比 20‰")
    check(sent[4:6] == be16(100), "电流：时长 100ms")
    check(sent[8] == 1, "电流：第 9 字节 mode=1（PWM）")

    # ⑥ 0x0017 自检
    link = FakeLink({T.CMD_JIG_STATUS: mk_status(True)})
    out = run(T.do_status, link)
    check("DUT链路已编入" in out, "自检：认出 DUT 链路已编入")
    check("1234" in out, "自检：收帧计数 1234")
    link2 = FakeLink({T.CMD_JIG_STATUS: mk_status(False)})
    out2 = run(T.do_status, link2)
    check("没有DUT链路" in out2, "自检：认出没编 DUT 链路")

    # ⑦ 0x0010 一键
    link = FakeLink({T.CMD_RUN_ALL: mk_runall()})
    out = run(T.do_run_all, link)
    check("12" in out and "步" in out, "一键：报出 12 步")
    check("②电压" in out and "⑥相电流" in out, "一键：步骤名对得上")

    # ⑧ 0x0011 透传 PING / INFO
    link = FakeLink({T.CMD_DUT_RAW: bytes([0x00]) + bytes.fromhex("DEADBEEF")})
    out = run(T.do_dut_ping, link)
    check("完全一致" in out, "PING：回显一致判 PASS")
    check(link.seen[0][1] == bytes([0x02, 0x00]) + bytes.fromhex("DEADBEEF"),
          "PING：透传 DATA = 0200 + payload")

    # 0x0011 的应答 = 治具状态(1) + 被测板 INFO 的 16B DATA
    info = bytes([0x00]) + (
        bytes([0x00]) + be16(0x0101) + bytes([1, 0, 0]) + be16(0x03FF) +
        be16(100) + be16(20000) + be16(0x01FF) + be16(0))
    assert len(info) == 17, len(info)
    link = FakeLink({T.CMD_DUT_RAW: info})
    out = run(T.do_dut_info, link)
    check("支持：PING, INFO, TEST_MODE, STOP, PWM, LEVEL, CURRENT, GET_ADC, GET_ENC" in out
          or ("GET_ENC" in out and "不支持：(无)" in out), "INFO：caps=0x03FF 九条全支持")
    check("20000" in out, "INFO：PWM 上限 20000 Hz")
    check("电流检测2" in out, "INFO：ADC 掩码 9 路都列出")

    # ⑨ 0x001F 发原始字节：对方不回字（最常见）
    link = FakeLink({T.CMD_DUT_SEND: mk_dutsend(7, b"")})
    out = run(T.do_dut_send, link, "AS:S800", 200, "AS:S800（转）")
    sent_req = link.seen[0][1]
    check(len(sent_req) == 2 + 7, "0x001F 请求 = 2 字节时长 + 7 字节正文")
    check(sent_req[0:2] == be16(200), "0x001F 请求里的时长 = 200ms（大端）")
    check(sent_req[2:] == b"AS:S800", "0x001F 正文就是 AS:S800")
    check("实际发出 7 字节" in out and "收到 0 字节" in out, "0x001F 回报发出/收到字节数")
    check("一个字节都没回" in out, "0x001F 没回字时给“电机转了就是通了”的判读")

    # ⑩ 0x001F 对方回字
    link = FakeLink({T.CMD_DUT_SEND: mk_dutsend(5, b"OK\r\n")})
    out = run(T.do_dut_send, link, "AS:S0", 200, "AS:S0（停）")
    check("收到 4 字节" in out, "0x001F 对方回 4 字节")
    check("双向都通" in out, "0x001F 有回字时判“双向都通”")
    check("4F 4B 0D 0A" in out, "0x001F 回字按 hex 打出来")

    # ⑪ 0x0020 只读
    link = FakeLink({T.CMD_DUT_RECV: mk_dutrecv(b"HI")})
    out = run(T.do_dut_recv, link)
    check("字节数 = 2" in out and "48 49" in out, "0x0020 取出 2 字节 HI")
    check(link.seen[0][1] == b"", "0x0020 请求 LEN=0")

    # ⑫ 0x0020 空
    link = FakeLink({T.CMD_DUT_RECV: mk_dutrecv(b"")})
    out = run(T.do_dut_recv, link)
    check("字节数 = 0" in out, "0x0020 没攒下字节时也正常回")

    # ⑨ 透传没应答（治具回 0C）
    link = FakeLink({T.CMD_DUT_RAW: bytes([0x0C])})
    out = run(T.do_dut_ping, link)
    check("没有实现 0x0200" in out or "没等到被测板回帧" in out,
          "PING：0C 时给出“FOC 固件没实现”的判读")

    print("\n=== 共 %d 项，失败 %d 项 ===" % (g_checks, g_fails))
    return 0 if g_fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
