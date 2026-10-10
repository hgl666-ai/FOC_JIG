#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FOC 烧录治具 — PC 端协议工具 / 上位机参考实现

协议来源: INFO/M1 测试治具PC端通讯协议.xlsx
帧格式  : A5 5B | CMD(2B 大端) | SEQ(2B 大端) | LEN(2B 大端) | DATA(L) | CRC16(2B 大端)
CRC16   : MODBUS (poly 0xA001 反射, init 0xFFFF), 计算范围 帧头..DATA 末字节
状态码  : 00 OK / 01 BUSY / 02 ERR_SIZE / 03 ERR_FLASH / 04 ERR_PROG / 05 ERR_TEST
          06 ERR_NO_FW / 07 ERR_INCOMPLETE / 08 ERR_CRC / 09 ERR_SEQ / 0A ERR_COUNT
          0B ERR_PARAM / 0C ERR_TIMEOUT / 0D ERR_OVERFLOW / FF NIY

用法:
    python foc_jig_tool.py selftest                     # 自检(不需要硬件)
    python foc_jig_tool.py demo                         # 打印全部命令的实际字节
    python foc_jig_tool.py -p COM5 burn --boot bl.bin --app app.bin -v 1.0.0
                                                        # ★ 一键完整烧录流程
    python foc_jig_tool.py -p COM5 query-flash          # 0x000C 查治具 Flash 中固件版本
    python foc_jig_tool.py -p COM5 query-foc            # 0x000D 查被烧板运行态版本
    python foc_jig_tool.py -p COM5 upload fw.bin -v 1.0.0   # 0x0009/0A/0B 上传固件
    python foc_jig_tool.py -p COM5 program              # 0x0112 烧录到被烧板
    python foc_jig_tool.py -p COM5 raw A5 5B 00 0C 00 01 00 00 48 21
"""

import sys
import os
import time
import argparse

# ─────────────────────────── 协议常量 ───────────────────────────
HDR0, HDR1 = 0xA5, 0x5B

CMD_FW_START        = 0x0009
CMD_FW_DATA         = 0x000A
CMD_FW_END          = 0x000B
CMD_QUERY_FLASH_VER = 0x000C
CMD_QUERY_FOC_VER   = 0x000D
CMD_PROGRAM         = 0x0112

STATUS_NAME = {
    0x00: "OK",
    0x01: "BUSY",
    0x02: "ERR_SIZE",
    0x03: "ERR_FLASH",
    0x04: "ERR_PROG",
    0x05: "ERR_TEST",
    0x06: "ERR_NO_FW",
    0x07: "ERR_INCOMPLETE",
    0x08: "ERR_CRC",
    0x09: "ERR_SEQ",
    0x0A: "ERR_COUNT",
    0x0B: "ERR_PARAM",
    0x0C: "ERR_TIMEOUT",
    0x0D: "ERR_OVERFLOW",
    0xFF: "NIY",
}
status_name = lambda c: STATUS_NAME.get(c, "UNKNOWN(0x%02X)" % c)

MAX_DATA = 256          # 单帧数据上限 (与固件 FRAME_MAX_DATA 一致)
ACK_TIMEOUT   = 3.0     # 普通 ACK 超时 (s)
UPLOAD_TIMEOUT = 10.0   # 上传期间 ACK 超时 (s), 固件端 10s 无数据即超时
PROGRAM_TIMEOUT = 180.0 # 0x0112 烧录 ACK 超时 (s), 协议规定烧完才回 ACK
# 0x0009 起始帧会触发**扇区擦除**: 治具按声明大小擦掉覆盖的 4KB 扇区,
# 最大镜像 65528B → 最多 16 个扇区, GD25Q128 单扇区擦除最长 400ms
# → 最坏约 6.4s。用默认 3s 会在真实硬件上偶发"起始帧超时"。
ERASE_TIMEOUT = 30.0

# 被烧板分区上限 (与固件 flash_map.h 的 DUT_BOOT_SIZE / DUT_APP_SIZE 一致)
DUT_BOOT_MAX = 0x5000   # 20KB  Bootloader
DUT_APP_MAX  = 0xA000   # 40KB  App


# ────────────────────── 固件镜像本地预检 (不碰硬件) ──────────────────────
def check_image_blob(data: bytes, label: str, max_size: int):
    """对镜像内容做本地预检。

    @return (ok, msgs)  ok=False 表示硬错误(不应继续上传), msgs 为要打印的行

    检查项:
      1. 非空
      2. 不超过被烧板对应分区上限 —— 与固件 APP_ProgramPreCheck 的上限一致,
         提前拦下比上传 40KB 后再被拒更省时间
      3. 向量表看起来像不像 Cortex-M 镜像 (软提示, 不拦)
    """
    msgs = []
    if len(data) == 0:
        return False, ["%s: 文件为空" % label]
    if len(data) > max_size:
        return False, ["%s: %d 字节 超过分区上限 %d 字节 (0x%X); "
                       "若分区表有变, 改 foc_jig_tool.py 的 DUT_*_MAX 与固件 flash_map.h"
                       % (label, len(data), max_size, max_size)]

    msgs.append("%s: %d 字节 (上限 %d)" % (label, len(data), max_size))
    if len(data) >= 8:
        sp = int.from_bytes(data[0:4], "little")
        pc = int.from_bytes(data[4:8], "little")
        msgs.append("    向量表: 初始 SP=0x%08X  复位向量 PC=0x%08X" % (sp, pc))
        if not (0x20000000 <= sp < 0x20010000):
            msgs.append("    ! 提示: SP 不像 STM32F1 的 RAM(0x2000xxxx), 确认没拿错文件")
        if (pc & 1) == 0:
            msgs.append("    ! 提示: 复位向量不是奇数(Thumb 位), 可能不是有效的 Cortex-M 镜像")
        elif pc in (0x00000000, 0xFFFFFFFF):
            msgs.append("    ! 提示: 复位向量异常")
    return True, msgs


def load_and_check(path: str, label: str, max_size: int):
    """读文件 + 预检。返回 (ok, data_or_None, msgs)。"""
    if not os.path.isfile(path):
        return False, None, ["%s: 文件不存在 %s" % (label, path)]
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as exc:
        return False, None, ["%s: 读取失败 %s (%s)" % (label, path, exc)]
    ok, msgs = check_image_blob(data, label, max_size)
    return ok, data, msgs


# ─────────────────────────── 帧层 ───────────────────────────
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def build_frame(cmd: int, seq: int, data: bytes = b"") -> bytes:
    if len(data) > MAX_DATA:
        raise ValueError("DATA 长度 %d 超过单帧上限 %d" % (len(data), MAX_DATA))
    body = bytes([HDR0, HDR1,
                  (cmd >> 8) & 0xFF, cmd & 0xFF,
                  (seq >> 8) & 0xFF, seq & 0xFF,
                  (len(data) >> 8) & 0xFF, len(data) & 0xFF]) + data
    c = crc16_modbus(body)
    return body + bytes([(c >> 8) & 0xFF, c & 0xFF])


def parse_frame(buf: bytes):
    """返回 (cmd, seq, data); 不合法返回 None"""
    if len(buf) < 10 or buf[0] != HDR0 or buf[1] != HDR1:
        return None
    dlen = (buf[6] << 8) | buf[7]
    if len(buf) != 8 + dlen + 2:
        return None
    if crc16_modbus(buf[:8 + dlen]) != ((buf[8 + dlen] << 8) | buf[9 + dlen]):
        return None
    return ((buf[2] << 8) | buf[3], (buf[4] << 8) | buf[5], buf[8:8 + dlen])


def hx(b: bytes) -> str:
    return " ".join("%02X" % x for x in b)


# ─────────────────────────── 串口链路 ───────────────────────────
class JigLink:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        try:
            import serial  # pyserial
        except ImportError:
            sys.exit("需要 pyserial:  pip install pyserial")
        # USB CDC 虚拟串口: 波特率仅为形式参数
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
        except Exception as exc:
            sys.exit("打不开串口 %s: %s\n"
                     "  排查: ① 端口号是否正确(设备管理器里看)\n"
                     "        ② 是否被其它程序占用(串口助手 / Keil 的调试串口)\n"
                     "        ③ 治具是否已插 USB 并完成枚举" % (port, exc))
        self.seq = 0
        self.rx = b""

    def next_seq(self) -> int:
        self.seq = 1 if self.seq >= 0xFFFF else self.seq + 1
        return self.seq

    def _read_some(self):
        n = self.ser.in_waiting
        if n:
            self.rx += self.ser.read(n)

    def request(self, cmd: int, data: bytes = b"", timeout: float = ACK_TIMEOUT,
                seq: int = None):
        """发一帧并等回帧; 返回 (seq, data) 或 None(超时)"""
        s = self.seq if seq is None else seq
        if seq is None:
            s = self.next_seq()
        self.ser.write(build_frame(cmd, s, data))
        self.ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            self._read_some()
            while len(self.rx) >= 10:
                i = self.rx.find(bytes([HDR0, HDR1]))
                if i < 0:
                    self.rx = b""
                    break
                if i > 0:
                    self.rx = self.rx[i:]
                if len(self.rx) < 8:
                    break
                dlen = (self.rx[6] << 8) | self.rx[7]
                total = 8 + dlen + 2
                if len(self.rx) < total:
                    break
                fr = parse_frame(self.rx[:total])
                self.rx = self.rx[total:]
                if fr is None:
                    continue
                rcmd, rseq, rdata = fr
                if rcmd == cmd and rseq == s:
                    return (rseq, rdata)
            time.sleep(0.002)
        return None


# ─────────────────────────── 业务命令 ───────────────────────────
def do_query_flash(link: JigLink):
    r = link.request(CMD_QUERY_FLASH_VER)
    if r is None:
        print("超时: 无应答")
        return 1
    _, d = r
    st = d[0]
    print("0x000C Flash 固件版本 -> %s" % status_name(st))
    if st == 0x00:
        print("        版本 = %d.%d.%d" % (d[1], d[2], d[3]))
    return 0 if st == 0x00 else 1


def do_query_foc(link: JigLink):
    r = link.request(CMD_QUERY_FOC_VER)
    if r is None:
        print("超时: 无应答")
        return 1
    _, d = r
    st = d[0]
    print("0x000D FOC 板版本 -> %s" % status_name(st))
    print("        版本 = %d.%d.%d" % (d[1], d[2], d[3]))
    return 0 if st == 0x00 else 1


def do_upload(link: JigLink, path: str, ver: str, max_size: int = DUT_APP_MAX,
              label: str = "固件"):
    ok, data, msgs = load_and_check(path, label, max_size)
    for m in msgs:
        print("  " + m)
    if not ok:
        print("!! 预检未通过, 不上传")
        return 1
    if len(data) > DUT_BOOT_MAX:
        print("  (注: 超过 Bootloader 分区上限 %d 字节, 只能作为 App 写入 Slot1)"
              % DUT_BOOT_MAX)
    vs = [int(x) for x in ver.split(".")]
    while len(vs) < 3:
        vs.append(0)
    ver_b = bytes(vs[:3])
    frames = (len(data) + MAX_DATA - 1) // MAX_DATA
    if frames == 0 or frames > 0xFFFF:
        print("帧数非法: %d" % frames)
        return 1

    print("上传 %s: %d 字节, %d 帧, 版本 %d.%d.%d"
          % (path, len(data), frames, ver_b[0], ver_b[1], ver_b[2]))

    # 0x0009 起始帧: [2B 总帧数][4B 文件大小][3B 版本号] 全大端
    payload = (bytes([(frames >> 8) & 0xFF, frames & 0xFF]) +
               bytes([(len(data) >> 24) & 0xFF, (len(data) >> 16) & 0xFF,
                      (len(data) >> 8) & 0xFF, len(data) & 0xFF]) + ver_b)
    r = link.request(CMD_FW_START, payload, timeout=ERASE_TIMEOUT)
    if r is None:
        print("起始帧超时 (已等 %.0fs; 该帧会触发扇区擦除, 最多 ~6.4s)" % ERASE_TIMEOUT); return 1
    if r[1][0] != 0x00:
        print("起始帧被拒: %s" % status_name(r[1][0])); return 1
    print("  起始帧 ACK OK")

    # 0x000A 数据帧
    for i in range(frames):
        chunk = data[i * MAX_DATA:(i + 1) * MAX_DATA]
        r = link.request(CMD_FW_DATA, chunk, timeout=UPLOAD_TIMEOUT)
        if r is None:
            print("第 %d 帧超时" % (i + 1)); return 1
        if r[1][0] != 0x00:
            print("第 %d 帧出错: %s (已锁存, 中止)" % (i + 1, status_name(r[1][0])))
            return 1
        if (i + 1) % 32 == 0 or (i + 1) == frames:
            print("  %d/%d 帧" % (i + 1, frames))

    # 0x000B 结束帧
    r = link.request(CMD_FW_END)
    if r is None:
        print("结束帧超时"); return 1
    if r[1][0] != 0x00:
        print("结束帧出错: %s" % status_name(r[1][0])); return 1
    print("  结束帧 ACK OK  (已写完整标志并切槽)")
    return 0


def do_program(link: JigLink):
    print("0x0112 烧录中 (协议规定烧完才回 ACK, 最长等 %ds)..." % int(PROGRAM_TIMEOUT))
    t0 = time.time()
    r = link.request(CMD_PROGRAM, timeout=PROGRAM_TIMEOUT)
    dt = time.time() - t0
    if r is None:
        print("超时: 无应答 (%.1fs)" % dt)
        return 1
    st = r[1][0]
    print("0x0112 -> %s   (耗时 %.1fs)" % (status_name(st), dt))
    return 0 if st == 0x00 else 1


def preflight_burn(boot_path: str, app_path: str):
    """一键烧录前的本地预检: 两个文件都检查通过才允许接触硬件。

    刻意放在**开串口之前**调用 —— 文件搞错不该先被"端口打不开"挡住。
    @return (ok, lines)
    """
    lines = []
    ok_b, _, mb = load_and_check(boot_path, "Bootloader", DUT_BOOT_MAX)
    ok_a, _, ma = load_and_check(app_path,  "App",        DUT_APP_MAX)
    lines += mb + ma
    return (ok_b and ok_a), lines


def do_burn(link: JigLink, boot_path: str, app_path: str,
            boot_ver: str, app_ver: str):
    """一键完整烧录流程: 上传 Bootloader → 上传 App → 校验 → 0x0112 烧录

    注: 固件文件预检已在 main() 里、开串口之前做过 (见 preflight_burn)。
    """
    print("=" * 60)
    print("FOC 烧录治具 — 一键烧录流程")
    print("=" * 60)

    # ── 1/4 上传 Bootloader (自动写 Slot0) ──
    print("\n[1/4] 上传 Bootloader → Slot0")
    if do_upload(link, boot_path, boot_ver, DUT_BOOT_MAX, "Bootloader") != 0:
        print("\n!! 上传 Bootloader 失败, 流程中止 (治具 Flash 未被破坏: 见 0x000C 状态码)")
        return 1

    # ── 2/4 上传 App (自动切到 Slot1) ──
    print("\n[2/4] 上传 App → Slot1")
    if do_upload(link, app_path, app_ver, DUT_APP_MAX, "App") != 0:
        print("\n!! 上传 App 失败, 流程中止")
        return 1

    # ── 3/4 回读治具 Flash 确认 (App 槽优先) ──
    print("\n[3/4] 校验治具 Flash")
    r = link.request(CMD_QUERY_FLASH_VER)
    if r is None:
        print("!! 查询无应答"); return 1
    if r[1][0] != 0x00:
        print("!! 查询失败: %s" % status_name(r[1][0])); return 1
    got = "%d.%d.%d" % (r[1][1], r[1][2], r[1][3])
    print("    治具 Flash 中版本 = %s (期望 %s)" % (got, app_ver))
    if got != app_ver:
        print("!! 版本号不符"); return 1

    # ── 4/4 烧录到被烧板 ──
    print("\n[4/4] 0x0112 烧录到被烧板 (烧完才回 ACK, 最长等 %ds)" % int(PROGRAM_TIMEOUT))
    t0 = time.time()
    r = link.request(CMD_PROGRAM, timeout=PROGRAM_TIMEOUT)
    dt = time.time() - t0
    if r is None:
        print("!! 烧录无应答 (%.1fs)" % dt)
        print("   排查: 目标是否供电/共地 / SWCLK-SWDIO 是否接对 / 目标是否把 PA13-PA14 当普通 IO 用")
        return 1
    st = r[1][0]
    if st != 0x00:
        print("!! 烧录失败: %s  (耗时 %.1fs)" % (status_name(st), dt))
        if st == 0x04:
            print("   排查: 目标未供电或未共地 / SWD 两线接错 / RDP 读保护(需先整片擦除)")
        return 1

    print("\n" + "=" * 60)
    print("★ 烧录完成 OK   (SWD 烧录耗时 %.1fs)" % dt)
    print("=" * 60)
    return 0


def do_raw(link: JigLink, hexs: str):
    try:
        raw = bytes(int(x, 16) for x in hexs.replace(",", " ").split())
    except ValueError:
        print("非法十六进制"); return 1
    print("TX: %s" % hx(raw))
    link.ser.write(raw)
    link.ser.flush()
    time.sleep(0.5)
    link._read_some()
    print("RX: %s" % (hx(link.rx) if link.rx else "(无)"))
    return 0


# ─────────────────────────── 自检 / 演示 ───────────────────────────
# 与固件侧宿主单测 (test/protocol_test.c) 输出的字节完全一致, 用于跨语言交叉验证
VECTORS = [
    ("0x0009 固件起始帧", CMD_FW_START, 0x1234,
     bytes([0x00, 0x02, 0x00, 0x00, 0x10, 0x00, 0x01, 0x02, 0x03]),
     "A5 5B 00 09 12 34 00 09 00 02 00 00 10 00 01 02 03 C8 61"),
    ("0x000A 固件数据帧", CMD_FW_DATA, 0x1234,
     bytes([0xDE, 0xAD, 0xBE, 0xEF]),
     "A5 5B 00 0A 12 34 00 04 DE AD BE EF 79 2E"),
    ("0x000B 固件结束帧", CMD_FW_END, 0x1234, b"",
     "A5 5B 00 0B 12 34 00 00 FE 81"),
    ("0x000C 查Flash版本", CMD_QUERY_FLASH_VER, 0x1234, b"",
     "A5 5B 00 0C 12 34 00 00 3E 34"),
    ("0x000D 查FOC板版本", CMD_QUERY_FOC_VER, 0x1234, b"",
     "A5 5B 00 0D 12 34 00 00 FE 09"),
    ("0x0112 烧录FOC板", CMD_PROGRAM, 0x1234, b"",
     "A5 5B 01 12 12 34 00 00 ED 9D"),
    ("ACK 0x0009 成功", CMD_FW_START, 0x0001, bytes([0x00]),
     "A5 5B 00 09 00 01 00 01 00 DD 89"),
    ("ACK 0x000C v1.0.0", CMD_QUERY_FLASH_VER, 0x0001,
     bytes([0x00, 0x01, 0x00, 0x00]),
     "A5 5B 00 0C 00 01 00 04 00 01 00 00 2A 2A"),
    ("ACK 0x000D 失败", CMD_QUERY_FOC_VER, 0x0005,
     bytes([0x0C, 0x00, 0xFF, 0xFF]),
     "A5 5B 00 0D 00 05 00 04 0C 00 FF FF 5A 31"),
]


def do_selftest():
    fails = 0
    checks = 0
    print("=== 交叉校验: Python 实现 vs 固件 C 实现 实际字节 ===\n")

    checks += 1
    c = crc16_modbus(b"123456789")
    ok = (c == 0x4B37)
    print("[CRC] CRC16(\"123456789\") = 0x%04X (标准 0x4B37)  %s" % (c, "OK" if ok else "FAIL"))
    fails += 0 if ok else 1

    for name, cmd, seq, data, expect in VECTORS:
        checks += 1
        got = hx(build_frame(cmd, seq, data))
        ok = (got == expect)
        print("[帧 ] %-20s %s  %s" % (name, got, "OK" if ok else "FAIL\n         expect: " + expect))
        fails += 0 if ok else 1

    # 解析往返
    checks += 1
    rt_ok = True
    for name, cmd, seq, data, expect in VECTORS:
        fr = parse_frame(bytes(int(x, 16) for x in expect.split()))
        if fr is None or fr[0] != cmd or fr[1] != seq or fr[2] != data:
            rt_ok = False
            print("  解析失败: %s -> %s" % (name, fr))
    print("[解帧] 全部向量解析往返                 %s" % ("OK" if rt_ok else "FAIL"))
    fails += 0 if rt_ok else 1

    # 篡改 CRC 必须被拒
    checks += 1
    bad = bytearray(build_frame(CMD_FW_END, 1))
    bad[-1] ^= 0xFF
    rej = (parse_frame(bytes(bad)) is None)
    print("[拒帧] 篡改 CRC 被拒绝                  %s" % ("OK" if rej else "FAIL"))
    fails += 0 if rej else 1

    # 固件镜像预检
    print("[预检] 固件镜像本地预检 (check_image_blob)")
    def _mk(size, sp=0x20005000, pc=0x08000101):
        if size == 0:
            return b""
        b = bytearray(size)
        if size >= 8:
            b[0:4] = sp.to_bytes(4, "little")
            b[4:8] = pc.to_bytes(4, "little")
        return bytes(b)

    for name, blob, mx, want in [
        ("空文件",           _mk(0),                      DUT_APP_MAX, False),
        ("超分区上限",       _mk(DUT_APP_MAX + 1),        DUT_APP_MAX, False),
        ("正常 F1 镜像",     _mk(1024),                   DUT_APP_MAX, True),
        ("SP 不在 RAM",      _mk(1024, sp=0x00000000),    DUT_APP_MAX, True),
        ("复位向量非 Thumb", _mk(1024, pc=0x08000100),    DUT_APP_MAX, True),
        ("Bootloader 超限",  _mk(DUT_BOOT_MAX + 1),       DUT_BOOT_MAX, False),
    ]:
        checks += 1
        ok, _ = check_image_blob(blob, name, mx)
        good = (ok == want)
        print("  %-16s ok=%-5s (期望 %-5s)  %s" % (name, ok, want, "OK" if good else "FAIL"))
        fails += 0 if good else 1

    print("\n=== 共 %d 项, 失败 %d 项 ===" % (checks, fails))
    return 0 if fails == 0 else 1


def do_demo():
    print("各命令的请求帧 (SEQ 示例 0x0001):\n")
    for cmd, name, data in [
        (CMD_FW_START, "0x0009 固件起始帧 (总帧数=2, 大小=0x1000, ver 1.2.3)",
         bytes([0x00, 0x02, 0x00, 0x00, 0x10, 0x00, 0x01, 0x02, 0x03])),
        (CMD_FW_DATA, "0x000A 固件数据帧 (4B)", bytes([0xDE, 0xAD, 0xBE, 0xEF])),
        (CMD_FW_END, "0x000B 固件结束帧", b""),
        (CMD_QUERY_FLASH_VER, "0x000C 查固件版本(Flash中)", b""),
        (CMD_QUERY_FOC_VER, "0x000D 查固件版本(FOC板中)", b""),
        (CMD_PROGRAM, "0x0112 烧录Flash固件到FOC板", b""),
    ]:
        print("  %-52s %s" % (name, hx(build_frame(cmd, 1, data))))
    print("\n各命令的应答帧示例:\n")
    for cmd, name, data in [
        (CMD_FW_START, "ACK 0x0009 成功", bytes([0x00])),
        (CMD_FW_DATA, "ACK 0x000A 溢出", bytes([0x0D])),
        (CMD_FW_END, "ACK 0x000B 字节数不符", bytes([0x0A])),
        (CMD_QUERY_FLASH_VER, "ACK 0x000C 无固件", bytes([0x06, 0x00, 0xFF, 0xFF])),
        (CMD_QUERY_FOC_VER, "ACK 0x000D 失败", bytes([0x0C, 0x00, 0xFF, 0xFF])),
        (CMD_PROGRAM, "ACK 0x0112 烧录成功", bytes([0x00])),
    ]:
        print("  %-52s %s" % (name, hx(build_frame(cmd, 1, data))))
    return 0


# ─────────────────────────── main ───────────────────────────
def main():
    ap = argparse.ArgumentParser(description="FOC 烧录治具 PC 端协议工具")
    ap.add_argument("action", choices=["selftest", "demo", "burn", "query-flash",
                                       "query-foc", "upload", "program", "raw"])
    ap.add_argument("arg", nargs="?", help="upload: 固件文件; raw: 十六进制字节")
    ap.add_argument("-p", "--port", help="串口, 如 COM5")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="形式波特率 (USB CDC 忽略)")
    ap.add_argument("-v", "--version", default="1.0.0", help="上传固件的版本号 主.次.修订")
    ap.add_argument("--boot", help="burn: Bootloader 固件文件 (写入 Slot0)")
    ap.add_argument("--app", help="burn: App 固件文件 (写入 Slot1)")
    ap.add_argument("--boot-ver", help="burn: Bootloader 版本号 (默认同 -v)")
    ap.add_argument("--app-ver", help="burn: App 版本号 (默认同 -v)")
    a = ap.parse_args()

    if a.action == "selftest":
        return do_selftest()
    if a.action == "demo":
        return do_demo()

    if not a.port:
        print("需要 -p/--port (或先用 selftest/demo 无硬件自检)")
        return 2

    # 先把参数校验完再开串口, 避免"参数错了却先报串口打不开"
    if a.action == "upload" and not a.arg:
        print("upload 需要固件文件路径"); return 2
    if a.action == "burn" and (not a.boot or not a.app):
        print("burn 需要 --boot <bl.bin> --app <app.bin>"); return 2
    if a.action == "raw" and not a.arg:
        print("raw 需要十六进制字节"); return 2

    # 一键烧录: 文件预检放在开串口之前 —— 文件搞错不该先被"端口打不开"挡住
    if a.action == "burn":
        ok, lines = preflight_burn(a.boot, a.app)
        print("预检固件文件:")
        for m in lines:
            print("  " + m)
        if not ok:
            print("\n!! 预检未通过, 未接触硬件 (未开串口, 治具 Flash 与被烧板都未被改动)")
            return 1
        print("  预检通过\n")

    link = JigLink(a.port, a.baud)

    if a.action == "query-flash":
        return do_query_flash(link)
    if a.action == "query-foc":
        return do_query_foc(link)
    if a.action == "upload":
        return do_upload(link, a.arg, a.version)
    if a.action == "program":
        return do_program(link)
    if a.action == "burn":
        return do_burn(link, a.boot, a.app,
                       a.boot_ver or a.version, a.app_ver or a.version)
    if a.action == "raw":
        return do_raw(link, a.arg)
    return 2


if __name__ == "__main__":
    sys.exit(main())
