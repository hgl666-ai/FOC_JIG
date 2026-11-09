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
        frame = build_frame(cmd, s, data)
        nw = self.ser.write(frame)
        if nw != len(frame):
            # 只写出去一部分 = 后面必定超时（帧被截断），必须当场看见
            print("   !! 发帧只写出去 %d/%d 字节（PC 侧就把帧截断了）" % (nw, len(frame)))
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


# ─────────────────────────────────────────────────────────────────────────
#  被测板五项功能（① 电压 / ② MCU 脉冲 / ③ 三相输出 / ④ 编码器 / ⑤ 相电流）
#----------------------------------------------------------------------------
#  这五条命令都要求治具固件是 **JIG_DUT_LINK=1** 的台架版（Keil target U1_DUT）。
#  台架接线：PC → USART1(PA9/PA10)，被测 FOC 板 → USART3(PB10/PB11)，
#            H3/H4 跳线拔掉（U2 不参与）。
#  布局与长度全部与 U1/Core/Inc/flash_map.h 对齐，改协议必改这里。
# ─────────────────────────────────────────────────────────────────────────
CMD_DUT_RAW      = 0x0011
CMD_CUR_TEST     = 0x0014
CMD_VOL_TEST     = 0x0015
CMD_PULSE_TEST   = 0x0019      # ③ 一条命令：进测试模式→发PWM→采集→关输出
CMD_PHASE_TEST   = 0x001A      # ④ 一条命令：…→判定→关输出
CMD_ENC_TEST     = 0x001B      # ⑤ 一条命令
CMD_JIG_STATUS   = 0x0017
CMD_RUN_ALL      = 0x0010
CMD_DUT_SEND     = 0x001F      # 给被测板发原始字节（不套帧）+ 抓回应
CMD_DUT_RECV     = 0x0020      # 只读：取走 DUT 口攒下的原始字节
DUT_SEND_MAX     = 62          # 一次最多发多少原始字节（与固件 MGR_DUT_SEND_MAX 一致）

DUT_PING, DUT_INFO, DUT_TEST_MODE, DUT_STOP = 0x0200, 0x0201, 0x0202, 0x0203
DUT_PWM, DUT_LEVEL, DUT_CURRENT, DUT_GET_ADC, DUT_GET_ENC = 0x0204, 0x0205, 0x0206, 0x0207, 0x0208

# 治具 9 路信号序号（与 docs/07 §2.1、dut_proto.h 的 DUT_ADC_* 完全一致）
CMD_JIG_STATUS_RAW = 0x0017      # 定位用（do_probe / do_status 共用）

CH_NAMES = ["24V 母线", "10V 驱动", "5V", "3.3V",
            "A 相电压", "B 相电压", "C 相电压", "电流检测1", "电流检测2"]
PH_NAMES = ["A", "B", "C"]

# 一路电压/电流的判定旗标（vol_chk.h / cur_chk.h 共用低 4 位）
FLAG_OK      = 0x01
FLAG_NODATA  = 0x02      # 数据不全 → **不判**（不是失败）
FLAG_NOFLOW  = 0x04
FLAG_DEVIATE = 0x08      # 与对侧（DUT）读数差超容差 = 真故障


def be16(b, o):
    return (b[o] << 8) | b[o + 1]


def be32(b, o):
    return (b[o] << 24) | (b[o + 1] << 16) | (b[o + 2] << 8) | b[o + 3]


def s16(v):
    return v - 0x10000 if v >= 0x8000 else v


def _dw(s):
    """字符串的**显示**宽度：中文/全角算 2 列（否则表格对不齐）"""
    import unicodedata
    return sum(2 if unicodedata.east_asian_width(c) in ("W", "F") else 1 for c in s)


def pad(s, width):
    """按显示宽度左对齐补空格"""
    n = width - _dw(s)
    return s + (" " * n if n > 0 else "")


def mv_str(mv):
    """mV → 人类可读（≥1000 显示 V）"""
    return ("%.3f V" % (mv / 1000.0)) if abs(mv) >= 1000 else ("%d mV" % mv)


def flags_str(f):
    if f & FLAG_DEVIATE:
        return "超差"
    if f & FLAG_NOFLOW:
        return "没通流"
    if f & FLAG_NODATA:
        return "不判(数据缺)"
    if f & FLAG_OK:
        return "合格"
    return "未判(0x%02X)" % f


def _req(link, cmd, data, timeout, what, retry=True):
    """发一条命令并等应答；超时**按协议重试一次**（同一 SEQ），再不行就打人话返回 None

    为什么默认重试：协议 docs/17 就规定"同一 SEQ 重试 1 次"。台架上单帧被打断
    （对端忙、线上丢字节、解析器刚好在复位）是常事，重试一次能省掉一轮人工操作。
    """
    r = link.request(cmd, data, timeout=timeout)
    if (r is None) and (retry is False):
        raw = bytes(link.rx)
        link.rx = b""
        print("!! %s 没有应答（%.1fs 超时），本命令**不重试**（重发会把动作做两遍）"
              % (what, timeout))
        if raw:
            print("   这段时间收到：%s" % hx(raw[:64]))
        return None
    if r is None:
        # 超时也要把“这段时间收到的字节”打出来：这是分清
        #   ① 一个字节都没回（请求没到 / 应答没发）
        #   ② 回了非协议文本（比如台架版的心跳）
        #   ③ 回了半截帧（被截断）
        # 的唯一证据，比“超时”三个字有用得多。
        raw = bytes(link.rx)
        link.rx = b""
        print("   第一次超时（%.1fs）" % timeout)
        if raw:
            print("   这段时间共收到 %d 字节：" % len(raw))
            print("     hex  : %s" % hx(raw[:80]))
            print("     ascii: %r" % raw[:80])
        else:
            print("   这段时间**一个字节都没收到**")
        # 先空等 0.2s：如果应答"迟到"（治具处理超时），它会在这一刻到达。
        time.sleep(0.20)
        link._read_some()
        late = bytes(link.rx)
        if late:
            print("   又等了 0.2s，收到 %d 字节：%s" % (len(late), hx(late[:80])))
            print("   ⇒ 这是**迟到的应答**：治具处理这条命令超过了 %.1fs，" % timeout)
            print("     不是没回，是回得太晚 —— 要查治具侧为什么慢")
        else:
            print("   又等 0.2s 仍然没有字节")
        link.rx = b""
        # 用**新序号**重试：迟到的旧应答（旧 SEQ）不会被误认成本次的结果
        link.seq = (link.seq + 1) & 0xFFFF
        if link.seq == 0:
            link.seq = 1
        print("   （换新 SEQ=%d 重试一次）" % link.seq)
        r = link.request(cmd, data, timeout=timeout)
    if r is None:
        print("!! %s 没有应答（%.1fs 超时）" % (what, timeout))
        print("   排查：① 治具是不是 U1_DUT 那版固件（0x0017 看 DUT_LINK 位）")
        print("         ② COM 口 / 波特率 115200 8N1 / USB-TTL 的 TX-RX 交叉、共地")
        print("         ③ 命令要等 0x0014~0x001B 跑完才回，别中途按 Ctrl-C 又重发")
    return r


# ─────────────────────────── ① DUT 协议自检 ───────────────────────────
def do_dut_ping(link, payload: bytes = b"\xDE\xAD\xBE\xEF"):
    """0x0011 透传 DUT 的 PING：整条链路（PC→治具→被测板→治具→PC）一次验完"""
    print("→ 0x0011 透传 0x0200 PING，DATA=%s" % hx(payload))
    r = _req(link, CMD_DUT_RAW, bytes([0x02, 0x00]) + payload, ACK_TIMEOUT, "PING")
    if r is None:
        return 1
    _, d = r
    st = d[0]
    echo = d[1:]
    print("  治具状态 = %s" % status_name(st))
    if st == 0x0C:
        print("  ⇒ **治具没等到被测板回帧**。这不是治具的问题，往下看：")
        print("     1. FOC 固件里有没有实现 0x0200~0x0208（docs/16 那 9 条）")
        print("     2. 被测板 TX → 治具 PB11、被测板 RX → 治具 PB10（**交叉**）")
        print("     3. 两边共地；被测板 3.3V 逻辑电平；115200 8N1")
        print("     4. 被测板是不是还在跑不响应串口的固件（电机控制主循环里没开串口）")
        return 1
    if st != 0x00:
        print("  ⇒ 治具拒发（%s）：多半是透传命令号不在 0x0200~0x02FF，或链路忙"
              % status_name(st))
        return 1
    print("  被测板回显 = %s" % hx(echo))
    if echo == payload:
        print("  ✅ 完全一致 —— **DUT 链路通了，被测板固件实现了 DUT 协议**")
        return 0
    print("  ⚠ 回显与请求不一致：链路通但内容不对（查被测板 PING 是不是真回环）")
    return 1


def do_dut_info(link):
    """0x0011 透传 0x0201 INFO：拿能力位图，决定后面哪几项能测"""
    print("→ 0x0011 透传 0x0201 INFO")
    r = _req(link, CMD_DUT_RAW, bytes([0x02, 0x01]), ACK_TIMEOUT, "INFO")
    if r is None:
        return 1
    _, d = r
    st = d[0]
    print("  治具状态 = %s" % status_name(st))
    if st != 0x00 or len(d) < 17:
        print("  ⇒ 没拿到被测板 INFO（治具状态 %s，回帧 %d 字节）" % (status_name(st), len(d)))
        if st == 0x00 and len(d) < 17:
            print("     被测板 INFO 应答短了：docs/16 §2.2 要求 16 字节（status+dut_ver+fw+caps+范围+mask）")
        return 1
    b = d[1:]
    dut_st  = b[0]
    dut_ver = be16(b, 1)
    fw      = "%d.%d.%d" % (b[3], b[4], b[5])
    caps    = be16(b, 6)
    fmin    = be16(b, 8)
    fmax    = be16(b, 10)
    amask   = be16(b, 12)
    print("  被测板状态   = %s" % status_name(dut_st))
    print("  DUT 协议版本 = 0x%04X（应为 0x0101）" % dut_ver)
    print("  固件版本     = %s" % fw)
    print("  能力位图     = 0x%04X" % caps)
    names = {DUT_PING: "PING", DUT_INFO: "INFO", DUT_TEST_MODE: "TEST_MODE",
             DUT_STOP: "STOP", DUT_PWM: "PWM", DUT_LEVEL: "LEVEL",
             DUT_CURRENT: "CURRENT", DUT_GET_ADC: "GET_ADC", DUT_GET_ENC: "GET_ENC"}
    have, miss = [], []
    for cmd, nm in names.items():
        (have if caps & (1 << (cmd - 0x0200)) else miss).append(nm)
    print("     支持：%s" % (", ".join(have) if have else "(无)"))
    print("     不支持：%s" % (", ".join(miss) if miss else "(无)"))
    print("  PWM 频率范围 = %d ~ %d Hz" % (fmin, fmax))
    print("  ADC 通道掩码 = 0x%04X → %s" % (amask,
          ", ".join(CH_NAMES[i] for i in range(9) if amask & (1 << i)) or "(一个都没声明)"))
    print()
    print("  ⇒ 后面能测什么（按这份位图）：")
    print("     ① 电压检测   0x0015 : %s" % ("可以（只要 GET_ADC 有通道）" if amask and (caps & (1 << 7)) else "被测板没声明 GET_ADC"))
    print("     ② MCU 脉冲   0x0019 : %s" % ("可以" if caps & (1 << 4) else "被测板没实现 PWM（0x0204）"))
    print("     ③ 三相输出   0x001A : %s" % ("可以" if caps & (1 << 4) else "被测板没实现 PWM（0x0204）"))
    print("     ④ 编码器     0x001B : %s" % ("可以" if caps & (1 << 8) else "被测板没实现 GET_ENC（0x0208）"))
    print("     ⑤ 相电流     0x0014 : %s" % ("可以" if (caps & (1 << 6)) and (caps & (1 << 7)) else "被测板缺 CURRENT 或 GET_ADC"))
    return 0


# ─────────────────────────── ① 电压检测 0x0015 ───────────────────────────
def _flag_word(f):
    """九路/两路共用的判定说法（三个旗标位，各判据层含义一致）"""
    if f & 0x08:
        return "超差"
    if f & 0x02:
        return "不判(数据缺)"
    if f & 0x01:
        return "合格"
    return "未判(0x%02X)" % f


def do_vol(link):
    """0x0015 电压检测：九路逐个比"治具读到的"和"被测板自己报的" """
    print("→ 0x0015 电压检测（治具九路 vs 被测板自己报的）")
    r = _req(link, CMD_VOL_TEST, b"", 3.0, "0x0015")
    if r is None:
        return 1
    _, d = r
    if len(d) < 2:
        print("  应答只有 %d 字节（状态 %s）" % (len(d), status_name(d[0])))
        return 1
    st, dut_st = d[0], d[1]
    need = 2 + 9 * 5
    if len(d) != need:
        print("  应答 %d 字节，应该是 %d 字节（固件与工具版本不一致？）" % (len(d), need))
        return 1
    print("  判定 = %s%s" % (status_name(st),
          "（**测完了、有通道不合格**，不是通讯错误）" if st == 0x05 else ""))
    print("  被测板应答 = %s（00 表示它按协议回了）" % status_name(dut_st))
    print()
    print("  ch  %s  %s  %s  %s" % (pad("测点", 12), pad("治具读到", 12),
                                     pad("被测板报的", 12), pad("差值", 8)) + "  判定")
    bad = 0
    for i in range(9):
        o = 2 + i * 5
        jm = be16(d, o)
        dm = be16(d, o + 2)
        f = d[o + 4]
        if (f & 0x08) or (f & 0x01) == 0 and (f & 0x02) == 0:
            bad += 1
        print("  %-2d  %s  %s  %s  %s  %s"
              % (i, pad(CH_NAMES[i], 12), pad(mv_str(jm), 12),
                 pad(mv_str(dm), 12), pad("%+d" % (jm - dm), 8), _flag_word(f)))
    print()
    print("  说明：差值那一列是工具算的（固件只发两个读数，省 2 字节/路）。")
    print("       24V/10V/5V/3.3V 应该就是电源的实际电压；标“不判(数据缺)”= 被测板没报这一路。")
    return 0 if st == 0x00 else 2


# ─────────────────────────── ② MCU 脉冲 0x0019 ───────────────────────────
def _peer_word(v):
    return {0: "没有对侧数据", 1: "对侧一致", 2: "**对侧不一致**"}.get(v, "?(%d)" % v)


def _pulse_flag_word(f):
    if f & 0x10:
        return "对侧不一致"
    if f & 0x02:
        return "没信号"
    if f & 0x04:
        return "超出量程"
    if f & 0x01:
        return "有信号"
    return "未测(0x%02X)" % f


def do_pulse(link):
    """0x0019 ② MCU 脉冲：让被测板发 1kHz/50%，治具量这三路的周期和占空比"""
    print("→ 0x0019 ② MCU 脉冲检测（进测试模式 → 让被测板发 1kHz/50% → 开 200ms 窗口 → 关输出）")
    print("   ⚠ 三相占空比必须相同（这里都是 50%），电机接着也不会产生净电压")
    r = _req(link, CMD_PULSE_TEST, b"", 8.0, "0x0019")
    if r is None:
        return 1
    _, d = r
    need = 2 + 3 * 7
    if len(d) != need:
        print("  应答 %d 字节，应该是 %d 字节（固件与工具版本不一致？）" % (len(d), need))
        return 1
    print("  判定 = %s" % status_name(d[0]))
    print("  对侧(U2)：%s" % _peer_word(d[1]))
    print()
    print("  相   频率        占空比     判定")
    for i in range(3):
        o = 2 + i * 7
        per = be32(d, o)
        duty = be16(d, o + 4)
        f = d[o + 6]
        freq = ("%.1f Hz" % (1e9 / per)) if per else "-"
        print("  %-2s   %s  %s   %s"
              % (PH_NAMES[i], pad(freq, 10),
                 pad(("%.1f%%" % (duty / 10.0)) if duty else "-", 10),
                 _pulse_flag_word(f)))
    print()
    print("  说明：1 kHz、50% 就是期望值。某相“没信号”= 那根线没接上，或被测板没输出那一路。")
    return 0 if d[0] == 0x00 else 2


# ─────────────────────────── ③ 三相输出 0x001A ───────────────────────────
def _phase_word(f):
    if f & 0x02:
        return "高阻(不判)"
    if f & 0x08:
        return "母线太低(不判)"
    if f & 0x04:
        return "数据不全(不判)"
    if f & 0x10:
        return "没输出"
    if f & 0x20:
        return "超差"
    if f & 0x40:
        return "占空比不符"
    if f & 0x01:
        return "合格"
    return "未判(0x%02X)" % f


def do_phase(link):
    """0x001A ③ 三相输出：量三相电压，看是不是约等于"占空比 × 母线电压" """
    print("→ 0x001A ③ 三相输出检测（同一次 PWM → 量相电压，判 V ≈ 占空比 × 母线）")
    r = _req(link, CMD_PHASE_TEST, b"", 8.0, "0x001A")
    if r is None:
        return 1
    _, d = r
    need = 1 + 3 * 5
    if len(d) != need:
        print("  应答 %d 字节，应该是 %d 字节（固件与工具版本不一致？）" % (len(d), need))
        return 1
    print("  判定 = %s%s" % (status_name(d[0]),
          "（**做完了但不合格**）" if d[0] == 0x05 else ""))
    print()
    print("  相   实测        期望(占空比×母线)   差值      判定")
    for i in range(3):
        o = 1 + i * 5
        mv = be16(d, o)
        ex = be16(d, o + 2)
        f = d[o + 4]
        print("  %-2s   %s  %s  %s  %s"
              % (PH_NAMES[i], pad(mv_str(mv), 12), pad(mv_str(ex), 18),
                 pad("%+d" % (mv - ex), 8), _phase_word(f)))
    print()
    print("  说明：50% 占空比 + 24V 母线 → 期望 12V。差值那一列是工具算的。")
    print("       实测≈0 且判“超差/没输出”→ 那一相没输出：驱动芯片、自举电容或相线有问题。")
    return 0 if d[0] == 0x00 else 2


# ─────────────────────────── ④ 编码器 0x001B ───────────────────────────
ENC_EXPECT = [2000, 4000, 2]        # 治具默认脉冲串 A=1000 / B=2000 / Z=1，折成边沿数


def _enc_word(f):
    if f & 0x02:
        return "不判(有一侧没数据)"
    if f & 0x04:
        return "对侧数不对"
    if f & 0x08:
        return "被测板数不对"
    if f & 0x01:
        return "合格"
    return "未判(0x%02X)" % f


def do_enc(link):
    """0x001B ④ 编码器：治具发定量脉冲，看被测板数到多少"""
    print("→ 0x001B ④ 编码器检测（进测试模式→清零→治具发 A/B/Z 脉冲→读被测板计数）")
    print("   默认脉冲串：A=1000  B=2000  Z=1（三路个数故意不同，接错线能看出来）")
    r = _req(link, CMD_ENC_TEST, b"", 8.0, "0x001B")
    if r is None:
        return 1
    _, d = r
    need = 3 + 3 * 3
    if len(d) != need:
        print("  应答 %d 字节，应该是 %d 字节（固件与工具版本不一致？）" % (len(d), need))
        return 1
    print("  判定 = %s%s" % (status_name(d[0]),
          "（**做完了但不合格**）" if d[0] == 0x05 else ""))
    dd = d[1] - 256 if d[1] >= 128 else d[1]
    print("  被测板报的方向 = %+d（只作参考，不参与判定）" % dd)
    print("  对侧(U2)：%s" % _peer_word(d[2]))
    print()
    print("  相   被测板数到   应该是   判定")
    for i in range(3):
        o = 3 + i * 3
        n = be16(d, o)
        f = d[o + 2]
        print("  %-2s   %s  %s   %s"
              % (PH_NAMES[i], pad(str(n), 10), pad(str(ENC_EXPECT[i]), 8), _enc_word(f)))
    print()
    print("  说明：1 个方波 = 2 个边沿，所以 A 发 1000 个脉冲应该数到 2000。")
    print("       某一路是 0 → 那根线没接对；只有一半 → 被测板只数了一种沿（固件问题）。")
    return 0 if d[0] == 0x00 else 2


# ─────────────────────────── ⑤ 相电流 0x0014 ───────────────────────────
def _cur_word(f):
    if f & 0x02:
        return "不判(数据缺)"
    if f & 0x04:
        return "没通上电流"
    if f & 0x08:
        return "两边不一致"
    if f & 0x01:
        return "合格"
    return "未判(0x%02X)" % f


def do_cur(link, hi=0x01, lo=0x02, duty=20, ms=100, mode=1):
    """0x0014 ⑤ 相电流：通流前后各读一次，比"变化量" """
    print("→ 0x0014 ⑤ 相电流检测：上桥 %s / 下桥 %s / %d‰ / %d ms / 模式=%s"
          % (chr(64 + hi.bit_length()), chr(64 + lo.bit_length()), duty, ms,
             "静态全通" if mode == 0 else "低频PWM"))
    if mode == 0:
        print("   ⚠ 静态全通 = 24V 直接压在两个绕组上（被测板会忽略占空比按 100% 处理）")
        print("      被测板接着电机时电流可能几十安培 —— 手放在电源开关上，随时断电")
    else:
        print("   小占空比 PWM：平均电流 = 24V × %d‰ / 绕组回路电阻，先小后大" % duty)
    data = bytes([hi, lo, (duty >> 8) & 0xFF, duty & 0xFF,
                  (ms >> 8) & 0xFF, ms & 0xFF, 0x03, 0x00, mode])
    r = _req(link, CMD_CUR_TEST, data, 6.0, "0x0014")
    if r is None:
        return 1
    _, d = r
    need = 2 + 2 * 5
    if len(d) != need:
        print("  应答 %d 字节，应该是 %d 字节（固件与工具版本不一致？）" % (len(d), need))
        return 1
    st, dut_st = d[0], d[1]
    print("  判定 = %s%s" % (status_name(st),
          "（**做完了但不合格**）" if st == 0x05 else ""))
    print("  被测板应答 = %s%s" % (status_name(dut_st),
          "  ← 0B=参数被拒（没实现这个 mode？） 0C=没应答" if dut_st else ""))
    print()
    print("  路   治具变化量   被测板变化量   判定")
    for i in range(2):
        o = 2 + i * 5
        dj = s16(be16(d, o))
        dd = s16(be16(d, o + 2))
        f = d[o + 4]
        print("  %-2d   %s  %s  %s"
              % (i + 1, pad("%+d mV" % dj, 12), pad("%+d mV" % dd, 14), _cur_word(f)))
    print()
    print("  说明：只比“通流中 − 通流前”的变化量，所以零点偏置不用管；")
    print("       两路都≈0 → 没电流流过（占空比太小，或被测板没真通流）。")
    return 0 if st == 0x00 else 2


# ────────────────────── DUT 口原始字节（0x001F / 0x0020）──────────────────────
def _phase_bit(name):
    """"A"/"B"/"C" → 相掩码 bit0/1/2；非法直接退出，别把 0 传下去"""
    m = {"A": 0x01, "B": 0x02, "C": 0x04}
    k = (name or "").strip().upper()
    if k not in m:
        raise SystemExit("--hi/--lo 只能是 A / B / C（收到 %r）" % name)
    return m[k]


def _ascii(b):
    """把字节串画成可读形式：可打印的原样输出，其余显示成十六进制"""
    return "".join(chr(c) if 32 <= c < 127 else ("\\x%02X" % c) for c in b)


EOL_MAP = {"none": b"", "crlf": b"\r\n", "lf": b"\n", "cr": b"\r"}


def do_dut_send(link, text, timeout_ms=200, label="原始字节", eol="none"):
    """0x001F：把一段字节原样发给被测板，并把这段时间它回的字节带回来。

    这是"对面固件还没实现 0x02xx 协议"时的过渡手段：FOC 板现在只认 ASCII 命令
    （AS:S800 转 / AS:S0 停），用它就能先证明 USART 这条线通不通。
    """
    if isinstance(text, str):
        payload = text.encode("ascii", "replace")
    else:
        payload = bytes(text)
    payload = payload + EOL_MAP.get(eol or "none", b"")   # 行结束符：多数 ASCII 协议按行解析
    if len(payload) == 0:
        print("要发的字节不能为空")
        return 1
    if len(payload) > DUT_SEND_MAX:
        print("一次最多发 %d 字节（这条发的是 %d 字节）" % (DUT_SEND_MAX, len(payload)))
        return 1
    if timeout_ms <= 0:
        timeout_ms = 200
    if timeout_ms > 2000:
        timeout_ms = 2000

    data = bytes([(timeout_ms >> 8) & 0xFF, timeout_ms & 0xFF]) + payload
    print("→ 0x001F 发 %s：%r（%d 字节，行结束符=%s），等 %d ms 看回应"
          % (label, payload, len(payload), eol or "none", timeout_ms))
    r = _req(link, CMD_DUT_SEND, data, 1.0 + timeout_ms / 1000.0, "0x001F", retry=False)
    if r is None:
        return 1
    _, d = r
    if len(d) < 5:
        print("  应答只有 %d 字节（状态 %s）" % (len(d), status_name(d[0])))
        return 1
    st    = d[0]
    sent  = be16(d, 1)
    rxlen = be16(d, 3)
    body  = bytes(d[5:5 + rxlen])
    print("  状态 = %s · 实际发出 %d 字节 · 收到 %d 字节"
          % (status_name(st), sent, rxlen))
    if body:
        print("  收到的内容 hex  : %s" % hx(body))
        print("  收到的内容 ascii: %r" % _ascii(body))
    print()
    if st != 0x00:
        print("  ⇒ 治具没发出去（%s）。多半是上一条还没发完，重试一次即可" % status_name(st))
        return 1
    if sent != len(payload):
        print("  ⇒ 只写出去 %d/%d 字节：发送队列被占，重发一次" % (sent, len(payload)))
        return 1
    if rxlen == 0:
        print("  ⇒ **字节已经发出去了，但对端一个字节都没回。**")
        print("     这不代表不通：很多固件收到命令就执行、不回应答。")
        print("     如果刚才发的是 AS:S800，看**电机有没有转** —— 转了就是通了；")
        print("     也可以用 dut-recv 过一会儿再看有没有迟到/主动上报的字节。")
        return 2
    print("  ⇒ ✅ 双向都通：发出去了，对端回了 %d 字节" % rxlen)
    return 0


def do_dut_recv(link):
    """0x0020：只读，把 DUT 口攒下的原始字节取走（读后清空）"""
    print("→ 0x0020 取走 DUT 口攒下的原始字节（只读，不发任何东西给对方）")
    r = _req(link, CMD_DUT_RECV, b"", 1.0, "0x0020")
    if r is None:
        return 1
    _, d = r
    if len(d) < 3:
        print("  应答只有 %d 字节（状态 %s）" % (len(d), status_name(d[0])))
        return 1
    n = be16(d, 1)
    body = bytes(d[3:3 + n])
    print("  状态 = %s · 字节数 = %d" % (status_name(d[0]), n))
    if n == 0:
        print("  （没攒下任何字节。若你刚发过 AS: 命令，说明对面确实不回字）")
        return 0
    print("  hex  : %s" % hx(body))
    print("  ascii: %r" % _ascii(body))
    return 0


# ─────────────────────────── 治具自检 0x0017 ───────────────────────────
# 必须与 U1/Core/Inc/app.h 的 APP_State_t **按顺序**一致
APP_STATES = ["INIT", "IDLE", "UPLOADING", "PROGRAMMING", "TESTING", "DUT",
              "ENC", "ENC_PEER", "CUR", "VOL", "RUN", "DUT_RAW"]


def do_status(link):
    print("→ 0x0017 治具在线状态")
    r = _req(link, CMD_JIG_STATUS, b"", 2.0, "0x0017")
    if r is None:
        return 1
    _, d = r
    if len(d) < 32:
        print("  应答 %d 字节（要 32）" % len(d))
        return 1
    fl = d[2]
    print("  治具状态 = %s · 应用状态 = %s" % (status_name(d[0]),
          APP_STATES[d[1]] if d[1] < len(APP_STATES) else "?%d" % d[1]))
    print("  标志 = 0x%02X（%s%s%s%s）" % (fl,
          "U2在线 " if fl & 0x01 else "U2不在 ",
          "版本一致 " if fl & 0x02 else "",
          "链路忙 " if fl & 0x04 else "",
          "**DUT链路已编入**" if fl & 0x08 else "**没有DUT链路(固件不是U1_DUT)**"))
    print("  U2 链路代码版本 = 0x%04X · 能力 = 0x%04X" % (be16(d, 3), be16(d, 5)))
    print("  探测：成功 %d 次 / 失败 %d 次" % (be16(d, 9), be16(d, 11)))
    print("  收帧 %d · CRC 错 %d · 发送超时 %d · 发送忙 %d"
          % (be32(d, 13), be32(d, 17), be16(d, 21), be16(d, 23)))
    print("  UART: 丢字节 %d · 发送错 %d" % (be16(d, 25), be16(d, 27)))
    if not (fl & 0x08):
        print()
        print("  ⚠ 这份固件没有 DUT 链路 → 0x0011/0x0014~0x001B 一律回 FF NIY。")
        print("     请烧 U1_DUT 那版（U1/MDK-ARM/U1_DUT/U1_DUT.hex）")
        return 1
    return 0


def do_run_all(link):
    print("→ 0x0010 一键全流程（进测试模式 → ②→③④→⑤→⑥ → 关输出），最长约 20 秒")
    print("   注意：子命令的应答会先到（同 SEQ、各自的命令号），最后才是本命令的汇总")
    r = _req(link, CMD_RUN_ALL, b"", 30.0, "0x0010")
    if r is None:
        return 1
    _, d = r
    if len(d) < 4:
        print("  应答 %d 字节（状态 %s）" % (len(d), status_name(d[0])))
        return 1
    steps = ["进测试模式", "②电压", "发PWM", "③脉冲采集", "④三相判定", "再进测试模式",
             "⑤清零计数", "⑤发脉冲", "⑤读U2", "⑤读被测板", "⑥相电流", "关输出"]
    print("  总判定 = %s · 步数 %d · 跑到第 %d 步 · 首个失败步 %d"
          % (status_name(d[0]), d[1], d[2], d[3]))
    stmap = {0: "未跑", 1: "OK", 2: "FAIL", 3: "超时", 4: "跳过"}
    for i in range(min(d[1], len(steps))):
        o = 4 + i * 2
        if o + 2 > len(d):
            break
        print("   %2d %-14s %-6s %s" % (i + 1, steps[i], stmap.get(d[o], "?"),
                                        status_name(d[o + 1])))
    return 0 if d[0] == 0x00 else 2


# ─────────────────────── 定位用（治具不回话时先跑这三个） ───────────────────────
def _open_serial(port, baud, parity=None, timeout=0.3):
    """打开串口；需要 8E1 时再传 parity（F1 系统 bootloader 是 8 位+偶校验）"""
    try:
        import serial
    except ImportError:
        sys.exit("需要 pyserial:  pip install pyserial")
    try:
        if parity is None:
            return serial.Serial(port, baud, timeout=timeout)
        return serial.Serial(port, baud, parity=parity, timeout=timeout)
    except Exception as exc:
        sys.exit("打不开串口 %s: %s\n"
                 "  排查: ① 端口号对不对（设备管理器）② 有没有被串口助手/Keil 占着\n"
                 "        ③ 治具有没有插 USB-TTL" % (port, exc))


def do_probe(port, bauds=(115200, 9600, 57600, 230400, 38400, 19200)):
    """同一条 0x0017 在几档波特率下各发一次 —— 先把“是不是波特率不对”排掉"""
    frame = build_frame(CMD_JIG_STATUS_RAW, 1)
    print("→ 用 0x0017 在各波特率各试一发（每次等 0.45s）:")
    print("   发：%s" % hx(frame))
    for b in bauds:
        ser = _open_serial(port, b, timeout=0.35)
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        time.sleep(0.45)
        rx = ser.read(128)
        ser.close()
        if rx:
            print("   %6d 8N1 → **收到 %d 字节**: %s" % (b, len(rx), hx(rx)))
            fr = parse_frame(rx[:10 + (rx[7] if len(rx) > 7 else 0)] if len(rx) >= 10 else rx)
            if fr is not None:
                print("            解析成功 CMD=0x%04X SEQ=%d LEN=%d → 就用 %d 8N1"
                      % (fr[0], fr[1], len(fr[2]), b))
            return 0
        print("   %6d 8N1 → 无应答" % b)
    print("\n⇒ 六档全无应答 → **不是波特率的事**。按顺序查（详见 docs/21 §5.1）:")
    print("   1. 板上是不是真的在跑应用：`isp-probe` 回 0x79 = 芯片还在 bootloader（BOOT0/H1 没拨回）")
    print("   2. USB-TTL 接在哪两颗针：必须是 **PA9/PA10（USART1）**，不是 PB10/PB11")
    print("   3. TXD/RXD 有没有交叉 + 共地")
    print("   4. 回退验证：烧回 U1_UART.hex，跑 `query-flash`。它也通不了 = 硬件/接线问题")
    return 1


def do_isp_probe(port, baud=115200):
    """STM32 系统 bootloader 握手：8E1 发 0x7F，正常应回 0x79

    为什么这条最值钱：USB 版/串口版固件**都不会**回 0x79，只有芯片进
    bootloader（BOOT0=1 且刚复位）才回。所以它能把“固件在跑但串口没通”
    和“芯片根本没跑应用”这两类原因**一刀切开**。
    """
    import serial as _s
    print("→ 8E1 @%d 发 0x7F（STM32F1 系统 bootloader 的同步字节）" % baud)
    ser = _open_serial(port, baud, parity=_s.PARITY_EVEN, timeout=0.6)
    ser.reset_input_buffer()
    ser.write(b"\x7F")
    ser.flush()
    time.sleep(0.25)
    rx = ser.read(32)
    ser.close()
    if rx[:1] == b"\x79":
        print("  收到 0x79 ✅ **芯片在 STM32 系统 bootloader 里**")
        print("  ⇒ 应用根本没在跑。做完这两件事再测：")
        print("     ① 把 H1（BOOT0）跳线拨回 **0 / GND**")
        print("     ② **按一下复位**（或断电再上电）—— 只拨跳线不复位，芯片还是待在 bootloader 里")
        return 0
    if rx:
        print("  收到 %s（不是 0x79）" % hx(rx))
        print("  ⇒ 芯片不在 bootloader。要么应用已经在跑（那就是串口/接线的问题），")
        print("     要么这个口上根本没有 MCU 在听。")
        return 1
    print("  无应答")
    print("  ⇒ 芯片**不在** bootloader（好事：应用应该已经在跑）")
    print("     那么问题在 PC ↔ 治具这一路：端口号 / 接线 / 共地 / 板上是不是这版固件")
    return 1


def do_watch(port, secs=8.0):
    """纯监听：把收到的每个字节打成 hex。配合“给治具断电再上电”看它到底有没有吐东西"""
    ser = _open_serial(port, 115200, timeout=0.2)
    print("→ 监听 %s 共 %.0f 秒。现在**给治具断电再上电**（或按复位），观察有没有字节:" % (port, secs))
    t0 = time.time()
    total = 0
    ser.reset_input_buffer()
    while (time.time() - t0) < secs:
        d = ser.read(64)
        if d:
            total += len(d)
            print("   [%5.2fs] %s" % (time.time() - t0, hx(d)))
    ser.close()
    print("→ 共收到 %d 字节" % total)
    if total == 0:
        print("   0 字节是**正常的**：U1_DUT 这版把调试打印关了（`BSP_DBG_ENABLE=0`），")
        print("   上电时不会吐任何字符。这条命令主要用来确认“串口线到底连没连上 MCU”。")
    return 0


def do_loopback(port, baud=115200):
    """USB-TTL 自环：把适配器自己的 TX 和 RX 短接，发什么就该收到什么。

    为什么需要它：治具不回话时，问题可能在**适配器这一侧**（坏的线/坏的适配器/
    COM 号其实是别的设备）。这一步把“适配器 + 线 + COM 号”单独证明一遍，
    剩下的嫌疑就只能在治具板那一段。
    """
    print("→ 自环测试前先把 **适配器自己的 TXD 和 RXD 短接**（拔掉接治具的那两根，只短接适配器这头）")
    print("   现在开始发 8 个字节...")
    ser = _open_serial(port, baud, timeout=0.5)
    ser.reset_input_buffer()
    payload = bytes([0x55, 0xAA, 0x00, 0xFF, 0x12, 0x34, 0x56, 0x78])
    ser.write(payload)
    ser.flush()
    time.sleep(0.2)
    rx = ser.read(32)
    ser.close()
    print("   发：%s" % hx(payload))
    print("   收：%s" % (hx(rx) if rx else "(无)"))
    if rx[:len(payload)] == payload:
        print("   ✅ 完全一致 → **适配器、线、COM 号都没问题**，问题在治具板那一段")
        return 0
    if rx:
        print("   ⚠ 收到东西但对不上 → 多半是波特率/编码问题，或这个 COM 是别的设备")
        return 1
    print("   ❌ 一个字节都没回来。两种可能：")
    print("      ① 没短接（TXD/RXD 还是接在治具上）→ 那就先把它们短接再跑一次")
    print("      ② 短接了还是没回 → **适配器坏了 / COM 号是别的设备**，换一个适配器或换根线")
    return 1


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
                                       "query-foc", "upload", "program", "raw",
                                       # ── 被测板五项功能（要求固件是 JIG_DUT_LINK=1，即 target U1_DUT）──
                                       "status", "dut-ping", "dut-info",
                                       "vol", "pulse", "phase", "enc", "cur",
                                       "run-all",
                                       # ── 定位用（治具不回话时先跑这三个）──
                                       "probe", "isp-probe", "watch", "loopback",
                                       # ── DUT 口原始字节（对面还没实现 0x02xx 时用）──
                                       "as-start", "as-stop", "as-send", "dut-recv"])
    ap.add_argument("arg", nargs="?", help="upload: 固件文件; raw: 十六进制字节")
    ap.add_argument("-p", "--port", help="串口, 如 COM5")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="形式波特率 (USB CDC 忽略)")
    ap.add_argument("-v", "--version", default="1.0.0", help="上传固件的版本号 主.次.修订")
    ap.add_argument("--boot", help="burn: Bootloader 固件文件 (写入 Slot0)")
    ap.add_argument("--app", help="burn: App 固件文件 (写入 Slot1)")
    ap.add_argument("--boot-ver", help="burn: Bootloader 版本号 (默认同 -v)")
    ap.add_argument("--app-ver", help="burn: App 版本号 (默认同 -v)")
    # cur 用：相、占空比、时长、通流模式
    ap.add_argument("--hi", default="A", help="cur: 上桥相 A/B/C (默认 A)")
    ap.add_argument("--lo", default="B", help="cur: 下桥相 A/B/C (默认 B)")
    ap.add_argument("--duty", type=int, default=20, help="cur: 占空比千分比 (默认 20 = 2%%)")
    ap.add_argument("--ms", type=int, default=100, help="cur: 通流时长 ms (默认 100)")
    ap.add_argument("--eol", default="none", choices=["none", "crlf", "lf", "cr"],
                    help="as-send/as-start/as-stop: 结尾加什么行结束符 (默认 none；"
                         "多数 ASCII 命令协议需要 crlf)")
    ap.add_argument("--tmo", type=int, default=200,
                    help="as-send/as-start/as-stop: 等对端回字的毫秒数 (默认 200, 上限 2000)")
    ap.add_argument("--secs", type=float, default=8.0,
                    help="watch: 监听多少秒 (默认 8)")
    ap.add_argument("--curmode", type=int, default=1, choices=[0, 1],
                    help="cur: 0=静态全通(危险, 会忽略占空比) 1=低频PWM(默认, 小占空比安全)")
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

    # ── 定位用：这三个自己开串口，不走 JigLink ──
    if a.action == "probe":     return do_probe(a.port)
    if a.action == "isp-probe": return do_isp_probe(a.port)
    if a.action == "watch":     return do_watch(a.port, a.secs)
    if a.action == "loopback":  return do_loopback(a.port)

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
    # ── 被测板五项功能 ──
    if a.action == "status":    return do_status(link)
    if a.action == "dut-ping":  return do_dut_ping(link)
    if a.action == "dut-info":  return do_dut_info(link)
    if a.action == "vol":       return do_vol(link)
    if a.action == "pulse":     return do_pulse(link)
    if a.action == "phase":     return do_phase(link)
    if a.action == "enc":       return do_enc(link)
    if a.action == "cur":
        return do_cur(link, hi=_phase_bit(a.hi), lo=_phase_bit(a.lo),
                      duty=a.duty, ms=a.ms, mode=a.curmode)
    if a.action == "run-all":   return do_run_all(link)
    # ── DUT 口原始字节（FOC 板现有 ASCII 命令的过渡验证）──
    if a.action == "as-start":  return do_dut_send(link, "AS:S800", a.tmo,
                                                   "AS:S800（转）", a.eol)
    if a.action == "as-stop":   return do_dut_send(link, "AS:S0",   a.tmo,
                                                   "AS:S0（停）", a.eol)
    if a.action == "as-send":
        if not a.arg:
            print("as-send 需要要发的文本，例如： as-send AS:S800")
            return 2
        return do_dut_send(link, a.arg, a.tmo, "自定义", a.eol)
    if a.action == "dut-recv":  return do_dut_recv(link)
    return 2


if __name__ == "__main__":
    sys.exit(main())
