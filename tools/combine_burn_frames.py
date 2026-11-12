#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把分散的帧文件合成为"每个镜像一套"的形式，并重新做逐字节核对。

产出（固件\\最终\\上传帧\\）：
    boot_上传.bin       = 起始 + 25 数据 + 结束   （一个文件装完，见下方警告）
    boot_起始.bin       = 起始帧（19 字节）
    boot_数据加结束.bin = 25 数据 + 结束

    app1_上传.bin       = 起始 + 99 数据 + 结束
    app1_起始.bin       = 起始帧（19 字节）
    app1_数据加结束.bin = 99 数据 + 结束

⚠ 为什么要拆出"起始"这一份：
   `0x0009` 收到后会**同步擦掉整个槽的扇区**再回 ACK：
       Boot 6319 B → 2 个扇区 → 0.1~0.8 秒
       App  25184 B → 7 个扇区 → 0.3~2.8 秒
   而治具只有 256 B(UART) + 512 B(帧) 的缓冲；115200 下 0.8 秒就涌进约 9 KB →
   必丢。所以**烧一个文件"一次点发送"不行**，必须在起始帧之后停一下。
   两种可行用法：
     A) 串口助手"发送文件"能设间隔：分两次点（起始 → 停 1~3 秒 → 数据加结束）
     B) 工具：foc_jig_tool.py burn（每帧都等 ACK，最稳）
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

D = os.path.join(os.path.dirname(HERE), "固件", "最终", "上传帧")


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
    return c


def parse_all(buf):
    """把字节流重新解析成帧，返回 (帧列表, 错误)"""
    i, out = 0, []
    while i + 10 <= len(buf):
        if buf[i] != 0xA5 or buf[i + 1] != 0x5B:
            return None, "偏移 %d 不是帧头" % i
        ln = (buf[i + 6] << 8) | buf[i + 7]
        if crc16(buf[i:i + 8 + ln]) != ((buf[i + 8 + ln] << 8) | buf[i + 9 + ln]):
            return None, "偏移 %d 的帧 CRC 错" % i
        out.append(((buf[i + 2] << 8) | buf[i + 3],
                    (buf[i + 4] << 8) | buf[i + 5],
                    bytes(buf[i + 8:i + 8 + ln])))
        i += 8 + ln + 2
    if i != len(buf):
        return None, "尾部有 %d 字节没解析" % (len(buf) - i)
    return out, None


def load(name):
    return open(os.path.join(D, name), "rb").read()


def combine(outname, parts):
    buf = b"".join(parts)
    p = os.path.join(D, outname)
    open(p, "wb").write(buf)
    frames, err = parse_all(buf)
    return p, len(buf), (len(frames) if frames else 0), err


def main():
    boot_src = open(os.path.join(os.path.dirname(D), "boot.bin"), "rb").read()
    app_src = open(os.path.join(os.path.dirname(D), "app1.bin"), "rb").read()

    b_start = load("01_boot_start.bin")
    b_data = load("02_boot_data.bin")
    b_end = load("03_boot_end.bin")
    a_start = load("04_app_start.bin")
    a_data = load("05_app_data.bin")
    a_end = load("06_app_end.bin")

    jobs = [
        ("boot_起始.bin", [b_start]),
        ("boot_数据加结束.bin", [b_data, b_end]),
        ("boot_上传.bin", [b_start, b_data, b_end]),
        ("app1_起始.bin", [a_start]),
        ("app1_数据加结束.bin", [a_data, a_end]),
        ("app1_上传.bin", [a_start, a_data, a_end]),
    ]

    print("=== 合成 ===")
    ok = True
    for name, parts in jobs:
        p, size, nf, err = combine(name, parts)
        tag = "✅" if err is None else "❌ " + err
        print("%-22s %6d 字节  %3d 帧  %s" % (name, size, nf, tag))
        ok = ok and err is None

    # ── 载荷级核对：合成文件里的数据帧拼起来必须等于源 bin ──
    print("\n=== 载荷核对 ===")
    for name, src in [("boot_上传.bin", boot_src), ("app1_上传.bin", app_src),
                      ("boot_数据加结束.bin", boot_src), ("app1_数据加结束.bin", app_src)]:
        buf = load(name)
        frames, err = parse_all(buf)
        assert err is None, err
        payload = b"".join(d for (cmd, sq, d) in frames if cmd == 0x000A)
        same = (payload == src)
        print("%-22s 数据帧载荷 %6d 字节  == 源文件? %s" % (name, len(payload), "✅" if same else "❌"))
        ok = ok and same

    # ── 帧序核对：必须是 起始 → 数据… → 结束 ──
    print("\n=== 帧序核对 ===")
    for name in ["boot_上传.bin", "app1_上传.bin"]:
        frames, err = parse_all(load(name))
        assert err is None, err
        cmds = [c for (c, s, d) in frames]
        good = (cmds[0] == 0x0009 and cmds[-1] == 0x000B
                and all(c == 0x000A for c in cmds[1:-1]))
        print("%-22s 首=%04X 末=%04X 中间数据帧 %d 帧  %s" %
              (name, cmds[0], cmds[-1], len(cmds) - 2, "✅" if good else "❌"))
        ok = ok and good

    print("\n" + ("全部核对通过" if ok else "有失败项，见上"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
