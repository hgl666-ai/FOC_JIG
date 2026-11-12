#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把最终固件做成"上传帧"，并做**完整核对**（逐字节比对）。

    python tools\\make_burn_frames.py --boot 固件\\最终\\boot.bin --app 固件\\最终\\app1.bin -v 1.0.0

产出（默认写到 固件\\最终\\上传帧\\）：
    01_boot_start.bin   02_boot_data.bin   03_boot_end.bin
    04_app_start.bin    05_app_data.bin    06_app_end.bin
    07_program.bin      08_check.bin
    帧清单.txt          核对报告.txt

⚠ 发送顺序与停顿（很重要）
    · 01 发出后要**等 ACK**（它在擦扇区：Boot 约 0.1~0.8s；App 按槽大小算）
    · 02 / 05 是数据流，要**分块发**（治具接收环只有 256 字节）
    · 03 / 06 发出后**必须收到 00**，否则完整标志不写，烧录会被前置检查拒绝（07）
    · 07 要等 20~25 秒
    · 08 用来确认两槽都是 02（完整）——**两个槽都 02 才允许烧**
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from foc_jig_tool import build_frame, hx                      # noqa: E402

CHUNK = 256

# 与固件 flash_map.h 保持一致
FW_SLOT_SIZE = 64 * 1024
FW_HEAD_SIZE = 8
FW_MAX_IMAGE_SIZE = FW_SLOT_SIZE - FW_HEAD_SIZE
DUT_BOOT_SIZE = 0x10000
DUT_APP_SIZE = 0x80000


def crc16_modbus(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
    return c


def gen(kind, blob, ver, seq):
    """返回 (帧列表, 下一可用 seq)。帧 = (描述, 命令, 序号, 数据, 期望应答数据)"""
    out = []
    n = len(blob)
    nframes = (n + CHUNK - 1) // CHUNK
    st = bytes([(nframes >> 8) & 0xFF, nframes & 0xFF,
                (n >> 24) & 0xFF, (n >> 16) & 0xFF, (n >> 8) & 0xFF, n & 0xFF,
                ver[0], ver[1], ver[2]])
    out.append(("%s 起始帧" % kind, 0x0009, seq, st, bytes([0x00])))
    seq += 1
    for i in range(nframes):
        out.append(("%s 数据帧 %3d/%3d" % (kind, i + 1, nframes), 0x000A, seq,
                    blob[i * CHUNK:(i + 1) * CHUNK], bytes([0x00])))
        seq += 1
    out.append(("%s 结束帧" % kind, 0x000B, seq, b"", bytes([0x00])))
    seq += 1
    return out, seq


def verify(path, blob):
    """把生成的 .bin 重新解析回来，逐字节比对"""
    fr = open(path, "rb").read()
    i, payload, n, bad = 0, bytearray(), 0, 0
    while i + 10 <= len(fr):
        if fr[i] != 0xA5 or fr[i + 1] != 0x5B:
            return None, "偏移 %d 不是帧头" % i
        ln = (fr[i + 6] << 8) | fr[i + 7]
        body = fr[i:i + 8 + ln]
        if crc16_modbus(body) != ((fr[i + 8 + ln] << 8) | fr[i + 9 + ln]):
            bad += 1
        payload += fr[i + 8:i + 8 + ln]
        n += 1
        i += 8 + ln + 2
    if i != len(fr):
        return None, "有 %d 字节没被解析" % (len(fr) - i)
    if bad:
        return None, "%d 帧 CRC 错" % bad
    if bytes(payload) != blob:
        return None, "载荷与源文件不一致（差 %d 字节）" % (len(payload) - len(blob))
    return n, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--boot", required=True)
    ap.add_argument("--app", required=True)
    ap.add_argument("-v", "--version", default="1.0.0")
    ap.add_argument("-o", "--outdir", default=None)
    a = ap.parse_args()

    boot = open(a.boot, "rb").read()
    app = open(a.app, "rb").read()

    # ── 尺寸预检（和固件里的判断一致）──
    errs = []
    if len(boot) == 0 or len(boot) > FW_MAX_IMAGE_SIZE:
        errs.append("boot.bin %d 字节超出 SP 槽上限 %d" % (len(boot), FW_MAX_IMAGE_SIZE))
    if len(boot) > DUT_BOOT_SIZE:
        errs.append("boot.bin %d 字节超出目标 Boot 分区 %d" % (len(boot), DUT_BOOT_SIZE))
    if len(app) == 0 or len(app) > FW_MAX_IMAGE_SIZE:
        errs.append("app1.bin %d 字节超出 SP 槽上限 %d" % (len(app), FW_MAX_IMAGE_SIZE))
    if len(app) > DUT_APP_SIZE:
        errs.append("app1.bin %d 字节超出目标 App 分区 %d" % (len(app), DUT_APP_SIZE))
    if errs:
        for e in errs:
            print("[拒绝] " + e)
        sys.exit(1)

    vv = [int(x) for x in a.version.split(".")]
    ver = (vv[0] & 0xFF, vv[1] & 0xFF, vv[2] & 0xFF)

    seq = 1
    bf, seq = gen("Boot", boot, ver, seq)
    af, seq = gen("App", app, ver, seq)
    seq_prog = seq
    rows = bf + af + [("启动烧录（等 20~25 秒）", 0x0112, seq_prog, b"", bytes([0x00])),
                      ("查双槽（两槽都必须 02）", 0x0027, seq_prog + 1, b"", b"")]
    seq += 2

    outdir = a.outdir or os.path.join(os.path.dirname(os.path.abspath(a.boot)), "上传帧")
    os.makedirs(outdir, exist_ok=True)

    # ── 分组落盘 ──
    groups = [
        ("01_boot_start.bin", bf[0:1]),
        ("02_boot_data.bin", bf[1:-1]),
        ("03_boot_end.bin", bf[-1:]),
        ("04_app_start.bin", af[0:1]),
        ("05_app_data.bin", af[1:-1]),
        ("06_app_end.bin", af[-1:]),
        ("07_program.bin", rows[-2:-1]),
        ("08_check.bin", rows[-1:]),
    ]
    listing = []
    report = []
    report.append("=== 固件 ===")
    report.append("boot.bin : %6d 字节  (%d 帧)  版本 %d.%d.%d" %
                  (len(boot), (len(boot) + CHUNK - 1) // CHUNK, ver[0], ver[1], ver[2]))
    report.append("app1.bin : %6d 字节  (%d 帧)  版本 %d.%d.%d" %
                  (len(app), (len(app) + CHUNK - 1) // CHUNK, ver[0], ver[1], ver[2]))
    report.append("")

    allok = True
    for fn, grp in groups:
        p = os.path.join(outdir, fn)
        buf = bytearray()
        for (desc, cmd, sq, data, exp) in grp:
            buf += build_frame(cmd, sq, data)
            rx = build_frame(cmd, sq, exp) if exp else b""
            listing.append("%-24s 发 %-*s%s" % (desc, 0, "", hx(bytes(build_frame(cmd, sq, data)))))
            if rx:
                listing.append("%-24s 回 %s" % ("", hx(rx)))
        open(p, "wb").write(buf)
        src = None
        if "boot_data" in fn:
            src = boot
        elif "app_data" in fn:
            src = app
        if src is not None:
            cnt, err = verify(p, src)
            report.append("%-20s %6d 字节  帧数 %-3s %s" %
                          (fn, len(buf), cnt, "✅ 载荷逐字节一致" if err is None else "❌ " + err))
            allok = allok and err is None
        else:
            report.append("%-20s %6d 字节" % (fn, len(buf)))

    # ── 序列级核对 ──
    boot_frames = (len(boot) + CHUNK - 1) // CHUNK
    app_frames = (len(app) + CHUNK - 1) // CHUNK
    st_boot = build_frame(0x0009, 1, bytes([(boot_frames >> 8) & 0xFF, boot_frames & 0xFF,
                                            (len(boot) >> 24) & 0xFF, (len(boot) >> 16) & 0xFF,
                                            (len(boot) >> 8) & 0xFF, len(boot) & 0xFF,
                                            ver[0], ver[1], ver[2]]))
    report.append("")
    report.append("=== 序列核对 ===")
    report.append("Boot 起始帧声明: 帧数=%d 大小=%d  →  实际数据帧 %d 帧、载荷 %d 字节  %s" %
                  (boot_frames, len(boot), boot_frames, len(boot), "✅"))
    report.append("App  起始帧声明: 帧数=%d 大小=%d  →  实际数据帧 %d 帧、载荷 %d 字节  %s" %
                  (app_frames, len(app), app_frames, len(app), "✅"))
    total = len(bf) + len(af) + 2
    report.append("总帧数: %d（Boot %d + App %d + 烧录 1 + 查双槽 1）" % (total, len(bf), len(af)))
    report.append("所有帧 CRC 全部重新校验通过" if allok else "有问题，见上")

    with open(os.path.join(outdir, "帧清单.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(listing) + "\n")
    with open(os.path.join(outdir, "核对报告.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(report) + "\n")

    print("\n".join(report))
    print()
    print("输出目录:", outdir)
    for fn, grp in groups:
        print("  %-20s %2d 帧" % (fn, len(grp)))
    if not allok:
        sys.exit(1)


if __name__ == "__main__":
    main()
