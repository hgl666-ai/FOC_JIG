#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把"上传到 SPI Flash → 启动烧录"的整串帧导出来，供串口助手手工发送。

用途：没有 Python 环境、或想一步步看着屏幕发帧的时候用。

    python tools\\gen_upload_seq.py --boot 固件\\boot.bin --app 固件\\app1.bin -v 1.2.3
    python tools\\gen_upload_seq.py --loopback          # 用 test/fixtures 的 68 字节最小件

输出：帧清单（发给谁、期望回什么）+ 一个 .txt 文件（每行一帧，可逐行复制）。

⚠ 三个必须知道的点
  1. 槽位靠**上传顺序**决定，不是靠命令字段：`FwUpload` 每收到一次结束帧就把
     upload_slot 翻转一次（0→1→0…）。所以**必须先 Boot 再 App**；
     只传单独一个镜像会落到错误的槽里。
  2. 每帧之间要等治具的应答（正常 10ms 内）；同一帧重发时序号可以不变（治具只告警不拒绝）。
  3. `0x0112` 要等 **20~25 秒** 才回，别以为卡死了。
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from foc_jig_tool import build_frame, hx                      # noqa: E402

CHUNK = 256                                # 每帧最多 256 字节（协议上限）


def frames_for(kind, blob, ver, seq):
    """生成某个镜像的三帧（起始 / 数据×N / 结束）"""
    out = []
    n = len(blob)
    nframes = (n + CHUNK - 1) // CHUNK
    data = bytes([(nframes >> 8) & 0xFF, nframes & 0xFF,
                  (n >> 24) & 0xFF, (n >> 16) & 0xFF, (n >> 8) & 0xFF, n & 0xFF,
                  ver[0], ver[1], ver[2]])
    out.append((seq, "%s 起始帧" % kind, 0x0009, data, bytes([0x00])))
    seq += 1
    for i in range(nframes):
        out.append((seq, "%s 数据帧 %d/%d" % (kind, i + 1, nframes),
                    0x000A, blob[i * CHUNK:(i + 1) * CHUNK], bytes([0x00])))
        seq += 1
    out.append((seq, "%s 结束帧" % kind, 0x000B, b"", bytes([0x00])))
    seq += 1
    return out, seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--boot")
    ap.add_argument("--app")
    ap.add_argument("--loopback", action="store_true",
                    help="用 test/fixtures/min_boot.bin + min_app.bin（各 68 字节，仅 8 帧）")
    ap.add_argument("-v", "--version", default="1.0.0")
    ap.add_argument("-o", "--out", default=None)
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    if a.loopback:
        boot_p = os.path.join(root, "test", "fixtures", "min_boot.bin")
        app_p = os.path.join(root, "test", "fixtures", "min_app.bin")
        boot_v, app_v = (1, 0, 0), (9, 9, 9)
    else:
        if not (a.boot and a.app):
            ap.error("要么 --loopback，要么同时给 --boot 和 --app")
        boot_p, app_p = a.boot, a.app
        vv = [int(x) for x in a.version.split(".")]
        boot_v = app_v = (vv[0] & 0xFF, vv[1] & 0xFF, vv[2] & 0xFF)

    boot = open(boot_p, "rb").read()
    app = open(app_p, "rb").read()

    rows, seq = frames_for("Boot", boot, boot_v, 1)
    r2, seq = frames_for("App", app, app_v, seq)
    rows += r2
    rows.append((seq, "启动烧录（等 20~25 秒）", 0x0112, b"", bytes([0x00])))
    seq += 1
    rows.append((seq, "读目标版本（可选）", 0x000D, b"", bytes([0x00, app_v[0], app_v[1], app_v[2]])))

    lines = []
    print("Boot: %s (%d 字节)" % (os.path.basename(boot_p), len(boot)))
    print("App : %s (%d 字节)" % (os.path.basename(app_p), len(app)))
    print("共 %d 帧\n" % len(rows))

    for (sq, desc, cmd, data, exp) in rows:
        tx = hx(build_frame(cmd, sq, data))
        rx = hx(build_frame(cmd, sq, exp))
        print("%-28s 发: %s" % (desc, tx))
        print("%-28s 回: %s" % ("", rx))
        lines.append(tx)

    out = a.out or os.path.join(root, "burn_frames.txt")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print("\n帧清单已写到: %s" % out)


if __name__ == "__main__":
    main()
