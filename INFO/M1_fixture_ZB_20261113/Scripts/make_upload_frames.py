#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_upload_frames.py — 把 boot/app 的 .bin 打包成 M1 治具的上传命令帧

协议（与 data_process.c 逐字节一致）：
    A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA(LEN) | CRC16(2)      全部大端
    CRC-16/MODBUS（初值 0xFFFF、多项式 0xA001），范围 A5…DATA 末字节

生成（默认放在 临时测试固件/ 下）：
    boot_frames.bin        Slot0 上传流：START + DATA×N + END
    app_frames.bin         Slot1 上传流：START + DATA×N + END
    upload_all.bin         boot + app 连在一起（串口助手一个文件发完两个槽）
    upload_and_burn.bin    upload_all + 0x0013 自检 + 0x0012 烧录
    cmd_hex.txt            小命令的十六进制，可直接粘到串口助手

用法：
    python make_upload_frames.py
    python make_upload_frames.py --ver 1.2.3
    python make_upload_frames.py --boot 临时测试固件/xxx.bin --app 临时测试固件/yyy.bin

⚠ 串口助手一次“发送文件”时，主机可能比治具写 SPI Flash 快得多，
   接收队列（32 包 ≈ 2KB）会溢出丢包。可靠做法见 cmd_hex.txt 末尾的说明：
   要么给串口助手设“每包间隔”，要么用 Scripts/fw_upload_test.py（等 ACK）。
"""
import argparse
import os
import sys

HDR0, HDR1 = 0xA5, 0x5B

CMD_FW_START = 0x0009
CMD_FW_DATA = 0x000A
CMD_FW_END = 0x000B
CMD_QUERY_FLASH_VER = 0x000C
CMD_SWD_PROGRAM = 0x0012
CMD_SWD_PROBE = 0x0013
CMD_SWD_STATUS = 0x0014

CHUNK = 256          # 单帧 DATA 上限
SLOT_SIZE = 128 * 1024
FW_MAX_IMAGE = SLOT_SIZE - 8


def crc16(data: bytes) -> int:
    """CRC-16/MODBUS：初值 0xFFFF，反射多项式 0xA001"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def frame(cmd: int, seq: int, data: bytes = b"") -> bytes:
    body = (bytes([HDR0, HDR1])
            + cmd.to_bytes(2, "big")
            + (seq & 0xFFFF).to_bytes(2, "big")
            + len(data).to_bytes(2, "big")
            + data)
    return body + crc16(body).to_bytes(2, "big")


def image_stream(img: bytes, ver: bytes, seq0: int = 1):
    """一个镜像的完整上传流：START + DATA... + END"""
    n_frames = (len(img) + CHUNK - 1) // CHUNK
    out = bytearray()
    seq = seq0

    start = n_frames.to_bytes(2, "big") + len(img).to_bytes(4, "big") + ver
    out += frame(CMD_FW_START, seq, start)
    seq += 1

    for i in range(n_frames):
        out += frame(CMD_FW_DATA, seq, img[i * CHUNK:(i + 1) * CHUNK])
        seq += 1

    out += frame(CMD_FW_END, seq)
    seq += 1
    return bytes(out), seq, n_frames


def verify_stream(stream: bytes) -> int:
    """按协议重新解析一遍，校验 CRC 与长度自洽；返回帧数，失败抛异常"""
    pos = 0
    n = 0
    while pos < len(stream):
        if stream[pos:pos + 2] != bytes([HDR0, HDR1]):
            raise ValueError("帧头错位 @%d" % pos)
        cmd = int.from_bytes(stream[pos + 2:pos + 4], "big")
        dlen = int.from_bytes(stream[pos + 6:pos + 8], "big")
        end = pos + 8 + dlen
        if end + 2 > len(stream):
            raise ValueError("帧不完整 @%d" % pos)
        calc = crc16(stream[pos:end])
        recv = int.from_bytes(stream[end:end + 2], "big")
        if calc != recv:
            raise ValueError("CRC 错 @%d (cmd=0x%04X)" % (pos, cmd))
        pos = end + 2
        n += 1
    return n


def hexline(b: bytes) -> str:
    return " ".join("%02X" % x for x in b)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="临时测试固件", help="bin 所在目录（默认 临时测试固件）")
    ap.add_argument("--boot", default=None, help="boot bin 路径（默认自动找 bootloader）")
    ap.add_argument("--app", default=None, help="app bin 路径（默认自动找 shredder）")
    ap.add_argument("--ver", default="1.0.0", help="版本号 主.次.修订（默认 1.0.0）")
    args = ap.parse_args()

    ver = bytes(int(x) & 0xFF for x in args.ver.split("."))
    if len(ver) != 3:
        print("版本号要写成 主.次.修，如 1.0.0")
        return 1

    boot_path = args.boot
    app_path = args.app
    if boot_path is None or app_path is None:
        # 排除本脚本自己生成的文件，避免把 *_frames.bin 当成固件再套一层协议
        generated = ("_frames.bin", "_data.bin")
        cands = sorted(f for f in os.listdir(args.dir)
                       if f.lower().endswith(".bin")
                       and not f.lower().endswith(generated)
                       and not f.lower().startswith("upload"))
        for f in cands:
            low = f.lower()
            if boot_path is None and "boot" in low:
                boot_path = os.path.join(args.dir, f)
            elif app_path is None and "boot" not in low:
                app_path = os.path.join(args.dir, f)
    if not boot_path or not app_path:
        print("找不到 boot/app bin，请用 --boot/--app 指定")
        return 1

    with open(boot_path, "rb") as fp:
        boot = fp.read()
    with open(app_path, "rb") as fp:
        app = fp.read()

    print("boot : %s  %d 字节" % (os.path.basename(boot_path), len(boot)))
    print("app  : %s  %d 字节" % (os.path.basename(app_path), len(app)))

    for name, img, limit in (("boot", boot, 0xA000), ("app", app, 0x16000)):
        if len(img) == 0:
            print("  ✗ %s 是空文件" % name)
            return 1
        if len(img) > FW_MAX_IMAGE:
            print("  ✗ %s 超过槽容量 %d（%d）" % (name, FW_MAX_IMAGE, len(img)))
            return 1
        if len(img) > limit:
            print("  ✗ %s 超过目标分区 %d（%d）" % (name, limit, len(img)))
            return 1
        if len(img) % 4:
            print("  ! %s 长度不是 4 的倍数（%d），尾部会按 0xFF 补齐" % (name, len(img) % 4))

    boot_stream, seq, boot_frames = image_stream(boot, ver, 1)
    app_stream, seq, app_frames = image_stream(app, ver, seq)
    all_stream = boot_stream + app_stream

    probe = frame(CMD_SWD_PROBE, seq)
    seq += 1
    program = frame(CMD_SWD_PROGRAM, seq)
    seq += 1
    burn_stream = all_stream + probe + program

    for name, blob in (("boot_frames.bin", boot_stream),
                       ("app_frames.bin", app_stream),
                       ("upload_all.bin", all_stream),
                       ("upload_and_burn.bin", burn_stream)):
        path = os.path.join(args.dir, name)
        with open(path, "wb") as fp:
            fp.write(blob)
        n = verify_stream(blob)
        print("  ✓ %-20s %7d 字节, %3d 帧（CRC 自检通过）" % (name, len(blob), n))

    # ---- 分两批（每批 3 条命令：START / DATA 文件 / END）----
    def split_parts(img, ver):
        n = (len(img) + CHUNK - 1) // CHUNK
        start = frame(CMD_FW_START, 1,
                      n.to_bytes(2, "big") + len(img).to_bytes(4, "big") + ver)
        data = b"".join(frame(CMD_FW_DATA, 2 + i, img[i * CHUNK:(i + 1) * CHUNK])
                        for i in range(n))
        end = frame(CMD_FW_END, 2 + n)
        return start, data, end, n

    parts = {}
    for name, img, tag in (("Boot", boot, "boot"), ("App", app, "app")):
        start, data, end, n = split_parts(img, ver)
        parts[tag] = (start, data, end, n)
        with open(os.path.join(args.dir, tag + "_data.bin"), "wb") as fp:
            fp.write(data)
        verify_stream(data)
        print("  ✓ %-20s %7d 字节, %3d 个数据帧（CRC 自检通过）"
              % (tag + "_data.bin", len(data), n))

    plan = []
    plan.append("# 用串口助手分两批上传（必须同一次上电，先 Boot 后 App）")
    plan.append("# 每批 3 条命令：[1] 粘 START 的 HEX  [2] 发送 DATA 文件  [3] 粘 END 的 HEX")
    plan.append("# 波特率随意（USB CDC 虚拟串口）；每帧回一帧应答，其中 DATA 里 1 字节 00 = OK")
    plan.append("# 发文件必须限速（每 256B >= 5ms），否则会丢包 -> END 回 ERR_COUNT")
    plan.append("")
    for i, (name, tag) in enumerate((("Boot -> Slot0", "boot"), ("App -> Slot1", "app")), 1):
        start, data, end, n = parts[tag]
        plan.append("=" * 60)
        plan.append("# 第 %d 批：%s" % (i, name))
        plan.append("=" * 60)
        plan.append("[1] START（直接按 HEX 发送）")
        plan.append(hexline(start))
        plan.append("")
        plan.append("[2] 发送文件：%s_data.bin  （%d 字节，%d 个数据帧）" % (tag, len(data), n))
        plan.append("")
        plan.append("[3] END（直接按 HEX 发送）")
        plan.append(hexline(end))
        plan.append("")
    plan.append("=" * 60)
    plan.append("# 传完后：发 cmd_hex.txt 里的 0x0012")
    plan.append("#   立即回 06/07/02 = 槽不完整（没碰目标）；没有立即错误 = 已在烧录")
    plan.append("# 中途失败：给治具断电重上电，从第 1 批重来（槽指针会回到 Slot0）")
    with open(os.path.join(args.dir, "send_plan.txt"), "w", encoding="utf-8") as fp:
        fp.write("\n".join(plan) + "\n")
    print("  ✓ send_plan.txt          两批 START/END 的 HEX 与操作顺序")

    # 小命令的十六进制（可直接粘到串口助手）
    lines = []
    lines.append("# M1 治具 SWD 命令帧（十六进制，直接按 HEX 发送）")
    lines.append("# 说明：一帧发完治具回一帧；0x0012 要等烧完才回（20~65 秒）")
    lines.append("")
    lines.append("# 0x0013 SWD 自检（不擦不写）")
    lines.append(hexline(frame(CMD_SWD_PROBE, 1)))
    lines.append("")
    lines.append("# 0x0013 SWD 自检 + 把延时设成 32（≈250kHz 档；越大越慢）")
    lines.append(hexline(frame(CMD_SWD_PROBE, 1, bytes([0x00, 0x20]))))
    lines.append("")
    lines.append("# 0x0012 SWD 烧录（把 Slot0/Slot1 写到目标；ACK 在烧完才回）")
    lines.append(hexline(frame(CMD_SWD_PROGRAM, 1)))
    lines.append("")
    lines.append("# 0x0014 烧录进度：状态(1) 结果(1) 已处理(4) 总量(4)")
    lines.append(hexline(frame(CMD_SWD_STATUS, 1)))
    lines.append("")
    lines.append("# 0x000C 查治具 SPI Flash 里的固件版本")
    lines.append(hexline(frame(CMD_QUERY_FLASH_VER, 1)))
    lines.append("")
    lines.append("# ⚠ 用串口助手“发送文件”烧 boot_frames.bin / app_frames.bin 时：")
    lines.append("#   治具写 SPI Flash 约 3ms/256B，主机一次猛发会冲掉接收队列（32 包≈2KB）。")
    lines.append("#   请把串口助手的“发送间隔”设成 ≥ 5ms/256B（≈ 50KB/s 以下）；")
    lines.append("#   或者改用 Scripts/fw_upload_test.py（每帧等 ACK，最稳）。")
    lines.append("#   发完用 0x000C 查版本：状态 0x00 才算两个槽都完整。")
    with open(os.path.join(args.dir, "cmd_hex.txt"), "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines) + "\n")
    print("  ✓ cmd_hex.txt           小命令十六进制")

    print("")
    print("汇总：boot %d 帧 + app %d 帧；upload_all %d 字节；upload_and_burn %d 字节"
          % (boot_frames + 2, app_frames + 2, len(all_stream), len(burn_stream)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
