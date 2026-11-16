#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fw_upload_test.py —— M1 治具固件上传联调脚本（与 FOC_JIG 协议逐字节相同）

用法：
    python fw_upload_test.py COM7 app.bin --ver 1.2.3
    python fw_upload_test.py COM7 boot.bin --ver 1.0.0 --no-query
    python fw_upload_test.py COM7 --query-only

说明：
  * 治具上电后槽位从 Slot0(Bootloader) 开始，每收到一次 FW_END 切槽。
    所以"先传 boot、再传 app"要**同一次上电会话**里连续运行两次本脚本
    （第一次传 boot.bin，第二次传 app.bin），中途不要给治具断电。
  * START/END 的应答较慢（START 要擦除扇区，最大约 1 秒），超时按 10s 处理。
  * 上电后建议先 --query-only 确认通讯正常。

依赖：pip install pyserial
"""
import argparse
import math
import sys
import time

try:
    import serial
except ImportError:
    print("需要 pyserial:  pip install pyserial")
    sys.exit(1)

HDR0, HDR1 = 0xA5, 0x5B
CMD_FW_START = 0x0009
CMD_FW_DATA = 0x000A
CMD_FW_END = 0x000B
CMD_QUERY_FLASH_VER = 0x000C
FRAME_MAX_DATA = 256

STATUS = {
    0x00: "OK",
    0x01: "BUSY",
    0x02: "ERR_SIZE",
    0x03: "ERR_FLASH",
    0x06: "ERR_NO_FW",
    0x07: "ERR_INCOMPLETE",
    0x0A: "ERR_COUNT",
    0x0C: "ERR_TIMEOUT",
    0x0D: "ERR_OVERFLOW",
    0xFF: "ERR_NIY",
}


def crc16(data: bytes) -> int:
    """CRC-16/MODBUS: 初值 0xFFFF, 反射多项式 0xA001"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc


def build_frame(cmd: int, seq: int, data: bytes = b"") -> bytes:
    body = bytes([
        HDR0, HDR1,
        (cmd >> 8) & 0xFF, cmd & 0xFF,
        (seq >> 8) & 0xFF, seq & 0xFF,
        (len(data) >> 8) & 0xFF, len(data) & 0xFF,
    ]) + data
    c = crc16(body)
    return body + bytes([(c >> 8) & 0xFF, c & 0xFF])


def _read_exact(ser: serial.Serial, n: int, deadline: float) -> bytes:
    """在 deadline 前尽量读满 n 字节（pyserial 的 read 可能只返回一部分）"""
    buf = b""
    while len(buf) < n and time.monotonic() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
    return buf


def read_frame(ser: serial.Serial, timeout_s: float = 10.0):
    """读一整帧，返回 (cmd, seq, data)；超时/CRC 错返回 None"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        # 找帧头
        b = ser.read(1)
        if not b:
            continue
        if b[0] != HDR0:
            continue
        b2 = ser.read(1)
        if not b2 or b2[0] != HDR1:
            continue
        head = _read_exact(ser, 6, deadline)
        if len(head) != 6:
            return None
        dlen = (head[4] << 8) | head[5]
        rest = _read_exact(ser, dlen + 2, deadline)
        if len(rest) != dlen + 2:
            return None
        raw = bytes([HDR0, HDR1]) + head + rest
        calc = crc16(raw[:-2])
        recv = (raw[-2] << 8) | raw[-1]
        if calc != recv:
            print(f"  ! CRC 错: calc=0x{calc:04X} recv=0x{recv:04X}")
            return None
        cmd = (head[0] << 8) | head[1]
        seq = (head[2] << 8) | head[3]
        return cmd, seq, rest[:dlen]
    return None


class Link:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.seq = 0

    def close(self):
        self.ser.close()

    def request(self, cmd: int, data: bytes = b"", timeout: float = 10.0):
        self.seq = (self.seq + 1) & 0xFFFF
        if self.seq == 0:
            self.seq = 1
        frame = build_frame(cmd, self.seq, data)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        # 可能先收到设备主动帧（上传超时通知 0x000A），按命令号过滤
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            r = read_frame(self.ser, max(0.1, deadline - time.monotonic()))
            if r is None:
                continue
            rcmd, rseq, rdata = r
            if rcmd == cmd and rseq == self.seq:
                return rdata
            print(f"  (忽略非应答帧 cmd=0x{rcmd:04X} seq={rseq})")
        return None


def status_text(data: bytes) -> str:
    if not data:
        return "<空>"
    st = data[0]
    return f"0x{st:02X} {STATUS.get(st, '?')}"


def query_version(link: Link) -> bool:
    r = link.request(CMD_QUERY_FLASH_VER, timeout=2.0)
    if r is None:
        print("✗ 查询版本无应答")
        return False
    if len(r) != 4:
        print(f"✗ 查询版本应答长度异常: {len(r)}")
        return False
    st = r[0]
    ver = ".".join(str(x) for x in r[1:4])
    print(f"✓ 查询版本: {status_text(r)}  ver={ver}")
    return st == 0x00


def upload(link: Link, image: bytes, version, progress: bool = True) -> bool:
    frames = max(1, math.ceil(len(image) / FRAME_MAX_DATA))
    start_data = bytes([
        (frames >> 8) & 0xFF, frames & 0xFF,
        (len(image) >> 24) & 0xFF, (len(image) >> 16) & 0xFF,
        (len(image) >> 8) & 0xFF, len(image) & 0xFF,
        version[0] & 0xFF, version[1] & 0xFF, version[2] & 0xFF,
    ])

    print(f"→ START: size={len(image)} frames={frames} ver="
          f"{version[0]}.{version[1]}.{version[2]}")
    r = link.request(CMD_FW_START, start_data, timeout=15.0)
    if r is None:
        print("✗ START 无应答")
        return False
    if r[0] != 0x00:
        print(f"✗ START 失败: {status_text(r)}")
        return False
    print("✓ START OK（已擦除目标扇区）")

    for i in range(frames):
        chunk = image[i * FRAME_MAX_DATA:(i + 1) * FRAME_MAX_DATA]
        r = link.request(CMD_FW_DATA, chunk, timeout=10.0)
        if r is None:
            print(f"✗ DATA 第 {i + 1}/{frames} 帧无应答")
            return False
        if r[0] != 0x00:
            print(f"✗ DATA 第 {i + 1}/{frames} 帧失败: {status_text(r)}")
            return False
        if progress and ((i + 1) == frames or (i + 1) % 16 == 0):
            done = min(len(image), (i + 1) * FRAME_MAX_DATA)
            print(f"  DATA {done}/{len(image)} B  ({done * 100 // max(1, len(image))}%)")

    print("→ END")
    r = link.request(CMD_FW_END, timeout=15.0)
    if r is None:
        print("✗ END 无应答")
        return False
    if r[0] != 0x00:
        print(f"✗ END 失败: {status_text(r)}")
        return False
    print("✓ END OK（已写完整标志并回读校验）")
    return True


def main():
    ap = argparse.ArgumentParser(description="M1 治具固件上传测试")
    ap.add_argument("port", help="USB CDC 虚拟串口，例如 COM7")
    ap.add_argument("image", nargs="?", help="要上传的 .bin 文件")
    ap.add_argument("--ver", default="1.0.0", help="版本号 主.次.修订（默认 1.0.0）")
    ap.add_argument("--query-only", action="store_true", help="只查询 Flash 里的版本")
    ap.add_argument("--no-query", action="store_true", help="上传完不查询版本")
    args = ap.parse_args()

    if not args.query_only and not args.image:
        ap.error("需要 image 文件，或使用 --query-only")

    try:
        link = Link(args.port)
    except Exception as e:
        print(f"✗ 打开串口失败: {e}")
        return 1

    try:
        if args.query_only:
            return 0 if query_version(link) else 1

        ver = [int(x) & 0xFF for x in args.ver.split(".")]
        if len(ver) != 3:
            print("✗ --ver 需要三段，例如 1.2.3")
            return 1

        with open(args.image, "rb") as f:
            image = f.read()
        if not image:
            print("✗ 镜像为空")
            return 1
        print(f"镜像: {args.image}  {len(image)} 字节")

        if not upload(link, image, ver):
            return 1
        if not args.no_query:
            query_version(link)
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
