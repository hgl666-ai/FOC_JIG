# find_com.py  —— 按 VID/PID 找 COM 口
import sys

try:
    import serial.tools.list_ports
except ImportError:
    print("需要 pyserial:  pip install pyserial")
    sys.exit(1)


def find_ports(vid, pid, silent=False):
    """
    vid/pid 可以传 int 或十六进制字符串，例如：
        find_ports(0x2327, 0x4A07)
        find_ports("2327", "4A07")
        find_ports(8999, 18999)      # 十进制也行
    """
    if isinstance(vid, str):
        vid = int(vid, 16)
    if isinstance(pid, str):
        pid = int(pid, 16)

    hits = []
    for p in serial.tools.list_ports.comports():
        # p.vid / p.pid 是 int 或 None；USB 位置信息在 hwid 里
        if p.vid == vid and p.pid == pid:
            hits.append(p)
        else:
            print(f"未知设备：{p.device}  {p.vid:04X}  {p.pid:04X}")

    if not silent:
        print(f"扫描 VID=0x{vid:04X} PID=0x{pid:04X}  ...")
        if not hits:
            print("  ✗ 没找到设备，检查接线 / VID_PID / 驱动")
        for p in hits:
            print(f"  ✓ {p.device:8}  serial={p.serial_number!r}  "
                  f"desc={p.description!r}")
    return hits


if __name__ == "__main__":
    # ---- 改成你自己的数 ----
    MY_VID = 0x2327   # 8999
    MY_PID = 0x4A37   # 18999

    ports = find_ports(MY_VID, MY_PID)

    if len(ports) == 1:
        print(f"\n=> 用这个口: {ports[0].device}")
    elif len(ports) > 1:
        print(f"\n=> 找到 {len(ports)} 个，按 serial_number 区分：")
        for p in ports:
            print(f"    {p.device}  (SN={p.serial_number})")
