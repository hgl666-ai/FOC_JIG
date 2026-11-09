# -*- coding: utf-8 -*-
"""复查审核脚本：把"固件常量 / 固件打包 / 上位机解码 / 文档"四处的布局对一遍。

检查项：
  A. 固件常量之间存在固定关系（头 + 项×数量）
  B. 上位机解码用的长度与固件常量一致
  C. 文档里写的长度与固件常量一致
  D. 烧录相关文件没有被本次重构碰到（git diff 里不该出现）
"""
import io
import re
import subprocess
import sys

ROOT = r"D:\Project\FOC_JIG"
FM = ROOT + r"\U1\Core\Inc\flash_map.h"
PP = ROOT + r"\U1\Core\Inc\pulse_pack.h"
TOOL = ROOT + r"\tools\foc_jig_tool.py"
D17 = ROOT + r"\docs\17_上位机命令清单.md"

ok = 0
bad = 0


def check(cond, msg):
    global ok, bad
    if cond:
        ok += 1
        print("  [OK ] %s" % msg)
    else:
        bad += 1
        print("  [BAD] %s" % msg)


def defs(path):
    s = io.open(path, encoding="utf-8").read()
    out = {}
    for m in re.finditer(r"#define\s+([A-Z_0-9]+)\s+\(?\s*([0-9]+)(?:u|U)?\s*\)?", s):
        out[m.group(1)] = int(m.group(2))
    return out


fm = defs(FM)
pp = defs(PP)

print("=== A. 固件常量自身的算术关系 ===")
CHECK_VOL = fm.get("MGR_VOL_TEST_RESP_LEN") == 2 + 9 * fm.get("MGR_VOL_CH_ITEM_LEN", -1)
check(CHECK_VOL, "电压 47 = 2 + 9×5")
check(fm.get("MGR_PHASE_JUDGE_RESP_LEN") == 1 + 3 * fm.get("MGR_PHASE_ITEM_LEN", -1),
      "三相 16 = 1 + 3×5")
check(fm.get("MGR_ENC_JUDGE_RESP_LEN") == 3 + 3 * fm.get("MGR_ENC_JUDGE_ITEM_LEN", -1),
      "编码器 12 = 3 + 3×3")
check(fm.get("MGR_CUR_TEST_RESP_LEN") == 2 + 2 * fm.get("MGR_CUR_ITEM_LEN", -1),
      "相电流 12 = 2 + 2×5")
check(pp.get("PULSE_PACK_RESP_LEN") == 2 + 3 * pp.get("PULSE_RESP_ITEM_LEN", -1),
      "脉冲 23 = 2 + 3×7")
check(pp.get("PULSE_PACK_LEN") == 45, "U1↔U2 交换块仍是 45B（内部链路，未动）")

print()
print("=== B. 上位机解码长度与固件一致 ===")
tool = io.open(TOOL, encoding="utf-8").read()
for name, expr, want in [
    ("do_vol", "need = 2 + 9 * 5", fm.get("MGR_VOL_TEST_RESP_LEN")),
    ("do_pulse", "need = 2 + 3 * 7", pp.get("PULSE_PACK_RESP_LEN")),
    ("do_phase", "need = 1 + 3 * 5", fm.get("MGR_PHASE_JUDGE_RESP_LEN")),
    ("do_enc", "need = 3 + 3 * 3", fm.get("MGR_ENC_JUDGE_RESP_LEN")),
    ("do_cur", "need = 2 + 2 * 5", fm.get("MGR_CUR_TEST_RESP_LEN")),
]:
    check(expr in tool, "%s 用的长度式 %r（固件=%s）" % (name, expr, want))

print()
print("=== C. 文档写的长度与固件一致 ===")
d17 = io.open(D17, encoding="utf-8").read()
for txt, want in [("应答 **47B**", 47), ("应答 **23B**", 23), ("应答 **16B**", 16),
                  ("应答 12B", 12)]:
    check(txt in d17, "docs/17 里写着 %r" % txt)
check("应答 **71B**" not in d17 and "应答 **91B**" not in d17, "docs/17 已无旧的 71B/91B")

print()
print("=== D. 烧录相关文件必须未被改动 ===")
try:
    out = subprocess.run(["git", "-C", ROOT, "diff", "--name-only"],
                         capture_output=True, text=True).stdout
    touched = [l for l in out.splitlines() if l.strip()]
    burn = [f for f in touched
            if ("fw_upload" in f) or ("swd_" in f) or ("stm32f4_flash" in f)
            or ("cortex_m" in f)]
    check(len(burn) == 0, "烧录/上传/SWD 源文件零改动%s" % ("" if not burn else "（%s）" % burn))
except Exception as exc:                     # noqa: BLE001
    print("  [skip] git 不可用: %s" % exc)

print()
print("=== 结论：%d 项通过，%d 项不通过 ===" % (ok, bad))
sys.exit(0 if bad == 0 else 1)
