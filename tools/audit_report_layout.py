# -*- coding: utf-8 -*-
"""复查审核脚本（v2）：把"固件常量 / 固件打包 / 上位机解码 / 三份文档"五处对一遍。

检查项：
  A. 固件常量自身的算术关系
  B. 上位机解码用的长度与固件常量一致
  C. docs/17、INFO/命令大全、docs/21 里写的长度与固件一致，且**没有旧长度残留**
  D. 烧录相关源文件零改动
  E. INFO/命令大全 的结构完整（章节/附录都在）
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
CMD = ROOT + r"\INFO\命令大全.txt"
D21 = ROOT + r"\docs\21_台架五项功能测试手册.md"

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
V, PU, PH, E, C = (fm["MGR_VOL_TEST_RESP_LEN"], pp["PULSE_PACK_RESP_LEN"],
                   fm["MGR_PHASE_JUDGE_RESP_LEN"], fm["MGR_ENC_JUDGE_RESP_LEN"],
                   fm["MGR_CUR_TEST_RESP_LEN"])

print("=== A. 固件常量自身的算术关系 ===")
check(V == 2 + 9 * fm["MGR_VOL_CH_ITEM_LEN"], "电压 %d = 2 + 9×%d" % (V, fm["MGR_VOL_CH_ITEM_LEN"]))
check(PH == 1 + 3 * fm["MGR_PHASE_ITEM_LEN"], "三相 %d = 1 + 3×%d" % (PH, fm["MGR_PHASE_ITEM_LEN"]))
check(E == 3 + 3 * fm["MGR_ENC_JUDGE_ITEM_LEN"], "编码器 %d = 3 + 3×%d" % (E, fm["MGR_ENC_JUDGE_ITEM_LEN"]))
check(C == 2 + 2 * fm["MGR_CUR_ITEM_LEN"], "相电流 %d = 2 + 2×%d" % (C, fm["MGR_CUR_ITEM_LEN"]))
check(PU == 2 + 3 * pp["PULSE_RESP_ITEM_LEN"], "脉冲 %d = 2 + 3×%d" % (PU, pp["PULSE_RESP_ITEM_LEN"]))
check(pp["PULSE_PACK_LEN"] == 45, "U1↔U2 交换块仍是 45B（内部链路，未动）")

print()
print("=== B. 上位机解码长度与固件一致 ===")
tool = io.open(TOOL, encoding="utf-8").read()
for name, expr in [("do_vol", "need = 2 + 9 * 5"), ("do_pulse", "need = 2 + 3 * 7"),
                   ("do_phase", "need = 1 + 3 * 5"), ("do_enc", "need = 3 + 3 * 3"),
                   ("do_cur", "need = 2 + 2 * 5")]:
    check(expr in tool, "%s 用 %r" % (name, expr))

print()
print("=== C. 三份文档与固件一致 ===")
d17 = io.open(D17, encoding="utf-8").read()
cmd = io.open(CMD, encoding="utf-8").read()
d21 = io.open(D21, encoding="utf-8").read()

for txt, want, tag in [("应答 **47B**", V, "docs/17 电压"),
                       ("应答 **23B**", PU, "docs/17 脉冲"),
                       ("应答 **16B**", PH, "docs/17 三相"),
                       ("应答 12B", C, "docs/17 相电流")]:
    check(txt in d17, "%s 写着 %d" % (tag, want))

# 71/91/34/22 只属于被精简的那几条命令，出现即错；
# 29 要区别对待 —— 0x0010（一键全流程）本来就是 29 字节。
old_pat = r"=\s*(71|91|34|22)\b"
for name, txt in [("docs/17", d17), ("命令大全", cmd), ("docs/21", d21)]:
    hits = [m.group(0) for m in re.finditer(old_pat, txt)]
    check(len(hits) == 0, "%s 已无旧长度 71/91/34/22（残留 %d 处%s）"
          % (name, len(hits), "" if not hits else "：%s" % hits[:5]))
    # 29：只允许出现在 0x0010 相关的行上
    bad29 = []
    for line in txt.splitlines():
        if re.search(r"=\s*29\b", line) and ("00 10" not in line) and ("0x0010" not in line) \
           and ("共 29 字节" not in line):
            bad29.append(line.strip()[:60])
    check(len(bad29) == 0, "%s 的「= 29」只出现在 0x0010 上（异常 %d 处%s）"
          % (name, len(bad29), "" if not bad29 else "：%s" % bad29[:3]))

# 命令大全里三条"一条命令跑完"的长度也得对
for hexlen, want, tag in [("00 17", PU, "0x0019"), ("00 10", PH, "0x001A"), ("00 0C", E, "0x001B")]:
    check(("LEN=%s(%d)" % (hexlen, want)) in cmd or ("长度 %s（= %d）" % (hexlen, want)) in cmd,
          "命令大全 %s 的应答长度写的是 %d" % (tag, want))

print()
print("=== D. 烧录相关源文件必须未被改动 ===")
try:
    out = subprocess.run(["git", "-C", ROOT, "diff", "--name-only"],
                         capture_output=True, text=True).stdout
    touched = [l for l in out.splitlines() if l.strip()]
    burn = [f for f in touched if ("fw_upload" in f) or ("swd_" in f)
            or ("stm32f4_flash" in f) or ("cortex_m" in f)]
    check(len(burn) == 0, "烧录/上传/SWD 源文件零改动%s" % ("" if not burn else "（%s）" % burn))
except Exception as exc:                                  # noqa: BLE001
    print("  [skip] git 不可用: %s" % exc)

print()
print("=== E. 命令大全结构完整 ===")
for sec in ["一、PC ↔ 治具 U1", "二、U1 ↔ U2", "附录 A", "附录 B", "附录 C",
            "附录 D", "附录 E", "附录 F", "附录 G", "附录 H"]:
    check(sec in cmd, "含「%s」" % sec)
check(cmd.rstrip().endswith("（完）\n================================================================================")
      or "（完）" in cmd[-200:], "结尾「（完）」还在")

print()
print("=== 结论：%d 项通过，%d 项不通过 ===" % (ok, bad))
sys.exit(0 if bad == 0 else 1)
