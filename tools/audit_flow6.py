# -*- coding: utf-8 -*-
"""全面审查（改后）：把 0x0022 的代码逐条过一遍，脚本能查的用脚本查。

检查项：
  A. 六步流程的路径覆盖：宿主测试里每条分支都有正反用例
  B. 流程代码的健壮性约定（不再被重试计数坑、超时都停捕获/停电机）
  C. 烧录/SWD 零改动
  D. 资源占用在预算内
"""
import io
import re
import subprocess
import sys

ROOT = r"D:\Project\FOC_JIG"
AC = ROOT + r"\U1\Core\Src\app.c"
AT = ROOT + r"\test\app_test.c"
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


app = io.open(AC, encoding="utf-8").read()
test = io.open(AT, encoding="utf-8").read()
doc = io.open(D17, encoding="utf-8").read()

print("=== A. 六步流程的路径覆盖（测试里必须都有）===")
COVER = [
    ("步1 通过",        "步1(ADC)：注入合格码值 → 通过"),
    ("步1 失败",        "步1(ADC)：宿主没注入电压 → 判失败"),
    ("步2 通过",        "步2(存活)：通过"),
    ("步2 超时",        "步2(存活)：判失败"),
    ("步3 A0",          "步3(自检)：通过，detail = A0"),
    ("步3 A1",          "步3 detail = 1（A1）"),
    ("步3 A2",          "A2 分支：发 AS:MC 进数编码器模式"),
    ("步3 A3",          "A3 分支能跑到第 4 步并给出报告"),
    ("步4 通过",        "步4(三相)：1kHz/50% 三路都有 → 通过"),
    ("步4 频率不对",    "步4：20kHz 不在 0.5~2ms 窗口内 → 判失败"),
    ("步4 占空比不对",  "步4：80% 占空比超出 450~550‰ → 判失败"),
    ("步4 无信号",      "步4(三相)：没有波形 → 失败"),
    ("步4 缺 C 相",     "C 相没波形（A/B 有）"),
    ("步4 缺 B 相",     "B 相没波形（A/C 有）"),
    ("步4 缺 A 相",     "A 相没波形（B/C 有）"),
    ("步5 通过",        "步5(编码器)：通过"),
    ("步5 无 Z",        "步5(编码器)：C0 且 A/B 对 → 仍判通过"),
    ("步5 计数不符",    "步5(编码器)：A 相数不对 → 判失败"),
    ("步5 超时",        "步5(编码器)：没收到报数 → 失败"),
    ("步6 收尾",        "步6(收尾)：通过"),
    ("参数错",          "LEN≠0 → 0B ERR_PARAM"),
    ("全绿 status=00",  "**六步全过 → 状态 00**"),
]
for tag, needle in COVER:
    check(needle in test, tag)

print()
print("=== B. 流程代码的健壮性约定 ===")
check("s_fl6_tries = 0u;" not in app.split("static uint8_t FL6_Send")[1].split("}")[0],
      "FL6_Send 里不再清重试计数（清了会死循环）")
check("s_fl6_tries = 0u;" in app.split("static void FL6_Goto")[1].split("}")[0],
      "FL6_Goto 里清重试计数（换一步重新数）")
check("PULSE_CfgStart((uint16_t)FL6_PULSE_WIN_MS)" in app,
      "步4 用 PULSE_CfgStart（cfg 层，真板上才采得到）")
check("PULSE_CfgPoll() != 0u" in app, "步4 用 PULSE_CfgPoll 推进")
check(app.count("AS_CMD_STOP") >= 5,
      "停电机的地方够多（自检后/步4/步5/收尾，共 %d 处）" % app.count("AS_CMD_STOP"))
check("PULSE_CfgStop();" in app, "步4 超时分支会关掉捕获")
check("(s_fl6_code[FL6_S_ALIVE] == 0x02u) ? FL6_S_DONE : FL6_S_SELF" in app,
      "存活失败直接收尾（不白等 5 秒自检）")
check("APP_STATE_FLOW6" in app and "app_state = APP_STATE_IDLE;" in app.split("MGR_CMD_FLOW_TEST")[0][-4000:],
      "流程结束会回 IDLE")
check("len != 0u" in app.split("static void APP_FlowTestCmd")[1][:600],
      "0x0022 对 LEN≠0 回 0B")

print()
print("=== C. 文档与实现对得上 ===")
for tag, needle in [
    ("docs/17 写了存活失败直接收尾", "直接跳到步6"),
    ("docs/17 写了 detail 保留值",   "0xFFFE"),
    ("docs/17 写了两种应答形态",     "一个可用报告都没拿到"),
]:
    check(needle in doc, tag)

print()
print("=== D. 烧录/SWD 零改动 ===")
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
print("=== 结论：%d 项通过，%d 项不通过 ===" % (ok, bad))
sys.exit(0 if bad == 0 else 1)
