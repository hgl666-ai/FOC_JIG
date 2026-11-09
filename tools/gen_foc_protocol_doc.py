# -*- coding: utf-8 -*-
"""生成《FOC 被测板固件 —— 与测试治具的通讯协议及功能要求》.docx

所有示例帧的 CRC 都用 tools/foc_jig_tool.py 的 crc16_modbus 现算（与治具固件同一套算法），
不是手抄的。改协议时改这里重新生成。
"""
import os
import re
import sys

sys.path.insert(0, r"D:\Project\FOC_JIG\tools")

from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn

from foc_jig_tool import build_frame, crc16_modbus, hx

OUT = r"D:\Project\FOC_JIG\docs\22_FOC固件对接要求_v1.2.docx"

EA_FONT = u"微软雅黑"
CODE_FONT = "Consolas"


# ─────────────────────────── 排版小工具 ───────────────────────────
def set_run_font(run, name=EA_FONT, size=10.5, bold=False, color=None):
    run.font.name = name
    run.font.size = Pt(size)
    run.bold = bold
    if color is not None:
        run.font.color.rgb = color
    rpr = run._element.get_or_add_rPr()
    rf = rpr.find(qn("w:rFonts"))
    if rf is None:
        rf = rpr.makeelement(qn("w:rFonts"), {})
        rpr.append(rf)
    rf.set(qn("w:eastAsia"), name)
    rf.set(qn("w:ascii"), name)
    rf.set(qn("w:hAnsi"), name)


BOLD_RE = re.compile(r"\*\*(.+?)\*\*")
CODE_RE = re.compile(r"`([^`]+)`")


def _emit(p, text, size, bold, color):
    """只处理行内代码（反引号），其余原样；bold 由外层决定"""
    pos = 0
    for m in CODE_RE.finditer(text):
        if m.start() > pos:
            set_run_font(p.add_run(text[pos:m.start()]), size=size, bold=bold, color=color)
        set_run_font(p.add_run(m.group(1)), name=CODE_FONT,
                     size=(size - 0.5) if size > 8.5 else size, bold=bold, color=color)
        pos = m.end()
    if pos < len(text):
        set_run_font(p.add_run(text[pos:]), size=size, bold=bold, color=color)


def add_rich(p, text, size=10.5, bold=False, color=None):
    """两级渲染：先切 **粗体**，再在每段里切行内代码

    两层都要支持，因为文档里有"**收到就回 X，不要等发完** —— 说明"这种写法：
    粗体里面还套着命令码。只做一层的话，内层的反引号会原样打进 Word。
    """
    pos = 0
    for m in BOLD_RE.finditer(text):
        if m.start() > pos:
            _emit(p, text[pos:m.start()], size, bold, color)
        _emit(p, m.group(1), size, True, color)
        pos = m.end()
    if pos < len(text):
        _emit(p, text[pos:], size, bold, color)


def para(text="", size=10.5, bold=False, align=None, space_after=4, indent=None):
    p = doc.add_paragraph()
    if align is not None:
        p.alignment = align
    p.paragraph_format.space_after = Pt(space_after)
    p.paragraph_format.space_before = Pt(0)
    if indent is not None:
        p.paragraph_format.left_indent = Cm(indent)
    if text:
        add_rich(p, text, size=size, bold=bold)
    return p


def h1(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(14)
    p.paragraph_format.space_after = Pt(6)
    add_rich(p, text, size=15, bold=True, color=RGBColor(0x1F, 0x3B, 0x63))
    return p


def h2(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(10)
    p.paragraph_format.space_after = Pt(4)
    add_rich(p, text, size=12, bold=True, color=RGBColor(0x24, 0x4A, 0x7A))
    return p


def h3(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(8)
    p.paragraph_format.space_after = Pt(2)
    add_rich(p, text, size=11, bold=True)
    return p


def bullet(text, size=10.5):
    p = doc.add_paragraph(style="List Bullet")
    p.paragraph_format.space_after = Pt(2)
    add_rich(p, text, size=size)
    return p


def code(text):
    """等宽小字，用于帧示例（一段一段贴，不折行）"""
    for line in text.split("\n"):
        p = doc.add_paragraph()
        p.paragraph_format.space_after = Pt(0)
        p.paragraph_format.space_before = Pt(0)
        p.paragraph_format.left_indent = Cm(0.5)
        set_run_font(p.add_run(line if line else " "), name=CODE_FONT, size=9)
    # 末尾留白
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


def table(headers, rows, widths=None, size=9.5, header_bg=None):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    hdr = t.rows[0].cells
    for i, htxt in enumerate(headers):
        hdr[i].text = ""
        pp = hdr[i].paragraphs[0]
        pp.paragraph_format.space_after = Pt(1)
        add_rich(pp, htxt, size=size, bold=True)
    for row in rows:
        cells = t.add_row().cells
        for i, val in enumerate(row):
            cells[i].text = ""
            pp = cells[i].paragraphs[0]
            pp.paragraph_format.space_after = Pt(1)
            add_rich(pp, str(val), size=size)
    if widths:
        for r in t.rows:
            for i, w in enumerate(widths):
                r.cells[i].width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t


def frame(cmd, seq, data_hex=""):
    """按当前协议现算一条帧的字节串"""
    return hx(build_frame(cmd, seq, bytes.fromhex(data_hex)))


# ─────────────────────────── 文档 ───────────────────────────
doc = Document()

# 页边距
for sec in doc.sections:
    sec.top_margin = Cm(2.2)
    sec.bottom_margin = Cm(2.0)
    sec.left_margin = Cm(2.4)
    sec.right_margin = Cm(2.2)

# 默认字体
style = doc.styles["Normal"]
style.font.name = EA_FONT
style.font.size = Pt(10.5)
style.element.rPr.rFonts.set(qn("w:eastAsia"), EA_FONT)

# ── 封面标题 ──
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
set_run_font(p.add_run(u"FOC 被测板固件"), size=20, bold=True,
             color=RGBColor(0x1F, 0x3B, 0x63))
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
set_run_font(p.add_run(u"与测试治具的通讯协议及功能要求"), size=16, bold=True,
             color=RGBColor(0x1F, 0x3B, 0x63))
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
set_run_font(p.add_run(u"版本 v1.2　·　2026-11-05"), size=11)

doc.add_paragraph()
doc.add_page_break()

table([u"项目", u"内容"], [
    [u"文档目的", u"请 FOC 板固件开发者按本文档实现 9 条串口命令（0x0200~0x0208）及配套的安全机制"],
    [u"当前状态", u"**治具侧已全部实现，并在台架上验证通过**（含 4 条板外命令的判据、帧编解码、超时重试）；本文档所要求的 DUT 侧功能**尚未实现**"],
    [u"改动须知", u"帧格式、命令码、字段布局、长度都是**定死的**；本文档每个示例帧的 CRC 都已用与治具同一套算法核算过，可直接抓包对拍"],
    [u"回传要求", u"实现完成后请按第 10 章自测，把每一步的收发字节回传给我们；第 11 章列了我们还需要你确认的参数"],
], widths=[3.0, 12.4], size=9.5)

# ── 1 ──
h1(u"1　这份文档要你做三件事")
bullet(u"**在 FOC 板上实现 9 条串口命令**（0x0200 ~ 0x0208）。其中 4 条是**必做项**（PING / INFO / TEST_MODE / STOP），"
       u"其余 5 条按你的实现能力在 INFO 里声明即可。")
bullet(u"**实现“测试模式”和“失联看门狗”**：这两个不是可选项。测试模式让治具能在不驱动电机的前提下控制功率级；"
       u"看门狗保证治具掉线时功率级会自动关断。")
bullet(u"**按第 10 章自测**并回传结果。自测只用串口助手就能做，不需要治具在场。")

para(u"另外说清楚一件事：**烧录不需要你做任何事**。是同一台治具、用 SWD 两线直接写你的 Flash"
     u"（见 2.1），你只要把那两个引脚留出来。也就是说 —— 一台治具、一份固件，"
     u"既负责烧录，也负责下面这五项功能测试，不分两台设备。", size=10)

para(u"为什么值得按这个协议做：治具侧已经把这套东西全写完了（帧编解码、CRC、超时、"
     u"五项测试的判据、报告打包），你这边只要按协议回帧，产线上“一压一按出结果”就能跑起来。",
     size=10)

# ── 2 ──
h1(u"2　系统分工与物理连接")
para(u"治具（测试夹具）是**主机**，FOC 板是**从机**。所有通讯由治具发起，FOC 板只需“收一帧、回一帧”。")
table([u"项目", u"约定"], [
    [u"电气", u"3.3V TTL 电平（治具侧为 3.3V 推挽输出），**必须共地**"],
    [u"接线", u"治具 TX → 你的 RX；治具 RX → 你的 TX（**交叉**）"],
    [u"串口参数", u"**115200、8 数据位、无校验、1 停止位、无硬件流控**"],
    [u"宿主外设", u"任意一路 USART（治具侧可配置），DMA 可选，中断或轮询均可"],
    [u"帧方向", u"治具发请求 → 你回应答；**你不需要主动发任何东西**"],
], widths=[3.0, 12.4], size=9.5)

h2(u"2.1　同一台治具还负责烧录（这部分你不用实现）")
para(u"整台治具做两件事：**烧固件** + **五项功能测试**，不分两台设备。烧录用 SWD 两线"
     u"（SWDIO = PA13、SWCLK = PA14）直接写你的 Flash，再复位运行 —— 不需要你在固件里加任何东西。")
para(u"你这边只需要保证一件事：**这两个引脚留出来**，别复用成普通 IO 或别的功能；"
     u"板上也要有地方能接到它们（排针/测试点都行）。")
para(u"`SWD_USE_NRST` 默认不接复位线：治具靠 `AIRCR.SYSRESETREQ` 让芯片复位运行，"
     u"所以你的固件里不要禁用这条复位路径。", size=10)

# ── 3 ──
h1(u"3　帧格式（必须逐字节一致）")

h2(u"3.1　帧结构")
code(u"偏移    0     1     2     3     4     5     6     7    8 .. 8+L-1   8+L   9+L\n"
     u"      A5    5B   CMD_H CMD_L SEQ_H SEQ_L LEN_H LEN_L    DATA...      CRC_H  CRC_L\n"
     u"      └── 帧头 ──┘└─ 命令 ─┘└── 序号 ──┘└─ 数据长度 ─┘└── 数据 ──┘└─ CRC16 ─┘")

table([u"字段", u"长度", u"说明"], [
    [u"帧头", u"2", u"固定 `A5 5B`"],
    [u"CMD", u"2", u"命令码，2 字节**大端**"],
    [u"SEQ", u"2", u"序号，2 字节大端。**你必须原样回显**（治具靠它配对请求与应答）"],
    [u"LEN", u"2", u"DATA 的字节数（**不含**帧头/CMD/SEQ/LEN/CRC），大端。**单帧 DATA ≤ 64**"],
    [u"DATA", u"L", u"见第 7 章每条命令"],
    [u"CRC", u"2", u"CRC-16/MODBUS，计算范围 = **从 `A5` 到 DATA 最后一个字节**，大端存放"],
], widths=[2.2, 1.2, 12.0], size=9.5)

h2(u"3.2　规则（这几条最容易出错）")
table([u"#", u"规则"], [
    [u"1", u"**全部大端**：CMD / SEQ / LEN / 所有多字节字段，一律高字节在前"],
    [u"2", u"CRC 算法：CRC-16/MODBUS（初值 0xFFFF，反射多项式 0xA001）。"
           u"**自检：CRC16(\"123456789\") 必须等于 0x4B37** —— 先过这一条再往下做"],
    [u"3", u"**CRC 错 → 直接丢弃，不要回应答，也不要回错误码**"],
    [u"4", u"**不认识的命令号 → 必须回 0xFF（未实现）**，不能不回"],
    [u"5", u"**收到完整帧后 ≤ 5 ms 内开始回**"],
    [u"6", u"**一帧不能拆开发**：相邻两字节间隔超过 5 ms 会被当成半截帧丢弃。"
           u"禁止在发送中间插入延时（如 HAL_Delay）或分段发送"],
    [u"7", u"**所有命令必须幂等**：治具在超时后会**用同一个 SEQ 重发一次**。重复收到完全相同的请求 → 只回 OK，不要重新执行一遍动作"],
    [u"8", u"应答的 CMD 和 SEQ 都**原样回填**；DATA 一般只有 1 字节状态码（INFO / GET_ADC / GET_ENC 例外）"],
], widths=[0.9, 14.5], size=9.5)

h2(u"3.3　应答通用模板")
code(u"A5 5B | CMD 原样 | SEQ 原样 | 00 01 | 状态码(1B) | CRC16")

# ── 4 ──
h1(u"4　状态码")
table([u"码", u"含义", u"什么时候回"], [
    [u"00", u"OK", u"正常"],
    [u"01", u"BUSY", u"没进测试模式就收到 PWM / LEVEL / CURRENT"],
    [u"0B", u"参数错", u"长度不对、取值越界、保留位非 0 —— 且**绝不能产生任何输出动作**"],
    [u"0C", u"超时", u"内部等待超时（例如等电流建立）"],
    [u"FF", u"未实现", u"命令号不认识"],
], widths=[1.4, 2.6, 11.4], size=9.5)

# ── 5 ──
h1(u"5　测试模式 0x0202（必须实现）")

h2(u"5.1　为什么必须有它")
para(u"治具要测“MCU 的 PWM 对不对”“三相输出有没有电压”“相电流采样准不准”，就必须让 FOC 板"
     u"**按治具的命令输出**，而不是按电机控制环输出。测试模式就是这个开关：进去以后停控制环、"
     u"交出功率级的控制权；退出以后恢复正常的电机控制。")

h2(u"5.2　进入测试模式时必须按这个顺序做（顺序错了整套测试都不成立）")
table([u"步", u"动作"], [
    [u"1", u"**立刻关断三相输出**（上下桥全关）"],
    [u"2", u"**停掉 FOC 控制环**（电流环 / 速度环 / 位置环）"],
    [u"3", u"**把编码器 A/B/Z 计数清零**"],
    [u"4", u"复位看门狗计时"],
    [u"5", u"回 `00 OK`"],
], widths=[1.0, 14.4], size=9.5)

h2(u"5.3　其它硬性规则")
bullet(u"**幂等**：已经在测试模式里又收到 `mode=1` → 只回 OK，**不要**再关一次输出、再清一次计数"
       u"（会把正在测的计数抹掉）。")
bullet(u"`wdt_ms` 范围 **200 ~ 10000**，越界（含 0）回 `0B`。**不允许关掉看门狗**。")
bullet(u"退出（`mode=0`）：**先关输出、再恢复控制环**，顺序不能反。")
bullet(u"**上电/复位后**：三相输出必须关断，且**不进入**测试模式。")

# ── 6 ──
h1(u"6　失联看门狗（安全底线，必须实现）")
para(u"场景：治具正在让功率级输出，PC 崩了 / 线被拔了 / 治具复位了 —— 没有看门狗，电机会一直转。")
bullet(u"进测试模式时由 `wdt_ms` 设定超时（200~10000 ms）。")
bullet(u"**每收到任何一帧合法请求就重置计时**（不只是 PWM 命令）。")
bullet(u"**超时 → 立刻关断三相输出**，回到“等治具指挥”状态（不退出测试模式）。")
bullet(u"`wdt_ms = 0` 或越界 → 回 `0B`，**不允许关掉看门狗**。")

# ── 7 ──
doc.add_page_break()
h1(u"7　九条命令逐条说明")
para(u"下面每条都给出：请求/应答的字节布局、硬性规则、以及**可以直接抓包对拍的完整帧**。"
     u"示例里的 SEQ 只是举例，你按收到的原样回填即可；CRC 已按当前协议算好。", size=10)

# 7.1 PING
h2(u"7.1　0x0200 PING —— 链路自检（必做）")
table([u"方向", u"DATA"], [
    [u"治具发", u"任意 0~64 字节"],
    [u"你回", u"**原样回显**（注意：PING 的应答**没有状态码**）"],
], widths=[2.4, 13.0], size=9.5)
code(u"治具发  " + frame(0x0200, 1, "DEADBEEF") + u"\n"
     u"你回    " + frame(0x0200, 1, "DEADBEEF") + u"      ← 完全一样")
para(u"用途：治具上电后第一件事就是发它。回对了，说明“接线、波特率、帧格式、CRC”全对。", size=10)

# 7.2 INFO
h2(u"7.2　0x0201 INFO —— 版本与功能声明（必做）")
table([u"方向", u"DATA"], [
    [u"治具发", u"0 字节（LEN=0）"],
    [u"你回", u"**16 字节**"],
], widths=[2.4, 13.0], size=9.5)
table([u"off", u"长度", u"字段", u"填什么", u"示例"], [
    [u"0", u"1", u"status", u"`00`", u"`00`"],
    [u"1..2", u"2", u"dut_ver", u"**固定 `0x0101`**（协议版本）", u"`01 01`"],
    [u"3..5", u"3", u"fw_ver", u"你的固件版本 主.次.修订", u"`01 00 00`"],
    [u"6..7", u"2", u"caps", u"**功能声明**：bit i = 1 表示支持命令 0x0200+i（算法与例子见第 8 章）", u"`03 FF`"],
    [u"8..9", u"2", u"pwm_min_hz", u"你能输出的最低 PWM 频率", u"`00 64` = 100 Hz"],
    [u"10..11", u"2", u"pwm_max_hz", u"最高 PWM 频率", u"`4E 20` = 20000 Hz"],
    [u"12..13", u"2", u"adc_mask", u"哪几路模拟量能报（bit i = 通道 i，见 8.5 与 7.8）", u"`01 FF` = 9 路全支持"],
    [u"14..15", u"2", u"rsvd", u"`00 00`", u"`00 00`"],
], widths=[1.3, 1.1, 2.2, 8.0, 3.2], size=9)
code(u"治具发  " + frame(0x0201, 2) + u"\n"
     u"你回    " + hx(build_frame(0x0201, 2, bytes.fromhex("00010101000003FF00644E2001FF0000"))))
para(u"**只填你真实现了的位**：治具按 `caps` 决定测哪些项、按 `adc_mask` 决定问哪些通道；"
     u"没声明的会被跳过，**不算你错**。", size=10)

# 7.3 TEST_MODE
h2(u"7.3　0x0202 TEST_MODE —— 进/出测试模式（必做）")
table([u"方向", u"DATA"], [
    [u"治具发", u"3 字节：`mode(1) + wdt_ms(2, 大端)`"],
    [u"你回", u"1 字节状态码"],
], widths=[2.4, 13.0], size=9.5)
table([u"off", u"长度", u"字段", u"取值"], [
    [u"0", u"1", u"mode", u"`01` = 进入；`00` = 退出（回正常电机控制）"],
    [u"1..2", u"2", u"wdt_ms", u"失联看门狗超时，**200~10000**；越界回 `0B`"],
], widths=[1.3, 1.1, 2.2, 10.8], size=9)
code(u"治具发  " + frame(0x0202, 3, "0103E8") + u"        ← 进入，wdt=1000ms\n"
     u"你回    " + hx(build_frame(0x0202, 3, b"\x00")))
code(u"治具发  " + frame(0x0202, 13, "010000") + u"        ← wdt=0（非法）\n"
     u"你回    " + hx(build_frame(0x0202, 13, b"\x0b")) + u"        ← 必须回 0B，且不动作")

# 7.4 STOP
h2(u"7.4　0x0203 STOP —— 关输出（必做）")
table([u"方向", u"DATA"], [
    [u"治具发", u"0 字节"],
    [u"你回", u"1 字节状态码 `00`"],
], widths=[2.4, 13.0], size=9.5)
para(u"**任何状态下都必须执行**：没进测试模式、正在发 PWM、正在通流，都要能停 —— 关全部桥臂、停 PWM、停通流。"
     u"治具在每一项测试结束时都会发它。", size=10)
code(u"治具发  " + frame(0x0203, 4) + u"\n"
     u"你回    " + hx(build_frame(0x0203, 4, b"\x00")))

# 7.5 PWM
h2(u"7.5　0x0204 PWM —— 让三相输出已知 PWM")
para(u"治具用它来测两件事：**② MCU 的 PWM 波形对不对**（治具用自己的定时器捕获，量周期和占空比）、"
     u"**③ 三相输出有没有真的变成电压**（治具量相电压，判 V ≈ 占空比 × 母线电压）。", size=10)
table([u"方向", u"DATA"], [
    [u"治具发", u"10 字节"],
    [u"你回", u"1 字节状态码"],
], widths=[2.4, 13.0], size=9.5)
table([u"off", u"长度", u"字段", u"取值", u"示例"], [
    [u"0..1", u"2", u"freq_hz", u"PWM 频率 Hz", u"`03 E8` = 1000 Hz"],
    [u"2..3", u"2", u"duty_a", u"A 相占空比，**千分比 0~1000**", u"`01 F4` = 500‰ = 50%"],
    [u"4..5", u"2", u"duty_b", u"同上", u"`01 F4`"],
    [u"6..7", u"2", u"duty_c", u"同上", u"`01 F4`"],
    [u"8..9", u"2", u"count", u"脉冲个数；**`0` = 一直发到我发 STOP / 看门狗超时**", u"`01 2C` = 300 个"],
], widths=[1.3, 1.1, 2.2, 8.2, 3.0], size=9)
code(u"治具发  " + frame(0x0204, 5, "03E801F401F401F4012C") + u"     ← 1kHz / 50% / 300 个\n"
     u"你回    " + hx(build_frame(0x0204, 5, b"\x00")))
h3(u"硬性要求")
bullet(u"**收到就回 `00 OK`，不要等脉冲发完** —— 治具要靠这个应答立刻开自己的捕获窗口。")
bullet(u"`count / freq_hz` 必须 **≥ 0.25 秒**（治具的捕获窗口 200 ms + 余量）。1 kHz + count=300 → 300 ms ✔；"
       u"1 kHz + count=100 → 只有 100 ms，窗口会被截尾 ✘。")
bullet(u"`freq_hz` 超出你在 INFO 里声明的范围、或 `duty_* > 1000` → 回 `0B` 且**不输出**。")
bullet(u"**幂等**：收到**完全相同**的请求 → 沿用当前这一轮输出，**不要重起一轮**。")
bullet(u"没进测试模式就收到 → 回 `01 BUSY`。")
code(u"治具发  " + frame(0x0204, 0x0E, "03E801F401F401F4012C") + u"\n"
     u"你回    " + hx(build_frame(0x0204, 0x0E, b"\x01")) + u"        ← 没进测试模式，回 BUSY")

# 7.6 LEVEL
h2(u"7.6　0x0205 LEVEL —— 三相静态电平")
table([u"方向", u"DATA"], [
    [u"治具发", u"2 字节：`level(1) + rsvd(1)`"],
    [u"你回", u"1 字节状态码"],
], widths=[2.4, 13.0], size=9.5)
para(u"`level` 每相 2 bit：**bit1..0 = A，bit3..2 = B，bit5..4 = C**；bit7..6 必须 `00`。", size=10)
table([u"取值", u"含义"], [
    [u"`00`", u"输出低（下桥导通）"],
    [u"`01`", u"输出高（上桥导通）"],
    [u"`10`", u"高阻 / 关断（**安全默认态**）"],
    [u"`11`", u"保留 → 回 `0B`"],
], widths=[2.0, 13.4], size=9.5)
para(u"例：A=高、B=低、C=高阻 → `level = 0b_00_10_00_01 = 0x21`。", size=10)
code(u"治具发  " + frame(0x0205, 6, "2100") + u"\n"
     u"你回    " + hx(build_frame(0x0205, 6, b"\x00")))
bullet(u"**必须做上下管互锁**（同一桥臂上下管不能同时导通）；不允许的组合回 `0B`。")
bullet(u"三相全高阻是安全默认态。")

# 7.7 CURRENT
h2(u"7.7　0x0206 CURRENT —— 相电流激励")
para(u"治具用它测**⑤ 相电流检测**：先读基线 → 让你通流 → 再读“通流中” → 比**变化量**。", size=10)
table([u"方向", u"DATA"], [
    [u"治具发", u"8 字节"],
    [u"你回", u"1 字节状态码"],
], widths=[2.4, 13.0], size=9.5)
table([u"off", u"长度", u"字段", u"取值", u"示例"], [
    [u"0", u"1", u"mode", u"`00` = 静态通流；`01` = 低频 PWM；其它 → `0B`", u"`00`"],
    [u"1", u"1", u"hi_mask", u"上桥相：bit0=A / bit1=B / bit2=C，**只允许 1 位为 1**", u"`01` = A"],
    [u"2", u"1", u"lo_mask", u"下桥相：同编码，**必须与 hi 不同相**（同相 = 直通短路 → `0B`）", u"`02` = B"],
    [u"3..4", u"2", u"duty_permille", u"千分比；`mode=0` 时按 1000 处理（**忽略本字段**）", u"`03 E8` = 1000‰"],
    [u"5..6", u"2", u"duration_ms", u"持续时间 ms；`0` = 一直保持到我发 STOP", u"`00 C8` = 200 ms"],
    [u"7", u"1", u"rsvd", u"`00`", u"`00`"],
], widths=[1.3, 1.1, 2.4, 8.0, 3.0], size=9)
code(u"治具发  " + frame(0x0206, 7, "000102" "03E8" "00C8" "00") + u"\n"
     u"你回    " + hx(build_frame(0x0206, 7, b"\x00")))
h3(u"硬性要求")
bullet(u"**收到就回 `00 OK`，然后才开始通流**。")
bullet(u"`duration_ms` 到点**自行关断**（不用等 STOP），但**采样值要保留**（治具之后还会读 GET_ADC）。")
bullet(u"幂等：同样的参数重复收到 → 回 OK，不重起计时。")
bullet(u"上报电流时**不用管零点/中点偏置** —— 治具比的是“通流前 → 通流中”的变化量，两边偏置各自减掉了。"
       u"如实报你自己 ADC 换算出的**信号侧 mV** 即可。")
h3(u"⚠ 请务必同时实现 mode=1（低频 PWM）")
para(u"`mode=0`（静态全通）意味着 24V 直压两个绕组。产线上是“夹具提供限流回路”才这么干；"
     u"**台架/实验室里被测板接着三相电机时，静态全通等于短路**，会烧功率管或电机。"
     u"治具在台架上会优先用 `mode=1` + 小占空比（例如 20‰、100 ms）来测。"
     u"如果只实现 `mode=0`，这台架上的相电流测试就没法安全进行。", size=10)

# 7.8 GET_ADC
h2(u"7.8　0x0207 GET_ADC —— 读你自己采到的模拟量")
table([u"方向", u"DATA"], [
    [u"治具发", u"2 字节：`ch_mask`（大端 u16，bit i = 要通道 i）"],
    [u"你回", u"`status(1) + 每个通道 5 字节`"],
], widths=[2.4, 13.0], size=9.5)
para(u"**通道编号是定死的，必须与治具的 9 路信号序号一致**：", size=10)
table([u"ch", u"0", u"1", u"2", u"3", u"4", u"5", u"6", u"7", u"8"], [
    [u"信号", u"24V 母线", u"10V 驱动", u"5V", u"3.3V", u"A 相电压", u"B 相电压", u"C 相电压",
     u"电流检测1", u"电流检测2"],
], widths=[1.2, 1.6, 1.6, 1.0, 1.2, 1.7, 1.7, 1.7, 1.8, 1.8], size=8.5)
para(u"每个通道 5 字节：", size=10)
table([u"off", u"长度", u"字段", u"说明"], [
    [u"0", u"1", u"ch", u"通道号（**必须回原通道号**）"],
    [u"1..2", u"2", u"raw", u"你自己的 ADC 原始码值"],
    [u"3..4", u"2", u"mv", u"你自己换算出的 mV，按**信号侧**算（见下）"],
], widths=[1.3, 1.1, 2.0, 11.0], size=9)
code(u"治具发  " + frame(0x0207, 8, "0003") + u"          ← 要 ch0 + ch1\n"
     u"你回    状态00 + ch0(00 | raw 081A | mv 012C) + ch1(01 | raw 0830 | mv 0138)")
code(u"治具发  " + frame(0x0207, 12, "0180") + u"          ← 只要 ch7 + ch8（两路电流）\n"
     u"你回    状态00 + ch7(...) + ch8(...)")
h3(u"⚠ `mv` 的参考点（最容易对不上的地方）")
para(u"治具报的是**信号侧 mV**（已经折回分压之前的值）。相电压那几路治具的分压比是 **k = 0.125**"
     u"（24V → 3V 进 ADC）。如果你报“ADC 引脚上的 mV”，会比治具的数小 8 倍，两边就对不上。"
     u"请统一成信号侧，并在代码里注释清楚。", size=10)
bullet(u"**`raw` 和 `mv` 都要给**：治具靠这两个分辨“是采样链路坏了”还是“换算系数写错了”。")
bullet(u"一次最多问 12 个通道（受单帧 64 字节限制）。你没在 `adc_mask` 里声明的通道不会被问。")

# 7.9 GET_ENC
h2(u"7.9　0x0208 GET_ENC —— 读你自己的编码器计数")
table([u"方向", u"DATA"], [
    [u"治具发", u"0 字节"],
    [u"你回", u"**14 字节**"],
], widths=[2.4, 13.0], size=9.5)
table([u"off", u"长度", u"字段", u"说明", u"示例"], [
    [u"0", u"1", u"status", u"`00`", u"`00`"],
    [u"1..4", u"4", u"cnt_a", u"A 相**边沿个数**，u32 大端", u"`00 00 03 E8` = 1000"],
    [u"5..8", u"4", u"cnt_b", u"B 相同上", u"`00 00 07 D0` = 2000"],
    [u"9..12", u"4", u"cnt_z", u"Z（索引）同上", u"`00 00 00 01` = 1"],
    [u"13", u"1", u"dir", u"最近一次解码方向：`01` / `FF` / `00`（有符号 s8）", u"`01` = +1"],
], widths=[1.3, 1.1, 2.0, 8.2, 3.0], size=9)
code(u"治具发  " + frame(0x0208, 9) + u"\n"
     u"你回    " + hx(build_frame(0x0208, 9,
                                   bytes.fromhex("00" "000003E8" "000007D0" "00000001" "01"))))
h3(u"计数语义（一定要这样，否则比不了）")
bullet(u"数的是**边沿个数**（上升沿 + 下降沿），**不是**解码后的位置值 —— 治具发 1 个完整方波 = 2 个边沿。")
bullet(u"**进入测试模式时清零**，之后**只增不减**。")
bullet(u"`dir` 只是方向指示，**不参与计数**。")
bullet(u"治具发的默认脉冲串是：A = 1000 个 @5 kHz、B = 2000 个 @10 kHz、Z = 1 个。"
       u"三路个数**故意各不相同**，这样接错线时能立刻看出是哪一根。")

# ── 8 ──
h1(u"8　“功能声明”字段：告诉治具你实现了哪几条命令")
para(u"这一章只解决一个问题：**治具怎么知道你的固件做到哪一步了**。"
     u"协议里这个字段叫 `caps`（capabilities 的缩写），本文章统一叫它**功能声明**。", size=10)

h2(u"8.1　它是干什么的（一句话）")
para(u"治具开始测之前，会先发一条 INFO 问你一句“你能干什么”。"
     u"你的回答是一个 **16 位整数**，每一位代表一条命令：", size=10)
bullet(u"这一位是 **1** = 这条命令我实现了，你可以调用；")
bullet(u"这一位是 **0** = 这条命令我没实现，**别调用我**。")
para(u"治具拿到这个数之后，**只测你声明为 1 的项**；声明为 0 的直接跳过，"
     u"在报告里标成“不判”，**不会判你失败**。", size=10)
para(u"所以规则只有两条：**① 只声明你真正实现了的；② 声明为 1 的命令必须真的能回 00，不能回 FF。**", size=10)

h2(u"8.2　每一位对应哪条命令")
table([u"命令", u"命令码", u"对应位", u"这一位单独是 1 时的值", u"不实现会怎样"], [
    [u"PING", u"0x0200", u"bit0", u"0x0001", u"**必做**"],
    [u"INFO", u"0x0201", u"bit1", u"0x0002", u"**必做**"],
    [u"TEST_MODE", u"0x0202", u"bit2", u"0x0004", u"**必做**"],
    [u"STOP", u"0x0203", u"bit3", u"0x0008", u"**必做**"],
    [u"PWM", u"0x0204", u"bit4", u"0x0010", u"②③ 测不了"],
    [u"LEVEL", u"0x0205", u"bit5", u"0x0020", u"不影响（可选）"],
    [u"CURRENT", u"0x0206", u"bit6", u"0x0040", u"⑤ 测不了"],
    [u"GET_ADC", u"0x0207", u"bit7", u"0x0080", u"① 测不了"],
    [u"GET_ENC", u"0x0208", u"bit8", u"0x0100", u"④ 测不了"],
], widths=[2.8, 2.2, 1.6, 4.4, 4.4], size=9.5)

h2(u"8.3　怎么把这个数算出来（三步）")
bullet(u"**第 1 步**：对着 8.2 的表，把你要支持的那几行挑出来 —— 必做那 4 行**一定**都有。")
bullet(u"**第 2 步**：把它们“这一位单独是 1 时的值”按十六进制**加起来**。"
       u"例如 4 条必做 = 0x0001 + 0x0002 + 0x0004 + 0x0008 = **0x000F**。")
bullet(u"**第 3 步**：把结果填进 INFO 应答的**第 6..7 字节**，**高字节在前**"
       u"（0x000F 写成 `00 0F`，0x03FF 写成 `03 FF`）。")

h2(u"8.4　五个现成的例子（挑最接近你情况的那个照抄）")
para(u"下面的“INFO 应答 16 字节”就是要回给治具的完整 DATA；"
     u"“线上完整帧”是套上帧头和 CRC 之后真正跑的字节。**CRC 已按协议算好，可直接抓包对拍。**", size=10)
h3(u"例 A：只做 4 条必做项（还不会输出 PWM，也不报任何模拟量）")
table([u"caps 填", u"adc_mask 填"], [[u"0x000F", u"0x0000"]], widths=[4.0, 4.0], size=10)
code(u"INFO 应答 16 字节  00 01 01 01 00 00 00 0F 00 00 00 00 00 00 00 00\n"
     u"线上完整帧      A5 5B 02 01 00 02 00 10 00 01 01 01 00 00 00 0F 00 00 00 00 00 00 00 00 8B 24")
h3(u"例 B：4 条必做 + PWM（能测 ② 脉冲、③ 三相输出）")
table([u"caps 填", u"adc_mask 填"], [[u"0x001F", u"0x0000"]], widths=[4.0, 4.0], size=10)
code(u"INFO 应答 16 字节  00 01 01 01 00 00 00 1F 00 64 4E 20 00 00 00 00\n"
     u"线上完整帧      A5 5B 02 01 00 03 00 10 00 01 01 01 00 00 00 1F 00 64 4E 20 00 00 00 00 58 7F")
h3(u"例 C：4 条必做 + PWM + GET_ADC + CURRENT（能测 ②③⑤ 和 ①；只报两路电流）")
table([u"caps 填", u"adc_mask 填"], [[u"0x00DF", u"0x0180"]], widths=[4.0, 4.0], size=10)
code(u"INFO 应答 16 字节  00 01 01 01 00 00 00 DF 00 64 4E 20 01 80 00 00\n"
     u"线上完整帧      A5 5B 02 01 00 04 00 10 00 01 01 01 00 00 00 DF 00 64 4E 20 01 80 00 00 3D 9A")
h3(u"例 D：4 条必做 + GET_ADC + GET_ENC（能测 ① ④；9 路模拟量全报）")
table([u"caps 填", u"adc_mask 填"], [[u"0x018F", u"0x01FF"]], widths=[4.0, 4.0], size=10)
code(u"INFO 应答 16 字节  00 01 01 01 00 00 01 8F 00 00 00 00 01 FF 00 00\n"
     u"线上完整帧      A5 5B 02 01 00 05 00 10 00 01 01 01 00 00 01 8F 00 00 00 00 01 FF 00 00 30 90")
h3(u"例 E：九条全做（推荐目标；治具五项测试都能跑）")
table([u"caps 填", u"adc_mask 填"], [[u"0x03FF", u"0x01FF"]], widths=[4.0, 4.0], size=10)
code(u"INFO 应答 16 字节  00 01 01 01 00 00 03 FF 00 64 4E 20 01 FF 00 00\n"
     u"线上完整帧      A5 5B 02 01 00 06 00 10 00 01 01 01 00 00 03 FF 00 64 4E 20 01 FF 00 00 93 BB")
para(u"**怎么选**：要测 ②③ 就必须有 PWM；要测 ① 就必须有 GET_ADC；"
     u"要测 ⑤ 就必须有 CURRENT **和** GET_ADC（它要读电流两路）；要测 ④ 就必须有 GET_ENC。"
     u"拿不准就先按**例 A** 发出去 —— 把 PWM / GET_ADC 做完再往上加，"
     u"治具每次连上都会重新问你一遍，不用改协议。", size=10)

h2(u"8.5　`adc_mask` 是同一套写法（但它管的是“哪几路模拟量”）")
para(u"`adc_mask` 也是位图，只是每一位对应**一路模拟量**（不是命令）。"
     u"bit i = 1 表示“这一路我能采、能报”。**没声明的通道治具根本不会问**，"
     u"所以少声明是安全的；但声明了就必须真的回得上那一路。", size=10)
table([u"ch", u"信号", u"这一位单独是 1 时的值", u"备注"], [
    [u"0", u"24V 母线", u"0x0001", u"③ 的母线电压判据要用"],
    [u"1", u"10V 驱动", u"0x0002", u""],
    [u"2", u"5V", u"0x0004", u""],
    [u"3", u"3.3V", u"0x0008", u""],
    [u"4", u"A 相电压", u"0x0010", u"③ 要用；必须是“信号侧 mV”（见 7.8）"],
    [u"5", u"B 相电压", u"0x0020", u"同上"],
    [u"6", u"C 相电压", u"0x0040", u"同上"],
    [u"7", u"电流检测1", u"0x0080", u"⑤ 要用"],
    [u"8", u"电流检测2", u"0x0100", u"⑤ 要用"],
], widths=[1.2, 3.0, 4.6, 6.6], size=9.5)
para(u"举例：只报 24V 母线和 5V → 0x0001 + 0x0004 = **0x0005**；"
     u"只报两路电流 → 0x0080 + 0x0100 = **0x0180**；9 路全报 → **0x01FF**；"
     u"完全不实现 GET_ADC → **0x0000**（这时 `caps` 的 bit7 也必须是 0）。", size=10)

h2(u"8.6　发布前自查两句话")
bullet(u"`caps` 里置 1 的命令，挨个发一遍，必须都回 `00`（不能回 `FF`）。")
bullet(u"`adc_mask` 里置 1 的通道，`GET_ADC` 必须真的能回那一路（每路 5 字节，通道号原样回）。")

# ── 9 ──
h1(u"9　治具的五项测试分别依赖哪条命令")
para(u"这张表说明“为什么 9 条都要”：任何一条缺失，对应那项功能就测不了。", size=10)
table([u"治具的测试项", u"用到的 DUT 命令", u"治具侧怎么判"], [
    [u"① 电压检测", u"INFO + GET_ADC", u"治具 9 路 ADC 与你的 mv **逐通道比对**，容差 max(5%, 200 mV)"],
    [u"② MCU 脉冲检测", u"TEST_MODE + PWM", u"治具用自己的定时器捕获这三路 PWM：量周期、占空比、边沿数"],
    [u"③ 三相输出检测", u"TEST_MODE + PWM", u"治具量相电压，判 **V ≈ 占空比 × 母线电压**"],
    [u"④ 编码器检测", u"TEST_MODE + GET_ENC", u"治具发定量脉冲，比“预期边沿数 vs 你数到的”"],
    [u"⑤ 相电流检测", u"TEST_MODE + CURRENT + GET_ADC", u"治具读基线 → 通流 → 再读，比**变化量**（两路电流）"],
], widths=[3.0, 4.4, 8.0], size=9)

# ── 10 ──
h1(u"10　对接与验收自测清单")
para(u"建议按这个顺序做，每做完一条就能往下走：", size=10)
table([u"步", u"做什么", u"判据"], [
    [u"1", u"CRC16(\"123456789\") == 0x4B37", u"算法对了"],
    [u"2", u"PING 回显", u"帧格式、接线、波特率全对"],
    [u"3", u"INFO 回 16 字节", u"功能声明与 adc_mask 能被解析（第 8 章）"],
    [u"4", u"未实现的命令号回 0xFF", u"异常处理规范"],
    [u"5", u"TEST_MODE 进入 → 回 00", u"测试模式可用"],
    [u"6", u"停手 1 秒以上", u"**三相输出必须自己关断**（看门狗）"],
    [u"7", u"PWM 1 kHz/50%/300 个", u"示波器上看到三相 1 kHz、50%"],
    [u"8", u"STOP → 输出全关", u"安全命令可用"],
    [u"9", u"GET_ENC 回 14 字节", u"计数语义正确"],
], widths=[0.9, 8.0, 6.5], size=9.5)

h2(u"10.1　用串口助手照抄着发（115200 8N1，HEX 模式）")
para(u"下面每一行“发”都能直接粘到串口助手的发送框；命令号和序号会原样回来。", size=10)
cases = [
    (u"自测 1) PING 回环（应收到**完全相同**的一帧）", 0x0200, 1, "DEADBEEF",
     hx(build_frame(0x0200, 1, bytes.fromhex("DEADBEEF"))),
     u"回显必须一模一样"),
    (u"自测 2) INFO（示例：9 条命令全支持、9 路 ADC 全支持）", 0x0201, 2, "",
     hx(build_frame(0x0201, 2, bytes.fromhex("00010101000003FF00644E2001FF0000"))),
     u"回 16 字节；数字随你自己的实现变"),
    (u"自测 3) 进测试模式（wdt=1000ms）", 0x0202, 3, "0103E8",
     hx(build_frame(0x0202, 3, b"\x00")),
     u"回 00 OK"),
    (u"自测 4) PWM 1kHz / 50% / 300 个（示波器上应看到三相 1kHz、50%）", 0x0204, 5,
     "03E801F401F401F4012C", hx(build_frame(0x0204, 5, b"\x00")),
     u"**收到就回，不要等发完**"),
    (u"自测 5) 关输出", 0x0203, 4, "", hx(build_frame(0x0203, 4, b"\x00")),
     u"任何状态都必须能停"),
    (u"自测 6) 读编码器计数", 0x0208, 9, "",
     hx(build_frame(0x0208, 9, bytes.fromhex("00" "000003E8" "000007D0" "00000001" "01"))),
     u"回 14 字节；数字随你实际编码器变"),
    (u"自测 7) 一条未实现的命令（0x0209）", 0x0209, 10, "",
     hx(build_frame(0x0209, 10, b"\xff")),
     u"不认识的命令必须回 FF，不能不回"),
]
for title, cmd, seq, hexdata, back, note in cases:
    h3(title)
    code(u"发  " + hx(build_frame(cmd, seq, bytes.fromhex(hexdata))) +
         u"\n回  " + back)
    para(u"→ " + note, size=9.5)
para(u"**最后一步**：发完 PWM 后**停手 1 秒以上**，三相输出必须自己关断 —— 这是看门狗在起作用，也是安全底线。", size=10)

# ── 11 ──
h1(u"11　我们还需要你确认/提供的参数")
table([u"#", u"要什么", u"用在哪"], [
    [u"1", u"你的 UART 是哪一路 USART、哪两个引脚", u"治具接线要对着这个来"],
    [u"2", u"能否稳定跑 115200 8N1（不能就给具体值）", u"治具侧改一处配置"],
    [u"3", u"相电流采样量程：采样电阻 / 放大倍数 / 基准，满量程对应多少安", u"现在只能比 mV，换算成安培需要这个"],
    [u"4", u"编码器接口形式 + 高电平门限（治具侧是 3.3V 推挽）", u"若你是 5V 逻辑且门限 3.5V，治具侧要改成开漏"],
    [u"5", u"GET_ADC 的 mv 参考点（是否已折回分压前）", u"不统一两边数字没法直接比"],
    [u"6", u"烧完固件后会不会自动开始驱动电机", u"若是，测试流程要先按住它"],
    [u"7", u"有没有现成的 OTA/UART 协议框架可复用", u"有的话你少写一半"],
], widths=[0.8, 8.2, 6.4], size=9.5)

# ── 12 ──
h1(u"12　常见错误清单（照这个自查）")
table([u"#", u"错误做法", u"后果"], [
    [u"1", u"没进测试模式就尝试输出 PWM / 通流", u"治具以为你在测，实际功率级没响应 → 判不合格。**请回 01 BUSY**"],
    [u"2", u"PWM 应答等到脉冲发完才回", u"治具的捕获窗口会晚开，采不到波形 → 判“无信号”"],
    [u"3", u"发 PWM 时 `count/freq` 太短（< 0.25 s）", u"窗口被截尾，周期/占空比算不准"],
    [u"4", u"`seq` 没有原样回显", u"治具配对不上，只能超时重试"],
    [u"5", u"CRC 算错范围（把 CRC 自己也算进去）", u"整帧被丢弃，且治具不会收到任何提示"],
    [u"6", u"`duty_permille` 用百分比（0~100）而不是千分比（0~1000）", u"占空比差 10 倍"],
    [u"7", u"`mv` 报的是 ADC 引脚电压而不是信号侧", u"相电压差 8 倍，① 电压检测全超差"],
    [u"8", u"编码器只数单沿", u"④ 的计数只有期望的一半"],
    [u"9", u"看门狗能被 `wdt_ms=0` 关掉", u"治具掉线时电机继续转 —— **安全事故**"],
    [u"10", u"进入测试模式时没有关输出就停控制环", u"功率级可能保持上一次的输出状态"],
], widths=[0.8, 7.6, 7.0], size=9.5)

# ── 结尾 ──
doc.add_paragraph()
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
set_run_font(p.add_run(u"— 完 —"), size=10)
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
set_run_font(p.add_run(u"本文档所有示例帧的 CRC 均由脚本按 CRC-16/MODBUS 现算并校验；"
                       u"与治具固件使用同一套帧编解码实现。"), size=9)

doc.save(OUT)
print("saved: %s (%d bytes)" % (OUT, os.path.getsize(OUT)))
