/**
  ******************************************************************************
  * @file    jig_as.h
  * @brief   治具 ↔ FOC 板之间的 **ASCII 文本命令**（一个字符串都不许散落在别处）
  *
  * 为什么单独一个头：这些字符串是两边约定的接口，FOC 那边改一个字，治具就全挂。
  * 全部集中在这里，改一处即可；`docs/24_FOC侧AS命令规格.md` 是同一份约定的正文。
  *
  * 约定（与 docs/24 一致）：
  *   · 命令**不带** `\r\n`（与已验证可用的 `AS:S800` / `AS:S0` 同一条路径）
  *   · FOC 回字带不带 `\r\n` 都行，治具解析时忽略 `\r\n`、空格、大小写
  *   · 未知命令对面不该回任何东西 → 治具靠"没回 = 超时"判失败
  ******************************************************************************
  */
#ifndef __JIG_AS_H
#define __JIG_AS_H

/* ── 治具 → FOC ─────────────────────────────────────────────────────────── */
#define AS_CMD_RUN        "AS:S800" /* 转（FOC 一直有的命令，台架手工用） */
#define AS_CMD_VERSION    "AS:Bb"   /* ② 问版本（顺便证明它还活着） */
#define AS_CMD_SELFTEST   "AS:CS"   /* ③ 自检 + 起转（A0 后 800rpm），回 AS:A0~A5 */
#define AS_CMD_STOP       "AS:S0"   /* 停电机 ⚠ 是 AS:S0，不是 AS:S0 */
#define AS_CMD_PWM        "AS:PW"   /* ④ PA8/9/10 三相 50%，驱动不使能；先回 AS:P0 */
#define AS_CMD_PWX        "AS:PX"   /* ④ 退出 PW，回 AS:P1 */
#define AS_CMD_ENC        "AS:MC"   /* ⑤ 进"数编码器"模式；先回 AS:MC0 */
#define AS_CMD_ME         "AS:ME"   /* ⑤ 提前结束 MC/MQ，对面回数据帧 */

/* ── FOC → 治具 ─────────────────────────────────────────────────────────── */
/* 版本号回字的前缀。**实测（2026-11-11）FOC 回的是 `bb:0.3.0`** ——
 * 既没有 `AS:`、也没有 `V`。所以两个前缀都认：
 *   AS_PREFIX_VERSION  = "bb:"    对面当前实际发的
 *   AS_PREFIX_VERSION2 = "AS:V"   我们原来约定的（对面若改了回法也不会挂）
 * 匹配逻辑见 APP_FlowTestTask 的 FL6_S_ALIVE 分支。*/
#define AS_PREFIX_VERSION  "bb:"
#define AS_PREFIX_VERSION2 "AS:V"
#define AS_CODE_OK        "AS:A0"   /* 自检通过 **且电机已经转起来** */
#define AS_CODE_ERR_ADC   "AS:A1"   /* ADC 校准失败 */
#define AS_CODE_ERR_ENC   "AS:A2"   /* 编码器校准失败 */
#define AS_CODE_ERR_PHASE "AS:A3"   /* 相位缺失 */
#define AS_PREFIX_ENCDATA "AS:D"    /* AS:D,A=25,B=83,Z=0 —— 编码器计数上报
                                     *   （FOC 实际格式；逗号 + 键值对 + 十进制） */

/* ── 控制台的几条固定提示（给人看的，不是协议） ─────────────────────────── */
#define AS_MSG_BUSY       "\r\n[JIG] busy, try again\r\n"
#define AS_MSG_NO_REPLY   "\r\n[JIG] no reply from FOC\r\n"

/* ── 时序参数 ───────────────────────────────────────────────────────────── */
#define AS_LINE_MAX       32u    /* 一行命令最长字节数（不含结束符） */
#define AS_LINE_IDLE_MS   50u    /* 没有结束符时：静默这么久就当这行发完了 */
#define AS_REPLY_MS      2000u   /* 发完命令后最多等这么久收回字
                                  * ⚠ 实测：`AS:MC` 要约 1.5~2 秒才回 `AS:MC0`，
                                  * 300ms 会先报一句"no reply"，很误导。
                                  * 快命令不受影响：收到字节后静默 30ms 就提前收尾。*/
#define AS_REPLY_GAP_MS   30u    /* 已经收到字节，静默这么久就算收完（提前结束） */

/* 命令尾部要不要追加 \r\n：默认不追加（见文件头约定）。
 * 万一对面把新命令写成"必须收满一行"，把它改成 1 重编即可。 */
#ifndef JIG_AS_APPEND_CRLF
#define JIG_AS_APPEND_CRLF 0
#endif

#endif /* __JIG_AS_H */
