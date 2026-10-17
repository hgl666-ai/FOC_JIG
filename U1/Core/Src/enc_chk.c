/**
  ******************************************************************************
  * @file    enc_chk.c
  * @brief   ⑤ 编码器检测判据层的实现（纯函数，不碰硬件、不碰 HAL）
  *
  * 为什么把判据单独拆一层（和 vol_chk / phase_chk / cur_chk 同一个理由）：
  *   判据要能被**宿主单测穷举**（给任意三个数就该得出确定的结论），
  *   而"发脉冲 / 读 U2 / 问 DUT"那部分是有状态的流程，两件事分开才好测。
  ******************************************************************************
  */
#include "enc_chk.h"

/* 绝对值差（有符号 → 无符号，防 INT32_MIN 取负溢出） */
static int32_t diffs32(int32_t a, int32_t b)
{
    int32_t d = a - b;

    return (d < 0) ? (int32_t)(0 - (uint32_t)d) : d;
}

uint16_t EncChk_EdgesFromPulses(uint16_t pulses)
{
    if (pulses > 32767u) {
        return 0xFFFFu;                  /* 饱和：再大 u16 表达不了 2× */
    }
    return (uint16_t)(pulses * 2u);
}

uint16_t EncChk_Judge(const enc_chk_in_t *in, uint32_t tol, enc_chk_out_t *out)
{
    uint16_t flags = 0u;
    int32_t  eu = 0;
    int32_t  ed = 0;

    if (in == 0) {
        return 0u;
    }

    if (in->has_u2 != 0u) {
        eu = (int32_t)in->u2_edges - (int32_t)in->exp_edges;
    }
    if (in->has_dut != 0u) {
        ed = (int32_t)in->dut_edges - (int32_t)in->exp_edges;
    }

    if ((in->has_u2 == 0u) && (in->has_dut == 0u)) {
        flags = ENC_CHK_F_NODATA;                     /* 两边都没数据：什么都不判 */
    } else if ((in->has_u2 == 0u) || (in->has_dut == 0u)) {
        /* 只有一侧：**不能**判合格（缺的那条腿可能正是坏的地方），
         * 但有数据的那一侧如果对不上，照样把故障指出来。 */
        flags |= ENC_CHK_F_NODATA;
        if ((in->has_u2 != 0u) && ((uint32_t)diffs32(eu, 0) > tol)) {
            flags |= ENC_CHK_F_U2_DIFF;
        }
        if ((in->has_dut != 0u) && ((uint32_t)diffs32(ed, 0) > tol)) {
            flags |= ENC_CHK_F_DUT_DIFF;
        }
    } else {
        if (((uint32_t)diffs32(eu, 0) > tol)) {
            flags |= ENC_CHK_F_U2_DIFF;
        }
        if (((uint32_t)diffs32(ed, 0) > tol)) {
            flags |= ENC_CHK_F_DUT_DIFF;
        }
        if (flags == 0u) {
            flags = ENC_CHK_F_OK;
        }
    }

    if (out != 0) {
        out->err_u2    = eu;
        out->err_dut   = ed;
        out->exp_edges = in->exp_edges;
        out->u2_edges  = in->u2_edges;
        out->dut_edges = in->dut_edges;
        out->flags     = flags;
    }
    return flags;
}

uint16_t EncChk_All(const enc_chk_in_t in[ENC_CHK_PH_COUNT], uint32_t tol,
                    uint16_t u2_lost, int8_t dut_dir,
                    enc_chk_out_t out[ENC_CHK_PH_COUNT])
{
    uint16_t sum = ENC_CHK_SUM_DONE;
    uint8_t  i;
    uint32_t pulses_any = 0u;

    if (in == 0) {
        return 0u;
    }

    for (i = 0u; i < (uint8_t)ENC_CHK_PH_COUNT; i++) {
        enc_chk_out_t tmp;
        uint16_t f = EncChk_Judge(&in[i], tol, &tmp);

        if ((f & (ENC_CHK_F_U2_DIFF | ENC_CHK_F_DUT_DIFF)) != 0u) {
            sum |= ENC_CHK_SUM_ANY_BAD;
        }
        if ((f & ENC_CHK_F_NODATA) != 0u) {
            sum |= ENC_CHK_SUM_ANY_NODATA;
        }
        if (in[i].exp_edges != 0u) {
            pulses_any = 1u;
        }
        if (out != 0) {
            out[i] = tmp;
            out[i].flags = f;
        }
    }

    if (u2_lost != 0u) {
        sum |= ENC_CHK_SUM_U2_LOST;
    }
    /* "发了脉冲但 DUT 报方向 = 没动"：正交解码的方向这一路没工作。
     * 只当**警告**（某些实现只在 A/B 都动时才更新方向，Z 单独跳不更新）。 */
    if ((pulses_any != 0u) && (dut_dir == 0)) {
        sum |= ENC_CHK_SUM_DIR_ZERO;
    }
    return sum;
}

/* ⚠ 这两个函数**必须只返回 ASCII**：ARMCC 5.06 把 UTF-8 源文件里的中文字符串
 *    字面量解析成一堆 "#8: missing closing quote"，直接编译失败（注释里的中文没事）。
 *    本项目固件里没有任何中文**字符串字面量**，这里也照这个规矩来。 */
const char *EncChk_FlagText(uint16_t flags)
{
    if ((flags & ENC_CHK_F_U2_DIFF) != 0u) {
        return "U2_DIFF";
    }
    if ((flags & ENC_CHK_F_DUT_DIFF) != 0u) {
        return "DUT_DIFF";
    }
    if ((flags & ENC_CHK_F_NODATA) != 0u) {
        return "NODATA";
    }
    if ((flags & ENC_CHK_F_OK) != 0u) {
        return "OK";
    }
    return "?";
}

const char *EncChk_SumText(uint16_t sum)
{
    if ((sum & ENC_CHK_SUM_ANY_BAD) != 0u) {
        return "BAD";
    }
    if ((sum & ENC_CHK_SUM_ANY_NODATA) != 0u) {
        return "NODATA";
    }
    if ((sum & ENC_CHK_SUM_DONE) != 0u) {
        return "PASS";
    }
    return "?";
}
