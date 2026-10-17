/**
  ******************************************************************************
  * @file    jig_mode.h
  * @brief   治具**运行模式**与**所有编译开关**的唯一入口（生产 / 调试统一管理）
  *
  * 用法：**只改一个宏**（在下面 §2），两片一起重编重烧：
  *   JIG_MODE_DEBUG       台架调试：判定放宽/关掉、调试打印开、诊断全开
  *   JIG_MODE_PRODUCTION  产线生产：判定全开（规格门限 + 交叉校验 + 脉冲判据 + U2 必需）
  *
  * 规矩：
  *   - 加新开关**先想能不能用 JIG_MODE 表达**；确实需要单点控制才另加宏。
  *   - 每个开关都留了 #ifndef，允许在 Keil 的 C/C++ → Define 里临时覆盖
  *     （例如只对某一次台架试验打开打印，而不动这个文件）。
  *   - 改这个文件 = 改两片的对外行为 → 两片必须一起重烧；
  *     回归脚本第 8 阶段用 fc /b 逐字节比对，**改了 U1 忘了改 U2 会直接判失败**。
  *   - 两片的差异（U1 有调试串口、U2 没有）通过各工程的 Keil Define 表达，
  *     本文件在两片里保持逐字节一致。
  *
  * 相关文档：docs/05 §2.6（开关汇总表）、docs/07（测量）、docs/12（脉冲捕获）
  ******************************************************************************
  */
#ifndef __JIG_MODE_H
#define __JIG_MODE_H

/*==============================================================================
 * 1. 板级差异（由各工程的 Keil Define 提供，不要在这里改）
 *============================================================================*/
#ifndef JIG_HAS_DEBUG_UART
#define JIG_HAS_DEBUG_UART   0      /* U1 工程 Define 里置 1（只有它有 bsp_uart.c） */
#endif

/*==============================================================================
 * 2. 运行模式 —— 出厂/产线前只需改这一行
 *============================================================================*/
#define JIG_MODE_DEBUG       0
#define JIG_MODE_PRODUCTION  1

#ifndef JIG_MODE
#define JIG_MODE   JIG_MODE_DEBUG   /* ← 量产前改成 JIG_MODE_PRODUCTION（两片都要改）*/
#endif

/*==============================================================================
 * 3. 由模式派生的开关（都可以单独覆盖）
 *============================================================================*/

/* --- 3.1 判定：U2（测量/校验片）没应答时是否直接判测试失败 ----------------
 * 调试期关：台架上可能只焊了 U1，或 U2 还没烧固件；
 * 产线期开：U2 不在 = 治具不完整，不能放行。 */
#ifndef MEAS_CC_REQUIRED
#define MEAS_CC_REQUIRED     ((JIG_MODE == JIG_MODE_PRODUCTION) ? 1 : 0)
#endif

/* --- 3.2 判定：脉冲捕获是否参与判定 --------------------------------------
 * 判据：输出端与输入端对不上（一端有信号另一端没有 / 周期差 > PULSE_PEER_TOL_PPM）
 *       或两端都没信号 → ERR_TEST。
 * 调试期关：先只把测量结果与差异标记放进应答，方便看清现象、标定容差。 */
#ifndef PULSE_JUDGE_ENABLE
#define PULSE_JUDGE_ENABLE   ((JIG_MODE == JIG_MODE_PRODUCTION) ? 1 : 0)
#endif

/* --- 3.3 判定：电压/电流门限 --------------------------------------------
 * 0 = 全量程（等于"先不判"，只有超量程/贴边才看得出来）；
 * 1 = 用规格门限（measure.c 的 k_lim_min/k_lim_max，数值待硬件确认）。 */
#ifndef MEAS_THRESHOLDS_ENABLED
#define MEAS_THRESHOLDS_ENABLED ((JIG_MODE == JIG_MODE_PRODUCTION) ? 1 : 0)
#endif

/* --- 3.4 DUT 串口（USART1）占用开关 --------------------------------------
 * U1 的 USART1（PA9/PA10）有**两个用途，只能二选一**：
 *   - **DUT 链路**：跟被烧 FOC 板通信（任务 ①③④⑤⑥ 要用，规范见 docs/15）
 *   - **调试打印**：SWD 烧录/测量的打点输出（`bsp_uart.c`）
 * 同时开的话，调试字符会灌到被测板的串口线上，被测板收到的是垃圾帧。
 *
 * 由各工程的 Keil Define 提供（和 JIG_HAS_DEBUG_UART 同一套路），
 * **默认 0**：不接 DUT 时保持现在的行为不变。
 * 下面有一条 `#error` 做机械保证 —— 两个都开直接编译不过，不靠人记。 */
#ifndef JIG_DUT_LINK
#define JIG_DUT_LINK         0
#endif

/* --- 3.5 调试打印（走 U1 的 USART1 / PA9）--------------------------------
 * U2 没有调试串口 → 恒为 0（否则 measure.c 会去引用不存在的 bsp_uart）。
 * **JIG_DUT_LINK=1 时强制关闭**（见 §3.4）。 */
#ifndef MEAS_PRINT_ENABLE
#define MEAS_PRINT_ENABLE    (((JIG_MODE == JIG_MODE_DEBUG) && (JIG_HAS_DEBUG_UART != 0) && (JIG_DUT_LINK == 0)) ? 1 : 0)
#endif
#ifndef BSP_DBG_ENABLE
#define BSP_DBG_ENABLE       MEAS_PRINT_ENABLE
#endif
#ifndef MEAS_PRINT_PERIOD_MS
#define MEAS_PRINT_PERIOD_MS 1000u
#endif

/* USART1 二选一的机械保证：两个都开 = 编译期报错 */
#if (JIG_DUT_LINK != 0) && (BSP_DBG_ENABLE != 0)
#error "JIG_DUT_LINK=1 时不能开调试打印：两者都占用 U1 的 USART1（见 jig_mode.h 3.4/3.5）"
#endif

/* --- 3.6 标定项（与运行模式无关，标定完手动置 1）-------------------------
 * VDDA 默认写死 3300mV（LDO ±1%）；要用内部 VREFINT 跟踪漂移时：
 *   先用万用表量出本板 VREFINT 的真实值 → 填 MEAS_VREFINT_MV → 置 1。 */
#ifndef MEAS_USE_VREFINT
#define MEAS_USE_VREFINT     0
#endif
#ifndef MEAS_VREFINT_MV
#define MEAS_VREFINT_MV      1200u
#endif

#endif /* __JIG_MODE_H */
