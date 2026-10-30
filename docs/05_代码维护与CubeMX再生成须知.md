# 代码维护与 CubeMX 再生成须知

> 本文说明：**哪些改动改在了 CubeMX 生成代码上、重新生成会怎样、需要手工恢复什么**。
> 结论先说：**不要用 CubeMX 重新生成整个工程**；要改外设配置时，改完 `.ioc` 后按本文 §3 逐项核对。

---

## 1. 背景

`U1` 工程由 STM32CubeMX 生成，CubeMX 只保护 `/* USER CODE BEGIN ... */` 与 `/* USER CODE END ... */`
之间的内容。**其它区域、以及整个 `MDK-ARM/*.uvprojx`（Keil 工程文件），再生成时会被 CubeMX 覆盖。**

本工程在生成代码之外做了若干必要修改，因此再生成会丢东西。

---

## 2. 我在生成代码上改了什么

| 文件 | 改动 | 在 USER CODE 区? | 再生成后 |
|---|---|---|---|
| `Core/Src/gpio.c` | 删除 PA13 的 EXTI13 输入配置 | ❌ | **不会回退** ✅（见 §3.1） |
| `Core/Src/adc.c` | 单通道配置 → **10 通道扫描 + DMA1_CH1 循环搬运**（9 路模拟量 + VREFINT）；`MX_ADC1_Init()` 里的通道循环写在 `USER CODE BEGIN ADC1_Init 2` 内、DMA 初始化写在 `USER CODE BEGIN ADC1_MspInit 1` 内 → **这两块能保留** ✅；但 `hadc1.Init.ScanConvMode/ContinuousConvMode/NbrOfConversion` 与 `k_adc_scan[]` 在外面，**再生成会被改回单通道** ⚠ | 部分 | 见 §2.1 |
| `Core/Inc/adc.h` | 加 `extern DMA_HandleTypeDef hdma_adc1;`（在 USER CODE 区） | ✅ | 保留 ✅ |
| `Core/Src/stm32f1xx_hal_msp.c` | `HAL_MspInit()` 里按 `SWD_FREE_DEBUG_PINS` **条件编译二选一**：`1`（默认/生产）→ `__HAL_AFIO_REMAP_SWJ_DISABLE()`（`SWJ_CFG=100`，能烧 DUT、U1 不能调试）／`0`（开发）→ `__HAL_AFIO_REMAP_SWJ_NOJTAG()`（`SWJ_CFG=010`，能调 U1、不能烧 DUT） | ❌ | **不会回退** ✅（见 §3.1） |
| `Core/Src/stm32f1xx_it.c` | 加 `USB_LP_CAN1_RX0_IRQHandler` / `USB_HP_CAN1_TX_IRQHandler` | ✅ | 保留 ✅ |
| `Core/Src/main.c` | `GD25Q_Init()` / `APP_Init()` / `APP_Run()` | ✅ | 保留 ✅ |
| `Core/Src/main.c` | `MX_USB_PCD_Init()` → `MX_USB_DEVICE_Init()` | ❌ | **会被换回 `MX_USB_PCD_Init()`** ⚠ |
| `Core/Src/usb.c` + `usb.h` | **已删除** | ❌ | **会被重新生成** ⚠⚠ |
| `MDK-ARM/startup_stm32f103xb.s` | `Stack_Size 0x400 → 0x800` | ❌ | **回退到 0x400** ⚠ |
| `MDK-ARM/U1.uvprojx` | 追加 15 个源文件 + 4 条 Include 路径 | ❌ | **会被整表重写，手动加的文件全部丢失** ⚠⚠ |
| `U1.ioc` | PA13/PA14 改回 `SYS_JTMS-SWDIO`/`SYS_JTCK-SWCLK`，删除 `VP_SYS_VS_ND` | —— | 这是配置源，已改好 ✅ |

---

## 2.1 `adc.c` 与测量功能

`MX_ADC1_Init()` 现在是 **10 通道扫描**（9 路模拟量 + 内部基准 VREFINT）配 DMA1_CH1 循环搬运；
通道表 `k_adc_scan[]` 的顺序必须与 `measure.h` 里 `meas_id_t` 的顺序一致
（这两处靠 `MEAS_ADC_CH_xxx` 宏共用同一个 ADC 通道定义，但**顺序**只有注释约束——
改动任何一处都要同步改另一处）。

再生成后必须手工恢复的三点：
1. `hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;`、`ContinuousConvMode = ENABLE;`、
   `NbrOfConversion = MEAS_SCAN_COUNT;`
2. 加上 `k_adc_scan[]` 表（在 `USER CODE BEGIN 0` 区，会保留）与
   `USER CODE BEGIN ADC1_Init 2` 里的通道配置循环（会保留）
3. `USER CODE BEGIN ADC1_MspInit 1` 里的 DMA 初始化（会保留）

另外：**刻意没有使能 `DMA1_Channel1_IRQn`**——循环模式下 DMA 硬件自己重装计数器，
不需要软件参与，所以 `stm32f1xx_it.c` 里也没有对应的中断处理函数。
如果将来要用 `HAL_ADC_ConvCpltCallback`，必须同时补上中断处理函数，否则会进默认死循环。

---

## 2.2 SWD 烧录引脚（PA14/PA13）与 USART3 的关系

> **2026-10-27 定案**：新版治具原理图把 U1 自己的 SWD 口**独立引出去烧 DUT**了，所以
> **SWCLK = PA14（网络 `U1-SCLK`）/ SWDIO = PA13（网络 `U1-SWD`）**，
> `swd_config.h` 里 `SWD_CLK = GPIOA/GPIO_PIN_14/14u`、`SWD_DIO = GPIOA/GPIO_PIN_13/13u`、
> `SWD_PINS_CONFIRMED = 1`（详见 `docs/09`）。**旧值 PB10/PB11 → PB1/PB3 只是历史留档，别当成现状。**
>
> 代价与开关：这两根就是 U1 自己的调试脚，要把它们当普通 IO 用就必须关掉 U1 的 SW-DP。
> `swd_config.h` 的 **`SWD_FREE_DEBUG_PINS`（默认 1）** 决定 `stm32f1xx_hal_msp.c` 里二选一：
> - `1`（生产）→ `__HAL_AFIO_REMAP_SWJ_DISABLE()`（AFIO `MAPR.SWJ_CFG = 100`）：PA13/PA14 变普通 GPIO，
>   **能烧 DUT，但 U1 自己的 SWD 调试/下载失效**；
> - `0`（开发）→ `__HAL_AFIO_REMAP_SWJ_NOJTAG()`（`SWJ_CFG = 010`）：保留 U1 的调试口，
>   但**不能烧 DUT**（只有 `SWJ_CFG=100` 才会把 PA13/PA14 释放成普通 IO）。
>
> 附带效果：`SWJ_CFG = 100` 时 **PA15 / PB3 / PB4 一并变成普通 IO**，其中 **PB4 = TIM3_CH1（B 相捕获）**，
> 因此不再依赖"关闭 NJTRST 才能用 PB4"。
> **不用 NRST**（`SWD_USE_NRST 0`）：目标板复位走 SWD 的 `AIRCR.SYSRESETREQ`，NRST 仍未引出。
>
> 救砖 / 升级 4 条路（推荐顺序）：
> ① **短接板上 H1（BOOT0↔3.3V）→ 复位 → 进系统 bootloader → 用 USART3 两针烧**（首选）；
> ② 调试器 Connect-under-Reset（NRST 未引出，需手扶复位）；
> ③ `SWD_FREE_DEBUG_PINS` 置 0 重编一版（能调 U1、不能烧 DUT）；
> ④ 全擦后 `SWJ_CFG` 复位成 000，调试口自动可用。

现在 U1 的 USART3 是**到 U2 的内部链路**（见 `docs/10_UART四路驱动设计.md`）。

- **ISP 方案已整体退役**（源码与测试全部删除，留存说明见 `docs/02` 附录 A）：
  老方案是"ISP 走 USART3 直连被测板"，和现在这条链路**抢同一个串口**，
  所以 `bsp_uart.c` 现在只做**调试打印**（`BSP_DBG_Send`），开关 `BSP_DBG_ENABLE`
  由 `jig_mode.h` 按模式派生，U1 工程另外用 Keil Define `JIG_HAS_DEBUG_UART=1` 打开。
  调试打印和 DUT 串口**二选一**：同时开会把调试字符灌到 DUT 串口线上。
- `swd_config.h` 里 `SWD_PINS_CONFIRMED = 1`：引脚已按 2026-10-27 新版原理图确认（PA14/PA13）并接线；
  只有置 0 才表示"占位值、等真实板确认"。

---

## 2.3 USART 中断（新增，CubeMX 再生成会丢）

四路串口的收发都是"每字节一个中断"，需要下面两组**USER CODE 区**的代码：

1. `Core/Src/usart.c`，`USER CODE BEGIN USART1_MspInit 1` / `USART3_MspInit 1` 里：

   ```c
   HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);   /* 3 = 低于 USB(0) */
   HAL_NVIC_EnableIRQ(USART1_IRQn);
   ```

2. `Core/Src/stm32f1xx_it.c`，`USER CODE BEGIN 1` 里：

   ```c
   void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
   void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }
   ```

   （U2 工程同样两份，`huart1`/`huart3` 的定义来自各自的 `usart.c`。）

回调分发在各自的 `Core/Src/uart_cfg.c`：`HAL_UART_RxCpltCallback/TxCpltCallback/ErrorCallback`
按句柄分发给 `UartPort_OnRxComplete/OnTxComplete/OnError`。**错误回调里必须先清 ORE 再让驱动重挂接收**，
否则这一路会永久哑掉（原因见 `docs/10_UART四路驱动设计.md` §3.1）。

---

## 2.4 两片重复的三个模块：各放各的工程目录，靠脚本保证副本一致

U1 与 U2 是同一颗 MCU（STM32F103C8T6），协议帧 / 串口驱动 / 链路协议的代码在两边是一样的。
**不搞公共目录**，而是**在两片各自的工程目录里各放一份副本**（工程自包含，Keil 里看到的就是全部）：

| 文件（U1 与 U2 各一份） | 内容 |
|---|---|
| `Core/Src/protocol_frame.c` + `Core/Inc/protocol_frame.h` | A5 5B 帧 + CRC16-MODBUS |
| `Core/Src/uart_port.c` + `Core/Inc/uart_port.h` | 串口端口驱动（环形缓冲 + 中断收发） |
| `Core/Src/uart_link.c` + `Core/Inc/uart_link.h` | U1↔U2 链路协议（请求/应答 + STATUS 编解码） |
| `Core/Src/measure.c` + `Core/Inc/measure.h` | 测量：信号表/换算/EMA/判定/快照/交叉校验 |
| `Core/Src/meas_pack.c` + `Core/Inc/meas_pack.h` | 测量数据的定长线格式（55B 快照 / 21B 校验块 / 77B 应答） |

两个工程的 `uvprojx` 就是普通的 `../Core/Src/xxx.c`，**没有任何跨工程路径**，不需要额外的 Include 路径。

⚠ **代价是"改一边忘一边"**，所以有两条机械保护（别删）：

1. **回归脚本的副本阶段（11/12）**（`test\run_all_tests.cmd`）用 `fc /b` 逐字节比对这 **17 个文件对**，
   只要有一对不一致就报 `[FAIL]` 并让整个回归失败。
   确实需要两份不同时，把这一阶段删掉即可（脚本里有注释说明）。
2. `uart_link.h` 里的 `LINK_CODE_VER`：改变了链路的对外行为（帧格式 / 字段布局 / 时序）就 +1。
   从机在 STATUS 里回这个值，主机比对 → `APP_U2VerOk()` 直接告诉你两片是不是同一版。

> 判据：**只改了某一片的 `Core/` → 只重烧那一片；改了这三个模块中的任何一个 → 两片都重烧。**
> 两片各自的编译产物与烧录方法见 `docs/10_UART四路驱动设计.md` §10。

---

## 2.5 `tim.c` 的定时器被改成了**输入捕获**（再生成会退回去）

| 片 | 定时器/通道 | 引脚 | 用途 |
|---|---|---|---|
| U1 | TIM3 CH1/CH2 | PB4 / PB5 | **输入捕获** ← MB_IN / MA_IN（主控驱动信号 A/B 相） |
| U1 | TIM1 CH1 | PA8 | **输入捕获** ← MC_IN（主控驱动信号 C 相） |
| U1 | TIM4 CH1/CH2/CH3 | PB6 / PB7 / PB8 | **本工程暂不用**（引脚配输入；留给 ⑤ 编码器脉冲发生：届时 U1 配输出、U2 保持输入） |
| U2 | TIM3 CH1/CH2 | PB4 / PB5 | **输入捕获** ← MB_IN / MA_IN（与 U1 同一组信号） |
| U2 | TIM1 CH1 | PA8 | **输入捕获** ← MC_IN |
| U2 | TIM4 CH1/CH2/CH3 | PB6 / PB7 / PB8 | **本工程暂不用**（引脚配输入；⑤ 时用来**采集**编码器线做回环） |

> ⚠ **2026-10 更正（这一版已实施）**：原表写 "U1 TIM4 ← MA0_OUT/MB0_OUT/MC0_OUT（电机输出端，经 3kΩ）"，**是错的**。
> 实际 `PB6/PB7/PB8` 接的是**编码器座 CN2**；`MA0_OUT` 组只有 ADC 通路（140k/20k 分压）。
> 所以 U1 的 TIM1/TIM3 已从"输出"改成**输入捕获**，和 U2 走**同一组信号**做真冗余互比；
> `pulse_cfg.{c,h}` 两片已做成**逐字节副本**（回归副本阶段 11/12 比对）。详见 `docs/00` §3.5、`docs/12` §1/§10。

再生成后必须手工恢复（原因见 `docs/12` §2）：

1. `MX_TIM4_Init()`：`HAL_TIM_PWM_Init` → `HAL_TIM_IC_Init`，三个通道改用
   `HAL_TIM_IC_ConfigChannel`（`ICPolarity=RISING`、`ICSelection=DIRECTTI`、
   `ICPrescaler=DIV1`、`ICFilter=0`）。U2 的 `MX_TIM3_Init`（CH1/CH2）与
   `MX_TIM1_Init`（CH1）同理。
2. `HAL_TIM_MspPostInit()`：上面这几个脚的 `GPIO_MODE_AF_PP` 全部改成
   `GPIO_MODE_INPUT + GPIO_PULLDOWN`；**本工程用不到的定时器引脚**（U1 的 PA8/PB4/PB5，
   U2 的 PA15/PB3/PB1/PB6/PB7/PB8）也要配成输入。
   ⚠ 原来是推挽输出且输出常低：U1 的 PB6/7/8 会经 3kΩ **把被测板的三相输出节点拉低**，
   连相电压的 ADC 读数都会跟着偏低，还等于跟被测板对打。
3. 捕获中断入口写在**手写文件** `pulse_cfg.c` 里（U1 的 `TIM4_IRQHandler`；U2 还要
   `TIM3_IRQHandler` + `TIM1_CC_IRQHandler` + `TIM1_UP_IRQHandler`），NVIC 优先级 4；
   再生成不会动它们，但 `stm32f1xx_it.c` 里**不要**再出现同名处理函数。

> `test/pulse_test.c`（84 项）只测算法，测不到寄存器配置。定时器配错的典型现象是
> "一直判无信号"——先查 `CCER/CCMR` 与引脚模式。

---

## 2.6 编译开关汇总表（统一入口 `jig_mode.h`）

**生产/调试只改一个宏**（`Core/Inc/jig_mode.h` §2，两片都要改，回归会比对）：

| 模式 | 含义 |
|---|---|
| `JIG_MODE_DEBUG`（默认） | 台架调试：判定放宽/关掉，调试打印可用，方便看清现象 |
| `JIG_MODE_PRODUCTION` | 产线生产：判定全开（规格门限 + 交叉校验必需 + 脉冲判据） |

由模式派生、也可单独覆盖的开关：

| 开关 | 默认（DEBUG / PRODUCTION） | 作用 | 打开后的影响 |
|---|---|---|---|
| `MEAS_CC_REQUIRED` | 0 / 1 | U2 没应答是否判测试失败 | 1 = U2 不在 → `05 ERR_TEST` |
| `PULSE_JUDGE_ENABLE` | 0 / 1 | 脉冲是否参与判定 | 1 = 两端对不上/没信号 → `05 ERR_TEST` |
| `MEAS_THRESHOLDS_ENABLED` | 0 / 1 | 电压门限用哪套 | 0 = 全量程（先不判）；1 = 用 `measure.c` 的规格门限 |
| `MEAS_PRINT_ENABLE` / `BSP_DBG_ENABLE` | DEBUG 且 `JIG_HAS_DEBUG_UART=1` | 调试打印（USART1） | 打开会占用 DUT 串口，二选一 |
| `MEAS_USE_VREFINT` | 0 / 0（标定项，与模式无关） | 用 VREFINT 反算 VDDA | 需先标定 `MEAS_VREFINT_MV` |

板级差异（写在**各工程的 Keil Define**里，不在 `jig_mode.h` 里改）：

| Define | 谁用 | 说明 |
|---|---|---|
| `JIG_HAS_DEBUG_UART=1` | 只有 U1 工程 | U1 有 `bsp_uart.c`；U2 没有 → 打印恒关 |

> 已删除的开关：`BSP_ISP_UART_ENABLE`、`ISP_VERIFY_ENABLED`、`ISP_CTRL_PINS_CONFIRMED`
> （ISP 方案整体退役，见 `docs/02` 附录 A）。

---
## 3. 为什么最危险的是 `usb.c`

CubeMX 生成的 `Core/Src/usb.c` 会定义：

```c
PCD_HandleTypeDef hpcd_USB_FS;
void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle) { ... }
```

而移植进来的 `USB_DEVICE/Target/usbd_conf.c` **也定义同名符号**。
两个文件同时在工程里 → **链接期重复定义（multiply defined）**。
所以本工程把 `Core/Src/usb.c` 与 `usb.h` 删掉了，USB 初始化统一由 `USB_DEVICE/` 那套负责。

### 3.1 为什么 gpio.c / hal_msp.c 不会回退

原 `.ioc` 里 SYS Debug 是 **No Debug**（`VP_SYS_VS_ND`），那时：
- CubeMX 把 PA13 当成可自由使用的引脚 → 生成了 EXTI13 输入配置（**会把 SWDIO 变成普通输入，破坏 SWD**）；
- `HAL_MspInit()` 里生成 `__HAL_AFIO_REMAP_SWJ_DISABLE()`（**永久关闭 SWJ，SWD 再也连不上**）。

我把 `.ioc` 改成 **Serial Wire**（PA13=`SYS_JTMS-SWDIO`、PA14=`SYS_JTCK-SWCLK`）。
**实证依据**：同工程的 `U2` 就是 Serial Wire 配置，它生成的 `stm32f1xx_hal_msp.c` 里正是

```c
  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
```

`U1` 的 `HAL_MspInit()` 现在是**条件编译**（见 §2.2）：生产配置走 `__HAL_AFIO_REMAP_SWJ_DISABLE()`（`SWJ_CFG=100`），
开发配置才走上面的 `NOJTAG`。所以再生成后 **CubeMX 会把这段换回纯 `NOJTAG`** ——
好处是 PA13 不会变成 EXTI、SW-DP 也还在，但 **PA13/PA14 又变回调试口 → 再也烧不了 DUT**，
必须按 §4 的第 6 步手工改回条件编译。
顺带说明：这与本设计的 **PB4 = TIM3_CH1** 有关 —— **生产配置**（`SWD_FREE_DEBUG_PINS=1` → `SWJ_CFG=100`）
会把 PA15/PB3/PB4 **一并释放成普通 IO**（PB4 = TIM3_CH1 直接可用）；
**开发配置**（`SWD_FREE_DEBUG_PINS=0` → `NOJTAG`）下 PB4 仍被 NJTRST 占着，还得另外关闭 NJTRST 才能当 TIM3_CH1 用。

---

## 4. 再生成之后必须手工恢复的清单

如果哪天真要用 CubeMX 重新生成，按顺序做：

1. **删掉重新出现的 `Core/Src/usb.c` 和 `Core/Inc/usb.h`**
   （否则链接重复定义）。
2. **把 `main.c` 里的 `MX_USB_PCD_Init();` 改回 `MX_USB_DEVICE_Init();`**
   （并在头部 include `usb_device.h`，删掉 `usb.h` 的 include）。
3. **`MDK-ARM/startup_stm32f103xb.s`**：`Stack_Size EQU 0x400` → `0x800`。
4. **`MDK-ARM/U1.uvprojx`**：把下面这些重新加回工程，并补 Include 路径：
   - 分组 Application/User/Core 追加：
     `app.c` `app_frame.c` `gd25q128.c` `fw_upload.c` `bsp_uart.c` `uart_cfg.c`
     `swd_programmer.c` `swd_phy.c` `swd_dp.c` `swd_memap.c` `cortex_m.c` `stm32f4_flash.c` `measure.c`
     `meas_pack.c` `pulse_cap.c` `pulse_pack.c` `pulse_cfg.c`
     `dut_proto.c` `phase_chk.c` `cur_chk.c` `vol_chk.c` `enc_plan.c` `enc_gen.c` `enc_chk.c` `sn_report.c`
     `protocol_frame.c` `uart_port.c` `uart_link.c`（各片自己的副本，均在 `../Core/Src/`）
     　　完整清单以 `U1/MDK-ARM/U1.uvprojx` 里现有的 37 个 `../Core/Src/*.c` 为准，
     　　上面这份只是"再生成后最容易漏"的那批（`sn_report.c` / `pulse_cfg.c` / 各判据层最容易漏）。
   - 分组 Application/User/USB_Device：`usb_device.c` `usbd_desc.c` `usbd_cdc_if.c` `usbd_conf.c`
   - 分组 Middlewares/USB：`usbd_core.c` `usbd_ctlreq.c` `usbd_ioreq.c` `usbd_cdc.c`
   - Include 路径追加：
     `../USB_DEVICE/App;../USB_DEVICE/Target;`
     `../Middlewares/ST/STM32_USB_Device_Library/Core/Inc;`
     `../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc`
5. **`Core/Src/usart.c` / `stm32f1xx_it.c`**：USART 的 NVIC 使能与中断入口（见 §2.3）。
6. **`Core/Src/stm32f1xx_hal_msp.c` 的 `HAL_MspInit()`**：把 CubeMX 换回的
   `__HAL_AFIO_REMAP_SWJ_NOJTAG();` 改回**条件编译**（见 §2.2）：

   ```c
   #if (SWD_FREE_DEBUG_PINS != 0)
     __HAL_AFIO_REMAP_SWJ_DISABLE();   /* SWJ_CFG=100：PA13/PA14 当普通 IO，能烧 DUT */
   #else
     __HAL_AFIO_REMAP_SWJ_NOJTAG();    /* SWJ_CFG=010：保留 U1 调试口，不能烧 DUT */
   #endif
   ```

   漏了这步的后果：编译照样过，但 **`0x0112` 烧录一定连不上 DUT**（PA13/PA14 还是调试口）。
7. 编译确认 **0 Error / 0 Warning**，并跑一遍 `test\run_all_tests.cmd`。

> 手写文件（`flash_map.h` `protocol_frame.*` `app_frame.*` `gd25q128.*` `fw_upload.*` `meas_pack.*`
> `bsp_uart.*` `app.*` `uart_cfg.*` `uart_port.*` `uart_link.*` `jig_mode.h` `swd_*` `cortex_m.*` `stm32f4_flash.*` `measure.*` `pulse_cap.*` `pulse_pack.*`
> 以及 `USB_DEVICE/`、`Middlewares/` 都不是 CubeMX 管理的，
> **再生成不会动它们**。
>
> ⚠ `U2` 工程的 `U2.uvprojx` 同样需要它自己的 15 个副本（`uart_port.*` `uart_link.*` `protocol_frame.*` `measure.*` `meas_pack.*` `pulse_cap.*` `pulse_pack.*` `jig_mode.h`）+
> `uart_cfg.c` `u2_app.c`。

---

## 5. 建议

- **改外设配置的优先做法**：直接手改 `U1.ioc` 里对应的 `PAxx.Signal=` / `IPParameters` 行，
  再按本文 §4 核对手工项；不要用 CubeMX 走一遍"生成"。
- 真要重新生成，先 **`git commit`** 或整目录备份，再用 `git diff` 逐项恢复 —— 否则很容易漏项，
  而漏掉 `usb.c` 那一条会直接编译不过、漏掉 `startup` 那一条会悄悄少 1KB 栈。
- 建议本工程纳入 git 管理（参考项目 `diameter_JIG` 就有 `.git`）。
