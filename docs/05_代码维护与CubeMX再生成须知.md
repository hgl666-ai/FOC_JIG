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
| `Core/Src/stm32f1xx_hal_msp.c` | `__HAL_AFIO_REMAP_SWJ_DISABLE()` → `__HAL_AFIO_REMAP_SWJ_NOJTAG()` | ❌ | **不会回退** ✅（见 §3.1） |
| `Core/Src/stm32f1xx_it.c` | 加 `USB_LP_CAN1_RX0_IRQHandler` / `USB_HP_CAN1_TX_IRQHandler` | ✅ | 保留 ✅ |
| `Core/Src/main.c` | `GD25Q_Init()` / `APP_Init()` / `APP_Run()` | ✅ | 保留 ✅ |
| `Core/Src/main.c` | `MX_USB_PCD_Init()` → `MX_USB_DEVICE_Init()` | ❌ | **会被换回 `MX_USB_PCD_Init()`** ⚠ |
| `Core/Src/usb.c` + `usb.h` | **已删除** | ❌ | **会被重新生成** ⚠⚠ |
| `MDK-ARM/startup_stm32f103xb.s` | `Stack_Size 0x400 → 0x800` | ❌ | **回退到 0x400** ⚠ |
| `MDK-ARM/U1.uvprojx` | 追加 15 个源文件 + 4 条 Include 路径 | ❌ | **会被整表重写，手动加的文件全部丢失** ⚠⚠ |
| `U1.ioc` | PA13/PA14 改回 `SYS_JTMS-SWDIO`/`SYS_JTCK-SWCLK`，删除 `VP_SYS_VS_ND` | —— | 这是配置源，已改好 ✅ |

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

与 `U1` 现在的内容一致 → 再生成不会把 SWD 关掉，PA13 也不会被配成 EXTI。
顺带说明：`NOJTAG` 同时释放了 PB3/PB4/PA15，而本设计 **PB4 = TIM3_CH1**，正好需要它。

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
     `app.c` `app_frame.c` `protocol_frame.c` `gd25q128.c` `fw_upload.c` `bsp_uart.c` `isp_programmer.c`
   - 分组 Application/User/USB_Device：`usb_device.c` `usbd_desc.c` `usbd_cdc_if.c` `usbd_conf.c`
   - 分组 Middlewares/USB：`usbd_core.c` `usbd_ctlreq.c` `usbd_ioreq.c` `usbd_cdc.c`
   - Include 路径追加：
     `../USB_DEVICE/App;../USB_DEVICE/Target;`
     `../Middlewares/ST/STM32_USB_Device_Library/Core/Inc;`
     `../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc`
5. 编译确认 **0 Error / 0 Warning**，并跑一遍 `test\run_all_tests.cmd`。

> 手写文件（`flash_map.h` `protocol_frame.*` `app_frame.*` `gd25q128.*` `fw_upload.*` `isp_config.h`
> `isp_programmer.*` `bsp_uart.*` `app.*`）以及 `USB_DEVICE/`、`Middlewares/` 都不是 CubeMX 管理的，
> **再生成不会动它们**。

---

## 5. 建议

- **改外设配置的优先做法**：直接手改 `U1.ioc` 里对应的 `PAxx.Signal=` / `IPParameters` 行，
  再按本文 §4 核对手工项；不要用 CubeMX 走一遍"生成"。
- 真要重新生成，先 **`git commit`** 或整目录备份，再用 `git diff` 逐项恢复 —— 否则很容易漏项，
  而漏掉 `usb.c` 那一条会直接编译不过、漏掉 `startup` 那一条会悄悄少 1KB 栈。
- 建议本工程纳入 git 管理（参考项目 `diameter_JIG` 就有 `.git`）。
