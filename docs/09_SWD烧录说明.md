# 09 FOC 固件烧录（SWD 两线）说明

> 状态：**已实现，并通过验证 + 已按权威资料逐条核对**（第 15 轮）
>  - 宿主全链路 170 项（含 9 项 SWD 专项）、SWD 引擎 spike 86 项（含 27 项规范自检）
>  - Keil 0 Error / 0 Warning
> 取代旧的 UART ISP 方案（`isp_programmer.c` 已退出固件构建，源码与测试保留作回退）
>
> 复核资料：ADIv5 规范 `IHI0031H`、F4 参考手册 `rm0090`、
> Cortex-M4 编程手册 `PM0214`、Arm 开源实现 `DAPLink`（含 STM32F103 平台移植）。
> 核对结果与改动见 §1.1——**抓到 2 个致命错误 + 2 处改进**。

## 1. 方案要点

| 项 | 本方案 |
|---|---|
| 物理链路 | **SWCLK + SWDIO 两根线**（+ 共地），**不用 NRST** |
| 目标 | STM32F405RGT6，SW-DP IDCODE `0x2BA01477`，DEV_ID `0x413` |
| 入口 | 协议 `0x0112`（不变），走治具 U1 内部状态机 |
| 固件来源 | 治具板载 GD25Q128 的两个槽（Slot0=Bootloader / Slot1=App），协议 `0x0009/0x000A/0x000B` 上传 |
| 引脚 | **占位：SWCLK=PB10 / SWDIO=PB11**（`swd_config.h`），等真实板子定下来再改 |
| 时钟 | `SWD_DELAY_LOOPS=8` ≈ 1 MHz（上板确认稳定后可以降到 4/2/0 提频） |

**为什么不用 NRST**：目标只要没把 PA13/PA14 当普通 IO 用、也没进 STOP/Standby，
直接连接就能通；烧完用 `AIRCR.SYSRESETREQ` 复位运行，同样不需要复位线。

**代价（必须知道）**：
- 没有 connect-under-reset：目标程序如果占用了 SWD 引脚或进了低功耗，**连不上**。
  固件会明确报 `ERR_PROG` 并打印底层错误码，不会死等。
- 建议：真实板子上**留一个 NRST 测试点**，将来若遇到连不上的板子，把
  `swd_config.h` 里的 `SWD_USE_NRST` 置 1、填上引脚即可（`swd_phy.c` 里已经写好时序分支）。

## 1.1 已按权威资料逐条核对（第 15 轮）

参考资料：`INFO/IHI0031H...debug_interface_v5_2_architecture_specification.pdf`（ADIv5 规范）、
`INFO/rm0090.pdf`（F4 参考手册）、`INFO/PM0214.pdf`（Cortex-M4 编程手册）、
`INFO/SWD烧录参考的开源项目/DAPLink-main/`（Arm 的 Apache-2.0 开源实现，含 **STM32F103 平台移植**）。

| 项 | 结论 | 依据 |
|---|---|---|
| 请求头 8 位（Start/APnDP/RnW/A2/A3/Parity/Stop/Park，LSB first） | ✅ 一致 | ADIv5 §B4.2；DAPLink `SW_D...transfer` |
| 奇偶校验 = APnDP^RnW^A2^A3 | ✅ 一致 | 同上 |
| 请求头→ACK 的 turnaround = 1 | ✅ 一致 | §B4.1.1；DAPLink `swd_conf.turnaround=1` |
| **读**：ACK→RDATA **无** turnaround；RDATA 之后要 turnaround | ✅ 一致 | §B4.1.1 原文；DAPLink 同 |
| **写**：ACK→WDATA 要 turnaround；WDATA 之后**无**额外 turnaround | ⚠ 改（原来多打了 1 拍） | §B4.1.1 原文 |
| 空闲周期 SWDIO 驱动为低；只有"要停时钟"时才需 ≥8 拍 | ✅ 一致 | §B4.1.1；DAPLink 驱动低 |
| 线复位 ≥50 拍高电平；JTAG→SWD 序列 `0xE79E` LSB first | ✅ 一致 | §B5.2.2（原文两种写法：0x79E7 MSB first = 0xE79E LSB first） |
| CSW = `0x23000052`（Size=word、AddrInc=0b01、DeviceEn、HPROT/MSTRDBG/RESERVED/DBGSTAT） | ✅ 与 DAPLink 的 `CSW_VALUE` 完全相同 | ADIv5 §C2.2/§B2.3；DAPLink `swd_host.c:39` |
| **CSW.AddrInc 语义**：0b00=自增关闭，0b01=每次访问后自增 | ❌ **改（原来记反了，块读写会一直打同一地址）** | ADIv5 表 C2-1 原文 |
| 清 sticky 用 ABORT（0x1E）而不是写 CTRL/STAT | ❌ **改（原来写 CTRL/STAT 会把 CDBGPWRUPREQ 撤掉）** | §B4.2.1：SW-DP 的 CTRL/STAT sticky 位是 RO/WI |
| 协议错误时补一个数据阶段的时钟数再收尾（DAPLink 的 back-off） | ➕ 新增（增强健壮性） | DAPLink `SWD_Transfer` 的 "Protocol error" 分支 |
| FLASH_SR 位（EOP/OPERR/WRPERR/PGAERR/PGPERR/PGSERR/BSY） | ✅ 一致 | RM0090 §3.9.6 |
| **FLASH_CR.STRT = bit16** | ❌ **改（原来写成 bit15，擦除根本不会启动！）** | RM0090 §3.9.7 原文 |
| FLASH_CR 其余位（PG/SER/MER/SNB=bits6:3/PSIZE=bits9:8/LOCK=bit31） | ✅ 一致 | 同上 |
| 解锁钥匙 `0x45670123` / `0xCDEF89AB` | ✅ 一致 | RM0090 §3.9.2 |
| 扇区 16/16/16/16/64/128/128/128 KB | ✅ 一致 | RM0090 §3.3.1 |
| DHCSR：DBGKEY=0xA05F<<16、C_DEBUGEN=bit0、C_HALT=bit1 | ✅ 一致 | PM0214 + DAPLink 用法 |
| AIRCR：VECTKEY=0x5FA<<16、SYSRESETREQ=bit2 | ✅ 一致 | PM0214 §C1.2 |

**新增 27 项"规范自检"回归**（`proto/swd_sim/tests/spike_main.c` 第 11 节）：
把上面这些常量按手册里的**字面值**钉死，另外用两条用例钉住 CSW.AddrInc 的语义
（0b01 时连续 DRW 读必须递增地址、0b00 时必须停在同一个地址）。以后谁改错一位都会当场红。

> ⚠ **这次最值得记住的教训**：仿真模型是我自己写的，所以它会**和固件共享同一个错误概念**。
> 上面两个致命错误（CSW 语义、STRT 位号）之所以"测试全绿"，正是因为模型里也写的是同一套错值。
> **凡是有权威资料可查的常量，必须对着资料逐条核，不能靠自证式的仿真。**

> 复核用的资料就在 `INFO/` 下，需要时这样转成文本再查：
> ```
> pdftotext -layout "INFO\IHI0031H_debug_interface_v5_2_architecture_specification.pdf" ihi0031.txt
> pdftotext -layout "INFO\rm0090.pdf" rm0090.txt
> pdftotext -layout "INFO\PM0214.pdf" pm0214.txt
> ```
> （转换产物体积很大，没有入库；`rm0008.pdf` 是治具 F103 的手册，本次没用到。）
>
> 另外：DAPLink 里有 **STM32F103 平台的现成移植**
> （`source/hic_hal/stm32/stm32f103xb/DAP_config.h`），
> 将来若要给 SWD 提速（DMA/定时器打时序），那是一份直接可对照的参考。

## 2. 引脚怎么填（重要）

`U1/Core/Inc/swd_config.h` 是**唯一**要改的文件：

```c
#define SWD_CLK_PORT   GPIOB          /* ← 改成真实引脚 */
#define SWD_CLK_PIN    GPIO_PIN_10
#define SWD_CLK_PINNUM 10u
#define SWD_DIO_PORT   GPIOB
#define SWD_DIO_PIN    GPIO_PIN_11
#define SWD_DIO_PINNUM 11u
#ifndef SWD_PINS_CONFIRMED
#define SWD_PINS_CONFIRMED 0          /* ← 接线确认后改 1 */
#endif
```

占位值的选择依据（现有原理图上"已经引出来"的脚）：
U1 空闲且已引出的脚只有 **PB10/PB11**（USART3 → 2 脚排针）与 **PA9/PA10**（USART1 → 4 脚排针）；
**PA13/PA14 是 U1 自己的 SWD，绝不能占用**（一旦当普通 IO 用，U1 自己就再也接不上调试器）。
选 PB10/PB11 的理由：同属 GPIOB、位操作与方向切换都在 CRH，代码最省事，
而且这两脚原本就是准备接被烧板的。

> ⚠ 副作用：`MX_USART3_UART_Init()` 仍然把 PB10/PB11 配成串口复用；`SWDP_Init()`
> 在 `APP_Init()` 里随后把它们抢回来当 GPIO（引脚模式一改，串口输出就与引脚断开）。
> 真实引脚确定后，如果不再需要 USART3，建议把它从 CubeMX 里去掉，避免混淆。

## 3. 烧录流程（非阻塞状态机）

```
SWDP_Start()
  └─ CONNECT   线复位(56 高) → JTAG→SWD 切换序列 0xE79E → 读 IDCODE
               → CTRL/STAT 上电请求(确认 CDBG/CSYS ACK) → SELECT AP0 + CSW
  └─ HALT      停内核(DHCSR 钥匙 0xA05F) → 冻结 IWDG/WWDG → 读 DBGMCU_IDCODE
               → **读两份镜像头并校验（在擦除之前！）**
  └─ ERASE     按扇区擦除，一次一个扇区（128KB 扇区要约 1.3s，不能一次擦完）
  └─ PROGRAM   一次 128 字节：从 SPI Flash 读 → 通过 AHB-AP 写目标 Flash
  └─ VERIFY    一次 128 字节回读比对（SWD 的读回是目标 AHB 上的真实内容，可信）
  └─ GO        AIRCR SYSRESETREQ 复位运行 → OK
```

任何一步失败 → `FAIL`，`APP_Run()` 回一条 `0x0112` ACK（状态码见下），**只回一次**。

错误码映射：

| 情况 | 状态码 |
|---|---|
| 成功 | `0x00` OK |
| 连接失败（无 IDCODE / 停不住内核 / 超时） | `0x04` ERR_PROG |
| 目标 Flash 擦写/校验失败 | `0x04` ERR_PROG |
| 镜像不完整（完整标志缺失 / 超分区上限被前置检查拦下） | `0x07` ERR_INCOMPLETE / `0x02` ERR_SIZE |
| 治具自己的 SPI Flash 读失败 | `0x03` ERR_FLASH |

失败时可用 `SWDP_LastIdcode() / SWDP_LastDevId() / SWDP_LastOffset() / SWDP_LastSwdError()`
定位卡在哪一步；调试串口(PA9)也会打印 `[SWD] idcode=...` / `[SWD] FAIL step=...`。

## 4. 实测节拍（从仿真里数出来的 SWCLK 周期）

| 项 | 实测 |
|---|---|
| 一个 SWD 写事务 | 46 个 SWCLK（改掉多余的尾部 turnaround 之后） |
| 每个字（写 + 轮询 BSY） | 约 313 个 SWCLK（含校验摊薄） |
| 折合 | **86 647 周期 / KB** |
| 300 KB 应用 | **26.0 s @1MHz / 13.0 s @2MHz / 6.5 s @4MHz** |
| 全片 1 MB | 87 s @1MHz / 43 s @2MHz / 22 s @4MHz |

（只算 SWD 链路；真实擦除还要等 Flash：16KB 扇区 0.3s、64KB 0.7s、128KB 1.3s（典型值）。

**提速路径**（按性价比排序）：
1. **降 `SWD_DELAY_LOOPS`**：8→4 就是 2 倍速，最省事（上板确认稳定后再降）；
2. **把轮询 BSY 从 3 个事务降到 1 个**（预置 TAR + 流水线读）→ 每字省 2 个事务，约再快 40%；
3. **在目标 SRAM 里跑一小段擦写例程**（ST-Link 的做法）：每字只剩 1 个事务，
   链路时间再降一半，但要维护一份跑在目标上的代码镜像，复杂度明显上升，**建议先不做**。

## 5. 上板检查清单（按顺序做，成本从低到高）

1. **改引脚 → 编译 → 用 SWD 或串口把 U1 固件烧进去**（U1 自己的 SWD 在 4 脚排针上）。
2. **接两根线 + 共地**：治具 SWCLK/SWDIO → 被烧板 PA14/PA13，GND 必须共。
   建议串 22~100Ω（顶针/飞线较长时）。
3. **先用 `0x000D` 做最小验证**：它只需要"连上 + 读 4 个字节"，
   不用上传任何固件，是最便宜的冒烟测试。
   - 回 `0x00` + 版本 = 链路和读写全通 ✅
   - 回 `0x06`（无固件，读到 0xFF）= **连上了**，只是 App 区是空的 ✅ 同样说明链路通
   - 回 `0x0C`（超时）= 连不上 ❌ 进入第 4 步
4. **连不上时用逻辑分析仪看 SWCLK/SWDIO**：
   - 应该看到 56 个高电平周期的线复位、16 位 `0xE79E` 切换序列、然后 8 位请求头；
   - 看不到 → 引脚配错/没接线；
   - 看得到但 ACK 一直是 `0b111`（无应答）→ 目标没供电、没共地、或 SWD 脚被目标程序占用；
   - ACK 是 `0b010`(WAIT) 或 `0b100`(FAULT) → 目标还活着但状态不对，多试几次连接。

   线级约定已按 ADIv5 规范核对过（见 §1.1），**上板不再需要靠试错去猜 turnaround 拍数**；
   逻辑分析仪这时只用来确认"线接对了、电平对不对"。
5. **调速率**：确认能稳定烧之后，把 `SWD_DELAY_LOOPS` 从 8 依次降到 4、2、0，
   每一步都重跑一次 `0x000D` + 一次整片烧录，取稳定值（预期 2~4 MHz）。
6. **烧录中间不要插拔线**：烧到一半断线会得到"擦了一半"的板子，
   重新烧一次即可（流程每次都从擦除开始）。

## 6. 与旧 UART ISP 方案的差异

| | UART ISP（旧，已退役） | SWD（现在） |
|---|---|---|
| 线数 | TX/RX + BOOT0 + RESET（4 根 + 地） | **SWCLK/SWDIO（2 根 + 地）** |
| 依赖 | 目标系统 bootloader + BOOT0 跳线 + 串口引脚 | 只依赖 SWD 引脚 |
| 目标状态要求 | 必须能进 bootloader（BOOT0 拉高后复位） | 只要 SWD 没被应用占用 |
| 回读校验 | AN3155 的 Read Memory，帧格式各种坑，默认关闭 | 目标 AHB 真实内容，**默认开启且可信** |
| 速度 | F4 bootloader 官方只保证 115200 → 1MB 约 90 s | 2 MHz 下 1MB 约 44 s（可提到 4 MHz → 22 s） |
| `0x000D` 查版本 | 要目标配合（UART 协议未知，一直没做） | **直接读目标 Flash**（偏移待 FOC 工程确认） |
| ROM 占用 | 约 5.5 KB | 约 4.2 KB（且更少的外设依赖） |

> `isp_programmer.c` / `isp_config.h` / `bsp_uart.c` 里的 ISP 串口收发**保留在源码树里**
> （`test/isp_test.c` 的 189 项断言仍然全过），但**不再进 Keil 工程**：
> 既不占 ROM，又保留了"万一要回退"的可能。

## 7. 明确没做（需要时再补）

1. **RDP 读保护**：量产板若开了 RDP1，AP 访问会被拒绝，需要先整片擦除。
   本版没有处理，会报 `ERR_PROG`（诊断上能通过 `SWDP_LastSwdError` 看出来）。
2. **选项字节 / 写保护**：没有读写 OPTCR，也没有做 mass erase（`FLASH_CR.MER`）。
3. **多 drop / 多 AP**：只有单 AP0、单 drop（F405 单核场景够用）。
4. **目标内 flashloader**：见 §4 提速路径第 3 条。
5. **`DUT_VER_OFFSET` 是占位值**（`flash_map.h`，当前 0x20）：0x000D 读的版本号在
   App 分区里的真实偏移要 FOC 工程确认。
6. **分区地址是占位值**：Boot `0x08000000`+32KB（扇区 0+1）、App `0x08008000`+992KB（扇区 2~7）。
   F4 的扇区是 16/16/16/16/64/128/128/128 KB，**分区边界必须落在扇区边界上**，
   等 FOC 工程的链接脚本确定后替换（`flash_map.h` 里有 5 条编译期断言钉住这几条约束）。
