# 治具四路串口（UART）驱动与 U1↔U2 内部链路设计

> 任务 ①「4 路 UART 驱动开发 & 数据收发」的设计与验证记录。
> 代码：`U1|U2/Core/{Inc,Src}/uart_port.*`（驱动）、`.../uart_link.*`（链路协议），两片各一份副本、
> `U1/Core/Src/uart_cfg.c`、`U2/Core/Src/uart_cfg.c`（各板装配）、`U2/Core/Src/u2_app.c`（从机应用）。

---

## 1. 范围与结论

治具一共 **4 路串口**，两片各 2 路：

| 片 | 端口号 | 外设 | 引脚 | 接到哪里 | 用途 |
|---|---|---|---|---|---|
| U1（主控） | 端口0 | USART1 | PA9(TX) / PA10(RX) | 排针 → 被测 FOC 板 | **DUT 串口**：读 FOC 板上报的数据，供治具做测试判定（判定流程与 PC 取数命令待定，见 §10） |
| U1（主控） | 端口1 | USART3 | PB10(TX) / PB11(RX) | H3/H4 跳线 → U2 | **内部链路**（U1↔U2） |
| U2（测量片） | 端口0 | USART1 | PA9(TX) / PA10(RX) | 排针 | 对外预留（调试/扩展） |
| U2（测量片） | 端口1 | USART3 | PB10(TX) / PB11(RX) | H3/H4 跳线 → U1 | **内部链路**（U2↔U1） |

- 四路统一 **115200 8N1，无硬件流控**（CubeMX 生成值；两片必须一致，否则链路全错）。
- 驱动与链路代码**两片各一份副本**（各自工程目录里，靠回归脚本第 7 阶段保证逐字节一致），U1 是链路主机、U2 是从机。
- 全程非阻塞：中断只搬字节，解析与组帧在主循环。

⚠ **上电前必须确认跳线是交叉的**：U1.PB10 → U2.PB11，U1.PB11 → U2.PB10。
接成直连（TX 对 TX）时两片都收不到任何东西，现象与"U2 没上电"完全一样。

---

## 2. 软件分层

```
        PC ──USB CDC──►  U1 app.c（PC 协议 A5 5B 帧）
                              │
                              │ ③ 测量交叉校验 / ① 保活 PING
                              ▼
                     uart_link.c   ← 链路帧（同一套 A5 5B 格式 + CRC16）
                              │
                     uart_port.c   ← 环形缓冲 / 中断收发 / 统计
                              │
        uart_cfg.c（各板）  ← HAL 句柄 ↔ 端口号、HAL 回调分发、NVIC
                              │
                    HAL_UART_Receive_IT / Transmit_IT
```

| 文件 | 位置 | 职责 |
|---|---|---|
| `uart_port.{h,c}` | 两片各一份 | 一路一个实例：256B 收/发环形缓冲、每字节中断收、TXE 链式中断发、丢字节计数。**不认识"这一路接什么"** |
| `uart_link.{h,c}` | 两片各一份 | 链路帧：组帧/解帧（复用 `protocol_frame.c`）、主机请求（阻塞版 + 非阻塞 Start/Poll 版）、从机应答、STATUS 编解码。**对象化**（`link_t`），不是单例 |
| `protocol_frame.{h,c}` | 两片各一份 | A5 5B 帧 + CRC16-MODBUS（PC 协议与内部链路共用同一套实现） |
| `uart_cfg.{h,c}` | 各板 `Core/` | 只做板级装配：`huart1`→端口0、`huart3`→端口1、启动接收、把 HAL 回调分发给对应端口 |
| `u2_app.{h,c}` | U2 `Core/` | U2 应用层：链路从机命令处理（PING/STATUS），③ 的测量功能将加在这里 |

---

## 3. 并发与中断约定（收发环**没有临界区**，这是有意的；只有"发起发送"那一处有）

| 状态 | 谁写 | 谁读 | 为什么不需要临界区 |
|---|---|---|---|
| `rx_head` | 中断 | 主循环 | 单生产者单消费者；两边各只改自己的索引 |
| `rx_tail` | 主循环 | 中断 | 同上 |
| `tx_head` | 主循环 | 中断 | 同上 |
| `tx_tail` | 中断 | 主循环 | 同上（中断推进 tail 时只用下面的 `tx_len`，**不再读 `tx_head` 倒推**） |
| `tx_len` / `tx_busy` | 只在 `uart_tx_kick()` 的**临界区**里置位/清零 | 中断侧只读 `tx_len`、只清 `tx_busy` | ⚠ **这一处需要临界区**（docs/03 §4 的 D6，2026-10 修）：<br>① 段长必须**快照**——`kick` 把这一段交给 HAL 的长度存进 `tx_len`，完成中断按 `tx_len` 推进 `tx_tail`。用"当前 `tx_head − tx_tail`"倒推会把**还没发出去**的字节算成已发 → 静默丢字节。<br>② 「判断 `tx_busy` → 置位 → 调 `HAL_UART_Transmit_IT` → 失败回退」必须**整体原子**，而且是**先进临界区再判断**（先判断再关中断不够：两步之间完成中断能插进来清 busy 并由它自己 kick 一段出去，我们这次调用就会撞 HAL_BUSY，回退又把状态清掉 → 与中断手里那一段错位）。<br>③ 不变式：`tx_busy≠0 ⟺ tx_len≠0 ⟺ HAL 手里正有一段在发`；HAL 拒发时两个字段一起清、字节留在环里重试，并计 `tx_err`（正常恒为 0）。<br>临界区只有几十个周期（~1 µs），115200 下一个字节 87 µs，挡不住收。 |

其他约定：

- **中断里只搬字节**：`HAL_UART_IRQHandler` → `HAL_UART_RxCpltCallback` → `UartPort_OnRxComplete`，
  只做"入环 + 重新挂接收 + 计数"，不解析、不打印、不加延时。
- **中断优先级**：USB = 0（`usbd_conf.c` 里设的），两路串口 = **3**。串口中断绝不能顶掉 USB CDC 的收发时序。
- **`Link_Poll` 不可重入**，只在主循环上下文调用（宿主测试里用 guard 挡重入来模拟"对侧是另一颗 MCU"）。
- 主循环里若要用链路，用 `Link_RequestStart` + `Link_RequestPoll` 两段式，**零阻塞**；
  `Link_Request`（阻塞版）只用于"可以等几十毫秒"的场合（例如未来上电自检），它内部不再喂 USB 侧。

### 3.1 一个必须记住的 HAL 坑：ORE 之后要重新挂接收

STM32F1 HAL 收到 **ORE（溢出）** 时走的是"阻塞错误"分支：先 `UART_EndRxTransfer()`
（关掉 RXNEIE/PEIE/EIE，`RxState = READY`），**然后**才回调 `HAL_UART_ErrorCallback()`。
如果错误回调里不重新调 `HAL_UART_Receive_IT()`，这一路从此**再也收不到任何字节，而且不报任何错**——
"串口突然哑了"最典型的成因。

本工程的做法：
- `UartPort_OnError()` 里重新挂接收（驱动不变量，写在 `uart_port.c`）；
- `HAL_UART_ErrorCallback()` 里先 `__HAL_UART_CLEAR_OREFLAG()` 再分发（寄存器细节留在板级 `uart_cfg.c`），
  否则重挂后会立刻再次进错误中断，形成中断风暴。
- 宿主测试 §5 专门盯这条：模拟"HAL 把接收停了"再调 `UartPort_OnError`，断言接收被重新挂上且之后真的收得到字节。

---

## 4. 缓冲与流控策略

- 每路收发各 **256B 环形缓冲**（容量 255，留一格判满）。
- **缓冲满时丢"新"字节并计数**，不丢最旧：半截帧比丢整帧更难查，
  而丢字节计数暴露在 `UartPort_Stats()` 里，排障时一眼看出是谁丢的。
- `UartPort_Write()` 返回**实际入队字节数**：调用方负责补写剩余部分（不阻塞等待）。
  链路侧只用 ≤74B 的短帧，正常永远一次写进；只有"对端不收/线卡住"才会看到截断。
- 统计（`uart_stats_t`）：`rx_bytes / tx_bytes / rx_drop / tx_drop / rx_err / rx_wrap / tx_err`，
  以及链路级 `link_stats_t`（`tx_frames / rx_frames / tx_timeout / tx_busy / rx_crc_err /
  rx_len_err / rx_resync / rx_stale / slave_reply / slave_drop`）。

---

## 5. 链路协议（U1↔U2）

> ⚠ **完整规范已独立成文：`docs/13_U1U2内部链路协议.md`**（帧格式、命令表、ARM 同步触发、
> 事件上报、重传与幂等、断链恢复、版本与能力协商）。本节只保留设计背景，**命令与字段以下面那份为准**：
> 本节提到的 PING/STATUS/MEAS_XCHK/PULSE_XCHK 之外，v1.3 还多了 `0x0105` ARM、`0x0106` LINK_RESET、
> `0x0107` EVENT、`0x0108` EVENT_GET，STATUS 也从 22B 扩到 28B。

- 帧格式与 PC 协议**完全一致**：`A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA | CRC16(2)`，全大端，CRC16-MODBUS。
- 命令码用 `LINK_CMD_*`（`0x0100` 起），与 PC 协议的 `MGR_CMD_*` 不冲突（U1 独占解析 PC 帧）。

| 命令 | 方向 | 应答 |
|---|---|---|
| `0x0100` PING | U1→U2 | 原样回显 payload（长度 0 也可，用于保活） |
| `0x0101` STATUS | U1→U2 | 22B 状态：版本/链路代码版本/存活计数/链路与端口统计（见下） |
| `0x0102` (预留) | — | 未使用；脉冲捕获交换可能用它 |
| `0x0103` MEAS_XCHK | U1→U2 | **请求 = U1 的 55B 快照，应答 = U2 的 55B 快照**（比对由 U1 做，见 `docs/07` §3.4） |
| `0x0104` PULSE_XCHK | U1→U2 | **请求 = U1 的 45B 脉冲结果，应答 = U2 的 45B**，见 `docs/12` §6。⚠ **2026-10 更正**：两片采的是**同一组信号**（MA_IN/MB_IN/MC_IN），用于冗余互比；原写"U1 三相输出端 / U2 三相驱动输入端"是错的（`MA0_OUT` 组只有 ADC 通路、没有定时器通路） |

- **SEQ 回显**：从机应答沿用请求的 SEQ（与 PC 侧 ACK 规则一致）。主机只认"当前在等的那一发"的 SEQ，
  不匹配的合法帧计入 `rx_stale` 丢弃——这样迟到的旧应答不会污染下一次请求。
- **`0xFF` NIY 而不是静默丢弃**：U1 才能区分"U2 不在/线没接"（超时）与"U2 在，但这条命令没实现"。
- `LINK_CMD_STATUS` 数据区（22B，大端；`Link_PackStatus`/`Link_UnpackStatus` 两片共用一份编解码）：
  `[0]固件主版本 [1]固件次版本 [2..3]链路代码版本 [4..5]存活计数 [6..9]rx_frames
  [10..13]rx_crc_err [14..17]rx_bytes [18..21]rx_drop`
- 抗干扰行为：CRC 错只计数不采纳；LEN > 64 直接判非法并丢弃；丢同步后重新找 `A5 5B`（`rx_resync` 计丢弃字节数）。

---

## 6. U1 侧：保活探测（非阻塞）

`app.c` 的 `APP_LinkTask()` 每轮主循环推进一步：

- 空闲且距上次 ≥500ms → 发一发探测帧（**PING**；每 5 次里插一次 **STATUS**，约 2.5s 一次）；
- 已发出 → `Link_RequestPoll(50ms)`：
  - 成功 → `s_u2_alive = 1`，探测 ok 计数 +1；若这一发是 STATUS，则解包 → `APP_U2Status()`、
    `APP_U2VerOk()`（两片链路代码版本是否一致）；
  - 超时 → `s_u2_alive = 0`，fail 计数 +1；
- **烧录期间（`app_state != IDLE`）完全不动链路**，保证 USB 侧及时响应；
- 诊断接口：`APP_U2Alive()`、`APP_U2ProbeStats(&ok, &fail)`、`APP_U2VerOk()`、`APP_U2Status(&st)`。

这条保活线的意义：它让"两片之间的串口是通的""两片烧的是同一版代码"成为**持续可观测**的事实，
而不是等到 ③ 做测量时才发现线接反了或者只重烧了一片。115200 下往返 < 2ms。

---

## 7. U2 侧：最小固件

- `main.c`：`MX_GPIO_Init()/MX_ADC1_Init()/... MX_USART1_UART_Init()/MX_USART3_UART_Init()` →
  `UartCfg_Init()` → `U2App_Init()`；主循环只调 `U2App_Run()`（= `Link_Poll`）。
- `u2_app.c`：从机命令处理。**刻意很薄**——帧层/收发都在本片自己的 `uart_link.c`/`uart_port.c`，这里只写"收到命令做什么"。
- ③（测量）搬过来时，只在这里加 case + 一个测量模块，不需要动帧层。

---

## 8. 验证手段

### 8.1 宿主单测（`test/uart_test.c`，745 项断言，`test\run_all_tests.cmd` 第 5 阶段）

编译的是**固件真源码**（U1 的 `uart_port.c` `uart_link.c` `protocol_frame.c` + `U1/Core/Src/uart_cfg.c`）+ 一个"假 HAL"：

- 假 HAL 模拟真 HAL 的 IT 收发语义：收 1 字节 → 回调 → 驱动重挂；发一段 → 逐字节推 → 推完 TxCplt → 驱动领下一段；
- 两根线交叉相接（`USART1` 侧与 `USART3` 侧互为对端），并可选"对侧主循环"（回调里跑对侧 `Link_Poll`）；
- `HAL_GetTick()` 自增，保证所有超时循环一定终止。

覆盖：环形缓冲回卷、300B>环容量的链式续传、接收环满丢新字节计数、发送队列满计数与解除后继续、
ORE 后重挂接收、链路 PING 往返（含 0 长度与连打 5 发防旧应答）、超时后状态可恢复、
CRC 错/垃圾前缀/超长 LEN 的抗干扰与恢复、忙保护与非阻塞语义、STATUS 编解码、
**两片链路代码版本的一致性检查（含"对端是旧版本"的检出）**、端到端 STATUS 与 NIY 应答。

### 8.2 编译期

- U1：**0 Error / 0 Warning**，`Total RO = 30444B (29.73KB)`、`Total RW = 11048B (10.79KB)`
  → ROM 30.6KB/64KB、RAM 10.8KB/20KB（UART 部分约 +2.5KB ROM / +1.5KB RAM）。
- U2：**0 Error / 0 Warning**，`Total RO = 10320B (10.08KB)`、`Total RW = 3584B (3.50KB)`。

> 注：Keil 的 UV4 会以子进程方式跑 armcc/armlink。如果构建环境对子进程写文件有限制，
> UV4 仍会打印 "0 Error(s), 0 Warning(s)" **却不产出 `.o/.axf/.map`**。
> 判断构建是否真的发生：看 `MDK-ARM/<target>/<name>.map` 的时间戳是否刷新。

---

## 9. 上板检查清单

1. H3/H4 交叉跳线（TX↔RX）**且共地**。
2. 两片波特率一致（当前都是 115200 8N1）。若两片用不同晶振/时钟源，先用示波器看一位宽度。
3. 看"链路是否通"：U1 侧 `APP_U2Alive()` 应为 1，`APP_U2ProbeStats()` 的 fail 不持续增长。
4. **看"两片烧的是不是同一版代码"**：`APP_U2VerOk()` 应为 1（详见 §10）。
5. 一端出现**持续 ORE**（`rx_err` 增长、`rx_drop` 增长）：通常是波特率不匹配或对端帧间隔过密，
   也可能是中断被长时间屏蔽（例如 SWD 位翻转循环里没有开中断——本项目没有关中断，所以不是这个原因）。
6. 串口线上预留的串阻（22~100Ω）与共地仍待硬件确认。

---

## 10. 两片各自的工程与副本：怎么编译、怎么烧、怎么防"改一边忘一边"

### 10.1 没有公共目录：三个模块在两片里各有一份副本

```
   U1 工程 (U1/MDK-ARM/U1.uvprojx)                 U2 工程 (U2/MDK-ARM/U2.uvprojx)
   ├─ U1/Core/Src/*.c                              ├─ U2/Core/Src/*.c
   │   ├─ uart_port.c   ┐                          │   ├─ uart_port.c   ┐
   │   ├─ uart_link.c   ├─ 副本 A                  │   ├─ uart_link.c   ├─ 副本 B
   │   └─ protocol_frame.c ┘                       │   └─ protocol_frame.c ┘
   └─► 编译 → 链接 → U1/MDK-ARM/U1/U1.hex          └─► 编译 → 链接 → U2/MDK-ARM/U2/U2.hex
                            │                                          │
                       烧进 U1 芯片                                烧进 U2 芯片
```

- 两个工程**互相独立、自包含**，`uvprojx` 里全是 `../Core/Src/...` 的相对路径，
  没有任何跨工程路径；在 Keil 里打开哪个工程就只看到哪一片的代码。
- 两颗芯片里各有一份 `uart_port`/`uart_link`/`protocol_frame` 的机器码，
  和两边各有一份 HAL 是一回事。
- 产物与烧录方式：

  | 烧到哪颗芯片 | 用哪个文件 | 用哪个工程 | 自己的调试口 |
  |---|---|---|---|
  | U1（主控） | `U1/MDK-ARM/U1/U1.hex`（或 `.axf`） | `U1/MDK-ARM/U1.uvprojx` | PA13/PA14（`hal_msp.c` 保留了 SW-DP） |
  | U2（测量片） | `U2/MDK-ARM/U2/U2.hex`（或 `.axf`） | `U2/MDK-ARM/U2.uvprojx` | PA13/PA14 |

  烧录方式与普通 STM32 相同（SWD/JTAG：ST-Link、J-Link、DAPLink）。
- 三个副本文件**不得依赖任何板级差异**（引脚、外设实例、`main.h` 里的宏）——
  它们只用到 `stdint.h` / `string.h` / HAL 的 `HAL_UART_*_IT` 基本接口，满足这条。

### 10.2 副本漂移的两道机械保护

副本方案的代价是"改一边忘一边"，而它的后果是**静默的**：
字段布局变了而对面还是旧的 → STATUS 解出垃圾；帧格式变了 → 一直 CRC 错/超时。
所以加了两道保护，**不要删**：

1. **回归脚本第 7 阶段**：`test\run_all_tests.cmd` 用 `fc /b` 逐字节比对 6 个文件对
   （`uart_port.{c,h}`、`uart_link.{c,h}`、`protocol_frame.{c,h}`），
   只要有一对不一致就打印 `xxx.c : DIFFERENT` 并让整个回归失败
   （已用"故意改一个字节"反证过：该阶段确实会失败、退出码 1）。
   确实需要两份不同时，把这一阶段删掉即可（脚本里有注释）。
2. **`LINK_CODE_VER`（`uart_link.h`）**：改变了链路的对外行为（帧格式 / 字段布局 / 时序）
   就把它 +1。从机在 `LINK_CMD_STATUS` 里回这个值（`[2..3]` 字节），
   主机用 `Link_CheckVersion()` 比对 → `APP_U2VerOk()`：**1 = 两片同一版，0 = 有一片没跟着重烧**。
   主机每 5 次保活探测（≈2.5 s）插一次 STATUS，所以"烧漏一片"会在 3 秒内暴露。

> 判据：只改了某一片的 `Core/`（如 `u2_app.c`）→ 只重烧那一片；
> 改了这三个模块中的任何一个（两份都要改）→ **两片都重烧**；只改文档/测试 → 都不用烧。

### 10.3 一键回归会同时构建两片

`test\run_all_tests.cmd` 现在是 8 个阶段，第 8 阶段**依次构建 U1 与 U2** 两个工程并分别判定
（Error 优先于 Warning，且按日志轮询而不是等 UV4 进程退出），
所以"宿主测试全过 + 副本一致 + 两片都编译得过"是一条命令的事。

---

## 11. 未决 / 下一步

- **DUT 串口（U1 端口0）的取数与判定流程**：已明确用途是"读 FOC 板上报的数据，供治具做后续判定"。
  还缺两件事，需要协议所有者拍板：
  1. FOC 板上报的数据格式（帧头/字段/周期，或者就是 FOC 自己的调试串口输出）；
  2. PC 怎么拿到这些数据：xlsx 里现有 6 条命令没有这一条，要么新增一条"读取 DUT 串口缓存"命令，
     要么判定在治具内部完成、PC 只拿 `ERR_TEST(0x05)` 一类的结论。
  本阶段驱动侧已就绪（`UartPort_Stats(UART_CFG_PORT_DUT)` 能看收了多少/丢了多少），
  拿到上面两条后只在 `app.c` 加解析与上报即可。
- ③ 测量迁移到 U2 之后：`LINK_CMD_MEAS_GET/XCHK` 的 payload 布局（9 路 × 4B + 状态）在实现时定义，
  并沿用 `Link_PackStatus` 那种"编解码同一份代码"的做法；改动会动到 `LINK_CODE_VER`（见 §10.2）。
- 链路波特率：115200 下探测往返 <2ms，够用；若将来要传测量流再考虑提到 460800/921600（两片同时改）。
- U2 的 USART1（对外排针）目前没有使用者，保留给调试/扩展。
