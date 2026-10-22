# DMX512 灯光控制模块 —— STM32 端实现计划

> 依据 `req.md`（2.4 灯光控制模块）整理，聚焦 **STM32F103C8T6 下位机侧** 需要实现的内容。
> 硬件电路（RS485 收发器、12V 功率电源、3.3V 稳压）不在本计划范围内，仅涉及 MCU 软件。

---

## 0. 需求概述

STM32 承担两项核心任务：

1. **协议解析（上行）**：通过 UART 以 115200 bps 接收上位机 7 字节私有帧，在接收中断中逐字节解析（状态机），得到目标灯段与 RGBW 颜色值。
2. **DMX512 输出（下行）**：将颜色数据按 DMX512 协议以 250 kbps 经 RS485 差分信号发送给 512 灯带，帧时序（Break/MAB）由硬件定时器保证微秒级精度，刷新率 ≥ 44 Hz。

数据流：`上位机 → UART(115200) → STM32 → RS485(250kbps DMX512) → 512 灯带`

---

## 1. 工程背景说明（重要）

- 现有工程基于 **STM32 标准外设库（FWlib，V3.5.0）**，而非 `req.md` 中提到的 HAL。
- 因此协议/驱动代码使用标准外设库 API（`USART_*`、`TIM_*`、`GPIO_*`、`NVIC_*`）实现，功能与 HAL 版本等价。
- 芯片：STM32F103C8T6（Cortex-M3，72 MHz）。

---

## 2. 模块划分与新增文件

| 文件 | 职责 |
|------|------|
| `User/protocol.h` / `User/protocol.c` | 协议驱动层：UART 接收中断状态机解析 7 字节帧，调用 DMX512 接口 |
| `User/dmx512.h` / `User/dmx512.c` | DMX512 驱动层：输出缓冲区、Break/MAB 时序、250kbps 发送、通道映射 |
| `User/main.c`（修改） | 初始化各外设、主循环触发 DMX512 帧发送 |
| `User/stm32f10x_it.c`（修改） | 添加 UART 接收中断、定时器中断处理 |

---

## 3. 详细任务

### 3.1 协议驱动层（`protocol.c` / `protocol.h`）

**帧格式（固定 7 字节）：**

| 偏移 | 字段 | 取值 | 含义 |
|------|------|------|------|
| 0 | Frame head 1 | `0xAA` | 帧头第一字节 |
| 1 | Frame head 2 | `0x55` | 帧头第二字节 |
| 2 | Segment | `0x00~0xAA` | 目标段号，`0x00` 广播全部段 |
| 3 | Red | `0x00~0xFF` | 红色亮度 |
| 4 | Green | `0x00~0xFF` | 绿色亮度 |
| 5 | Blue | `0x00~0xFF` | 蓝色亮度 |
| 6 | White | `0x00~0xFF` | 白色亮度 |

**状态机（3 状态）：**

| 状态 | 枚举值 | 含义 |
|------|--------|------|
| `STATE_IDLE` | 0 | 等待帧头第一字节 |
| `STATE_HEAD1` | 1 | 已收到 `0xAA`，等待 `0x55` |
| `STATE_PLAYLOAD` | 2 | 帧头校验通过，接收 5 字节负载（Segment + RGBW） |

**转移规则：**
- IDLE：收到 `0xAA` → HEAD1；否则停留 IDLE。
- HEAD1：收到 `0x55` → PLAYLOAD；收到 `0xAA` → 保持 HEAD1（容忍帧头重复）；收到其他字节 → 回退 IDLE。
- PLAYLOAD：累计接收 5 字节后 → 校验通过 → 写 DMX512 缓冲区 → 回 IDLE。
- 非法帧（如 `12 34 56 78`）自动回退 IDLE，原灯带状态保持不变。

**实现要点：**
- 采用 **UART 接收中断逐字节处理**，无阻塞、无队列、无堆分配。
- 状态机、负载计数器、暂存数组定义为模块级 `static`（或 `volatile`）变量。
- 解析完成后调用 `DMX512_SetSegment(segment, r, g, b, w)`。
- 对外提供初始化接口 `Protocol_Init()` 与中断回调入口 `Protocol_RxByte(uint8_t b)`。

### 3.2 DMX512 驱动层（`dmx512.c` / `dmx512.h`）

**输出缓冲区：**
- `uint8_t dmx_buffer[513]`（`dmx_buffer[0]` 为 Start Code `0x00`，`dmx_buffer[1..512]` 为 512 通道数据）。

**通道映射：**
```
DMX_startchannel = (segment - 1) * 4 + 1      // segment ∈ [1,170]
```
- 单段（4 通道）：`buffer[ch]=R, buffer[ch+1]=G, buffer[ch+2]=B, buffer[ch+3]=W`。
- 广播（`segment == 0`）：遍历全部段（实际灯带 20 段），逐段写入相同 RGBW。

**核心接口：**
```c
void DMX512_Init(void);                                     // 初始化 USART/TIM/GPIO
void DMX512_SetSegment(uint8_t segment, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void DMX512_SendFrame(void);                                // 发送一帧，由主循环或定时器调用
```

**时序要求（微秒级，硬件定时器实现）：**

| 参数 | 数值 |
|------|------|
| 波特率 | 250 kbps，8 数据位 + 2 停止位（每字节 44 µs） |
| Break（低电平） | ≥ 92 µs（取 100 µs 左右） |
| MAB（高电平） | ≥ 12 µs（取 12~20 µs） |
| Start Code | `0x00` |
| 数据通道 | 最多 512 字节 |

**发送流程（推荐方案：USART + GPIO + 定时器）：**
1. 关闭/暂停 USART，将 TX 引脚复用为 GPIO 输出低电平（Break）。
2. 定时器延时 ~100 µs 后拉高 TX（MAB）~12 µs。
3. TX 引脚复用回 USART 的 TX，使能发送，依次发送 `0x00` + 512 字节。
4. 等待发送完成（TC 标志 / DMA 完成）后切换 RS485 方向为接收，等待下一帧。
5. 刷新周期约 23 ms（≈ 44 Hz），可由 SysTick 或定时器控制。

> 可选优化：使用 DMA 发送 513 字节，减少 CPU 占用；方向切换须在 TC（发送完成）后执行，避免截断末字节。

### 3.3 UART 初始化（115200 接收）

- 选择空闲 UART（如 `USART3`），波特率 `115200`、8 数据位、无校验、1 停止位。
- 使能接收中断（`USART_IT_RXNE`）并配置 NVIC 优先级。
- 配置对应 RX 引脚为浮空输入（GPIO），TX 可暂不用于上行。

### 3.4 DMX512 发送定时器初始化

- 选择一个硬件定时器（如 `TIM2`）用于生成 Break/MAB 延时，或使用中断周期控制帧刷新。
- 关键点：**帧时序必须由硬件定时器保证**，不可使用毫秒级软件延时。

### 3.5 GPIO 与 RS485 方向控制

- 配置 TX 引脚（可复用，用于 Break 拉低/拉高）。
- 配置 RS485 收发器方向控制引脚（`DE`/`RE`）：发送时输出高，发送完成后输出低（回到接收态）。
- 灯带 12V 功率电源独立供电，MCU 仅输出逻辑控制，不直接驱动灯带功率。

### 3.6 中断服务函数（`stm32f10x_it.c`）

- `USARTx_IRQHandler`：读取 `USART_ReceiveData`，清除标志，调用 `Protocol_RxByte()`。
- 定时器中断（用于 Break/MAB 时序或帧周期）。
- 在 `stm32f10x_it.h` 中补充相应中断处理函数声明。

### 3.7 主循环整合（`main.c`）

- 初始化系统时钟、GPIO、UART、定时器、协议层、DMX512 层、NVIC。
- 主循环周期性调用 `DMX512_SendFrame()`（或由定时器/中断驱动）。
- 状态指示（可选，复用 `bsp_led` 的 RGB LED 作运行/错误指示）。

### 3.8 工程配置

- `stm32f10x_conf.h`：确保启用 `stm32f10x_usart.h`、`stm32f10x_tim.h`、`stm32f10x_gpio.h`、`stm32f10x_rcc.h`、`misc.h`（已默认包含）。
- Keil 工程（`Project/RVMDK（uv5）/BH-F103.uvprojx`）需将新增的 `protocol.c`、`dmx512.c` 加入编译源文件列表。

---

## 4. 测试用例（对应 req.md 表 2-6）

| 灯带状态 | 上位机数据 | 预期 |
|----------|------------|------|
| 单段变红 | `AA 55 01 FF 00 00 00` | 第 1 段变红 |
| 单段变蓝 | `AA 55 05 00 00 FF 00` | 第 5 段变蓝 |
| 广播变白 | `AA 55 00 00 00 00 FF` | 全部 20 段变白 |
| 帧头容错 | `AA AA 55 03 00 FF 00 00` | 忽略多余 `AA`，第 3 段变绿 |
| 非法帧过滤 | `12 34 56 78` | 状态机重置，灯带保持原状态 |

---

## 5. 注意事项 / 潜在问题

1. **库类型差异**：req 写 HAL，实际工程为标准外设库，实现时以 FWlib API 为准。
2. **segment 范围与通道上限矛盾**：`segment ∈ [1,170]` × 4 通道 = 680 通道 > DMX512 单链路 512 通道上限。实际灯带为 20 段（80 通道），实现时缓冲区按 512 通道设计，广播段数按实际灯带段数（20）配置，超出 128 段需跨 universe（当前需求暂不考虑）。
3. **时序精度**：Break/MAB 必须由硬件定时器产生，禁止用软件 `Delay()`（毫秒级、不可靠）。
4. **RS485 方向切换**：必须在 USART 发送完成（TC/DMA 完成）后把 `DE/RE` 拉低，避免末字节被截断或总线残留。
5. **中断实时性**：UART 接收解析在中断上下文完成，保持轻量（仅状态转移 + 写缓冲区），避免在中断内做阻塞操作。
6. **编码**：现有源文件为 GBK 编码（含中文注释乱码显示），新增文件建议统一使用 UTF-8 或与工程一致，避免中文注释乱码。

---

## 6. 接口定义与数据传递

### 6.1 模块依赖关系

```
main.c
 ├─► Protocol_Init()          // 协议层初始化（初始化状态机）
 ├─► DMX512_Init()            // DMX512 层初始化（USART/TIM/GPIO/NVIC）
 └─► DMX512_SendFrame()       // 主循环周期性发送 DMX512 帧

stm32f10x_it.c
 ├─► USARTx_IRQHandler()      // UART 接收中断 → Protocol_RxByte()
 ├─► TIMx_IRQHandler()        // Break/MAB 时序或帧周期定时
 └─► DMAx_Channelx_IRQHandler() // 可选：DMA 发送完成

protocol.c  ──调用──►  DMX512_SetSegment()   // 解析完成写入 DMX 缓冲区
```

### 6.2 协议层接口（`protocol.h`）

```c
/* 状态机状态 */
typedef enum {
    STATE_IDLE    = 0,   // 等待帧头第一字节 0xAA
    STATE_HEAD1   = 1,   // 已收到 0xAA，等待 0x55
    STATE_PLAYLOAD = 2   // 帧头校验通过，接收 5 字节负载
} ProtocolState_t;

void Protocol_Init(void);                 // 复位状态机，清空暂存
void Protocol_RxByte(uint8_t byte);       // UART 接收中断中逐字节调用
```

- `Protocol_RxByte()` 内部维护 `static` 状态机变量与负载索引，解析成功后调用 `DMX512_SetSegment()`。
- 模块对外只暴露这两个接口；状态、负载缓存、计数器均为 `static`，不暴露。

### 6.3 DMX512 层接口（`dmx512.h`）

```c
#define DMX512_CHANNEL_NUM     512u   // 单链路最大通道数
#define DMX512_SEGMENT_CHANNEL 4u     // 每段占用通道数（RGBW）
#define DMX512_START_CODE      0x00u  // DMX512 起始码

void    DMX512_Init(void);                                  // 初始化 USART/TIM/GPIO/NVIC
void    DMX512_SetSegment(uint8_t segment,
                          uint8_t r, uint8_t g,
                          uint8_t b, uint8_t w);            // 按段写 RGBW；segment==0 广播全段
void    DMX512_SetChannel(uint16_t ch, uint8_t value);      // 按单通道写（内部/调试用）
void    DMX512_SendFrame(void);                             // 发送完整一帧（主循环调用）
uint8_t DMX512_IsBusy(void);                                // 返回是否正在发送（供上层判断）
```

- 输出缓冲区：`static uint8_t dmx_buffer[DMX512_CHANNEL_NUM + 1]`，`dmx_buffer[0]` 为 Start Code `0x00`。
- 通道映射：`ch = (segment - 1) * 4 + 1`（`segment ∈ [1,170]`）。
- 广播（`segment == 0`）：遍历实际灯带段数（20 段），每段写入同一组 RGBW。

### 6.4 数据传递（数据流）

**上行（接收路径）**
```
上位机 7字节帧
   │  UART RX (115200, 8N1)
   ▼
USARTx_IRQHandler()
   │  USART_ReceiveData() 读 1 字节
   ▼
Protocol_RxByte(byte)
   │  状态机逐字节解析（IDLE→HEAD1→PLAYLOAD）
   │  收到完整 7 字节后：
   ▼
DMX512_SetSegment(segment, r, g, b, w)
   │  计算起始通道 ch = (segment-1)*4+1
   │  写入 dmx_buffer[ch..ch+3]
   ▼
dmx_buffer[0..512]  （内存中的 DMX 映像）
```

**下行（发送路径）**
```
dmx_buffer[0..512]
   │  DMX512_SendFrame()
   ▼
① Break：TX 引脚复用为 GPIO 拉低，TIM 延时 ~100µs
② MAB  ：TX 拉高，TIM 延时 ~12µs
③ 数据 ：TX 复用回 USART，250kbps 8N2 发送 Start Code + 512 字节
④ 完成 ：TC/DMA 完成 → RS485 DE/RE 拉低（回到接收）
   │  RS485 差分信号
   ▼
512 像素灯带
```

**关键共享数据：`dmx_buffer[]`**
- 由 `protocol.c`（通过 `DMX512_SetSegment`）写入。
- 由 `dmx512.c` 的 `DMX512_SendFrame()` 读取发送。
- 中断写、主循环读：写入单位 ≤ 4 字节，读取为整帧拷贝/发送；建议发送时对 `dmx_buffer` 做快照或直接 DMA 发送，避免发送中途被中断改写。

---

## 7. STM32 标准外设库函数清单（FWlib V3.5.0）

### 7.1 时钟 RCC
| 函数 | 用途 |
|------|------|
| `RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)` / `RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)` | 使能 GPIOB（APB2）、USART3（APB1）时钟 |
| `RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE)` | 使能 TIM2 时钟 |

### 7.2 GPIO
| 函数 | 用途 |
|------|------|
| `GPIO_Init(GPIOx, &GPIO_InitStructure)` | 配置 USART TX/RX、RS485 DE/RE 引脚模式与速度 |
| `GPIO_SetBits(GPIOx, Pin)` | 拉高（MAB 高电平、DE/RE 置发送） |
| `GPIO_ResetBits(GPIOx, Pin)` | 拉低（Break 低电平、DE/RE 置接收） |
| `GPIO_PinRemapConfig(...)` | 本工程未使用（USART3 默认引脚 PB10/PB11，无需重映射） |

### 7.3 串口 USART
| 函数 | 用途 |
|------|------|
| `USART_Init(USART3, &USART_InitStructure)` | 配置 115200 8N1（接收） |
| `USART_Init(USARTx, &USART_InitStructure)` | 配置 250kbps 8N2（DMX 发送） |
| `USART_Cmd(USARTx, ENABLE)` | 使能 USART |
| `USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE)` | 使能接收中断 |
| `USART_ReceiveData(USARTx)` | 读取接收字节 |
| `USART_SendData(USARTx, data)` | 发送一字节 |
| `USART_GetFlagStatus(USARTx, USART_FLAG_TXE)` | 查询发送寄存器空 |
| `USART_GetFlagStatus(USARTx, USART_FLAG_TC)` | 查询发送完成（用于方向切换） |
| `USART_ClearFlag(USARTx, USART_FLAG_TC)` | 清除 TC 标志 |
| `USART_GetITStatus(USARTx, USART_IT_RXNE)` | 中断中判断接收就绪 |
| `USART_ClearITPendingBit(USARTx, USART_IT_RXNE)` | 清除中断挂起（读数据后自动清，可选） |

### 7.4 定时器 TIM
| 函数 | 用途 |
|------|------|
| `TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStructure)` | 配置计数周期/分频，生成精确 Break/MAB 延时 |
| `TIM_Cmd(TIMx, ENABLE)` | 使能定时器 |
| `TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE)` | 使能更新中断（延时到点） |
| `TIM_SetCounter(TIMx, 0)` | 清零计数 |
| `TIM_GetCounter(TIMx)` | 读取计数（轮询延时方案） |
| `TIM_GetITStatus(TIMx, TIM_IT_Update)` | 中断中判断更新事件 |
| `TIM_ClearITPendingBit(TIMx, TIM_IT_Update)` | 清除更新中断标志 |

### 7.5 中断控制器 NVIC（`misc.c`）
| 函数 | 用途 |
|------|------|
| `NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2)` | 配置中断优先级分组 |
| `NVIC_Init(&NVIC_InitStructure)` | 配置 USART/TIM/DMA 中断使能与抢占优先级 |

### 7.6 DMA（可选，用于 513 字节连续发送）
| 函数 | 用途 |
|------|------|
| `DMA_Init(DMA1_Channelx, &DMA_InitStructure)` | 配置内存→USART 发送 |
| `DMA_Cmd(DMA1_Channelx, ENABLE)` | 使能 DMA 通道 |
| `DMA_ITConfig(DMA1_Channelx, DMA_IT_TC, ENABLE)` | 使能传输完成中断 |
| `DMA_GetFlagStatus(DMA1_FLAG_TCx)` | 查询传输完成 |
| `DMA_ClearFlag(DMA1_FLAG_TCx)` | 清除完成标志 |

### 7.7 中断服务函数（在 `stm32f10x_it.c` 中实现，需同步在 `stm32f10x_it.h` 声明）
| 函数 | 用途 |
|------|------|
| `USART3_IRQHandler()` | UART 接收中断，调用 `Protocol_RxByte()` |
| `TIM2_IRQHandler()` | Break/MAB 定时或帧周期定时 |
| `DMA1_Channelx_IRQHandler()` | 可选：DMA 发送完成 |

### 7.8 结构体类型（初始化时使用）
| 结构体 | 来源 |
|--------|------|
| `GPIO_InitTypeDef` | `stm32f10x_gpio.h` |
| `USART_InitTypeDef` | `stm32f10x_usart.h` |
| `TIM_TimeBaseInitTypeDef` | `stm32f10x_tim.h` |
| `NVIC_InitTypeDef` | `misc.h` |
| `DMA_InitTypeDef` | `stm32f10x_dma.h` |

> 注：以上库函数在 `Libraries/FWlib/src/` 下均有实现（`stm32f10x_gpio.c`、`stm32f10x_usart.c`、`stm32f10x_tim.c`、`stm32f10x_dma.c`、`misc.c` 等），`stm32f10x_conf.h` 已默认包含对应头文件。

---

## 8. 调试模块（`debug.h` / `debug.c`）

### 8.1 目的

独立于业务逻辑的轻量串口调试输出，用于运行期定位协议解析与 DMX512 发送问题。

### 8.2 硬件映射

| 项 | 值 |
|----|----|
| 调试控制台 UART | `USART3`，TX = `PB10`，RX = `PB11`（外部 USB-TTL） |
| 波特率 | 115200，8N1，全双工（收发） |
| 接线 | `PB10`（TX）、`PB11`（RX）经杜邦线接外部 USB-TTL，模块侧 TX↔RX 交叉、共地 |

> 调试控制台与上位机私有帧共用 `USART3`：首字节 `0xAA` 走帧协议，其余走命令行；DMX 发送口（`USART2`）独立，互不干扰。

### 8.3 接口

| 函数 / 宏 | 用途 |
|-----------|------|
| `Debug_Init()` | 初始化调试 UART |
| `Debug_SendByte(uint8_t)` | 发送一字节 |
| `Debug_SendString(const char *)` | 发送字符串 |
| `Debug_SendHex8(uint8_t)` | 一字节转 2 位大写十六进制（`XX`） |
| `Debug_SendHex16(uint16_t)` | 两字节转 4 位大写十六进制（`XXXX`） |
| `DBG_STR(s)` / `DBG_BYTE(b)` / `DBG_HEX8(v)` / `DBG_HEX16(v)` | 宏封装，`DEBUG_ENABLE=0` 时编译为空 |

### 8.4 特性

- **不依赖 printf / C 库重定向**，Keil AC5 下无论是否勾选 MicroLIB 均可编译。
- `debug.h` 顶部 `DEBUG_ENABLE` 为总开关，置 `0` 即从构建中剔除全部调试输出。
- `Debug_SendByte()` 阻塞等待 TXE；打印统一放在主循环的 `Protocol_Poll()` 中执行，**不在 UART 接收中断内打印**，避免阻塞中断而破坏 DMX512 帧时序。

### 8.5 埋点位置

| 位置 | 日志 | 触发条件 |
|------|------|----------|
| `main.c` | `=== DMX512 Light Controller Boot ===` | 上电最先打印 |
| `dmx512.c` 初始化 | `DMX512 Init OK` | 上电 |
| `protocol.c` 初始化 | `Protocol Init OK` | 上电 |
| 收到完整帧（`Protocol_Poll`） | `RX seg=XX R=XX G=XX B=XX W=XX -> DMX start ch=XXXX` | 每次解析成功一帧（单段） |
| 收到广播帧（`Protocol_Poll`） | `RX seg=00 ... -> broadcast all segments` | segment=0 时 |
| 非法字节（`Protocol_Poll`） | `ERR: bad byte 0xXX in HEAD1, reset` | HEAD1 状态收到非 `0xAA/0x55` 字节 |

### 8.6 串口助手期望输出

```
=== DMX512 Light Controller Boot ===
DMX512 Init OK
Protocol Init OK
RX seg=01 R=FF G=00 B=00 W=00 -> DMX start ch=0001
RX seg=05 R=00 G=00 B=FF W=00 -> DMX start ch=0011
RX seg=00 R=00 G=00 B=00 W=FF -> broadcast all segments
ERR: bad byte 0x12 in HEAD1, reset
```

---

## 9. 引脚分配（依据《野火F103指南者开发板IO分配表》）

### 9.1 本工程使用的引脚

| 功能 | 引脚 | 默认第二功能 | 说明 |
|------|------|--------------|------|
| DMX512 发送 TX | `PA2` | `USART2_TX` | 250kbps 8N2，复用推挽 |
| RS485 方向 DE/RE | `PA1` | `USART2_RTS` | 推挽输出，高=发送/低=接收 |
| 上位机接收 RX | `PB11` | `USART3_RX` | 115200 8N1，浮空输入 |
| 调试控制台 TX | `PB10` | `USART3_TX` | 115200 8N1，复用推挽（命令行回显） |
| 调试控制台 RX | `PB11` | `USART3_RX` | 与上位机 RX 共用，115200 8N1 |
| 延时定时器 | `TIM4` | — | Break/MAB 微秒级计时 |
| RGB 指示灯 R | `PB5` | `I2C1_SMBA/SPI3_MOSI` | 低电平点亮（红） |
| RGB 指示灯 G | `PB0` | `ADC12_IN8/TIM3_CH3` | 低电平点亮（绿） |
| RGB 指示灯 B | `PB1` | `ADC12_IN9/TIM3_CH4` | 低电平点亮（蓝） |

> 全部引脚均来自 IO 分配表的默认第二功能，无需重映射（`GPIO_PinRemapConfig` 未使用）。
> RGB LED 使用野火开发板板载三色 LED，由 `bsp_led.h` 统一宏定义。

### 9.2 状态指示灯（`status.h` / `status.c`）

用板载 RGB LED 颜色直观反馈系统状态（低电平点亮）：

| 状态 | 枚举 | LED 颜色 | R | G | B |
|------|------|----------|---|---|---|
| 上电/初始化 | `STATUS_BOOT` | 白 | 亮 | 亮 | 亮 |
| 正常运行 | `STATUS_RUNNING` | 绿 | 灭 | 亮 | 灭 |
| 收到单段帧 | `STATUS_SINGLE` | 蓝 | 灭 | 灭 | 亮 |
| 收到广播帧 | `STATUS_BROADCAST` | 青 | 灭 | 亮 | 亮 |
| 非法帧/错误 | `STATUS_ERROR` | 红 | 亮 | 灭 | 灭 |

- `Status_Set()` 仅记录状态（中断安全），`Status_Refresh()` 在主循环中刷新到 LED。
- 复用了 `bsp_led` 的 `LED1/LED2/LED3` 宏与 `LED_GPIO_Config()`。

---

## 10. 交互式调试命令行（`shell.h` / `shell.c`）

### 10.1 目的

把独立调试口 `USART3`（PB10/PB11，经杜邦线接外部 USB-TTL）升级为**全双工交互式控制台**：上位机通过串口助手发送文本指令，STM32 解析并执行（设置灯带颜色、查询状态、跑自检等），再把结果回显到同一串口。用于脱离上位机私有帧协议的快速人工调试与灯带验收。

### 10.2 硬件映射

| 项 | 值 |
|----|----|
| 控制台 UART | `USART3`，TX = `PB10`，RX = `PB11`（外部 USB-TTL） |
| 波特率 | 115200，8N1 |
| 接线 | `PB10`/`PB11` 经杜邦线接外部 USB-TTL（TX↔RX 交叉、共地） |

> RX 接收统一在 `USART3_IRQHandler`（`debug.c`）内完成，按首字节 `0xAA` 分流：帧协议 → `Protocol_RxByte()`，其余 → `Shell_RxByte()` 入环形缓冲；命令解析与执行在 `Shell_Poll()`（主循环）中完成，避免破坏 DMX512 帧时序。

### 10.3 指令集（不区分大小写，数值支持十进制或 `0x` 十六进制）

| 指令 | 参数 | 说明 |
|------|------|------|
| `help` / `?` | — | 打印帮助 |
| `info` | — | 打印固件与引脚映射信息 |
| `echo <text...>` | 任意文本 | 回显测试串口连通性 |
| `set <seg> <r> <g> <b> <w>` | seg 0~20，r/g/b/w 0~255 | 设置第 `seg` 段（`0` = 广播全部段）RGBW |
| `all <r> <g> <b> <w>` | r/g/b/w 0~255 | 广播设置所有段（等价 `set 0 ...`） |
| `ch <channel> <value>` | channel 1~512，value 0~255 | 写单个 DMX 通道 |
| `off` | — | 所有段熄灭 |
| `status` | — | 打印系统状态 + 第 1 段当前数据 |
| `test` | — | 跑红→绿→蓝→白→灭自检动画 |

### 10.4 接口

```c
void Shell_Init(void);           // 复位缓冲、打印横幅与提示符 "> "
void Shell_RxByte(uint8_t b);    // USART3 接收中断中调用，入环形缓冲
void Shell_Poll(void);           // 主循环调用：解析整行命令、推进自检动画
```

### 10.5 测试指令示例（RGBW 四色灯带）

```text
help                          # 查看全部指令
info                          # 查看固件/引脚信息
echo hello dmx512             # 测试串口收发连通
set 1 255 0 0 0               # 第 1 段变红
set 1 0 255 0 0               # 第 1 段变绿
set 1 0 0 255 0               # 第 1 段变蓝
set 1 0 0 0 255               # 第 1 段变白
all 255 0 0 0                 # 全部 20 段变红
all 0 0 0 255                 # 全部 20 段变白（白通道，用于暖白灯珠）
set 5 255 255 0 0             # 第 5 段黄（红+绿）
set 5 0 255 255 0             # 第 5 段青（绿+蓝）
set 5 255 0 255 0             # 第 5 段品红（红+蓝）
set 5 128 128 128 128         # 第 5 段 50% 全开（含白）
ch 1 255                      # 直接写通道 1 = 255（第 1 段 R）
ch 4 255                      # 直接写通道 4 = 255（第 1 段 W）
status                        # 查询状态与第 1 段数据
test                          # 跑自检动画
off                           # 全部熄灭
```

> 注意：`set`/`all`/`ch`/`off` 会立即写入 `dmx_buffer`，由于主循环持续发送 DMX 帧，灯带约在下一帧（~23ms 内）更新。
