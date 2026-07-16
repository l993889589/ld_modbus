# ld_modbus

`ld_modbus` 是面向 MCU 的独立 C99 Modbus 协议栈，支持 RTU/TCP、主站/客户端和从站/服务器，不使用动态内存，不依赖 STM32 HAL、RTOS、Socket 或 LDC。

当前版本：`0.2.0`。

## 功能范围

- Modbus RTU ADU 编解码与 CRC16；
- Modbus TCP MBAP 编解码；
- 主站请求构造与响应解析；
- 从站映射及 FC01、02、03、04、05、06、0F、10、16、17；
- 广播写、地址过滤和标准异常响应；
- 直接连接 UART 的 RTU T1.5/T3.5 接收器；
- 全部缓冲区和寄存器表由调用者静态提供；
- MinGW/GCC 主机回归测试。

## 依赖与内存约束

库中不会调用 `malloc`、`calloc`、`realloc` 或 `free`。协议上下文、双接收缓冲区、响应缓冲区和寄存器表全部属于应用程序。

`ld_modbus` 不依赖 LDC。早期 STM32 示例曾使用 LDC 收集 RTU 数据，但从 `0.2.0` 开始，RTU 接收时序和双缓冲完整地由 `ld_modbus_rtu_framer` 提供。

## 分层

```text
UART/BSP：收发字节、提供字节完成时间戳和当前时间
                         │
                         ▼
ld_modbus_rtu_framer：T1.5/T3.5、超长帧、双缓冲、claim/release
                         │ 完整 RTU ADU
                         ▼
ld_modbus RTU codec + client/server
                         │
                         ▼
应用静态寄存器表和业务逻辑
```

UART 层不需要理解 Modbus；协议层也不包含 MCU 寄存器或 HAL 调用。

## 最小 RTU 接收流程

```c
static ld_modbus_rtu_framer_t receiver;
static uint8_t rx_active[LD_MODBUS_RTU_MAX_ADU_LENGTH];
static uint8_t rx_ready[LD_MODBUS_RTU_MAX_ADU_LENGTH];

ld_modbus_rtu_framer_init(&receiver,
                          rx_active,
                          rx_ready,
                          sizeof(rx_active),
                          115200U,
                          10U,          /* 8N1 = 10 bits/character */
                          timer_hz);
```

UART 单字节接收中断在读取 RDR 后立即记录同一个自由运行计数器：

```c
void uart_rx_irq(void)
{
    uint32_t errors = uart_get_and_clear_errors();
    uint8_t byte = uart_read_rdr();
    uint32_t completed_at = timer_get_ticks();

    if(errors != 0U)
        ld_modbus_rtu_framer_on_error(&receiver, completed_at);
    else
        ld_modbus_rtu_framer_on_byte(&receiver, byte, completed_at);
}
```

这种“ISR 中读软件计数器”的值是字符完成后的软件观察时刻，不是 UART
硬件捕获时刻；相邻时间戳的差值包含两次中断延迟之差。它可以作为轻载、
抖动有界系统的实现，但在测得最坏中断延迟并留出 T1.5 判定裕量之前，不能
宣称达到严格硬件时序合规。需要严格证明时，应由平台提供硬件关联的字符
完成/接收超时事件或等价保证；协议栈本身仍不绑定具体定时器。

主循环或任务使用同一个计数器轮询：

```c
disable_uart_rx_irq();
if(uart_rx_quiescent()) /* no BUSY, RXNE, or receive error pending */
    ld_modbus_rtu_framer_poll(&receiver, timer_get_ticks());
enable_uart_rx_irq();
```

检查硬件静止状态是必要的：任务屏蔽 UART RX 后，如果一个字符已经完成但
RXNE 尚未交给回调，或者字符仍处于 BUSY，不能先提交旧帧。`T3.5 + 1 字符`
解决的是“字符尚未完成”的观察延迟，不能替代平台对 pending/in-progress
接收事件的门控。

任务只在很短的临界区内 claim/release 元数据，协议处理期间不需要关中断：

```c
ld_modbus_rtu_frame_view_t frame;

disable_uart_rx_irq();
bool available = ld_modbus_rtu_framer_claim(&receiver, &frame);
enable_uart_rx_irq();

if(available)
{
    process_complete_rtu_adu(frame.data, frame.length);

    disable_uart_rx_irq();
    ld_modbus_rtu_framer_release(&receiver, &frame);
    enable_uart_rx_irq();
}
```

完成帧发布只交换两个缓冲区指针，不会在 UART ISR 中复制最多 256 字节。完成帧尚未释放时，后续完整帧会被丢弃并计入 `dropped_while_ready`，不会覆盖已 claim 的数据。

## T1.5/T3.5 规则

- 波特率不高于 19200：按实际每字符位数计算 T1.5/T3.5；
- 波特率高于 19200：使用 Modbus Serial Line 建议的 750 us/1750 us；
- 时间戳必须表示每个字符接收完成的时刻；
- 相邻完成时间包含当前字符时间，因此字节路径比较 `字符时间 + T1.5/T3.5`；
- T1.5 违规后丢弃整个流，直到重新观察到合法 T3.5；
- UART 奇偶、帧、噪声或溢出错误必须调用 `on_error()`，当前半帧会立即作废；
- 只有字节完成时间戳时，自动 poll 在 `T3.5 + 1 字符时间` 后提交，避免把已经开始但尚未完成的字符错误切到下一帧。

详细契约见 [`docs/rtu-timing-porting.md`](docs/rtu-timing-porting.md)。

## DMA 与 ReceiveToIdle

只有“整块数据 + 块末时间戳”时，无法严格证明块内不存在 T1.5 非法间隔。严格 RTU 接入必须满足至少一项：

- 每字节完成时间戳；
- UART 硬件接收超时/捕获事件能证明字符间隔；
- 驱动明确保证在协议所需的间隔处分块，并给出可靠的最后字节时间。

普通 DMA/ReceiveToIdle 回调不能被默认宣传为严格 T1.5 检测。

## 构建与测试

```powershell
cmake -S . -B build/direct -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=Debug `
  -DLD_MODBUS_BUILD_TESTS=ON
cmake --build build/direct
ctest --test-dir build/direct --output-on-failure
```

也可以运行：

```powershell
.\scripts\test_mingw.ps1
.\scripts\check_quality.ps1
```

质量检查会拒绝协议核心中的堆分配、HAL、RTOS 和 Socket 依赖。

## 从站处理入口

```c
ld_modbus_server_action_t action;
size_t response_length;

ld_modbus_status_t status = ld_modbus_server_process_rtu_adu(
    &map,
    1U,
    frame.data,
    frame.length,
    response,
    sizeof(response),
    &response_length,
    &action);
```

线圈和保持寄存器允许远程写入；离散输入和输入寄存器对远程主站只读。应用仍可通过本地 checked accessor 更新传感器值。

## License

Apache-2.0。Modbus 是 Schneider Electric 的注册商标；本项目与 Schneider Electric 和 Modbus Organization 无隶属关系。
