# Architecture contract

## 职责边界

`ld_modbus` 负责 Modbus RTU/TCP 线格式、主从协议语义和 RTU 接收时序，但不拥有 UART、RS-485 DE、DMA、Socket、任务、定时器或应用业务。

- 应用拥有全部静态内存；
- UART/BSP 搬运字节，并提供字符接收完成时间戳；
- `ld_modbus_rtu_framer` 把时间戳字节流变成完整 RTU ADU；
- codec/client/server 只处理完整 ADU；
- 应用负责协议上下文和寄存器表的并发保护。

LDC 不属于 `ld_modbus` 的构建、头文件或运行依赖。

## RTU 双缓冲

接收器使用两个调用者提供的等容量缓冲区：一个写入当前流，一个保存待处理帧。发布时交换指针，不复制完整帧。

完成帧通过 `claim/release` 固定生命周期。ready 槽被占用时，新完成帧被明确丢弃并计数；生产者绝不会覆盖 claimed 数据。这个单槽背压模型适合 Modbus 一问一答，不试图复制通用多帧队列。

## 执行模型

- `on_byte()` 和 `poll()` 是同一个逻辑生产者，必须串行；
- UART ISR 可调用 `on_byte()`；
- UART 接收错误事件调用 `on_error()`，不得静默清除数据丢失；
- 任务调用 `poll()` 时应短暂屏蔽该 UART 接收中断；
- 任务 claim/release 元数据时同样短暂屏蔽；
- claim 与 release 之间的 CRC、PDU 和业务处理不应全局关中断。

## RTU 时序

在不高于 19200 baud 时，根据 start/data/parity/stop 的总位数计算字符、T1.5 和 T3.5；更高波特率固定为 750 us 和 1750 us。

时间戳定义为字符接收完成时刻。相邻完成时间等于总线静默时间加当前字符时间，因此字节到达路径使用：

```text
completion_delta >  char_time + T1.5  -> 非法流
completion_delta >= char_time + T3.5  -> 旧帧完成，新帧开始
```

T1.5 违规不会把尾部当成新帧，而是进入 discard-until-T3.5。

任务自动提交使用 `T3.5 + char_time`。额外字符窗口不是修改 Modbus 帧间隔，而是保证任何在 T3.5 前开始的字符先完成并进入 `on_byte()`，避免仅凭“尚未完成的字符不可见”而提前提交。

32 位时间戳允许自然回绕，前提是所有相关间隔小于 `2^31` ticks，并且调用频率足够观察一次合法边界。反向或超过半周期的字节时间戳会作废当前流并计入 `timestamp_errors`；平台报告的接收错误计入 `rx_errors`。

## DMA 能力边界

一个 DMA 数据块只有一个结束时间戳时，块内 T1.5 无法被恢复。此类驱动可以交付完整 ADU，但不能宣称严格检测字符间非法间隔。严格模式需要每字节时间信息、硬件接收超时事件或等价保证。

## 非目标

- 动态寄存器映射发现；
- 隐藏的重试线程；
- UART/HAL 初始化；
- 通用串口帧队列；
- 与 libmodbus、nanoMODBUS 或 LDC 的源码兼容。
