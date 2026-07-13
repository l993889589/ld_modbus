# ld_modbus

STM32H563 示例、原工程的 LDC 依赖关系及两个示例工程入口，见
[`docs/STM32H563示例与LDC说明.md`](docs/STM32H563示例与LDC说明.md)。

`ld_modbus` is a C99 Modbus protocol stack for microcontrollers and other
resource-constrained systems.

The v0.1 development baseline provides:

- no dynamic memory allocation;
- Modbus RTU and Modbus TCP ADU support;
- client and server roles;
- transport, operating-system, and MCU independence;
- bounded buffers and explicit error results;
- optional platform-independent RTU T1.5/T3.5 receive framing;
- host tests for valid, malformed, boundary, and interoperability vectors.

## Memory contract

The library never calls `malloc`, `calloc`, `realloc`, or `free`. Contexts,
register maps, request buffers, response buffers, and transport storage belong
to the application.

## Layering

```text
UART DMA / interrupt timestamps / TCP socket
        |
optional RTU framer or LDC adapter
        |
RTU or TCP ADU codec
        |
Modbus PDU client/server core
        |
application-owned register map
```

LDC is an optional framing integration. The portable Modbus core can also be
used with any transport that supplies a complete RTU or TCP ADU.

`ld_modbus_rtu_framer` is the strict RTU receive helper shipped with the
library. It implements T1.5 rejection and T3.5 frame completion without HAL,
UART, timer, RTOS, or LDC dependencies. A platform port supplies each received
byte with an end-of-character microsecond timestamp and calls the poll API with
the same wrapping clock. The RTU/TCP codec and client/server core do not depend
on this optional helper.

To build the optional adapter tests, point CMake at a directory containing
`ldc_easy.c`, `ldc_core.c`, `ldc_packet.c`, `ldc_ring.c`, and their headers:

```powershell
cmake -S . -B build/ldc -G "MinGW Makefiles" `
  -DLD_MODBUS_LDC_ROOT=D:/path/to/ldc/core
cmake --build build/ldc
ctest --test-dir build/ldc --output-on-failure
```

## Build tests

```powershell
cmake --preset mingw-debug
cmake --build --preset mingw-debug
ctest --test-dir build/mingw-debug --output-on-failure
```

The preset selects the installed MinGW compiler explicitly, so an unrelated
or stale system-wide Visual Studio default cannot change the build result.
On Windows, `scripts/test_mingw.ps1` runs all three commands and also supports
older CMake versions that do not implement test presets.

If the host has `gcc.exe` but no Make or Ninja program, run the generator-free
fallback:

```powershell
.\scripts\test_gcc.ps1
```

## Status

The current implementation contains complete RTU/TCP ADU codecs, common client
request builders and response parsers, complete-ADU server entry points, RTU
broadcast/unit handling, and a static server map for function codes 01, 02,
03, 04, 05, 06, 0F, 10, 16, and 17. The distribution also contains a static,
platform-independent RTU T1.5/T3.5 framer.

Run the release contract check with:

```powershell
.\scripts\check_quality.ps1
```

The check rejects heap calls and HAL/RTOS/socket dependencies in the portable
core. GitHub Actions builds with warnings as errors and runs the host tests.

The STM32H563 reference integration, ThreadX/IT/DMA build matrix, W800 transport,
and RS485 hardware acceptance evidence are maintained in
[`l993889589/STM32`](https://github.com/l993889589/STM32/tree/codex/stm32h563-modbus/STM32H563_Modbus).

## Minimal server flow

```c
static uint16_t holding[64];
static uint8_t response[LD_MODBUS_RTU_MAX_ADU_LENGTH];

ld_modbus_server_map_t map = {0};
map.holding_registers = holding;
map.holding_registers_count = 64U;

/* request/request_length come from LDC or another complete-frame transport. */
ld_modbus_server_action_t action;
size_t response_length;
ld_modbus_status_t status = ld_modbus_server_process_rtu_adu(
    &map, 1U, request, request_length,
    response, sizeof(response), &response_length, &action);

if(status == LD_MODBUS_STATUS_OK && action == LD_MODBUS_SERVER_ACTION_REPLY)
    uart_send(response, response_length);
```

The example uses application-owned static storage. The optional LDC adapter
packages the same flow into `ld_modbus_ldc_rtu_server_poll()`.

## Application-side table access

The server map also provides checked helpers for application code. These
helpers use the configured start address, reject unmapped addresses, normalize
bit values to `0` or `1`, and never allocate memory:

```c
uint16_t value;

ld_modbus_server_map_write_holding_register(&map, 10U, 1234U);
ld_modbus_server_map_read_holding_register(&map, 10U, &value);
```

The complete helper set covers coils, discrete inputs, holding registers, and
input registers. Discrete inputs and input registers use local `set` helpers:
the application may update sensor values, while those tables remain read-only
to a remote Modbus client. The caller must provide synchronization if protocol
processing and application updates can run concurrently.

## License

Apache-2.0. Modbus is a registered trademark of Schneider Electric. This
project is independent and is not affiliated with Schneider Electric or the
Modbus Organization.
