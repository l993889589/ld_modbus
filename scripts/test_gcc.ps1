<#
.SYNOPSIS
Builds and runs ld_modbus tests using only GCC.

.DESCRIPTION
This fallback is intentionally independent of CMake build generators. It is
useful on minimal Windows hosts that provide gcc.exe but no make or Ninja.
#>

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $root "build\gcc-direct"
$executable = Join-Path $outputDirectory "ld_modbus_tests.exe"
$framerExecutable = Join-Path $outputDirectory "ld_modbus_rtu_framer_tests.exe"
$receiverExecutable = Join-Path $outputDirectory "ld_modbus_rtu_receiver_tests.exe"

New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null

& gcc `
    -std=c99 -Wall -Wextra -Wpedantic -Werror `
    -I (Join-Path $root "include") `
    (Join-Path $root "src\ld_modbus_core.c") `
    (Join-Path $root "src\ld_modbus_client.c") `
    (Join-Path $root "src\ld_modbus_server.c") `
    (Join-Path $root "tests\test_ld_modbus.c") `
    -o $executable
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $executable
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& gcc `
    -std=c99 -Wall -Wextra -Wpedantic -Werror `
    -I (Join-Path $root "include") `
    (Join-Path $root "src\ld_modbus_rtu_framer.c") `
    (Join-Path $root "tests\test_ld_modbus_rtu_framer.c") `
    -o $framerExecutable
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $framerExecutable
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& gcc `
    -std=c99 -Wall -Wextra -Wpedantic -Werror `
    -I (Join-Path $root "include") `
    (Join-Path $root "src\ld_modbus_core.c") `
    (Join-Path $root "src\ld_modbus_client.c") `
    (Join-Path $root "src\ld_modbus_server.c") `
    (Join-Path $root "src\ld_modbus_rtu_framer.c") `
    (Join-Path $root "tests\test_ld_modbus_rtu_receiver.c") `
    -o $receiverExecutable
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $receiverExecutable
exit $LASTEXITCODE
