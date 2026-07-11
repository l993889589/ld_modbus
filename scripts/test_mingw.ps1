<#
.SYNOPSIS
Configures, builds, and tests ld_modbus with the installed MinGW toolchain.

.DESCRIPTION
The generator is explicit because some Windows hosts retain a stale default
Visual Studio generator. The script never removes an existing build tree.
#>

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root "build\mingw-debug"

cmake -S $root -B $build -G "MinGW Makefiles" `
    -DCMAKE_BUILD_TYPE=Debug `
    -DLD_MODBUS_BUILD_TESTS=ON
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

cmake --build $build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

ctest --test-dir $build --output-on-failure
exit $LASTEXITCODE

