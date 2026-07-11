<#
.SYNOPSIS
Checks the portable-core source contract before a release or CI build.
#>

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$sourceFiles = Get-ChildItem -Path "$root\include", "$root\src", "$root\integrations" -Recurse -File -Include *.c, *.h
$failures = [System.Collections.Generic.List[string]]::new()

foreach ($file in $sourceFiles) {
    $text = Get-Content -LiteralPath $file.FullName -Raw
    if ($text -notmatch '@file\s+') {
        $failures.Add("missing @file header: $($file.FullName)")
    }
    if ($text -match '\b(malloc|calloc|realloc|free)\s*\(') {
        $failures.Add("dynamic allocation call: $($file.FullName)")
    }
}

$portableFiles = Get-ChildItem -Path "$root\include", "$root\src" -Recurse -File -Include *.c, *.h
foreach ($file in $portableFiles) {
    $text = Get-Content -LiteralPath $file.FullName -Raw
    if ($text -match '\b(HAL_|osThread|tx_|xTask|socket\s*\()') {
        $failures.Add("platform/OS dependency in portable core: $($file.FullName)")
    }
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host "quality_check: file headers, static-memory contract, and core portability verified"
