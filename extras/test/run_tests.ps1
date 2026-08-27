# Build and run the PIDEasy-Improved host test suite.
# Usage:  .\run_tests.ps1
#
# Needs a C++ compiler on PATH (g++ from MSYS2/MinGW, or set $env:CXX).

$ErrorActionPreference = "Stop"

$dir = $PSScriptRoot
$src = Join-Path $dir "..\..\src"
$cxx = if ($env:CXX) { $env:CXX } else { "g++" }

if (-not (Get-Command $cxx -ErrorAction SilentlyContinue)) {
    Write-Error "Compiler '$cxx' not found on PATH. Install MSYS2/MinGW g++ or set `$env:CXX."
}

$exe = Join-Path $dir "test_pideasy.exe"

& $cxx -std=c++11 -Wall -Wextra -I $dir -I $src `
    (Join-Path $dir "test_pideasy.cpp") (Join-Path $src "PIDEasy.cpp") -o $exe
if ($LASTEXITCODE -ne 0) { Write-Error "Compilation failed." }

& $exe
exit $LASTEXITCODE
