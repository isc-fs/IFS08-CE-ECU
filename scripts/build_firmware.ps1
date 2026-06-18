param(
    [string]$BuildDir = "build-fw",
    [string]$OutDir = "bot"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$toolchainFile = Join-Path $repoRoot "cmake\gcc-arm-none-eabi.cmake"
$buildPath = Join-Path $repoRoot $BuildDir
$firmwarePath = Join-Path $repoRoot "firmware"
$outPath = Join-Path $repoRoot $OutDir

$gcc = Get-ChildItem "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins" -Recurse -Filter arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue |
    Sort-Object FullName |
    Select-Object -First 1 -ExpandProperty FullName

if (-not $gcc) {
    throw "No se encontro arm-none-eabi-gcc.exe dentro de STM32CubeIDE."
}

$toolBin = Split-Path -Parent $gcc
$env:Path = "$toolBin;$env:Path"
$objcopy = Join-Path $toolBin "arm-none-eabi-objcopy.exe"

if (Test-Path $buildPath) {
    Remove-Item -LiteralPath $buildPath -Recurse -Force
}

cmake -G "MinGW Makefiles" -S $firmwarePath -B $buildPath "-DCMAKE_TOOLCHAIN_FILE=$toolchainFile"
cmake --build $buildPath

if (-not (Test-Path $outPath)) {
    New-Item -ItemType Directory -Path $outPath | Out-Null
}

$elfSrc = Join-Path $buildPath "ECU08.elf"
$elfDst = Join-Path $outPath "ECU08.elf"
$binDst = Join-Path $outPath "ECU08.bin"

Copy-Item -LiteralPath $elfSrc -Destination $elfDst -Force
& $objcopy -O binary $elfSrc $binDst

Write-Host ""
Write-Host "Firmware listo en:"
Write-Host "  ELF: $elfDst"
Write-Host "  BIN: $binDst"
