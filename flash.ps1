# ============================================================
#  STM32F407 烧录脚本 (Hero-Gimbal)
#  用法: .\flash.ps1 [-Release] [-NoReset]
#    -Release   烧录 Release 版本（默认 Debug）
#    -NoReset   烧录后不复位（默认烧录后复位运行）
# ============================================================
param(
    [switch]$Release,
    [switch]$NoReset
)

$ErrorActionPreference = "Stop"

# ---- 可配置项 ----
$CLT_ROOT = "E:\STM32CubeCLT_1.18.0"
$PROG     = Join-Path $CLT_ROOT "STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
$ProjectRoot = "E:\stm32_project\arm\123"
$Config   = if ($Release) { "Release" } else { "Debug" }
$ELF      = Join-Path $ProjectRoot "build\$Config\Hero-Gimbal.elf"
$PORT     = "SWD"
$MODE     = "UR"

# 烧录主流程
function Invoke-Flash {
    Write-Host ""
    Write-Host "[1/3] 连接 ST-Link 与目标芯片..." -ForegroundColor Yellow
    & $PROG -c "port=$PORT" "mode=$MODE"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[错误] 无法连接目标芯片。请检查 ST-Link 连接与供电。" -ForegroundColor Red
        exit 1
    }
    Write-Host "      连接成功。" -ForegroundColor Green

    Write-Host ""
    Write-Host "[2/3] 烧录并校验固件..." -ForegroundColor Yellow
    & $PROG -c "port=$PORT" "mode=$MODE" -w $ELF -v
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[错误] 烧录失败。" -ForegroundColor Red
        exit 1
    }

    if (-not $NoReset) {
        Write-Host ""
        Write-Host "[3/3] 复位芯片，开始运行..." -ForegroundColor Yellow
        & $PROG -c "port=$PORT" "mode=$MODE" -rst
    }
}

# ---- 检查 ----
if (-not (Test-Path $PROG)) {
    Write-Host "[错误] 找不到 STM32_Programmer_CLI.exe: $PROG" -ForegroundColor Red
    exit 1
}
if (-not (Test-Path $ELF)) {
    Write-Host "[错误] 找不到固件文件: $ELF" -ForegroundColor Red
    Write-Host "       请先在 CLion 中编译 $Config 版本。" -ForegroundColor Red
    exit 1
}

Write-Host "============================================================" -ForegroundColor Cyan
Write-Host " STM32F407 烧录脚本 (Hero-Gimbal)" -ForegroundColor Cyan
Write-Host " 配置: $Config" -ForegroundColor Cyan
Write-Host " 固件: $ELF" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan

# ---- 执行烧录 ----
Invoke-Flash

Write-Host ""
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host " 烧录完成！" -ForegroundColor Green
Write-Host "============================================================" -ForegroundColor Cyan
