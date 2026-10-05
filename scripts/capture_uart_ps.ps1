# RobotCar01 串口采集（在 Honor PC 上运行，带宿主时间戳）——Iteration 002.5
#
# 目的：为验收 4/5 提供**带时间戳**的串口日志（评审 H-5：心跳间隔必须可量化）。
# 输出每行格式： <elapsed_ms>\t<原始行>
# 用法（由 Mac 侧 scripts/capture_uart.sh 调用，或本机直接运行）：
#   powershell -ExecutionPolicy Bypass -File capture_uart_ps.ps1 -Port COM15 -Baud 115200 -Seconds 15

param(
  [string]$Port = "COM15",
  [int]$Baud = 115200,
  [int]$Seconds = 15,
  [string]$Out = "E:\File\robotcar01_bringup\uart_log.txt"
)

$ErrorActionPreference = "Stop"
$outDir = Split-Path -Parent $Out
if ($outDir -and -not (Test-Path $outDir)) { New-Item -ItemType Directory -Force -Path $outDir | Out-Null }

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 500
$sp.NewLine = "`n"
$sp.Open()

$sw = New-Object System.IO.StreamWriter($Out, $false, [System.Text.Encoding]::ASCII)
$sw.NewLine = "`n"
$watch = [System.Diagnostics.Stopwatch]::StartNew()

# 提示：应在复位板子**之前**启动本采集，否则会漏掉启动横幅（评审 C-6 第⑦项）。
while ($watch.Elapsed.TotalSeconds -lt $Seconds) {
  try {
    $line = $sp.ReadLine()
  } catch [System.TimeoutException] {
    continue
  } catch {
    break
  }
  if ($null -eq $line) { continue }
  $ms = [int]$watch.Elapsed.TotalMilliseconds
  $sw.WriteLine(("{0}`t{1}" -f $ms, $line.TrimEnd("`r")))
  $sw.Flush()
}

$sw.Close()
$sp.Close()
Write-Output ("captured {0}s to {1}" -f $Seconds, $Out)
