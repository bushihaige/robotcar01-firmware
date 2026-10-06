# RobotCar01 UART capture (runs on the Honor PC, timestamped) -- Iteration 002.5
#
# Purpose: produce a timestamped serial log so heartbeat intervals can be measured
# (review H-5: the heartbeat period must be quantified, not eyeballed).
# Output line format: <elapsed_ms>TAB<original line>
#
# NOTE: this file must stay ASCII-only. Windows PowerShell 5.1 reads .ps1 files using
# the system ANSI code page unless a BOM is present, so non-ASCII comments get
# mis-decoded and break parsing ("UnexpectedToken").
#
# Usage (normally invoked by ../scripts/capture_uart.sh from the Mac):
#   powershell -ExecutionPolicy Bypass -File capture_uart_ps.ps1
# Optional params: -Port COM15 -Baud 115200 -Seconds 45 -Out E:\File\robotcar01_bringup\uart_log.txt
#
# Start this BEFORE resetting the board, otherwise the boot banner is missed.

param(
  [string]$Port = "COM15",
  [int]$Baud = 115200,
  [int]$Seconds = 45,
  [string]$Out = "E:\File\robotcar01_bringup\uart_log.txt"
)

$ErrorActionPreference = "Stop"

$outDir = Split-Path -Parent $Out
if ($outDir -and -not (Test-Path $outDir)) {
  New-Item -ItemType Directory -Force -Path $outDir | Out-Null
}

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 500
$sp.NewLine = "`n"
$sp.Open()

$sw = New-Object System.IO.StreamWriter($Out, $false, [System.Text.Encoding]::ASCII)
$sw.NewLine = "`n"
$watch = [System.Diagnostics.Stopwatch]::StartNew()

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
Write-Output ("captured {0}s on {1} -> {2}" -f $Seconds, $Port, $Out)
