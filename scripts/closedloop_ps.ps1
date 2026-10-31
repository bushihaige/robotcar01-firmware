# 闭环阶跃粗调（Iteration 006.5 验收 A3）
#
# 一次串口会话内自动完成"逐档设定目标速度 + 持续采集"，避免多会话争用 COM 口。
#   - '+' 命令在 BoardApp 里会同时切到闭环模式并 +0.05 m/s 目标，因此不需要先发 'o'
#   - 每档保持 DwellMs，让 PI 有时间稳定
#   - 结束发 's' 停止
#
# 用法（Honor PC）：
#   powershell -ExecutionPolicy Bypass -File closedloop_ps.ps1 -Port COM16
#
# 输出：与 capture_uart_ps.ps1 同格式 "<elapsed_ms>\t<设备行>"，可直接喂给 step_metrics.py。

param(
    [string]$Port = "COM16",
    [int]$Baud = 115200,
    [int]$Steps = 4,          # 4 次 '+' ⇒ 0.05 / 0.10 / 0.15 / 0.20 m/s
    [int]$DwellMs = 6000,
    [int]$WarmupMs = 1500,
    [string]$Out = "E:\File\robotcar01_bringup\closedloop_log.txt"
)

$ErrorActionPreference = "Stop"

$sp = New-Object System.IO.Ports.SerialPort
$sp.PortName = $Port
$sp.BaudRate = $Baud
$sp.Parity = [System.IO.Ports.Parity]::None
$sp.DataBits = 8
$sp.StopBits = [System.IO.Ports.StopBits]::One
$sp.Handshake = [System.IO.Ports.Handshake]::None
$sp.ReadTimeout = 100
$sp.DtrEnable = $true
$sp.RtsEnable = $false

$lines = New-Object System.Collections.Generic.List[string]
$buffer = ""

try {
    $sp.Open()
} catch {
    Write-Output ("OPEN_FAILED: " + $_.Exception.Message)
    exit 1
}

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$next_send_at = $WarmupMs
$sent = 0
$started = $false
$stop_sent = $false

while ($true) {
    $chunk = ""
    try { $chunk = $sp.ReadExisting() } catch { $chunk = "" }
    if ($chunk.Length -gt 0) {
        $buffer += $chunk
        while ($true) {
            $idx = $buffer.IndexOf("`n")
            if ($idx -lt 0) { break }
            $line = $buffer.Substring(0, $idx).TrimEnd("`r")
            $buffer = $buffer.Substring($idx + 1)
            if ($line.Length -gt 0) {
                $lines.Add(("{0}`t{1}" -f [int]$sw.Elapsed.TotalMilliseconds, $line))
            }
        }
    }

    $now = $sw.Elapsed.TotalMilliseconds
    if (-not $started -and $now -ge $WarmupMs) {
        $started = $true
        $next_send_at = $now
    }

    if ($started -and $sent -lt $Steps -and $now -ge $next_send_at) {
        try { $sp.Write("+") } catch { }
        ++$sent
        $lines.Add(("{0}`t[CMD]+ step {1} (target {2:N2} m/s)" -f [int]$now, $sent, (0.05 * $sent)))
        $next_send_at = $now + $DwellMs
    } elseif ($started -and $sent -ge $Steps -and -not $stop_sent -and $now -ge $next_send_at) {
        try { $sp.Write("s") } catch { }
        $stop_sent = $true
        $lines.Add(("{0}`t[CMD]s (stop)" -f [int]$now))
        $next_send_at = $now + 2500
    } elseif ($stop_sent -and $now -ge $next_send_at) {
        break
    }

    Start-Sleep -Milliseconds 12
}

try { $sp.Write("s") } catch { }
Start-Sleep -Milliseconds 200
$sp.Close()
$lines | Set-Content -Path $Out -Encoding UTF8
Write-Output ("closedloop done: {0} lines -> {1}" -f $lines.Count, $Out)
