# 开环占空比扫点（Iteration 006.5 验收 A2）
#
# 目的：一次串口会话内自动完成 "逐档加占空比 + 持续采集"，避免多会话争用同一 COM 口。
#   - 先发 '1' 切到仅左轮（安全：只驱动接线完成的那一侧）
#   - 每档发送 ']'（+10% 占空比），保持 DwellMs，全程连续读串口
#   - 结束发 's' 停止，并保证串口关闭前电机已断
#
# 用法（Honor PC）：
#   powershell -ExecutionPolicy Bypass -File sweep_ps.ps1 -Port COM16 -Steps 9 -DwellMs 2500
#
# 输出：每行 "<elapsed_ms>\t<设备行>"，与 capture_uart_ps.ps1 同格式，可直接喂给 sweep_table.py。

param(
    [string]$Port = "COM16",
    [int]$Baud = 115200,
    [int]$Steps = 9,
    [int]$DwellMs = 2500,
    [int]$WarmupMs = 1200,
    [string]$Out = "E:\File\robotcar01_bringup\sweep_log.txt"
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
$sent_count = 0
$started = $false

while ($true) {
    $chunk = ""
    try {
        $chunk = $sp.ReadExisting()
    } catch {
        $chunk = ""
    }
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

    if (-not $started -and $sw.Elapsed.TotalMilliseconds -ge $WarmupMs) {
        # 先切仅左轮，再开始加占空比
        try { $sp.Write("1") } catch { }
        $lines.Add(("{0}`t[CMD]1 (wheel=left)" -f [int]$sw.Elapsed.TotalMilliseconds))
        $started = $true
        $next_send_at = $sw.Elapsed.TotalMilliseconds + 400
    } elseif ($started -and $sent_count -lt $Steps -and $sw.Elapsed.TotalMilliseconds -ge $next_send_at) {
        try { $sp.Write("]") } catch { }
        ++$sent_count
        $lines.Add(("{0}`t[CMD]] duty step {1}" -f [int]$sw.Elapsed.TotalMilliseconds, $sent_count))
        $next_send_at = $sw.Elapsed.TotalMilliseconds + $DwellMs
    } elseif ($started -and $sent_count -ge $Steps -and $sw.Elapsed.TotalMilliseconds -ge $next_send_at) {
        # 全部档位走完：发停止并留一点时间记录减速
        try { $sp.Write("s") } catch { }
        $lines.Add(("{0}`t[CMD]s (stop)" -f [int]$sw.Elapsed.TotalMilliseconds))
        $next_send_at = $sw.Elapsed.TotalMilliseconds + 1500
        $sent_count = $Steps + 1000  # 只发一次
    } elseif ($sent_count -eq ($Steps + 1000) -and $sw.Elapsed.TotalMilliseconds -ge $next_send_at) {
        break
    }

    Start-Sleep -Milliseconds 12
}

try { $sp.Write("s") } catch { }   # 兜底：确保退出前已断输出
Start-Sleep -Milliseconds 200
$sp.Close()
$lines | Set-Content -Path $Out -Encoding UTF8
Write-Output ("sweep done: {0} lines -> {1}" -f $lines.Count, $Out)
