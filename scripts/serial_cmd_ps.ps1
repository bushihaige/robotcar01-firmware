# 串口单命令注入 + 回读（Iteration 006.5 验证 RX 命令通道）
#
# 用途：在不依赖人手按复位/看屏幕的前提下，验证板子的"命令→响应"链路是否工作。
#   - 打开串口会让板子被 DTR 复位，因此脚本 open 后立即开始读，尽力捕获启动横幅；
#   - 预热 WarmupMs 之后发送一个命令字节（默认 '1' = 仅左轮），
#     该命令的响应在状态行里表现为 wheel=left，易于与默认 wheel=both 区分。
#
# 用法（在 Honor PC 上）：
#   powershell -ExecutionPolicy Bypass -File serial_cmd_ps.ps1 -Port COM7 -Send 1 -Seconds 8
#
# 输出：每行 "<elapsed_ms>\t<设备行>"，另有一行 "<elapsed_ms>\t[SENT]x" 标记发送时刻。

param(
    [string]$Port = "COM7",
    [int]$Baud = 115200,
    [string]$Send = "1",
    [int]$Seconds = 8,
    [int]$WarmupMs = 1800,
    [string]$Out = "E:\File\robotcar01_bringup\serial_cmd.txt"
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
    $sp.Open()   # 注意：open 会因 DTR 复位板子
} catch {
    Write-Output ("OPEN_FAILED: " + $_.Exception.Message)
    exit 1
}

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$sent = $false

while ($sw.Elapsed.TotalMilliseconds -lt ($Seconds * 1000)) {
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
    if (-not $sent -and $sw.Elapsed.TotalMilliseconds -ge $WarmupMs) {
        try {
            $sp.Write($Send)
            $lines.Add(("{0}`t[SENT]{1}" -f [int]$sw.Elapsed.TotalMilliseconds, $Send))
        } catch {
            $lines.Add(("{0}`t[SEND_FAILED]" -f [int]$sw.Elapsed.TotalMilliseconds))
        }
        $sent = $true
    }
    Start-Sleep -Milliseconds 15
}

$sp.Close()
$lines | Set-Content -Path $Out -Encoding UTF8
Write-Output ("captured {0} lines -> {1}" -f $lines.Count, $Out)
