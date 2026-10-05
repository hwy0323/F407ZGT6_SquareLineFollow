param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[LRFB][0-9]{3}$')]
    [string]$Command,
    [string]$PortName = 'COM14'
)

$ErrorActionPreference = 'Stop'
$logDir = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$logPath = Join-Path $logDir ('step_' + (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + $Command + '.csv')
$port = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$port.NewLine = "`n"
$port.ReadTimeout = 250
$writer = $null

try {
    $port.Open()
    $port.DiscardInBuffer()
    $port.Write("Q`n")
    $ack = $false
    $readyRows = @()
    $deadline = (Get-Date).AddSeconds(4)
    while ((Get-Date) -lt $deadline -and (!$ack -or $readyRows.Count -lt 4)) {
        try { $line = $port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        if ($line.StartsWith('ACK,Q')) { $ack = $true }
        if ($line.StartsWith('S,')) { $readyRows += $line }
    }
    if (!$ack -or $readyRows.Count -lt 4) {
        throw 'UART precheck failed; no movement command sent.'
    }
    foreach ($row in $readyRows) {
        $fields = $row.Split(',')
        if ($fields.Length -lt 18 -or $fields[2] -ne '0' -or
            $fields[10] -ne '0' -or $fields[11] -ne '0' -or
            [int]$fields[16] -ge 150 -or [int]$fields[17] -eq 0) {
            throw "Idle or IMU precheck failed; no movement command sent: $row"
        }
    }
    Write-Output "PRECHECK_OK,$($readyRows[-1])"
    $writer = [System.IO.StreamWriter]::new($logPath, $false, [System.Text.Encoding]::UTF8)
    $writer.WriteLine('pc_time,uart_line')
    $writer.WriteLine('"' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') + '","' + $readyRows[-1] + '"')
    Write-Output "LOG_PATH,$logPath"
    $port.Write("$Command`n")
    Write-Output "COMMAND_SENT,$Command"

    $deadline = (Get-Date).AddSeconds(12)
    $stepEnd = ''
    while ((Get-Date) -lt $deadline) {
        try { $line = $port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        if (!$line) { continue }
        $writer.WriteLine('"' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') + '","' + $line.Replace('"', '""') + '"')
        if ($line.StartsWith('EVENT,')) { Write-Output $line }
        if ($line.StartsWith('EVENT,STEP_END')) { $stepEnd = $line; break }
    }
    if (!$stepEnd) {
        $port.Write('X')
        Write-Output 'TIMEOUT_SENT_X'
    }
    $idleRow = ''
    $deadline = (Get-Date).AddSeconds(2)
    while ((Get-Date) -lt $deadline) {
        try { $line = $port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        if (!$line) { continue }
        $writer.WriteLine('"' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') + '","' + $line.Replace('"', '""') + '"')
        if ($line.StartsWith('EVENT,STEP_END')) {
            $stepEnd = $line
            Write-Output $line
        }
        if ($line.StartsWith('S,')) {
            $fields = $line.Split(',')
            if ($fields.Length -ge 18 -and $fields[2] -eq '0' -and
                $fields[10] -eq '0' -and $fields[11] -eq '0') {
                $idleRow = $line
                break
            }
        }
    }
    Write-Output "FINAL_IDLE,$idleRow"
    Write-Output "STEP_END_SEEN,$([bool]$stepEnd)"
} finally {
    if ($null -ne $writer) { $writer.Dispose() }
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
