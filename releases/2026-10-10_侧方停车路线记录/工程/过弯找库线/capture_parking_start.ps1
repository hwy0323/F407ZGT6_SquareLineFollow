param(
    [string]$PortName = 'COM14',
    [ValidateRange(8, 30)]
    [int]$RunTimeoutSeconds = 18
)

$ErrorActionPreference = 'Stop'
$logDir = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$logPath = Join-Path $logDir ('parking_start_' + (Get-Date -Format 'yyyyMMdd_HHmmss') + '.csv')
$port = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$port.ReadTimeout = 300
$port.NewLine = "`n"
$writer = $null
$started = $false
$cornerDone = $false
$parkingStart = $false
$stopLine = ''

try {
    $port.Open()
    $port.DiscardInBuffer()
    $writer = [System.IO.StreamWriter]::new(
        $logPath, $false, [System.Text.Encoding]::UTF8
    )
    $writer.AutoFlush = $true
    $writer.WriteLine('pc_time,uart_line')

    # Observe several stopped reports before sending any motion command.
    $preflightDeadline = (Get-Date).AddSeconds(3)
    $safeReports = 0
    while ((Get-Date) -lt $preflightDeadline -and $safeReports -lt 3) {
        try { $line = $port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        if (!$line) { continue }
        $writer.WriteLine('"' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') +
                          '","' + $line.Replace('"', '""') + '"')
        if (!$line.StartsWith('D,')) { continue }
        $fields = $line.Split(',')
        if ($fields.Length -lt 19) { throw "Invalid status line: $line" }
        $bits = [Convert]::ToInt32($fields[3], 16)
        $lit = 0
        for ($i = 0; $i -lt 8; $i++) { if ($bits -band (1 -shl $i)) { $lit++ } }
        if ([int]$fields[2] -ne 0 -or
            ($bits -band 0x18) -eq 0 -or $lit -gt 3 -or
            [int]$fields[4] -gt 200 -or
            [int]$fields[10] -ne 0 -or [int]$fields[11] -ne 0 -or
            [int]$fields[17] -gt 300 -or [int]$fields[18] -eq 0) {
            throw "Preflight failed: $line"
        }
        $safeReports++
    }
    if ($safeReports -lt 3) { throw 'No three fresh stopped reports received.' }
    Write-Output "CAPTURE_READY,$PortName,$logPath,gray_centered,motors_stopped"

    $port.Write('G')
    Write-Output 'START_COMMAND_SENT,G'
    $deadline = (Get-Date).AddSeconds($RunTimeoutSeconds)
    while ((Get-Date) -lt $deadline -and !$stopLine) {
        try { $line = $port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        if (!$line) { continue }
        $writer.WriteLine('"' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') +
                          '","' + $line.Replace('"', '""') + '"')
        if ($line.StartsWith('EVENT,START,')) {
            $started = $true
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,CORNER,DONE,1,')) {
            $cornerDone = $true
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,PARKING_START,')) {
            $parkingStart = $true
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,STOP,')) {
            $stopLine = $line
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,REFUSE_START,')) {
            throw "Start refused: $line"
        }
    }
    if (!$stopLine) {
        $port.Write('X')
        throw 'Run timeout; X stop command sent. Cut motor power if the car does not stop.'
    }
} finally {
    if ($null -ne $writer) { $writer.Dispose() }
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}

Write-Output "LOG_PATH,$logPath"
if (!$started -or !$cornerDone -or !$parkingStart -or
    $stopLine -notlike '*PARKING_START_READY*') {
    throw 'The car stopped, but the one-corner-to-parking-start goal was not confirmed.'
}
Write-Output 'PARKING_START_CONFIRMED'
