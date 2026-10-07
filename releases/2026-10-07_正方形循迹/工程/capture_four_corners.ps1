param(
    [string]$PortName = 'COM14',
    [ValidateRange(10, 900)]
    [int]$WaitForStartSeconds = 10,
    [ValidateRange(30, 300)]
    [int]$RunTimeoutSeconds = 70
)

$ErrorActionPreference = 'Stop'
$logDir = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$logPath = Join-Path $logDir ('four_corners_' + (Get-Date -Format 'yyyyMMdd_HHmmss') + '.csv')
$port = [System.IO.Ports.SerialPort]::new(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One
)
$port.ReadTimeout = 500
$port.NewLine = "`n"
$started = $false
$stopped = $false
$cornerStarts = 0
$cornerDone = 0
$parkingPauses = 0
$stopReason = ''
$startDeadline = (Get-Date).AddSeconds($WaitForStartSeconds)
$runDeadline = $null

try {
    $port.Open()
    $port.DiscardInBuffer()
    $writer = [System.IO.StreamWriter]::new(
        $logPath, $false, [System.Text.Encoding]::UTF8
    )
    $writer.AutoFlush = $true
    Write-Output "CAPTURE_READY,$PortName,$logPath"
    $writer.WriteLine('pc_time,uart_line')
    $ready = 0
    $preflightDeadline = (Get-Date).AddSeconds(5)
    while ($ready -lt 3 -and (Get-Date) -lt $preflightDeadline) {
        try { $line = $port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        if (!$line) { continue }
        $writer.WriteLine('"' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') +
                          '","' + $line.Replace('"', '""') + '"')
        if (!$line.StartsWith('D,')) { continue }
        $f = $line.Split(',')
        if ($f.Length -lt 19) { continue }
        $bits = [Convert]::ToInt32($f[3], 16)
        if ($f[2] -eq '0' -and $f[10] -eq '0' -and $f[11] -eq '0' -and
            ($bits -band 0x18) -ne 0 -and [int]$f[4] -lt 100 -and
            [int]$f[17] -lt 150 -and [int]$f[18] -gt 0) {
            $ready++
        } else { $ready = 0 }
    }
    if ($ready -lt 3) { throw 'Preflight failed: motors, center line, gray or IMU not ready.' }
    Write-Output 'PREFLIGHT_OK,motors_stopped,center_line,gray_fresh,imu_fresh'
    $port.Write('G')
    Write-Output 'REMOTE_START_SENT'

    while (!$stopped) {
        if (!$started -and (Get-Date) -gt $startDeadline) {
            throw 'Remote start was not seen before the wait timeout.'
        }
        if ($started -and (Get-Date) -gt $runDeadline) {
            $port.Write('X')
            throw 'Run timed out; X stop command sent.'
        }
        try { $line = $port.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        if (!$line) { continue }

        $writer.WriteLine('"' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss.fff') +
                          '","' + $line.Replace('"', '""') + '"')
        if ($line.StartsWith('EVENT,START,four_corner_garage_line_test')) {
            $started = $true
            $runDeadline = (Get-Date).AddSeconds($RunTimeoutSeconds)
            Write-Output 'RUN_STARTED'
        } elseif ($line.StartsWith('EVENT,CORNER,START,')) {
            $cornerStarts++
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,CORNER,DONE,')) {
            $cornerDone++
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,PARKING_LINE,PAUSE,')) {
            $parkingPauses++
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,PARKING_LINE,')) {
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,STOP,')) {
            $stopped = $true
            $stopReason = $line
            Write-Output $line
        } elseif ($line.StartsWith('EVENT,REFUSE_START,')) {
            throw $line
        }
    }
} finally {
    if ($port.IsOpen -and !$stopped) { try { $port.Write('X') } catch {} }
    if ($null -ne $writer) { $writer.Dispose() }
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}

Write-Output "CAPTURE_DONE,starts=$cornerStarts,done=$cornerDone,parking_pauses=$parkingPauses,$stopReason"
Write-Output "LOG_PATH,$logPath"
if ($stopReason -notlike '*FOUR_CORNERS_COMPLETE*' -or
    $cornerStarts -ne 4 -or $cornerDone -ne 4) {
    throw 'The run did not complete four confirmed corners.'
}
