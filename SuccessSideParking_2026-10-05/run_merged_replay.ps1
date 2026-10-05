param(
    [string]$PortName = 'COM14',
    [ValidateRange(0, 10)]
    [int]$PauseSeconds = 1
)

$ErrorActionPreference = 'Stop'
$steps = @(
    'L030', 'F480', 'L060', 'B340', 'R088',
    'L060', 'F450', 'R088', 'F300', 'R063', 'F100'
)

for ($index = 0; $index -lt $steps.Count; $index++) {
    $command = $steps[$index]
    Write-Output "ROUTE_STEP_START,$($index + 1),$command"
    $result = @(& (Join-Path $PSScriptRoot 'record_step.ps1') -Command $command -PortName $PortName)
    $result | ForEach-Object { Write-Output $_ }

    $endLine = $result | Where-Object { $_ -like 'EVENT,STEP_END,*' } | Select-Object -Last 1
    $idleLine = $result | Where-Object { $_ -like 'FINAL_IDLE,S,*' } | Select-Object -Last 1
    if (!$endLine -or !$idleLine -or !($result -contains 'STEP_END_SEEN,True')) {
        throw "Step $($index + 1) did not end with confirmed idle. Route stopped."
    }
    $reason = $endLine.Split(',')[3]
    if ($reason -notin @('DONE', 'OVERSHOOT', 'DISTANCE_OVERSHOOT')) {
        throw "Step $($index + 1) stopped: $reason. Route stopped."
    }
    Write-Output "ROUTE_STEP_DONE,$($index + 1),$command,$reason"
    if ($index -lt $steps.Count - 1 -and $PauseSeconds -gt 0) {
        Start-Sleep -Seconds $PauseSeconds
    }
}

Write-Output 'ROUTE_COMPLETE,11'
