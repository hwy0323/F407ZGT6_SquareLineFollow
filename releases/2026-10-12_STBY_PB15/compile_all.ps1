param(
  [string]$MakePath = 'F:/cbide/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin/make.exe'
)
$ErrorActionPreference = 'Stop'
$items = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'versions.json') -Raw | ConvertFrom-Json
$firmwareDir = Join-Path $PSScriptRoot '固件'
$logDir = Join-Path $PSScriptRoot '编译记录'
New-Item -ItemType Directory -Path $firmwareDir,$logDir -Force | Out-Null
$hashLines = @()
foreach ($item in $items) {
  $sourceDir = Join-Path $PSScriptRoot $item.id
  $logPath = Join-Path $logDir ($item.id + '.txt')
  # -B forces a full rebuild; this script never touches a probe or serial port.
  Push-Location -LiteralPath $sourceDir
  try {
    & $MakePath -B -j4 2>&1 | Out-File -LiteralPath $logPath -Encoding utf8
    if ($LASTEXITCODE -ne 0) { throw ('编译失败：' + $item.id + '，查看 ' + $logPath) }
    $target = $item.name + '_STBY_PB15'
    Copy-Item -LiteralPath ('build/' + $target + '.bin'),('build/' + $target + '.elf') -Destination $firmwareDir -Force
    $hash = (Get-FileHash -LiteralPath (Join-Path $firmwareDir ($target + '.bin')) -Algorithm SHA256).Hash
    $hashLines += $hash + '  固件/' + $target + '.bin'
    Write-Output ($item.id + ': OK ' + $hash)
  } finally {
    Pop-Location
  }
}
$hashLines | Out-File -LiteralPath (Join-Path $PSScriptRoot 'SHA256SUMS.txt') -Encoding utf8
Write-Output '所有工程编译完成；未烧录、未启动小车。'
