param(
  [string]$IdeRoot = 'D:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE',
  [switch]$Flash,
  [switch]$FlashOnly,
  [ValidateSet('cmsis-dap','stlink')][string]$Probe = 'stlink',
  [string]$ProbeSerial = ''
)
$ErrorActionPreference = 'Stop'
$plugins = Join-Path $IdeRoot 'plugins'
function Find-Plugin([string]$Pattern) {
  $found = Get-ChildItem -LiteralPath $plugins -Directory | Where-Object Name -Like $Pattern | Sort-Object Name -Descending | Select-Object -First 1
  if (!$found) { throw "Missing CubeIDE plugin: $Pattern. Check -IdeRoot." }
  return $found.FullName
}
Push-Location $PSScriptRoot
try {
  if (!$FlashOnly) {
    $make = Join-Path (Find-Plugin 'com.st.stm32cube.ide.mcu.externaltools.make.win32_*') 'tools\bin\make.exe'
    $gccBin = (Join-Path (Find-Plugin 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*.win32_*') 'tools\bin').Replace('\','/')
    & $make -j2 "TOOL_ROOT=$gccBin"
    if ($LASTEXITCODE -ne 0) { throw 'Build failed.' }
  }
  if (!$Flash -and !$FlashOnly) { return }
  $openocd = Join-Path (Find-Plugin 'com.st.stm32cube.ide.mcu.externaltools.openocd.win32_*') 'tools\bin\openocd.exe'
  $scripts = Join-Path (Find-Plugin 'com.st.stm32cube.ide.mcu.debug.openocd_*') 'resources\openocd\st_scripts'
  $elf = (Join-Path $PSScriptRoot 'build/F407ZGT6_OLED_Menu.elf').Replace('\','/')
  $interfaceFile = if ($Probe -eq 'stlink') { 'interface/stlink-dap.cfg' } else { 'interface/cmsis-dap.cfg' }
  $openArgs = @('-s', $scripts, '-f', $interfaceFile)
  if ($ProbeSerial) { $openArgs += @('-c', "adapter serial $ProbeSerial") }
  $transport = if ($Probe -eq 'stlink') { 'dapdirect_swd' } else { 'swd' }
  $openArgs += @('-c',"transport select $transport",'-f','target/stm32f4x.cfg','-c','adapter speed 100',
    '-c','gdb_port disabled','-c','tcl_port disabled','-c','telnet_port disabled',
    '-c',"program {$elf} verify reset exit")
  & $openocd @openArgs
  if ($LASTEXITCODE -ne 0) { throw 'Programming or readback verification failed.' }
  Write-Output 'FLASH_VERIFIED_RESET_IDLE_NO_MOTION_COMMAND_SENT'
} finally { Pop-Location }
