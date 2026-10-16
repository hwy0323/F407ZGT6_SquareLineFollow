param(
  [string]$MakePath='F:/cbide/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin/make.exe',
  [string]$ZigPath='E:/实验室招新题/tmp/vision_test_toolchain/ziglang/zig.exe'
)
$ErrorActionPreference='Stop'
New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot '验证'),(Join-Path $PSScriptRoot '固件'),(Join-Path $PSScriptRoot 'tests/build') -Force | Out-Null
Push-Location $PSScriptRoot
try {
  $env:ZIG_GLOBAL_CACHE_DIR=Join-Path $PSScriptRoot 'tests/build/zig-cache'
  $results=@()
  foreach($test in @('logic','app')) {
    & $ZigPath cc -std=c11 -Wall -Wextra -Werror "tests/test_$test.c" search_logic.c entry_plan.c -o "tests/build/test_$test.exe"
    if($LASTEXITCODE -ne 0){throw "Host compile failed: $test"}
    $result=& "./tests/build/test_$test.exe"
    if($LASTEXITCODE -ne 0){throw "Host test failed: $test"}
    $results+=$result;$result
  }
  $results | Out-File -LiteralPath '验证/主机测试结果.txt' -Encoding utf8
  # -B rebuilds everything; does not delete or change other project builds.
  # Windows PowerShell 5 treats native stderr as ErrorRecord even for warnings.
  $ErrorActionPreference='Continue'
  $output=& $MakePath -B -s -j4 all 2>&1
  $code=$LASTEXITCODE
  $ErrorActionPreference='Stop'
  $output | ForEach-Object {$_.ToString()} | Out-File -LiteralPath '验证/ARM完整编译.txt' -Encoding utf8 -Width 4096
  if($code -ne 0){throw 'ARM build failed'}
  foreach($ext in @('bin','elf')) {
    Copy-Item -LiteralPath "build/F407ZGT6_Ext2_LaserSearch_UNTESTED_STBY_PB15.$ext" -Destination "固件/F407ZGT6_Ext2_LaserSearch_UNTESTED_STBY_PB15.$ext"
  }
  Get-FileHash -Algorithm SHA256 -LiteralPath '固件/F407ZGT6_Ext2_LaserSearch_UNTESTED_STBY_PB15.bin','固件/F407ZGT6_Ext2_LaserSearch_UNTESTED_STBY_PB15.elf' |
    ForEach-Object {"$($_.Hash)  $([IO.Path]::GetFileName($_.Path))"} |
    Out-File -LiteralPath '验证/SHA256.txt' -Encoding utf8
  'PASS ARM_COMPILE_ONLY; NO_FLASH; NO_SERIAL; NO_MOTOR_COMMAND'
} finally {Pop-Location}
