param(
 [string]$MakePath='F:/cbide/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin/make.exe',
 [string]$ZigPath='E:/实验室招新题/tmp/vision_test_toolchain/ziglang/zig.exe'
)
$ErrorActionPreference='Stop'
New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot 'tests/build'),(Join-Path $PSScriptRoot '验证'),(Join-Path $PSScriptRoot '固件') -Force | Out-Null
if(-not $env:ZIG_GLOBAL_CACHE_DIR){$env:ZIG_GLOBAL_CACHE_DIR=Join-Path $PSScriptRoot 'tests/build/zig-cache'}
Push-Location $PSScriptRoot
try{
 $testOutput=@()
 foreach($testName in @('navigation','motion','app','gimbal_pwm')){
  $argsList=@('cc','-std=c11','-Wall','-Wextra','-Werror',('tests/test_'+$testName+'.c'))
  if($testName -eq 'gimbal_pwm'){$argsList+=@('-DGIMBAL_PWM_HOST_TEST','gimbal_pose.c')}
  else{$argsList+='task4_plan.c'}
  if($testName -eq 'app'){$argsList+=@('mv_client.c','start_command.c')}
  $testExe='tests/build/test_'+$testName+'.exe'
  $argsList+=@('-o',$testExe)
  & $ZigPath @argsList
  if($LASTEXITCODE -ne 0){throw ('Host compile failed: '+$testName)}
  $result=& ('./'+$testExe)
  if($LASTEXITCODE -ne 0){throw ('Host test failed: '+$testName)}
  $testOutput+=$result;$result
 }
 $testOutput | Out-File -LiteralPath '验证/主机测试.txt' -Encoding utf8 -Width 4096
 $ErrorActionPreference='Continue'
 $build=& $MakePath -B -s -j4 all 2>&1
 $buildExit=$LASTEXITCODE;$ErrorActionPreference='Stop'
 $build | ForEach-Object {$_.ToString()} | Out-File -LiteralPath '验证/ARM编译.txt' -Encoding utf8 -Width 4096
 if($buildExit -ne 0){throw 'ARM build failed'}
 $target='F407ZGT6_Task4_SixBranch_Vision_STBY_PB15'
 foreach($ext in @('bin','elf')){Copy-Item -LiteralPath ('build/'+$target+'.'+$ext) -Destination ('固件/'+$target+'.'+$ext)}
 Get-FileHash -Algorithm SHA256 -LiteralPath ('固件/'+$target+'.bin'),('固件/'+$target+'.elf') | ForEach-Object {$_.Hash+'  固件/'+[IO.Path]::GetFileName($_.Path)} | Out-File -LiteralPath 'SHA256SUMS.txt' -Encoding utf8 -Width 4096
 'PASS BUILD_ONLY; NO_PROBE; NO_FLASH; NO_SERIAL_ACCESS; NO_MOTION'
}finally{Pop-Location}
