param(
  [string]$MakePath='F:/cbide/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin/make.exe',
  [string]$ZigPath='E:/实验室招新题/tmp/vision_test_toolchain/ziglang/zig.exe'
)
$ErrorActionPreference='Stop'
New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot 'tests/build'),(Join-Path $PSScriptRoot '验证'),(Join-Path $PSScriptRoot '固件') -Force|Out-Null
if(-not $env:ZIG_GLOBAL_CACHE_DIR){$env:ZIG_GLOBAL_CACHE_DIR=Join-Path $PSScriptRoot 'tests/build/zig-cache'}
Push-Location $PSScriptRoot
try{
  & $ZigPath cc -std=c11 -Wall -Wextra -Werror tests/test_menu.c menu_app.c keys.c menu_command.c completion.c mv_client.c ../2026-10-17_发挥一六分支视觉联动/task4_plan.c -o tests/build/test_menu.exe
  if($LASTEXITCODE -ne 0){throw 'Menu host compile failed'}
  $result=& ./tests/build/test_menu.exe
  if($LASTEXITCODE -ne 0){throw 'Menu host tests failed'}
  $results=@($result);$result
  & $ZigPath cc -std=c11 -Wall -Wextra -Werror tests/test_ui.c menu_ui.c -o tests/build/test_ui.exe
  if($LASTEXITCODE -ne 0){throw 'OLED UI host compile failed'}
  $result=& ./tests/build/test_ui.exe
  if($LASTEXITCODE -ne 0){throw 'OLED UI host tests failed'}
  $results+=$result;$result
  # 回归实际复用的视觉、运动、导航、激光分类和云台代码。每项独立链接，动作表互不串用。
  $task4='../2026-10-17_发挥一六分支视觉联动'
  foreach($test in @('navigation','motion','app','gimbal_pwm')){
    $argsList=@('cc','-std=c11','-Wall','-Wextra','-Werror',($task4+'/tests/test_'+$test+'.c'))
    if($test -eq 'gimbal_pwm'){$argsList+=@('-DGIMBAL_PWM_HOST_TEST',($task4+'/gimbal_pose.c'))}
    else{$argsList+=($task4+'/task4_plan.c')}
    if($test -eq 'app'){$argsList+=@(($task4+'/mv_client.c'),($task4+'/start_command.c'))}
    $exe='tests/build/baseline4_'+$test+'.exe';$argsList+=@('-o',$exe)
    & $ZigPath @argsList
    if($LASTEXITCODE -ne 0){throw ('Baseline compile failed: '+$test)}
    $result=& ('./'+$exe)
    if($LASTEXITCODE -ne 0){throw ('Baseline failed: '+$test)}
    $results+=$result;$result
  }
  $laser='../2026-10-17_发挥二_激光搜索入库_未测试'
  foreach($test in @('logic','app')){
    $exe='tests/build/baseline5_'+$test+'.exe'
    & $ZigPath cc -std=c11 -Wall -Wextra -Werror ($laser+'/tests/test_'+$test+'.c') ($laser+'/search_logic.c') ($laser+'/entry_plan.c') -o $exe
    if($LASTEXITCODE -ne 0){throw ('Laser compile failed: '+$test)}
    $result=& ('./'+$exe)
    if($LASTEXITCODE -ne 0){throw ('Laser baseline failed: '+$test)}
    $results+=$result;$result
  }
  $results|Out-File -LiteralPath '验证/主机测试.txt' -Encoding utf8 -Width 4096
  $ErrorActionPreference='Continue'
  $build=& $MakePath -B -s -j4 all 2>&1;$buildExit=$LASTEXITCODE
  $ErrorActionPreference='Stop'
  $build|ForEach-Object {$_.ToString()}|Out-File -LiteralPath '验证/ARM编译.txt' -Encoding utf8 -Width 4096
  if($buildExit -ne 0){throw 'ARM build failed; see 验证/ARM编译.txt'}
  $target='F407ZGT6_AllTasks_OLED_Menu_STBY_PB15'
  foreach($ext in @('bin','elf')){Copy-Item -LiteralPath ('build/'+$target+'.'+$ext) -Destination ('固件/'+$target+'.'+$ext)}
  Get-FileHash -Algorithm SHA256 -LiteralPath ('固件/'+$target+'.bin'),('固件/'+$target+'.elf')|
    ForEach-Object {$_.Hash+'  固件/'+[IO.Path]::GetFileName($_.Path)}|Out-File -LiteralPath 'SHA256SUMS.txt' -Encoding utf8
  'PASS BUILD_ONLY; NO_PROBE; NO_FLASH; NO_SERIAL; NO_MOTION'
}finally{Pop-Location}
