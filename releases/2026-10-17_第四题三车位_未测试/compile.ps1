param(
  [switch]$RunHostTests,
  [string]$MakePath='F:/cbide/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin/make.exe',
  [string]$ZigPath=''
)
$ErrorActionPreference='Stop'
$repo=(Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
$names=@('4-BC_倒车入库','4-CD_侧方入库','4-DA_倒车入库')
New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot '验证') -Force|Out-Null
foreach($name in $names){
  $dir=Join-Path $PSScriptRoot $name
  $output=& $MakePath -s -C $dir -j4 all 2>&1
  $code=$LASTEXITCODE
  $output|Out-File -LiteralPath (Join-Path $PSScriptRoot "验证/编译_$name.txt") -Encoding utf8
  if($code -ne 0){throw "Build failed: $name"}
  "COMPILED_UNTESTED,$name"
}
if($RunHostTests){
  if(!$ZigPath){$ZigPath=Join-Path $repo '../../tmp/vision_test_toolchain/ziglang/zig.exe'}
  if(!(Test-Path -LiteralPath $ZigPath)){
    $ZigPath='E:/实验室招新题/tmp/vision_test_toolchain/ziglang/zig.exe'
  }
  $env:ZIG_GLOBAL_CACHE_DIR=Join-Path $PSScriptRoot 'tests/build/zig-cache'
  New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot 'tests/build') -Force|Out-Null
  Push-Location $PSScriptRoot
  try{
    $results=@()
    foreach($case in 1..3){
      foreach($test in @('navigation','motion')){
        & $ZigPath cc -std=c11 -Wall -Wextra "-DTASK4_CASE=$case" "tests/test_$test.c" common/entry_plan.c -o "tests/build/test_${test}_${case}.exe"
        if($LASTEXITCODE -ne 0){throw "Host compile failed: $test $case"}
        $result=& "./tests/build/test_${test}_${case}.exe"
        if($LASTEXITCODE -ne 0){throw "Host test failed: $test $case"}
        $results+=$result;$result
      }
    }
    $results|Out-File -LiteralPath '验证/主机测试结果.txt' -Encoding utf8
  }finally{Pop-Location}
}
'COMPILE_ONLY_NO_FLASH_NO_SERIAL_NO_MOTOR_COMMAND'
