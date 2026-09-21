$ErrorActionPreference = "SilentlyContinue"

. (Join-Path $PSScriptRoot "COURSEWORK_CONFIG.ps1")
$config = Get-CourseworkConfig -Directory $PSScriptRoot
$distro = $config.Distro
$linuxUser = $config.LinuxUser
$restartScript = Join-Path $PSScriptRoot "RESTART_DEMO.ps1"
$selectedCurvesFile = Join-Path $PSScriptRoot "current_curves.txt"
$selectedAmmoFile = Join-Path $PSScriptRoot "current_ammo.txt"

$createdNew = $false
$mutex = [System.Threading.Mutex]::new(
  $true, "Local\CourseworkAirSimHotkeys", [ref]$createdNew)
if (-not $createdNew) {
  exit 0
}

Add-Type @"
using System;
using System.Runtime.InteropServices;

public static class CourseworkAirSimKeyboard
{
    [DllImport("user32.dll")]
    public static extern short GetAsyncKeyState(int virtualKey);

    [DllImport("user32.dll")]
    public static extern IntPtr GetForegroundWindow();

    [DllImport("user32.dll")]
    public static extern uint GetWindowThreadProcessId(IntPtr window, out uint processId);
}
"@

function Test-AirSimForeground {
  $window = [CourseworkAirSimKeyboard]::GetForegroundWindow()
  if ($window -eq [IntPtr]::Zero) {
    return $false
  }

  [uint32]$processId = 0
  [CourseworkAirSimKeyboard]::GetWindowThreadProcessId($window, [ref]$processId) | Out-Null
  $process = Get-Process -Id $processId -ErrorAction SilentlyContinue
  return $process -and $process.ProcessName -eq "MSBuild2018"
}

function Test-KeyDown([int]$virtualKey) {
  return ([CourseworkAirSimKeyboard]::GetAsyncKeyState($virtualKey) -band 0x8000) -ne 0
}

$escapeWasDown = $false
$restartWasDown = $false

try {
  while (Get-Process -Name "MSBuild2018" -ErrorAction SilentlyContinue) {
    $airSimActive = Test-AirSimForeground
    $escapeDown = Test-KeyDown 0x1B
    $restartDown = Test-KeyDown 0x4A

    if ($airSimActive -and $escapeDown -and -not $escapeWasDown) {
      $containerNames = @(& wsl.exe -d $distro -u $linuxUser -- docker ps --format "{{.Names}}")
      foreach ($name in $containerNames) {
        & wsl.exe -d $distro -u $linuxUser -- docker exec $name `
          test -d "$($config.LinuxProject)/coursework_ros" 2>$null
        if ($LASTEXITCODE -ne 0) { continue }
        & wsl.exe -d $distro -u $linuxUser -- docker exec $name `
          pkill -SIGINT -f "[r]os2 launch coursework_guidance_ros airsim_online.launch.xml" `
          2>$null
      }
      Get-Process -Name "MSBuild2018" -ErrorAction SilentlyContinue | Stop-Process -Force
      break
    }

    if ($airSimActive -and $restartDown -and -not $restartWasDown) {
      $curves = "Базовий тест Кола"
      $ammo = "VOG-17"
      if (Test-Path -LiteralPath $selectedCurvesFile) {
        $savedCurves = [System.IO.File]::ReadAllText($selectedCurvesFile).Trim()
        if ($savedCurves) { $curves = $savedCurves }
      }
      if (Test-Path -LiteralPath $selectedAmmoFile) {
        $savedAmmo = [System.IO.File]::ReadAllText($selectedAmmoFile).Trim()
        if ($savedAmmo) { $ammo = $savedAmmo }
      }
      Start-Process powershell.exe -WindowStyle Hidden -ArgumentList @(
        "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$restartScript`"",
        "-NoBrowser", "-Curves", "`"$curves`"", "-Ammo", "`"$ammo`"")
    }

    $escapeWasDown = $escapeDown
    $restartWasDown = $restartDown
    Start-Sleep -Milliseconds 60
  }
} finally {
  $mutex.ReleaseMutex()
  $mutex.Dispose()
}
