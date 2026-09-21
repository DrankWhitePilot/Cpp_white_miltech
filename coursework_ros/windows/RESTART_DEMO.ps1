param(
  [switch]$NoBrowser,
  [Alias("Scenario")]
  [string]$Curves = "Базовий тест Кола",
  [string]$Ammo = "VOG-17"
)

$ErrorActionPreference = "Stop"

. (Join-Path $PSScriptRoot "COURSEWORK_CONFIG.ps1")
$config = Get-CourseworkConfig -Directory $PSScriptRoot
$distro = $config.Distro
$linuxUser = $config.LinuxUser
$project = $config.LinuxProject
$airSimExe = $config.SceneExe
$dashboardUrl = "http://127.0.0.1:8080/"
$dashboardServer = Join-Path $PSScriptRoot "DASHBOARD_SERVER.ps1"
$textureDirectory = Join-Path $PSScriptRoot "assets\target_textures"
$selectedCurvesFile = Join-Path $PSScriptRoot "current_curves.txt"
$selectedAmmoFile = Join-Path $PSScriptRoot "current_ammo.txt"
$allowedCurves = @(
  "Базовий тест Кола",
  "Важкі боєприпаси",
  "Вісімки Ліссажу",
  "Екстремальний",
  "Еліптичні траекторії",
  "Кардіоіди і епітрохоїди",
  "Квіткові траекторії",
  "Лісажу складні криві",
  "Планеруючі боєприпаси",
  "Швидкий дрон повільні цілі"
)
$allowedAmmo = @("VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG")

if ($Curves -notin $allowedCurves) {
  throw "Unknown target curves: $Curves"
}
if ($Ammo -notin $allowedAmmo) {
  throw "Unknown ammunition: $Ammo"
}
[System.IO.File]::WriteAllText($selectedCurvesFile, $Curves, [System.Text.Encoding]::UTF8)
[System.IO.File]::WriteAllText($selectedAmmoFile, $Ammo, [System.Text.Encoding]::UTF8)

Write-Host "Coursework AirSim demo restart" -ForegroundColor Cyan
Write-Host "Target curves: $Curves" -ForegroundColor Cyan
Write-Host "Ammunition: $Ammo" -ForegroundColor Cyan
Write-Host "Finding the coursework container..."

$dashboardOnline = $false
try {
  $health = Invoke-WebRequest -UseBasicParsing -Uri "${dashboardUrl}api/health" -TimeoutSec 1
  $dashboardOnline = $health.StatusCode -eq 200
} catch {
  $dashboardOnline = $false
}
if (-not $dashboardOnline -and (Test-Path -LiteralPath $dashboardServer)) {
  Start-Process powershell.exe -WindowStyle Hidden -ArgumentList @(
    "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$dashboardServer`"")
  for ($attempt = 0; $attempt -lt 30; ++$attempt) {
    Start-Sleep -Milliseconds 100
    try {
      $health = Invoke-WebRequest -UseBasicParsing -Uri "${dashboardUrl}api/health" -TimeoutSec 1
      if ($health.StatusCode -eq 200) { break }
    } catch {}
  }
}

$container = $null
$containerNames = @(& wsl.exe -d $distro -u $linuxUser -- docker ps --format "{{.Names}}")
foreach ($name in $containerNames) {
  if ([string]::IsNullOrWhiteSpace($name)) {
    continue
  }
  & wsl.exe -d $distro -u $linuxUser -- docker exec $name test -d "$project/coursework_ros" 2>$null
  if ($LASTEXITCODE -eq 0) {
    $container = $name.Trim()
    break
  }
}

if (-not $container) {
  throw "The running C++ coursework container was not found. Start the dev container and try again."
}

$airSimHost = $config.AirSimHost
if (-not $airSimHost) {
  $route = & wsl.exe -d $distro -u $linuxUser -- sh -lc "ip -4 route show default"
  $routeText = @($route) -join " "
  if ($routeText -match 'default via (\d{1,3}(?:\.\d{1,3}){3})') {
    $airSimHost = $Matches[1]
  }
}
if ($airSimHost -notmatch '^\d{1,3}(?:\.\d{1,3}){3}$') {
  throw "Could not determine Windows host IP. Set AirSimHost in coursework.local.json."
}
if (-not (Test-Path -LiteralPath (Join-Path $textureDirectory "target_1.png"))) {
  throw "Target texture assets are missing: $textureDirectory"
}
$textureDirectoryForAirSim = $textureDirectory.Replace('\', '/')

Write-Host "Stopping the previous run..."
& wsl.exe -d $distro -u $linuxUser -- docker exec $container `
  pkill -SIGINT -f "[r]os2 launch coursework_guidance_ros airsim_online.launch.xml" 2>$null

for ($attempt = 0; $attempt -lt 20; ++$attempt) {
  & wsl.exe -d $distro -u $linuxUser -- docker exec $container `
    pgrep -f "[a]irsim_online_node" 2>$null | Out-Null
  if ($LASTEXITCODE -ne 0) {
    break
  }
  Start-Sleep -Milliseconds 250
}

if (-not (Get-Process -Name "MSBuild2018" -ErrorAction SilentlyContinue)) {
  if (-not (Test-Path -LiteralPath $airSimExe)) {
    throw "AirSim scene was not found: $airSimExe"
  }
  Write-Host "Starting AirSim..."
  Start-Process -FilePath $airSimExe `
    -WorkingDirectory (Split-Path -Parent $airSimExe) `
    -ArgumentList @("-windowed", "-ResX=1440", "-ResY=810")
}

$timestamp = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
$remoteLog = "/tmp/coursework_airsim_restart_$timestamp.log"
$launchCommand = "cd $project && source /opt/ros/jazzy/setup.bash && source install/coursework/setup.bash && ros2 launch coursework_guidance_ros airsim_online.launch.xml scenario:='$Curves' ammo:='$Ammo' host:='$airSimHost' dashboard_host:='$airSimHost' target_texture_directory:='$textureDirectoryForAirSim' > $remoteLog 2>&1"

& wsl.exe -d $distro -u $linuxUser -- docker exec $container touch $remoteLog
if ($LASTEXITCODE -ne 0) {
  throw "Could not prepare the simulation log."
}

Write-Host "Starting a fresh simulation..."
& wsl.exe -d $distro -u $linuxUser -- docker exec -d $container bash -lc $launchCommand
if ($LASTEXITCODE -ne 0) {
  throw "Could not start the ROS AirSim module."
}

for ($attempt = 0; $attempt -lt 60; ++$attempt) {
  & wsl.exe -d $distro -u $linuxUser -- docker exec $container `
    grep -q "ONLINE START" $remoteLog 2>$null
  if ($LASTEXITCODE -eq 0) {
    Write-Host "DEMO STARTED. Watch the AirSim window." -ForegroundColor Green
    Write-Host "J restarts the selected curves and ammunition. Esc closes AirSim and stops ROS."
    $hotkeyScript = Join-Path $PSScriptRoot "AIRSIM_HOTKEYS.ps1"
    if (Test-Path -LiteralPath $hotkeyScript) {
      Start-Process powershell.exe -WindowStyle Hidden -ArgumentList @(
        "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$hotkeyScript`"")
    }
    Add-Type -AssemblyName Microsoft.VisualBasic
    $airSimWindow = Get-Process -Name "MSBuild2018" -ErrorAction SilentlyContinue |
      Where-Object { $_.MainWindowHandle -ne 0 } |
      Select-Object -First 1
    if ($airSimWindow) {
      [Microsoft.VisualBasic.Interaction]::AppActivate($airSimWindow.Id) | Out-Null
    }
    if (-not $NoBrowser) {
      Start-Process $dashboardUrl
    }
    exit 0
  }

  & wsl.exe -d $distro -u $linuxUser -- docker exec $container `
    pgrep -f "[a]irsim_online_node" 2>$null | Out-Null
  if ($LASTEXITCODE -ne 0 -and $attempt -gt 2) {
    Write-Host "The module stopped. Diagnostic output:" -ForegroundColor Red
    & wsl.exe -d $distro -u $linuxUser -- docker exec $container tail -n 30 $remoteLog
    throw "The simulation did not start."
  }
  Start-Sleep -Seconds 1
}

throw "AirSim did not become ready within 60 seconds."
