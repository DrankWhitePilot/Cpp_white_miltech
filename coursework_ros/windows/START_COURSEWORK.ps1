param([switch]$NoBrowser)

$ErrorActionPreference = "Stop"
$dashboardUrl = "http://127.0.0.1:8080/"
$serverScript = Join-Path $PSScriptRoot "DASHBOARD_SERVER.ps1"
. (Join-Path $PSScriptRoot "COURSEWORK_CONFIG.ps1")
$config = Get-CourseworkConfig -Directory $PSScriptRoot
$airSimExe = $config.SceneExe
$project = "$($config.LinuxProject)/coursework_ros"
$distro = $config.Distro
$linuxUser = $config.LinuxUser
$containerName = $config.ContainerName

if (-not (Test-Path -LiteralPath $serverScript)) {
  throw "Не знайдено DASHBOARD_SERVER.ps1 поруч із файлом запуску."
}
if (-not (Test-Path -LiteralPath $airSimExe)) {
  throw "Не знайдено сцену AirSim: $airSimExe"
}

Write-Host "Перевіряю Docker і контейнер курсової..." -ForegroundColor Cyan
$sessionMarker = "$distro -- sleep 14400"
$sessionActive = @(Get-CimInstance Win32_Process -Filter "Name = 'wsl.exe'" |
  Where-Object { $_.CommandLine -like "*$sessionMarker*" }).Count -gt 0
if (-not $sessionActive) {
  Start-Process -FilePath wsl.exe -WindowStyle Hidden -ArgumentList @(
    "-d", $distro, "--", "sleep", "14400") | Out-Null
}
$names = @(& wsl.exe -d $distro -- docker ps --format "{{.Names}}" 2>$null)
if ($LASTEXITCODE -ne 0) {
  throw "Docker не працює. Відкрий Docker Desktop, дочекайся запуску Engine і натисни цей файл ще раз."
}

$stoppedNames = @(& wsl.exe -d $distro -- docker ps -a --filter "status=exited" --format "{{.Names}}" 2>$null)
foreach ($name in $stoppedNames) {
  if ([string]::IsNullOrWhiteSpace($name)) { continue }
  $inspectJson = & wsl.exe -d $distro -- docker inspect $name 2>$null
  if ($LASTEXITCODE -ne 0) { continue }
  $details = @($inspectJson | ConvertFrom-Json)[0]
  $sourceFolder = [string]$details.Config.Labels.'devcontainer.local_folder'
  if ($sourceFolder -match 'Cpp_white_miltech$') {
    Write-Host "Запускаю контейнер курсової: $name" -ForegroundColor Cyan
    & wsl.exe -d $distro -- docker start $name | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "Не вдалося запустити контейнер $name." }
    $names += $name
  }
}

if ($containerName) {
  & wsl.exe -d $distro -- docker inspect $containerName 2>$null | Out-Null
  if ($LASTEXITCODE -ne 0) { throw "Не знайдено контейнер курсової: $containerName" }
  & wsl.exe -d $distro -- docker start $containerName | Out-Null
  $name = $containerName
} else {
  $name = $names | Where-Object {
    & wsl.exe -d $distro -u $linuxUser -- docker exec $_ test -d $project 2>$null
    $LASTEXITCODE -eq 0
  } | Select-Object -First 1
}
if (-not $name) {
  throw "Контейнер C++ курсової не знайдено."
}

& wsl.exe -d $distro -u $linuxUser -- docker exec $name test -f "$($config.LinuxProject)/install/coursework/setup.bash" 2>$null
if ($LASTEXITCODE -ne 0) {
  throw "Курсову ще не зібрано в контейнері. Виконай збірку за README.md і повтори запуск."
}

Write-Host "Відкриваю панель керування..." -ForegroundColor Cyan
$serverOnline = $false
try {
  $response = Invoke-RestMethod -Uri "${dashboardUrl}api/health" -TimeoutSec 2
  $serverOnline = $response.ok -eq $true -and $response.app -eq 'coursework-dashboard'
} catch {}

if (-not $serverOnline) {
  Start-Process powershell.exe -WindowStyle Hidden -ArgumentList @(
    "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$serverScript`"")
  for ($attempt = 0; $attempt -lt 40; ++$attempt) {
    Start-Sleep -Milliseconds 250
    try {
      $response = Invoke-RestMethod -Uri "${dashboardUrl}api/health" -TimeoutSec 1
      if ($response.ok -eq $true -and $response.app -eq 'coursework-dashboard') { $serverOnline = $true; break }
    } catch {}
  }
}
if (-not $serverOnline) {
  throw "Не вдалося відкрити панель на ${dashboardUrl}. Перевір, чи порт 8080 не зайнятий іншою програмою."
}

$cameraFeed = Join-Path $PSScriptRoot 'CAMERA_FEED.ps1'
if (Test-Path -LiteralPath $cameraFeed -PathType Leaf) {
  Start-Process powershell.exe -WindowStyle Hidden -ArgumentList @(
    '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', "`"$cameraFeed`"") | Out-Null
}

if (-not $NoBrowser) { Start-Process $dashboardUrl }
Write-Host "Панель готова: $dashboardUrl" -ForegroundColor Green
Write-Host "1. Вибери криві руху цілей."
Write-Host "2. Вибери боєприпас."
Write-Host "3. Натисни 'ЗАПУСТИТИ СИМУЛЯЦІЮ' у панелі."
Write-Host "Для свого simulation.json вибери файл і натисни 'ВІДТВОРИТИ'."
