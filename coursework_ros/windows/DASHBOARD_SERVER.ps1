param(
  [int]$HttpPort = 8080,
  [int]$UdpPort = 8091
)

$ErrorActionPreference = "Stop"

$httpPort = $HttpPort
$udpPort = $UdpPort
$dashboardFile = Join-Path $PSScriptRoot "dashboard\index.html"
$restartScript = Join-Path $PSScriptRoot "RESTART_DEMO.ps1"
$resultsDirectory = Join-Path $PSScriptRoot "results"
$resultsFile = Join-Path $resultsDirectory "coursework_results.csv"
. (Join-Path $PSScriptRoot "COURSEWORK_CONFIG.ps1")
$config = Get-CourseworkConfig -Directory $PSScriptRoot
$distro = $config.Distro
$linuxUser = $config.LinuxUser
$curveMap = @{
  base_circles = "Базовий тест Кола"
  heavy_ammo = "Важкі боєприпаси"
  lissajous_eight = "Вісімки Ліссажу"
  extreme = "Екстремальний"
  ellipses = "Еліптичні траекторії"
  cardioids_epitrochoids = "Кардіоіди і епітрохоїди"
  flowers = "Квіткові траекторії"
  lissajous_complex = "Лісажу складні криві"
  gliding_ammo = "Планеруючі боєприпаси"
  fast_drone_slow_targets = "Швидкий дрон повільні цілі"
}
$allowedAmmo = @("VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG")

$createdNew = $false
$mutex = [System.Threading.Mutex]::new(
  $true, "Local\CourseworkAirSimDashboard_${httpPort}_${udpPort}", [ref]$createdNew)
if (-not $createdNew) {
  exit 0
}

function Write-Response {
  param(
    [System.Net.HttpListenerContext]$Context,
    [int]$StatusCode,
    [string]$ContentType,
    [string]$Body
  )
  $bytes = [System.Text.Encoding]::UTF8.GetBytes($Body)
  $Context.Response.StatusCode = $StatusCode
  $Context.Response.ContentType = "$ContentType; charset=utf-8"
  $Context.Response.ContentLength64 = $bytes.Length
  $Context.Response.Headers["Cache-Control"] = "no-store"
  $Context.Response.OutputStream.Write($bytes, 0, $bytes.Length)
  $Context.Response.Close()
}

function Stop-CourseworkRos {
  $names = @(& wsl.exe -d $distro -u $linuxUser -- docker ps --format "{{.Names}}")
  foreach ($name in $names) {
    if ([string]::IsNullOrWhiteSpace($name)) {
      continue
    }
    & wsl.exe -d $distro -u $linuxUser -- docker exec $name `
      test -d "$($config.LinuxProject)/coursework_ros" 2>$null
    if ($LASTEXITCODE -ne 0) { continue }
    & wsl.exe -d $distro -u $linuxUser -- docker exec $name `
      pkill -SIGINT -f "[r]os2 launch coursework_guidance_ros airsim_online.launch.xml" `
      2>$null
  }
}

function Format-Metric([object]$Value) {
  if ($null -eq $Value) { return "" }
  return [string]::Format(
    [System.Globalization.CultureInfo]::InvariantCulture, "{0:F3}", [double]$Value)
}

New-Item -ItemType Directory -Path $resultsDirectory -Force | Out-Null
$results = @()
$loggedRunIds = [System.Collections.Generic.HashSet[string]]::new()
if (Test-Path -LiteralPath $resultsFile) {
  try {
    $results = @(Import-Csv -LiteralPath $resultsFile | Sort-Object Timestamp -Descending)
    foreach ($row in $results) {
      if ($row.RunId) { [void]$loggedRunIds.Add([string]$row.RunId) }
    }
  } catch {
    $results = @()
  }
}

$listener = [System.Net.HttpListener]::new()
$listener.Prefixes.Add("http://127.0.0.1:$httpPort/")
$udp = $null

try {
  if (-not (Test-Path -LiteralPath $dashboardFile)) {
    throw "Dashboard file not found: $dashboardFile"
  }

  $udp = [System.Net.Sockets.UdpClient]::new($udpPort)
  $udp.Client.Blocking = $false
  $remoteEndpoint = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
  $latestState = '{"connected":false,"phase":"offline"}'
  $lastTelemetryAt = [DateTime]::MinValue
  $traceRunId = ""
  $traceScenario = ""
  $traceSamples = [System.Collections.Generic.List[object]]::new()
  $lastTraceTime = [double]::NegativeInfinity

  $listener.Start()
  $pendingContext = $listener.GetContextAsync()
  $running = $true

  while ($running) {
    while ($udp.Available -gt 0) {
      try {
        $packet = $udp.Receive([ref]$remoteEndpoint)
        $latestState = [System.Text.Encoding]::UTF8.GetString($packet)
        $lastTelemetryAt = [DateTime]::UtcNow
        try {
          $decoded = $latestState | ConvertFrom-Json
          $runId = [string]$decoded.run_id
          if ($runId -and $runId -ne $traceRunId) {
            $traceRunId = $runId
            $traceScenario = [string]$decoded.scenario
            $traceSamples.Clear()
            $lastTraceTime = [double]::NegativeInfinity
          }
          if ($runId -and $decoded.drone -and
            $decoded.phase -in @("preparing", "guidance") -and
            $null -ne $decoded.sim_time -and
            $null -ne $decoded.drone.x -and $null -ne $decoded.drone.y)
          {
            $sampleTime = [double]$decoded.sim_time
            $sampleX = [double]$decoded.drone.x
            $sampleY = [double]$decoded.drone.y
            if (-not [double]::IsNaN($sampleTime) -and
              -not [double]::IsInfinity($sampleTime) -and
              -not [double]::IsNaN($sampleX) -and
              -not [double]::IsInfinity($sampleX) -and
              -not [double]::IsNaN($sampleY) -and
              -not [double]::IsInfinity($sampleY) -and
              $sampleTime -gt $lastTraceTime)
            {
              $traceSamples.Add([pscustomobject]@{
                t = $sampleTime
                x = $sampleX
                y = $sampleY
              })
              $lastTraceTime = $sampleTime
              if ($traceSamples.Count -gt 5000) { $traceSamples.RemoveAt(0) }
            }
          }
          if ($decoded.phase -eq "complete" -and $decoded.result.ready -and
            $runId -and $loggedRunIds.Add($runId))
          {
            $releaseX = if ($null -ne $decoded.projectile.release_x) {
              Format-Metric $decoded.projectile.release_x
            } else { "" }
            $releaseY = if ($null -ne $decoded.projectile.release_y) {
              Format-Metric $decoded.projectile.release_y
            } else { "" }
            $row = [pscustomobject][ordered]@{
              Timestamp = [DateTime]::Now.ToString("yyyy-MM-dd HH:mm:ss")
              RunId = $runId
              Scenario = [string]$decoded.scenario
              Ammo = [string]$decoded.ammo
              Target = [string]$decoded.result.target
              ReleaseX = $releaseX
              ReleaseY = $releaseY
              ImpactX = Format-Metric $decoded.projectile.impact_x
              ImpactY = Format-Metric $decoded.projectile.impact_y
              ErrorM = Format-Metric $decoded.result.error
              LimitM = Format-Metric $decoded.result.limit
              Result = if ($decoded.result.hit) { "HIT" } else { "MISS" }
            }
            $row | Export-Csv -LiteralPath $resultsFile -NoTypeInformation -Append -Encoding UTF8
            $results = @($row) + @($results) | Select-Object -First 100
          }
        } catch {
          # A malformed telemetry datagram must not stop the dashboard server.
        }
      } catch [System.Net.Sockets.SocketException] {
        break
      }
    }

    if ($pendingContext.IsCompleted) {
      $context = $pendingContext.GetAwaiter().GetResult()
      $pendingContext = $listener.GetContextAsync()
      $path = $context.Request.Url.AbsolutePath
      $method = $context.Request.HttpMethod

      if ($method -eq "GET" -and $path -eq "/") {
        Write-Response $context 200 "text/html" (
          [System.IO.File]::ReadAllText($dashboardFile, [System.Text.Encoding]::UTF8))
      } elseif ($method -eq "GET" -and $path -eq "/api/state") {
        if (([DateTime]::UtcNow - $lastTelemetryAt).TotalSeconds -gt 2.0) {
          Write-Response $context 200 "application/json" '{"connected":false,"phase":"offline"}'
        } else {
          Write-Response $context 200 "application/json" $latestState
        }
      } elseif ($method -eq "GET" -and $path -eq "/api/health") {
        Write-Response $context 200 "application/json" '{"ok":true}'
      } elseif ($method -eq "GET" -and $path -eq "/api/results") {
        $recentResults = @($results | Select-Object -First 25)
        Write-Response $context 200 "application/json" (
          ConvertTo-Json -InputObject $recentResults -Compress)
      } elseif ($method -eq "GET" -and $path -eq "/api/trace") {
        Write-Response $context 200 "application/json" (
          ConvertTo-Json -InputObject ([pscustomobject]@{
            schema = 1
            run_id = $traceRunId
            scenario = $traceScenario
            samples = @($traceSamples.ToArray())
          }) -Depth 5 -Compress)
      } elseif ($method -eq "POST" -and $path -eq "/api/restart") {
        $curveId = [string]$context.Request.QueryString["curve_id"]
        $ammo = [string]$context.Request.QueryString["ammo"]
        if (-not $curveId) { $curveId = "base_circles" }
        if (-not $ammo) { $ammo = "VOG-17" }
        if (-not $curveMap.ContainsKey($curveId)) {
          Write-Response $context 400 "application/json" '{"error":"unknown target curves"}'
          continue
        }
        if ($ammo -notin $allowedAmmo) {
          Write-Response $context 400 "application/json" '{"error":"unknown ammunition"}'
          continue
        }
        $curves = $curveMap[$curveId]
        Write-Response $context 202 "application/json" '{"accepted":true}'
        Start-Process powershell.exe -WindowStyle Hidden -ArgumentList @(
          "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$restartScript`"",
          "-NoBrowser", "-Curves", "`"$curves`"", "-Ammo", "`"$ammo`"")
      } elseif ($method -eq "POST" -and $path -eq "/api/stop") {
        Stop-CourseworkRos
        Write-Response $context 200 "application/json" '{"stopped":true}'
      } elseif ($method -eq "POST" -and $path -eq "/api/close") {
        Stop-CourseworkRos
        Get-Process -Name "MSBuild2018" -ErrorAction SilentlyContinue | Stop-Process -Force
        Write-Response $context 200 "application/json" '{"closed":true}'
        $running = $false
      } else {
        Write-Response $context 404 "application/json" '{"error":"not found"}'
      }
    }

    Start-Sleep -Milliseconds 20
  }
} finally {
  if ($listener.IsListening) {
    $listener.Stop()
  }
  $listener.Close()
  if ($udp) {
    $udp.Dispose()
  }
  $mutex.ReleaseMutex()
  $mutex.Dispose()
}
