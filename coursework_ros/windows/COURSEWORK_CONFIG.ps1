function Get-CourseworkConfig {
  param([Parameter(Mandatory = $true)][string]$Directory)

  $path = Join-Path $Directory "coursework.local.json"
  if (-not (Test-Path -LiteralPath $path)) {
    throw "Missing $path. Copy coursework.local.example.json to coursework.local.json and set the paths."
  }

  try {
    $raw = Get-Content -LiteralPath $path -Raw -Encoding UTF8 | ConvertFrom-Json
  } catch {
    throw "Invalid JSON in $path : $($_.Exception.Message)"
  }

  foreach ($key in @("Distro", "LinuxUser", "LinuxProject", "SceneExe")) {
    if ([string]::IsNullOrWhiteSpace([string]$raw.$key)) {
      throw "Missing $key in $path"
    }
  }
  if ([string]$raw.LinuxProject -notmatch '^/[A-Za-z0-9_./-]+$') {
    throw "LinuxProject must be an absolute Linux path without spaces or shell characters."
  }

  return [pscustomobject]@{
    Distro = [string]$raw.Distro
    LinuxUser = [string]$raw.LinuxUser
    LinuxProject = ([string]$raw.LinuxProject).TrimEnd([char]'/')
    SceneExe = [string]$raw.SceneExe
    AirSimHost = [string]$raw.AirSimHost
  }
}
