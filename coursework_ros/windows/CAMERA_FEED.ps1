$ErrorActionPreference = 'Stop'

$identityBytes = [Text.Encoding]::UTF8.GetBytes(
  ([IO.Path]::GetFullPath($PSScriptRoot)).ToLowerInvariant())
$sha256 = [Security.Cryptography.SHA256]::Create()
try {
  $identity = ([BitConverter]::ToString($sha256.ComputeHash($identityBytes))).Replace('-', '').Substring(0, 16)
} finally {
  $sha256.Dispose()
}
$newFeed = $false
$mutex = [Threading.Mutex]::new($true, "Local\CourseworkAirSimCameraFeed_$identity", [ref]$newFeed)
if (-not $newFeed) {
  $mutex.Dispose()
  exit 0
}

Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;

public static class NativeAirSimWindow {
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT {
        public int Left;
        public int Top;
        public int Right;
        public int Bottom;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct POINT {
        public int X;
        public int Y;
    }

    [DllImport("user32.dll")]
    public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);

    [DllImport("user32.dll")]
    public static extern bool GetClientRect(IntPtr hWnd, out RECT rect);

    [DllImport("user32.dll")]
    public static extern bool ClientToScreen(IntPtr hWnd, ref POINT point);

    [DllImport("user32.dll")]
    public static extern bool PrintWindow(IntPtr hWnd, IntPtr hdcBlt, uint flags);

    [DllImport("user32.dll")]
    public static extern bool IsIconic(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern bool ShowWindowAsync(IntPtr hWnd, int command);

    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    public static extern bool MoveFileEx(string existingPath, string newPath, uint flags);
}
'@

$destination = Join-Path $PSScriptRoot 'web\live'
$outputPath = Join-Path $destination 'camera.json'
New-Item -ItemType Directory -Path $destination -Force | Out-Null

function Write-CameraPacket {
  param([hashtable]$Packet)

  $temporaryPath = "$outputPath.$PID.tmp"
  $json = $Packet | ConvertTo-Json -Compress -Depth 4
  [IO.File]::WriteAllText($temporaryPath, $json, [Text.UTF8Encoding]::new($false))
  for ($attempt = 0; $attempt -lt 50; ++$attempt) {
    if ([NativeAirSimWindow]::MoveFileEx($temporaryPath, $outputPath, 9)) { return }
    $code = [Runtime.InteropServices.Marshal]::GetLastWin32Error()
    Start-Sleep -Milliseconds 20
  }
  throw "Не вдалося оновити кадр AirSim (Windows error $code)."
}

function Find-AirSimWindow {
  $process = Get-Process -Name 'MSBuild2018' -ErrorAction SilentlyContinue |
    Where-Object { $_.MainWindowHandle -ne 0 -and $_.Responding } |
    Sort-Object StartTime |
    Select-Object -Last 1
  if ($process) { return [IntPtr]$process.MainWindowHandle }
  return [IntPtr]::Zero
}

$sequence = 0
$observedAirSim = $false
$missingSince = $null
try {
  while ($true) {
    $handle = Find-AirSimWindow
    if ($handle -eq [IntPtr]::Zero) {
      if ($observedAirSim) {
        if ($null -eq $missingSince) { $missingSince = [DateTime]::UtcNow }
        if (([DateTime]::UtcNow - $missingSince).TotalSeconds -ge 5) {
          Write-CameraPacket @{
            schema = 'camera-preview-v1'
            connected = $false
            source = 'native-window'
            error = 'Вікно AirSim закрито.'
            received_unix_ms = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
          }
          break
        }
      }
      Write-CameraPacket @{
        schema = 'camera-preview-v1'
        connected = $false
        source = 'native-window'
        error = 'Вікно AirSim ще не відкрите.'
        received_unix_ms = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
      }
      Start-Sleep -Milliseconds 500
      continue
    }

    $observedAirSim = $true
    $missingSince = $null

    if ([NativeAirSimWindow]::IsIconic($handle)) {
      [void][NativeAirSimWindow]::ShowWindowAsync($handle, 4)
      Start-Sleep -Milliseconds 250
    }

    $rect = New-Object NativeAirSimWindow+RECT
    if (-not [NativeAirSimWindow]::GetWindowRect($handle, [ref]$rect)) {
      throw 'Не вдалося визначити розмір вікна AirSim.'
    }

    $windowWidth = $rect.Right - $rect.Left
    $windowHeight = $rect.Bottom - $rect.Top
    if ($windowWidth -lt 64 -or $windowHeight -lt 64) {
      throw "Некоректний розмір вікна AirSim: ${windowWidth}x${windowHeight}."
    }

    $windowBitmap = [Drawing.Bitmap]::new($windowWidth, $windowHeight, [Drawing.Imaging.PixelFormat]::Format24bppRgb)
    $graphics = [Drawing.Graphics]::FromImage($windowBitmap)
    $hdc = $graphics.GetHdc()
    try {
      $captured = [NativeAirSimWindow]::PrintWindow($handle, $hdc, 2)
    } finally {
      $graphics.ReleaseHdc($hdc)
      $graphics.Dispose()
    }

    if (-not $captured) {
      $windowBitmap.Dispose()
      throw 'AirSim не віддав кадр свого вікна.'
    }

    $clientRect = New-Object NativeAirSimWindow+RECT
    $clientOrigin = New-Object NativeAirSimWindow+POINT
    if (-not [NativeAirSimWindow]::GetClientRect($handle, [ref]$clientRect) -or
        -not [NativeAirSimWindow]::ClientToScreen($handle, [ref]$clientOrigin)) {
      $windowBitmap.Dispose()
      throw 'Не вдалося визначити область зображення AirSim.'
    }
    $width = $clientRect.Right - $clientRect.Left
    $height = $clientRect.Bottom - $clientRect.Top
    $cropX = $clientOrigin.X - $rect.Left
    $cropY = $clientOrigin.Y - $rect.Top
    $bitmap = [Drawing.Bitmap]::new($width, $height, [Drawing.Imaging.PixelFormat]::Format24bppRgb)
    $clientGraphics = [Drawing.Graphics]::FromImage($bitmap)
    try {
      $source = [Drawing.Rectangle]::new($cropX, $cropY, $width, $height)
      $clientGraphics.DrawImage($windowBitmap, 0, 0, $source, [Drawing.GraphicsUnit]::Pixel)
    } finally {
      $clientGraphics.Dispose()
      $windowBitmap.Dispose()
    }

    $stream = [IO.MemoryStream]::new()
    try {
      $bitmap.Save($stream, [Drawing.Imaging.ImageFormat]::Png)
      $png = [Convert]::ToBase64String($stream.ToArray())
    } finally {
      $stream.Dispose()
      $bitmap.Dispose()
    }

    $sequence++
    Write-CameraPacket @{
      schema = 'camera-preview-v1'
      connected = $true
      source = 'native-window'
      sequence = $sequence
      width = $width
      height = $height
      png_base64 = $png
      received_unix_ms = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
    }
    Start-Sleep -Milliseconds 200
  }
} catch {
  Write-CameraPacket @{
    schema = 'camera-preview-v1'
    connected = $false
    source = 'native-window'
    error = $_.Exception.Message
    received_unix_ms = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
  }
  throw
} finally {
  $mutex.ReleaseMutex()
  $mutex.Dispose()
}
