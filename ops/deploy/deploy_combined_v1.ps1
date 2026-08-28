#!/usr/bin/env pwsh
<#
.SYNOPSIS
  Deploy the combined_v1 multitask model to the Mazge inference server (macOS).

.DESCRIPTION
  Swaps MAZGE_PREY_ONNX to the new 4-head ONNX and sets the calibrated per-frame
  prey threshold (MAZGE_PREY_DETECT=0.76), restarts the launchd service, and
  verifies /healthz. The ONNX is drop-in (same inputs; server reads outputs 0/1),
  so no server/ code change is required. Safe + idempotent: backs up server.env,
  supports -Rollback.

  PREREQS (cannot be scripted — do these first):
    1. cd ~/Mazge && git pull                         # code + env-threshold support
    2. AirDrop the ONNX onto this Mac:
         _bench_weights/multitask_combined_v1_224.onnx -> ~/Mazge/_bench_weights/
       (or AirDrop best.pt and export with tools/export_multitask_onnx.py)

.EXAMPLE
  pwsh ops/deploy/deploy_combined_v1.ps1
.EXAMPLE
  pwsh ops/deploy/deploy_combined_v1.ps1 -Rollback
#>
[CmdletBinding()]
param(
  [string]$RepoDir     = "$HOME/Mazge",
  [string]$OnnxPath    = "$HOME/Mazge/_bench_weights/multitask_combined_v1_224.onnx",
  [string]$OldOnnxPath = "$HOME/Mazge/_bench_weights/prey_v3_bodyA_s480_224.onnx",
  [string]$EnvFile     = "$HOME/.config/mazge/server.env",
  [double]$PreyDetect  = 0.76,
  [string]$Service     = "com.mazge.server",
  [string]$HealthUrl   = "http://mazge.local:8080/healthz",
  [switch]$RunTests,
  [switch]$Rollback
)

$ErrorActionPreference = "Stop"

function Set-EnvVar([string]$File, [string]$Key, [string]$Value) {
  $pattern = "^$([regex]::Escape($Key))="
  $lines = @(Get-Content -Path $File)
  if (($lines | Select-String -Pattern $pattern -Quiet)) {
    $lines = $lines | ForEach-Object { if ($_ -match $pattern) { "$Key=$Value" } else { $_ } }
  } else {
    $lines += "$Key=$Value"
  }
  Set-Content -Path $File -Value $lines
}

function Remove-EnvVar([string]$File, [string]$Key) {
  $pattern = "^$([regex]::Escape($Key))="
  @(Get-Content -Path $File) | Where-Object { $_ -notmatch $pattern } | Set-Content -Path $File
}

if (-not (Test-Path $EnvFile)) { throw "server.env not found: $EnvFile" }

# --- backup server.env ---
$backup = "$EnvFile.bak.$(Get-Date -Format yyyyMMdd_HHmmss)"
Copy-Item -Path $EnvFile -Destination $backup
Write-Host "backed up server.env -> $backup"

if ($Rollback) {
  Write-Host "ROLLBACK: MAZGE_PREY_ONNX -> $OldOnnxPath ; removing MAZGE_PREY_DETECT" -ForegroundColor Yellow
  if (-not (Test-Path $OldOnnxPath)) { Write-Warning "old ONNX missing: $OldOnnxPath (check the path)" }
  Set-EnvVar $EnvFile "MAZGE_PREY_ONNX" $OldOnnxPath
  Remove-EnvVar $EnvFile "MAZGE_PREY_DETECT"
} else {
  if (-not (Test-Path $OnnxPath)) {
    throw "ONNX not found: $OnnxPath`n  AirDrop it to the server first (see PREREQS in this script's header)."
  }
  Write-Host "deploying: MAZGE_PREY_ONNX=$OnnxPath  MAZGE_PREY_DETECT=$PreyDetect"
  Set-EnvVar $EnvFile "MAZGE_PREY_ONNX" $OnnxPath
  Set-EnvVar $EnvFile "MAZGE_PREY_DETECT" ([string]$PreyDetect)
}

Write-Host "`n--- server.env prey settings now ---"
Get-Content $EnvFile | Select-String -Pattern "MAZGE_PREY_ONNX|MAZGE_PREY_DETECT"

# --- restart launchd service ---
$uid = (& id -u).Trim()
Write-Host "`nkickstarting gui/$uid/$Service ..."
& launchctl kickstart -k "gui/$uid/$Service"

# --- verify health ---
Start-Sleep -Seconds 5
$resp = $null
for ($i = 0; $i -lt 6; $i++) {
  try { $resp = Invoke-RestMethod -Uri $HealthUrl -TimeoutSec 5; if ($resp.ok) { break } }
  catch { Start-Sleep -Seconds 3 }
}
if ($resp -and $resp.ok) {
  Write-Host "HEALTH OK: backend=$($resp.backend) uptime_s=$($resp.uptime_s)" -ForegroundColor Green
} else {
  Write-Warning "HEALTH CHECK FAILED. Inspect logs (~/Mazge/logs/server/launchd.err.log)."
  Write-Warning "Rollback: pwsh ops/deploy/deploy_combined_v1.ps1 -Rollback"
  exit 1
}

# --- optional: server test suite ---
if ($RunTests) {
  Write-Host "`nrunning server tests ..."
  Push-Location $RepoDir
  try { & uv run python -m pytest server/tests/ -q } finally { Pop-Location }
}

Write-Host "`nDEPLOYED. Now watch a real burst decision before trusting it:" -ForegroundColor Cyan
Write-Host "  - a clean cat should get cat_recognized=true and door_action=allow"
Write-Host "  - confirm prey_score/cat_id look sane in ~/Mazge/logs/server/server.jsonl"
Write-Host "Rollback anytime: pwsh ops/deploy/deploy_combined_v1.ps1 -Rollback"
