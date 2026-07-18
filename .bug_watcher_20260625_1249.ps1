$sleeperPath = Join-Path (Get-Location) '.bug_sleeper_20260625_1249.ps1'
function Read-Prompt { param([string]$Path) if (Test-Path $Path) { try { Get-Content $Path -Raw -ErrorAction SilentlyContinue } catch {} } }
$lastPayload = Read-Prompt -Path $sleeperPath
if (-not $lastPayload) {
  $payload = ''{prompt: tìm l?i và fix và tìm di?m c?i ti?n, tick: 1249, now: 2026-06-25T12:49:00+07:00}''
  Set-Content -Path $sleeperPath -Value $payload
}
while ($true) {
  $sleepSeconds = 900
  Start-Sleep -Seconds $sleepSeconds
  $message = ''AGENT_LOOP_WAKE_bugfix_20260625_1249 {"prompt":"tìm l?i và fix và tìm di?m c?i ti?n","tick":1249}''
  $message | Out-File -FilePath $sleeperPath -Encoding utf8
}
