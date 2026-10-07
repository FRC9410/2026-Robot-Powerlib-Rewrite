$toolRoot = Split-Path -Parent $PSScriptRoot
$isWindowsHost = [System.Environment]::OSVersion.Platform -eq "Win32NT"
$electron = if ($isWindowsHost) {
    Join-Path $toolRoot "node_modules/electron/dist/electron.exe"
} else {
    Join-Path $toolRoot "node_modules/.bin/electron"
}

if (Test-Path $electron) {
    Start-Process -FilePath $electron -ArgumentList $toolRoot -WorkingDirectory $toolRoot
} else {
    $npm = if ($isWindowsHost) { "npm.cmd" } else { "npm" }
    Start-Process -FilePath $npm -ArgumentList "start" -WorkingDirectory $toolRoot -WindowStyle Hidden
}

