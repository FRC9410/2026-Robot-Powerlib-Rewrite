@echo off
for %%I in ("%~dp0..") do set "TOOL_ROOT=%%~fI"
set "ELECTRON_EXE=%TOOL_ROOT%\node_modules\electron\dist\electron.exe"
if exist "%ELECTRON_EXE%" (
  start "" "%ELECTRON_EXE%" "%TOOL_ROOT%"
) else (
  powershell -NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -Command "Start-Process -FilePath npm.cmd -ArgumentList start -WorkingDirectory '%TOOL_ROOT%' -WindowStyle Hidden"
)

