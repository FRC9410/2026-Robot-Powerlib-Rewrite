# 2026-Robot-Powerlib-Rewrite

The robot-specific Electron dashboard is in `power-tool`. See [Power Tool build instructions](power-tool/README.md).

From the repository root on Windows, build the standalone app with:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\build-dashboard.ps1
```

Requires Node.js 22 LTS and npm. The executable is `PowerTool-local\win-unpacked\Power Tool.exe`.
