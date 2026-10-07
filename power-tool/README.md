# Power Tool

The 2026 robot dashboard is installed in this repository's `power-tool` folder.

## Build and run on Windows

Install Node.js 22 LTS, including npm, then open PowerShell at the robot repository root:

```powershell
cd power-tool
npm ci
npm run build
npm start
```

`npm ci` restores the dependencies and Electron binary for this computer from the lockfile. Compiled dashboard files are generated locally and are not committed. To work on the dashboard with live reload, use `npm run dev` instead of the last two commands.

## Build a standalone Electron app

From the robot repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\build-dashboard.ps1
```

The script installs dependencies, builds the renderer and Electron process, and packages the app. On Windows, open `PowerTool-local\win-unpacked\Power Tool.exe`. Keep the entire `win-unpacked` folder together if you copy it to another computer.

If you have already run `npm ci`, skip dependency installation:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\build-dashboard.ps1 -SkipInstall
```

Inside `power-tool`, `npm run package:app` does the same build with `-SkipInstall`. `npm run package:publish` writes to `PowerTool` instead of `PowerTool-local`; it builds locally and does not upload anything. `-Platform win`, `mac`, or `linux` selects a platform; build on the target operating system for the simplest setup.

The build script belongs to this robot repository and uses the installed dashboard directly. Reinstalling the dashboard from Robot-Library replaces its app source, including robot-specific dashboard changes.

## NetworkTables

Use `10.94.10.2`, `roborio-9410-frc.local`, or `localhost` for robot or simulation telemetry. The dashboard connects over NT4 WebSocket on port `5810`. A Driver Station laptop address works only when that computer runs a NetworkTables server.

See [GAME2026-DASHBOARD.md](GAME2026-DASHBOARD.md) for the game layout, generated configuration, telemetry, cameras, and preview mode.
