# 2026-Robot-Powerlib-Rewrite

## PowerLib installation

This branch starts from the fresh `main` robot project at `c37a98f33`.
PowerLib source and Power Tool were installed from Robot-Library commit
`5f73c86bbc72bbc5c4092657fabae3199e848722` with all options enabled:
library, robot templates, vendor dependencies, generator/tools, Electron dashboard,
and simulation skills. Robot and Power Tool production builds passed.

Subsystem, drivetrain, and custom-constant configuration comes from the committed
`codex/powerlib-rewrite` at `edfea02a5f141514a71831b7bce8a596b1d6c2cc`.
Power Tool saved `power-tool/generated/powerlib-subsystems.json` and its installed
generator produced the robot constants, subsystem instances, characterization bindings,
and tuning registration. The constants and tuning-selection JSON retain the committed
rewrite configuration. CAN bus, LED, field, and CTRE Tuner constants are also preserved.
`power-tool/generated/comparison.json` records the verified source commits and results:
seven subsystem configurations match exactly; 59 saved constants and 174 mechanism
constants were checked against generated Java; hardware constants match exactly.

The new state-handler scaffold is retained. `IDLE`, `READY`, and `SHOOTING` are available
as state names, preserving the committed `DEFAULT_STATE = READY`. Only the Idle handler
is implemented so far. Current state starts at `IDLE`; `READY` and `SHOOTING` requests
remain pending until their handlers are added. The drive command receives live current
state through a supplier.

Build and simulate with the WPILib 2026 JDK:

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2026/jdk'
./gradlew.bat build
./gradlew.bat simulateJava
```

Power Tool dependencies are installed locally and reproducible from `package-lock.json`.
On another checkout, run `npm ci` and `npm run build` from `power-tool`, then use
`power-tool/scripts/power-tool.cmd` to launch Electron. Build artifacts, dependencies,
and machine-specific paths are ignored.
