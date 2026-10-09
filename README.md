# 2026-Robot-Powerlib-Rewrite

## PowerLib installation

This branch starts from the fresh `main` robot project at `c37a98f33`.
PowerLib source and Power Tool were installed from Robot-Library commit
`5f73c86bbc72bbc5c4092657fabae3199e848722` with all options enabled:
library, robot templates, vendor dependencies, generator/tools, Electron dashboard,
and simulation skills. Robot and Power Tool production builds passed.

Robot configuration uses original 2026-Robot commit
`0f75d1b75d1428dd8b73213b2447adb292ab2125` (April 4, 2026, on
`NewShooterTesting`). Seven mechanisms, swerve settings, shooting tables, field
geometry and all 28 auto waypoints were compared against that commit. Power Tool
configuration is saved in `power-tool/generated/powerlib-subsystems.json` and
`power-tool/generated/powerlib-constants.json`; regenerate with
`power-tool/scripts/generate-subsystem.ps1 -UpdateSubsystems`.

The four autos use the original intake/return/turn/shoot sequence, per-leg speeds,
tolerances and heading-lock choices. Red headings are converted to blue-field
coordinates. Auto selection follows the reported alliance, with `None` as the default.
The rewrite retains 100 ms dashboard updates, 200 ms tuning, 50 Hz logging,
readiness/freshness checks, disabled-only pose seeding, start-pose guards and auto
leg timeouts. Shooting uses the original calibrated 2.5-4.9 m interval; the legacy
code clamped outside that interval. `gradlew build` tests shot outputs, route values,
state transitions, cancellation and timeout behavior.

Motor neutral and reversal declarations match April 4. The legacy leader reversal
implementation wrote CounterClockwise_Positive for a true reversal flag, and later
full configuration writes could overwrite output settings. PowerLib retains the
declared neutral mode and uses Clockwise_Positive for a true reversal flag. Motor
direction needs hardware verification; matching declarations does not guarantee
identical historical electrical direction.

State handlers implement Idle/Ready, Intaking, Ejecting, Spinning Up, and Shooting.
Left trigger collects, B requests eject, and right trigger requests shooting.
A shooting request spins up first and feeds only when shooter velocity, hood position,
heading, shot range, and feedback are ready. Losing readiness immediately stops feeding.
Releasing B resumes a held shooting or intake trigger; releasing all triggers returns
to `IDLE`. Intake can run while shooting. Shared geometry, interpolation,
readiness, and shooter/feeder actions live in `ShootingUtils`. Values and driver behavior
use the April 4 calibrations described above. Back seeds a fresh vision pose while disabled;
Start zeros heading while disabled, preserving X/Y. Test mode leaves outputs to SysId.

Each state request has an owner: `requestState(state, owner)` updates only that owner's
entry, and `clearRequest(owner)` removes it. The ordered handler list in `StateMachine`
is the sole request priority: Ejecting, Shooting, Spinning Up, Intaking, then Idle/Ready.
ButtonBindings.bindStates(button, pressState, releaseState, stateMachine::requestState)
creates a stable owner per binding. Both triggers and B request IDLE on release, so an
older button release cannot overwrite another held button's request. The function
helper is ButtonBindings.bindFunctions(button, onPress, onRelease).

HealthChecks in PowerLib owns signal freshness, finite-pose checks, and mechanism/drivetrain
feedback checks. Vision, drive commands, and shooting readiness use it. Robot code chooses
the mechanisms required for shooting. IntakingState owns collection and wrist travel limits;
EjectingState owns reverse intake outputs; IdleState owns roller stops and idle posture.
ShootingState owns the shot snapshot, required-mechanism health checks, and wrist-pulse
timer, shared with SpinningUpState. It delegates held collection to IntakingState.
StateMachine calls each handler's optional prepare hook before selection and reset hook
on mode changes. Shooting preparation runs only while a shooting or spin-up request is
pending; execution rechecks readiness every cycle. PowerDashboard publishes distance
independently, so it continues updating during idle and intake. Shared shooting utilities
contain no intake, wrist, or health behavior.

The drive command receives live current state for intake/shooting speed scaling and
hub alignment. Robot tests cover shot-table values, readiness, transitions,
and button priority. PowerLib feedback-ratio validation also accepts signed CTRE
ratios, retaining the rewrite's negative sensor-to-mechanism ratios.

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
