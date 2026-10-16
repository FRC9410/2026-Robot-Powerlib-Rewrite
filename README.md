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
leg timeouts. Shooting allows distances from 1.95-4.9 m. A new 1.95 m row extends
the closest April 4 segment (2.5-3.0 m): 27.4 RPS and 0.044 hood rotations before
offsets. Shots closer than 1.95 m are rejected. The new values are extrapolated and need
hardware tuning. `gradlew build` tests shot outputs, route values,
state transitions, cancellation and timeout behavior.

Motor configuration was audited again against April 4 commit `0f75d1b`. Shooter
leader CAN 51 uses `reversed: false` in PowerLib to preserve the legacy
CounterClockwise_Positive direction; followers 52 and 53 remain opposed. The other
active mechanism leaders already use that direction. Feeder, shooter and spindexer
also restore the legacy 140 A stator current limit on their leaders, explicitly
saved in the subsystem JSON and emitted by the generator. PowerLib preserves the
declared neutral modes; legacy full configuration writes could reset those modes.
RegionalMotorConfigurationTest checks the Phoenix output configuration without
hardware. Physical direction still needs verification on the robot.

The audit includes feeder, intake roller/wrist, shooter/hood and spindexer CAN IDs,
follower alignment, PID/feedforward, feedback ratios, encoder offsets, Motion Magic
settings and position defaults/limits. Shared auto waypoints, Tuner hardware,
controller, LED and camera constants retain the April 4 values. Field dimensions
are corrected to 16.541 by 8.069 m and hub centers to red (11.915521, 4.0346376)
and blue (4.6255178, 4.0346376), derived from the WPILib 2026 welded layout's opposing
hub-face tags. The red zone ends at the corrected field length. The screenshot pose
(13.27, 3.18) is about 1.602 m from the corrected red hub, below the retained 1.95 m
minimum shooting distance; its target heading is about -32.25 degrees. The removed turret
and legacy sweep feature are not reinstated; their active shooter/hood tables live
in ShootingConstants. New PowerLib vision/readiness settings remain rewrite-specific.

State handlers implement Idle/Ready, Intaking, Ejecting, Spinning Up, and Shooting.
Left trigger collects, B requests eject, and right trigger requests shooting.
A shooting request spins up first and feeds only when shooter velocity, hood position,
heading, shot range, and feedback are ready. Losing readiness immediately stops feeding.
Releasing B resumes a held shooting or intake trigger; releasing all triggers returns
to `IDLE`. Intake can run while shooting. Shared geometry, interpolation,
readiness, and shooter/feeder actions live in `ShootingUtils`. Values and driver behavior
use the April 4 calibrations described above. Back seeds fresh vision position while disabled
and uses fresh independent multi-tag heading when available, preserving heading otherwise;
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

Heading corrections track independent MT1 frame timestamps. When an MT1 frame
arrives behind an already-applied MT2 position frame, its capture-time heading
error is carried forward to the latest fusion timestamp, preserving intervening
rotation and newer position corrections. Cameras remain unthrottled while disabled
to keep reset frames fresh. SignalLogger records Vision/Heading/Status, Accepted,
ErrorDegrees, PigeonYawDegrees and EstimatedDegrees. While disabled, fresh multi-tag
MT1 frames automatically initialize heading once per disabled period after at least
five distinct frames agree within 2 degrees for 0.25 s. Speed must be at most
0.05 m/s and yaw rate at most 5 degrees/s, with fresh drivetrain feedback. This
heading-only reset preserves X/Y and can recover errors larger than 15 degrees.
Vision/HeadingStatus and Vision/MT1HeadingFresh are also published to Power Tool.
While enabled, automatic correction still
requires two reliable tags within 4 m, frame age at most 0.2 s, yaw rate at most
45 degrees/s, and heading error at most 15 degrees. Back can reseed a larger heading
error from fresh independent heading while disabled; Start zeros blue-field heading.

The drive command receives live current state for intake/shooting speed scaling and
hub alignment. Robot tests cover shot-table values, readiness, transitions,
and button priority. PowerLib feedback-ratio validation also accepts signed CTRE
ratios, retaining the rewrite's negative sensor-to-mechanism ratios.

Autonomous point commands now request braking before the tolerance boundary and require
position, optional heading, and low measured translation/angular speed to hold continuously.
The provisional arrival limits are 0.05 m/s, 5 degrees/s and 0.1 s, configured in AutoConstants;
validate them and route timing on hardware. The April 4 waypoints, per-leg tolerances and
heading locks are unchanged. Normal braking uses the shared chassis motion limiter;
cancellation and faults stop immediately. Leg deadlines remain 3 s, final aiming 1 s and
the overall routine 20 s. Final aiming uses the same completion check with a live hub heading
and the last waypoint's position tolerance; a timeout aborts before requesting SHOOTING.
The command caches completion during execute(), so isFinished() has no control side effects.

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
