# 2026-Robot-Powerlib-Rewrite

## Shoot on the move (`sotm-testing`)

Holding shoot now keeps driver translation available at 40% of normal speed
(`DRIVE_WHILE_SHOOTING_COEFFICIENT`, tuned separately from intake speed). The runtime firecontrol classes `ShotCalculator`,
`ShotLUT` and `ShotParameters` were adapted from
[2026-ROBOT-SIM-TESTING](https://github.com/aydenterranella/2026-ROBOT-SIM-TESTING/tree/e2bfadbd89e17429873ec029ce2a8b123377f4ec/src/main/java/frc/firecontrol).
Their FRC Team 5962 perSEVERE MIT attribution is retained. No MapleSim, projectile
simulator, LUT generator, or simulation-only dependencies were added.

The solver converts measured robot-relative velocity into field coordinates, low-pass
filters it (`SOTM_VELOCITY_FILTER_SECONDS`, 0.1 s), predicts the launcher through a
20 ms release delay, includes rotational velocity at the rear-mounted launcher, and
solves a drag-adjusted virtual target using Newton's method. Acceleration is not
estimated: differencing measured velocity roughly tripled noise in the aim heading.
Current RPS/hood tables and offsets remain intact: robot-center calibration distances
are shifted by the 0.20 m rear launcher offset inside the lookup table so stationary
shots retain their original demands. Phoenix already supplies a current pose, so no
additional vision latency is applied.

The imported flight-time table is **provisional**: the reference used different RPS
and hood settings. Measure flight times for this robot's retained calibration before
relying on moving-shot accuracy. Verify the 0.20 m launcher offset and release delay
on hardware too. No new on-robot shot calibration is claimed by the software tests.

When the robot isn't driving (filtered translation under 0.1 m/s, including turning in
place to aim), it uses the original standing shot unchanged, and tilt is not checked.
While driving, both actual center distance and compensated calibration distance must
remain within 1.95-4.9 m in the scoring zone. Feeding still requires the existing
5-degree heading, hood, flywheel and feedback checks. Excessive tilt (over the tunable
`SOTM_MAX_TILT_DEGREES`, 5 degrees), speed (over 3 m/s), invalid input, uncalibrated lead
distance or solver nonconvergence prevents a moving shot and stops the shooter until a valid shot returns. Solver
confidence is diagnostic; feed permission uses the explicit validity and readiness checks.

Teleop aiming slews the heading target at up to `AIM_HEADING_SLEW_DEGREES_PER_SECOND`
(90) so lead changes and moving/static fallbacks turn the robot smoothly, and adds the
configured `Heading/kD` as damping. Lower the slew rate if the wheels still splay while
aiming. Once translation and rotation settle within 1 degree of aim, wheels park
instead of retaining the turn-in-place stance: parallel with SOTM on, or X-locked
(`SwerveDriveBrake`) with SOTM off so the stationary shot resists pushes. Driver
translation or drift beyond 2 degrees resumes normal aiming. This applies to teleop and
the autonomous final aim. The turn-in-place wheel angles remain necessary while the
robot is rotating.

The **Shoot on move** button on Power Tool's Shot ready card is the kill switch. OFF
restores the original stationary shot with driver translation held at zero while
aiming. The button writes `/PowerLib/Shooting/SOTMRequestedEnabled`; `SotmSwitch`
applies each new request every loop, with or without tuning, and the button shows the
robot's reported `Shooting/SOTMEnabled`. Robot code starts with
`SOTM_ENABLED_DEFAULT` (on).

The Shot ready card also shows SOTM confidence, lead angle and flight time, and the
compensated aim distance. The robot publishes
`Shooting/SOTMEnabled`, `LeadDegrees`, `EffectiveDistanceMeters`, `TimeOfFlightSeconds`,
`MotionCompensated`, `SOTMValid`, `SOTMConfidence`, `SolutionStatus`,
`Drive/AimWheelsParked` and `Drive/AimWheelsLocked`. Tunable constants include
`RobotContainer/Custom/DRIVE_WHILE_SHOOTING_COEFFICIENT` (shooting drive speed)
and Shooting's `SOTM_MAX_DRIVETRAIN_SPEED_MPS`, `SOTM_RELEASE_DELAY_SECONDS`, `SOTM_DRAG_COEFFICIENT`,
`SOTM_MAX_TILT_DEGREES`, `SOTM_VELOCITY_FILTER_SECONDS` and
`AIM_HEADING_SLEW_DEGREES_PER_SECOND`. Source
constants and `powerlib-constants.json` are kept in sync. Tests cover velocity lead,
filtering, release latency, rotating-launcher velocity, solver failures, retained
stationary calibrations, compensated readiness, the kill switch, shooting drive speed, aim
slewing and parking.

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
