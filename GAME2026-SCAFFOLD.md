# 2026 game scaffold

Implemented on `codex/powerlib-rewrite`, using the verified pre-May-1 configuration. The scaffold supports driver control, intake/eject, hub shooting, wrist agitation, and the original four quadrant autonomous sequences. Passing and climbing have no commands, states, or controller bindings. The generated turret configuration is retained; aiming follows the original StateMachine's chassis-based approach rather than adding turret tracking.

## Controls

| Control | Behavior |
| --- | --- |
| Left stick / right stick X | Field-relative translation / rotation, with red driver perspective handled explicitly |
| Left trigger above 0.5 | Deploy intake wrist and collect at 145 rps; release stops the roller and returns wrist to idle |
| B | Deploy intake wrist and eject at -100 rps; eject takes priority when both intake inputs are held |
| Right trigger above 0.5 | Aim the chassis at the alliance hub, spin shooter/position hood from the shot table, and feed only when ready |
| Back, while disabled | Seed pose from a fresh accepted Limelight measurement; this does not manufacture an autonomous starting pose |

The shooting and intake bindings run in Teleop. Shooting uses zero translational drive coefficient, and intake uses 0.4, matching the reference command. Intake and eject can still run if the alliance is unknown, but driving and hub shooting wait for alliance data. Requests clear at mode transitions. Enabled Back presses do not reset pose.

## Shooting

`Game2026Controller` computes mechanism demands from the verified game JSON. Shooter demand is positive interpolated speed plus the live offset (default 1 rps); feeder demand is negative interpolated speed; spindexer demand is 60 rps. Intake wrist cycles between feed and idle after one second of continuous shooting, every half second, corresponding to the old 50/25-loop timing at 20 ms. Collection/ejection overrides that agitation. Releasing shooting resets the timer.

Feeding requires the robot to be inside its alliance scoring zone and within the measured shot-table range, with known alliance, healthy drive/leader feedback, correct signed shooter speed within 2 rps, hood within 0.005 rotations, and heading within 5 degrees. Ejection inhibits feeding. Outside the scoring zone, the shooter/feed outputs stop; no corner or passing target is selected. The source robot's Red shooter heading offset is retained and applied in the blue field frame.

The 0.005-rotation hood tolerance is a new scaffold default, rather than a recovered calibration. Real-hardware health checks use the public leader-motor feedback available in the installed PowerLib API and each drivetrain module's motor/encoder feedback. Simulation uses PowerLib's mechanism IO snapshots. Position demands are clamped to the generated wrist/hood limits. Test mode suspends competition output writes so the existing generated SysId commands retain control.

## Autonomous

PowerLib's standard `Auto Chooser` registers fresh instances of Red Left, Red Right, Blue Left and Blue Right. Each option is filtered to its alliance; the default remains None. The seven stored positions remain unchanged. Legacy Red headings are converted to blue-field headings once, so -90 becomes 90 and 135 becomes -45.

The sequence drives the first two poses, collects through poses 3–5, releases intake, drives poses 6–7, then aims/shoots at the hub. The original speed coefficients are 0.5, 1, 0.75, 0.75, 0.4, 0.75, 1; distance tolerances are 6, 6, 12, 6, 6, 3, 3 inches. Legs 2 and 7 intentionally do not lock heading, as in the reference sequence. Translation uses kP 3.2/kD 0.2 and reference friction feedforward; turn control uses the Swerve heading kP. Drive vectors are capped by magnitude, with an explicit blue-field request perspective.

Starting pose must be within 0.5 m and 15 degrees of pose 1. Each leg is bounded to 3 seconds, and the entire routine to the reference's configured 15 seconds. These start/leg thresholds are new scaffold defaults. Wrong/unknown alliance, stale drive feedback, timeout, invalid settings, or leaving Autonomous aborts/cancels the routine and stops drive/mechanisms. The command does not reset odometry to force a starting-pose match. Real-robot route following and the inherited shooter orientation still need physical validation.

## Configuration and telemetry

Edit shot rows, intake roller speeds and autonomous poses in `power-tool/generated/powerlib-game-2026.json`. Gradle's `syncGameConfiguration` task copies that exact file into `src/main/deploy` for every build/deploy that needs a refresh. The runtime validates finite data, ordered shot rows, field bounds and seven poses per routine before use. Wrist positions use the live IntakeWrist custom constants: `INTAKE_MAX` for collecting/ejecting, `INTAKE_IDLE` when released, `INTAKE_FEED` during shooting agitation, and `INTAKE_DEFAULT` in IDLE. Targets remain clamped between `INTAKE_MIN` and `INTAKE_MAX`. These values are rotations despite the existing dashboard units label. The shot tables also retained in generated TurretConstants are historical reference constants.

Behavior defaults are saved in native `powerlib-constants.json`: StateMachine has hood tolerance, shooter extra speed and spindexer speed; RobotContainer has start tolerances, leg timeout, translation PID and the two driver coefficients. Power Tool Update Code regenerates those fields and their tuning registrations. Heading control reads the existing live Swerve `Heading/kP` variable. Keep behavior tolerances/gains/timeouts positive and shooter extra speed nonnegative; invalid settings stop outputs. The original configuration-preparation script recreates the historical baseline for comparison; ordinary edits use the saved JSON and Power Tool Update Code.

Shooting velocity uses `TurretConstants.SHOOTER_VELOCITY_INTERPOLATOR` and its `SHOOTER_SPEEDS` calibration, matching the old robot. Each loop evaluates robot-to-hub distance in meters and commands the positive interpolated RPS plus the live `SHOOTER_EXTRA_RPS` offset (default 1). Feed readiness compares measured velocity against that same signed demand. Game JSON shot rows still supply hood and feeder targets and the allowed distance range; their shooter-speed column is retained as reference data and does not set flywheel demand. Edit the Turret custom table in Power Tool's generated configuration and regenerate/deploy to change the calibration.

All three interpolation tables were restored from `FRC9410/2026-Robot` commit `0f75d1b75d1428dd8b73213b2447adb292ab2125` (April 4, 2026, “city of fountains 😔”), the final commit during the April 1–4 City of Fountains Regional. The tables are synchronized in TurretConstants, Power Tool's custom constants JSON, and the game shot rows. Calibrated distances are 2.5–4.9 m; shooter table speeds are 28.5–37 RPS. Shooting uses the event code's positive velocity sign with the current tunable speed offset. The event code also applied an additional -0.005 rotation hood adjustment; that adjustment is not part of the table restoration.

The shooter leader (51) uses PowerLib `reversed=false` for counterclockwise-positive direction, preserving the old robot's effective configuration with positive shooting demand. Followers 52 and 53 remain opposed to the leader. Both feedback ratios are 1. Velocity subsystem initialization includes motor direction and neutral mode in the full Talon configuration so subsequent configuration does not reset them. After deploying, any existing live `/PowerLib/Subsystems/Shooter/Variables/Motors/51/Reversed` value must also be false when tuning is enabled.

Game state, shooting reason, intake/eject flags, feed readiness, hub distance, heading error and autonomous status/reason publish under `/PowerLib/Subsystems/Game2026/Data`. Autonomous status/reason also appear on SmartDashboard. Existing Drive telemetry and characterization publishing remain available.

## Validation

The Gradle build and 25 JUnit tests pass. Tests exercise shot interpolation/validation, both alliance heading conventions, scoring-zone gating, signed flywheel readiness, hood/alignment/feedback guards, mode resets, Test-mode handoff, intake/eject release, agitation timing, all four autonomous sequences, motion caps, timeouts and cancellation. The historical configuration verifier also passes for all four JSON files and 215 Java values, including the new behavior constants. Runtime/deploy game JSON hashes match. No hardware deployment was performed.

The screenshot-based 2026 Game dashboard is installed in Power Tool. See [power-tool/GAME2026-DASHBOARD.md](power-tool/GAME2026-DASHBOARD.md) for its telemetry contract, camera settings, route preview and development-only sample data.
