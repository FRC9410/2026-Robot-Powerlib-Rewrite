# Cumulative vision contracts

Run from the robot project root, using the installed WPILib JDK:

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2026/jdk'
.\gradlew.bat --offline -I tests/vision-contracts.init.gradle test
```

The init script enables the otherwise disabled test task only for this invocation and selects these contracts. Results are in `build/reports/tests/test/index.html`.

Tests are independent: every stage starts with fresh state and exercises the preceding layers before adding its next assertion. Stages 1–5 use real local NetworkTables, the production camera reader, safety gates and a real WPILib pose estimator. Stages 6–8 run the actual `Vision.periodic()` with HAL and CTRE drivetrain simulation. No simulated optical images or camera-calibration claims are involved.

| Stage | Added contract | Realistic failure/recovery coverage |
| --- | --- | --- |
| 1 | Decode MT1/MT2 and capture timestamps | Frozen publications, leftover fiducials with zero tag count, truncated frames, delayed packets |
| 2 | Select by measurement quality | Conflicting cameras, ambiguous heading, two-tag preference and strict stationary single-tag fallback |
| 3 | Apply capture-time physical safeguards | Impossible field location/height, bump and buffered images after landing, gyro feedback loss and settling |
| 4 | Establish a stable stationary heading | Different camera rates, camera handoffs, frozen frames, a push during collection, unstable solves, +/-180 wrapping |
| 5 | Fuse timestamped measurements | Keep MT1 X/Y and MT2 yaw out of the estimator, unequal pipeline delays during motion, pre-reset image rejection and subsequent recovery |
| 6 | Complete the disabled startup flow | Hold X/Y until heading is established, discard pre-reset MT2, then remain ready through a five-second disabled wait |
| 7 | Continue across enabling and motion | Early enable, stationary hard recovery, moving 90-degree rejection, recovery after stopping |
| 8 | Recover from operational interruptions | Camera disconnect and reconnect, accepted-data freshness expiry, corrections-off during partial evidence collection |

Camera calibration, image accuracy, physical bump dynamics and roboRIO/radio performance still require on-robot validation.
## Autonomous checks

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2026/jdk'
.\gradlew.bat --offline -I tests/autonomous-checks.init.gradle test
```

These checks cover unchanged route points and leg profiles, continuing an unreached leg past three seconds, advancing only on arrival, cancellation cleanup, stopping on disable, and the existing overall autonomous-duration cutoff. The per-leg timeout and its tuning setting have been removed. The separate overall cutoff remains configured by `FieldConstants.AUTO_LENGTH_IN_TIME` (currently 15 seconds).
## Full competition regression suite

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2026/jdk'
.\gradlew.bat --offline -I tests/competition-checks.init.gradle verifyRobot
```

This runs every robot test, including the camera-recovery interactions and shooter readiness/feed regressions. Single-tag heading recovery is stationary-only: distance <=2 m, ambiguity <=0.05, age <=100 ms, at least ten distinct frames spanning 0.5 s within 1 degree. Two-tag recovery retains five frames spanning 0.25 s within 2 degrees. Eligible cameras collect independent evidence; an unstable higher-ranked camera cannot starve a stable backup. Position fallback clears only its own camera's recovery window.

`ShooterConstants.SHOOTER_EXTRA_RPS` is the one custom trim, defaulting to 2 RPS to preserve the previous configured +1 and hard-coded +1 total. Stationary/moving flywheel demand and readiness use the complete shooter target. Feeder demand retains its original lookup/solver +1 RPS reference, multiplied by the feeder ratio, independently of shooter trim. At the 2.5 m reference point the flywheel remains 30.5 RPS and the feeder remains -59 RPS. The 15-second total auto period and operator pose confirmation remain the operating policy; no exact-start-position interlock is introduced.
Cross-camera position ordering is tracked per camera. A better camera with 90 ms latency can recover after a 50 ms fallback; same-camera duplicates/out-of-order images, stale/future images and pre-reset captures remain rejected. Later accepted corrections are replayed at their original timestamps and weights.

## Conflicting-camera recovery

Heading confirmation keeps independent camera windows. After confirmation (or the first accepted
moving correction), heading selection survives ordinary frame gaps and mode changes. Equally
ranked or poorer cameras cannot take over just because they publish faster. Better cameras need
five distinct frames spanning 0.25 seconds of stronger quality; stationary handoffs also need
their independent stability window, including stricter single-tag requirements. A stale,
downstream-rejected, or stationary-inconsistent selected camera releases selection automatically.
Aligned stable candidates do not reset heading or discard position recovery evidence.

Native disagreement tests cover slower 90 ms / 16.7 Hz cameras against faster 50 ms / 50 Hz
cameras, repeated-reset prevention, simultaneous large-position recovery, selected-camera loss
and return, unstable selected solves, and gradual moving handoffs without hard resets. Capture
ordering and duplicates are tracked per camera for both heading and position; all pre-reset
images remain rejected. Later accepted samples are replayed with their original timestamps and
weights. Consistently wrong calibration/vision can still pass quality checks; this is selection
and recovery validation, not proof of optical accuracy or a cross-camera agreement vote.

## Vision cleanup regressions

Qualified stationary single-tag frames also correct small heading drift through weighted fusion.
The same strict single-tag evidence window applies; qualification is invalidated by motion,
stale/invalid evidence or inconsistent solves. Multi-tag evidence cannot authorize single-tag
fusion. A three-degree native regression verifies convergence without a hard reset or interruption
of position updates. Initialization-disabled tests verify passive stability and a sustained better
camera handoff without hard resets. Stationary thresholds and drivetrain-feedback freshness now
come from `VisionConstants.HEADING_INITIALIZATION_CONFIG`; weighted correction uses the same
stationary classification when selecting the 180/45-degree limits.

Mode-transition tests assert reset counts, heading establishment and resumed position acceptance,
rather than a collector's private initialization flag. Four disabled frames cannot combine with
four enabled frames to trigger a hard reset; each phase requires its own complete evidence window.
Normal weighted multi-tag heading correction remains allowed during enabled collection.
`verifyRobot` runs the full suite only when the competition init script is supplied; ordinary
build/deploy behavior is unchanged. No added localization/start interlock or per-leg timeout is
part of this cleanup.

Test mode blocks automatic heading initialization/recovery and weighted heading corrections.
After heading has been established, valid position updates continue in Test mode. The native
`testModeKeepsPositionFusionButBlocksHeadingChanges` regression verifies the transition from
disabled localization, position convergence with heading held fixed, corrections-off suppression,
and position recovery when updates are restored, without hard resets.

Stationary inconsistency rejection applies to both hard recovery and ordinary weighted heading
fusion. Native regressions assert no transient heading kick from alternating bad solves, continued
position acceptance and backup-camera fallback, and recovery after a fresh stable window in both
disabled and enabled modes. The 180-degree stationary and 45-degree moving limits are unchanged.
Shooter-only trim regressions vary the flywheel trim at a fixed shot and require unchanged feeder
demand for stationary and moving shots, while readiness follows the actual flywheel target.