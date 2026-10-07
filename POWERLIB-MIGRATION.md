# PowerLib configuration migration

Reference: `FRC9410/2026-Robot` commit `a5f9d31c35f06de953dcdb5e9d7d4ce57001a65c` (March 27, 2026), the latest commit on local `main` before May 1, 2026 at midnight America/Chicago. The rewrite repository starts in October and has no pre-May history.

The installed library-test-robot provided the Power Tool JSON shapes. Its six mechanism entries were checked against the historical Java configuration. The verified configuration includes Feeder, Intake Roller, Intake Wrist, Shooter, Shooter Hood, Spindexer, and Turret. Turret had a historical configuration but was not instantiated in the old StateMachine; the generator now instantiates it. Sweep defines navigation targets, not a motor mechanism.

## Corrections before Update Code

- Swerve heading kP: 7 -> 6.
- Swerve drive-to-point request angular limit: pi -> 1.5 pi radians/second, matching the old Swerve request clamp. The old SwerveDriveCommand separately declared an unused 2 pi limit.
- Intake Wrist and Shooter Hood FOC: true -> false, matching the historical MotionMagicVoltage requests. Turret also uses false. Velocity mechanisms retain FOC; Intake Roller retains torque-current control and zero torque feedforward.
- Position telemetry units: rotations -> the historical `degrees` label. Position/default values remain rotations; the old code's label did not convert the values.
- Added Turret CAN IDs 60/61, kP 57, ratio 51 * (8.5 / 9), encoder offset 0.1255, limits and all three shot interpolation tables.
- Restored the CANdle bus from the template Rio bus to historical CANivore, preserving its CAN ID and LED strip values.
- Preserved ancillary CAN IDs: feeder CANdi 42/43 and spindexer lasers 31/32.
- Replaced test-only reduced-speed OI value with historical operator port 1 and interchange speed coefficient 0.75. Restored all three camera names and historical camera constants.
- Supplementary game reference JSON uses the historical field bounds 17.5 by 8.0, actual source commit, shot tables, intake setpoints, autonomous poses, and location/sweep constants. Removed test-only alert thresholds, image metadata and start/hood tolerances without historical counterparts.

`power-tool/generated/powerlib-subsystems.json`, `powerlib-constants.json`, and `powerlib-tuning-selection.json` are native Power Tool configuration files. `powerlib-game-2026.json` is supplementary reference data from the test robot; stock Update Code does not consume it. Tuning selection starts empty because it is UI state rather than a robot calibration.

## Scope and verification

The Power Tool Update Code action generates the mechanism configurations, initialization, characterization commands, custom constants and tuning registry. It does not port the old game's StateMachine behavior, controller bindings or autonomous command sequences. The installed scaffold retains an empty periodic method; this branch is a configuration migration, not a complete operational robot rewrite.

The machine-readable `power-tool/generated/comparison.json` records 351 reference checks and the corrected test-robot differences. The CTRE TunerConstants hardware calibration already matches the reference; its only differences are comments and whitespace. The installed PowerLib vision validation retains `LimelightVisionConfig.DEFAULT` as new library behavior. `motorType: X60` is inherited Power Tool simulation metadata: historical TalonFX declarations do not distinguish an X60 from an X44. These two items are not claimed as recovered legacy calibration.

The preparation script is `scripts/prepare-powerlib-config.mjs`. It accepts an extracted copy of the reference commit, the library-test-robot root, and an output directory. It creates JSON first and does not modify Java. Run Update Code only after reviewing its output:

```powershell
node scripts/prepare-powerlib-config.mjs <reference-snapshot> <library-test-robot> <output-directory>
powershell -ExecutionPolicy Bypass -File power-tool/scripts/generate-subsystem.ps1 -UpdateSubsystems
```

Power Tool's installed node_modules is included in the final migration commit at the user's request.

## Validation results

- 351 historical configuration checks passed before generation.
- 205 Java configuration values, four JSON documents, seven subsystem initializations and 451 installed dependency versions passed verification.
- Gradle build passed with WPILib JDK 17. There are no JUnit test sources; the build reports test NO-SOURCE.
- Renderer and Electron TypeScript checks passed.
- Existing CANdle deprecation warning remains.

The entire installed node_modules tree is tracked. The 210,896,896-byte Electron executable is stored with Git LFS; fresh checkouts need Git LFS to restore that binary. Node module files are marked binary to preserve their installed bytes.

To check generated values after another Update Code run:

```powershell
node scripts/verify-powerlib-config.mjs . <verified-json-directory>
```
