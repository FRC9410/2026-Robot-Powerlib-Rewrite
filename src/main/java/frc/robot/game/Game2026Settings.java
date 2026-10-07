package frc.robot.game;

import frc.robot.Constants;
import frc.powerlib.PowerRobotContainer;

/** Live behavior settings; scaffold-only defaults are documented in GAME2026-SCAFFOLD.md. */
public record Game2026Settings(double hoodTolerance, double shooterExtraRps, double spindexerRps,
    double headingKp, double autoStartMeters, double autoStartDegrees, double autoLegSeconds,
    double autoTranslationKp, double autoTranslationKd) {
  public boolean valid() {
    return Double.isFinite(hoodTolerance) && hoodTolerance > 0 && Double.isFinite(shooterExtraRps) && shooterExtraRps >= 0
        && Double.isFinite(spindexerRps) && spindexerRps > 0 && Double.isFinite(headingKp) && headingKp > 0
        && Double.isFinite(autoStartMeters) && autoStartMeters >= 0 && Double.isFinite(autoStartDegrees) && autoStartDegrees >= 0
        && Double.isFinite(autoLegSeconds) && autoLegSeconds > 0 && Double.isFinite(autoTranslationKp) && autoTranslationKp > 0
        && Double.isFinite(autoTranslationKd) && autoTranslationKd >= 0;
  }
  public static Game2026Settings current() {
    return new Game2026Settings(Constants.StateMachine.HOOD_TOLERANCE_ROTATIONS,
        Constants.StateMachine.SHOOTER_EXTRA_RPS, Constants.StateMachine.SPINDEXER_SHOOT_RPS,
        PowerRobotContainer.getSubsystemVariable("Swerve", "Heading/kP", Constants.Swerve.HEADING_KP), Constants.RobotContainer.AUTO_START_TOLERANCE_METERS,
        Constants.RobotContainer.AUTO_START_TOLERANCE_DEGREES, Constants.RobotContainer.AUTO_LEG_TIMEOUT_SECONDS,
        Constants.RobotContainer.AUTO_TRANSLATION_KP, Constants.RobotContainer.AUTO_TRANSLATION_KD);
  }
}
