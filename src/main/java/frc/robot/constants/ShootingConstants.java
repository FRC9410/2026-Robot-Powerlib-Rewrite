package frc.robot.constants;

/** Shot calibration from 2026-Robot's April 4 commit 0f75d1b. */
public class ShootingConstants {
  protected ShootingConstants() {}

  public static final edu.wpi.first.math.geometry.Translation2d BLUE_HUB = FieldConstants.HOPPER_BLUE;
  public static final edu.wpi.first.math.geometry.Translation2d RED_HUB = FieldConstants.HOPPER_RED;
  public static final double FIELD_WIDTH = FieldConstants.Y_MAX;
  public static final double BLUE_ZONE_START = FieldConstants.BLUE_START_X;
  public static final double BLUE_ZONE_END = FieldConstants.BLUE_END_X;
  public static final double RED_ZONE_START = FieldConstants.RED_START_X;
  public static final double RED_ZONE_END = FieldConstants.RED_END_X;
  public static final double CENTER_ZONE_START = FieldConstants.CENTER_START_X;
  public static final double CENTER_ZONE_END = FieldConstants.CENTER_END_X;
  public static final double MIN_SHOT_DISTANCE = 2.5;
  public static final double MAX_SHOT_DISTANCE = 4.9;
  public static final double COLLECT_RPS = 145;
  public static final double EJECT_RPS = -145;
  // Readiness/freshness guards added by the rewrite; these are not April 4 calibrations.
  public static final double SHOOTER_TOLERANCE_RPS = 2;
  public static final double HEADING_TOLERANCE_DEGREES = 5;
  public static final double MAX_SIGNAL_AGE_SECONDS = 0.5;
}
