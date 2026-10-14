package frc.robot.constants;

/** April 4 calibration, extended to the 1.95 m minimum shooting distance. */
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
  public static final double MIN_SHOT_DISTANCE = 1.95;
  public static final double MAX_SHOT_DISTANCE = 4.9;
  public static final double COLLECT_RPS = 145;
  public static final double EJECT_RPS = -145;
  // Readiness/freshness guards added by the rewrite; these are not April 4 calibrations.
  public static final double SHOOTER_TOLERANCE_RPS = 2;
  public static final double HEADING_TOLERANCE_DEGREES = 5;
  public static final double MAX_SIGNAL_AGE_SECONDS = 0.5;

  // POWERLIB CUSTOM CONSTANTS START - DO NOT DELETE
  @frc.powerlib.tuning.TunableConstant
  public static volatile double SHOOT_MAX_DRIVETRAIN_SPEED_MPS = 0.5;
  public static final double[][] HOOD_ANGLES = {
            { 4.9, 0.09 },
            { 4.1, 0.08 },
            { 3.5, 0.07 },
            { 3.0, 0.065 },
            { 2.5, 0.055 },
            { 1.95, 0.044 }
    };
  public static final double[][] SHOOTER_SPEEDS = {
             { 4.9, 37 },
             { 4.1, 34 },
             { 3.5, 32 },
             { 3.0, 29.5 },
             { 2.5, 28.5 },
             { 1.95, 27.4 }
    };
  public static final double[][] FEEDER_SPEEDS = {
        { 4.9, 80 },
        { 4.1, 85 },
        { 3.5, 85 },
        { 3.0, 85 },
        { 2.5, 85 },
    };
  public static final frc.powerlib.math.LinearInterpolator HOOD_ANGLE_INTERPOLATOR = new frc.powerlib.math.LinearInterpolator(HOOD_ANGLES);
  public static final frc.powerlib.math.LinearInterpolator SHOOTER_VELOCITY_INTERPOLATOR = new frc.powerlib.math.LinearInterpolator(SHOOTER_SPEEDS);
  public static final frc.powerlib.math.LinearInterpolator FEEDER_VELOCITY_INTERPOLATOR = new frc.powerlib.math.LinearInterpolator(FEEDER_SPEEDS);
  // POWERLIB CUSTOM CONSTANTS END - DO NOT DELETE
}
