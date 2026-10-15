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
  // Startup state of the Drive tab's Shoot on move button; off = original stationary shot, no driving while aiming.
  public static final boolean SOTM_ENABLED_DEFAULT = true;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double SOTM_MAX_DRIVETRAIN_SPEED_MPS = 3.0;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double SOTM_RELEASE_DELAY_SECONDS = 0.02;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double SOTM_DRAG_COEFFICIENT = 0.24;
  // Once feeding, keep feeding while flywheel, hood and heading stay within this multiple of their start tolerances.
  @frc.powerlib.tuning.TunableConstant
  public static volatile double FEED_HOLD_TOLERANCE_SCALE = 1.5;
  // Low-pass time constant on the robot velocity used for the moving-shot lead.
  @frc.powerlib.tuning.TunableConstant
  public static volatile double SOTM_VELOCITY_FILTER_SECONDS = 0.1;
  // Fastest the teleop aim target may turn; lower it if the wheels still splay while aiming.
  @frc.powerlib.tuning.TunableConstant
  public static volatile double AIM_HEADING_SLEW_DEGREES_PER_SECOND = 90;
  public static final double SOTM_MIN_SPEED_MPS = 0.1;
  public static final double SOTM_MAX_TILT_DEGREES = 5.0;
  public static final double LAUNCHER_OFFSET_X_METERS = -0.20;
  public static final double LAUNCHER_OFFSET_Y_METERS = 0.0;
  public static final double SHOOTER_ANGLE_OFFSET_RADIANS = Math.PI;
  // Provisional flight times from SIM-TESTING e2bfadbd; measure again for our retained RPS/hood table.
  public static final double[][] SHOOTER_TOF = {
      {1.68, 0.515}, {1.97, 0.574}, {2.38, 0.658}, {2.75, 0.733}, {3.29, 0.812},
      {3.75, 0.856}, {4.05, 0.884}, {4.60, 0.947}, {4.85, 0.976}, {5.50, 1.052}
  };
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
