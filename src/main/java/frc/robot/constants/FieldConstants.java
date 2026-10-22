package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
  @frc.powerlib.tuning.TunableConstant
  public static volatile double X_MIN = 0.0;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double Y_MIN = 0.0;
  // WPILib 2026 REBUILT welded field layout, in the blue-origin coordinate frame.
  @frc.powerlib.tuning.TunableConstant
  public static volatile double X_MAX = 16.541;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double Y_MAX = 8.069;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double TOL = 2.0;

  @frc.powerlib.tuning.TunableConstant
  public static volatile double BLUE_START_X = 0.0;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double BLUE_END_X = 4;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double CENTER_START_X = 5.2;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double CENTER_END_X = 11.35;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double RED_START_X = 12.55;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double RED_END_X = X_MAX;

  @frc.powerlib.tuning.TunableConstant
  public static volatile double AUTO_LENGTH_IN_TIME = 15;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double TIME_BETWEEN_SHIFTS = 25;

  public static final Translation3d BLUE_TOP_CORNER = new Translation3d(0.5, 7.5, 0.0);
  public static final Translation3d BLUE_BOTTOM_CORNER = new Translation3d(0.5, 0.5, 0.0);

  public static final Translation3d RED_TOP_CORNER = new Translation3d(16, 7.5, 0.0);
  public static final Translation3d RED_BOTTOM_CORNER = new Translation3d(16, 0.5, 0.0);

  public static final Translation2d NEU_TOP_LEFT = new Translation2d(5.872, 7.443);
  public static final Translation2d NEU_TOP_RIGHT = new Translation2d(10.692, 7.443);
  public static final Translation2d NEU_BOTTOM_LEFT = new Translation2d(5.872, 0.595);
  public static final Translation2d NEU_BOTTOM_RIGHT = new Translation2d(10.692, 0.595);

  // Hub centers are the midpoints of opposing hub-face tags (4/10 red, 20/26 blue).
  public static final Translation2d HOPPER_RED = new Translation2d(11.915521, 4.0346376);
  public static final Translation2d HOPPER_BLUE = new Translation2d(4.6255178, 4.0346376);

  @frc.powerlib.tuning.TunableConstant
  public static volatile double MAX_DISTANCE_TO_HOPPER = 5.406;
  // POWERLIB CUSTOM CONSTANTS START - DO NOT DELETE

  // POWERLIB CUSTOM CONSTANTS END - DO NOT DELETE
}
