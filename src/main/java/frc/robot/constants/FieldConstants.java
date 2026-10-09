package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
  public static final double X_MIN = 0.0;
  public static final double Y_MIN = 0.0;
  public static final double X_MAX = 17.5;
  public static final double Y_MAX = 8.0;
  public static final double TOL = 2.0;

  public static final double BLUE_START_X = 0.0;
  public static final double BLUE_END_X = 4;
  public static final double CENTER_START_X = 5.2;
  public static final double CENTER_END_X = 11.35;
  public static final double RED_START_X = 12.55;
  public static final double RED_END_X = 16.5;

  public static final double AUTO_LENGTH_IN_TIME = 15;
  public static final double TIME_BETWEEN_SHIFTS = 25;

  public static final Translation3d BLUE_TOP_CORNER = new Translation3d(0.5, 7.5, 0.0);
  public static final Translation3d BLUE_BOTTOM_CORNER = new Translation3d(0.5, 0.5, 0.0);

  public static final Translation3d RED_TOP_CORNER = new Translation3d(16, 7.5, 0.0);
  public static final Translation3d RED_BOTTOM_CORNER = new Translation3d(16, 0.5, 0.0);

  public static final Translation2d NEU_TOP_LEFT = new Translation2d(5.872, 7.443);
  public static final Translation2d NEU_TOP_RIGHT = new Translation2d(10.692, 7.443);
  public static final Translation2d NEU_BOTTOM_LEFT = new Translation2d(5.872, 0.595);
  public static final Translation2d NEU_BOTTOM_RIGHT = new Translation2d(10.692, 0.595);

  public static final Translation2d HOPPER_RED = new Translation2d(11.5,4);
  public static final Translation2d HOPPER_BLUE = new Translation2d(4.62,4);

  public static final double MAX_DISTANCE_TO_HOPPER =  5.406;
}
