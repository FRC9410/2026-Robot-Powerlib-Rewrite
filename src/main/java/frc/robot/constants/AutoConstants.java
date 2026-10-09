package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {
  public static final double TRANSLATION_TOLERANCE = 0.05;
  public static final double ROTATION_TOLERANCE = 1.0;

  // ---- Red left (driver station left) ----
  public static final Pose2d RED_LEFT_1 = new Pose2d(13.5, 2.4, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d RED_LEFT_2 = new Pose2d(10.0, 2.4, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d RED_LEFT_3 = new Pose2d(9.55, 0.73, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d RED_LEFT_4 = new Pose2d(8.2, 1, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d RED_LEFT_5 = new Pose2d(8.2, 3.5, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d RED_LEFT_6 = new Pose2d(8.4, 2.4, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d RED_LEFT_7 = new Pose2d(14.3, 2.4, Rotation2d.fromDegrees(135.0));

  // ---- Red right (driver station right) ----
  public static final Pose2d RED_RIGHT_1 = new Pose2d(13.5, 5.7, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_RIGHT_2 = new Pose2d(10.0, 5.7, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_RIGHT_3 = new Pose2d(9.55, 7.27, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_RIGHT_4 = new Pose2d(8.2, 7, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_RIGHT_5 = new Pose2d(8.2, 4.5, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_RIGHT_6 = new Pose2d(8.4, 5.7, Rotation2d.fromDegrees(90.0));
  public static final Pose2d RED_RIGHT_7 = new Pose2d(14.3, 5.7, Rotation2d.fromDegrees(-135.0));

  // ---- Blue left (driver station left) ----
  public static final Pose2d BLUE_LEFT_1 = new Pose2d(3.1, 5.7, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d BLUE_LEFT_2 = new Pose2d(6.5, 5.7, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d BLUE_LEFT_3 = new Pose2d(6.95, 7.27, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d BLUE_LEFT_4 = new Pose2d(8.2, 7, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d BLUE_LEFT_5 = new Pose2d(8.2, 4.5, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d BLUE_LEFT_6 = new Pose2d(8.1, 5.7, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d BLUE_LEFT_7 = new Pose2d(2.1, 5.7, Rotation2d.fromDegrees(135.0));

  // ---- Blue right (driver station right) ----
  public static final Pose2d BLUE_RIGHT_1 = new Pose2d(3.1, 2.4, Rotation2d.fromDegrees(90.0));
  public static final Pose2d BLUE_RIGHT_2 = new Pose2d(6.5, 2.4, Rotation2d.fromDegrees(90.0));
  public static final Pose2d BLUE_RIGHT_3 = new Pose2d(6.95, 0.73, Rotation2d.fromDegrees(90.0));
  public static final Pose2d BLUE_RIGHT_4 = new Pose2d(8.2, 1, Rotation2d.fromDegrees(90.0));
  public static final Pose2d BLUE_RIGHT_5 = new Pose2d(8.2, 3.5, Rotation2d.fromDegrees(90.0));
  public static final Pose2d BLUE_RIGHT_6 = new Pose2d(8.1, 2.4, Rotation2d.fromDegrees(90.0));
  public static final Pose2d BLUE_RIGHT_7 = new Pose2d(2.1, 2.4, Rotation2d.fromDegrees(-135.0));

  // Per-leg settings from April 4 RobotContainer/SwerveDriveCommand.
  public static final double[] LEG_TOLERANCE_INCHES = {6, 6, 12, 6, 6, 3, 3};
  public static final double[] LEG_SPEED_COEFFICIENTS = {0.5, 1, 0.75, 0.75, 0.4, 0.75, 1};
  public static final boolean[] LEG_LOCK_HEADING = {true, false, true, true, true, true, false};
  public static final double LEG_HEADING_TOLERANCE_DEGREES = 10;
  public static final double STATIC_FEEDFORWARD_THRESHOLD_INCHES = 3;
  public static final double TURN_TOLERANCE_DEGREES = 3;
  public static final double TURN_TIMEOUT_SECONDS = 1;
}
