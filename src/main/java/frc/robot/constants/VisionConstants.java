package frc.robot.constants;

import frc.powerlib.configs.LimelightVisionConfig;

public class VisionConstants {
  public static final String LIMELIGHT_NAME = "limelight-left";
  // First fresh, valid camera wins. Use an empty array for robots without Limelights.
  // Example: {"limelight-b", "limelight-l", "limelight-r"}
  public static final String[] LIMELIGHT_NAMES = {"limelight-left", "limelight-right", "limelight-turret"};
  public static final LimelightVisionConfig CONFIG = LimelightVisionConfig.DEFAULT;

  // POWERLIB CUSTOM CONSTANTS START - DO NOT DELETE
  public static final String CAMERA_NAME = "limelight";
  @frc.powerlib.tuning.TunableConstant
  public static volatile double CAMERA_FOV_DEGREES = 60;
  @frc.powerlib.tuning.TunableConstant
  public static volatile int IMAGE_WIDTH = 320;
  @frc.powerlib.tuning.TunableConstant
  public static volatile int IMAGE_HEIGHT = 240;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double TARGET_AREA_THRESHOLD = 1;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double MAX_TARGET_DISTANCE_METERS = 5;
  public static final String LEFT_TABLE = "limelight-left";
  public static final String RIGHT_TABLE = "limelight-right";
  public static final String TURRET_TABLE = "limelight-turret";
  // POWERLIB CUSTOM CONSTANTS END - DO NOT DELETE
}
