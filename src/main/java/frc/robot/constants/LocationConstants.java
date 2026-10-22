package frc.robot.constants;


public class LocationConstants {
    @frc.powerlib.tuning.TunableConstant
  public static volatile double PLEASE_DONT_HIT_THE_WALL_OFFSET = 0.5;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double BLUE_ALLIANCE_WALL_X = 0.785 + PLEASE_DONT_HIT_THE_WALL_OFFSET;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double BLUE_HUB_WALL_X = 3.15 - PLEASE_DONT_HIT_THE_WALL_OFFSET;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double BLUE_CENTER_WALL_X = 6.0 + PLEASE_DONT_HIT_THE_WALL_OFFSET;

    @frc.powerlib.tuning.TunableConstant
  public static volatile double RED_ALLIANCE_WALL_X = 15.73 - PLEASE_DONT_HIT_THE_WALL_OFFSET;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double RED_HUB_WALL_X = 13.36 + PLEASE_DONT_HIT_THE_WALL_OFFSET;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double RED_CENTER_WALL_X = 10.5 - PLEASE_DONT_HIT_THE_WALL_OFFSET;

    @frc.powerlib.tuning.TunableConstant
  public static volatile double NEAR_WALL_Y = 0.82 + PLEASE_DONT_HIT_THE_WALL_OFFSET;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double FAR_WALL_Y = 7.25 - PLEASE_DONT_HIT_THE_WALL_OFFSET;

    @frc.powerlib.tuning.TunableConstant
  public static volatile double X_MIN_ = 0.0;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double X_MAX = 0.0;

    @frc.powerlib.tuning.TunableConstant
  public static volatile double Y_MIN = 0.0;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double Y_MAX = 0.0;

    @frc.powerlib.tuning.TunableConstant
  public static volatile double BLUE_ZONE_X_MID = 2.0;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double RED_ZONE_X_MID = 14.54;
    @frc.powerlib.tuning.TunableConstant
  public static volatile double NEUTRAL_X_MID = 8.27;
  // POWERLIB CUSTOM CONSTANTS START - DO NOT DELETE

  // POWERLIB CUSTOM CONSTANTS END - DO NOT DELETE
}
