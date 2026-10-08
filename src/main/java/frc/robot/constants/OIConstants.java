package frc.robot.constants;

public class OIConstants {
  @frc.powerlib.tuning.TunableConstant
  public static volatile int DRIVER_CONTROLLER_PORT = 0;

  @frc.powerlib.tuning.TunableConstant
  public static volatile double DEADBAND = 0.05;

  /** Scales max drive/strafe speed from joystick (0 = stop, 1 = full speed). */
  @frc.powerlib.tuning.TunableConstant
  public static volatile double MAX_SPEED_COEFFICIENT = 0.75;


  // POWERLIB CUSTOM CONSTANTS START - DO NOT DELETE
  @frc.powerlib.tuning.TunableConstant
  public static volatile int OPERATOR_CONTROLLER_PORT = 1;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double INTERCHANGE_SPEED_COEFFICIENT = 0.75;
  // POWERLIB CUSTOM CONSTANTS END - DO NOT DELETE
}






