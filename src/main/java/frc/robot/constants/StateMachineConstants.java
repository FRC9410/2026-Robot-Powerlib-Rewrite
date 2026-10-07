package frc.robot.constants;

import frc.robot.subsystems.StateMachine.RobotState;

public class StateMachineConstants {
  public static final RobotState DEFAULT_STATE = RobotState.READY;

  // POWERLIB CUSTOM CONSTANTS START - DO NOT DELETE
  @frc.powerlib.tuning.TunableConstant
  public static volatile double HOOD_TOLERANCE_ROTATIONS = 0.005;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double SHOOTER_EXTRA_RPS = 1.0;
  @frc.powerlib.tuning.TunableConstant
  public static volatile double SPINDEXER_SHOOT_RPS = 60.0;
  // POWERLIB CUSTOM CONSTANTS END - DO NOT DELETE
}
