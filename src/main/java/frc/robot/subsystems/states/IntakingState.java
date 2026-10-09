package frc.robot.subsystems.states;

import edu.wpi.first.math.MathUtil;
import frc.powerlib.statemachine.State;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.utils.ShootingUtils;

public final class IntakingState implements State<RobotState, StateMachine> {
  @Override
  public boolean match(RobotState requestedState, StateMachine robot) {
    return requestedState == RobotState.INTAKING;
  }

  @Override
  public void execute(RobotState requestedState, StateMachine robot) {
    ShootingUtils.stopShooter(robot);
    ShootingUtils.stopFeeding(robot);
    applyIntake(robot);
    robot.setActualState(RobotState.INTAKING);
  }

  /** Collection can also run alongside a shooting request. */
  public static void applyIntake(StateMachine robot) {
    setWristPosition(robot, Constants.IntakeWrist.INTAKE_MAX);
    robot.intakeRoller.setVelocity(Constants.Shooting.COLLECT_RPS);
  }

  /** All intake wrist demands share these live travel limits. */
  public static void setWristPosition(StateMachine robot, double position) {
    robot.intakeWrist.setPositionRotations(MathUtil.clamp(position,
        Math.min(Constants.IntakeWrist.INTAKE_MIN, Constants.IntakeWrist.INTAKE_MAX),
        Math.max(Constants.IntakeWrist.INTAKE_MIN, Constants.IntakeWrist.INTAKE_MAX)));
  }
}
