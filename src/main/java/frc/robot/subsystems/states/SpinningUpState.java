package frc.robot.subsystems.states;

import frc.powerlib.statemachine.State;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.utils.ShootingUtils;

public final class SpinningUpState implements State<RobotState, StateMachine> {
  private final ShootingState shootingState;

  public SpinningUpState(ShootingState shootingState) {
    this.shootingState = java.util.Objects.requireNonNull(shootingState);
  }

  @Override
  public boolean match(RobotState requestedState, StateMachine robot) {
    return requestedState == RobotState.SPINNING_UP || requestedState == RobotState.SHOOTING;
  }

  @Override
  public void execute(RobotState requestedState, StateMachine robot) {
    ShootingUtils.stopFeeding(robot);
    shootingState.applyShot(requestedState, robot);
    robot.setActualState(RobotState.SPINNING_UP);
  }
}
