package frc.robot.subsystems.states;

import frc.powerlib.statemachine.State;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.utils.ShootingUtils;

public final class EjectingState implements State<RobotState, StateMachine> {
  @Override
  public boolean match(RobotState requestedState, StateMachine robot) {
    return requestedState == RobotState.EJECTING;
  }

  @Override
  public void execute(RobotState requestedState, StateMachine robot) {
    ShootingUtils.stopShooter(robot);
    ShootingUtils.stopFeeding(robot);
    IntakingState.setWristPosition(robot, Constants.IntakeWrist.INTAKE_MAX);
    robot.intakeRoller.setVelocity(Constants.Shooting.EJECT_RPS);
    robot.setActualState(RobotState.EJECTING);
  }
}
