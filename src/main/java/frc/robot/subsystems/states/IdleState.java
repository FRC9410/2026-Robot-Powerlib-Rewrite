package frc.robot.subsystems.states;

import frc.powerlib.statemachine.State;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.utils.ShootingUtils;

public final class IdleState implements State<RobotState, StateMachine> {
  @Override
  public boolean match(RobotState requestedState, StateMachine robot) {
    return requestedState == RobotState.IDLE || requestedState == RobotState.READY;
  }

  @Override
  public void execute(RobotState requestedState, StateMachine robot) {
    ShootingUtils.stopShooter(robot);
    ShootingUtils.stopFeeding(robot);
    stopIntake(robot);
    if (requestedState == RobotState.READY) IntakingState.setWristPosition(robot, Constants.IntakeWrist.INTAKE_IDLE);
    if (requestedState == RobotState.IDLE) robot.intakeWrist.setPositionRotations(Constants.IntakeWrist.INTAKE_DEFAULT);
    // READY is the rewrite's standby name, retaining its deployed idle-wrist position.
    robot.setActualState(requestedState == RobotState.READY ? RobotState.READY : RobotState.IDLE);
  }

  public static void stopIntake(StateMachine robot) {
    robot.intakeRoller.stopVelocity();
    robot.intakeRoller.brake();
  }
}
