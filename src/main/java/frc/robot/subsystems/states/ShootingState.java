package frc.robot.subsystems.states;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.powerlib.health.HealthChecks;
import frc.powerlib.statemachine.State;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.utils.ShootingUtils;
import frc.robot.utils.ShootingUtils.ShotSolution;

public final class ShootingState implements State<RobotState, StateMachine> {
  private ShotSolution shot = ShootingUtils.UNAVAILABLE;
  private double shootingStarted = Double.NaN;
  private double shootingElapsed = Double.NaN;

  @Override
  public void prepare(StateMachine robot) {
    if (robot.getCurrentState() != RobotState.SHOOTING
        && robot.getCurrentState() != RobotState.SPINNING_UP) resetTimer();
    if (!robot.hasRequest(RobotState.SHOOTING) && !robot.hasRequest(RobotState.SPINNING_UP)) {
      reset();
      return;
    }
    shot = ShootingUtils.calculate(robot.drivetrain.getState().Pose,
        DriverStation.getAlliance().orElse(null), robot.shooter.inputs.velocityRotationsPerSecond,
        robot.shooterHood.inputs.positionRotations, feedbackHealthy(robot));
    if (!shot.valid()) resetTimer();
  }

  @Override
  public boolean match(RobotState requestedState, StateMachine robot) {
    return requestedState == RobotState.SHOOTING && shot.canFeed();
  }

  @Override
  public void execute(RobotState requestedState, StateMachine robot) {
    applyShot(requestedState, robot);
    // Recheck even when selection is skipped because requested and current state agree.
    if (!match(requestedState, robot)) {
      ShootingUtils.stopFeeding(robot);
      robot.setActualState(RobotState.SPINNING_UP);
      return;
    }
    robot.feeder.setVelocity(shot.feederRps());
    robot.spindexer.setVelocity(Constants.StateMachine.SPINDEXER_SHOOT_RPS);
    robot.setActualState(RobotState.SHOOTING);
  }

  /** Spin-up uses the same prepared shot and intake behavior. */
  void applyShot(RobotState requestedState, StateMachine robot) {
    if (requestedState == RobotState.SHOOTING && shot.valid()) {
      double now = Timer.getFPGATimestamp();
      if (!Double.isFinite(shootingStarted) || now < shootingStarted) shootingStarted = now;
      shootingElapsed = now - shootingStarted;
    } else {
      resetTimer();
    }
    ShootingUtils.applyShooter(robot, shot);
    applyIntake(robot);
  }

  @Override
  public void reset() {
    shot = ShootingUtils.UNAVAILABLE;
    resetTimer();
  }

  private void resetTimer() {
    shootingStarted = shootingElapsed = Double.NaN;
  }

  private boolean feedbackHealthy(StateMachine robot) {
    double maxAge = Constants.Shooting.MAX_SIGNAL_AGE_SECONDS;
    return HealthChecks.driveHealthy(robot.drivetrain, maxAge)
        && HealthChecks.mechanismHealthy(robot.shooter, robot.shooter.getVelocityMotor(), maxAge)
        && HealthChecks.mechanismHealthy(robot.shooterHood, robot.shooterHood.getPositionMotor(), maxAge)
        && HealthChecks.mechanismHealthy(robot.feeder, robot.feeder.getVelocityMotor(), maxAge)
        && HealthChecks.mechanismHealthy(robot.spindexer, robot.spindexer.getVelocityMotor(), maxAge)
        && HealthChecks.mechanismHealthy(robot.intakeWrist, robot.intakeWrist.getPositionMotor(), maxAge);
  }

  private void applyIntake(StateMachine robot) {
    if (robot.hasRequest(RobotState.INTAKING)) {
      IntakingState.applyIntake(robot);
      return;
    }
    IdleState.stopIntake(robot);
    double position = Constants.IntakeWrist.INTAKE_IDLE;
    // Pulse for half a second each second, starting after one second of shooting.
    if (Double.isFinite(shootingElapsed) && shootingElapsed >= 1 && shootingElapsed % 1 < 0.5) {
      position = Constants.IntakeWrist.INTAKE_FEED;
    }
    IntakingState.setWristPosition(robot, position);
  }
}