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
  private final frc.robot.utils.MovingShotCalculator fireControl = new frc.robot.utils.MovingShotCalculator();

  public ShotSolution getShotSolution() { return shot; }
  private double shootingStarted = Double.NaN;
  private double shootingElapsed = Double.NaN;
  private boolean feedAllowed;

  @Override
  public void prepare(StateMachine robot) {
    if (robot.getCurrentState() != RobotState.SHOOTING
        && robot.getCurrentState() != RobotState.SPINNING_UP) resetTimer();
    if (!robot.hasRequest(RobotState.SHOOTING) && !robot.hasRequest(RobotState.SPINNING_UP)) {
      reset();
      return;
    }
    var drive = robot.drivetrain.getState();
    shot = fireControl.calculate(drive.Pose, drive.Speeds,
        DriverStation.getAlliance().orElse(null), robot.shooter.inputs.velocityRotationsPerSecond,
        robot.shooterHood.inputs.positionRotations, feedbackHealthy(robot), robot.vision.hasFreshMeasurement(),
        robot.drivetrain.getPigeon2().getPitch().getValueAsDouble(),
        robot.drivetrain.getPigeon2().getRoll().getValueAsDouble());
    if (!shot.valid()) resetTimer();
    // Start on the normal readiness checks; once feeding, ride through small dips so
    // moving-shot jitter or a ball passing the flywheel does not brake the spindexer.
    feedAllowed = shot.canFeed()
        || (robot.getCurrentState() == RobotState.SHOOTING && withinHoldTolerances(robot));
  }

  @Override
  public boolean match(RobotState requestedState, StateMachine robot) {
    return requestedState == RobotState.SHOOTING && feedAllowed;
  }

  private boolean withinHoldTolerances(StateMachine robot) {
    double scale = Constants.Shooting.FEED_HOLD_TOLERANCE_SCALE;
    if (!shot.valid() || !shot.feedbackHealthy() || !Double.isFinite(scale) || scale < 1) return false;
    return Math.abs(robot.shooter.inputs.velocityRotationsPerSecond - shot.shooterRps())
            <= Constants.Shooting.SHOOTER_TOLERANCE_RPS * scale
        && Math.abs(robot.shooterHood.inputs.positionRotations - shot.hoodRotations())
            <= Constants.StateMachine.HOOD_TOLERANCE_ROTATIONS * scale
        && Math.abs(shot.headingErrorDegrees()) <= Constants.Shooting.HEADING_TOLERANCE_DEGREES * scale;
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
    feedAllowed = false;
    fireControl.reset();
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
