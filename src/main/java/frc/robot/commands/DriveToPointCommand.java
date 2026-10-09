package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.Consumer;

/** April 4 distance controller and per-leg limits, expressed in the blue-field frame. */
public final class DriveToPointCommand extends Command {
  private final Swerve drivetrain;
  private final Pose2d destination;
  private final double toleranceMeters;
  private final double speedCoefficient;
  private final boolean lockHeading;
  private final Consumer<String> abort;
  private final PIDController distanceController = new PIDController(
      Constants.RobotContainer.AUTO_TRANSLATION_KP, 0, Constants.RobotContainer.AUTO_TRANSLATION_KD);
  private final Timer timer = new Timer();
  private final SwerveRequest.FieldCentric unlocked = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private boolean failed;

  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, double toleranceInches,
      double speedCoefficient, boolean lockHeading, Consumer<String> abort) {
    this.drivetrain = drivetrain;
    this.destination = destination;
    this.toleranceMeters = Units.inchesToMeters(toleranceInches);
    this.speedCoefficient = speedCoefficient;
    this.lockHeading = lockHeading;
    this.abort = abort;
    addRequirements(drivetrain);
  }

  @Override public void initialize() {
    failed = false;
    distanceController.reset();
    timer.restart();
  }

  @Override public void execute() {
    if (!HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      failed = true;
      abort.accept("Drivetrain feedback unavailable");
      stop();
      return;
    }
    var pose = drivetrain.getState().Pose;
    var delta = destination.getTranslation().minus(pose.getTranslation());
    double distance = delta.getNorm();
    double feedforward = distance >= Units.inchesToMeters(Constants.Auto.STATIC_FEEDFORWARD_THRESHOLD_INCHES)
        ? Constants.Swerve.DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * drivetrain.MAX_SPEED : 0;
    double speed = Math.min(Math.abs(distanceController.calculate(distance, 0)) + feedforward,
        drivetrain.MAX_SPEED * Constants.Swerve.DRIVE_TO_POINT_MAX_SPEED_COEFFICIENT * speedCoefficient);
    double vx = distance > 0 ? speed * delta.getX() / distance : 0;
    double vy = distance > 0 ? speed * delta.getY() / distance : 0;
    if (lockHeading) {
      drivetrain.applyRequest(drivetrain.DRIVE_AT_ANGLE
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withVelocityX(vx).withVelocityY(vy)
          .withTargetDirection(destination.getRotation())
          .withMaxAbsRotationalRate(drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE),
          destination.getRotation().getDegrees());
    } else {
      drivetrain.applyRequest(unlocked.withVelocityX(vx).withVelocityY(vy).withRotationalRate(0));
    }
  }

  @Override public boolean isFinished() {
    if (failed) return true;
    var pose = drivetrain.getState().Pose;
    boolean reached = HealthChecks.finitePose(pose)
        && pose.getTranslation().getDistance(destination.getTranslation()) < toleranceMeters
        && (!lockHeading || Math.abs(pose.getRotation().minus(destination.getRotation()).getDegrees())
            < Constants.Auto.LEG_HEADING_TOLERANCE_DEGREES);
    if (reached) return true;
    if (timer.hasElapsed(Constants.RobotContainer.AUTO_LEG_TIMEOUT_SECONDS)) {
      failed = true;
      abort.accept("Drive-to-point leg timed out");
      return true;
    }
    return false;
  }

  private void stop() { drivetrain.applyRequest(unlocked.withVelocityX(0).withVelocityY(0).withRotationalRate(0)); }
  @Override public void end(boolean interrupted) { timer.stop(); stop(); }
}
