package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.powerlib.auto.ChassisMotionLimiter;
import frc.powerlib.configs.MotionConstraints;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.Consumer;

/** April 4 distance control with constrained field-relative translation and rotation requests. */
public final class DriveToPointCommand extends Command {
  private final Swerve drivetrain;
  private final Pose2d destination;
  private final double toleranceMeters;
  private final boolean lockHeading;
  private final Consumer<String> abort;
  private final ChassisMotionLimiter limiter;
  private final ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private final PIDController distanceController = new PIDController(
      Constants.RobotContainer.AUTO_TRANSLATION_KP, 0, Constants.RobotContainer.AUTO_TRANSLATION_KD);
  private final Timer timer = new Timer();
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private double lastTimestamp;
  private boolean failed;

  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, double toleranceInches,
      MotionConstraints constraints, boolean lockHeading, Consumer<String> abort) {
    this.drivetrain = drivetrain;
    this.destination = destination;
    this.toleranceMeters = Units.inchesToMeters(toleranceInches);
    this.lockHeading = lockHeading;
    this.abort = abort;
    limiter = new ChassisMotionLimiter(constraints);
    driveRequest.withDriveRequestType(lockHeading ? DriveRequestType.Velocity : DriveRequestType.OpenLoopVoltage);
    addRequirements(drivetrain);
  }

  @Override public void initialize() {
    failed = false;
    distanceController.reset();
    drivetrain.resetAutonomousHeadingController();
    timer.restart();
    lastTimestamp = Timer.getFPGATimestamp();
    if (!HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      fail("Drivetrain feedback unavailable");
      return;
    }
    var state = drivetrain.getState();
    try {
      limiter.reset(ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation()));
    } catch (IllegalArgumentException exception) {
      fail("Drivetrain velocity unavailable");
    }
  }

  @Override public void execute() {
    if (failed) return;
    if (!HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      fail("Drivetrain feedback unavailable");
      return;
    }
    double now = Timer.getFPGATimestamp();
    double elapsed = now - lastTimestamp;
    if (!Double.isFinite(elapsed) || elapsed < 0) {
      fail("Invalid autonomous control time");
      return;
    }
    if (elapsed == 0) return;
    lastTimestamp = now;
    var pose = drivetrain.getState().Pose;
    var delta = destination.getTranslation().minus(pose.getTranslation());
    double distance = delta.getNorm();
    double feedforward = distance >= Units.inchesToMeters(Constants.Auto.STATIC_FEEDFORWARD_THRESHOLD_INCHES)
        ? Constants.Swerve.DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * drivetrain.MAX_SPEED : 0;
    double speed = Math.abs(distanceController.calculate(distance, 0)) + feedforward;
    desiredSpeeds.vxMetersPerSecond = distance > 0 ? speed * delta.getX() / distance : 0;
    desiredSpeeds.vyMetersPerSecond = distance > 0 ? speed * delta.getY() / distance : 0;
    desiredSpeeds.omegaRadiansPerSecond = lockHeading
        ? drivetrain.calculateAutonomousHeadingRate(pose.getRotation(), destination.getRotation()) : 0;
    try {
      var constrained = limiter.calculate(desiredSpeeds, elapsed);
      drivetrain.applyRequest(driveRequest.withVelocityX(constrained.vxMetersPerSecond)
          .withVelocityY(constrained.vyMetersPerSecond).withRotationalRate(constrained.omegaRadiansPerSecond),
          lockHeading ? destination.getRotation().getDegrees() : null);
    } catch (IllegalArgumentException exception) {
      fail("Invalid autonomous motion request");
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
      fail("Drive-to-point leg timed out");
      return true;
    }
    return false;
  }

  private void fail(String reason) {
    failed = true;
    abort.accept(reason);
    stop();
  }

  private void stop() {
    drivetrain.applyRequest(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override public void end(boolean interrupted) { timer.stop(); stop(); }
}
