package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;
import frc.robot.autos.AutoPoint;
import frc.robot.subsystems.Swerve;
import java.util.function.Consumer;

/** Original point controller with optional per-leg completion and translation limits. */
public final class DriveToPointCommand extends Command {
  private final Swerve drivetrain;
  private final Pose2d destination;
  private final double toleranceMeters;
  private final double speedCoefficient;
  private final boolean lockHeading;
  private final AutoPoint.Params params;
  private final SwerveRequest.FieldCentricFacingAngle configuredFacing;
  private final Consumer<String> abort;
  private final PIDController distanceController = new PIDController(
      Constants.RobotContainer.AUTO_TRANSLATION_KP, 0, Constants.RobotContainer.AUTO_TRANSLATION_KD);
  private final SwerveRequest.FieldCentric unlocked = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private Translation2d previousVelocity;
  private double lastUpdateSeconds;
  private boolean failed;

  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, double toleranceInches,
      double speedCoefficient, boolean lockHeading, Consumer<String> abort) {
    this(drivetrain, destination, toleranceInches, speedCoefficient, lockHeading, AutoPoint.Params.DEFAULT, abort);
  }

  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, double toleranceInches,
      double speedCoefficient, boolean lockHeading, AutoPoint.Params params, Consumer<String> abort) {
    this.params = params == null ? AutoPoint.Params.DEFAULT : params;
    this.drivetrain = drivetrain;
    this.destination = destination;
    this.toleranceMeters = this.params.endDistanceToleranceMeters() == null
        ? Units.inchesToMeters(toleranceInches) : this.params.endDistanceToleranceMeters();
    this.speedCoefficient = speedCoefficient;
    this.lockHeading = lockHeading;
    this.abort = abort;
    if (this.params.equals(AutoPoint.Params.DEFAULT)) {
      configuredFacing = null;
    } else {
      // Joystick deadbands would discard slow requests and turn acceleration ramps into steps.
      // A private request lets configured legs use small velocities without changing driver requests.
      configuredFacing = new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(0).withDriveRequestType(drivetrain.DRIVE_AT_ANGLE.DriveRequestType);
      configuredFacing.HeadingController = drivetrain.DRIVE_AT_ANGLE.HeadingController;
    }
    addRequirements(drivetrain);
  }

  @Override public void initialize() {
    failed = false;
    distanceController.reset();
    previousVelocity = null;
    if (params.maxAccelerationMetersPerSecondSquared() != null) lastUpdateSeconds = Timer.getFPGATimestamp();
  }

  @Override public void execute() {
    if (!HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      failed = true;
      abort.accept("Drivetrain feedback unavailable");
      stop();
      return;
    }
    var state = drivetrain.getState();
    if (params.maxAccelerationMetersPerSecondSquared() != null && !finiteTranslation(state.Speeds)) {
      failed = true;
      abort.accept("Drivetrain velocity unavailable for acceleration limiting");
      stop();
      return;
    }
    var pose = state.Pose;
    var delta = destination.getTranslation().minus(pose.getTranslation());
    double distance = delta.getNorm();
    double feedforward = distance >= Units.inchesToMeters(Constants.Auto.STATIC_FEEDFORWARD_THRESHOLD_INCHES)
        ? Constants.Swerve.DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * drivetrain.MAX_SPEED : 0;
    double maximumSpeed = speedLimit(drivetrain.MAX_SPEED,
        Constants.Swerve.DRIVE_TO_POINT_MAX_SPEED_COEFFICIENT, speedCoefficient, params.maxSpeedMetersPerSecond());
    double speed = Math.min(Math.abs(distanceController.calculate(distance, 0)) + feedforward, maximumSpeed);
    double vx = distance > 0 ? speed * delta.getX() / distance : 0;
    double vy = distance > 0 ? speed * delta.getY() / distance : 0;
    if (params.maxAccelerationMetersPerSecondSquared() != null) {
      double now = Timer.getFPGATimestamp();
      if (previousVelocity == null) {
        // Seed in the field frame, preserving actual motion when a new leg begins.
        previousVelocity = new Translation2d(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond)
            .rotateBy(pose.getRotation());
      }
      previousVelocity = limitVelocity(previousVelocity, new Translation2d(vx, vy),
          params.maxAccelerationMetersPerSecondSquared(), now-lastUpdateSeconds);
      lastUpdateSeconds = now;
      vx = previousVelocity.getX();
      vy = previousVelocity.getY();
    }
    if (lockHeading) {
      var facing = configuredFacing == null ? drivetrain.DRIVE_AT_ANGLE
          : configuredFacing.withRotationalDeadband(drivetrain.DRIVE_AT_ANGLE.RotationalDeadband);
      drivetrain.applyRequest(facing
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
    var state = drivetrain.getState();
    return atDestination(state.Pose, destination, toleranceMeters, lockHeading,
        state.Speeds, params.endSpeedToleranceMetersPerSecond());
  }

  static boolean atDestination(Pose2d pose, Pose2d destination, double toleranceMeters,
      boolean lockHeading, ChassisSpeeds speeds, Double endSpeedTolerance) {
    return HealthChecks.finitePose(pose)
        && pose.getTranslation().getDistance(destination.getTranslation()) < toleranceMeters
        && (!lockHeading || Math.abs(pose.getRotation().minus(destination.getRotation()).getDegrees())
            < Constants.Auto.LEG_HEADING_TOLERANCE_DEGREES)
        && (endSpeedTolerance == null || finiteTranslation(speeds)
            && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < endSpeedTolerance);
  }

  private static boolean finiteTranslation(ChassisSpeeds speeds) {
    return speeds != null && Double.isFinite(speeds.vxMetersPerSecond) && Double.isFinite(speeds.vyMetersPerSecond);
  }

  /** Explicit speed replaces the leg coefficient while retaining the robot/global ceiling. */
  static double speedLimit(double robotMaximum, double globalCoefficient, double legCoefficient, Double maximum) {
    double ceiling = robotMaximum * globalCoefficient;
    return maximum == null ? ceiling * legCoefficient : Math.min(Math.min(robotMaximum, ceiling), maximum);
  }

  /** Limit the vector change, including turns and braking. Existing overspeed decays at the configured rate. */
  static Translation2d limitVelocity(Translation2d previous, Translation2d target, double acceleration, double dt) {
    var change = target.minus(previous);
    double maximumChange = acceleration * Math.max(0, dt);
    return change.getNorm() > maximumChange
        ? previous.plus(change.times(maximumChange/change.getNorm())) : target;
  }

  // Interrupts, failed feedback, and normal completion retain the original immediate stop.
  private void stop() { drivetrain.applyRequest(unlocked.withVelocityX(0).withVelocityY(0).withRotationalRate(0)); }
  @Override public void end(boolean interrupted) { stop(); }
}
