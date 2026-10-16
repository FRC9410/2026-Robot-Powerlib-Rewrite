package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.powerlib.auto.ChassisMotionLimiter;
import frc.powerlib.auto.CompletionChecker;
import frc.powerlib.auto.DistanceSpeedController;
import frc.powerlib.auto.PathDistance;
import frc.powerlib.configs.CompletionCriteria;
import frc.powerlib.configs.MotionConstraints;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;

/** Constrained point control that finishes only after measured motion has settled at the goal. */
public final class DriveToPointCommand extends Command {
  private final Swerve drivetrain;
  private final Pose2d destination;
  private final CompletionCriteria criteria;
  private final MotionConstraints constraints;
  private final CompletionChecker completion;
  private final double timeoutSeconds;
  private final boolean lockHeading;
  private final Consumer<String> abort;
  private final ChassisMotionLimiter limiter;
  private final ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private final PathDistance pathDistance;
  private final DistanceSpeedController distanceController = new DistanceSpeedController(
      new PIDController(Constants.RobotContainer.AUTO_TRANSLATION_KP, 0, Constants.RobotContainer.AUTO_TRANSLATION_KD),
      Units.inchesToMeters(Constants.Auto.STATIC_FEEDFORWARD_THRESHOLD_INCHES));
  private final Timer timer = new Timer();
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private Function<Pose2d, Rotation2d> headingTarget;
  private double lastTimestamp;
  private boolean failed;
  private boolean finished;

  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, double toleranceInches,
      MotionConstraints constraints, boolean lockHeading, Consumer<String> abort) {
    this(drivetrain, destination, Constants.Auto.completionCriteria(toleranceInches,
        Constants.Auto.LEG_HEADING_TOLERANCE_DEGREES), constraints, lockHeading,
        Constants.RobotContainer.AUTO_LEG_TIMEOUT_SECONDS, abort);
  }

  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, CompletionCriteria criteria,
      MotionConstraints constraints, boolean lockHeading, double timeoutSeconds, Consumer<String> abort) {
    this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain");
    this.destination = Objects.requireNonNull(destination, "destination");
    this.criteria = Objects.requireNonNull(criteria, "criteria");
    this.constraints = Objects.requireNonNull(constraints, "constraints");
    if (!HealthChecks.finitePose(destination)) throw new IllegalArgumentException("destination must be finite");
    if (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0) {
      throw new IllegalArgumentException("timeoutSeconds must be positive and finite");
    }
    this.timeoutSeconds = timeoutSeconds;
    this.lockHeading = lockHeading;
    this.abort = Objects.requireNonNull(abort, "abort");
    completion = new CompletionChecker(criteria);
    limiter = new ChassisMotionLimiter(constraints);
    pathDistance = new PathDistance(List.of(destination.getTranslation()));
    driveRequest.withDriveRequestType(lockHeading ? DriveRequestType.Velocity : DriveRequestType.OpenLoopVoltage);
    addRequirements(drivetrain);
  }

  /** Configure before scheduling, for goals such as aiming at the hub from the current pose. */
  public DriveToPointCommand withHeadingTarget(Function<Pose2d, Rotation2d> target) {
    headingTarget = Objects.requireNonNull(target, "target");
    return this;
  }

  @Override public void initialize() {
    failed = false;
    finished = false;
    completion.reset();
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
    if (failed || finished) return;
    if (!HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      fail("Drivetrain feedback unavailable");
      return;
    }
    if (timer.hasElapsed(timeoutSeconds)) {
      fail("Drive-to-point leg timed out");
      return;
    }
    double now = Timer.getFPGATimestamp();
    double elapsed = now - lastTimestamp;
    if (!Double.isFinite(elapsed) || elapsed < 0) {
      fail("Invalid autonomous control time");
      return;
    }
    lastTimestamp = now;
    var state = drivetrain.getState();
    var pose = state.Pose;
    var heading = headingTarget == null ? destination.getRotation() : headingTarget.apply(pose);
    if (heading == null || !Double.isFinite(heading.getRadians())) {
      fail("Autonomous heading target unavailable");
      return;
    }
    var goal = headingTarget == null ? destination : new Pose2d(destination.getTranslation(), heading);
    try {
      if (elapsed > 0) {
        double distance = pathDistance.remainingDistance(pose.getTranslation(), 0);
        double speed = distanceController.calculateStopping(distance,
            Constants.Swerve.DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * drivetrain.MAX_SPEED,
            criteria.translationToleranceMeters(), constraints);
        var delta = destination.getTranslation().minus(pose.getTranslation());
        desiredSpeeds.vxMetersPerSecond = distance > 0 ? speed * delta.getX() / distance : 0;
        desiredSpeeds.vyMetersPerSecond = distance > 0 ? speed * delta.getY() / distance : 0;
        desiredSpeeds.omegaRadiansPerSecond = headingRate(pose.getRotation(), heading);
        var constrained = limiter.calculate(desiredSpeeds, elapsed);
        drivetrain.applyRequest(driveRequest.withVelocityX(constrained.vxMetersPerSecond)
            .withVelocityY(constrained.vyMetersPerSecond).withRotationalRate(constrained.omegaRadiansPerSecond),
            lockHeading ? heading.getDegrees() : null);
      }
      finished = completion.update(pose, goal, state.Speeds, lockHeading, now);
    } catch (IllegalArgumentException exception) {
      fail("Invalid autonomous motion request");
    }
  }

  private double headingRate(Rotation2d current, Rotation2d target) {
    if (!lockHeading) return 0;
    double remainingAngle = Math.abs(target.minus(current).getRadians()) - criteria.headingToleranceRadians();
    if (remainingAngle <= 0) {
      drivetrain.resetAutonomousHeadingController();
      return 0;
    }
    double requested = drivetrain.calculateAutonomousHeadingRate(current, target);
    double limit = constraints.maxAngularSpeedToStop(remainingAngle);
    return Math.max(-limit, Math.min(limit, requested));
  }

  @Override public boolean isFinished() { return failed || finished; }

  private void fail(String reason) {
    failed = true;
    completion.reset();
    abort.accept(reason);
    stop();
  }

  private void stop() {
    drivetrain.applyRequest(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override public void end(boolean interrupted) { timer.stop(); stop(); }
}
