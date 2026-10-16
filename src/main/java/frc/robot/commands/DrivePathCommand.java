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
import frc.powerlib.auto.CrossTrackController;
import frc.powerlib.auto.DistanceSpeedController;
import frc.powerlib.auto.PathProgress;
import frc.powerlib.configs.CrossTrackConfig;
import frc.powerlib.configs.PathWaypoint;
import frc.powerlib.configs.WaypointHandoffConfig;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;

/** Keeps control history through intermediate targets and settles only at the section endpoint. */
public class DrivePathCommand extends Command {
  private final Swerve drivetrain;
  private final PathProgress progress;
  private final CompletionChecker completion;
  private final Consumer<String> abort;
  private final ChassisMotionLimiter limiter;
  private final CrossTrackController steering;
  private final ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private final DistanceSpeedController distanceController = new DistanceSpeedController(
      new PIDController(Constants.RobotContainer.AUTO_TRANSLATION_KP, 0, Constants.RobotContainer.AUTO_TRANSLATION_KD),
      Units.inchesToMeters(Constants.Auto.STATIC_FEEDFORWARD_THRESHOLD_INCHES));
  private final Timer waypointTimer = new Timer();
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private Function<Pose2d, Rotation2d> headingTarget;
  private double lastTimestamp;
  private boolean failed;
  private boolean finished;

  public DrivePathCommand(Swerve drivetrain, List<PathWaypoint> waypoints, Consumer<String> abort) {
    this(drivetrain, waypoints, WaypointHandoffConfig.RADIUS_ONLY, abort);
  }

  public DrivePathCommand(Swerve drivetrain, List<PathWaypoint> waypoints, WaypointHandoffConfig handoff,
      Consumer<String> abort) {
    this(drivetrain, waypoints, handoff, CrossTrackConfig.DISABLED, abort);
  }

  public DrivePathCommand(Swerve drivetrain, List<PathWaypoint> waypoints, WaypointHandoffConfig handoff,
      CrossTrackConfig crossTrack, Consumer<String> abort) {
    this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain");
    this.abort = Objects.requireNonNull(abort, "abort");
    progress = new PathProgress(waypoints, handoff);
    completion = new CompletionChecker(progress.endpoint().completion());
    limiter = new ChassisMotionLimiter(progress.constraints());
    steering = new CrossTrackController(crossTrack);
    addRequirements(drivetrain);
  }

  /** Configure before scheduling, for goals such as aiming at the hub from the current pose. */
  public DrivePathCommand withHeadingTarget(Function<Pose2d, Rotation2d> target) {
    headingTarget = Objects.requireNonNull(target, "target");
    return this;
  }

  @Override public void initialize() {
    failed = false;
    finished = false;
    progress.reset();
    completion.reset();
    distanceController.reset();
    drivetrain.resetAutonomousHeadingController();
    waypointTimer.restart();
    lastTimestamp = Timer.getFPGATimestamp();
    if (!HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      fail("Drivetrain feedback unavailable");
      return;
    }
    var state = drivetrain.getState();
    try {
      progress.reset(state.Pose.getTranslation());
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
    // A late arrival cannot restart an already-expired waypoint deadline.
    if (waypointTimer.hasElapsed(progress.current().timeoutSeconds())) {
      fail("Autonomous waypoint timed out");
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
    try {
      if (progress.advance(pose.getTranslation())) waypointTimer.restart();
      var waypoint = progress.current();
      var heading = headingTarget == null ? waypoint.pose().getRotation() : headingTarget.apply(pose);
      if (heading == null || !Double.isFinite(heading.getRadians())) {
        fail("Autonomous heading target unavailable");
        return;
      }
      var goal = headingTarget == null ? waypoint.pose() : new Pose2d(waypoint.pose().getTranslation(), heading);
      if (elapsed > 0) {
        double speed = Math.min(progress.maxTranslationSpeed(pose.getTranslation()),
            distanceController.calculateStopping(progress.remainingDistance(pose.getTranslation()),
                Constants.Swerve.DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * drivetrain.MAX_SPEED,
                progress.atEndpoint() ? waypoint.completion().translationToleranceMeters() : 0, progress.constraints()));
        steering.calculate(progress.currentSegment(), pose.getTranslation(), speed, desiredSpeeds);
        desiredSpeeds.omegaRadiansPerSecond = headingRate(pose.getRotation(), heading, waypoint);
        var constrained = limiter.calculate(desiredSpeeds, elapsed);
        drivetrain.applyRequest(driveRequest
            .withDriveRequestType(waypoint.lockHeading() ? DriveRequestType.Velocity : DriveRequestType.OpenLoopVoltage)
            .withVelocityX(constrained.vxMetersPerSecond).withVelocityY(constrained.vyMetersPerSecond)
            .withRotationalRate(constrained.omegaRadiansPerSecond), waypoint.lockHeading() ? heading.getDegrees() : null);
      }
      finished = progress.atEndpoint() && completion.update(pose, goal, state.Speeds, waypoint.lockHeading(), now);
    } catch (IllegalArgumentException exception) {
      fail("Invalid autonomous motion request");
    }
  }

  private double headingRate(Rotation2d current, Rotation2d target, PathWaypoint waypoint) {
    if (!waypoint.lockHeading()) return 0;
    double remainingAngle = Math.abs(target.minus(current).getRadians()) - waypoint.completion().headingToleranceRadians();
    if (remainingAngle <= 0) {
      drivetrain.resetAutonomousHeadingController();
      return 0;
    }
    double requested = drivetrain.calculateAutonomousHeadingRate(current, target);
    double limit = progress.constraints().maxAngularSpeedToStop(remainingAngle);
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

  @Override public void end(boolean interrupted) { waypointTimer.stop(); stop(); }
}
