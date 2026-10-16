package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.powerlib.auto.RotationTarget;
import frc.powerlib.configs.CompletionCriteria;
import frc.powerlib.configs.MotionConstraints;
import frc.powerlib.configs.PathWaypoint;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

/** Single-point compatibility wrapper around the continuous path controller. */
public final class DriveToPointCommand extends DrivePathCommand {
  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, double toleranceInches,
      MotionConstraints constraints, boolean lockHeading, Consumer<String> abort) {
    this(drivetrain, destination, Constants.Auto.completionCriteria(toleranceInches,
        Constants.Auto.LEG_HEADING_TOLERANCE_DEGREES), constraints, lockHeading,
        Constants.RobotContainer.AUTO_LEG_TIMEOUT_SECONDS, abort);
  }

  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, CompletionCriteria criteria,
      MotionConstraints constraints, boolean lockHeading, double timeoutSeconds, Consumer<String> abort) {
    super(drivetrain, List.of(new PathWaypoint(destination, constraints, criteria, lockHeading, timeoutSeconds)), abort);
  }

  public DriveToPointCommand(Swerve drivetrain, Pose2d destination, CompletionCriteria criteria,
      MotionConstraints constraints, RotationTarget rotation, double timeoutSeconds, Consumer<String> abort) {
    super(drivetrain, List.of(new PathWaypoint(destination, constraints, criteria, rotation, timeoutSeconds)), abort);
  }

  @Override public DriveToPointCommand withRotationTarget(RotationTarget target) {
    super.withRotationTarget(target);
    return this;
  }

  @Override public DriveToPointCommand withHeadingTarget(Function<Pose2d, Rotation2d> target) {
    super.withHeadingTarget(target);
    return this;
  }
}
