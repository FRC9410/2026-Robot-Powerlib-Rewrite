package frc.robot.autos;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.powerlib.auto.Auto;
import frc.powerlib.auto.AutoBuilder;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.RequestState;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.utils.ShootingUtils;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

/** Four April 4 routes share one intake/return/shoot sequence. */
public final class RobotAutos {
  private RobotAutos() {}

  public record Route(String name, Alliance alliance, List<Pose2d> points) {
    public Route { points = List.copyOf(points); }
    /** April 4 red headings were operator-relative; blue-field requests need a 180-degree offset. */
    public List<Pose2d> fieldPoints() {
      return points.stream().map(point -> alliance == Alliance.Red
          ? new Pose2d(point.getTranslation(), point.getRotation().plus(Rotation2d.fromDegrees(180))) : point).toList();
    }
  }

  public static final List<Route> ROUTES = List.of(
      new Route("Red Left", Alliance.Red, List.of(Constants.Auto.RED_LEFT_1, Constants.Auto.RED_LEFT_2,
          Constants.Auto.RED_LEFT_3, Constants.Auto.RED_LEFT_4, Constants.Auto.RED_LEFT_5, Constants.Auto.RED_LEFT_6, Constants.Auto.RED_LEFT_7)),
      new Route("Red Right", Alliance.Red, List.of(Constants.Auto.RED_RIGHT_1, Constants.Auto.RED_RIGHT_2,
          Constants.Auto.RED_RIGHT_3, Constants.Auto.RED_RIGHT_4, Constants.Auto.RED_RIGHT_5, Constants.Auto.RED_RIGHT_6, Constants.Auto.RED_RIGHT_7)),
      new Route("Blue Left", Alliance.Blue, List.of(Constants.Auto.BLUE_LEFT_1, Constants.Auto.BLUE_LEFT_2,
          Constants.Auto.BLUE_LEFT_3, Constants.Auto.BLUE_LEFT_4, Constants.Auto.BLUE_LEFT_5, Constants.Auto.BLUE_LEFT_6, Constants.Auto.BLUE_LEFT_7)),
      new Route("Blue Right", Alliance.Blue, List.of(Constants.Auto.BLUE_RIGHT_1, Constants.Auto.BLUE_RIGHT_2,
          Constants.Auto.BLUE_RIGHT_3, Constants.Auto.BLUE_RIGHT_4, Constants.Auto.BLUE_RIGHT_5, Constants.Auto.BLUE_RIGHT_6, Constants.Auto.BLUE_RIGHT_7)));

  public static void register(AutoBuilder chooser, StateMachine robot) {
    for (Route route : ROUTES) chooser.addAuto(route.name(), route.alliance(), () -> build(route, robot));
  }

  /** Factories keep all commands, request owners and timers fresh for every run. */
  public static Command build(Route route, StateMachine robot) {
    Object owner = new Object();
    Run run = new Run();
    List<Pose2d> points = route.fieldPoints();
    Command sequence = sequence(route, state -> new RequestState(state, owner, robot),
        () -> aimAtHub(robot, route.alliance()).until(() -> aligned(robot, route.alliance(), Constants.Auto.TURN_TOLERANCE_DEGREES))
            .withTimeout(Constants.Auto.TURN_TIMEOUT_SECONDS),
        () -> aimAtHub(robot, route.alliance()))
        .build(point -> {
          int leg = points.indexOf(point);
          return new DriveToPointCommand(robot.drivetrain, point, Constants.Auto.LEG_TOLERANCE_INCHES[leg],
              Constants.Auto.LEG_SPEED_COEFFICIENTS[leg], Constants.Auto.LEG_LOCK_HEADING[leg], run::abort);
        });
    Command guarded = Commands.either(sequence, Commands.none(), () -> run.start(route, robot));
    return guarded.until(() -> run.aborted || run.timer.hasElapsed(Constants.Field.AUTO_LENGTH_IN_TIME)
        || !DriverStation.isAutonomousEnabled())
        .finallyDo(interrupted -> {
          run.timer.stop();
          robot.clearRequest(owner);
          robot.stopAll();
          robot.drivetrain.applyRequest(new SwerveRequest.SwerveDriveBrake());
          if (!run.aborted) SignalLogger.writeString("Robot/Auto/Status", interrupted ? "Interrupted" : "Finished");
        }).withName(route.name());
  }

  static Auto sequence(Route route, Function<RobotState, Command> request,
      Supplier<Command> turn, Supplier<Command> hold) {
    return new Auto(route.name()).withAlliance(route.alliance()).withPath(route.fieldPoints().toArray(Pose2d[]::new))
        .toNextPoint().toNextPoint()
        .addCommand(() -> request.apply(RobotState.INTAKING))
        .toNextPoint().toNextPoint().toNextPoint()
        .addCommand(() -> request.apply(RobotState.READY))
        .toNextPoint().toNextPoint()
        .addCommand(turn)
        .addCommand(() -> request.apply(RobotState.SHOOTING))
        .addCommand(hold);
  }

  private static Command aimAtHub(StateMachine robot, Alliance alliance) {
    return Commands.run(() -> {
      var drive = robot.drivetrain;
      if (!HealthChecks.driveHealthy(drive, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
        drive.applyRequest(new SwerveRequest.SwerveDriveBrake());
        return;
      }
      double heading = ShootingUtils.headingDegrees(drive.getState().Pose, alliance);
      drive.applyRequest(drive.DRIVE_AT_ANGLE.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withVelocityX(0).withVelocityY(0).withTargetDirection(Rotation2d.fromDegrees(heading))
          .withMaxAbsRotationalRate(drive.MAX_DRIVE_TO_POINT_ANGULAR_RATE), heading);
    }, robot.drivetrain);
  }

  private static boolean aligned(StateMachine robot, Alliance alliance, double tolerance) {
    var pose = robot.drivetrain.getState().Pose;
    return HealthChecks.finitePose(pose) && Math.abs(Rotation2d.fromDegrees(ShootingUtils.headingDegrees(pose, alliance))
        .minus(pose.getRotation()).getDegrees()) < tolerance;
  }

  private static final class Run {
    final Timer timer = new Timer();
    boolean aborted;

    boolean start(Route route, StateMachine robot) {
      timer.restart();
      if (!DriverStation.isAutonomousEnabled() || DriverStation.getAlliance().orElse(null) != route.alliance()) {
        abort("Robot mode/alliance does not match auto");
      } else if (!HealthChecks.driveHealthy(robot.drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
        abort("Drivetrain feedback unavailable at start");
      } else {
        var pose = robot.drivetrain.getState().Pose;
        var start = route.fieldPoints().get(0);
        if (pose.getTranslation().getDistance(start.getTranslation()) > Constants.RobotContainer.AUTO_START_TOLERANCE_METERS
            || Math.abs(pose.getRotation().minus(start.getRotation()).getDegrees()) > Constants.RobotContainer.AUTO_START_TOLERANCE_DEGREES) {
          abort("Starting pose is outside position/heading tolerance");
        }
      }
      if (!aborted) SignalLogger.writeString("Robot/Auto/Status", "Running " + route.name());
      return !aborted;
    }

    void abort(String reason) {
      if (aborted) return;
      aborted = true;
      SignalLogger.writeString("Robot/Auto/Status", "Aborted");
      SignalLogger.writeString("Robot/Auto/Reason", reason);
    }
  }
}
