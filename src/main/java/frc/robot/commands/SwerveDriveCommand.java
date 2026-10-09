package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.powerlib.PowerRobotContainer;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.ShootingUtils;
import java.util.Objects;
import java.util.function.Supplier;

/** Uses a blue-field frame and live current state for drive scaling and shot alignment. */
public final class SwerveDriveCommand extends Command {
  private final Swerve drivetrain;
  private final CommandXboxController controller;
  private final Supplier<RobotState> currentStateSupplier;
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  public SwerveDriveCommand(Swerve drivetrain, CommandXboxController controller,
      Supplier<RobotState> currentStateSupplier) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.currentStateSupplier = Objects.requireNonNull(currentStateSupplier);
    addRequirements(drivetrain);
  }

  public RobotState getCurrentState() { return currentStateSupplier.get(); }

  @Override
  public void execute() {
    var alliance = DriverStation.getAlliance();
    if (!DriverStation.isTeleopEnabled() || alliance.isEmpty() || !HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      stop();
      return;
    }
    var pose = drivetrain.getState().Pose;
    var state = getCurrentState();
    boolean aiming = state == RobotState.SPINNING_UP || state == RobotState.SHOOTING;
    double coefficient = FieldUtils.getZone(pose) == FieldUtils.GameZone.INTERCHANGE
        ? Constants.OI.INTERCHANGE_SPEED_COEFFICIENT : drivetrain.getDriverMaxSpeedCoefficient();
    if (aiming) coefficient *= Constants.RobotContainer.DRIVE_WHILE_SHOOTING_COEFFICIENT;
    else if (state == RobotState.INTAKING || state == RobotState.EJECTING) coefficient *= Constants.RobotContainer.DRIVE_WHILE_INTAKING_COEFFICIENT;
    double direction = alliance.get() == DriverStation.Alliance.Red ? 1 : -1;
    double scale = drivetrain.MAX_SPEED * drivetrain.getDriverVelocityScale() * coefficient;
    double vx = shaped(controller.getLeftY()) * direction * scale;
    double vy = shaped(controller.getLeftX()) * direction * scale;
    double omega = -shaped(controller.getRightX()) * drivetrain.getDriverMaxAngularRateRadiansPerSecond()
        * drivetrain.getDriverVelocityScale();
    double skew = drivetrain.getState().Speeds.omegaRadiansPerSecond * drivetrain.getDriverSkewCompensation();
    double rotatedX = vx * Math.cos(skew) - vy * Math.sin(skew);
    vy = vx * Math.sin(skew) + vy * Math.cos(skew);
    vx = rotatedX;
    Double heading = null;
    if (aiming && ShootingUtils.inScoringZone(pose, alliance.get())) {
      heading = ShootingUtils.headingDegrees(pose, alliance.get());
      double error = MathUtil.inputModulus(heading - pose.getRotation().getDegrees(), -180, 180);
      double kp = PowerRobotContainer.getSubsystemVariable("Swerve", "Heading/kP", Constants.Swerve.HEADING_KP);
      if (!Double.isFinite(kp) || kp <= 0) { stop(); return; }
      omega = MathUtil.clamp(Math.toRadians(error) * kp,
          -drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE, drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE);
    }
    drivetrain.applyRequest(request.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega), heading);
  }

  private double shaped(double value) {
    double adjusted = MathUtil.applyDeadband(value, drivetrain.getDriverJoystickDeadband());
    return adjusted * adjusted * adjusted;
  }

  private void stop() { drivetrain.applyRequest(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0)); }
  @Override public void end(boolean interrupted) { stop(); }
  @Override public boolean isFinished() { return false; }
}
