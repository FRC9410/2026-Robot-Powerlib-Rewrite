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
import frc.robot.utils.AimingDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Objects;
import java.util.function.Supplier;

/** Uses a blue-field frame and live current state for drive scaling and shot alignment. */
public final class SwerveDriveCommand extends Command {
  private static final double LOOP_SECONDS = 0.02;
  private final Swerve drivetrain;
  private final CommandXboxController controller;
  private final Supplier<RobotState> currentStateSupplier;
  private final Supplier<ShootingUtils.ShotSolution> shotSupplier;
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private final SwerveRequest.FieldCentric aimingRequest = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
      .withDeadband(0.02).withRotationalDeadband(0);
  private final SwerveRequest.PointWheelsAt parallelWheels = new SwerveRequest.PointWheelsAt()
      .withModuleDirection(Rotation2d.kZero);
  private final AimingDriveController aimingDrive = new AimingDriveController();

  public SwerveDriveCommand(Swerve drivetrain, CommandXboxController controller,
      Supplier<RobotState> currentStateSupplier) {
    this(drivetrain, controller, currentStateSupplier, () -> ShootingUtils.UNAVAILABLE);
  }

  public SwerveDriveCommand(Swerve drivetrain, CommandXboxController controller,
      Supplier<RobotState> currentStateSupplier, Supplier<ShootingUtils.ShotSolution> shotSupplier) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.currentStateSupplier = Objects.requireNonNull(currentStateSupplier);
    this.shotSupplier = Objects.requireNonNull(shotSupplier);
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
    if (aiming && !ShootingUtils.sotmEnabled()) {
      // Kill switch off: original stationary shot, so hold translation while aiming.
      vx = 0;
      vy = 0;
    } else if (aiming) {
      double cap = Math.min(Constants.RobotContainer.SHOOTING_MAX_TRANSLATION_SPEED_MPS,
          Constants.Shooting.SOTM_MAX_DRIVETRAIN_SPEED_MPS);
      if (!Double.isFinite(cap) || cap < 0) { stop(); return; }
      var limited = AimingDriveController.limitTranslation(vx, vy, cap);
      vx = limited.getX();
      vy = limited.getY();
    }
    Double heading = null;
    if (aiming && ShootingUtils.inScoringZone(pose, alliance.get())) {
      var shot = shotSupplier.get();
      boolean solved = shot != null && shot.solverValid() && shot.valid();
      var aim = aimingDrive.slewHeading(
          solved ? shot.headingDegrees() : ShootingUtils.headingDegrees(pose, alliance.get()),
          Constants.Shooting.AIM_HEADING_SLEW_DEGREES_PER_SECOND, LOOP_SECONDS);
      heading = aim.headingDegrees();
      double error = MathUtil.inputModulus(heading - pose.getRotation().getDegrees(), -180, 180);
      double kp = PowerRobotContainer.getSubsystemVariable("Swerve", "Heading/kP", Constants.Swerve.HEADING_KP);
      double kd = PowerRobotContainer.getSubsystemVariable("Swerve", "Heading/kD", Constants.Swerve.HEADING_KD);
      if (!Double.isFinite(kp) || kp <= 0 || !Double.isFinite(kd) || kd < 0) { stop(); return; }
      // Follow the slewed target's rate while it catches up, otherwise the solver's tracking rate.
      double feedforward = aim.limited() ? aim.rateRadiansPerSecond()
          : solved ? shot.headingFeedforwardRadiansPerSecond() : 0;
      double measuredOmega = drivetrain.getState().Speeds.omegaRadiansPerSecond;
      // D acts on heading-error rate (target rate minus measured rate) to damp overshoot.
      double damping = Double.isFinite(measuredOmega) ? kd * (feedforward - measuredOmega) : 0;
      omega = MathUtil.clamp(Math.toRadians(error) * kp + feedforward + damping,
          -drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE, drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE);
      if (aimingDrive.updateParking(vx, vy, drivetrain.getState().Speeds, error)) {
        PowerRobotContainer.setSubsystemData("Drive", "AimWheelsParked", true);
        drivetrain.applyRequest(parallelWheels, heading);
        return;
      }
    } else {
      aimingDrive.reset();
    }
    PowerRobotContainer.setSubsystemData("Drive", "AimWheelsParked", false);
    drivetrain.applyRequest((aiming ? aimingRequest : request)
        .withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega), heading);
  }

  private double shaped(double value) {
    double adjusted = MathUtil.applyDeadband(value, drivetrain.getDriverJoystickDeadband());
    return adjusted * adjusted * adjusted;
  }

  private void stop() {
    aimingDrive.reset();
    PowerRobotContainer.setSubsystemData("Drive", "AimWheelsParked", false);
    drivetrain.applyRequest(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
  @Override public void end(boolean interrupted) { stop(); }
  @Override public boolean isFinished() { return false; }
}
