package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.game.Game2026Health;
import frc.robot.game.Game2026Settings;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.FieldUtils;

/** Driver controls use an explicit blue field frame, including red-alliance joystick perspective. */
public final class SwerveDriveCommand extends Command {
  private final StateMachine robot;
  private final Swerve drivetrain;
  private final CommandXboxController controller;
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  public SwerveDriveCommand(StateMachine robot, CommandXboxController controller) {
    this.robot = robot;
    drivetrain = robot.drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
  }

  @Override public void execute() {
    var alliance = DriverStation.getAlliance();
    if (!DriverStation.isTeleopEnabled() || alliance.isEmpty() || !Game2026Health.drive(drivetrain)) {
      stop();
      return;
    }
    var pose = drivetrain.getState().Pose;
    var settings = Game2026Settings.current();
    if (!settings.valid()) { stop(); return; }
    double direction = alliance.get() == DriverStation.Alliance.Red ? 1 : -1;
    double coefficient = FieldUtils.getZone(pose) == FieldUtils.GameZone.INTERCHANGE
        ? Constants.OI.INTERCHANGE_SPEED_COEFFICIENT : drivetrain.getDriverMaxSpeedCoefficient();
    boolean shootRequested = controller.getRightTriggerAxis() > 0.5;
    boolean intakeRequested = controller.getLeftTriggerAxis() > 0.5 || controller.getHID().getBButton();
    if (shootRequested) coefficient *= Constants.RobotContainer.DRIVE_WHILE_SHOOTING_COEFFICIENT;
    else if (intakeRequested) coefficient *= Constants.RobotContainer.DRIVE_WHILE_INTAKING_COEFFICIENT;
    double scale = drivetrain.MAX_SPEED * drivetrain.getDriverVelocityScale() * coefficient;
    double vx = shaped(controller.getLeftY()) * direction * scale;
    double vy = shaped(controller.getLeftX()) * direction * scale;
    double omega = -shaped(controller.getRightX()) * drivetrain.getDriverMaxAngularRateRadiansPerSecond()
        * drivetrain.getDriverVelocityScale();
    // Preserve source skew compensation without mixing CTRE operator and blue-field perspectives.
    double skew = drivetrain.getState().Speeds.omegaRadiansPerSecond * drivetrain.getDriverSkewCompensation();
    double rotatedX = vx * Math.cos(skew) - vy * Math.sin(skew);
    vy = vx * Math.sin(skew) + vy * Math.cos(skew);
    vx = rotatedX;
    if (shootRequested && robot.getGameConfig().inScoringZone(pose, alliance.get())) {
      double error = MathUtil.inputModulus(robot.getGameConfig().hubHeadingDegrees(pose, alliance.get())
          - pose.getRotation().getDegrees(), -180, 180);
      omega = MathUtil.clamp(Math.toRadians(error) * settings.headingKp(),
          -drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE, drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE);
    }
    drivetrain.applyRequest(request.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
  }

  private double shaped(double value) {
    double adjusted = MathUtil.applyDeadband(value, drivetrain.getDriverJoystickDeadband());
    return adjusted * adjusted * adjusted;
  }
  private void stop() { drivetrain.applyRequest(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0)); }
  @Override public void end(boolean interrupted) { stop(); }
}
