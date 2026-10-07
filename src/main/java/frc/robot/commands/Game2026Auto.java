package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.game.Game2026Health;
import frc.robot.game.Game2026Settings;
import frc.robot.game.QuadrantAutoController;
import frc.robot.subsystems.StateMachine;

/** Fresh instances are supplied by the PowerLib AutoBuilder chooser. */
public final class Game2026Auto extends Command {
  private final StateMachine robot;
  private final QuadrantAutoController controller;
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  public Game2026Auto(StateMachine robot, String selection) {
    this.robot = robot;
    controller = new QuadrantAutoController(robot.getGameConfig(), selection);
    addRequirements(robot, robot.drivetrain, robot.shooter, robot.shooterHood, robot.intakeRoller,
        robot.intakeWrist, robot.feeder, robot.spindexer);
  }

  @Override public void initialize() {
    robot.resetState();
    apply(controller.start(robot.drivetrain.getState().Pose, DriverStation.getAlliance().orElse(null),
        Game2026Health.drive(robot.drivetrain), Timer.getFPGATimestamp(), Game2026Settings.current()));
  }

  @Override public void execute() {
    apply(controller.update(robot.drivetrain.getState().Pose, DriverStation.getAlliance().orElse(null),
        DriverStation.isAutonomousEnabled(), Game2026Health.drive(robot.drivetrain), Timer.getFPGATimestamp(),
        robot.drivetrain.MAX_SPEED, robot.drivetrain.MAX_DRIVE_TO_POINT_ANGULAR_RATE, Game2026Settings.current()));
  }

  private void apply(QuadrantAutoController.Output output) {
    robot.drivetrain.applyRequest(request.withVelocityX(output.vx()).withVelocityY(output.vy()).withRotationalRate(output.omega()));
    robot.setIntake(output.collect(), false);
    robot.setWantedState(output.shoot() ? StateMachine.RobotState.SHOOTING : StateMachine.RobotState.READY);
    robot.setAutoStatus(output.status().name(), output.reason());
    if (controller.done()) robot.stopAll();
  }

  @Override public boolean isFinished() { return controller.done(); }
  @Override public void end(boolean interrupted) {
    if (interrupted) controller.cancel();
    robot.drivetrain.applyRequest(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    robot.stopAll();
    robot.setAutoStatus(controller.status().name(), controller.reason());
  }
}
