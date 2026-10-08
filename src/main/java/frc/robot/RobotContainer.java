package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.powerlib.PowerRobotContainer;
import frc.powerlib.auto.AutoBuilder;
import frc.robot.commands.Game2026Auto;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.PowerDashboard;
import frc.robot.subsystems.StateMachine;

public class RobotContainer implements PowerRobotContainer {
  private final StateMachine stateMachine = new StateMachine();
  private final PowerDashboard powerDashboard = new PowerDashboard(stateMachine);
  private final CommandXboxController driverController = new CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);
  private final AutoBuilder autoBuilder = new AutoBuilder();
  private boolean collecting, ejecting;

  public RobotContainer() {
    configureBindings();
    stateMachine.drivetrain.setDefaultCommand(new SwerveDriveCommand(stateMachine, driverController));
    configureAutos();
  }

  private void configureBindings() {
    driverController.leftTrigger(0.5).and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> { collecting = true; updateIntake(); }))
        .onFalse(Commands.runOnce(() -> { collecting = false; updateIntake(); }));
    driverController.b().and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> { ejecting = true; updateIntake(); }))
        .onFalse(Commands.runOnce(() -> { ejecting = false; updateIntake(); }));
    driverController.rightTrigger(0.5).and(DriverStation::isTeleopEnabled)
        .onTrue(Commands.runOnce(() -> stateMachine.setWantedState(StateMachine.RobotState.SHOOTING)))
        .onFalse(Commands.runOnce(() -> stateMachine.setWantedState(StateMachine.RobotState.READY)));
    driverController.back().onTrue(Commands.runOnce(() -> {
      boolean seeded = stateMachine.vision.seedFromFreshVision();
      stateMachine.setAutoStatus("POSE SEED", seeded ? "Fresh vision pose seeded" : "Requires disabled robot and a fresh vision frame");
    }).ignoringDisable(true));
    driverController.start().onTrue(Commands.runOnce(() -> {
      if (!DriverStation.isDisabled()) {
        stateMachine.setAutoStatus("HEADING RESET", "Disable robot before zeroing heading");
        return;
      }
      // Reset the pose heading used by field-centric drive, aiming, and vision orientation.
      // The robot must physically face blue-field +X (toward the red end).
      stateMachine.drivetrain.resetRotation(new Rotation2d());
      stateMachine.setAutoStatus("HEADING RESET", "Heading zeroed; X/Y preserved");
    }).ignoringDisable(true));
  }
  private void updateIntake() { stateMachine.setIntake(collecting, ejecting); }
  private void configureAutos() {
    autoBuilder.addAuto("Red Left", Alliance.Red, () -> new Game2026Auto(stateMachine, "Red Left"));
    autoBuilder.addAuto("Red Right", Alliance.Red, () -> new Game2026Auto(stateMachine, "Red Right"));
    autoBuilder.addAuto("Blue Left", Alliance.Blue, () -> new Game2026Auto(stateMachine, "Blue Left"));
    autoBuilder.addAuto("Blue Right", Alliance.Blue, () -> new Game2026Auto(stateMachine, "Blue Right"));
    autoBuilder.publish();
  }
  public void resetForMode() {
    collecting = ejecting = false;
    stateMachine.stopAll();
  }
  public Command getAutonomousCommand() {
    stateMachine.setAutoStatus("NONE", "No autonomous selected");
    return autoBuilder.getAutonomousCommand();
  }
  public StateMachine getStateMachine() { return stateMachine; }
}
