package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.powerlib.PowerRobotContainer;
import frc.powerlib.auto.AutoBuilder;
import frc.powerlib.controls.ButtonBindings;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.autos.RobotAutos;
import frc.robot.subsystems.PowerDashboard;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

public class RobotContainer implements PowerRobotContainer {
  private final StateMachine stateMachine = new StateMachine();
  private final PowerDashboard powerDashboard = new PowerDashboard(stateMachine);
  private final CommandXboxController driverController = new CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);
  private final AutoBuilder autoBuilder = new AutoBuilder();

  public RobotContainer() {
    configureBindings();
    stateMachine.drivetrain.setDefaultCommand(new SwerveDriveCommand(
        stateMachine.drivetrain, driverController, stateMachine::getCurrentState));
    configureAutos();
  }

  private void configureBindings() {
    ButtonBindings.bindStates(driverController.leftTrigger(0.5).and(DriverStation::isTeleopEnabled),
        RobotState.INTAKING, RobotState.IDLE, stateMachine::requestState);
    ButtonBindings.bindStates(driverController.b().and(DriverStation::isTeleopEnabled),
        RobotState.EJECTING, RobotState.IDLE, stateMachine::requestState);
    ButtonBindings.bindStates(driverController.rightTrigger(0.5).and(DriverStation::isTeleopEnabled),
        RobotState.SHOOTING, RobotState.IDLE, stateMachine::requestState);
    ButtonBindings.bindFunctions(driverController.back(), this::seedVisionPose, true);
    ButtonBindings.bindFunctions(driverController.start(), this::zeroHeading, true);
  }

  private void seedVisionPose() {
    boolean seeded = stateMachine.vision.seedFromFreshVision();
    SignalLogger.writeString("Robot/Control/Action", seeded ? "Fresh vision pose seeded"
        : "Pose seed requires disabled robot and fresh vision");
  }

  private void zeroHeading() {
    if (!DriverStation.isDisabled()) return;
    // Robot must physically face blue-field +X; preserve X/Y while zeroing heading.
    stateMachine.drivetrain.resetRotation(new Rotation2d());
    SignalLogger.writeString("Robot/Control/Action", "Heading zeroed; X/Y preserved");
  }

  private void configureAutos() {
    RobotAutos.register(autoBuilder, stateMachine);
    autoBuilder.publish();
  }

  public void resetForMode() { stateMachine.stopAll(); }
  public Command getAutonomousCommand() { return autoBuilder.getAutonomousCommand(); }
  public StateMachine getStateMachine() { return stateMachine; }
}
