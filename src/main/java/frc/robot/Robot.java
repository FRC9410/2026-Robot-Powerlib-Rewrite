package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final RobotContainer robotContainer = new RobotContainer();
  private Command autonomousCommand;
  @Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }
  @Override public void disabledInit() { cancelAuto(); robotContainer.resetForMode(); }
  @Override public void autonomousInit() {
    cancelAuto();
    robotContainer.resetForMode();
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) CommandScheduler.getInstance().schedule(autonomousCommand);
  }
  @Override public void autonomousExit() { cancelAuto(); robotContainer.resetForMode(); }
  @Override public void teleopInit() { cancelAuto(); robotContainer.resetForMode(); }
  @Override public void teleopExit() { robotContainer.resetForMode(); }
  @Override public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.resetForMode();
  }
  private void cancelAuto() {
    if (autonomousCommand != null) { autonomousCommand.cancel(); autonomousCommand = null; }
  }
}
