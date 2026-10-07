package frc.robot.game;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import frc.powerlib.subsystems.AbsolutePositionSubsystem;
import frc.powerlib.subsystems.PowerSubsystem;
import frc.powerlib.subsystems.VelocitySubsystem;
import frc.powerlib.subsystems.VelocityTorqueSubsystem;
import frc.robot.subsystems.Swerve;

/** Real hardware must provide recent successful CAN signals; simulation uses its IO snapshots. */
public final class Game2026Health {
  private Game2026Health() {}
  private static boolean signal(BaseStatusSignal signal) {
    return signal != null && signal.getStatus().isOK() && signal.getTimestamp().isValid()
        && signal.getTimestamp().getLatency() >= 0 && signal.getTimestamp().getLatency() <= 0.5;
  }
  public static boolean mechanism(PowerSubsystem subsystem) {
    if (RobotBase.isSimulation()) return true;
    TalonFX motor = null;
    if (subsystem instanceof AbsolutePositionSubsystem position) motor = position.getPositionMotor();
    else if (subsystem instanceof VelocitySubsystem velocity) motor = velocity.getVelocityMotor();
    else if (subsystem instanceof VelocityTorqueSubsystem velocity) motor = velocity.getVelocityMotor();
    return motor != null && motor.isConnected() && signal(motor.getPosition()) && signal(motor.getVelocity());
  }
  public static boolean drive(Swerve drive) {
    if (!Game2026Config.finitePose(drive.getState().Pose)) return false;
    if (RobotBase.isSimulation()) return true;
    if (!signal(drive.getPigeon2().getYaw())) return false;
    for (var module : drive.getModules()) {
      if (!signal(module.getDriveMotor().getVelocity()) || !signal(module.getSteerMotor().getPosition())
          || !signal(module.getEncoder().getAbsolutePosition())) return false;
    }
    return true;
  }
}
