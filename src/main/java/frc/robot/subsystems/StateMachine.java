// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.powerlib.subsystems.AbsolutePositionSubsystem;
import frc.powerlib.subsystems.RelativePositionSubsystem;
import frc.powerlib.subsystems.VelocitySubsystem;
import frc.powerlib.subsystems.VelocityTorqueSubsystem;
import frc.robot.Constants;

public class StateMachine extends SubsystemBase {
  public enum RobotState {
    IDLE, READY, SHOOTING
  }

  public final Swerve drivetrain = Constants.Tuner.createDrivetrain();
  public final Vision vision = new Vision(drivetrain);

  // POWERLIB GENERATED SUBSYSTEMS START - DO NOT DELETE
  public final VelocitySubsystem feeder = new VelocitySubsystem(Constants.Feeder.FEEDER_CONFIG);
  public final VelocityTorqueSubsystem intakeRoller = new VelocityTorqueSubsystem(Constants.IntakeRoller.INTAKE_ROLLER_CONFIG);
  public final AbsolutePositionSubsystem intakeWrist = new AbsolutePositionSubsystem(Constants.IntakeWrist.INTAKE_WRIST_CONFIG);
  public final VelocitySubsystem shooter = new VelocitySubsystem(Constants.Shooter.SHOOTER_CONFIG);
  public final AbsolutePositionSubsystem shooterHood = new AbsolutePositionSubsystem(Constants.ShooterHood.SHOOTER_HOOD_CONFIG);
  public final VelocitySubsystem spindexer = new VelocitySubsystem(Constants.Spindexer.SPINDEXER_CONFIG);
  public final AbsolutePositionSubsystem turret = new AbsolutePositionSubsystem(Constants.Turret.TURRET_CONFIG);
  // POWERLIB GENERATED SUBSYSTEMS END - DO NOT DELETE

  private RobotState wantedState = Constants.StateMachine.DEFAULT_STATE;

  public RobotState getWantedState() {
    return wantedState;
  }

  public void setWantedState(RobotState wantedState) {
    this.wantedState = wantedState;
  }

  @Override
  public void periodic() {}
}
