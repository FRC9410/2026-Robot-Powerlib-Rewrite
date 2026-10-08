package frc.powerlib.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.powerlib.configs.MotorConfig;
import frc.robot.constants.ShooterConstants;
import org.junit.jupiter.api.Test;

class VelocityConfigurationTest {
  @Test void shooterStartupPreservesCompetitionDirectionAndCoast() {
    var shooter = ShooterConstants.SHOOTER_CONFIG;
    var leader = shooter.motorConfigs().get(0);
    var config = VelocitySubsystem.velocityConfiguration(
        shooter.leadConfig(), shooter.motionMagicConfig(), leader);
    assertEquals(InvertedValue.CounterClockwise_Positive, config.MotorOutput.Inverted);
    assertEquals(NeutralModeValue.Coast, config.MotorOutput.NeutralMode);
    assertEquals(1, config.Feedback.SensorToMechanismRatio);
    assertEquals(1, config.Feedback.RotorToSensorRatio);
    assertEquals(200, config.MotionMagic.MotionMagicAcceleration);
    assertEquals(PowerSubsystem.motorOutputConfig(leader.isReversed(), leader.neutralMode()).Inverted,
        config.MotorOutput.Inverted);
    assertTrue(shooter.motorConfigs().get(1).isReversed());
    assertTrue(shooter.motorConfigs().get(2).isReversed());
  }

  @Test void reversedVelocityLeaderSurvivesFullConfiguration() {
    var shooter = ShooterConstants.SHOOTER_CONFIG;
    var config = VelocitySubsystem.velocityConfiguration(shooter.leadConfig(),
        shooter.motionMagicConfig(), MotorConfig.leader(51, NeutralModeValue.Brake, true));
    assertEquals(InvertedValue.Clockwise_Positive, config.MotorOutput.Inverted);
    assertEquals(NeutralModeValue.Brake, config.MotorOutput.NeutralMode);
  }
}
