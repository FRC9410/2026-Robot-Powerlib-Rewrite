package frc.powerlib.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.powerlib.configs.VelocitySubsystemConfig;
import frc.robot.constants.*;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** Checks the actual Phoenix configuration, rather than just copied reversal flags. */
class RegionalMotorConfigurationTest {
  @Test void velocityLeadersRetainApril4DirectionAndCurrentLimits() {
    for (var mechanism : List.of(ShooterConstants.SHOOTER_CONFIG,
        FeederConstants.FEEDER_CONFIG, SpindexerConstants.SPINDEXER_CONFIG)) {
      var leader = mechanism.motorConfigs().get(0);
      var output = MotorConfiguration.velocity(leader, mechanism);
      assertEquals(InvertedValue.CounterClockwise_Positive, output.MotorOutput.Inverted,
          mechanism.subsystemName());
      assertTrue(output.CurrentLimits.StatorCurrentLimitEnable);
      assertEquals(140, output.CurrentLimits.StatorCurrentLimit);
      for (var follower : mechanism.motorConfigs().subList(1, mechanism.motorConfigs().size())) {
        assertTrue(follower.isReversed());
        assertEquals(InvertedValue.CounterClockwise_Positive,
            MotorConfiguration.output(follower).MotorOutput.Inverted);
      }
    }
    var defaults = new com.ctre.phoenix6.configs.TalonFXConfiguration().CurrentLimits;
    var roller = MotorConfiguration.velocity(IntakeRollerConstants.INTAKE_ROLLER_CONFIG.motorConfigs().get(0),
        IntakeRollerConstants.INTAKE_ROLLER_CONFIG).CurrentLimits;
    assertEquals(defaults.StatorCurrentLimitEnable, roller.StatorCurrentLimitEnable);
    assertEquals(defaults.StatorCurrentLimit, roller.StatorCurrentLimit);
  }

  @Test void positionLeadersKeepSignedFeedbackAndEncoderCalibration() {
    for (var mechanism : List.of(IntakeWristConstants.INTAKE_WRIST_CONFIG,
        ShooterHoodConstants.SHOOTER_HOOD_CONFIG)) {
      var output = MotorConfiguration.absolute(mechanism.motorConfigs().get(0), mechanism);
      assertEquals(InvertedValue.CounterClockwise_Positive, output.MotorOutput.Inverted);
      assertEquals(-1, output.Feedback.SensorToMechanismRatio);
      assertEquals(mechanism.cancoderConfig().encoderId(), output.Feedback.FeedbackRemoteSensorID);
    }
    assertEquals(-0.036, IntakeWristConstants.CANCODER_MAGNET_OFFSET);
    assertEquals(0.058, ShooterHoodConstants.CANCODER_MAGNET_OFFSET);
  }

  @Test void optionalCurrentLimitIsBackwardCompatibleAndRejectsInvalidValues() {
    var config = ShooterConstants.SHOOTER_CONFIG;
    var legacy = new VelocitySubsystemConfig(config.motorConfigs(), config.leadConfig(),
        config.motionMagicConfig(), config.subsystemName(), config.torqueFeedForward());
    assertTrue(legacy.statorCurrentLimitAmps().isEmpty());
    for (double invalid : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
      assertThrows(IllegalArgumentException.class, () -> new VelocitySubsystemConfig(
          config.motorConfigs(), config.leadConfig(), config.motionMagicConfig(),
          config.subsystemName(), 0, Optional.of(invalid)));
    }
  }
}
