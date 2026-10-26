package frc.robot.dashboard;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.powerlib.PowerRobotContainer;
import frc.robot.utils.ShootingUtils;
import org.junit.jupiter.api.Test;

class SotmSwitchTest {
  @Test void dashboardRequestsToggleTheSwitchWithTuningOff() {
    boolean previous = ShootingUtils.sotmEnabled();
    boolean tuning = PowerRobotContainer.isTuningEnabled();
    var publisher = NetworkTableInstance.getDefault().getBooleanTopic(SotmSwitch.REQUEST_TOPIC)
        .publish(edu.wpi.first.networktables.PubSubOption.keepDuplicates(true));
    try {
      PowerRobotContainer.setTuningEnabled(false);
      ShootingUtils.setSotmEnabled(true);
      SotmSwitch.poll();
      publisher.set(false);
      SotmSwitch.poll();
      assertFalse(ShootingUtils.sotmEnabled());
      // No new request: the switch keeps its state even if code changes it.
      ShootingUtils.setSotmEnabled(true);
      SotmSwitch.poll();
      assertTrue(ShootingUtils.sotmEnabled());
      // Repeating the same request still applies.
      publisher.set(false);
      SotmSwitch.poll();
      assertFalse(ShootingUtils.sotmEnabled());
      publisher.set(false);
      publisher.set(true);
      SotmSwitch.poll();
      assertTrue(ShootingUtils.sotmEnabled());
    } finally {
      publisher.close();
      PowerRobotContainer.setTuningEnabled(tuning);
      ShootingUtils.setSotmEnabled(previous);
    }
  }
}
