package frc.powerlib.dashboard;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.powerlib.PowerRobotContainer;
import org.junit.jupiter.api.Test;

class SubsystemTelemetryTest {
  @Test void publishesOnlyLatestCachedReadingsAtOneHundredMillisecondIntervals() {
    var table = NetworkTableInstance.getDefault().getTable("PowerLib/Subsystems/CadenceTest/Data");
    try {
      PowerRobotContainer.setSubsystemData("CadenceTest", "Velocity", 10.0);
      PowerRobotContainer.setSubsystemData("CadenceTest", "Connected", true);
      PowerRobotContainer.setSubsystemData("CadenceTest", "State", "READY");
      SubsystemTelemetry.publish(0.0);
      assertEquals(10.0, table.getEntry("Velocity").getDouble(-1));
      assertTrue(table.getEntry("Connected").getBoolean(false));
      assertEquals("READY", table.getEntry("State").getString(""));

      for (int tick = 1; tick < 5; tick++) {
        PowerRobotContainer.setSubsystemData("CadenceTest", "Velocity", 10.0 + tick);
        SubsystemTelemetry.publish(tick * 0.02);
        assertEquals(10.0, table.getEntry("Velocity").getDouble(-1));
      }
      PowerRobotContainer.setSubsystemData("CadenceTest", "Velocity", 15.0);
      SubsystemTelemetry.publish(0.1);
      assertEquals(15.0, table.getEntry("Velocity").getDouble(-1));

      PowerRobotContainer.setSubsystemData("CadenceTest", "Velocity", 20.0);
      SubsystemTelemetry.publish(0.199);
      assertEquals(15.0, table.getEntry("Velocity").getDouble(-1));
      SubsystemTelemetry.publish(0.2);
      assertEquals(20.0, table.getEntry("Velocity").getDouble(-1));

      // Resuming after a pause sends one latest sample; a clock reset sends immediately.
      PowerRobotContainer.setSubsystemData("CadenceTest", "Velocity", 30.0);
      SubsystemTelemetry.publish(10.0);
      assertEquals(30.0, table.getEntry("Velocity").getDouble(-1));
      PowerRobotContainer.setSubsystemData("CadenceTest", "Velocity", 40.0);
      SubsystemTelemetry.publish(0.0);
      assertEquals(40.0, table.getEntry("Velocity").getDouble(-1));
    } finally {
      PowerRobotContainer.getAllSubsystemData().remove("CadenceTest");
    }
  }
}
