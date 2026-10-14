package frc.robot.dashboard;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

class HubTimerTest {
  @Test void bothHubsAreActiveInAutonomousTransitionAndEndgame() {
    assertEquals(true, HubTimer.calculate(Alliance.Blue, true, false, 12, "").active());
    assertEquals(12, HubTimer.calculate(Alliance.Blue, true, false, 12, "").secondsRemaining());
    assertEquals(true, HubTimer.calculate(Alliance.Red, false, true, 135, "").active());
    assertEquals(5, HubTimer.calculate(Alliance.Red, false, true, 135, "").secondsRemaining());
    assertEquals(true, HubTimer.calculate(Alliance.Red, false, true, 30, "").active());
    assertEquals(30, HubTimer.calculate(Alliance.Red, false, true, 30, "").secondsRemaining());
  }

  @Test void firstInactiveAllianceAlternatesAtExactShiftBoundaries() {
    for (Alliance alliance : Alliance.values()) {
      for (String data : new String[] {"R", "B"}) {
        boolean inactiveFirst = data.equals(alliance == Alliance.Red ? "R" : "B");
        double[] times = {130, 105, 80, 55};
        for (int i = 0; i < times.length; i++) {
          var window = HubTimer.calculate(alliance, false, true, times[i], data);
          assertEquals(i % 2 == 0 ? !inactiveFirst : inactiveFirst, window.active());
          assertEquals(25, window.secondsRemaining());
          assertEquals("Shift " + (i + 1), window.phase());
        }
      }
    }
  }

  @Test void missingOrInvalidInputsNeverInventHubStatus() {
    for (String data : new String[] {"", "?", "red"}) {
      assertNull(HubTimer.calculate(Alliance.Red, false, true, 100, data).active());
    }
    for (double time : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY, 141}) {
      assertNull(HubTimer.calculate(Alliance.Blue, false, true, time, "R").active());
    }
    assertNull(HubTimer.calculate(null, false, true, 100, "R").active());
    assertNull(HubTimer.calculate(Alliance.Red, false, false, 100, "R").active());
    assertTrue(Double.isNaN(HubTimer.calculate(Alliance.Blue, true, false, -1, "").secondsRemaining()));
  }
}
