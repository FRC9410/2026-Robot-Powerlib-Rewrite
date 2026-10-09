package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class RobotAutosTest {
  @BeforeAll static void initialize() { HAL.initialize(500, 0); }

  @Test void everyRouteMatchesAllSevenApril4Waypoints() {
    double[][][] expected = {
      {{13.5,2.4,-90},{10,2.4,-90},{9.55,0.73,-90},{8.2,1,-90},{8.2,3.5,-90},{8.4,2.4,-90},{14.3,2.4,135}},
      {{13.5,5.7,90},{10,5.7,90},{9.55,7.27,90},{8.2,7,90},{8.2,4.5,90},{8.4,5.7,90},{14.3,5.7,-135}},
      {{3.1,5.7,-90},{6.5,5.7,-90},{6.95,7.27,-90},{8.2,7,-90},{8.2,4.5,-90},{8.1,5.7,-90},{2.1,5.7,135}},
      {{3.1,2.4,90},{6.5,2.4,90},{6.95,0.73,90},{8.2,1,90},{8.2,3.5,90},{8.1,2.4,90},{2.1,2.4,-135}}
    };
    assertEquals(4, RobotAutos.ROUTES.size());
    for (int r = 0; r < expected.length; r++) {
      var route = RobotAutos.ROUTES.get(r);
      assertEquals(7, route.points().size());
      for (int p = 0; p < 7; p++) {
        Pose2d point = route.points().get(p);
        assertEquals(expected[r][p][0], point.getX(), 1e-12);
        assertEquals(expected[r][p][1], point.getY(), 1e-12);
        assertEquals(expected[r][p][2], point.getRotation().getDegrees(), 1e-12);
      }
    }
    assertEquals(90, RobotAutos.ROUTES.get(0).fieldPoints().get(0).getRotation().getDegrees(), 1e-12);
    assertEquals(-45, RobotAutos.ROUTES.get(0).fieldPoints().get(6).getRotation().getDegrees(), 1e-12);
    assertEquals(-90, RobotAutos.ROUTES.get(1).fieldPoints().get(0).getRotation().getDegrees(), 1e-12);
    assertEquals(45, RobotAutos.ROUTES.get(1).fieldPoints().get(6).getRotation().getDegrees(), 1e-12);
  }

  @Test void everyAutoStartsIntakeAfterTwoLegsAndStopsItBeforeReturning() {
    for (var route : RobotAutos.ROUTES) {
      List<String> events = new ArrayList<>();
      var auto = RobotAutos.sequence(route,
          state -> Commands.runOnce(() -> events.add(state.name())),
          () -> Commands.runOnce(() -> events.add("turn")),
          () -> Commands.runOnce(() -> events.add("hold")));
      var command = auto.build(point -> Commands.runOnce(() -> events.add("drive" + route.fieldPoints().indexOf(point))));
      command.initialize();
      for (int i = 0; i < 20 && !command.isFinished(); i++) command.execute();
      assertTrue(command.isFinished());
      command.end(false);
      assertEquals(List.of("drive0", "drive1", "INTAKING", "drive2", "drive3", "drive4", "READY",
          "drive5", "drive6", "turn", "SHOOTING", "hold"), events);
    }
  }

  @Test void originalLegProfilesArePreserved() {
    assertArrayEquals(new double[] {6,6,12,6,6,3,3}, Constants.Auto.LEG_TOLERANCE_INCHES);
    assertArrayEquals(new double[] {0.5,1,0.75,0.75,0.4,0.75,1}, Constants.Auto.LEG_SPEED_COEFFICIENTS);
    assertArrayEquals(new boolean[] {true,false,true,true,true,true,false}, Constants.Auto.LEG_LOCK_HEADING);
    assertEquals(10, Constants.Auto.LEG_HEADING_TOLERANCE_DEGREES);
    assertEquals(3, Constants.Auto.STATIC_FEEDFORWARD_THRESHOLD_INCHES);
    assertEquals(3, Constants.Auto.TURN_TOLERANCE_DEGREES);
    assertEquals(1, Constants.Auto.TURN_TIMEOUT_SECONDS);
  }
}
