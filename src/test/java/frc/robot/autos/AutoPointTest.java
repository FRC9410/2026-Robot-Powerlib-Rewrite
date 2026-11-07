package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class AutoPointTest {
  @Test void paramsObjectAndEachFieldCanBeOmitted() {
    var point = new AutoPoint(new Pose2d());
    assertEquals(AutoPoint.Params.DEFAULT, point.params());
    assertEquals(point, new AutoPoint(new Pose2d(), null));
    assertNull(point.params().endDistanceToleranceMeters());
    assertNull(point.params().endSpeedToleranceMetersPerSecond());
    assertNull(point.params().maxAccelerationMetersPerSecondSquared());
    assertNull(point.params().maxSpeedMetersPerSecond());
    var speedOnly = new AutoPoint.Params().withMaxSpeedMetersPerSecond(3);
    assertNull(speedOnly.endDistanceToleranceMeters());
    assertNull(speedOnly.endSpeedToleranceMetersPerSecond());
    assertNull(speedOnly.maxAccelerationMetersPerSecondSquared());
    assertEquals(3, speedOnly.maxSpeedMetersPerSecond());
  }

  @Test void fluentOverridesPreserveEachOtherWithoutChangingSharedParams() {
    var original = new AutoPoint.Params().withEndDistanceToleranceMeters(.5);
    var configured = original.withEndSpeedToleranceMetersPerSecond(1)
        .withMaxAccelerationMetersPerSecondSquared(2).withMaxSpeedMetersPerSecond(3);
    var point = new AutoPoint(new Pose2d(4, 4, Rotation2d.fromDegrees(180)), configured);
    assertEquals(.5, point.params().endDistanceToleranceMeters());
    assertEquals(1, point.params().endSpeedToleranceMetersPerSecond());
    assertEquals(2, point.params().maxAccelerationMetersPerSecondSquared());
    assertEquals(3, point.params().maxSpeedMetersPerSecond());
    assertNull(original.endSpeedToleranceMetersPerSecond());
    assertNull(original.maxAccelerationMetersPerSecondSquared());
    assertNull(original.maxSpeedMetersPerSecond());
    assertEquals(.5, original.endDistanceToleranceMeters());
    assertNull(AutoPoint.Params.DEFAULT.endDistanceToleranceMeters());
  }

  @Test void rejectsValuesThatCannotDefineFinitePositiveLimits() {
    for (double value : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      var params = new AutoPoint.Params();
      assertThrows(IllegalArgumentException.class, () -> params.withEndDistanceToleranceMeters(value));
      assertThrows(IllegalArgumentException.class, () -> params.withEndSpeedToleranceMetersPerSecond(value));
      assertThrows(IllegalArgumentException.class, () -> params.withMaxAccelerationMetersPerSecondSquared(value));
      assertThrows(IllegalArgumentException.class, () -> params.withMaxSpeedMetersPerSecond(value));
    }
    assertThrows(IllegalArgumentException.class, () -> new AutoPoint(null));
    assertThrows(IllegalArgumentException.class, () -> new AutoPoint(new Pose2d(Double.NaN, 0, new Rotation2d())));
  }

  @Test void originalSideAndCenterRoutesAllUseUnmodifiedDefaults() {
    var points = Stream.concat(RobotAutos.ROUTES.stream(), RobotAutos.CENTER_ROUTES.stream())
        .flatMap(route -> route.points().stream()).toList();
    assertEquals(32, points.size());
    points.forEach(point -> assertEquals(AutoPoint.Params.DEFAULT, point.params()));
    assertEquals(new Pose2d(14, 4.05, Rotation2d.fromDegrees(180)), RobotAutos.CENTER_ROUTES.get(0).points().get(0).pose());
    assertEquals(new Pose2d(1.3, 6.15, Rotation2d.fromDegrees(180)), RobotAutos.CENTER_ROUTES.get(1).points().get(1).pose());
  }
}
