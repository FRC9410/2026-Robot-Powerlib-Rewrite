package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class MovingShotCalculatorTest {
  private static Pose2d pose(Alliance alliance, double distance) {
    var hub = alliance == Alliance.Red ? Constants.Shooting.RED_HUB : Constants.Shooting.BLUE_HUB;
    double xDistance = Math.min(distance, 4);
    var location = new Pose2d(hub.getX() + (alliance == Alliance.Red ? xDistance : -xDistance),
        hub.getY() + Math.sqrt(distance * distance - xDistance * xDistance), new Rotation2d());
    return new Pose2d(location.getTranslation(), Rotation2d.fromDegrees(ShootingUtils.headingDegrees(location, alliance)));
  }

  @Test void everyStationaryRowPreservesExistingCalibrationForBothAlliances() {
    for (var alliance : Alliance.values()) for (double[] row : Constants.Shooting.SHOOTER_SPEEDS) {
      double hood = Constants.Shooting.HOOD_ANGLE_INTERPOLATOR.getInterpolatedValue(row[0])
          + Constants.StateMachine.HOOD_SHOT_OFFSET_ROTATIONS;
      var shot = new MovingShotCalculator().calculate(pose(alliance, row[0]), new ChassisSpeeds(), alliance,
          row[1] + 1, hood, true, true, 0, 0);
      assertTrue(shot.valid(), shot.status());
      assertTrue(shot.solverValid(), shot.status());
      assertFalse(shot.motionCompensated());
      assertTrue(shot.canFeed());
      assertEquals(row[1] + 1, shot.shooterRps(), 1e-8);
      assertEquals(hood, shot.hoodRotations(), 1e-8);
      assertEquals(row[0], shot.effectiveDistanceMeters(), 1e-8);
    }
  }

  @Test void robotRelativeSpeedsAreConvertedBeforeLeadingTheHub() {
    var location = pose(Alliance.Blue, 3);
    // Blue shooter faces east, robot front west: robot -Y is field +Y.
    var shot = new MovingShotCalculator().calculate(location, new ChassisSpeeds(0, -0.5, 0),
        Alliance.Blue, 30.5, 0.06, true, true, 0, 0);
    assertTrue(shot.valid(), shot.status());
    assertTrue(shot.motionCompensated());
    assertTrue(shot.headingDegrees() < 180 && shot.headingDegrees() > 150);
    assertTrue(shot.headingFeedforwardRadiansPerSecond() < 0);
    assertTrue(shot.effectiveDistanceMeters() > shot.distanceMeters());
  }

  @Test void approachLowersSetpointsAndRetreatRaisesThem() {
    var location = pose(Alliance.Blue, 3);
    var approach = new MovingShotCalculator().calculate(location, new ChassisSpeeds(-0.5, 0, 0),
        Alliance.Blue, 30.5, 0.06, true, true, 0, 0);
    var retreat = new MovingShotCalculator().calculate(location, new ChassisSpeeds(0.5, 0, 0),
        Alliance.Blue, 30.5, 0.06, true, true, 0, 0);
    assertTrue(approach.valid(), approach.status());
    assertTrue(retreat.valid(), retreat.status());
    assertTrue(approach.shooterRps() < 30.5);
    assertTrue(retreat.shooterRps() > 30.5);
  }

  @Test void feedReadinessUsesTheCompensatedHeadingAndFreshMechanismFeedback() {
    var calculator = new MovingShotCalculator();
    var location = pose(Alliance.Blue, 2.5);
    var worldVelocity = new ChassisSpeeds(0, 0.5, 0);
    var first = calculator.calculate(location,
        ChassisSpeeds.fromFieldRelativeSpeeds(worldVelocity, location.getRotation()),
        Alliance.Blue, 29.5, 0.05, true, true, 0, 0);
    assertTrue(first.valid(), first.status());
    assertFalse(first.aligned());
    assertFalse(first.canFeed());
    var aligned = new Pose2d(location.getTranslation(), Rotation2d.fromDegrees(first.headingDegrees()));
    var ready = calculator.calculate(aligned,
        ChassisSpeeds.fromFieldRelativeSpeeds(worldVelocity, aligned.getRotation()),
        Alliance.Blue, first.shooterRps(), first.hoodRotations(), true, true, 0, 0);
    assertTrue(ready.canFeed(), ready.status());
    var unhealthy = calculator.calculate(aligned,
        ChassisSpeeds.fromFieldRelativeSpeeds(worldVelocity, aligned.getRotation()),
        Alliance.Blue, ready.shooterRps(), ready.hoodRotations(), false, true, 0, 0);
    assertFalse(unhealthy.canFeed());
  }

  @Test void unsafeMovingSolutionsNeverFallBackToFeedingStaticShots() {
    var calculator = new MovingShotCalculator();
    var location = pose(Alliance.Blue, 2.5);
    var tooFast = calculator.calculate(location, new ChassisSpeeds(3.1, 0, 0),
        Alliance.Blue, 29.5, 0.05, true, true, 0, 0);
    assertFalse(tooFast.valid());
    assertEquals("Above SOTM speed limit", tooFast.status());
    // Blocked shots never feed but keep the static setpoints so the flywheel stays spun up.
    var stationary = ShootingUtils.calculate(location, Alliance.Blue, 29.5, 0.05, true);
    assertTrue(tooFast.hasSetpoints());
    assertEquals(stationary.shooterRps(), tooFast.shooterRps(), 1e-9);
    assertEquals(stationary.hoodRotations(), tooFast.hoodRotations(), 1e-9);
    assertEquals(0, tooFast.feederRps());
    var outsideLeadTable = calculator.calculate(location, new ChassisSpeeds(-2, 0, 0),
        Alliance.Blue, 29.5, 0.05, true, true, 0, 0);
    assertFalse(outsideLeadTable.valid());
    assertFalse(outsideLeadTable.canFeed());
    assertFalse(calculator.calculate(location, new ChassisSpeeds(), Alliance.Blue,
        29.5, 0.05, true, true, 6, 0).valid());
    assertFalse(calculator.calculate(location, new ChassisSpeeds(Double.NaN, 0, 0), Alliance.Blue,
        29.5, 0.05, true, true, 0, 0).valid());
    assertFalse(calculator.calculate(pose(Alliance.Blue, 1.9), new ChassisSpeeds(), Alliance.Blue,
        28.4, 0.039, true, true, 0, 0).valid());
    // Center translation and rotation are each slow, but add to 0.18 m/s at the launcher.
    // At maximum range the necessary lateral lead falls outside calibration.
    var rotatingAtRangeEdge = calculator.calculate(pose(Alliance.Blue, 4.9),
        new ChassisSpeeds(0, -0.09, 0.45), Alliance.Blue, 38, 0.085, true, true, 0, 0);
    assertFalse(rotatingAtRangeEdge.valid());
    assertFalse(rotatingAtRangeEdge.canFeed());
  }
}
