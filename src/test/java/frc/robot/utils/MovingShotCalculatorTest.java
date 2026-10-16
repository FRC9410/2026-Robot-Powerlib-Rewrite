package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.MathUtil;
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
      assertEquals(ShootingUtils.calculate(pose(alliance, row[0]), alliance, row[1] + 1, hood, true), shot);
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
    // Fresh calculators: each case must not inherit the previous case's filtered velocity.
    var location = pose(Alliance.Blue, 2.5);
    var tooFast = new MovingShotCalculator().calculate(location, new ChassisSpeeds(3.1, 0, 0),
        Alliance.Blue, 29.5, 0.05, true, true, 0, 0);
    assertFalse(tooFast.valid());
    assertEquals("Above SOTM speed limit", tooFast.status());
    var outsideLeadTable = new MovingShotCalculator().calculate(location, new ChassisSpeeds(-2, 0, 0),
        Alliance.Blue, 29.5, 0.05, true, true, 0, 0);
    assertFalse(outsideLeadTable.valid());
    assertFalse(outsideLeadTable.canFeed());
    var tiltedWhileDriving = new MovingShotCalculator().calculate(location, new ChassisSpeeds(0, 0.5, 0),
        Alliance.Blue, 29.5, 0.05, true, true, 6, 0);
    assertFalse(tiltedWhileDriving.valid());
    assertEquals("Robot tilt exceeds shooting limit", tiltedWhileDriving.status());
    assertFalse(new MovingShotCalculator().calculate(location, new ChassisSpeeds(Double.NaN, 0, 0), Alliance.Blue,
        29.5, 0.05, true, true, 0, 0).valid());
    assertFalse(new MovingShotCalculator().calculate(pose(Alliance.Blue, 1.9), new ChassisSpeeds(), Alliance.Blue,
        28.4, 0.039, true, true, 0, 0).valid());
  }

  @Test void notDrivingAlwaysUsesTheOriginalStandingShot() {
    // Turning in place to aim at long range: the rear launcher swings out past the calibrated range,
    // which used to block the shot and stop the shooter mid-turn.
    var far = pose(Alliance.Blue, 4.6);
    var turning = new Pose2d(far.getTranslation(), far.getRotation().plus(Rotation2d.fromDegrees(150)));
    var turningShot = new MovingShotCalculator().calculate(turning, new ChassisSpeeds(0, 0, 2), Alliance.Blue,
        36, 0.08, true, true, 0, 0);
    assertTrue(turningShot.valid(), turningShot.status());
    assertEquals(ShootingUtils.calculate(turning, Alliance.Blue, 36, 0.08, true), turningShot);
    // Slow creep plus rotation at the edge of the range is still treated as standing still.
    var edge = pose(Alliance.Blue, 4.9);
    assertEquals(ShootingUtils.calculate(edge, Alliance.Blue, 38, 0.085, true),
        new MovingShotCalculator().calculate(edge, new ChassisSpeeds(0, -0.09, 0.45), Alliance.Blue,
            38, 0.085, true, true, 0, 0));
    // A tilted Pigeon never blocks a standing shot.
    var location = pose(Alliance.Blue, 2.5);
    var tilted = new MovingShotCalculator().calculate(location, new ChassisSpeeds(), Alliance.Blue,
        29.5, 0.05, true, true, 8, -7);
    assertEquals(ShootingUtils.calculate(location, Alliance.Blue, 29.5, 0.05, true), tilted);
    assertTrue(tilted.canFeed());
  }

  @Test void leadFollowsFilteredVelocityAfterAStickStep() {
    var location = pose(Alliance.Blue, 3);
    var strafe = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0, 1, 0), location.getRotation());
    double settled = new MovingShotCalculator().calculate(location, strafe, Alliance.Blue,
        30.5, 0.06, true, true, 0, 0).headingDegrees();
    double stationary = ShootingUtils.headingDegrees(location, Alliance.Blue);
    var calculator = new MovingShotCalculator();
    calculator.calculate(location, new ChassisSpeeds(), Alliance.Blue, 30.5, 0.06, true, true, 0, 0);
    double firstLoop = calculator.calculate(location, strafe, Alliance.Blue,
        30.5, 0.06, true, true, 0, 0).headingDegrees();
    double lead = Math.abs(MathUtil.inputModulus(settled - stationary, -180, 180));
    double partial = Math.abs(MathUtil.inputModulus(firstLoop - stationary, -180, 180));
    // One loop after the step the lead has only started to build.
    assertTrue(partial > 0 && partial < 0.3 * lead, partial + " of " + lead);
    double latest = firstLoop;
    for (int i = 0; i < 50; i++) {
      latest = calculator.calculate(location, strafe, Alliance.Blue, 30.5, 0.06, true, true, 0, 0).headingDegrees();
    }
    assertEquals(settled, latest, 0.01);
    calculator.reset();
    double afterReset = calculator.calculate(location, new ChassisSpeeds(), Alliance.Blue,
        30.5, 0.06, true, true, 0, 0).headingDegrees();
    assertEquals(0, MathUtil.inputModulus(afterReset - stationary, -180, 180), 1e-6);
  }

  @Test void killSwitchRestoresTheStationaryShotWhileMoving() {
    var location = pose(Alliance.Blue, 3);
    var strafe = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0, 1, 0), location.getRotation());
    boolean previous = ShootingUtils.sotmEnabled();
    try {
      ShootingUtils.setSotmEnabled(false);
      var shot = new MovingShotCalculator().calculate(location, strafe, Alliance.Blue,
          30.5, 0.06, true, true, 0, 0);
      assertEquals(ShootingUtils.calculate(location, Alliance.Blue, 30.5, 0.06, true), shot);
      assertFalse(shot.solverValid());
      assertFalse(shot.motionCompensated());
    } finally {
      ShootingUtils.setSotmEnabled(previous);
    }
  }
}
