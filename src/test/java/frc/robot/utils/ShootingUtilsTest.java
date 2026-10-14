package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class ShootingUtilsTest {
  private static Pose2d bluePose(double distance, double heading) {
    return new Pose2d(Constants.Shooting.BLUE_HUB.getX() - distance,
        Constants.Shooting.BLUE_HUB.getY(), Rotation2d.fromDegrees(180 + heading));
  }

  @Test void interpolatesCommittedTablesAndPreservesFeederDirection() {
    var shot = ShootingUtils.calculate(bluePose(2.75, 0), Alliance.Blue, 30, 0.05, true);
    assertTrue(shot.valid());
    assertEquals(30, shot.shooterRps(), 1e-9);
    assertEquals(0.055, shot.hoodRotations(), 1e-9);
    assertEquals(-60, shot.feederRps(), 1e-9);
    assertTrue(shot.canFeed());
  }

  @Test void matchesEveryCommittedShotTableRow() {
    for (double[] row : Constants.Shooting.SHOOTER_SPEEDS) {
      double hood = Constants.Shooting.HOOD_ANGLE_INTERPOLATOR.getInterpolatedValue(row[0]) - 0.005;
      double shooter = row[1] + Constants.StateMachine.SHOOTER_EXTRA_RPS;
      double xDistance = Math.min(row[0], 4);
      Pose2d location = new Pose2d(Constants.Shooting.BLUE_HUB.getX() - xDistance,
          Constants.Shooting.BLUE_HUB.getY() + Math.sqrt(row[0] * row[0] - xDistance * xDistance), new Rotation2d());
      Pose2d pose = new Pose2d(location.getTranslation(), Rotation2d.fromDegrees(ShootingUtils.headingDegrees(location, Alliance.Blue)));
      var shot = ShootingUtils.calculate(pose, Alliance.Blue, shooter, hood, true);
      assertTrue(shot.valid());
      assertEquals(shooter, shot.shooterRps(), 1e-9);
      assertEquals(hood, shot.hoodRotations(), 1e-9);
    }
  }

  @Test void rejectsUnknownAllianceBadPoseAndOutOfRange() {
    assertFalse(ShootingUtils.calculate(bluePose(2.5, 0), null, 0, 0, true).valid());
    assertFalse(ShootingUtils.calculate(new Pose2d(Double.NaN, 4, new Rotation2d()), Alliance.Blue, 0, 0, true).valid());
    assertFalse(ShootingUtils.calculate(bluePose(1.949, 0), Alliance.Blue, 28.4, 0.039, true).valid());
    var closeShot = ShootingUtils.calculate(bluePose(1.95, 0), Alliance.Blue, 28.4, 0.039, true);
    assertTrue(closeShot.canFeed());
    assertEquals(28.4, closeShot.shooterRps(), 1e-9);
    assertEquals(0.039, closeShot.hoodRotations(), 1e-9);
    assertFalse(ShootingUtils.calculate(new Pose2d(6, 4, new Rotation2d()), Alliance.Blue, 0, 0, true).valid());
    assertFalse(ShootingUtils.calculate(new Pose2d(0.01, 0, new Rotation2d()), Alliance.Blue, 0, 0, true).valid());
  }

  @Test void readinessRequiresVelocityHoodAlignmentAndHealthyFeedback() {
    assertTrue(ShootingUtils.calculate(bluePose(2.5, 0), Alliance.Blue, 29.5, 0.05, true).canFeed());
    assertFalse(ShootingUtils.calculate(bluePose(2.5, 0), Alliance.Blue, 0, 0.05, true).canFeed());
    assertFalse(ShootingUtils.calculate(bluePose(2.5, 0), Alliance.Blue, 29.5, 0.12, true).canFeed());
    assertFalse(ShootingUtils.calculate(bluePose(2.5, 10), Alliance.Blue, 29.5, 0.05, true).canFeed());
    assertFalse(ShootingUtils.calculate(bluePose(2.5, 0), Alliance.Blue, 29.5, 0.05, false).canFeed());
    assertFalse(ShootingUtils.calculate(bluePose(2.5, 0), Alliance.Blue, Double.NaN, 0.05, true).canFeed());
  }

  @Test void usesApril4RearFacingShooterOrientationAndWrapsHeadingError() {
    var shot = ShootingUtils.calculate(new Pose2d(Constants.Shooting.RED_HUB.getX() + 2.5,
        Constants.Shooting.RED_HUB.getY(), new Rotation2d()), Alliance.Red, 29.5, 0.05, true);
    assertEquals(0, shot.headingDegrees(), 1e-9);
    assertEquals(0, shot.headingErrorDegrees(), 1e-9);
    assertTrue(shot.canFeed());
    assertEquals(179, ShootingUtils.calculate(bluePose(2.5, -179), Alliance.Blue, 29.5, 0.05, true).headingErrorDegrees(), 1e-9);
  }

}
