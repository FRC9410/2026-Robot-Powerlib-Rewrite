package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import org.junit.jupiter.api.Test;

class FieldGeometryTest {
  @Test void geometryMatchesInstalledOfficial2026WeldedLayout() {
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    assertEquals(layout.getFieldLength(), FieldConstants.X_MAX, 1e-9);
    assertEquals(layout.getFieldWidth(), FieldConstants.Y_MAX, 1e-9);
    assertEquals(FieldConstants.X_MAX, FieldConstants.RED_END_X);
    var red = layout.getTagPose(4).orElseThrow().toPose2d().getTranslation()
        .plus(layout.getTagPose(10).orElseThrow().toPose2d().getTranslation()).div(2);
    var blue = layout.getTagPose(20).orElseThrow().toPose2d().getTranslation()
        .plus(layout.getTagPose(26).orElseThrow().toPose2d().getTranslation()).div(2);
    assertEquals(0, red.getDistance(FieldConstants.HOPPER_RED), 1e-9);
    assertEquals(0, blue.getDistance(FieldConstants.HOPPER_BLUE), 1e-9);
  }

  @Test void screenshotTargetsCorrectRedHubAndFallsBelowMinimumRange() {
    var pose = new Pose2d(13.27, 3.18, Rotation2d.fromDegrees(-26));
    assertEquals(1.601567604, ShootingUtils.distanceMeters(pose, Alliance.Red), 1e-8);
    assertEquals(-32.250756276, ShootingUtils.headingDegrees(pose, Alliance.Red), 1e-8);
    assertFalse(ShootingUtils.calculate(pose, Alliance.Red, 28.4, 0.039, true).valid());
  }

  @Test void shootingUsesFullOfficialFieldWidthAndRejectsPositionsBeyondIt() {
    var inside = new Pose2d(2, 8.03, new Rotation2d());
    assertTrue(ShootingUtils.calculate(inside, Alliance.Blue, 0, 0, true).valid());
    assertFalse(ShootingUtils.calculate(new Pose2d(2, 8.07, new Rotation2d()),
        Alliance.Blue, 0, 0, true).valid());
  }
}
