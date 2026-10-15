package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class AimingDriveControllerTest {
  @Test void speedCapPreservesDirectionAndLimitsDiagonalCommands() {
    var limited = AimingDriveController.limitTranslation(3, 4, 1.5);
    assertEquals(1.5, limited.getNorm(), 1e-9);
    assertEquals(0.9, limited.getX(), 1e-9);
    assertEquals(1.2, limited.getY(), 1e-9);
    assertEquals(0.01, AimingDriveController.limitTranslation(0.01, 0, 1.5).getX());
    assertEquals(0, AimingDriveController.limitTranslation(1, 1, 0).getNorm());
    assertThrows(IllegalArgumentException.class,
        () -> AimingDriveController.limitTranslation(1, 0, Double.NaN));
  }

  @Test void stationaryAlignedWheelsParkWithHysteresisAndResumeToCorrectDrift() {
    var controller = new AimingDriveController();
    var stopped = new ChassisSpeeds();
    assertFalse(controller.updateParking(0, 0, stopped, 1.1));
    assertTrue(controller.updateParking(0, 0, stopped, 0.9));
    assertTrue(controller.updateParking(0, 0, stopped, -1.9));
    assertFalse(controller.updateParking(0, 0, stopped, 2.1));
    assertFalse(controller.updateParking(0, 0, stopped, 1.5));
    assertTrue(controller.updateParking(0, 0, stopped, 0));
    controller.reset();
    assertFalse(controller.updateParking(0, 0, stopped, 1.5));
  }

  @Test void driverIntentMotionAndInvalidFeedbackAlwaysUnpark() {
    var controller = new AimingDriveController();
    assertTrue(controller.updateParking(0, 0, new ChassisSpeeds(), 0));
    assertFalse(controller.updateParking(0.03, 0, new ChassisSpeeds(), 0));
    assertFalse(controller.updateParking(0, 0, new ChassisSpeeds(0.06, 0, 0), 0));
    assertFalse(controller.updateParking(0, 0, new ChassisSpeeds(0, 0, Math.toRadians(6)), 0));
    assertFalse(controller.updateParking(0, 0, new ChassisSpeeds(), Double.NaN));
  }
}
