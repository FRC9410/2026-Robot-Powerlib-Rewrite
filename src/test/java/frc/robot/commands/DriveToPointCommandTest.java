package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class DriveToPointCommandTest {
  private static final Pose2d DESTINATION = pose(4, 4, 180);

  @Test void endDistanceHeadingAndMeasuredSpeedAreRequiredTogether() {
    assertTrue(reached(pose(3.6, 4, 180), 4.9, 0, 5.0));
    assertFalse(reached(pose(3, 4, 180), 0, 0, 5.0));
    assertFalse(reached(pose(3.6, 4, 140), 0, 0, 5.0));
    assertFalse(reached(DESTINATION, 5.1, 0, 5.0));
    assertFalse(reached(DESTINATION, 5, 0, 5.0));
    assertFalse(reached(DESTINATION, 4, 4, 5.0));
    assertTrue(reached(DESTINATION, -3, -3, 5.0));
    assertFalse(reached(DESTINATION, Double.NaN, 0, 5.0));
    assertFalse(reached(DESTINATION, 0, Double.POSITIVE_INFINITY, 5.0));
  }

  @Test void omittedEndSpeedAndOriginalHeadingFlagsKeepOriginalCompletionBehavior() {
    assertTrue(reached(DESTINATION, 5.5, 0, null));
    assertTrue(DriveToPointCommand.atDestination(pose(3.6, 4, 0), DESTINATION, .5, false,
        new ChassisSpeeds(), null));
    assertFalse(reached(pose(3.4, 4, 180), 0, 0, null));
    assertFalse(reached(pose(Double.NaN, 4, 180), 0, 0, null));
    assertTrue(reached(pose(3.6, 4, -180), 0, 0, 5.0));
  }

  @Test void explicitTravelSpeedReplacesTheLegCapButRespectsRobotAndGlobalCeilings() {
    assertEquals(5.72*.4, DriveToPointCommand.speedLimit(5.72, 1, .4, null), 1e-12);
    assertEquals(3, DriveToPointCommand.speedLimit(5.72, 1, .4, 3.0), 1e-12);
    assertEquals(5.72, DriveToPointCommand.speedLimit(5.72, 1, .4, 10.0), 1e-12);
    assertEquals(2.86, DriveToPointCommand.speedLimit(5.72, .5, .4, 3.0), 1e-12);
    assertEquals(5.72, DriveToPointCommand.speedLimit(5.72, 1.2, .4, 10.0), 1e-12);
  }

  @Test void accelerationLimitAppliesToTheVectorForLaunchTurningAndBraking() {
    var origin = new Translation2d();
    var launch = DriveToPointCommand.limitVelocity(origin, new Translation2d(3, 4), 2, .02);
    assertEquals(.04, launch.getNorm(), 1e-12);
    assertEquals(.024, launch.getX(), 1e-12);
    assertEquals(.032, launch.getY(), 1e-12);
    var moving = new Translation2d(3, 0);
    var turn = DriveToPointCommand.limitVelocity(moving, new Translation2d(0, 3), 2, .02);
    assertEquals(.04, turn.minus(moving).getNorm(), 1e-12);
    assertTrue(turn.getNorm() <= 3);
    var brake = DriveToPointCommand.limitVelocity(moving, origin, 2, .02);
    assertEquals(2.96, brake.getX(), 1e-12);
  }

  @Test void enteringAboveASpeedCapBrakesProgressivelyInsteadOfJumpingToTheCap() {
    var velocity = new Translation2d(4, 0);
    var cappedTarget = new Translation2d(2, 0);
    for (int tick = 0; tick < 60; tick++) {
      var next = DriveToPointCommand.limitVelocity(velocity, cappedTarget, 2, .02);
      assertTrue(next.minus(velocity).getNorm() <= .04 + 1e-12);
      assertTrue(next.getX() >= 2 - 1e-12);
      if (tick == 0) assertEquals(3.96, next.getX(), 1e-12);
      velocity = next;
    }
    assertEquals(2, velocity.getX(), 1e-12);
  }

  @Test void variableLoopTimesDoNotOvershootTheRequestedVectorChange() {
    var velocity = new Translation2d();
    double[] periods = {.01, .02, .04};
    for (int tick = 0; tick < 150; tick++) {
      double dt = periods[tick % periods.length];
      var target = tick < 50 ? new Translation2d(3, 0)
          : tick < 100 ? new Translation2d(0, 3) : new Translation2d();
      var next = DriveToPointCommand.limitVelocity(velocity, target, 2, dt);
      assertTrue(next.minus(velocity).getNorm() <= 2*dt + 1e-12);
      assertTrue(next.getNorm() <= 3 + 1e-12);
      velocity = next;
    }
    assertEquals(0, velocity.getNorm(), 1e-12);
    var moving = new Translation2d(1, 2);
    assertEquals(moving, DriveToPointCommand.limitVelocity(moving, new Translation2d(), 2, 0));
    assertEquals(moving, DriveToPointCommand.limitVelocity(moving, new Translation2d(), 2, -.01));
  }

  private static boolean reached(Pose2d current, double vx, double vy, Double tolerance) {
    return DriveToPointCommand.atDestination(current, DESTINATION, .5, true,
        new ChassisSpeeds(vx, vy, 0), tolerance);
  }

  private static Pose2d pose(double x, double y, double degrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }
}
