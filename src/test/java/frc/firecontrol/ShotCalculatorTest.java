package frc.firecontrol;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
  private static ShotCalculator.Config config() {
    var c = new ShotCalculator.Config();
    c.launcherOffsetX = c.launcherOffsetY = 0;
    c.phaseDelayMs = c.mechLatencyMs = c.sotmDragCoeff = 0;
    c.minScoringDistance = 1;
    c.maxScoringDistance = 8;
    return c;
  }

  private static ShotCalculator calculator(ShotCalculator.Config c) {
    var solver = new ShotCalculator(c);
    var lut = new ShotLUT();
    lut.put(1, 12, 10, 1);
    lut.put(8, 26, 20, 1);
    solver.loadShotLUT(lut);
    return solver;
  }

  private static ShotCalculator.ShotInputs inputs(double vx, double vy) {
    return new ShotCalculator.ShotInputs(new Pose2d(), new ChassisSpeeds(vx, vy, 0),
        new ChassisSpeeds(vx, vy, 0), new Translation2d(4, 0), new Translation2d(), 1);
  }

  @Test void sidewaysLeadCancelsInheritedVelocityAtTheTarget() {
    var solver = calculator(config());
    var result = solver.calculate(inputs(0, 1));
    assertTrue(result.isValid());
    assertEquals(Math.sqrt(17), result.solvedDistanceM(), 1e-9);
    assertEquals(-Math.atan2(1, 4), result.driveAngle().getRadians(), 1e-9);
    assertEquals(10 + 2 * Math.sqrt(17), result.shooterRps(), 1e-9);
    assertEquals(-4.0 / 17, result.driveAngularVelocityRadPerSec(), 1e-9);
    var aim = new Translation2d(result.solvedDistanceM(), result.driveAngle());
    var landing = aim.plus(new Translation2d(0, result.timeOfFlightSec()));
    assertEquals(4, landing.getX(), 1e-9);
    assertEquals(0, landing.getY(), 1e-9);
  }

  @Test void approachAndRetreatChooseDifferentEffectiveDistances() {
    assertEquals(3, calculator(config()).calculate(inputs(1, 0)).solvedDistanceM(), 1e-9);
    assertEquals(5, calculator(config()).calculate(inputs(-1, 0)).solvedDistanceM(), 1e-9);
  }

  @Test void releaseLatencyIsAppliedOnceWithoutAnArtificialFirstSampleAcceleration() {
    var c = config();
    c.mechLatencyMs = 100;
    var solver = calculator(c);
    var first = solver.calculate(inputs(0, 1));
    var second = solver.calculate(inputs(0, 1));
    assertTrue(first.isValid());
    assertEquals(-Math.atan2(1.1, 4), first.driveAngle().getRadians(), 1e-9);
    assertEquals(first.driveAngle(), second.driveAngle());
    solver.resetWarmStart();
    assertEquals(first.driveAngle(), solver.calculate(inputs(0, 1)).driveAngle());
  }

  @Test void angularVelocityAtRearLauncherProducesLateralLead() {
    var c = config();
    c.launcherOffsetX = -0.2;
    c.shooterAngleOffsetRad = Math.PI;
    var result = calculator(c).calculate(new ShotCalculator.ShotInputs(
        new Pose2d(0, 0, Rotation2d.fromDegrees(180)), new ChassisSpeeds(0, 0, 1),
        new ChassisSpeeds(0, 0, 1), new Translation2d(4, 0), new Translation2d(), 1));
    assertTrue(result.isValid());
    assertEquals(Math.sqrt(3.8 * 3.8 + 0.2 * 0.2), result.solvedDistanceM(), 1e-9);
    assertEquals(180 - Math.toDegrees(Math.atan2(0.2, 3.8)), result.driveAngle().getDegrees(), 1e-9);
  }

  @Test void velocityStepsDoNotAddAnEstimatedAccelerationLead() {
    var c = config();
    c.mechLatencyMs = 100;
    var solver = calculator(c);
    assertTrue(solver.calculate(inputs(0, 0)).isValid());
    var accelerating = solver.calculate(inputs(0, 0.2));
    assertTrue(accelerating.isValid());
    // Release Y = .2*.1 and inherited speed .2 over one second of flight; no acceleration term.
    assertEquals(-Math.atan2(0.22, 4), accelerating.driveAngle().getRadians(), 1e-9);
  }

  @Test void invalidMotionEmptyLutTiltAndUncalibratedProjectedRangeAreRejected() {
    var c = config();
    assertFalse(new ShotCalculator(c).calculate(inputs(0, 0)).isValid());
    assertFalse(calculator(c).calculate(inputs(Double.NaN, 0)).isValid());
    assertFalse(calculator(c).calculate(inputs(3.01, 0)).isValid());
    assertFalse(calculator(c).calculate(new ShotCalculator.ShotInputs(new Pose2d(),
        new ChassisSpeeds(), new ChassisSpeeds(), null, new Translation2d(), 1)).isValid());
    assertFalse(calculator(c).calculate(new ShotCalculator.ShotInputs(new Pose2d(),
        new ChassisSpeeds(), new ChassisSpeeds(), new Translation2d(4, 0),
        new Translation2d(), 1, 6, 0)).isValid());
    assertFalse(calculator(c).calculate(new ShotCalculator.ShotInputs(new Pose2d(),
        new ChassisSpeeds(-2, 0, 0), new ChassisSpeeds(-2, 0, 0), new Translation2d(7, 0),
        new Translation2d(), 1)).isValid());
  }

  @Test void aClampedUnconvergedFlightTimeCannotBeMarkedValid() {
    var c = config();
    c.tofMin = 1;
    var solver = new ShotCalculator(c);
    var lut = new ShotLUT();
    lut.put(1, 12, 10, 0.5);
    lut.put(8, 26, 20, 1.9);
    solver.loadShotLUT(lut);
    assertFalse(solver.calculate(inputs(1, 0)).isValid());
  }

  @Test void lutCountsDistinctKeysAndReportsItsActualDomain() {
    var lut = new ShotLUT();
    lut.put(2, 20, 10, 0.5);
    lut.put(2, 22, 12, 0.6);
    lut.put(4, 30, 20, 0.9);
    assertEquals(2, lut.size());
    assertTrue(lut.containsDistance(2));
    assertFalse(lut.containsDistance(1.99));
    assertFalse(lut.containsDistance(4.01));
    assertThrows(IllegalArgumentException.class, () -> lut.put(3, 30, 20, Double.NaN));
    lut.clear();
    assertFalse(lut.containsDistance(3));
  }
}
