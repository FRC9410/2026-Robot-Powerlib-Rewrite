package frc.powerlib.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.powerlib.configs.VisionHeadingInitializationConfig;
import frc.robot.constants.VisionConstants;
import java.util.Arrays;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Cumulative contracts: real NT input -> quality -> safety -> heading seed -> WPILib estimator.
 * Every test owns a fresh fixture; numbered stages do not depend on test execution order.
 * Full Vision.periodic contracts continue at stage 6 in DisabledVisionSimulationTest.
 */
@org.junit.jupiter.api.TestMethodOrder(org.junit.jupiter.api.MethodOrderer.MethodName.class)
class VisionFlowContractTest {
  private static final String[] NAMES = VisionConstants.LIMELIGHT_NAMES;

  @BeforeAll static void initialize() { assertTrue(HAL.initialize(500, 0)); }

  private static final class CameraRig implements AutoCloseable {
    double now = 20;
    final NetworkTableInstance nt = NetworkTableInstance.create();
    final DoubleArrayPublisher[][] publishers = new DoubleArrayPublisher[NAMES.length][2];
    final LimelightVision reader = new LimelightVision(nt, VisionConstants.CONFIG,
        VisionConstants.HEADING_CONFIG, () -> now, NAMES);
    final VisionPoseFilter filter = new VisionPoseFilter(VisionConstants.POSE_FILTER_CONFIG,
        VisionConstants.CONFIG.maxMeasurementAgeSeconds());
    final VisionHeadingInitializer initializer = new VisionHeadingInitializer(
        VisionHeadingInitializationConfig.DEFAULT, VisionConstants.HEADING_CONFIG.maxMeasurementAgeSeconds());

    CameraRig() {
      for (int i = 0; i < NAMES.length; i++) {
        var table = nt.getTable(NAMES[i]);
        // A disconnected/no-tag camera can leave tv=1 and old raw fiducial slots behind.
        table.getEntry("tv").setDouble(1);
        publishers[i][0] = table.getDoubleArrayTopic("botpose_orb_wpiblue")
            .publish(PubSubOption.keepDuplicates(true));
        publishers[i][1] = table.getDoubleArrayTopic("botpose_wpiblue")
            .publish(PubSubOption.keepDuplicates(true));
      }
    }

    void publish(int camera, double[] position, double[] heading) {
      publishers[camera][0].set(position, Math.round(now * 1e6));
      publishers[camera][1].set(heading, Math.round(now * 1e6));
    }

    void publish(int camera, int tags, double x, double heading, double distance) {
      publish(camera, frame(tags, x, 90, distance), frame(tags, x, heading, distance));
    }

    void read(boolean allowSingleTagHeading, double pitch, double yawRate) {
      filter.observe(now, pitch, 0, true);
      reader.update(90, pitch, 0, yawRate, allowSingleTagHeading);
    }

    void settleLevel() {
      for (int i = 20; i >= 0; i--) filter.observe(now - i * .02, 0, 0, true);
    }

    @Override public void close() {
      reader.close();
      for (var camera : publishers) for (var publisher : camera) publisher.close();
      nt.close();
    }
  }

  private static double[] frame(int tags, double x, double heading, double distance) {
    var values = new double[11 + tags * 7];
    values[0] = x; values[1] = 3; values[5] = heading;
    values[6] = 50; values[7] = tags; values[8] = tags >= 2 ? 1 : 0; values[9] = distance;
    for (int i = 0; i < tags; i++) {
      int index = 11 + i * 7;
      values[index] = i + 1;
      values[index + 4] = distance; values[index + 5] = distance; values[index + 6] = .05;
    }
    return values;
  }

  private static void assertDecoded(CameraRig rig, int camera, int tags, double distance) {
    var position = rig.reader.getPositionMeasurements().get(0);
    var heading = rig.reader.getHeadingMeasurement().orElseThrow();
    assertEquals(NAMES[camera], position.cameraName());
    assertEquals(NAMES[camera], heading.cameraName());
    assertEquals(tags, position.tagCount());
    assertEquals(rig.now - .05, position.timestampSeconds(), 1e-8);
    assertEquals(VisionConstants.CONFIG.basePositionStdDev() * distance * distance / tags,
        position.positionStdDev(), 1e-12, "Ranking must not alter fusion strength");
    assertEquals(90, position.pose().getRotation().getDegrees(), 1e-8);
    assertEquals(0, heading.pose().getRotation().getDegrees(), 1e-8,
        "MT1 must remain independent of the robot-dependent MT2 yaw");
  }

  @Test void stage01DecodesCaptureTimeAndKeepsTheTwoPipelinesIndependent() {
    try (var rig = new CameraRig()) {
      rig.publish(0, 2, 2, 0, 2);
      rig.read(true, 0, 0);
      assertDecoded(rig, 0, 2, 2);
      rig.now += .02;
      rig.read(true, 0, 0);
      assertTrue(rig.reader.getPositionMeasurements().isEmpty(), "A stopped camera is not a new frame");
      assertTrue(rig.reader.getHeadingMeasurements().isEmpty());
    }
  }

  @Test void stage01RejectsZeroTagLeftoversTruncatedUpdatesAndDelayedPackets() {
    try (var rig = new CameraRig()) {
      var zeroTags = frame(2, 12, -135, 2);
      zeroTags[7] = 0;
      rig.publish(0, zeroTags, zeroTags);
      var truncated = Arrays.copyOf(frame(2, 12, -135, 2), 10);
      rig.publish(1, truncated, truncated);
      var stale = frame(2, 12, -135, 2);
      stale[6] = 650; // A buffered packet after a radio interruption.
      rig.publish(2, stale, stale);
      rig.read(true, 0, 0);
      assertTrue(rig.reader.getPositionMeasurements().isEmpty());
      assertTrue(rig.reader.getHeadingMeasurements().isEmpty());
      rig.now += .02;
      rig.publish(1, 2, 2, 0, 2);
      rig.read(true, 0, 0);
      assertDecoded(rig, 1, 2, 2);
    }
  }

  @Test void stage02AddsQualitySelectionWithoutChangingTheDecodedMeasurements() {
    try (var rig = new CameraRig()) {
      rig.publish(0, 2, 8, 25, 3.5);
      rig.publish(1, 3, 2, 0, 1.5);
      rig.publish(2, 0, 12, -135, 2);
      rig.read(true, 0, 0);
      assertDecoded(rig, 1, 3, 1.5);
      assertEquals(2, rig.reader.getPositionMeasurements().size());
      assertEquals(2, rig.reader.getHeadingMeasurements().size());
    }
  }

  @Test void stage02RejectsAmbiguousHeadingAndPrefersTwoTagsOverOneCloseTag() {
    try (var rig = new CameraRig()) {
      rig.publish(0, 1, 2, 45, .8);
      rig.publish(1, 2, 2, 0, 2);
      var ambiguous = frame(3, 8, -30, 1);
      ambiguous[17] = .35;
      rig.publish(2, ambiguous, ambiguous);
      rig.read(true, 0, 0);
      assertEquals(NAMES[2], rig.reader.getPositionMeasurements().get(0).cameraName());
      assertEquals(NAMES[1], rig.reader.getHeadingMeasurement().orElseThrow().cameraName());
      assertEquals(2, rig.reader.getHeadingMeasurements().size(), "Ambiguity rejects heading, not MT2 position");
      rig.now += .02;
      for (int i = 0; i < 3; i++) rig.publish(i, i == 0 ? 1 : 0, 2, 0, 2);
      rig.read(false, 0, 0);
      assertEquals(1, rig.reader.getPositionMeasurements().size());
      assertTrue(rig.reader.getHeadingMeasurements().isEmpty(), "Single-tag candidates require explicit reader permission");
    }
  }

  @Test void stage03AddsPhysicalBoundsAndFallsBackFromTheBestButImpossibleSolve() {
    for (boolean offField : new boolean[] {true, false}) try (var rig = new CameraRig()) {
      rig.settleLevel();
      var impossible = frame(3, offField ? 30 : 2, 0, 1.5);
      if (!offField) impossible[2] = .9;
      rig.publish(0, impossible, impossible);
      rig.publish(1, 2, 2, 0, 2);
      rig.read(true, 0, 0);
      var candidates = rig.reader.getPositionMeasurements();
      assertEquals(NAMES[0], candidates.get(0).cameraName(), "Quality cannot detect impossible field poses");
      assertFalse(rig.filter.acceptPosition(candidates.get(0), new Pose2d(2, 3, Rotation2d.kZero), rig.now));
      assertTrue(rig.filter.acceptPosition(candidates.get(1), new Pose2d(2, 3, Rotation2d.kZero), rig.now));
    }
  }

  @Test void stage03RejectsBumpCapturesAfterLandingAndRecoversWithNewLevelImages() {
    try (var rig = new CameraRig()) {
      rig.settleLevel();
      rig.publish(0, 2, 2, 0, 2);
      rig.read(true, 0, 0);
      assertDecoded(rig, 0, 2, 2);
      assertTrue(rig.filter.plausible(rig.reader.getPositionMeasurements().get(0), rig.now));
      rig.now += .02;
      rig.publish(0, 2, 8, 0, 2);
      rig.read(true, 14, 0);
      assertFalse(rig.filter.captureAllowed(rig.now - .05, rig.now));
      for (int i = 0; i < 12; i++) {
        rig.now += .02;
        var buffered = frame(2, 8, 0, 2);
        buffered[6] = 260;
        rig.publish(0, buffered, buffered);
        rig.read(true, 0, 0);
        assertFalse(rig.filter.plausible(rig.reader.getPositionMeasurements().get(0), rig.now),
            "A level robot cannot make a buffered bump image safe");
      }
      rig.now += .02;
      rig.publish(1, 2, 2, 0, 2);
      rig.read(true, 0, 0);
      assertDecoded(rig, 1, 2, 2);
      assertTrue(rig.filter.plausible(rig.reader.getPositionMeasurements().get(0), rig.now));
    }
  }

  @Test void stage03GyroFeedbackLossRequiresFreshSettlingAndNewCaptures() {
    try (var rig = new CameraRig()) {
      rig.settleLevel();
      rig.publish(0, 2, 2, 0, 2);
      rig.read(true, 0, 0);
      assertDecoded(rig, 0, 2, 2);
      assertTrue(rig.filter.plausible(rig.reader.getPositionMeasurements().get(0), rig.now));
      rig.now += .02;
      rig.filter.observe(rig.now, 0, 0, false); // Gyro CAN feedback timed out.
      assertFalse(rig.filter.captureAllowed(rig.now - .05, rig.now));
      double restoredAt = rig.now + .02;
      for (int loop = 0; loop < 20; loop++) {
        rig.now += .02;
        rig.publish(0, 2, 2, 0, 2);
        rig.read(true, 0, 0);
        boolean accepted = rig.filter.plausible(rig.reader.getPositionMeasurements().get(0), rig.now);
        if (rig.now - restoredAt < .19) assertFalse(accepted);
        if (rig.now - restoredAt > .25) assertTrue(accepted, "Valid gyro feedback and new frames must recover");
      }
    }
  }

  @Test void stage04AddsStableHeadingEvidenceDespiteDifferentCameraRates() {
    try (var rig = new CameraRig()) {
      int seeds = 0;
      double began = rig.now;
      for (int loop = 0; loop < 65; loop++) {
        if (loop % 3 == 0) rig.publish(0, 2, 2, 0, 1.5); // 16.7 Hz camera, versus 50 Hz robot.
        rig.publish(1, 2, 2, 0, 2);
        rig.publish(2, 0, 12, -135, 2);
        rig.read(true, 0, 0);
        var best = rig.reader.getHeadingMeasurement().orElseThrow();
        assertEquals(NAMES[loop % 3 == 0 ? 0 : 1], best.cameraName());
        var seed = rig.initializer.update(rig.filter.captureAllowed(best.timestampSeconds(), rig.now)
            ? best : null, rig.now, 0, 0);
        if (seed.isPresent()) {
          assertTrue(rig.now - began >= .45, "Level settling and the stable window must both complete");
          assertEquals(0, seed.orElseThrow().getDegrees(), 1e-8);
          seeds++;
        }
        rig.now += .02;
      }
      assertEquals(1, seeds, "Camera handoffs cannot starve initialization or produce repeated seeds");
    }
  }

  @Test void stage04DoesNotCountFrozenFramesAndDiscardsEvidenceWhenMotionResumes() {
    try (var rig = new CameraRig()) {
      rig.settleLevel();
      double afterMotion = Double.NaN;
      for (int loop = 0; loop < 40; loop++) {
        // Camera stalls for 100 ms, then the robot is gently pushed while waiting for the match.
        if (loop < 4 || loop >= 9) rig.publish(0, 2, 2, 0, 2);
        rig.read(true, 0, 0);
        var candidate = rig.reader.getHeadingMeasurement().orElse(null);
        double speed = loop == 12 ? .12 : 0;
        if (loop == 12) afterMotion = rig.now;
        var seed = rig.initializer.update(candidate, rig.now, speed, 0);
        if (loop <= 12 || (Double.isFinite(afterMotion) && rig.now - afterMotion < .25)) {
          assertTrue(seed.isEmpty(), "Neither duplicate packets nor pre-motion frames establish heading");
        }
        if (seed.isPresent()) assertEquals(0, seed.orElseThrow().getDegrees(), 1e-8);
        rig.now += .02;
      }
      assertTrue(rig.initializer.isInitialized(), "Valid stationary images must recover after a push");
    }
  }

  @Test void stage04UnstableSolvesCannotSeedAndWrappedStableHeadingsCanRecover() {
    try (var rig = new CameraRig()) {
      rig.settleLevel();
      int seeds = 0;
      for (int loop = 0; loop < 65; loop++) {
        // Initially switching between two plausible solves; afterwards stable around the +/-180 boundary.
        double angle = loop < 30 ? (loop % 2 == 0 ? 4 : -4) : (loop % 2 == 0 ? 179.7 : -179.7);
        rig.publish(0, 2, 2, angle, 2);
        rig.read(true, 0, 0);
        var heading = rig.reader.getHeadingMeasurement().orElseThrow();
        assertTrue(rig.filter.captureAllowed(heading.timestampSeconds(), rig.now));
        var seed = rig.initializer.update(heading, rig.now, 0, 0);
        if (loop < 43) assertTrue(seed.isEmpty(), "Consistent evidence must restart after a changed solve");
        if (seed.isPresent()) {
          assertEquals(180, Math.abs(seed.orElseThrow().getDegrees()), .05,
              "A wraparound heading must not average to zero");
          seeds++;
        }
        rig.now += .02;
      }
      assertEquals(1, seeds);
    }
  }

  private static final class EstimatorRig {
    final SwerveModulePosition[] modules = new SwerveModulePosition[4];
    final SwerveDrivePoseEstimator estimator;
    final VisionMeasurementFusion fusion;
    int applied;

    EstimatorRig(CameraRig camera) { this(camera, 90); }

    EstimatorRig(CameraRig camera, double initialHeading) {
      Arrays.setAll(modules, i -> new SwerveModulePosition());
      var kinematics = new SwerveDriveKinematics(new Translation2d(.3, .3), new Translation2d(.3, -.3),
          new Translation2d(-.3, .3), new Translation2d(-.3, -.3));
      estimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, modules,
          new Pose2d(2, 3, Rotation2d.fromDegrees(initialHeading)), VecBuilder.fill(.1, .1, .1), VecBuilder.fill(1, 1, 1));
      for (int i = 25; i >= 0; i--) estimator.updateWithTime(camera.now - i * .02, Rotation2d.kZero, modules);
      fusion = new VisionMeasurementFusion(.5, estimator::sampleAt, (pose, time, deviations) -> {
        applied++;
        estimator.addVisionMeasurement(pose, time, deviations);
      });
    }
  }

  @Test void stage05AddsTimestampedFusionWithoutMixingMt1PositionOrMt2Yaw() {
    try (var rig = new CameraRig()) {
      rig.settleLevel();
      var drive = new EstimatorRig(rig);
      int seeds = 0;
      for (int loop = 0; loop < 100; loop++) {
        drive.estimator.updateWithTime(rig.now, Rotation2d.kZero, drive.modules);
        rig.publish(0, frame(2, 4, 90, 2), frame(2, 12, 0, 2));
        rig.read(true, 0, 0);
        var heading = rig.reader.getHeadingMeasurement().orElseThrow();
        assertDecoded(rig, 0, 2, 2);
        assertTrue(rig.filter.captureAllowed(heading.timestampSeconds(), rig.now));
        var seed = rig.initializer.update(heading, rig.now, 0, 0);
        if (seed.isPresent()) {
          drive.estimator.resetPosition(Rotation2d.kZero, drive.modules,
              new Pose2d(drive.estimator.getEstimatedPosition().getTranslation(), seed.orElseThrow()));
          drive.fusion.resetPose(rig.now);
          seeds++;
        }
        var position = rig.reader.getPositionMeasurements().get(0);
        if (rig.initializer.isInitialized() && drive.fusion.isNewPosition(position)) {
          var history = drive.estimator.sampleAt(position.timestampSeconds()).orElseThrow();
          assertTrue(rig.filter.acceptPosition(position, history, rig.now));
          assertTrue(drive.fusion.addPosition(position, rig.now, history));
        }
        if (rig.initializer.isInitialized() && drive.fusion.isNewHeading(heading)) {
          assertTrue(drive.fusion.addHeading(heading, rig.now, 180));
        }
        var pose = drive.estimator.getEstimatedPosition();
        assertTrue(pose.getX() <= 4 + 1e-8, "Independent MT1 X=12 must never pull the robot across the field");
        if (rig.initializer.isInitialized()) assertEquals(0, pose.getRotation().getDegrees(), 1e-5);
        rig.now += .02;
      }
      assertEquals(1, seeds);
      assertEquals(4, drive.estimator.getEstimatedPosition().getX(), .001);
      assertEquals(3, drive.estimator.getEstimatedPosition().getY(), 1e-8);
      assertTrue(drive.applied > 80);
    }
  }

  @Test void stage05RejectsBufferedPreResetImagesWithoutConsumingNewerSafeCorrections() {
    try (var rig = new CameraRig()) {
      rig.settleLevel();
      var drive = new EstimatorRig(rig);
      // An operator has explicitly established field heading. An older MT2 solve used the wrong yaw.
      drive.estimator.resetPosition(Rotation2d.kZero, drive.modules, new Pose2d(2, 3, Rotation2d.kZero));
      drive.fusion.resetPose(rig.now);
      rig.now += .02;
      drive.estimator.updateWithTime(rig.now, Rotation2d.kZero, drive.modules);
      var buffered = frame(2, 8, 90, 2);
      buffered[6] = 180;
      rig.publish(0, buffered, buffered);
      rig.read(true, 0, 0);
      var old = rig.reader.getPositionMeasurements().get(0);
      assertTrue(rig.filter.plausible(old, rig.now), "The frame is physically plausible but predates the reset");
      assertFalse(drive.fusion.addPosition(old, rig.now));
      assertFalse(drive.fusion.addHeading(rig.reader.getHeadingMeasurement().orElseThrow(), rig.now, 180));
      assertEquals(0, drive.applied);
      assertEquals(2, drive.estimator.getEstimatedPosition().getX(), 1e-8);
      for (int loop = 0; loop < 45; loop++) {
        rig.now += .02;
        drive.estimator.updateWithTime(rig.now, Rotation2d.kZero, drive.modules);
        rig.publish(1, 2, 4, 0, 2);
        rig.read(true, 0, 0);
        assertDecoded(rig, 1, 2, 2);
        var good = rig.reader.getPositionMeasurements().get(0);
        if (!drive.fusion.isNewPosition(good)) continue;
        var history = drive.estimator.sampleAt(good.timestampSeconds()).orElseThrow();
        assertTrue(rig.filter.acceptPosition(good, history, rig.now));
        assertTrue(drive.fusion.addPosition(good, rig.now, history));
      }
      assertEquals(4, drive.estimator.getEstimatedPosition().getX(), .001);
      assertEquals(0, drive.estimator.getEstimatedPosition().getRotation().getDegrees(), 1e-5);
    }
  }

  @Test void stage05DifferentPipelineLatenciesPreserveMotionAndConvergeBothAxes() {
    for (double[] latency : new double[][] {{120, 40}, {40, 120}}) try (var rig = new CameraRig()) {
      rig.settleLevel();
      var drive = new EstimatorRig(rig, 30);
      for (int loop = 0; loop < 80; loop++) {
        double travel = loop * .006; // .3 m/s straight travel, integrated by real WPILib odometry.
        for (var module : drive.modules) module.distanceMeters = travel;
        drive.estimator.updateWithTime(rig.now, Rotation2d.kZero, drive.modules);
        var positionFrame = frame(2, 4 + .3 * (rig.now - 20 - latency[0] / 1000), 90, 2);
        positionFrame[6] = latency[0];
        var headingFrame = frame(2, 12, 0, 2);
        headingFrame[6] = latency[1];
        rig.publish(1, positionFrame, headingFrame);
        rig.read(false, 0, 0);
        var position = rig.reader.getPositionMeasurements().get(0);
        var heading = rig.reader.getHeadingMeasurement().orElseThrow();
        assertEquals(NAMES[1], position.cameraName());
        assertEquals(rig.now - latency[0] / 1000, position.timestampSeconds(), 1e-8);
        assertEquals(rig.now - latency[1] / 1000, heading.timestampSeconds(), 1e-8);
        assertEquals(.2, position.positionStdDev(), 1e-10);
        assertTrue(rig.filter.captureAllowed(heading.timestampSeconds(), rig.now));
        var history = drive.estimator.sampleAt(position.timestampSeconds()).orElseThrow();
        assertTrue(rig.filter.acceptPosition(position, history, rig.now));
        assertTrue(drive.fusion.addPosition(position, rig.now, history));
        assertTrue(drive.fusion.addHeading(heading, rig.now,
            VisionConstants.HEADING_CONFIG.headingErrorLimitDegrees(.3, 0)));
        var corrected = drive.estimator.getEstimatedPosition();
        assertTrue(corrected.getX() <= 4 + travel + .02, "Delayed heading cannot erase/reapply translation incorrectly");
        assertTrue(Math.abs(corrected.getRotation().getDegrees()) <= 30 + 1e-8);
        rig.now += .02;
      }
      assertEquals(4 + 79 * .006, drive.estimator.getEstimatedPosition().getX(), .03);
      assertEquals(0, drive.estimator.getEstimatedPosition().getRotation().getDegrees(), .1);
    }
  }
}
