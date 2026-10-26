package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.powerlib.PowerRobotContainer;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Exercises the actual Vision.periodic path with HAL, CTRE's drivetrain simulator and NT frames. */
class DisabledVisionSimulationTest {
  private static Swerve drivetrain;
  private static final DoubleArrayPublisher[][] poses = new DoubleArrayPublisher[3][2];
  private static final DoublePublisher[] targetValid = new DoublePublisher[3];

  @BeforeAll static void initialize() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    assertTrue(DriverStation.isDisabled());
    for (int camera = 0; camera < VisionConstants.LIMELIGHT_NAMES.length; camera++) {
      var table = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAMES[camera]);
      targetValid[camera] = table.getDoubleTopic("tv").publish();
      poses[camera][0] = table.getDoubleArrayTopic("botpose_wpiblue").publish(PubSubOption.keepDuplicates(true));
      poses[camera][1] = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish(PubSubOption.keepDuplicates(true));
    }
    drivetrain = TunerConstants.createDrivetrain();
    Timer.delay(.1);
  }

  @BeforeEach void resetMode() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
  }

  @AfterAll static void close() throws Exception {
    if (drivetrain != null) {
      var field = Swerve.class.getDeclaredField("m_simNotifier");
      field.setAccessible(true);
      ((Notifier) field.get(drivetrain)).close();
      drivetrain.close();
    }
    SignalLogger.stop();
    for (var publishers : poses) for (var publisher : publishers) publisher.close();
    for (var publisher : targetValid) publisher.close();
  }

  private static double[] frame(int tags, double x, double y, double heading) {
    var values = new double[11 + 7 * tags];
    values[0] = x; values[1] = y; values[5] = heading;
    values[6] = 50; values[7] = tags; values[8] = tags > 1 ? 1 : 0; values[9] = 2;
    for (int i = 0; i < tags; i++) {
      int start = 11 + i * 7;
      values[start] = i + 1;
      values[start + 4] = 2; values[start + 5] = 2; values[start + 6] = .05;
    }
    return values;
  }

  private static void tick(Vision vision, int seeingCamera, int tags, double x, double y, double heading) {
    for (int camera = 0; camera < VisionConstants.LIMELIGHT_NAMES.length; camera++) {
      targetValid[camera].set(1);
      var values = frame(camera == seeingCamera ? tags : 0,
          camera == seeingCamera ? x : 12, camera == seeingCamera ? y : 7,
          camera == seeingCamera ? heading : -135);
      for (var publisher : poses[camera]) publisher.set(values);
    }
    vision.periodic();
    Timer.delay(.02);
  }

  private static void startMoving() {
    drivetrain.applyRequest(new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds(.3, 0, 0)));
    Timer.delay(.2);
    var speeds = drivetrain.getState().Speeds;
    assertTrue(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > .05,
        "Native drivetrain simulation must report actual motion");
  }

  private static void stopMoving() {
    drivetrain.applyRequest(new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric());
    Timer.delay(.15);
  }
  private static void runFrames(Vision vision, int camera, int tags, double x, double y, double heading) {
    for (int loop = 0; loop < 50; loop++) tick(vision, camera, tags, x, y, heading);
  }

  @Test void movingHeadingConvergesThroughFusionInAutoAndTeleopWithoutSnapping() {
    for (boolean autonomous : new boolean[] {true, false}) {
      DriverStationSim.setAutonomous(autonomous);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(30)));
      Timer.delay(.06);
      startMoving();
      try (var vision = new Vision(drivetrain)) {
        long resetVersion = drivetrain.getPoseResetVersion();
        boolean corrected = false;
        for (int loop = 0; loop < 65; loop++) {
          var state = drivetrain.getState();
          assertTrue(Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond) > .05);
          double before = Math.abs(state.Pose.getRotation().getDegrees());
          tick(vision, 0, 2, state.Pose.getX(), state.Pose.getY(), 0);
          double after = Math.abs(drivetrain.getState().Pose.getRotation().getDegrees());
          assertNotEquals("Stationary two-tag heading recovered", PowerRobotContainer.getData("Vision/HeadingStatus", ""));
          if (before > 1 && after < before - .01) {
            corrected = true;
            assertTrue(after > .01, "Moving correction must converge gradually");
          }
        }
        assertTrue(corrected);
        assertEquals(resetVersion, drivetrain.getPoseResetVersion(), "Motion must prevent hard resets");
        assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), 1);
      } finally { stopMoving(); }
    }
  }

  @Test void manualSeedStillUsesFreshFramesWhenAutomaticCorrectionsAreOff() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    try (var vision = new Vision(drivetrain)) {
      vision.setShouldUpdatePose(false);
      for (int loop = 0; loop < 30; loop++) tick(vision, 1, 2, 4, 3, 0);
      assertEquals(2, drivetrain.getState().Pose.getX(), .01);
      assertTrue(vision.hasFreshMeasurement());
      assertFalse(vision.hasFreshAcceptedMeasurement(), "A plausible seed candidate is not an accepted update");
      assertTrue(vision.seedFromFreshVision());
      assertTrue(vision.hasFreshAcceptedMeasurement(), "An explicit operator pose seed was applied");
      assertEquals(4, drivetrain.getState().Pose.getX(), .01);
      assertEquals(3, drivetrain.getState().Pose.getY(), .01);
    }
  }

  @Test void manualSeedCanUseAPlausibleFrameRejectedForAnInconsistentLargeCorrection() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    try (var vision = new Vision(drivetrain)) {
      for (int loop = 0; loop < 30; loop++) {
        // Both are on the field and exceed the automatic 10 m correction limit,
        // but they disagree too much to complete the automatic recovery window.
        tick(vision, 0, 1, loop % 2 == 0 ? 14.7 : 15.3, 3, 0);
      }
      assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
      // Native wheel odometry continues running; reject the 13 m jump while allowing reset settling.
      assertEquals(2, drivetrain.getState().Pose.getX(), .1);
      assertTrue(vision.hasFreshMeasurement());
      assertFalse(vision.hasFreshAcceptedMeasurement(), "A plausible seed candidate is not an accepted update");
      assertTrue(vision.seedFromFreshVision());
      assertTrue(vision.hasFreshAcceptedMeasurement(), "An explicit operator pose seed was applied");
      assertEquals(15.3, drivetrain.getState().Pose.getX(), .01);
      assertEquals(3, drivetrain.getState().Pose.getY(), .01);
    }
  }

  @Test void initialDisabledSeedCorrectsLargeErrors() {
    assertEquals(180, VisionConstants.HEADING_CONFIG.maxHeadingErrorDegrees(), 0);
    for (int camera = 0; camera < VisionConstants.LIMELIGHT_NAMES.length; camera++) {
      for (int tags : new int[] { 1, 2 }) {
        for (double initialError : new double[] { 16, 45, 90, 180 }) {
          drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(initialError)));
          Timer.delay(.06);
          try (var vision = new Vision(drivetrain)) {
            boolean initialized = false;
            double started = Timer.getFPGATimestamp();
            for (int loop = 1; loop <= 75; loop++) {
              tick(vision, camera, tags, 2, 3, 0);
              String status = PowerRobotContainer.getData("Vision/HeadingStatus", "");
              double error = Math.abs(drivetrain.getState().Pose.getRotation().getDegrees());
              if ("MT1 heading initialized".equals(status)) {
                assertEquals(0, error, .01, "Initial seed must reset heading directly");
                System.out.printf("INITIALIZATION PASS camera=%s tags=%d initialError=%.0f finalError=%.6f loops=%d elapsed=%.3fs configuredLimit=180%n",
                    VisionConstants.LIMELIGHT_NAMES[camera], tags, initialError, error, loop,
                    Timer.getFPGATimestamp() - started);
                initialized = true;
                break;
              }
              assertEquals(initialError, error, .01, "Heading should stay unchanged until stability passes");
            }
            assertTrue(initialized, "Did not initialize " + VisionConstants.LIMELIGHT_NAMES[camera]
                + " with " + tags + " tags and " + initialError + " degrees error");
          }
        }
      }
    }
  }

  @Test void rejectedTaggedPrimaryFallsBackToSecondaryForBoundsHeightAndLargeOutliers() {
    for (int rejection = 0; rejection < 3; rejection++) {
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
      try (var vision = new Vision(drivetrain)) {
        int accepted = 0;
        for (int loop = 0; loop < 65; loop++) {
          var bad = frame(2, rejection == 0 ? 30 : rejection == 2 ? 15 : 2, 3, 0);
          if (rejection == 1) bad[2] = 1;
          for (var publisher : poses[0]) publisher.set(bad);
          for (var publisher : poses[1]) publisher.set(frame(2, 2, 3, 0));
          for (var publisher : poses[2]) publisher.set(frame(0, 12, 7, -135));
          vision.periodic();
          if (PowerRobotContainer.getData("Vision/PositionAccepted", false)) {
            accepted++;
            assertEquals(VisionConstants.LIMELIGHT_NAMES[1], PowerRobotContainer.getData("Vision/PositionCamera"));
          }
          Timer.delay(.02);
        }
        assertTrue(accepted > 25, "Good secondary frames should keep being accepted");
        assertEquals(2, drivetrain.getState().Pose.getX(), .01);
        assertEquals(3, drivetrain.getState().Pose.getY(), .01);
        System.out.printf("FALLBACK PASS rejection=%d accepted=%d/65 camera=%s%n", rejection, accepted,
            PowerRobotContainer.getData("Vision/PositionCamera"));
      }
    }
  }
  @Test void bufferedFramesCannotUndoManualHeadingZero() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 90);
      assertEquals(90, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      vision.zeroHeading();
      for (int loop = 0; loop < 5; loop++) {
        tick(vision, 0, 2, 2, 3, 0);
        assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      }
      assertEquals(2, drivetrain.getState().Pose.getX(), .01);
      assertEquals(3, drivetrain.getState().Pose.getY(), .01);
    }
  }
  @Test void disabledPoseUpdatesForEverySoleTagCameraAndHeading() {
    for (int camera = 0; camera < VisionConstants.LIMELIGHT_NAMES.length; camera++) {
      for (int tags : new int[] { 1, 2 }) {
        for (double heading : new double[] { 0, 90, 180, -90 }) {
          drivetrain.resetPose(new Pose2d());
          try (var vision = new Vision(drivetrain)) {
            // Include convergence time after the stricter half-second single-tag seed window.
            for (int loop = 0; loop < 75; loop++) tick(vision, camera, tags, 2, 3, heading);
            var actual = drivetrain.getState().Pose;
            String context = VisionConstants.LIMELIGHT_NAMES[camera] + " tags=" + tags + " yaw=" + heading
                + " actual=" + actual + " diagnostics=" + PowerRobotContainer.getAllSubsystemData().get("Vision");
            System.out.println(context);
            assertTrue(actual.getTranslation().getDistance(new Pose2d(2, 3, new Rotation2d()).getTranslation()) < .15, context);
            assertEquals(0, actual.getRotation().minus(Rotation2d.fromDegrees(heading)).getDegrees(), 1, context);
          }
        }
      }
    }
  }

  @Test void onlyTagCameraCanChangeWhileDisabledAndContinueUpdatingPosition() {
    drivetrain.resetPose(new Pose2d());
    try (var vision = new Vision(drivetrain)) {
      for (int camera = 0; camera < VisionConstants.LIMELIGHT_NAMES.length; camera++) {
        runFrames(vision, camera, 1, 2 + camera, 3, 0);
        assertEquals(2 + camera, drivetrain.getState().Pose.getX(), .15,
            VisionConstants.LIMELIGHT_NAMES[camera] + " " + PowerRobotContainer.getAllSubsystemData().get("Vision"));
      }
    }
  }

  @Test void strictSingleTagHeadingCanRecoverAfterFirstDisabledSeed() {
    drivetrain.resetPose(new Pose2d());
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 1, 2, 3, 0);
      // Simulate an incorrect estimator heading after its first disabled seed.
      drivetrain.resetRotation(Rotation2d.fromDegrees(90));
      runFrames(vision, 1, 1, 2, 3, 0);
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), 1,
          "Strong single-tag evidence must repair heading after earlier initialization");
      assertTrue(PowerRobotContainer.getData("Vision/MT1HeadingFresh", false));
      System.out.println("PASS: strict single-tag recovery repairs an already-initialized disabled heading.");
    }
  }

  @Test void twoTagsRepairALargeErrorAfterTheFirstDisabledSeed() {
    drivetrain.resetPose(new Pose2d());
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      drivetrain.resetRotation(Rotation2d.fromDegrees(90));
      runFrames(vision, 2, 2, 2, 3, 0);
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), 1);
      assertEquals("Independent heading applied", PowerRobotContainer.getData("Vision/HeadingStatus"));
      System.out.println("PASS: after disabled initialization, two tags repair a 90-degree error with the 180-degree correction limit.");
    }
  }

  @Test void rejectedFramesDoNotKeepAcceptedVisionFresh() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      for (int loop = 0; loop < 35; loop++) {
        tick(vision, 0, 1, loop % 2 == 0 ? 14.7 : 15.3, 3, 0);
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
      }
      assertTrue(vision.hasFreshMeasurement(), "Manual seeds may still use plausible rejected frames");
      assertFalse(vision.hasFreshAcceptedMeasurement(), "Rejected frames must not refresh accepted vision");
      runFrames(vision, 1, 2, 2, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
    }
  }

  @Test void startupBlocksWrongHeadingPositionsAndBufferedPreSeedFrames() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      boolean initialized = false;
      for (int loop = 0; loop < 75; loop++) {
        // Synthetic MT2 translation calculated with a wrong initial heading.
        tick(vision, 0, 2, 8, 3, 0);
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        assertFalse(vision.hasFreshAcceptedMeasurement());
        assertEquals(2, drivetrain.getState().Pose.getX(), .01);
        if ("MT1 heading initialized".equals(PowerRobotContainer.getData("Vision/HeadingStatus", ""))) {
          initialized = true;
          break;
        }
      }
      assertTrue(initialized);
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      // Fresh publications whose image capture predates the seed must remain blocked.
      for (int loop = 0; loop < 5; loop++) {
        var buffered = frame(2, 8, 3, 0);
        buffered[6] = 180;
        for (var publisher : poses[0]) publisher.set(buffered);
        for (var publisher : poses[1]) publisher.set(frame(0, 0, 0, 0));
        for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
        vision.periodic();
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        assertFalse(vision.hasFreshMeasurement(), "Pre-seed MT2 frames must not become manual-seed candidates");
        assertEquals(2, drivetrain.getState().Pose.getX(), .01);
        Timer.delay(.02);
      }
      runFrames(vision, 0, 2, 4, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(4, drivetrain.getState().Pose.getX(), .15);
    }
  }

  @Test void enablingBeforeHeadingInitializationDoesNotUnlockPositionFusion() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      // No independent MT1 heading, even though MT2 reports tags and a plausible translation.
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      for (int loop = 0; loop < 35; loop++) {
        for (int camera = 0; camera < poses.length; camera++) {
          poses[camera][0].set(frame(0, 0, 0, 0));
          poses[camera][1].set(frame(camera == 0 ? 2 : 0, 8, 3, 90));
        }
        vision.periodic();
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        Timer.delay(.02);
      }
      assertEquals(2, drivetrain.getState().Pose.getX(), .01);
      assertTrue(vision.hasFreshMeasurement());
      assertFalse(vision.hasFreshAcceptedMeasurement());
      assertEquals("Waiting for absolute heading initialization",
          PowerRobotContainer.getData("Vision/PositionStatus", ""));
      // Explicit operator zero establishes heading and releases the startup guard.
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      vision.zeroHeading();
      runFrames(vision, 0, 1, 4, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(4, drivetrain.getState().Pose.getX(), .15);
    }
  }

  @Test void disabledInitializationFallsBackAfterPrimaryCaptureIsRejected() throws Exception {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      var field = Vision.class.getDeclaredField("poseFilter");
      field.setAccessible(true);
      var filter = (frc.powerlib.vision.VisionPoseFilter) field.get(vision);
      var apply = Vision.class.getDeclaredMethod("applyMeasurements", java.util.List.class, java.util.List.class);
      apply.setAccessible(true);
      for (int loop = 0; loop < 25; loop++) {
        double now = Timer.getFPGATimestamp();
        // First camera captured while tilted; second captured after settling.
        filter.observe(now - .19, 20, 0, true);
        for (int i = 0; i <= 18; i++) filter.observe(now - .18 + i * .01, 0, 0, true);
        var old = new frc.powerlib.vision.LimelightVision.HeadingMeasurement("primary",
            new Pose2d(0, 0, Rotation2d.fromDegrees(90)), now - .18, .1, 2);
        var recent = new frc.powerlib.vision.LimelightVision.HeadingMeasurement("secondary",
            new Pose2d(0, 0, Rotation2d.kZero), now - .01, .1, 2);
        apply.invoke(vision, java.util.List.of(), java.util.List.of(old, recent));
        Timer.delay(.02);
      }
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      assertTrue(vision.hasFreshHeading());
    }
  }

  @Test void normalHeadingFusionTriesNextCameraAfterHistoryRejection() throws Exception {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      var filterField = Vision.class.getDeclaredField("poseFilter");
      filterField.setAccessible(true);
      var filter = (frc.powerlib.vision.VisionPoseFilter) filterField.get(vision);
      double now = Timer.getFPGATimestamp();
      for (int i = 0; i <= 30; i++) filter.observe(now - .3 + i * .01, 0, 0, true);
      var applied = new java.util.ArrayList<Pose2d>();
      var fusionField = Vision.class.getDeclaredField("fusion");
      fusionField.setAccessible(true);
      fusionField.set(vision, new frc.powerlib.vision.VisionMeasurementFusion(.5,
          timestamp -> timestamp < now - .03 ? java.util.Optional.empty()
              : java.util.Optional.of(new Pose2d(2, 3, Rotation2d.kZero)),
          (pose, timestamp, deviations) -> applied.add(pose)));
      var primary = new frc.powerlib.vision.LimelightVision.HeadingMeasurement("primary",
          new Pose2d(0, 0, Rotation2d.fromDegrees(90)), now - .05, .1, 2);
      var secondary = new frc.powerlib.vision.LimelightVision.HeadingMeasurement("secondary",
          new Pose2d(0, 0, Rotation2d.fromDegrees(5)), now - .01, .1, 2);
      var third = new frc.powerlib.vision.LimelightVision.HeadingMeasurement("third",
          new Pose2d(0, 0, Rotation2d.fromDegrees(10)), now - .01, .1, 2);
      var apply = Vision.class.getDeclaredMethod("applyMeasurements", java.util.List.class, java.util.List.class);
      apply.setAccessible(true);
      apply.invoke(vision, java.util.List.of(), java.util.List.of(primary, secondary, third));
      assertEquals(1, applied.size(), "Stop after the first accepted camera");
      assertEquals(5, applied.get(0).getRotation().getDegrees(), 1e-9);
      assertEquals("Independent heading applied", PowerRobotContainer.getData("Vision/HeadingStatus", ""));
    }
  }

  @Test void movingHeadingRecoveryReleasesPositionGuardWithoutSnapping() {
    for (boolean autonomous : new boolean[] {true, false}) {
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(30)));
      Timer.delay(.06);
      DriverStationSim.setAutonomous(autonomous);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      startMoving();
      try (var vision = new Vision(drivetrain)) {
        long resetVersion = drivetrain.getPoseResetVersion();
        int agreeingFrames = 0;
        boolean ready = false;
        for (int loop = 0; loop < 125; loop++) {
          var state = drivetrain.getState();
          assertTrue(Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond) > .05);
          tick(vision, 0, 2, 8, 3, 0);
          var after = drivetrain.getState().Pose;
          assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
          assertTrue(after.getTranslation().getDistance(state.Pose.getTranslation()) < .05,
              "Only measured motion may update position before heading is trusted");
          if (Math.abs(after.getRotation().getDegrees()) <= 5) agreeingFrames++;
          if (vision.isHeadingEstablished()) {
            assertTrue(agreeingFrames >= 5);
            ready = true;
            break;
          }
        }
        assertTrue(ready);
        assertEquals(resetVersion, drivetrain.getPoseResetVersion(), "Moving recovery must not reset heading");
        for (int loop = 0; loop < 5; loop++) {
          var delayed = frame(2, 8, 3, 0);
          delayed[6] = 180;
          for (var publisher : poses[0]) publisher.set(delayed);
          for (int camera = 1; camera < poses.length; camera++) {
            for (var publisher : poses[camera]) publisher.set(frame(0, 0, 0, 0));
          }
          vision.periodic();
          assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
          Timer.delay(.02);
        }
        runFrames(vision, 0, 2, 4, 3, 0);
        assertTrue(vision.hasFreshAcceptedMeasurement());
        assertEquals(4, drivetrain.getState().Pose.getX(), .15);
      } finally { stopMoving(); }
    }
  }

  @Test void directDrivetrainPoseAndHeadingSeedsReleaseGuardWithoutTags() {
    for (boolean fullPose : new boolean[] {true, false}) {
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
      Timer.delay(.06);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      try (var vision = new Vision(drivetrain)) {
        for (int loop = 0; loop < 25; loop++) {
          for (int camera = 0; camera < poses.length; camera++) {
            poses[camera][0].set(frame(0, 0, 0, 0));
            poses[camera][1].set(frame(camera == 0 ? 2 : 0, 8, 3, 90));
          }
          vision.periodic();
          Timer.delay(.02);
        }
        assertFalse(vision.isHeadingEstablished());
        assertEquals(2, drivetrain.getState().Pose.getX(), .01);
        long version = drivetrain.getPoseResetVersion();
        if (fullPose) drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
        else drivetrain.resetRotation(Rotation2d.kZero);
        assertTrue(drivetrain.getPoseResetVersion() > version);
        // A normal autonomous reset must be recognized without calling a Vision-specific API.
        for (int loop = 0; loop < 5; loop++) {
          var delayed = frame(2, 8, 3, 90);
          delayed[6] = 180;
          poses[0][1].set(delayed);
          vision.periodic();
          assertTrue(vision.isHeadingEstablished());
          assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
          assertEquals(2, drivetrain.getState().Pose.getX(), .01);
          Timer.delay(.02);
        }
        for (int loop = 0; loop < 50; loop++) {
          poses[0][1].set(frame(2, 4, 3, 0));
          vision.periodic();
          Timer.delay(.02);
        }
        assertTrue(vision.hasFreshAcceptedMeasurement());
        assertEquals(4, drivetrain.getState().Pose.getX(), .15);
      }
    }
  }

  @Test void correctionSwitchStillBlocksFusionAfterHeadingRecovery() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    try (var vision = new Vision(drivetrain)) {
      vision.setShouldUpdatePose(false);
      runFrames(vision, 0, 2, 8, 3, 0);
      assertFalse(vision.isHeadingEstablished());
      assertEquals(90, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      assertEquals(2, drivetrain.getState().Pose.getX(), .01);
      // Explicit seeds establish heading, but must not override the corrections-off switch.
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
      runFrames(vision, 0, 2, 4, 3, 0);
      assertTrue(vision.isHeadingEstablished());
      assertFalse(vision.hasFreshAcceptedMeasurement());
      assertEquals(2, drivetrain.getState().Pose.getX(), .01);
      vision.setShouldUpdatePose(true);
      runFrames(vision, 0, 2, 4, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(4, drivetrain.getState().Pose.getX(), .15);
    }
  }

  @Test void stationaryEnabledHeadingHardResetsAfterStableTwoTagEvidence() {
    for (boolean autonomous : new boolean[] {true, false}) for (double initialError : new double[] {90, 180}) {
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(initialError)));
      Timer.delay(.06);
      DriverStationSim.setAutonomous(autonomous);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      try (var vision = new Vision(drivetrain)) {
        long resetVersion = drivetrain.getPoseResetVersion();
        boolean reset = false;
        for (int loop = 0; loop < 75; loop++) {
          tick(vision, 0, 2, 8, 3, 0);
          assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
          assertEquals(2, drivetrain.getState().Pose.getX(), .01);
          if (drivetrain.getPoseResetVersion() != resetVersion) {
            assertTrue(loop >= 13, "A single good frame cannot trigger a hard reset");
            assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
            assertEquals("Stationary two-tag heading recovered", PowerRobotContainer.getData("Vision/HeadingStatus"));
            reset = true;
            break;
          }
        }
        assertTrue(reset, "A stationary enabled robot must recover directly from " + initialError);
        assertTrue(vision.isHeadingEstablished());
        resetVersion = drivetrain.getPoseResetVersion();
        runFrames(vision, 0, 2, 2, 3, 0);
        assertEquals(resetVersion, drivetrain.getPoseResetVersion(), "Do not repeatedly reset an already-correct heading");
        assertTrue(vision.hasFreshAcceptedMeasurement());
        assertEquals(2, drivetrain.getState().Pose.getX(), .01);
        assertEquals(3, drivetrain.getState().Pose.getY(), .01);
      }
    }
  }

  @Test void stationaryEnabledHardResetRejectsWeakSingleTagsAndTestMode() {
    for (boolean testMode : new boolean[] {false, true}) {
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
      Timer.delay(.06);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setTest(testMode);
      DriverStationSim.notifyNewData();
      try (var vision = new Vision(drivetrain)) {
        long resetVersion = drivetrain.getPoseResetVersion();
        for (int loop = 0; loop < 50; loop++) {
          for (int camera = 0; camera < poses.length; camera++) {
            var values = frame(camera == 0 ? (testMode ? 2 : 1) : 0, 8, 3, 0);
            if (!testMode && camera == 0) values[17] = .1; // Too ambiguous for single-tag recovery.
            for (var publisher : poses[camera]) publisher.set(values);
          }
          vision.periodic();
          Timer.delay(.02);
        }
        assertEquals(resetVersion, drivetrain.getPoseResetVersion());
        assertEquals(90, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
        assertFalse(vision.isHeadingEstablished());
      }
    }
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
  }

  @Test void asynchronousCamerasInitializeWhileDisabledAndChooseBetterMeasurements() throws Exception {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      int accepted = 0;
      for (int loop = 0; loop < 100; loop++) {
        var primary = frame(2, 2, 3, 0);
        primary[9] = 1.5; // Better geometry but a slower frame cadence.
        if (loop % 3 == 0) for (var publisher : poses[0]) publisher.set(primary);
        for (var publisher : poses[1]) publisher.set(frame(2, 2, 3, 0));
        for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
        vision.periodic();
        if (PowerRobotContainer.getData("Vision/PositionAccepted", false)) accepted++;
        Timer.delay(.02);
      }
      assertTrue(vision.isHeadingEstablished(), "Different camera rates must not starve initialization");
      assertTrue(accepted > 25);
      // The primary now reports a poorer conflicting solve; the secondary should win by geometry.
      for (int loop = 0; loop < 50; loop++) {
        var poor = frame(2, 8, 3, 30);
        poor[9] = 3.5; poor[17] = .15;
        var best = frame(3, 2, 3, 0);
        best[9] = 1.5;
        for (var publisher : poses[0]) publisher.set(poor);
        for (var publisher : poses[1]) publisher.set(best);
        for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
        vision.periodic(); Timer.delay(.02);
      }
      assertEquals(VisionConstants.LIMELIGHT_NAMES[1], PowerRobotContainer.getData("Vision/PositionCamera"));
      var field = Vision.class.getDeclaredField("lastHeadingMeasurement");
      field.setAccessible(true);
      assertEquals(VisionConstants.LIMELIGHT_NAMES[1],
          ((frc.powerlib.vision.LimelightVision.HeadingMeasurement) field.get(vision)).cameraName());
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .1);
      assertEquals(2, drivetrain.getState().Pose.getX(), .05);
    }
  }

  @Test void stage06FullDisabledFlowSeedsHeadingBeforePositionAndStaysReadyForFiveSeconds() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      long originalVersion = drivetrain.getPoseResetVersion();
      boolean seeded = false;
      for (int loop = 0; loop < 75; loop++) {
        // One camera sees tags; peers claim tv=1 but have no raw fiducials.
        tick(vision, loop % 3, 2, 8, 3, 0);
        assertEquals(2, drivetrain.getState().Pose.getX(), .02,
            "Robot-dependent MT2 position must wait for independent heading");
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        if (drivetrain.getPoseResetVersion() != originalVersion) {
          seeded = true;
          assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
          assertTrue(vision.isHeadingEstablished());
          break;
        }
      }
      assertTrue(seeded, "Real Vision.periodic must survive rapid camera handoffs");
      for (int loop = 0; loop < 5; loop++) {
        for (int camera = 0; camera < poses.length; camera++) {
          var delayed = frame(camera == 0 ? 2 : 0, 8, 3, 90);
          delayed[6] = 180;
          for (var publisher : poses[camera]) publisher.set(delayed);
        }
        vision.periodic();
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        assertEquals(2, drivetrain.getState().Pose.getX(), .02);
        Timer.delay(.02);
      }
      runFrames(vision, 1, 2, 4, 3, 0);
      assertEquals(4, drivetrain.getState().Pose.getX(), .1);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      long readyVersion = drivetrain.getPoseResetVersion();
      double waitStarted = Timer.getFPGATimestamp();
      while (Timer.getFPGATimestamp() - waitStarted < 5) {
        tick(vision, 1, 2, 4, 3, 0);
        assertTrue(vision.isHeadingEstablished());
        assertTrue(vision.hasFreshAcceptedMeasurement());
        assertEquals(readyVersion, drivetrain.getPoseResetVersion(), "Waiting disabled must not repeatedly reseed");
      }
    }
  }

  @Test void stage07EarlyEnableRecoversStationaryThenRejectsPreResetPositionImages() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      for (int loop = 0; loop < 5; loop++) tick(vision, 0, 2, 8, 3, 0);
      assertFalse(vision.isHeadingEstablished());
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      long version = drivetrain.getPoseResetVersion();
      double enabledAt = Timer.getFPGATimestamp();
      for (int loop = 0; loop < 75 && drivetrain.getPoseResetVersion() == version; loop++) {
        tick(vision, 1, 2, 8, 3, 0);
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        assertEquals(2, drivetrain.getState().Pose.getX(), .02);
      }
      assertTrue(drivetrain.getPoseResetVersion() > version);
      assertTrue(Timer.getFPGATimestamp() - enabledAt >= .25, "Enabling does not bypass evidence collection");
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      assertTrue(vision.isHeadingEstablished());
      for (int loop = 0; loop < 5; loop++) {
        var delayed = frame(2, 8, 3, 90);
        delayed[6] = 180;
        poses[1][1].set(delayed);
        vision.periodic();
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        assertEquals(2, drivetrain.getState().Pose.getX(), .02);
        Timer.delay(.02);
      }
      runFrames(vision, 1, 2, 4, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(4, drivetrain.getState().Pose.getX(), .1);
    }
  }

  @Test void stage07MovingRobotRejectsLargeHeadingErrorThenRecoversAfterStopping() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    startMoving();
    try (var vision = new Vision(drivetrain)) {
      long version = drivetrain.getPoseResetVersion();
      for (int loop = 0; loop < 30; loop++) {
        var before = drivetrain.getState().Pose;
        tick(vision, 0, 2, 8, 3, 0);
        var speeds = drivetrain.getState().Speeds;
        assertTrue(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > .05);
        assertFalse(vision.isHeadingEstablished());
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        assertEquals(90, drivetrain.getState().Pose.getRotation().getDegrees(), .1,
            "A 90-degree error exceeds the 45-degree moving limit");
        assertTrue(drivetrain.getState().Pose.getTranslation().getDistance(before.getTranslation()) < .1);
        assertEquals(version, drivetrain.getPoseResetVersion(), "Motion must not trigger hard resets");
      }
      stopMoving();
      for (int loop = 0; loop < 75 && drivetrain.getPoseResetVersion() == version; loop++) {
        tick(vision, 0, 2, 8, 3, 0);
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
      }
      assertTrue(drivetrain.getPoseResetVersion() > version, "The same good tags must recover after motion stops");
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      runFrames(vision, 0, 2, 4, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(4, drivetrain.getState().Pose.getX(), .1);
    } finally { stopMoving(); }
  }

  @Test void stage08CameraDisconnectExpiresAcceptedFreshnessAndReconnectRecoversWithoutReset() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      assertTrue(vision.isHeadingEstablished());
      assertTrue(vision.hasFreshAcceptedMeasurement());
      long version = drivetrain.getPoseResetVersion();
      // Disconnected camera leaves its last NT values and tv flag intact; no new publications occur.
      for (int loop = 0; loop < 35; loop++) {
        vision.periodic();
        assertFalse(PowerRobotContainer.getData("Vision/PositionAccepted", false));
        Timer.delay(.02);
      }
      assertFalse(vision.hasFreshAcceptedMeasurement(), "Stale NT values cannot masquerade as recent fusion");
      assertTrue(vision.isHeadingEstablished(), "A brief camera loss must not erase established gyro heading");
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      runFrames(vision, 2, 2, 4, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(version, drivetrain.getPoseResetVersion());
      assertEquals(4, drivetrain.getState().Pose.getX(), .1);
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .1);
    }
  }

  @Test void stage08CorrectionsOffDiscardsPartialEvidenceBeforeStationaryRecoveryResumes() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    try (var vision = new Vision(drivetrain)) {
      long version = drivetrain.getPoseResetVersion();
      // Settle the tilt gate without heading evidence, then provide fewer than the required five frames.
      // This tests partial collection without relying on the workstation executing 17 loops fast enough.
      for (int loop = 0; loop < 15; loop++) tick(vision, 0, 0, 8, 3, 0);
      for (int loop = 0; loop < 4; loop++) tick(vision, 0, 2, 8, 3, 0);
      assertEquals(version, drivetrain.getPoseResetVersion());
      vision.setShouldUpdatePose(false);
      runFrames(vision, 0, 2, 8, 3, 0);
      assertEquals(version, drivetrain.getPoseResetVersion());
      assertFalse(vision.hasFreshAcceptedMeasurement());
      assertEquals(2, drivetrain.getState().Pose.getX(), .02);
      double resumeAt = Timer.getFPGATimestamp();
      vision.setShouldUpdatePose(true);
      for (int loop = 0; loop < 75 && drivetrain.getPoseResetVersion() == version; loop++) {
        tick(vision, 0, 2, 8, 3, 0);
        if (Timer.getFPGATimestamp() - resumeAt < .25) assertEquals(version, drivetrain.getPoseResetVersion());
      }
      assertTrue(drivetrain.getPoseResetVersion() > version);
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      runFrames(vision, 0, 2, 4, 3, 0);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(4, drivetrain.getState().Pose.getX(), .1);
    }
  }

  @Test void unstableBestHeadingCameraCannotStarveStableTwoTagBackup() throws Exception {
    for (boolean enabled : new boolean[] {false, true}) {
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
      Timer.delay(.06);
      DriverStationSim.setEnabled(enabled);
      DriverStationSim.notifyNewData();
      try (var vision = new Vision(drivetrain)) {
        long version = drivetrain.getPoseResetVersion();
        boolean recovered = false;
        for (int loop = 0; loop < 100; loop++) {
          var unstable = frame(2, 2, 3, loop % 2 == 0 ? 8 : -8);
          unstable[9] = 1.5;
          for (var publisher : poses[0]) publisher.set(unstable);
          for (var publisher : poses[1]) publisher.set(frame(2, 2, 3, 0));
          for (var publisher : poses[2]) publisher.set(frame(0, 12, 7, 0));
          vision.periodic();
          if (drivetrain.getPoseResetVersion() != version) {
            assertTrue(vision.isHeadingEstablished());
            assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
            var field = Vision.class.getDeclaredField("lastHeadingMeasurement");
            field.setAccessible(true);
            assertEquals(VisionConstants.LIMELIGHT_NAMES[1],
                ((frc.powerlib.vision.LimelightVision.HeadingMeasurement) field.get(vision)).cameraName());
            recovered = true;
            break;
          }
          Timer.delay(.02);
        }
        assertTrue(recovered, "A geometrically better but unstable camera must not block the stable backup");
      }
    }
  }

  @Test void acceptedFallbackCannotStarveBetterCamerasThirteenMeterRecovery() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      assertTrue(vision.isHeadingEstablished());
      int betterAccepted = 0;
      for (int loop = 0; loop < 75; loop++) {
        var better = frame(2, 15, 3, 0);
        better[9] = 1.5;
        var fallback = frame(2, 2, 3, 0);
        fallback[9] = 3;
        poses[0][1].set(better);
        poses[1][1].set(fallback);
        poses[2][1].set(frame(0, 0, 0, 0));
        vision.periodic();
        if (PowerRobotContainer.getData("Vision/PositionAccepted", false)
            && VisionConstants.LIMELIGHT_NAMES[0].equals(PowerRobotContainer.getData("Vision/PositionCamera"))) {
          betterAccepted++;
        }
        Timer.delay(.02);
      }
      assertTrue(betterAccepted > 30, "The best camera's consistent large correction must eventually win");
      assertEquals(15, drivetrain.getState().Pose.getX(), .1);
      assertEquals(3, drivetrain.getState().Pose.getY(), .1);
    }
  }

  @Test void strictSingleTagFallbackRecoversStationaryHeadingDisabledAndEnabled() {
    for (boolean enabled : new boolean[] {false, true}) {
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
      Timer.delay(.06);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      try (var vision = new Vision(drivetrain)) {
        runFrames(vision, 0, 2, 2, 3, 0);
        assertTrue(vision.isHeadingEstablished());
        drivetrain.resetRotation(Rotation2d.fromDegrees(90));
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        long version = drivetrain.getPoseResetVersion();
        double started = Timer.getFPGATimestamp();
        boolean recovered = false;
        for (int loop = 0; loop < 100; loop++) {
          // A close, low-ambiguity single tag; the other cameras have no tags.
          tick(vision, 1, 1, 2, 3, 0);
          if (drivetrain.getPoseResetVersion() != version) {
            assertTrue(Timer.getFPGATimestamp() - started >= .5,
                "Single-tag recovery requires the longer high-confidence window");
            assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
            recovered = true;
            break;
          }
        }
        assertTrue(recovered, "A high-confidence single tag must recover after an earlier initialization");
        assertEquals(2, drivetrain.getState().Pose.getX(), .02);
        assertEquals(3, drivetrain.getState().Pose.getY(), .02);
        version = drivetrain.getPoseResetVersion();
        runFrames(vision, 1, 1, 2, 3, 0);
        assertEquals(version, drivetrain.getPoseResetVersion(), "An already-correct heading must not reset repeatedly");
      }
    }
  }
  @Test void betterCameraCanRecoverDespiteLongerPipelineLatency() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      assertTrue(vision.isHeadingEstablished());
      int betterAccepted = 0;
      for (int loop = 0; loop < 75; loop++) {
        var better = frame(2, 15, 3, 0);
        better[9] = 1.5;
        better[6] = 90; // Better geometry, but 40 ms more processing delay than the fallback.
        var fallback = frame(2, 2, 3, 0);
        fallback[9] = 3;
        poses[0][1].set(better);
        poses[1][1].set(fallback);
        poses[2][1].set(frame(0, 0, 0, 0));
        vision.periodic();
        if (PowerRobotContainer.getData("Vision/PositionAccepted", false)
            && VisionConstants.LIMELIGHT_NAMES[0].equals(PowerRobotContainer.getData("Vision/PositionCamera"))) {
          betterAccepted++;
        }
        Timer.delay(.02);
      }
      assertTrue(betterAccepted > 30, "The best camera's consistent large correction must eventually win");
      assertEquals(15, drivetrain.getState().Pose.getX(), .1);
      assertEquals(3, drivetrain.getState().Pose.getY(), .1);
    }
  }


  @Test void conflictingCameraCadencesCannotRepeatedlyResetHeadingOrStarvePosition() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      long version = drivetrain.getPoseResetVersion();
      int recoveredPositions = 0;
      for (int loop = 0; loop < 110; loop++) {
        // A close camera runs at 16.7 Hz with 90 ms latency; a disagreeing farther
        // camera runs at 50 Hz with 50 ms latency. Neither camera is disconnected.
        if (loop % 3 == 0) {
          var preferred = frame(2, 15, 3, 0);
          preferred[9] = 1.5;
          preferred[6] = 90;
          for (var publisher : poses[0]) publisher.set(preferred);
        }
        var conflicting = frame(2, 15, 3, 12);
        conflicting[9] = 3;
        for (var publisher : poses[1]) publisher.set(conflicting);
        for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
        vision.periodic();
        if (VisionConstants.LIMELIGHT_NAMES[0].equals(PowerRobotContainer.getData("Vision/PositionCamera"))
            && PowerRobotContainer.getData("Vision/PositionAccepted", false)) recoveredPositions++;
        Timer.delay(.02);
      }
      assertEquals(version, drivetrain.getPoseResetVersion(),
          "An available trusted camera must prevent repeated competing heading resets");
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .1);
      assertTrue(recoveredPositions > 10, "Heading disagreement must not keep clearing large-position evidence");
      assertEquals(15, drivetrain.getState().Pose.getX(), .15);
    }
  }

  @Test void headingCameraCanFailOverAndReturnWithoutAlternatingResets() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      long version = drivetrain.getPoseResetVersion();
      // The selected camera stops publishing. A stable backup must recover automatically.
      runFrames(vision, 1, 2, 2, 3, 12);
      assertEquals(12, drivetrain.getState().Pose.getRotation().getDegrees(), .1);
      assertTrue(drivetrain.getPoseResetVersion() <= version + 1, "Failover may fuse gradually or reset once");
      long backupVersion = drivetrain.getPoseResetVersion();
      for (int loop = 0; loop < 110; loop++) {
        if (loop % 3 == 0) {
          var preferred = frame(2, 2, 3, 0);
          preferred[9] = 1.5;
          preferred[6] = 90;
          for (var publisher : poses[0]) publisher.set(preferred);
        }
        var backup = frame(2, 2, 3, 12);
        backup[9] = 3;
        for (var publisher : poses[1]) publisher.set(backup);
        for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
        vision.periodic();
        Timer.delay(.02);
      }
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .1,
          "The returning camera's sustained better geometry must allow a controlled handoff");
      assertTrue(drivetrain.getPoseResetVersion() <= backupVersion + 1,
          "A handoff must not turn into repeated resets between conflicting cameras");
    }
  }

  @Test void selectedCameraThatBecomesUnstableCannotBlockStableBackup() throws Exception {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      for (int loop = 0; loop < 80; loop++) {
        var unstable = frame(2, 2, 3, loop % 2 == 0 ? 8 : -8);
        unstable[9] = 1.5;
        for (var publisher : poses[0]) publisher.set(unstable);
        for (var publisher : poses[1]) publisher.set(frame(2, 2, 3, 0));
        for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
        vision.periodic();
        Timer.delay(.02);
      }
      var field = Vision.class.getDeclaredField("lastHeadingMeasurement");
      field.setAccessible(true);
      assertEquals(VisionConstants.LIMELIGHT_NAMES[1],
          ((frc.powerlib.vision.LimelightVision.HeadingMeasurement) field.get(vision)).cameraName(),
          "A selected camera that loses stationary consistency must release selection to a stable backup");
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .1);
    }
  }

  @Test void movingHeadingConfirmationCanHandoffToSlowerBetterCamera() throws Exception {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(20)));
    Timer.delay(.06);
    startMoving();
    try (var vision = new Vision(drivetrain)) {
      long version = drivetrain.getPoseResetVersion();
      // Give the faster camera a head start. There is no stationary seed in this scenario.
      for (int loop = 0; loop < 16; loop++) {
        var state = drivetrain.getState();
        tick(vision, 1, 2, state.Pose.getX(), state.Pose.getY(), 20);
      }
      for (int loop = 0; loop < 120; loop++) {
        var state = drivetrain.getState();
        if (loop % 3 == 0) {
          var better = frame(2, state.Pose.getX(), state.Pose.getY(), 0);
          better[9] = 1.5;
          better[6] = 90;
          for (var publisher : poses[0]) publisher.set(better);
        }
        var fast = frame(2, state.Pose.getX(), state.Pose.getY(), 20);
        fast[9] = 3;
        for (var publisher : poses[1]) publisher.set(fast);
        for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
        vision.periodic();
        Timer.delay(.02);
      }
      assertTrue(vision.isHeadingEstablished());
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), 1,
          "A faster conflicting heading pipeline cannot starve the qualified slower camera");
      assertEquals(version, drivetrain.getPoseResetVersion(), "Moving handoffs must use gradual fusion");
      var field = Vision.class.getDeclaredField("lastHeadingMeasurement");
      field.setAccessible(true);
      assertEquals(VisionConstants.LIMELIGHT_NAMES[0],
          ((frc.powerlib.vision.LimelightVision.HeadingMeasurement) field.get(vision)).cameraName());
    } finally { stopMoving(); }
  }

  @Test void qualifiedSingleTagCorrectsSmallDriftWithoutHardResetsOrPositionInterruptions() {
    for (boolean enabled : new boolean[] {false, true}) {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
      Timer.delay(.06);
      try (var vision = new Vision(drivetrain)) {
        runFrames(vision, 0, 2, 2, 3, 0);
        assertTrue(vision.isHeadingEstablished());
        drivetrain.resetRotation(Rotation2d.fromDegrees(3));
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        long version = drivetrain.getPoseResetVersion();
        double started = Timer.getFPGATimestamp();
        int positions = 0;
        for (int loop = 0; loop < 100; loop++) {
          tick(vision, 0, 1, 2, 3, 0);
          if (Timer.getFPGATimestamp() - started < .5) {
            assertEquals(3, drivetrain.getState().Pose.getRotation().getDegrees(), .01,
                "Single-tag correction must wait for its stronger evidence window");
          }
          if (PowerRobotContainer.getData("Vision/PositionAccepted", false)) positions++;
        }
        assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .5);
        assertEquals(version, drivetrain.getPoseResetVersion(), "Small drift must use weighted fusion");
        assertTrue(positions > 50, "Small heading corrections must not restart position fusion");
      }
    }
  }

  @Test void initializationDisabledStillAllowsStationaryBetterCameraHandoffWithoutHardReset() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(20)));
    Timer.delay(.06);
    startMoving();
    try (var vision = new Vision(drivetrain,
        frc.powerlib.configs.VisionHeadingInitializationConfig.DEFAULT.withEnabled(false))) {
      long version = drivetrain.getPoseResetVersion();
      runFrames(vision, 1, 2, 2, 3, 20);
      stopMoving();
      for (int loop = 0; loop < 100; loop++) {
        var better = frame(2, 2, 3, 0);
        better[9] = 1.5;
        var selected = frame(2, 2, 3, 20);
        selected[9] = 3;
        for (var publisher : poses[0]) publisher.set(better);
        for (var publisher : poses[1]) publisher.set(selected);
        for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
        vision.periodic();
        Timer.delay(.02);
      }
      assertTrue(vision.isHeadingEstablished());
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .5);
      assertEquals(version, drivetrain.getPoseResetVersion(), "Initialization disabled must prohibit automatic hard resets");
      assertTrue(vision.hasFreshAcceptedMeasurement());
    } finally { stopMoving(); }
  }
  @Test void disabledAndEnabledRecoveryRequireTheirOwnFreshEvidence() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.fromDegrees(90)));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      // Settle the physical capture filter without supplying localization evidence.
      for (int loop = 0; loop < 25; loop++) tick(vision, 0, 0, 2, 3, 0);
      long version = drivetrain.getPoseResetVersion();
      for (int loop = 0; loop < 4; loop++) {
        tick(vision, 0, 2, 2, 3, 0);
        Timer.delay(.06);
      }
      assertFalse(vision.isHeadingEstablished(), "Four frames cannot complete the five-frame window");
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      for (int loop = 0; loop < 4; loop++) {
        tick(vision, 0, 2, 2, 3, 0);
        assertFalse(vision.isHeadingEstablished(), "Enabled recovery must not inherit disabled samples");
        assertEquals(version, drivetrain.getPoseResetVersion());
        assertTrue(Math.abs(drivetrain.getState().Pose.getRotation().getDegrees()) < 90.01);
        Timer.delay(.06);
      }
      runFrames(vision, 0, 2, 2, 3, 0);
      assertTrue(vision.isHeadingEstablished());
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .1);
      assertEquals(version + 1, drivetrain.getPoseResetVersion());
      assertTrue(vision.hasFreshAcceptedMeasurement(), "Position must resume after the fresh enabled recovery");
    }
  }

  @Test void testModeKeepsPositionFusionButBlocksHeadingChanges() {
    drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
    Timer.delay(.06);
    try (var vision = new Vision(drivetrain)) {
      runFrames(vision, 0, 2, 2, 3, 0);
      assertTrue(vision.isHeadingEstablished());
      long version = drivetrain.getPoseResetVersion();
      DriverStationSim.setEnabled(true);
      DriverStationSim.setTest(true);
      DriverStationSim.notifyNewData();

      runFrames(vision, 0, 2, 4, 3, 30);
      assertEquals(4, drivetrain.getState().Pose.getX(), .1);
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(version, drivetrain.getPoseResetVersion());

      vision.setShouldUpdatePose(false);
      runFrames(vision, 0, 2, 8, 3, 60);
      assertEquals(4, drivetrain.getState().Pose.getX(), .1);
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      assertFalse(vision.hasFreshAcceptedMeasurement());

      vision.setShouldUpdatePose(true);
      runFrames(vision, 0, 2, 6, 3, 60);
      assertEquals(6, drivetrain.getState().Pose.getX(), .1);
      assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01);
      assertTrue(vision.hasFreshAcceptedMeasurement());
      assertEquals(version, drivetrain.getPoseResetVersion());
    }
  }

  @Test void stationaryHeadingOutliersWaitForStableRequalification() {
    for (boolean enabled : new boolean[] {false, true}) {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
      Timer.delay(.06);
      try (var vision = new Vision(drivetrain)) {
        runFrames(vision, 0, 2, 2, 3, 0);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        runFrames(vision, 0, 2, 2, 3, 0);
        long version = drivetrain.getPoseResetVersion();
        for (int loop = 0; loop < 6; loop++) {
          tick(vision, 0, 2, 2, 3, loop % 2 == 0 ? 30 : -30);
          assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01,
              "An inconsistent stationary solve must not leak into weighted fusion");
          assertEquals(version, drivetrain.getPoseResetVersion());
          assertTrue(vision.hasFreshAcceptedMeasurement(), "Position updates must continue");
        }
        for (int loop = 0; loop < 4; loop++) {
          tick(vision, 0, 2, 2, 3, 30);
          assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01,
              "A flagged camera must complete a fresh stability window");
          assertEquals(version, drivetrain.getPoseResetVersion());
        }
        runFrames(vision, 0, 2, 2, 3, 30);
        assertEquals(30, drivetrain.getState().Pose.getRotation().getDegrees(), .1);
        assertEquals(version + 1, drivetrain.getPoseResetVersion(), "Stable evidence must recover once");
        assertTrue(vision.hasFreshAcceptedMeasurement());
        assertEquals(2, drivetrain.getState().Pose.getX(), .01);
        assertEquals(3, drivetrain.getState().Pose.getY(), .01);
      }
    }
  }

  @Test void inconsistentStationaryCameraCannotKickHeadingBeforeBackupTakesOver() {
    for (boolean enabled : new boolean[] {false, true}) {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      drivetrain.resetPose(new Pose2d(2, 3, Rotation2d.kZero));
      Timer.delay(.06);
      try (var vision = new Vision(drivetrain)) {
        runFrames(vision, 0, 2, 2, 3, 0);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        runFrames(vision, 0, 2, 2, 3, 0);
        long version = drivetrain.getPoseResetVersion();
        int acceptedPositions = 0;
        for (int loop = 0; loop < 65; loop++) {
          var unstable = frame(2, 2, 3, loop % 2 == 0 ? 30 : -30);
          unstable[9] = 1.5;
          var backup = frame(2, 2, 3, 0);
          backup[9] = 3;
          for (var publisher : poses[0]) publisher.set(unstable);
          for (var publisher : poses[1]) publisher.set(backup);
          for (var publisher : poses[2]) publisher.set(frame(0, 0, 0, 0));
          vision.periodic();
          Timer.delay(.02);
          assertEquals(0, drivetrain.getState().Pose.getRotation().getDegrees(), .01,
              "Backup recovery must not allow a transient bad-camera correction");
          assertEquals(version, drivetrain.getPoseResetVersion());
          if (PowerRobotContainer.getData("Vision/PositionAccepted", false)) acceptedPositions++;
        }
        assertTrue(vision.hasFreshHeading());
        assertTrue(acceptedPositions > 50);
      }
    }
  }
}
