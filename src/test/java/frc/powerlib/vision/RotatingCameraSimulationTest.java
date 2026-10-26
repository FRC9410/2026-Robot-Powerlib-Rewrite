package frc.powerlib.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.constants.VisionConstants;
import frc.powerlib.configs.VisionHeadingInitializationConfig;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Real NetworkTables readers and the SOTM gates, with deterministic 20 ms camera frames. */
class RotatingCameraSimulationTest {
  @BeforeAll static void initialize() { assertTrue(HAL.initialize(500, 0)); }

  private static final class Fixture implements AutoCloseable {
    final NetworkTableInstance nt = NetworkTableInstance.create();
    double now = 10;
    final LimelightVision reader = new LimelightVision(nt, VisionConstants.CONFIG,
        VisionConstants.HEADING_CONFIG, () -> now, VisionConstants.LIMELIGHT_NAMES);
    final DoubleArrayPublisher[][] poses = new DoubleArrayPublisher[VisionConstants.LIMELIGHT_NAMES.length][2];
    final DoublePublisher[] targetValid = new DoublePublisher[VisionConstants.LIMELIGHT_NAMES.length];

    Fixture() {
      for (int camera = 0; camera < VisionConstants.LIMELIGHT_NAMES.length; camera++) {
        var table = nt.getTable(VisionConstants.LIMELIGHT_NAMES[camera]);
        targetValid[camera] = table.getDoubleTopic("tv").publish();
        poses[camera][0] = table.getDoubleArrayTopic("botpose_wpiblue").publish(PubSubOption.keepDuplicates(true));
        poses[camera][1] = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish(PubSubOption.keepDuplicates(true));
      }
    }

    void publish(int seeingCamera, int tags, double heading, double distance, boolean paddedEmptyFrames) {
      for (int i = 0; i < VisionConstants.LIMELIGHT_NAMES.length; i++) {
        // Deliberately misleading target-valid flag and plausible nonzero pose on blind cameras.
        targetValid[i].set(1);
        var values = frame(i == seeingCamera ? tags : 0, i == seeingCamera ? heading : -135, distance);
        if (i != seeingCamera && paddedEmptyFrames) {
          values = frame(2, -135, distance);
          values[7] = 0; // Header says no tags despite leftover raw fiducial slots.
        }
        for (var publisher : poses[i]) publisher.set(values, Math.round(now * 1e6));
      }
    }

    @Override public void close() {
      reader.close();
      for (var publishers : poses) for (var publisher : publishers) publisher.close();
      for (var publisher : targetValid) publisher.close();
      nt.close();
    }
  }

  private static double[] frame(int tags, double heading, double distance) {
    var values = new double[11 + tags * 7];
    values[0] = 2; values[1] = 3; values[5] = heading;
    values[6] = 50; values[7] = tags; values[8] = tags >= 2 ? 1 : 0; values[9] = distance;
    for (int tag = 0; tag < tags; tag++) {
      int start = 11 + tag * 7;
      values[start] = tag + 1;
      values[start + 4] = distance; values[start + 5] = distance; values[start + 6] = .05;
    }
    return values;
  }

  @Test void everyCameraCanInitializeAtEveryHeadingWithOnlyZeroTagPeers() {
    for (boolean paddedEmptyFrames : new boolean[] { false, true }) {
      for (int camera = 0; camera < VisionConstants.LIMELIGHT_NAMES.length; camera++) {
        for (int tags : new int[] { 1, 2 }) {
          for (double heading : new double[] { 0, 90, 180, -90 }) {
            try (var fixture = new Fixture()) {
              var initializer = new VisionHeadingInitializer(VisionHeadingInitializationConfig.DEFAULT, VisionConstants.HEADING_CONFIG.maxMeasurementAgeSeconds());
              var filter = new VisionPoseFilter(VisionConstants.POSE_FILTER_CONFIG,
                  VisionConstants.CONFIG.maxMeasurementAgeSeconds());
              boolean initialized = false;
              int acceptedPositions = 0;
              for (int loop = 0; loop < 50; loop++) {
                fixture.now = 10 + loop * .02;
                fixture.publish(camera, tags, heading, 2, paddedEmptyFrames);
                filter.observe(fixture.now, 0, 0, true);
                var position = fixture.reader.update(0, 0, 0, 0, true).orElseThrow();
                var independentHeading = fixture.reader.getHeadingMeasurement().orElseThrow();
                String name = VisionConstants.LIMELIGHT_NAMES[camera];
                assertEquals(name, position.cameraName());
                assertEquals(name, independentHeading.cameraName());
                assertEquals(tags, position.tagCount());
                assertEquals(tags, independentHeading.tagCount());
                if (filter.acceptPosition(position, new Pose2d(), fixture.now)) acceptedPositions++;
                var result = initializer.update(filter.captureAllowed(independentHeading.timestampSeconds(), fixture.now)
                    ? independentHeading : null, fixture.now, 0, 0);
                if (result.isPresent()) {
                  initialized = true;
                  assertEquals(0, result.get().minus(independentHeading.pose().getRotation()).getDegrees(), 1e-9);
                }
              }
              assertTrue(initialized, "Heading failed: " + VisionConstants.LIMELIGHT_NAMES[camera] + ", " + heading);
              assertTrue(acceptedPositions > 20);
            }
          }
        }
      }
    }
    System.out.println("PASS: 48 camera/tag/heading/zero-tag-shape scenarios, 2,400 real NT reader updates.");
  }

  @Test void onlyTagCameraCanRotateThroughThePriorityListWithoutZeroTagCamerasWinning() {
    try (var fixture = new Fixture()) {
      for (int loop = 0; loop < 150; loop++) {
        int camera = (loop / 25) % VisionConstants.LIMELIGHT_NAMES.length;
        fixture.now = 10 + loop * .02;
        fixture.publish(camera, 1 + camera % 2, camera * 90, 2, false);
        assertEquals(VisionConstants.LIMELIGHT_NAMES[camera],
            fixture.reader.update(0, 0, 0, 0, true).orElseThrow().cameraName());
        assertEquals(VisionConstants.LIMELIGHT_NAMES[camera],
            fixture.reader.getHeadingMeasurement().orElseThrow().cameraName());
      }
    }
    System.out.println("PASS: only-tag camera rotates left/right/turret twice; zero-tag peers never win.");
  }

  @Test void allZeroTagFramesAreRejectedEvenWhenTargetValidIsTrue() {
    try (var fixture = new Fixture()) {
      for (int loop = 0; loop < 30; loop++) {
        fixture.now = 10 + loop * .02;
        fixture.publish(-1, 0, 90, 2, loop % 2 == 0);
        assertTrue(fixture.reader.update(0, 0, 0, 0, true).isEmpty());
        assertTrue(fixture.reader.getHeadingMeasurement().isEmpty());
      }
    }
  }

  @Test void rapidCameraSwitchingPreservesIndependentHeadingStabilityAndPositions() {
    try (var fixture = new Fixture()) {
      var initializer = new VisionHeadingInitializer(VisionHeadingInitializationConfig.DEFAULT, VisionConstants.HEADING_CONFIG.maxMeasurementAgeSeconds());
      int initializationCount = 0;
      for (int loop = 0; loop < 100; loop++) {
        fixture.now = 10 + loop * .02;
        fixture.publish(loop % 3, 2, 90, 2, false);
        assertTrue(fixture.reader.update(0, 0, 0, 0, true).isPresent());
        var heading = initializer.update(
            fixture.reader.getHeadingMeasurement().orElseThrow(), fixture.now, 0, 0);
        if (heading.isPresent()) {
          assertEquals(90, heading.orElseThrow().getDegrees(), 1e-9);
          initializationCount++;
        }
      }
      assertTrue(initializer.isInitialized());
      assertEquals(1, initializationCount);
    }
  }

  @Test void fiveMeterTagsProvidePositionButFailTheIndependentHeadingDistanceLimit() {
    try (var fixture = new Fixture()) {
      fixture.publish(2, 2, 90, 5, false);
      assertTrue(fixture.reader.update(0, 0, 0, 0, true).isPresent());
      assertTrue(fixture.reader.getHeadingMeasurement().isEmpty());
    }
  }

  @Test void laterCameraCanSupplyPositionWhenAnEarlierTaggedCameraFailsTheFilter() {
    try (var fixture = new Fixture()) {
      var filter = new VisionPoseFilter(VisionConstants.POSE_FILTER_CONFIG,
          VisionConstants.CONFIG.maxMeasurementAgeSeconds());
      for (int loop = 0; loop < 30; loop++) {
        fixture.now = 10 + loop * .02;
        filter.observe(fixture.now, 0, 0, true);
        for (int camera = 0; camera < 3; camera++) {
          var values = frame(camera == 2 ? 0 : 2, 0, 2);
          if (camera == 0) values[0] = 30; // Finite, tagged pose, but well outside the field.
          for (var publisher : fixture.poses[camera]) publisher.set(values, Math.round(fixture.now * 1e6));
        }
        var selected = fixture.reader.update(0, 0, 0, 0, true).orElseThrow();
        assertEquals(VisionConstants.LIMELIGHT_NAMES[0], selected.cameraName());
        boolean accepted = false;
        for (var candidate : fixture.reader.getPositionMeasurements()) {
          if (filter.acceptPosition(candidate, new Pose2d(), fixture.now)) {
            assertEquals(VisionConstants.LIMELIGHT_NAMES[1], candidate.cameraName());
            accepted = true;
            break;
          }
        }
        if (loop >= 11) assertTrue(accepted, "Valid second camera must not be hidden by the first");
      }
      var goodValues = frame(2, 0, 2);
      for (var publisher : fixture.poses[0]) publisher.set(frame(0, 0, 2), Math.round((fixture.now + .02) * 1e6));
      fixture.now += .02;
      for (var publisher : fixture.poses[1]) publisher.set(goodValues, Math.round(fixture.now * 1e6));
      filter.observe(fixture.now, 0, 0, true);
      var fallback = fixture.reader.update(0, 0, 0, 0, true).orElseThrow();
      assertEquals(VisionConstants.LIMELIGHT_NAMES[1], fallback.cameraName());
      assertTrue(filter.acceptPosition(fallback, new Pose2d(), fixture.now));
      System.out.println("PASS: a tagged out-of-field first camera cannot prevent fallback to the valid second camera.");
    }
  }
}
