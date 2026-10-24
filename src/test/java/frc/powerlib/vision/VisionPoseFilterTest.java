package frc.powerlib.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.powerlib.configs.VisionPoseFilterConfig;
import org.junit.jupiter.api.Test;

class VisionPoseFilterTest {
  @Test void largeCorrectionRecoversAfterBumpAndSettling() {
    var filter = new VisionPoseFilter(VisionPoseFilterConfig.forField(0, 16.541, 0, 8.069), .5);
    var historical = new Pose2d(2, 2, Rotation2d.kZero);
    filter.observe(10, 12, 0, true);
    for (int i = 1; i <= 20; i++) filter.observe(10 + i * .02, 0, 0, true);
    var delayed = new LimelightVision.Measurement("test", new Pose2d(15, 2, Rotation2d.kZero),
        10.1, 1, 4, 11.2);
    assertFalse(filter.acceptPosition(delayed, historical, 10.4));
    for (int i = 0; i < 7; i++) {
      double now = 10.42 + i * .02;
      filter.observe(now, 0, 0, true);
      var measurement = new LimelightVision.Measurement("test", new Pose2d(15, 2, Rotation2d.kZero),
          now - .02, 1, 4, 11.2);
      boolean accepted = filter.acceptPosition(measurement, historical, now);
      if (i < 5) assertFalse(accepted);
      if (i == 6) assertTrue(accepted, filter.status());
    }
  }
}
