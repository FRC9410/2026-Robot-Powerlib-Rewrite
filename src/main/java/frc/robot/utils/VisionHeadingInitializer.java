package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.powerlib.health.HealthChecks;
import frc.powerlib.vision.LimelightVision.HeadingMeasurement;
import java.util.Optional;

/** One heading seed per disabled period, after stable independent camera frames. */
public final class VisionHeadingInitializer {
  private static final double MAX_FRAME_AGE_SECONDS = 0.2;
  private static final double MIN_STABLE_SECONDS = 0.25;
  private static final int MIN_FRAMES = 5;
  private static final double MAX_SPREAD_DEGREES = 2;
  private boolean initialized;
  private String camera;
  private Rotation2d anchor;
  private double firstTimestamp;
  private double lastTimestamp = Double.NEGATIVE_INFINITY;
  private double cosineSum;
  private double sineSum;
  private double minDelta;
  private double maxDelta;
  private int count;
  private String status = "Waiting for stable MT1 heading";

  public String status() { return status; }

  public void reset() {
    initialized = false;
    clearSamples();
    status = "Waiting for stable MT1 heading";
  }

  private void clearSamples() {
    camera = null;
    anchor = null;
    lastTimestamp = Double.NEGATIVE_INFINITY;
    count = 0;
    cosineSum = sineSum = 0;
    minDelta = maxDelta = 0;
  }

  public Optional<Rotation2d> update(HeadingMeasurement measurement, double now,
      double speedMetersPerSecond, double yawRateDegreesPerSecond) {
    return update(measurement, now, speedMetersPerSecond, yawRateDegreesPerSecond, 1);
  }

  /** Require multi-tag evidence for enabled recovery; single-tag frames break its stable window. */
  public Optional<Rotation2d> update(HeadingMeasurement measurement, double now,
      double speedMetersPerSecond, double yawRateDegreesPerSecond, int minimumTags) {
    if (measurement != null && measurement.tagCount() < minimumTags) {
      clearSamples();
      status = "Waiting for two-tag heading";
      return Optional.empty();
    }
    if (initialized) { status = "MT1 heading initialized"; return Optional.empty(); }
    if (!Double.isFinite(speedMetersPerSecond) || !Double.isFinite(yawRateDegreesPerSecond)
        || Math.abs(speedMetersPerSecond) > 0.05 || Math.abs(yawRateDegreesPerSecond) > 5) {
      clearSamples();
      status = "Waiting for stationary robot";
      return Optional.empty();
    }
    if (measurement == null) {
      if (!HealthChecks.isFresh(now - lastTimestamp, MAX_FRAME_AGE_SECONDS)) clearSamples();
      status = "Waiting for fresh independent heading";
      return Optional.empty();
    }
    if (!HealthChecks.finitePose(measurement.pose())
        || !HealthChecks.isFresh(now - measurement.timestampSeconds(), MAX_FRAME_AGE_SECONDS)) {
      clearSamples();
      status = "Rejected stale or invalid heading";
      return Optional.empty();
    }
    if (measurement.timestampSeconds() <= lastTimestamp) return Optional.empty();
    var rotation = measurement.pose().getRotation();
    double delta = anchor == null ? 0 : rotation.minus(anchor).getDegrees();
    if (anchor == null || !measurement.cameraName().equals(camera)
        || measurement.timestampSeconds() - lastTimestamp > MAX_FRAME_AGE_SECONDS
        || Math.max(maxDelta, delta) - Math.min(minDelta, delta) > MAX_SPREAD_DEGREES) {
      clearSamples();
      camera = measurement.cameraName();
      anchor = rotation;
      firstTimestamp = measurement.timestampSeconds();
      delta = 0;
    }
    minDelta = Math.min(minDelta, delta);
    maxDelta = Math.max(maxDelta, delta);
    lastTimestamp = measurement.timestampSeconds();
    cosineSum += rotation.getCos();
    sineSum += rotation.getSin();
    count++;
    status = "Collecting consistent MT1 heading";
    if (count < MIN_FRAMES || lastTimestamp - firstTimestamp < MIN_STABLE_SECONDS) return Optional.empty();
    initialized = true;
    status = "MT1 heading initialized";
    return Optional.of(new Rotation2d(cosineSum, sineSum));
  }
}
