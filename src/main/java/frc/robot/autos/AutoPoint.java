package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.powerlib.health.HealthChecks;

/** A destination and optional per-leg overrides. All units are SI. */
public record AutoPoint(Pose2d pose, Params params) {
  public AutoPoint {
    if (!HealthChecks.finitePose(pose)) throw new IllegalArgumentException("Auto point pose must be finite");
    if (params == null) params = Params.DEFAULT;
  }

  public AutoPoint(Pose2d pose) { this(pose, Params.DEFAULT); }

  /** Null fields retain the original leg defaults; fluent methods return independent copies. */
  public record Params(Double endDistanceToleranceMeters, Double endSpeedToleranceMetersPerSecond,
      Double maxAccelerationMetersPerSecondSquared, Double maxSpeedMetersPerSecond) {
    public static final Params DEFAULT = new Params();

    public Params {
      requirePositive(endDistanceToleranceMeters, "End distance tolerance");
      requirePositive(endSpeedToleranceMetersPerSecond, "End speed tolerance");
      requirePositive(maxAccelerationMetersPerSecondSquared, "Maximum acceleration");
      requirePositive(maxSpeedMetersPerSecond, "Maximum speed");
    }

    public Params() { this(null, null, null, null); }

    public Params withEndDistanceToleranceMeters(double value) {
      return new Params(value, endSpeedToleranceMetersPerSecond, maxAccelerationMetersPerSecondSquared,
          maxSpeedMetersPerSecond);
    }

    public Params withEndSpeedToleranceMetersPerSecond(double value) {
      return new Params(endDistanceToleranceMeters, value, maxAccelerationMetersPerSecondSquared,
          maxSpeedMetersPerSecond);
    }

    public Params withMaxAccelerationMetersPerSecondSquared(double value) {
      return new Params(endDistanceToleranceMeters, endSpeedToleranceMetersPerSecond, value,
          maxSpeedMetersPerSecond);
    }

    public Params withMaxSpeedMetersPerSecond(double value) {
      return new Params(endDistanceToleranceMeters, endSpeedToleranceMetersPerSecond,
          maxAccelerationMetersPerSecondSquared, value);
    }

    private static void requirePositive(Double value, String name) {
      if (value != null && (!Double.isFinite(value) || value <= 0))
        throw new IllegalArgumentException(name + " must be positive and finite");
    }
  }
}
