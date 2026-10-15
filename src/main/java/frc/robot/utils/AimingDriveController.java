package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Translation limiting, aim-target slewing and parallel-wheel parking for aiming. */
public final class AimingDriveController {
  /** Slewed aim heading; when limited, the robot should turn at {@code rateRadiansPerSecond}. */
  public record AimTarget(double headingDegrees, boolean limited, double rateRadiansPerSecond) {}

  private boolean parked;
  private double slewedHeadingDegrees = Double.NaN;

  public static Translation2d limitTranslation(double vx, double vy, double maximumSpeed) {
    if (!Double.isFinite(vx) || !Double.isFinite(vy)
        || !Double.isFinite(maximumSpeed) || maximumSpeed < 0) {
      throw new IllegalArgumentException("Aiming speed limit must be nonnegative and finite");
    }
    double speed = Math.hypot(vx, vy);
    double scale = speed > maximumSpeed ? maximumSpeed / speed : 1;
    return new Translation2d(vx * scale, vy * scale);
  }

  public boolean updateParking(double requestedVx, double requestedVy,
      ChassisSpeeds measured, double headingErrorDegrees) {
    boolean stopped = measured != null && Double.isFinite(requestedVx) && Double.isFinite(requestedVy)
        && Double.isFinite(headingErrorDegrees) && Double.isFinite(measured.vxMetersPerSecond)
        && Double.isFinite(measured.vyMetersPerSecond) && Double.isFinite(measured.omegaRadiansPerSecond)
        && Math.hypot(requestedVx, requestedVy) <= 0.02
        && Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) <= 0.05
        && Math.abs(measured.omegaRadiansPerSecond) <= Math.toRadians(5);
    // Both thresholds stay inside the 5-degree feed tolerance; hysteresis avoids chatter.
    parked = stopped && Math.abs(headingErrorDegrees) <= (parked ? 2 : 1);
    return parked;
  }

  /**
   * Rate-limits the aim target so lead changes and moving/static fallbacks turn the robot
   * smoothly instead of snapping. The first target after a reset is taken directly.
   */
  public AimTarget slewHeading(double targetDegrees, double maxRateDegreesPerSecond, double dtSeconds) {
    if (!Double.isFinite(slewedHeadingDegrees) || !Double.isFinite(maxRateDegreesPerSecond)
        || maxRateDegreesPerSecond <= 0 || !Double.isFinite(targetDegrees)) {
      slewedHeadingDegrees = targetDegrees;
      return new AimTarget(targetDegrees, false, 0);
    }
    double maxStep = maxRateDegreesPerSecond * dtSeconds;
    double remaining = MathUtil.inputModulus(targetDegrees - slewedHeadingDegrees, -180, 180);
    double step = MathUtil.clamp(remaining, -maxStep, maxStep);
    slewedHeadingDegrees = MathUtil.inputModulus(slewedHeadingDegrees + step, -180, 180);
    return new AimTarget(slewedHeadingDegrees, Math.abs(remaining) > maxStep,
        Math.toRadians(step / dtSeconds));
  }

  public void reset() {
    parked = false;
    slewedHeadingDegrees = Double.NaN;
  }
}
