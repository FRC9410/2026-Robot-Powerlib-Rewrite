package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Translation limiting and parallel-wheel parking for teleop and autonomous aiming. */
public final class AimingDriveController {
  private boolean parked;

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

  public void reset() { parked = false; }
}
