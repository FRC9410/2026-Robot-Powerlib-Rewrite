package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachine;

/** Shared shot calculations and mechanism actions for the robot's state handlers. */
public final class ShootingUtils {
  private ShootingUtils() {}

  public record ShotSolution(boolean valid, double distanceMeters, double headingDegrees,
      double headingErrorDegrees, double shooterRps, double hoodRotations, double feederRps,
      boolean velocityReady, boolean hoodReady, boolean aligned, boolean feedbackHealthy) {
    public boolean canFeed() {
      return valid && velocityReady && hoodReady && aligned && feedbackHealthy;
    }
  }

  public static final ShotSolution UNAVAILABLE = new ShotSolution(false, Double.NaN,
      Double.NaN, Double.NaN, 0, 0, 0, false, false, false, false);

  public static boolean inScoringZone(Pose2d pose, Alliance alliance) {
    if (!HealthChecks.finitePose(pose) || alliance == null || pose.getY() < 0
        || pose.getY() > Constants.Shooting.FIELD_WIDTH) return false;
    return alliance == Alliance.Red
        ? pose.getX() > Constants.Shooting.RED_ZONE_START && pose.getX() < Constants.Shooting.RED_ZONE_END
        : pose.getX() > Constants.Shooting.BLUE_ZONE_START && pose.getX() < Constants.Shooting.BLUE_ZONE_END;
  }

  public static double distanceMeters(Pose2d pose, Alliance alliance) {
    if (!HealthChecks.finitePose(pose) || alliance == null) return Double.NaN;
    var hub = alliance == Alliance.Red ? Constants.Shooting.RED_HUB : Constants.Shooting.BLUE_HUB;
    return pose.getTranslation().getDistance(hub);
  }

  public static double headingDegrees(Pose2d pose, Alliance alliance) {
    if (!HealthChecks.finitePose(pose) || alliance == null) return Double.NaN;
    var hub = alliance == Alliance.Red ? Constants.Shooting.RED_HUB : Constants.Shooting.BLUE_HUB;
    return MathUtil.inputModulus(hub.minus(pose.getTranslation()).getAngle().getDegrees()
        - 180, -180, 180);
  }

  /** Pure calculation, with fresh feedback and current tunable values supplied each cycle. */
  public static ShotSolution calculate(Pose2d pose, Alliance alliance, double shooterRps,
      double hoodRotations, boolean feedbackHealthy) {
    if (!HealthChecks.finitePose(pose) || alliance == null) return UNAVAILABLE;
    double distance = distanceMeters(pose, alliance);
    double heading = headingDegrees(pose, alliance);
    double error = MathUtil.inputModulus(heading - pose.getRotation().getDegrees(), -180, 180);
    boolean valid = Double.isFinite(Constants.StateMachine.HOOD_SHOT_OFFSET_ROTATIONS)
        && Double.isFinite(Constants.StateMachine.FEEDER_SHOOTER_RATIO) && Constants.StateMachine.FEEDER_SHOOTER_RATIO < 0
        && inScoringZone(pose, alliance) && Double.isFinite(distance)
        && distance >= Constants.Shooting.MIN_SHOT_DISTANCE && distance <= Constants.Shooting.MAX_SHOT_DISTANCE
        && Double.isFinite(Constants.StateMachine.SHOOTER_EXTRA_RPS) && Constants.StateMachine.SHOOTER_EXTRA_RPS >= 0
        && Double.isFinite(Constants.StateMachine.HOOD_TOLERANCE_ROTATIONS) && Constants.StateMachine.HOOD_TOLERANCE_ROTATIONS > 0
        && Double.isFinite(Constants.StateMachine.SPINDEXER_SHOOT_RPS) && Constants.StateMachine.SPINDEXER_SHOOT_RPS > 0;
    if (!valid) return new ShotSolution(false, distance, heading, error, 0, 0, 0, false, false, false, feedbackHealthy);
    double demand = Constants.Turret.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance)
        + Constants.StateMachine.SHOOTER_EXTRA_RPS;
    double hood = MathUtil.clamp(Constants.Turret.HOOD_ANGLE_INTERPOLATOR.getInterpolatedValue(distance)
        + Constants.StateMachine.HOOD_SHOT_OFFSET_ROTATIONS,
        Constants.ShooterHood.SHOOTER_HOOD_MIN, Constants.ShooterHood.SHOOTER_HOOD_MAX);
    double feeder = Constants.StateMachine.FEEDER_SHOOTER_RATIO * demand;
    return new ShotSolution(true, distance, heading, error, demand, hood, feeder,
        Double.isFinite(shooterRps) && Math.abs(shooterRps - demand) <= Constants.Shooting.SHOOTER_TOLERANCE_RPS,
        Double.isFinite(hoodRotations) && Math.abs(hoodRotations - hood) <= Constants.StateMachine.HOOD_TOLERANCE_ROTATIONS,
        Math.abs(error) <= Constants.Shooting.HEADING_TOLERANCE_DEGREES, feedbackHealthy);
  }

  public static void stopFeeding(StateMachine robot) {
    robot.feeder.stopVelocity();
    robot.feeder.brake();
    robot.spindexer.stopVelocity();
    robot.spindexer.brake();
  }

  public static void stopShooter(StateMachine robot) {
    robot.shooter.stopVelocity();
    robot.shooter.brake();
  }

  public static void applyShooter(StateMachine robot, ShotSolution shot) {
    if (!shot.valid()) { stopShooter(robot); return; }
    robot.shooter.setVelocity(shot.shooterRps());
    robot.shooterHood.setPositionRotations(shot.hoodRotations());
  }

}
