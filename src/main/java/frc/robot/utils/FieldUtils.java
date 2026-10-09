package frc.robot.utils;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.powerlib.health.HealthChecks;

public final class FieldUtils {
  public enum GameZone { RED_ALLIANCE, NEUTRAL, BLUE_ALLIANCE, INTERCHANGE }
  private FieldUtils() {}

  public static GameZone getZone(Pose2d pose) {
    if (!HealthChecks.finitePose(pose)) return GameZone.INTERCHANGE;
    double x = pose.getX();
    if (x > Constants.Shooting.BLUE_ZONE_START && x < Constants.Shooting.BLUE_ZONE_END) return GameZone.BLUE_ALLIANCE;
    if (x > Constants.Shooting.RED_ZONE_START && x < Constants.Shooting.RED_ZONE_END) return GameZone.RED_ALLIANCE;
    if (x > Constants.Shooting.CENTER_ZONE_START && x < Constants.Shooting.CENTER_ZONE_END) return GameZone.NEUTRAL;
    return GameZone.INTERCHANGE;
  }
}
