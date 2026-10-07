package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.game.Game2026Config;

public final class FieldUtils {
  public enum GameZone { RED_ALLIANCE, NEUTRAL, BLUE_ALLIANCE, INTERCHANGE }
  private static Game2026Config config;
  private FieldUtils() {}
  public static void configure(Game2026Config gameConfig) { config = gameConfig; }
  public static GameZone getZone(Pose2d pose) {
    if (config == null || !Game2026Config.finitePose(pose)) return GameZone.INTERCHANGE;
    double x = pose.getX();
    if (between(x, "BLUE")) return GameZone.BLUE_ALLIANCE;
    if (between(x, "RED")) return GameZone.RED_ALLIANCE;
    if (between(x, "CENTER")) return GameZone.NEUTRAL;
    return GameZone.INTERCHANGE;
  }
  private static boolean between(double x, String prefix) {
    return x > config.fieldConstant(prefix + "_START_X") && x < config.fieldConstant(prefix + "_END_X");
  }
}
