package frc.robot.game;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Validated game data copied from power-tool/generated into deploy by Gradle. */
public final class Game2026Config {
  public record Shot(double distance, double shooterRps, double hoodRotations, double feederRps) {}
  public record Intake(double deployed, double idle, double feed, double stowed,
      double collectRps, double ejectRps) {}

  private final JsonNode data;
  private final List<Shot> shots;
  private final Map<String, List<Pose2d>> autos;
  private final Intake intake;

  private Game2026Config(JsonNode source) {
    if (source == null || !source.isObject()) throw new IllegalArgumentException("Game config must be an object");
    data = source.deepCopy();
    if (data.path("version").asInt() != 1) throw new IllegalArgumentException("Unsupported game config version");
    requirePositive(number("field", "length"), "field.length");
    requirePositive(number("field", "width"), "field.width");
    point(data.path("field").path("blueHub"));
    point(data.path("field").path("redHub"));
    for (String key : new String[]{"BLUE_START_X", "BLUE_END_X", "RED_START_X", "RED_END_X",
        "CENTER_START_X", "CENTER_END_X", "AUTO_LENGTH_IN_TIME"}) fieldConstant(key);
    for (String key : new String[]{"shooterRps", "headingDegrees", "autoTranslationMeters", "autoRotationDegrees"}) {
      requirePositive(number("tolerances", key), "tolerances." + key);
    }
    intake = new Intake(number("intake", "deployed"), number("intake", "idle"),
        number("intake", "feed"), number("intake", "stowed"),
        number("intake", "collectRps"), number("intake", "ejectRps"));
    List<Shot> parsedShots = new ArrayList<>();
    JsonNode shotRows = data.path("shots");
    if (!shotRows.isArray() || shotRows.size() < 2) throw new IllegalArgumentException("At least two shot rows are required");
    double previous = -1;
    for (JsonNode row : shotRows) {
      if (!row.isArray() || row.size() != 4) throw new IllegalArgumentException("Shot rows need four numbers");
      double distance = finite(row.get(0));
      if (distance <= previous) throw new IllegalArgumentException("Shot distances must increase");
      double shooter = finite(row.get(1)), hood = finite(row.get(2)), feeder = finite(row.get(3));
      requirePositive(shooter, "shot.shooterRps");
      requirePositive(feeder, "shot.feederRps");
      parsedShots.add(new Shot(distance, shooter, hood, feeder));
      previous = distance;
    }
    shots = List.copyOf(parsedShots);
    Map<String, List<Pose2d>> parsedAutos = new LinkedHashMap<>();
    for (String name : new String[]{"Red Left", "Red Right", "Blue Left", "Blue Right"}) {
      JsonNode route = data.path("autos").path(name);
      if (!route.isArray() || route.size() != 7) throw new IllegalArgumentException("Auto needs seven poses: " + name);
      List<Pose2d> poses = new ArrayList<>();
      for (JsonNode row : route) {
        if (!row.isArray() || row.size() != 3) throw new IllegalArgumentException("Auto poses need three numbers");
        double x = finite(row.get(0)), y = finite(row.get(1)), degrees = finite(row.get(2));
        if (x < 0 || x > number("field", "length") || y < 0 || y > number("field", "width")) {
          throw new IllegalArgumentException("Auto pose outside field: " + name);
        }
        // Legacy red headings were operator-relative. Control below uses the blue field frame.
        poses.add(new Pose2d(x, y, Rotation2d.fromDegrees(MathUtil.inputModulus(
            degrees + (name.startsWith("Red") ? 180 : 0), -180, 180))));
      }
      parsedAutos.put(name, List.copyOf(poses));
    }
    autos = Map.copyOf(parsedAutos);
  }

  public static Game2026Config load() {
    try {
      return read(Filesystem.getDeployDirectory().toPath().resolve("powerlib-game-2026.json"));
    } catch (IOException | IllegalArgumentException error) {
      throw new IllegalStateException("Invalid deployed game config; build to copy power-tool/generated/powerlib-game-2026.json", error);
    }
  }

  public static Game2026Config read(Path path) throws IOException {
    return fromJson(new ObjectMapper().readTree(path.toFile()));
  }

  public static Game2026Config fromJson(JsonNode data) { return new Game2026Config(data); }
  public Intake intake() { return intake; }
  public List<Shot> shots() { return shots; }
  public List<Pose2d> auto(String name) {
    List<Pose2d> route = autos.get(name);
    if (route == null) throw new IllegalArgumentException("Unknown autonomous: " + name);
    return route;
  }
  public double number(String section, String key) { return finite(data.path(section).path(key)); }
  public double fieldConstant(String key) { return finite(data.path("field").path("constants").path(key)); }
  public Translation2d hub(Alliance alliance) {
    return point(data.path("field").path(alliance == Alliance.Red ? "redHub" : "blueHub"));
  }

  public boolean inScoringZone(Pose2d pose, Alliance alliance) {
    if (!finitePose(pose) || alliance == null || pose.getY() < 0 || pose.getY() > number("field", "width")) return false;
    String prefix = alliance == Alliance.Red ? "RED" : "BLUE";
    return pose.getX() > fieldConstant(prefix + "_START_X") && pose.getX() < fieldConstant(prefix + "_END_X");
  }

  /** Preserve the source robot's alliance-dependent shooter orientation. */
  public double hubHeadingDegrees(Pose2d pose, Alliance alliance) {
    return hub(alliance).minus(pose.getTranslation()).getAngle().getDegrees() - (alliance == Alliance.Red ? 180 : 0);
  }

  public boolean inShotRange(double distance) {
    return Double.isFinite(distance) && distance >= shots.get(0).distance() && distance <= shots.get(shots.size() - 1).distance();
  }

  public Shot shot(double distance) {
    if (!Double.isFinite(distance)) throw new IllegalArgumentException("Shot distance must be finite");
    if (distance <= shots.get(0).distance()) return shots.get(0);
    for (int i = 1; i < shots.size(); i++) {
      Shot high = shots.get(i), low = shots.get(i - 1);
      if (distance <= high.distance()) {
        double t = (distance - low.distance()) / (high.distance() - low.distance());
        return new Shot(distance, lerp(low.shooterRps(), high.shooterRps(), t),
            lerp(low.hoodRotations(), high.hoodRotations(), t), lerp(low.feederRps(), high.feederRps(), t));
      }
    }
    return shots.get(shots.size() - 1);
  }

  public static boolean finitePose(Pose2d pose) {
    return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }
  private static Translation2d point(JsonNode row) {
    if (!row.isArray() || row.size() != 2) throw new IllegalArgumentException("Hub needs two coordinates");
    return new Translation2d(finite(row.get(0)), finite(row.get(1)));
  }
  private static double finite(JsonNode node) {
    if (node == null || !node.isNumber() || !Double.isFinite(node.asDouble())) throw new IllegalArgumentException("Missing or invalid game number: " + node);
    return node.asDouble();
  }
  private static void requirePositive(double value, String name) {
    if (value <= 0) throw new IllegalArgumentException(name + " must be positive");
  }
  private static double lerp(double low, double high, double t) { return low + (high - low) * t; }
}
