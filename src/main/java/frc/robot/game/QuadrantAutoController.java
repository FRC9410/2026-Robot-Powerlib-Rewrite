package frc.robot.game;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

/** Original seven-leg collection sequence with explicit field-frame control and bounded duration. */
public final class QuadrantAutoController {
  public enum Status { IDLE, RUNNING, SHOOTING, ABORTED, FINISHED, CANCELLED }
  public record Output(double vx, double vy, double omega, boolean collect, boolean shoot,
      Status status, String reason) {}
  private static final double[] TOLERANCE_INCHES = {6, 6, 12, 6, 6, 3, 3};
  private static final double[] SPEED_COEFFICIENTS = {0.5, 1, 0.75, 0.75, 0.4, 0.75, 1};
  private final Game2026Config config;
  private final String selection;
  private final List<Pose2d> path;
  private Alliance alliance;
  private Status status = Status.IDLE;
  private String reason = "";
  private int waypoint;
  private double started, legStarted, previousTime, previousDistance;

  public QuadrantAutoController(Game2026Config config, String selection) {
    this.config = config;
    this.selection = selection;
    path = config.auto(selection);
  }
  public Status status() { return status; }
  public String reason() { return reason; }
  public boolean done() { return status == Status.ABORTED || status == Status.FINISHED || status == Status.CANCELLED; }

  public Output start(Pose2d pose, Alliance reportedAlliance, boolean healthy, double now, Game2026Settings settings) {
    waypoint = 0;
    started = legStarted = previousTime = now;
    previousDistance = 0;
    alliance = reportedAlliance;
    status = Status.RUNNING;
    reason = "";
    if (!settings.valid()) return abort("INVALID BEHAVIOR SETTINGS");
    if (alliance == null || !selection.startsWith(alliance.name())) return abort("ALLIANCE DOES NOT MATCH AUTO");
    if (!healthy || !Game2026Config.finitePose(pose) || !Double.isFinite(now)) return abort("DRIVETRAIN FEEDBACK UNAVAILABLE");
    if (pose.getTranslation().getDistance(path.get(0).getTranslation()) > settings.autoStartMeters()
        || Math.abs(headingError(pose, path.get(0))) > settings.autoStartDegrees()) return abort("STARTING POSE OUTSIDE TOLERANCE");
    previousDistance = pose.getTranslation().getDistance(path.get(0).getTranslation());
    return stopped();
  }

  public Output update(Pose2d pose, Alliance reportedAlliance, boolean autonomousEnabled, boolean healthy,
      double now, double maxSpeed, double maxOmega, Game2026Settings settings) {
    if (done() || status == Status.IDLE) return stopped();
    if (!settings.valid()) return abort("INVALID BEHAVIOR SETTINGS");
    if (!autonomousEnabled) { status = Status.CANCELLED; reason = "AUTONOMOUS ENDED"; return stopped(); }
    if (reportedAlliance != alliance) return abort("ALLIANCE CHANGED");
    if (!healthy || !Game2026Config.finitePose(pose) || !Double.isFinite(now) || now < previousTime) return abort("DRIVETRAIN FEEDBACK LOST");
    if (!Double.isFinite(maxSpeed) || maxSpeed <= 0 || !Double.isFinite(maxOmega) || maxOmega <= 0) return abort("INVALID DRIVE LIMITS");
    if (now - started >= config.fieldConstant("AUTO_LENGTH_IN_TIME")) {
      if (status == Status.SHOOTING) { status = Status.FINISHED; reason = "AUTO DURATION COMPLETE"; return stopped(); }
      return abort("AUTO DURATION EXCEEDED");
    }
    if (waypoint >= path.size()) {
      double error = MathUtil.inputModulus(config.hubHeadingDegrees(pose, alliance) - pose.getRotation().getDegrees(), -180, 180);
      double omega = MathUtil.clamp(Math.toRadians(error) * settings.headingKp(), -maxOmega, maxOmega);
      status = Status.SHOOTING;
      reason = "AIMING AT HUB";
      previousTime = now;
      return new Output(0, 0, omega, false, true, status, reason);
    }
    Pose2d goal = path.get(waypoint);
    double dx = goal.getX() - pose.getX(), dy = goal.getY() - pose.getY();
    double distance = Math.hypot(dx, dy), angle = headingError(pose, goal);
    boolean lockHeading = waypoint != 1 && waypoint != 6;
    if (distance < TOLERANCE_INCHES[waypoint] * 0.0254 && (!lockHeading || Math.abs(angle) < 10)) {
      waypoint++;
      legStarted = previousTime = now;
      previousDistance = waypoint < path.size() ? pose.getTranslation().getDistance(path.get(waypoint).getTranslation()) : 0;
      reason = "WAYPOINT " + waypoint + " COMPLETE";
      return new Output(0, 0, 0, waypoint >= 2 && waypoint < 5, false, status, reason);
    }
    if (now - legStarted >= settings.autoLegSeconds()) return abort("WAYPOINT " + (waypoint + 1) + " TIMED OUT");
    double dt = now - previousTime;
    double derivative = dt > 0 ? (distance - previousDistance) / dt : 0;
    double speed = Math.abs(settings.autoTranslationKp() * distance + settings.autoTranslationKd() * derivative);
    if (distance >= 3 * 0.0254) speed += 0.085 * maxSpeed;
    speed = Math.min(speed, maxSpeed * SPEED_COEFFICIENTS[waypoint]);
    double omega = lockHeading ? MathUtil.clamp(Math.toRadians(angle) * settings.headingKp(), -maxOmega, maxOmega) : 0;
    previousTime = now;
    previousDistance = distance;
    reason = "WAYPOINT " + (waypoint + 1) + " / " + path.size();
    return new Output(distance == 0 ? 0 : dx / distance * speed, distance == 0 ? 0 : dy / distance * speed,
        omega, waypoint >= 2 && waypoint < 5, false, status, reason);
  }

  public void cancel() { if (!done()) { status = Status.CANCELLED; reason = "CANCELLED"; } }
  private Output abort(String message) { status = Status.ABORTED; reason = message; return stopped(); }
  private Output stopped() { return new Output(0, 0, 0, false, false, status, reason); }
  private static double headingError(Pose2d pose, Pose2d goal) {
    return MathUtil.inputModulus(goal.getRotation().getDegrees() - pose.getRotation().getDegrees(), -180, 180);
  }
}
