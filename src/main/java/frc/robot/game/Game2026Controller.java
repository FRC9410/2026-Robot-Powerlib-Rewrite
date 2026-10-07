package frc.robot.game;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Mechanism decisions independent of CAN devices, for deterministic playbook tests. */
public final class Game2026Controller {
  public enum State { IDLE, READY, SHOOTING }
  public enum IntakeMode { OFF, COLLECT, EJECT }
  public record Inputs(boolean enabled, boolean testMode, Alliance alliance, Pose2d pose,
      double shooterRps, double hoodRotations, boolean feedbackHealthy, double nowSeconds) {}
  public record Output(double rollerRps, double wristRotations, double shooterRps, double hoodRotations,
      double feederRps, double spindexerRps, boolean runShooter, boolean feedReady,
      boolean hubAvailable, double headingErrorDegrees, double distance, String reason) {}

  private final Game2026Config config;
  private State wanted = State.READY;
  private IntakeMode intakeMode = IntakeMode.OFF;
  private double shootingStarted = Double.NaN;

  public Game2026Controller(Game2026Config config) { this.config = config; }
  public void setWantedState(State state) { wanted = state; }
  public void setIntakeMode(IntakeMode mode) { intakeMode = mode; }
  public IntakeMode intakeMode() { return intakeMode; }
  public void reset() { wanted = State.READY; intakeMode = IntakeMode.OFF; shootingStarted = Double.NaN; }

  public Output update(Inputs in, Game2026Settings settings) {
    var intake = config.intake();
    if (!in.enabled() || in.testMode()) {
      reset();
      return idle(intake.idle(), 0, in.testMode() ? "SYSID OWNS OUTPUTS" : "DISABLED");
    }
    if (!settings.valid()) return idle(intake.idle(), 0, "INVALID BEHAVIOR SETTINGS");
    if (wanted == State.IDLE) { shootingStarted = Double.NaN; return idle(intake.stowed(), 0, "IDLE"); }
    double roller = intakeMode == IntakeMode.COLLECT ? intake.collectRps() : intakeMode == IntakeMode.EJECT ? intake.ejectRps() : 0;
    double wrist = intakeMode == IntakeMode.OFF ? intake.idle() : intake.deployed();
    if (wanted != State.SHOOTING) { shootingStarted = Double.NaN; return idle(wrist, roller, wanted.name()); }
    if (!Game2026Config.finitePose(in.pose()) || !Double.isFinite(in.nowSeconds()) || in.alliance() == null) {
      shootingStarted = Double.NaN;
      return idle(wrist, roller, "POSE OR ALLIANCE UNKNOWN");
    }
    double distance = in.pose().getTranslation().getDistance(config.hub(in.alliance()));
    double headingError = MathUtil.inputModulus(config.hubHeadingDegrees(in.pose(), in.alliance())
        - in.pose().getRotation().getDegrees(), -180, 180);
    boolean inZone = config.inScoringZone(in.pose(), in.alliance());
    if (!inZone || !config.inShotRange(distance)) {
      shootingStarted = Double.NaN;
      return new Output(roller, wrist, 0, 0, 0, 0, false, false, inZone, headingError, distance,
          inZone ? "OUTSIDE SHOT TABLE" : "OUTSIDE SCORING ZONE");
    }
    var shot = config.shot(distance);
    double demand = -shot.shooterRps() - settings.shooterExtraRps();
    boolean velocityReady = Double.isFinite(in.shooterRps()) && Math.abs(in.shooterRps() - demand) <= config.number("tolerances", "shooterRps");
    boolean hoodReady = Double.isFinite(in.hoodRotations()) && Math.abs(in.hoodRotations() - shot.hoodRotations()) <= settings.hoodTolerance();
    boolean aligned = Math.abs(headingError) <= config.number("tolerances", "headingDegrees");
    boolean ready = intakeMode != IntakeMode.EJECT && in.feedbackHealthy() && velocityReady && hoodReady && aligned;
    if (Double.isNaN(shootingStarted) || in.nowSeconds() < shootingStarted) shootingStarted = in.nowSeconds();
    double elapsed = in.nowSeconds() - shootingStarted;
    if (intakeMode == IntakeMode.OFF && elapsed >= 1 && elapsed % 1 < 0.5) wrist = intake.feed();
    String reason = intakeMode == IntakeMode.EJECT ? "EJECTING" : !in.feedbackHealthy() ? "FEEDBACK UNAVAILABLE" : !velocityReady ? "SPINNING UP"
        : !hoodReady ? "HOOD MOVING" : !aligned ? "AIMING" : "READY TO FEED";
    return new Output(roller, wrist, demand, shot.hoodRotations(), ready ? -shot.feederRps() : 0,
        ready ? settings.spindexerRps() : 0, true, ready, true, headingError, distance, reason);
  }

  private static Output idle(double wrist, double roller, String reason) {
    return new Output(roller, wrist, 0, 0, 0, 0, false, false, false, 0, 0, reason);
  }
}
