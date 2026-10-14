package frc.robot.dashboard;

import edu.wpi.first.wpilibj.DriverStation;
import frc.powerlib.PowerRobotContainer;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/** Dashboard-only season data; control remains in the existing state handlers. */
public final class Game2026Telemetry {
  private Game2026Telemetry() {}

  public static void publish(StateMachine robot) {
    var shot = robot.getShotSolution();
    boolean requested = DriverStation.isEnabled() && !DriverStation.isTest()
        && (robot.getWantedState() == RobotState.SHOOTING || robot.getWantedState() == RobotState.SPINNING_UP);
    setShot("ShotRequested", requested);
    setShot("VelocityReady", requested && shot.velocityReady());
    setShot("HoodReady", requested && shot.hoodReady());
    setShot("Aligned", requested && shot.aligned());
    setShot("CalibratedRange", requested && shot.valid());
    setShot("FeedReady", requested && shot.canFeed());
    setShot("HeadingErrorDegrees", requested ? shot.headingErrorDegrees() : Double.NaN);
    setShot("VisionAccepted", robot.vision.hasFreshMeasurement());
    setShot("Status", !requested ? "No shot requested" : !shot.valid() ? "No valid calibrated shot"
        : !shot.feedbackHealthy() ? "Waiting for fresh control feedback"
        : !shot.velocityReady() ? "Waiting for flywheel"
        : !shot.hoodReady() ? "Waiting for hood"
        : !shot.aligned() ? "Waiting for alignment" : "Mechanisms ready");

    var hub = HubTimer.calculate(DriverStation.getAlliance().orElse(null),
        DriverStation.isAutonomousEnabled(), DriverStation.isTeleopEnabled(),
        DriverStation.getMatchTime(), DriverStation.getGameSpecificMessage());
    PowerRobotContainer.setSubsystemData("Game2026", "HubActive", hub.active());
    PowerRobotContainer.setSubsystemData("Game2026", "HubPhase", hub.phase());
    PowerRobotContainer.setSubsystemData("Game2026", "HubTimeSeconds", hub.secondsRemaining(), "seconds");
  }

  private static void setShot(String key, Object value) {
    PowerRobotContainer.setSubsystemData("Shooting", key, value);
  }
}
