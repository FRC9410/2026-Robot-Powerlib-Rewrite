package frc.robot.dashboard;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Estimated 2026 hub windows from DS match time and the FMS first-inactive alliance. */
public final class HubTimer {
  private HubTimer() {}

  public record Window(Boolean active, String phase, double secondsRemaining) {}

  public static Window calculate(Alliance alliance, boolean autonomous, boolean teleop,
      double time, String gameData) {
    if (!autonomous && !teleop) return new Window(null, "Disabled / Test", Double.NaN);
    if (autonomous) return new Window(alliance == null ? null : true, "Autonomous",
        Double.isFinite(time) && time >= 0 && time <= 20 ? time : Double.NaN);
    if (!Double.isFinite(time) || time < 0 || time > 140) {
      return new Window(null, "No match clock", Double.NaN);
    }
    if (time > 130) return new Window(alliance == null ? null : true, "Transition", time - 130);
    if (time <= 30) return new Window(alliance == null ? null : true, "End game", time);
    int shift = time > 105 ? 1 : time > 80 ? 2 : time > 55 ? 3 : 4;
    double remaining = time - (130 - shift * 25);
    if (alliance == null || gameData == null || gameData.length() != 1
        || (gameData.charAt(0) != 'R' && gameData.charAt(0) != 'B')) {
      return new Window(null, "Shift " + shift, remaining);
    }
    boolean inactiveFirst = gameData.charAt(0) == (alliance == Alliance.Red ? 'R' : 'B');
    boolean active = shift % 2 == 0 ? inactiveFirst : !inactiveFirst;
    return new Window(active, "Shift " + shift, remaining);
  }
}
