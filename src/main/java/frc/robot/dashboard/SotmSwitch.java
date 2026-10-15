package frc.robot.dashboard;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.utils.ShootingUtils;

/**
 * Drive tab Shoot on move button. Power Tool writes the request topic; the robot applies each
 * new request every loop, independent of tuning mode, and reports the result as
 * Shooting/SOTMEnabled.
 */
public final class SotmSwitch {
  public static final String REQUEST_TOPIC = "/PowerLib/Shooting/SOTMRequestedEnabled";
  private static BooleanSubscriber request;

  private SotmSwitch() {}

  /** Applies requests received since the last call; call once per loop. */
  public static void poll() {
    if (request == null) {
      // Keep duplicates so a repeated request still applies after code changed the switch.
      request = NetworkTableInstance.getDefault().getBooleanTopic(REQUEST_TOPIC)
          .subscribe(ShootingUtils.sotmEnabled(), PubSubOption.keepDuplicates(true));
    }
    // Only new writes change the switch, so a stale value cannot fight the startup default.
    for (boolean requested : request.readQueueValues()) ShootingUtils.setSotmEnabled(requested);
  }
}
