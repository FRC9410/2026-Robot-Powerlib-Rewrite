package frc.robot.game;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.nio.file.Path;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class Game2026ControllerTest {
  private Game2026Config config;
  private Game2026Controller controller;
  private final Game2026Settings settings = new Game2026Settings(.005, 1, 60, 6, .5, 15, 3, 3.2, .2);
  @BeforeEach void setup() throws Exception {
    config = Game2026Config.read(Path.of("power-tool/generated/powerlib-game-2026.json"));
    controller = new Game2026Controller(config);
    controller.setWantedState(Game2026Controller.State.SHOOTING);
  }
  private Game2026Controller.Inputs input(Alliance alliance, Pose2d pose, double speed, double hood, boolean health, double now) {
    return new Game2026Controller.Inputs(true, false, alliance, pose, speed, hood, health, now);
  }
  private Pose2d bluePose() { return new Pose2d(2, 4, new Rotation2d()); }
  private Game2026Controller.Output ready(Alliance alliance, Pose2d pose, double now) {
    var shot = config.shot(pose.getTranslation().getDistance(config.hub(alliance)));
    return controller.update(input(alliance, pose, -shot.shooterRps()-1, shot.hoodRotations(), true, now), settings);
  }
  @Test void alignedReadyRobotFeedsForBothAlliances() {
    for (Alliance alliance : Alliance.values()) {
      Pose2d pose = alliance == Alliance.Blue ? bluePose() : new Pose2d(14, 4, new Rotation2d());
      var out = ready(alliance, pose, 0);
      assertTrue(out.feedReady());
      assertTrue(out.shooterRps() < 0);
      assertTrue(out.feederRps() < 0);
      assertEquals(60, out.spindexerRps());
    }
  }
  @Test void oppositeSignedSpeedNeverOpensFeedGate() {
    var shot = config.shot(2.62);
    var out = controller.update(input(Alliance.Blue, bluePose(), shot.shooterRps()+1, shot.hoodRotations(), true, 0), settings);
    assertFalse(out.feedReady()); assertEquals(0, out.feederRps()); assertEquals(0, out.spindexerRps());
  }
  @Test void hoodHeadingAndFeedbackEachBlockFeeding() {
    var shot = config.shot(2.62);
    assertFalse(controller.update(input(Alliance.Blue, bluePose(), -shot.shooterRps()-1, shot.hoodRotations()+.02, true, 0), settings).feedReady());
    assertFalse(controller.update(input(Alliance.Blue, new Pose2d(2, 4, Rotation2d.fromDegrees(30)), -shot.shooterRps()-1, shot.hoodRotations(), true, 0), settings).feedReady());
    assertFalse(controller.update(input(Alliance.Blue, bluePose(), -shot.shooterRps()-1, shot.hoodRotations(), false, 0), settings).feedReady());
  }
  @Test void outsideScoringZoneNeverSelectsPassingTarget() {
    var out = ready(Alliance.Blue, new Pose2d(8, 4, new Rotation2d()), 0);
    assertFalse(out.runShooter()); assertFalse(out.feedReady()); assertEquals(0, out.feederRps());
    assertEquals("OUTSIDE SCORING ZONE", out.reason());
  }
  @Test void unknownAllianceAndNonfiniteFeedbackBlockFeeding() {
    assertFalse(controller.update(input(null, bluePose(), -70, .05, true, 0), settings).runShooter());
    assertFalse(controller.update(input(Alliance.Blue, bluePose(), Double.NaN, .05, true, 0), settings).feedReady());
  }
  @Test void disabledResetsLatchedShootAndIntakeRequests() {
    controller.setIntakeMode(Game2026Controller.IntakeMode.COLLECT);
    var off = controller.update(new Game2026Controller.Inputs(false, false, Alliance.Blue, bluePose(), -70, .05, true, 0), settings);
    assertFalse(off.runShooter()); assertEquals(0, off.rollerRps());
    var resumed = controller.update(input(Alliance.Blue, bluePose(), -70, .05, true, 1), settings);
    assertFalse(resumed.runShooter()); assertEquals(0, resumed.rollerRps());
  }
  @Test void testModeYieldsOutputsAndClearsRequests() {
    var out = controller.update(new Game2026Controller.Inputs(true, true, Alliance.Blue, bluePose(), -70, .05, true, 0), settings);
    assertFalse(out.runShooter()); assertFalse(out.feedReady()); assertEquals(0, out.feederRps());
    assertEquals("SYSID OWNS OUTPUTS", out.reason());
  }
  @Test void collectEjectAndReleaseUseReferenceSetpoints() {
    controller.setIntakeMode(Game2026Controller.IntakeMode.COLLECT);
    var collecting = ready(Alliance.Blue, bluePose(), 0);
    assertEquals(145, collecting.rollerRps()); assertEquals(-.445, collecting.wristRotations());
    controller.setIntakeMode(Game2026Controller.IntakeMode.EJECT);
    var ejecting = ready(Alliance.Blue, bluePose(), .1);
    assertEquals(-100, ejecting.rollerRps()); assertFalse(ejecting.feedReady());
    controller.setWantedState(Game2026Controller.State.READY);
    controller.setIntakeMode(Game2026Controller.IntakeMode.OFF);
    var released = ready(Alliance.Blue, bluePose(), .2);
    assertEquals(0, released.rollerRps()); assertEquals(-.4, released.wristRotations());
  }
  @Test void agitationStartsAfterOneSecondAndStopsOnRelease() {
    assertEquals(-.4, ready(Alliance.Blue, bluePose(), 0).wristRotations());
    assertEquals(-.25, ready(Alliance.Blue, bluePose(), 1.02).wristRotations());
    assertEquals(-.4, ready(Alliance.Blue, bluePose(), 1.52).wristRotations());
    controller.setWantedState(Game2026Controller.State.READY);
    assertEquals(-.4, ready(Alliance.Blue, bluePose(), 2).wristRotations());
    controller.setWantedState(Game2026Controller.State.SHOOTING);
    assertEquals(-.4, ready(Alliance.Blue, bluePose(), 2.1).wristRotations());
  }
  @Test void idleStopsIntakeEvenWithHeldCollectRequest() {
    controller.setWantedState(Game2026Controller.State.IDLE);
    controller.setIntakeMode(Game2026Controller.IntakeMode.COLLECT);
    var out = ready(Alliance.Blue, bluePose(), 0);
    assertEquals(0, out.rollerRps()); assertFalse(out.runShooter());
  }
  @Test void invalidLiveSettingsBlockAllMechanismDemands() {
    var invalid = new Game2026Settings(.005, Double.NaN, 60, 6, .5, 15, 3, 3.2, .2);
    var out = controller.update(input(Alliance.Blue, bluePose(), -70, .05, true, 0), invalid);
    assertFalse(out.runShooter()); assertEquals(0, out.rollerRps()); assertFalse(out.feedReady());
  }
}
