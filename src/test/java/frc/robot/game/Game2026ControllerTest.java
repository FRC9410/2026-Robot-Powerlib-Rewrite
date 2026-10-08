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
    return controller.update(input(alliance, pose, shot.shooterRps()+1, shot.hoodRotations(), true, now), settings);
  }
  @Test void alignedReadyRobotFeedsForBothAlliances() {
    for (Alliance alliance : Alliance.values()) {
      Pose2d pose = alliance == Alliance.Blue ? bluePose() : new Pose2d(14, 4, new Rotation2d());
      var out = ready(alliance, pose, 0);
      assertTrue(out.feedReady());
      assertTrue(out.shooterRps() > 0);
      assertTrue(out.feederRps() < 0);
      assertEquals(60, out.spindexerRps());
    }
  }
  @Test void oppositeSignedSpeedNeverOpensFeedGate() {
    var shot = config.shot(2.62);
    var out = controller.update(input(Alliance.Blue, bluePose(), -shot.shooterRps()-1, shot.hoodRotations(), true, 0), settings);
    assertFalse(out.feedReady()); assertEquals(0, out.feederRps()); assertEquals(0, out.spindexerRps());
  }
  @Test void dashboardReadinessReportsEachIndependentFeedCondition() {
    var shot = config.shot(2.62);
    var spinning = controller.update(input(Alliance.Blue, bluePose(), 0, shot.hoodRotations(), true, 0), settings);
    assertFalse(spinning.velocityReady()); assertTrue(spinning.hoodReady());
    assertTrue(spinning.aligned()); assertTrue(spinning.calibratedRange()); assertFalse(spinning.feedReady());
    var hoodMoving = controller.update(input(Alliance.Blue, bluePose(), shot.shooterRps()+1, shot.hoodRotations()+.02, true, 0), settings);
    assertTrue(hoodMoving.velocityReady()); assertFalse(hoodMoving.hoodReady()); assertFalse(hoodMoving.feedReady());
    var aiming = controller.update(input(Alliance.Blue, new Pose2d(2, 4, Rotation2d.fromDegrees(30)), shot.shooterRps()+1, shot.hoodRotations(), true, 0), settings);
    assertFalse(aiming.aligned()); assertFalse(aiming.feedReady());
    var outside = ready(Alliance.Blue, new Pose2d(8, 4, new Rotation2d()), 0);
    assertFalse(outside.calibratedRange()); assertFalse(outside.feedReady());
  }
  @Test void hoodHeadingAndFeedbackEachBlockFeeding() {
    var shot = config.shot(2.62);
    assertFalse(controller.update(input(Alliance.Blue, bluePose(), shot.shooterRps()+1, shot.hoodRotations()+.02, true, 0), settings).feedReady());
    assertFalse(controller.update(input(Alliance.Blue, new Pose2d(2, 4, Rotation2d.fromDegrees(30)), shot.shooterRps()+1, shot.hoodRotations(), true, 0), settings).feedReady());
    assertFalse(controller.update(input(Alliance.Blue, bluePose(), shot.shooterRps()+1, shot.hoodRotations(), false, 0), settings).feedReady());
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
  @Test void shooterDemandUsesLegacyInterpolatorInsteadOfGameJsonVelocity() throws Exception {
    var data = (com.fasterxml.jackson.databind.node.ObjectNode) new com.fasterxml.jackson.databind.ObjectMapper()
        .readTree(Path.of("power-tool/generated/powerlib-game-2026.json").toFile());
    for (var row : data.withArray("shots")) {
      ((com.fasterxml.jackson.databind.node.ArrayNode) row).set(1,
          com.fasterxml.jackson.databind.node.JsonNodeFactory.instance.numberNode(120));
    }
    config = Game2026Config.fromJson(data);
    controller = new Game2026Controller(config);
    controller.setWantedState(Game2026Controller.State.SHOOTING);
    double distance = (2.5 + 3.0) / 2;
    var hub = config.hub(Alliance.Blue);
    var pose = new Pose2d(hub.getX() - distance, hub.getY(), new Rotation2d());
    var out = controller.update(input(Alliance.Blue, pose, 30,
        config.shot(distance).hoodRotations(), true, 0), settings);
    assertEquals(30, out.shooterRps(), 1e-9);
    assertTrue(out.velocityReady());
    assertTrue(out.feedReady());
    var offset = new Game2026Settings(.005, 3, 60, 6, .5, 15, 3, 3.2, .2);
    assertEquals(32, controller.update(input(Alliance.Blue, pose, 32,
        config.shot(distance).hoodRotations(), true, 0), offset).shooterRps(), 1e-9);
  }

  @Test void liveWristSnapshotChangesTargetsWithoutRestartingController() {
    var live = new Game2026Config.Intake(-.42, -.35, -.22, -.1, 145, -100);
    var in = input(Alliance.Blue, bluePose(), -70, .05, true, 0);
    controller.setWantedState(Game2026Controller.State.READY);
    assertEquals(-.35, controller.update(in, settings, live).wristRotations());
    var edited = new Game2026Config.Intake(-.43, -.36, -.23, -.11, 145, -100);
    assertEquals(-.36, controller.update(in, settings, edited).wristRotations());
    controller.setIntakeMode(Game2026Controller.IntakeMode.COLLECT);
    assertEquals(-.43, controller.update(in, settings, edited).wristRotations());
    controller.setWantedState(Game2026Controller.State.IDLE);
    assertEquals(-.11, controller.update(in, settings, edited).wristRotations());
    controller.setIntakeMode(Game2026Controller.IntakeMode.OFF);
    controller.setWantedState(Game2026Controller.State.SHOOTING);
    controller.update(in, settings, edited);
    assertEquals(-.23, controller.update(
        input(Alliance.Blue, bluePose(), -70, .05, true, 1.02), settings, edited).wristRotations());
  }
}
