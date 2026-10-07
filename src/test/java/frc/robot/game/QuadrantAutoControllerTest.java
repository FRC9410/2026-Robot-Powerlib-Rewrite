package frc.robot.game;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.nio.file.Path;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class QuadrantAutoControllerTest {
  private Game2026Config config;
  private final Game2026Settings settings = new Game2026Settings(.005, 1, 60, 6, .5, 15, 3, 3.2, .2);
  @BeforeEach void setup() throws Exception { config = Game2026Config.read(Path.of("power-tool/generated/powerlib-game-2026.json")); }
  private QuadrantAutoController.Output update(QuadrantAutoController auto, Pose2d pose, Alliance alliance, double now) {
    return auto.update(pose, alliance, true, true, now, 5.72, 4.71238898038469, settings);
  }
  @Test void allFourAutosCollectOnlyBetweenSecondAndFifthWaypoints() {
    for (String selection : new String[]{"Blue Left", "Blue Right", "Red Left", "Red Right"}) {
      Alliance alliance = selection.startsWith("Red") ? Alliance.Red : Alliance.Blue;
      var auto = new QuadrantAutoController(config, selection);
      var route = config.auto(selection);
      auto.start(route.get(0), alliance, true, 0, settings);
      for (int i=0; i<7; i++) {
        var out = update(auto, route.get(i), alliance, .1*(i+1));
        assertEquals(i+1 >= 2 && i+1 < 5, out.collect());
        assertFalse(out.shoot());
      }
      var shooting = update(auto, route.get(6), alliance, 1);
      assertTrue(shooting.shoot()); assertFalse(shooting.collect());
      assertEquals(0, shooting.vx()); assertEquals(0, shooting.vy());
      assertEquals(QuadrantAutoController.Status.SHOOTING, shooting.status());
    }
  }
  @Test void startingPoseUnknownAllianceOrWrongAllianceAborts() {
    var pose = config.auto("Blue Left").get(0);
    for (Alliance alliance : new Alliance[]{null, Alliance.Red}) {
      var auto = new QuadrantAutoController(config, "Blue Left");
      assertEquals(QuadrantAutoController.Status.ABORTED, auto.start(pose, alliance, true, 0, settings).status());
    }
    var far = new QuadrantAutoController(config, "Blue Left");
    assertEquals(QuadrantAutoController.Status.ABORTED, far.start(new Pose2d(), Alliance.Blue, true, 0, settings).status());
    var reversed = new QuadrantAutoController(config, "Blue Left");
    assertEquals(QuadrantAutoController.Status.ABORTED, reversed.start(new Pose2d(pose.getTranslation(), pose.getRotation().plus(Rotation2d.fromDegrees(180))), Alliance.Blue, true, 0, settings).status());
  }
  @Test void waypointTimeoutStopsDriveIntakeAndShooting() {
    var auto = new QuadrantAutoController(config, "Blue Left");
    var start = config.auto("Blue Left").get(0);
    auto.start(start, Alliance.Blue, true, 0, settings);
    update(auto, start, Alliance.Blue, .1);
    var out = update(auto, start, Alliance.Blue, 3.2);
    assertEquals(QuadrantAutoController.Status.ABORTED, out.status());
    assertEquals(0, out.vx()); assertEquals(0, out.vy()); assertEquals(0, out.omega());
    assertFalse(out.collect()); assertFalse(out.shoot());
  }
  @Test void lostFeedbackAndModeExitImmediatelyStop() {
    var pose = config.auto("Blue Left").get(0);
    for (boolean enabled : new boolean[]{true, false}) {
      var auto = new QuadrantAutoController(config, "Blue Left");
      auto.start(pose, Alliance.Blue, true, 0, settings);
      var out = auto.update(pose, Alliance.Blue, enabled, false, .1, 5.72, 4.7, settings);
      assertTrue(auto.done()); assertEquals(0, out.vx()); assertFalse(out.shoot());
    }
  }
  @Test void motionPointsTowardGoalAndRespectsMagnitudeCap() {
    var auto = new QuadrantAutoController(config, "Blue Left");
    var start = config.auto("Blue Left").get(0);
    auto.start(start, Alliance.Blue, true, 0, settings);
    update(auto, start, Alliance.Blue, .1);
    var out = update(auto, start, Alliance.Blue, .12);
    assertTrue(out.vx() > 0);
    assertEquals(0, out.vy(), 1e-9);
    assertTrue(Math.hypot(out.vx(),out.vy()) <= 5.72);
    assertEquals(0, out.omega()); // Second leg intentionally does not lock rotation.
  }
  @Test void totalDurationBoundsEvenACompletedRoute() {
    var auto = new QuadrantAutoController(config, "Blue Left");
    var route = config.auto("Blue Left");
    auto.start(route.get(0), Alliance.Blue, true, 0, settings);
    for (int i=0;i<7;i++) update(auto,route.get(i),Alliance.Blue,.1*(i+1));
    update(auto,route.get(6),Alliance.Blue,1);
    var out = update(auto,route.get(6),Alliance.Blue,15);
    assertEquals(QuadrantAutoController.Status.FINISHED,out.status());
    assertEquals(0,out.omega()); assertFalse(out.shoot());
  }
  @Test void timeReversalAndAllianceChangesAbort() {
    var pose = config.auto("Blue Left").get(0);
    var reversedTime = new QuadrantAutoController(config,"Blue Left");
    reversedTime.start(pose,Alliance.Blue,true,1,settings);
    assertEquals(QuadrantAutoController.Status.ABORTED,update(reversedTime,pose,Alliance.Blue,0).status());
    var changedAlliance = new QuadrantAutoController(config,"Blue Left");
    changedAlliance.start(pose,Alliance.Blue,true,0,settings);
    assertEquals(QuadrantAutoController.Status.ABORTED,update(changedAlliance,pose,Alliance.Red,.1).status());
  }
}
