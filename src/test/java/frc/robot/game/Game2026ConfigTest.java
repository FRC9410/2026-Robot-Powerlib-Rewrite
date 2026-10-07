package frc.robot.game;

import static org.junit.jupiter.api.Assertions.*;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class Game2026ConfigTest {
  private static final Path FILE = Path.of("power-tool/generated/powerlib-game-2026.json");
  private static ObjectNode json() throws Exception { return (ObjectNode) new ObjectMapper().readTree(FILE.toFile()); }
  @Test void referenceShotTableInterpolatesAndClamps() throws Exception {
    var config = Game2026Config.read(FILE);
    assertEquals(62, config.shot(1.68).shooterRps());
    var shot = config.shot((1.68 + 1.97) / 2);
    assertEquals(63, shot.shooterRps(), 1e-9);
    assertEquals(0.025, shot.hoodRotations(), 1e-9);
    assertEquals(58.5, shot.feederRps(), 1e-9);
    assertEquals(62, config.shot(0).shooterRps());
    assertEquals(89, config.shot(100).shooterRps());
    assertFalse(config.inShotRange(100));
    assertThrows(IllegalArgumentException.class, () -> config.shot(Double.NaN));
  }
  @Test void headingsConvertRedExactlyOnce() throws Exception {
    var config = Game2026Config.read(FILE);
    assertEquals(90, config.auto("Red Left").get(0).getRotation().getDegrees(), 1e-9);
    assertEquals(-45, config.auto("Red Left").get(6).getRotation().getDegrees(), 1e-9);
    assertEquals(-90, config.auto("Blue Left").get(0).getRotation().getDegrees(), 1e-9);
    assertEquals(0, config.hubHeadingDegrees(new Pose2d(2, 4, new Rotation2d()), Alliance.Blue), 1e-9);
    assertEquals(0, config.hubHeadingDegrees(new Pose2d(14, 4, new Rotation2d()), Alliance.Red), 1e-9);
  }
  @Test void scoringZonesExcludeNeutralOtherAllianceAndUnknown() throws Exception {
    var config = Game2026Config.read(FILE);
    assertTrue(config.inScoringZone(new Pose2d(2, 4, new Rotation2d()), Alliance.Blue));
    assertFalse(config.inScoringZone(new Pose2d(2, 4, new Rotation2d()), Alliance.Red));
    assertFalse(config.inScoringZone(new Pose2d(8, 4, new Rotation2d()), Alliance.Blue));
    assertFalse(config.inScoringZone(new Pose2d(2, 9, new Rotation2d()), Alliance.Blue));
    assertFalse(config.inScoringZone(new Pose2d(4, 4, new Rotation2d()), Alliance.Blue));
    assertFalse(config.inScoringZone(new Pose2d(2, 4, new Rotation2d()), null));
  }
  @Test void malformedAndUnsortedShotTablesAreRejected() throws Exception {
    var data = json();
    ((com.fasterxml.jackson.databind.node.ArrayNode)data.get("shots").get(0)).remove(3);
    assertThrows(IllegalArgumentException.class, () -> Game2026Config.fromJson(data));
    var unsorted = json();
    ((com.fasterxml.jackson.databind.node.ArrayNode)unsorted.get("shots").get(0)).set(0, new ObjectMapper().getNodeFactory().numberNode(10));
    assertThrows(IllegalArgumentException.class, () -> Game2026Config.fromJson(unsorted));
  }
  @Test void missingAndInvalidNumbersAreRejected() throws Exception {
    var data = json();
    ((ObjectNode)data.get("tolerances")).remove("shooterRps");
    assertThrows(IllegalArgumentException.class, () -> Game2026Config.fromJson(data));
    var invalid = json();
    ((ObjectNode)invalid.get("field")).put("width", -1);
    assertThrows(IllegalArgumentException.class, () -> Game2026Config.fromJson(invalid));
  }
  @Test void autonomousNeedsSevenFiniteInFieldPoses() throws Exception {
    var data = json();
    ((com.fasterxml.jackson.databind.node.ArrayNode)data.get("autos").get("Red Left")).remove(6);
    assertThrows(IllegalArgumentException.class, () -> Game2026Config.fromJson(data));
    var outside = json();
    ((com.fasterxml.jackson.databind.node.ArrayNode)outside.get("autos").get("Blue Left").get(0)).set(0, new ObjectMapper().getNodeFactory().numberNode(99));
    assertThrows(IllegalArgumentException.class, () -> Game2026Config.fromJson(outside));
  }
}
