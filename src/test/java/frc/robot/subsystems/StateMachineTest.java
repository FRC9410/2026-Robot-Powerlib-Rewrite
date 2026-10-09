package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.StateMachine.RobotState;
import org.junit.jupiter.api.*;

/** Exercises the real handlers, button bindings, and simulated mechanism IO. */
class StateMachineTest {
  private static RobotContainer container;
  private static StateMachine robot;
  private static XboxControllerSim controller;
  private final Object requestOwner = new Object();

  @BeforeAll static void initialize() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    container = new RobotContainer();
    robot = container.getStateMachine();
    controller = new XboxControllerSim(Constants.OI.DRIVER_CONTROLLER_PORT);
  }

  @BeforeEach void reset() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    controller.setLeftTriggerAxis(0);
    controller.setRightTriggerAxis(0);
    controller.setBButton(false);
    controller.setBackButton(false);
    controller.setStartButton(false);
    DriverStationSim.notifyNewData();
    // Poll released controls before resetting so each test starts with fresh trigger edges.
    CommandScheduler.getInstance().run();
    container.resetForMode();
    robot.drivetrain.resetPose(new Pose2d(2.12, 4, Rotation2d.fromDegrees(180)));
    robot.shooter.inputs.velocityRotationsPerSecond = 0;
    robot.shooterHood.inputs.positionRotations = 0.09;
  }

  @AfterAll static void close() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    robot.vision.close();
    robot.drivetrain.close();
    DriverStationSim.resetData();
  }

  private void cycleBindings() {
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().run();
  }

  @Test void intakingAndEjectingHaveSeparateOutputsAndReturnToStandby() {
    robot.requestState(RobotState.INTAKING, requestOwner);
    robot.execute();
    assertEquals(RobotState.INTAKING, robot.getCurrentState());
    assertEquals(145, robot.intakeRoller.getVelocitySetpoint());
    assertEquals(Constants.IntakeWrist.INTAKE_MAX, robot.intakeWrist.getSetpointRotations());
    assertEquals(0, robot.feeder.getVelocitySetpoint());
    robot.requestState(RobotState.EJECTING, requestOwner);
    robot.execute();
    assertEquals(RobotState.EJECTING, robot.getCurrentState());
    assertEquals(-145, robot.intakeRoller.getVelocitySetpoint());
    assertEquals(Constants.IntakeWrist.INTAKE_MAX, robot.intakeWrist.getSetpointRotations());
    assertEquals(0, robot.shooter.getVelocitySetpoint());
    robot.requestState(RobotState.READY, requestOwner);
    robot.execute();
    assertEquals(RobotState.READY, robot.getCurrentState());
    assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
    assertEquals(Constants.IntakeWrist.INTAKE_IDLE, robot.intakeWrist.getSetpointRotations());
  }

  @Test void shootingSpinsUpThenFeedsAndStopsImmediatelyIfReadinessIsLost() {
    robot.requestState(RobotState.SHOOTING, requestOwner);
    robot.execute();
    assertEquals(RobotState.SPINNING_UP, robot.getCurrentState());
    assertEquals(29.5, robot.shooter.getVelocitySetpoint(), 0.01);
    assertEquals(0, robot.feeder.getVelocitySetpoint());
    robot.shooter.inputs.velocityRotationsPerSecond = 29.5;
    robot.shooterHood.inputs.positionRotations = 0.05;
    robot.execute();
    assertEquals(RobotState.SHOOTING, robot.getCurrentState());
    assertEquals(-59, robot.feeder.getVelocitySetpoint());
    assertEquals(80, robot.spindexer.getVelocitySetpoint());
    robot.shooter.inputs.velocityRotationsPerSecond = 0;
    robot.execute();
    assertEquals(RobotState.SPINNING_UP, robot.getCurrentState());
    assertEquals(RobotState.SHOOTING, robot.getWantedState());
    assertEquals(0, robot.feeder.getVelocitySetpoint());
    assertEquals(0, robot.spindexer.getVelocitySetpoint());
  }

  @Test void shootingPulseRestartsAfterEjectingAndModeReset() throws Exception {
    var stateField = StateMachine.class.getDeclaredField("shootingState");
    stateField.setAccessible(true);
    var shooting = (frc.robot.subsystems.states.ShootingState) stateField.get(robot);
    var started = shooting.getClass().getDeclaredField("shootingStarted");
    started.setAccessible(true);
    robot.requestState(RobotState.SHOOTING, requestOwner);
    robot.execute();
    started.setDouble(shooting, edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - 1.1);
    robot.execute();
    assertEquals(Constants.IntakeWrist.INTAKE_FEED, robot.intakeWrist.getSetpointRotations());

    Object ejectOwner = new Object();
    robot.requestState(RobotState.EJECTING, ejectOwner);
    robot.execute();
    robot.clearRequest(ejectOwner);
    robot.execute();
    assertEquals(RobotState.SPINNING_UP, robot.getCurrentState());
    assertEquals(Constants.IntakeWrist.INTAKE_IDLE, robot.intakeWrist.getSetpointRotations());

    started.setDouble(shooting, edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - 1.1);
    container.resetForMode();
    assertTrue(Double.isNaN(started.getDouble(shooting)));
    robot.requestState(RobotState.SHOOTING, requestOwner);
    robot.execute();
    assertEquals(Constants.IntakeWrist.INTAKE_IDLE, robot.intakeWrist.getSetpointRotations());
  }

  @Test void dashboardDistanceUpdatesWithoutAShootingRequest() throws Exception {
    var dashboardField = RobotContainer.class.getDeclaredField("powerDashboard");
    dashboardField.setAccessible(true);
    var dashboard = (PowerDashboard) dashboardField.get(container);
    robot.requestState(RobotState.INTAKING, requestOwner);
    robot.execute();
    dashboard.periodic();
    assertEquals(2.5, (double) frc.powerlib.PowerRobotContainer.getData("Shooting/Distance"), 0.001);
    robot.drivetrain.resetPose(new Pose2d(1.62, 4, new Rotation2d()));
    dashboard.periodic();
    assertEquals(3, (double) frc.powerlib.PowerRobotContainer.getData("Shooting/Distance"), 0.001);
    assertFalse(robot.hasRequest(RobotState.SHOOTING));
  }

  @Test void explicitSpinUpNeverFeedsEvenWhenReady() {
    robot.shooter.inputs.velocityRotationsPerSecond = 29.5;
    robot.shooterHood.inputs.positionRotations = 0.05;
    robot.requestState(RobotState.SPINNING_UP, requestOwner);
    robot.execute();
    robot.execute();
    assertEquals(RobotState.SPINNING_UP, robot.getCurrentState());
    assertEquals(0, robot.feeder.getVelocitySetpoint());
  }

  @Test void disabledAndTestModesLeaveCharacterizationOutputsAlone() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    robot.requestState(RobotState.SHOOTING, requestOwner);
    robot.execute();
    assertEquals(RobotState.IDLE, robot.getCurrentState());
    DriverStationSim.setEnabled(true);
    DriverStationSim.setTest(true);
    DriverStationSim.notifyNewData();
    robot.shooter.setVelocity(12);
    robot.feeder.setVelocity(8);
    robot.execute();
    assertEquals(12, robot.shooter.getVelocitySetpoint());
    assertEquals(8, robot.feeder.getVelocitySetpoint());
  }

  @Test void buttonRequestsFollowHandlerPriorityAndResumeHeldRequests() {
    controller.setLeftTriggerAxis(1);
    cycleBindings();
    assertEquals(RobotState.INTAKING, robot.getWantedState());
    controller.setBButton(true);
    cycleBindings();
    assertEquals(RobotState.EJECTING, robot.getWantedState());
    controller.setRightTriggerAxis(1);
    cycleBindings();
    assertEquals(RobotState.EJECTING, robot.getWantedState());
    controller.setBButton(false);
    controller.setBackButton(false);
    controller.setStartButton(false);
    cycleBindings();
    assertEquals(RobotState.SHOOTING, robot.getWantedState());
    controller.setRightTriggerAxis(0);
    cycleBindings();
    assertEquals(RobotState.INTAKING, robot.getWantedState());
    controller.setLeftTriggerAxis(0);
    cycleBindings();
    assertEquals(RobotState.IDLE, robot.getWantedState());
  }

  @Test void releasingIntakeCannotCancelHeldShooting() {
    controller.setLeftTriggerAxis(1);
    cycleBindings();
    controller.setRightTriggerAxis(1);
    cycleBindings();
    assertEquals(RobotState.SHOOTING, robot.getWantedState());
    assertTrue(robot.hasRequest(RobotState.INTAKING));
    controller.setLeftTriggerAxis(0);
    cycleBindings();
    assertEquals(RobotState.SHOOTING, robot.getWantedState());
    assertFalse(robot.hasRequest(RobotState.INTAKING));
    assertEquals(RobotState.SPINNING_UP, robot.getCurrentState());
    controller.setRightTriggerAxis(0);
    cycleBindings();
    assertEquals(RobotState.IDLE, robot.getWantedState());
    assertEquals(RobotState.IDLE, robot.getCurrentState());
  }

  @Test void simultaneousPressesAndShootingReleasePreserveIntakeOwner() {
    controller.setLeftTriggerAxis(1);
    controller.setRightTriggerAxis(1);
    cycleBindings();
    assertTrue(robot.hasRequest(RobotState.INTAKING));
    assertTrue(robot.hasRequest(RobotState.SHOOTING));
    assertEquals(RobotState.SHOOTING, robot.getWantedState());
    controller.setRightTriggerAxis(0);
    cycleBindings();
    assertEquals(RobotState.INTAKING, robot.getWantedState());
    assertEquals(145, robot.intakeRoller.getVelocitySetpoint());
  }

  @Test void twoOwnersRequestingTheSameStateReleaseIndependently() {
    Object secondOwner = new Object();
    robot.requestState(RobotState.INTAKING, requestOwner);
    robot.requestState(RobotState.INTAKING, secondOwner);
    robot.execute();
    robot.requestState(RobotState.IDLE, requestOwner);
    robot.execute();
    assertEquals(RobotState.INTAKING, robot.getCurrentState());
    robot.clearRequest(secondOwner);
    robot.execute();
    assertEquals(RobotState.IDLE, robot.getCurrentState());
  }

  @Test void reorderingHandlersChangesPriorityWithoutChangingBindingsOrGuards() throws Exception {
    var field = StateMachine.class.getDeclaredField("states");
    field.setAccessible(true);
    Object original = field.get(robot);
    try {
      robot.requestState(RobotState.EJECTING, requestOwner);
      robot.requestState(RobotState.SHOOTING, new Object());
      robot.shooter.inputs.velocityRotationsPerSecond = 29.5;
      robot.shooterHood.inputs.positionRotations = 0.05;
      robot.execute();
      assertEquals(RobotState.EJECTING, robot.getCurrentState());
      var shooting = new frc.robot.subsystems.states.ShootingState();
      field.set(robot, java.util.List.of(
          shooting,
          new frc.robot.subsystems.states.SpinningUpState(shooting),
          new frc.robot.subsystems.states.EjectingState(),
          new frc.robot.subsystems.states.IntakingState(),
          new frc.robot.subsystems.states.IdleState()));
      robot.execute();
      assertEquals(RobotState.SHOOTING, robot.getCurrentState());
      assertEquals(-59, robot.feeder.getVelocitySetpoint());
      assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
    } finally { field.set(robot, original); }
  }

  @Test void visionSeedUsesFreshAcceptedPoseOnlyWhileDisabled() throws Exception {
    var field = Vision.class.getDeclaredField("lastMeasurement");
    field.setAccessible(true);
    Object previous = field.get(robot.vision);
    try {
      var pose = new Pose2d(1.5, 2.5, Rotation2d.fromDegrees(24));
      field.set(robot.vision, new frc.powerlib.vision.LimelightVision.Measurement(
          "test-camera", pose, edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), 1, 2, 0.1));
      assertFalse(robot.vision.seedFromFreshVision());
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      assertTrue(robot.vision.seedFromFreshVision());
      assertEquals(pose.getX(), robot.drivetrain.getState().Pose.getX(), 0.001);
      assertEquals(pose.getY(), robot.drivetrain.getState().Pose.getY(), 0.001);
      assertEquals(24, robot.drivetrain.getState().Pose.getRotation().getDegrees(), 0.001);
    } finally { field.set(robot.vision, previous); }
  }

  @Test void visionSeedRejectsMissingStaleFutureAndInvalidMeasurements() throws Exception {
    var field = Vision.class.getDeclaredField("lastMeasurement");
    field.setAccessible(true);
    Object previous = field.get(robot.vision);
    try {
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      field.set(robot.vision, null);
      assertFalse(robot.vision.seedFromFreshVision());
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      for (double timestamp : new double[] {
          now - Constants.Vision.CONFIG.maxMeasurementAgeSeconds() - 0.1,
          now + 1, Double.NaN, Double.POSITIVE_INFINITY}) {
        field.set(robot.vision, new frc.powerlib.vision.LimelightVision.Measurement(
            "test-camera", new Pose2d(), timestamp, 1, 2, 0.1));
        assertFalse(robot.vision.seedFromFreshVision());
      }
      for (Pose2d invalid : new Pose2d[] {null, new Pose2d(Double.NaN, 2, new Rotation2d())}) {
        field.set(robot.vision, new frc.powerlib.vision.LimelightVision.Measurement(
            "test-camera", invalid, edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), 1, 2, 0.1));
        assertFalse(robot.vision.seedFromFreshVision());
      }
      assertEquals(2.12, robot.drivetrain.getState().Pose.getX(), 0.001);
      assertEquals(4, robot.drivetrain.getState().Pose.getY(), 0.001);
    } finally { field.set(robot.vision, previous); }
  }

  @Test void shootingWristPulseBelongsToShootingAndYieldsToCollection() throws Exception {
    var stateField = StateMachine.class.getDeclaredField("shootingState");
    stateField.setAccessible(true);
    var shooting = (frc.robot.subsystems.states.ShootingState) stateField.get(robot);
    var started = shooting.getClass().getDeclaredField("shootingStarted");
    started.setAccessible(true);
    robot.requestState(RobotState.SHOOTING, requestOwner);
    robot.execute();
    for (double seconds : new double[] {Double.NaN, 0.2, 1.1, 1.6, 2.1}) {
      started.setDouble(shooting, edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - seconds);
      robot.execute();
      double expected = seconds == 1.1 || seconds == 2.1
          ? Constants.IntakeWrist.INTAKE_FEED : Constants.IntakeWrist.INTAKE_IDLE;
      assertEquals(expected, robot.intakeWrist.getSetpointRotations());
      assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
    }
    robot.requestState(RobotState.INTAKING, new Object());
    started.setDouble(shooting, edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - 1.1);
    robot.execute();
    assertEquals(RobotState.SHOOTING, robot.getWantedState());
    assertEquals(Constants.IntakeWrist.INTAKE_MAX, robot.intakeWrist.getSetpointRotations());
    assertEquals(145, robot.intakeRoller.getVelocitySetpoint());
  }
  @Test void sharedHealthChecksPreserveConfiguredSimulationBehavior() {
    assertTrue(frc.powerlib.health.HealthChecks.driveHealthy(robot.drivetrain, 0.5));
    assertTrue(frc.powerlib.health.HealthChecks.mechanismHealthy(robot.shooter, robot.shooter.getVelocityMotor(), 0.5));
  }

  @Test void headingButtonHelperRunsWhileDisabledAndPreservesEnabledHeading() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    robot.drivetrain.resetPose(new Pose2d(2.12, 4, Rotation2d.fromDegrees(25)));
    controller.setStartButton(true);
    cycleBindings();
    assertEquals(0, robot.drivetrain.getState().Pose.getRotation().getDegrees(), 0.001);
    assertEquals(2.12, robot.drivetrain.getState().Pose.getX(), 0.001);
    assertEquals(4, robot.drivetrain.getState().Pose.getY(), 0.001);
    controller.setStartButton(false);
    cycleBindings();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.drivetrain.resetPose(new Pose2d(2.12, 4, Rotation2d.fromDegrees(25)));
    controller.setStartButton(true);
    cycleBindings();
    assertEquals(25, robot.drivetrain.getState().Pose.getRotation().getDegrees(), 0.001);
  }

  @Test void autoCancellationClearsItsRequestsAndStopsAllOutputs() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var route = frc.robot.autos.RobotAutos.ROUTES.get(3);
    robot.drivetrain.resetPose(route.fieldPoints().get(0));
    var auto = frc.robot.autos.RobotAutos.build(route, robot);
    auto.initialize();
    auto.execute();
    robot.drivetrain.resetPose(route.fieldPoints().get(1));
    auto.execute();
    auto.execute();
    assertTrue(robot.hasRequest(RobotState.INTAKING));
    auto.end(true);
    assertFalse(robot.hasRequest(RobotState.INTAKING));
    assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
    assertEquals(0, robot.feeder.getVelocitySetpoint());
    assertEquals(0, robot.shooter.getVelocitySetpoint());
  }

  @Test void autoRejectsWrongStartingPoseWithoutResettingOdometry() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var before = robot.drivetrain.getState().Pose;
    var auto = frc.robot.autos.RobotAutos.build(frc.robot.autos.RobotAutos.ROUTES.get(3), robot);
    auto.initialize();
    auto.execute(); // ParallelRaceGroup records completion during execute, as the scheduler does.
    assertTrue(auto.isFinished());
    auto.end(false);
    assertEquals(before.getX(), robot.drivetrain.getState().Pose.getX(), 0.001);
    assertFalse(robot.hasRequest(RobotState.INTAKING));
    assertFalse(robot.hasRequest(RobotState.SHOOTING));
  }

  @Test void timedOutAutoLegAbortsWithoutStartingIntake() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var route = frc.robot.autos.RobotAutos.ROUTES.get(3);
    robot.drivetrain.resetPose(route.fieldPoints().get(0));
    var auto = frc.robot.autos.RobotAutos.build(route, robot);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      auto.execute(); // Reach leg 1; leg 2 is now active.
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(Constants.RobotContainer.AUTO_LEG_TIMEOUT_SECONDS + 0.1);
      auto.execute();
      assertTrue(auto.isFinished());
      auto.end(false);
      assertFalse(robot.hasRequest(RobotState.INTAKING));
      assertFalse(robot.hasRequest(RobotState.SHOOTING));
      assertEquals(0, robot.shooter.getVelocitySetpoint());
    } finally { edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming(); }
  }
}
