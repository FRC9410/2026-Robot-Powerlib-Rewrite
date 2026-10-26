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
  private static final double TEST_X = Constants.Shooting.BLUE_HUB.getX() - 2.5;
  private static final double TEST_Y = Constants.Shooting.BLUE_HUB.getY();

  @BeforeAll static void initialize() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    container = new RobotContainer();
    robot = container.getStateMachine();
    controller = new XboxControllerSim(Constants.OI.DRIVER_CONTROLLER_PORT);
  }

  @BeforeEach void reset() throws InterruptedException {
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
    controller.setLeftX(0);
    controller.setLeftY(0);
    controller.setRightX(0);
    DriverStationSim.notifyNewData();
    // Poll released controls before resetting so each test starts with fresh trigger edges.
    CommandScheduler.getInstance().run();
    container.resetForMode();
    settleDrivetrain();
    robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
    robot.shooter.inputs.velocityRotationsPerSecond = 0;
    robot.shooterHood.inputs.positionRotations = 0.09;
  }

  /**
   * Drive tests command real motion in the Phoenix drivetrain sim. Brake and wait for it to stop,
   * so later tests see a stationary robot instead of a moving shot or a drifting pose.
   */
  private static void settleDrivetrain() {
    robot.drivetrain.applyRequest(new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric());
    long deadline = System.nanoTime() + 1_000_000_000L;
    while (System.nanoTime() < deadline) {
      var speeds = robot.drivetrain.getState().Speeds;
      if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0
          && speeds.omegaRadiansPerSecond == 0) return;
      try {
        Thread.sleep(5);
      } catch (InterruptedException exception) {
        Thread.currentThread().interrupt();
        return;
      }
    }
    fail("Simulated drivetrain did not stop within 1 s");
  }

  @AfterAll static void close() throws Exception {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    robot.vision.close();
    var notifier = Swerve.class.getDeclaredField("m_simNotifier");
    notifier.setAccessible(true);
    ((edu.wpi.first.wpilibj.Notifier) notifier.get(robot.drivetrain)).close();
    robot.drivetrain.close();
    DriverStationSim.resetData();
  }

  private void cycleBindings() {
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().run();
    CommandScheduler.getInstance().run();
  }

  /** Field velocity commanded for the current stick input while intaking, for comparison. */
  private double[] intakeVelocity(edu.wpi.first.wpilibj2.command.button.CommandXboxController pad) throws Exception {
    var intake = new frc.robot.commands.SwerveDriveCommand(robot.drivetrain, pad, () -> RobotState.INTAKING);
    intake.execute();
    var field = intake.getClass().getDeclaredField("request");
    field.setAccessible(true);
    var request = (com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric) field.get(intake);
    double[] velocity = { request.VelocityX, request.VelocityY };
    intake.end(true);
    return velocity;
  }

  /** Field velocity expected while shooting on the move: the intake velocity rescaled to the shooting coefficient. */
  private double[] shootingVelocity(edu.wpi.first.wpilibj2.command.button.CommandXboxController pad) throws Exception {
    double[] intake = intakeVelocity(pad);
    double scale = Constants.RobotContainer.DRIVE_WHILE_SHOOTING_COEFFICIENT
        / Constants.RobotContainer.DRIVE_WHILE_INTAKING_COEFFICIENT;
    return new double[] { intake[0] * scale, intake[1] * scale };
  }

  @Test void driveUsesMovingShotHeadingAndFeedforwardAtShootingSpeed() throws Exception {
    var pad = new edu.wpi.first.wpilibj2.command.button.CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);
    var shot = new frc.robot.utils.MovingShotCalculator().calculate(robot.drivetrain.getState().Pose,
        new edu.wpi.first.math.kinematics.ChassisSpeeds(0, -0.5, 0),
        edu.wpi.first.wpilibj.DriverStation.Alliance.Blue, 29.5, 0.05, true, true, 0, 0);
    assertTrue(shot.valid(), shot.status());
    var command = new frc.robot.commands.SwerveDriveCommand(robot.drivetrain, pad,
        () -> RobotState.SHOOTING, () -> shot);
    controller.setLeftX(-1);
    controller.setLeftY(-1);
    controller.notifyNewData();
    DriverStationSim.notifyNewData();
    double previousCoefficient = Constants.RobotContainer.DRIVE_WHILE_SHOOTING_COEFFICIENT;
    // Differs from the intake coefficient so the test shows the two speeds are tuned separately.
    Constants.RobotContainer.DRIVE_WHILE_SHOOTING_COEFFICIENT = 0.25;
    try {
      double[] expected = shootingVelocity(pad);
      double poseHeading = robot.drivetrain.getState().Pose.getRotation().getDegrees();
      double measuredOmega = robot.drivetrain.getState().Speeds.omegaRadiansPerSecond;
      command.execute();
      var requestField = command.getClass().getDeclaredField("aimingRequest");
      requestField.setAccessible(true);
      var request = (com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric) requestField.get(command);
      assertEquals(shot.headingDegrees(), robot.drivetrain.getHeadingSetpointDegrees(), 1e-9);
      assertEquals(expected[0], request.VelocityX, 1e-3);
      assertEquals(expected[1], request.VelocityY, 1e-3);
      double error = edu.wpi.first.math.MathUtil.inputModulus(shot.headingDegrees() - poseHeading, -180, 180);
      double kp = frc.powerlib.PowerRobotContainer.getSubsystemVariable("Swerve", "Heading/kP", Constants.Swerve.HEADING_KP);
      double kd = frc.powerlib.PowerRobotContainer.getSubsystemVariable("Swerve", "Heading/kD", Constants.Swerve.HEADING_KD);
      double feedforward = shot.headingFeedforwardRadiansPerSecond();
      assertEquals(Math.toRadians(error) * kp + feedforward + kd * (feedforward - measuredOmega),
          request.RotationalRate, 0.001);
      assertEquals(0.02, request.Deadband);
      assertEquals(0, request.RotationalDeadband);
      assertEquals(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity, request.DriveRequestType);
    } finally {
      Constants.RobotContainer.DRIVE_WHILE_SHOOTING_COEFFICIENT = previousCoefficient;
      command.end(true);
    }
  }

  @Test void killSwitchHoldsTranslationWhileAimingTheStationaryShot() throws Exception {
    var pad = new edu.wpi.first.wpilibj2.command.button.CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);
    var command = new frc.robot.commands.SwerveDriveCommand(robot.drivetrain, pad,
        () -> RobotState.SHOOTING, robot::getShotSolution);
    controller.setLeftX(-1);
    controller.setLeftY(-1);
    controller.notifyNewData();
    DriverStationSim.notifyNewData();
    // Ten degrees off target so the stopped robot keeps aiming instead of parking its wheels.
    robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(170)));
    boolean previous = frc.robot.utils.ShootingUtils.sotmEnabled();
    try {
      var requestField = command.getClass().getDeclaredField("aimingRequest");
      requestField.setAccessible(true);
      var request = (com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric) requestField.get(command);
      double[] expected = shootingVelocity(pad);
      frc.robot.utils.ShootingUtils.setSotmEnabled(true);
      command.execute();
      assertEquals(expected[0], request.VelocityX, 1e-3);
      assertEquals(expected[1], request.VelocityY, 1e-3);
      frc.robot.utils.ShootingUtils.setSotmEnabled(false);
      command.execute();
      assertEquals(0, request.VelocityX);
      assertEquals(0, request.VelocityY);
    } finally {
      frc.robot.utils.ShootingUtils.setSotmEnabled(previous);
      command.end(true);
    }
  }

  @Test void aimedStoppedRobotXLocksOnlyForTheStationaryShot() {
    var pad = new edu.wpi.first.wpilibj2.command.button.CommandXboxController(Constants.OI.DRIVER_CONTROLLER_PORT);
    var command = new frc.robot.commands.SwerveDriveCommand(robot.drivetrain, pad,
        () -> RobotState.SHOOTING, robot::getShotSolution);
    boolean previous = frc.robot.utils.ShootingUtils.sotmEnabled();
    try {
      // The reset pose faces straight away from the hub, so the rear shooter is aimed.
      frc.robot.utils.ShootingUtils.setSotmEnabled(true);
      command.execute();
      assertEquals(true, frc.powerlib.PowerRobotContainer.getData("Drive/AimWheelsParked"));
      assertEquals(false, frc.powerlib.PowerRobotContainer.getData("Drive/AimWheelsLocked"));
      frc.robot.utils.ShootingUtils.setSotmEnabled(false);
      command.execute();
      assertEquals(true, frc.powerlib.PowerRobotContainer.getData("Drive/AimWheelsParked"));
      assertEquals(true, frc.powerlib.PowerRobotContainer.getData("Drive/AimWheelsLocked"));
    } finally {
      frc.robot.utils.ShootingUtils.setSotmEnabled(previous);
      command.end(true);
    }
    assertEquals(false, frc.powerlib.PowerRobotContainer.getData("Drive/AimWheelsLocked"));
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
    assertEquals(30.5, robot.shooter.getVelocitySetpoint(), 0.01);
    assertEquals(0, robot.feeder.getVelocitySetpoint());
    robot.shooter.inputs.velocityRotationsPerSecond = 30.5;
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
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
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

    } finally { edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming(); }
  }

  @Test void dashboardDistanceUpdatesWithoutAShootingRequest() throws Exception {
    var dashboardField = RobotContainer.class.getDeclaredField("powerDashboard");
    dashboardField.setAccessible(true);
    var dashboard = (PowerDashboard) dashboardField.get(container);
    robot.requestState(RobotState.INTAKING, requestOwner);
    robot.execute();
    dashboard.periodic();
    assertEquals(2.5, (double) frc.powerlib.PowerRobotContainer.getData("Shooting/Distance"), 0.001);
    robot.drivetrain.resetPose(new Pose2d(Constants.Shooting.BLUE_HUB.getX() - 3,
        Constants.Shooting.BLUE_HUB.getY(), new Rotation2d()));
    dashboard.periodic();
    assertEquals(3, (double) frc.powerlib.PowerRobotContainer.getData("Shooting/Distance"), 0.001);
    assertFalse(robot.hasRequest(RobotState.SHOOTING));
  }

  @Test void explicitSpinUpNeverFeedsEvenWhenReady() {
    robot.shooter.inputs.velocityRotationsPerSecond = 30.5;
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
      robot.shooter.inputs.velocityRotationsPerSecond = 30.5;
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
      // Native simulation may move slightly after reset; verify the selected handler applies its prepared shot.
      assertEquals(shooting.getShotSolution().feederRps(), robot.feeder.getVelocitySetpoint());
      assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
    } finally { field.set(robot, original); }
  }

  @Test void visionSeedUsesFreshAcceptedPoseOnlyWhileDisabled() throws Exception {
    var field = Vision.class.getDeclaredField("lastMeasurement");
    field.setAccessible(true);
    Object previous = field.get(robot.vision);
    var headingField = Vision.class.getDeclaredField("lastHeadingMeasurement");
    headingField.setAccessible(true);
    Object previousHeading = headingField.get(robot.vision);
    prepareVisionFilter();
    try {
      headingField.set(robot.vision, null);
      var pose = new Pose2d(1.5, 2.5, Rotation2d.fromDegrees(24));
      field.set(robot.vision, new frc.powerlib.vision.LimelightVision.Measurement(
          "test-camera", pose, edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), 1, 2, 0.1));
      assertFalse(robot.vision.seedFromFreshVision());
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      assertTrue(robot.vision.seedFromFreshVision());
      assertEquals(pose.getX(), robot.drivetrain.getState().Pose.getX(), 0.001);
      assertEquals(pose.getY(), robot.drivetrain.getState().Pose.getY(), 0.001);
      // MT2 alone must not restore its supplied/drifted yaw during a position seed.
      assertEquals(180, robot.drivetrain.getState().Pose.getRotation().getDegrees(), 0.001);
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      headingField.set(robot.vision, new frc.powerlib.vision.LimelightVision.HeadingMeasurement(
          "test-camera", new Pose2d(9, 9, Rotation2d.fromDegrees(12)), now, 0.1, 1, 1.5, .02, 0));
      assertTrue(robot.vision.seedFromFreshVision());
      assertEquals(12, robot.drivetrain.getState().Pose.getRotation().getDegrees(), 0.001);
      assertEquals(pose.getTranslation(), robot.drivetrain.getState().Pose.getTranslation());
      headingField.set(robot.vision, new frc.powerlib.vision.LimelightVision.HeadingMeasurement(
          "test-camera", new Pose2d(9, 9, Rotation2d.fromDegrees(50)), now - 1, 0.1));
      assertFalse(robot.vision.hasFreshHeading());
      assertTrue(robot.vision.seedFromFreshVision());
      assertEquals(12, robot.drivetrain.getState().Pose.getRotation().getDegrees(), 0.001);
    } finally { field.set(robot.vision, previous); headingField.set(robot.vision, previousHeading); }
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
      assertEquals(TEST_X, robot.drivetrain.getState().Pose.getX(), 0.001);
      assertEquals(TEST_Y, robot.drivetrain.getState().Pose.getY(), 0.001);
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
    robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(25)));
    controller.setStartButton(true);
    cycleBindings();
    assertEquals(0, robot.drivetrain.getState().Pose.getRotation().getDegrees(), 0.001);
    assertEquals(TEST_X, robot.drivetrain.getState().Pose.getX(), 0.001);
    assertEquals(TEST_Y, robot.drivetrain.getState().Pose.getY(), 0.001);
    controller.setStartButton(false);
    cycleBindings();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(25)));
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
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      auto.execute();
      assertFalse(robot.hasRequest(RobotState.INTAKING));
      for (int i = 0; i < 7; i++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        auto.execute();
      }
      robot.drivetrain.resetPose(route.fieldPoints().get(1));
      for (int i = 0; i < 12 && !robot.hasRequest(RobotState.INTAKING); i++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        auto.execute();
      }
      assertTrue(robot.hasRequest(RobotState.INTAKING));
      auto.end(true);
      assertFalse(robot.hasRequest(RobotState.INTAKING));
      assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
      assertEquals(0, robot.feeder.getVelocitySetpoint());
      assertEquals(0, robot.shooter.getVelocitySetpoint());
    } finally {
      if (!auto.isFinished()) auto.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void autoUsesConfirmedLivePoseWithoutRequiringExactStartingWaypoint() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var before = robot.drivetrain.getState().Pose;
    var auto = frc.robot.autos.RobotAutos.build(frc.robot.autos.RobotAutos.ROUTES.get(3), robot);
    auto.initialize();
    auto.execute(); // ParallelRaceGroup records completion during execute, as the scheduler does.
    assertFalse(auto.isFinished());
    auto.end(true);
    assertEquals(before.getX(), robot.drivetrain.getState().Pose.getX(), 0.001);
    assertFalse(robot.hasRequest(RobotState.INTAKING));
    assertFalse(robot.hasRequest(RobotState.SHOOTING));
  }

  private static com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric pointRequest(
      frc.robot.commands.DrivePathCommand command) throws ReflectiveOperationException {
    var field = frc.robot.commands.DrivePathCommand.class.getDeclaredField("driveRequest");
    field.setAccessible(true);
    return (com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric) field.get(command);
  }

  @Test void pointCommandConstrainsCombinedMotionAndStopsImmediately() throws Exception {
    var limits = new frc.powerlib.configs.MotionConstraints(0.6, 0.5, 0.8, 1);
    var reasons = new java.util.ArrayList<String>();
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    var command = new frc.robot.commands.DriveToPointCommand(robot.drivetrain,
        new Pose2d(TEST_X + 10, TEST_Y + 10, Rotation2d.fromDegrees(90)),
        Constants.Auto.completionCriteria(3, Constants.Auto.LEG_HEADING_TOLERANCE_DEGREES),
        limits, true, 3, reasons::add);
    try {
      var state = robot.drivetrain.getState();
      var previous = edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
          state.Speeds, state.Pose.getRotation());
      var initial = new frc.powerlib.auto.ChassisMotionLimiter(limits);
      initial.reset(previous);
      previous = initial.calculate(new edu.wpi.first.math.kinematics.ChassisSpeeds(), 0);
      command.initialize();
      for (double dt : new double[] {0.02, 0.01, 0.03, 0.06, 0.02}) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(dt);
        command.execute();
        var request = pointRequest(command);
        assertTrue(Math.hypot(request.VelocityX, request.VelocityY) <= 0.6 + 1e-6);
        assertTrue(Math.hypot(request.VelocityX - previous.vxMetersPerSecond,
            request.VelocityY - previous.vyMetersPerSecond) <= 0.5 * dt + 1e-6);
        assertTrue(Math.abs(request.RotationalRate) <= 0.8 + 1e-6);
        assertTrue(Math.abs(request.RotationalRate - previous.omegaRadiansPerSecond) <= dt + 1e-6);
        assertEquals(0, request.Deadband);
        assertEquals(0, request.RotationalDeadband);
        previous = new edu.wpi.first.math.kinematics.ChassisSpeeds(
            request.VelocityX, request.VelocityY, request.RotationalRate);
      }
      assertTrue(Math.hypot(previous.vxMetersPerSecond, previous.vyMetersPerSecond) > 0);
      assertEquals(90, robot.drivetrain.getHeadingSetpointDegrees(), 1e-9);
      assertTrue(reasons.isEmpty());
      command.end(true);
      assertEquals(0, pointRequest(command).VelocityX);
      assertEquals(0, pointRequest(command).VelocityY);
      assertEquals(0, pointRequest(command).RotationalRate);
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(3.1);
      assertFalse(command.isFinished()); // Reading completion does not process a timeout.
      command.execute();
      assertTrue(command.isFinished());
      assertEquals(java.util.List.of("Autonomous waypoint timed out"), reasons);
      assertEquals(0, pointRequest(command).VelocityX);
      assertEquals(0, pointRequest(command).RotationalRate);
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void pointCommandBrakesBeforeArrivalAndResetsItsHistory() throws Exception {
    var destination = new Pose2d(TEST_X + 2, TEST_Y, new Rotation2d());
    var limits = new frc.powerlib.configs.MotionConstraints(1, 1, 1, 1);
    double tolerance = edu.wpi.first.math.util.Units.inchesToMeters(3);
    var command = new frc.robot.commands.DriveToPointCommand(robot.drivetrain, destination, 3,
        limits, false, reason -> fail(reason));
    var desiredField = frc.robot.commands.DrivePathCommand.class.getDeclaredField("desiredSpeeds");
    desiredField.setAccessible(true);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try (var baseline = new edu.wpi.first.math.controller.PIDController(
        Constants.RobotContainer.AUTO_TRANSLATION_KP, 0, Constants.RobotContainer.AUTO_TRANSLATION_KD)) {
      for (int run = 0; run < 2; run++) {
        robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, new Rotation2d()));
        command.initialize();
        baseline.reset();
        for (double offset : new double[] {0, 1.8, 1.95}) {
          robot.drivetrain.resetPose(new Pose2d(TEST_X + offset, TEST_Y, new Rotation2d()));
          edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
          var pose = robot.drivetrain.getState().Pose;
          double distance = destination.getTranslation().minus(pose.getTranslation()).getNorm();
          double expected = distance <= tolerance ? 0 : Math.min(limits.maxTranslationSpeedToStop(distance - tolerance),
              Math.abs(baseline.calculate(distance, 0))
                  + Constants.Swerve.DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * robot.drivetrain.MAX_SPEED);
          command.execute();
          var desired = (edu.wpi.first.math.kinematics.ChassisSpeeds) desiredField.get(command);
          assertEquals(expected, Math.hypot(desired.vxMetersPerSecond, desired.vyMetersPerSecond), 1e-6);
          assertFalse(command.isFinished());
        }
        for (int query = 0; query < 10; query++) assertFalse(command.isFinished());
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(Constants.Auto.ARRIVAL_SETTLE_TIME_SECONDS + 0.02);
        command.execute();
        assertTrue(command.isFinished());
        command.end(false);
        assertEquals(0, pointRequest(command).VelocityX);
        assertEquals(0, pointRequest(command).VelocityY);
      }
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void headingRequestBrakesOutsideToleranceAndStopsInsideIt() throws Exception {
    var limits = new frc.powerlib.configs.MotionConstraints(1, 1, 2, 1);
    var goal = new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180));
    var command = new frc.robot.commands.DriveToPointCommand(robot.drivetrain, goal, 3,
        limits, true, reason -> fail(reason));
    var desiredField = frc.robot.commands.DrivePathCommand.class.getDeclaredField("desiredSpeeds");
    desiredField.setAccessible(true);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(169)));
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      var desired = (edu.wpi.first.math.kinematics.ChassisSpeeds) desiredField.get(command);
      assertEquals(limits.maxAngularSpeedToStop(Math.toRadians(1)), desired.omegaRadiansPerSecond, 1e-6);
      robot.drivetrain.resetPose(goal);
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(0, desired.omegaRadiansPerSecond);
      assertFalse(command.isFinished());
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.12);
      command.execute();
      assertTrue(command.isFinished());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void autonomousHeadingWrapsWithoutChangingManualControllerHistory() {
    var drive = robot.drivetrain;
    double manualOutput = drive.DRIVE_AT_ANGLE.HeadingController.getLastAppliedOutput();
    drive.resetAutonomousHeadingController();
    double positive = drive.calculateAutonomousHeadingRate(Rotation2d.fromDegrees(179), Rotation2d.fromDegrees(-179));
    assertTrue(positive > 0 && positive < 2);
    drive.resetAutonomousHeadingController();
    double negative = drive.calculateAutonomousHeadingRate(Rotation2d.fromDegrees(-179), Rotation2d.fromDegrees(179));
    assertEquals(-positive, negative, 1e-9);
    assertEquals(manualOutput, drive.DRIVE_AT_ANGLE.HeadingController.getLastAppliedOutput());
  }

  @Test void unlockedPointCommandDoesNotActivateHeadingControl() throws Exception {
    var command = new frc.robot.commands.DriveToPointCommand(robot.drivetrain,
        new Pose2d(TEST_X + 10, TEST_Y, Rotation2d.fromDegrees(45)), 3,
        new frc.powerlib.configs.MotionConstraints(1, 1, 1, 1), false,
        reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      double initialOmega = Math.max(-1, Math.min(1, robot.drivetrain.getState().Speeds.omegaRadiansPerSecond));
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.1);
      command.execute();
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
      assertEquals(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage,
          pointRequest(command).DriveRequestType);
      double expectedOmega = Math.copySign(Math.max(0, Math.abs(initialOmega) - 0.1), initialOmega);
      assertEquals(expectedOmega, pointRequest(command).RotationalRate, 1e-6);
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  private static void arriveAtFirstSectionEndpoint(edu.wpi.first.wpilibj2.command.Command auto, Pose2d endpoint) {
    robot.drivetrain.resetPose(endpoint);
    for (int i = 0; i < 300 && !robot.hasRequest(RobotState.INTAKING) && !auto.isFinished(); i++) {
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      auto.execute();
    }
    assertTrue(robot.hasRequest(RobotState.INTAKING), () -> "Settled arrival must advance the route: pose="
        + robot.drivetrain.getState().Pose + ", speeds=" + robot.drivetrain.getState().Speeds
        + ", finished=" + auto.isFinished());
  }

  @Test void autoLegContinuesPastThreeSecondsWithoutSkippingWaypoints() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var route = frc.robot.autos.RobotAutos.ROUTES.get(3);
    robot.drivetrain.resetPose(route.fieldPoints().get(0));
    var auto = frc.robot.autos.RobotAutos.build(route, robot);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      auto.execute(); // Reach leg 1; leg 2 is now active.
      for (double delay : new double[] {3.1, 4, 4}) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(delay);
        auto.execute();
        assertFalse(auto.isFinished(), "An unreached leg must remain active beyond three seconds");
        assertFalse(robot.hasRequest(RobotState.INTAKING), "Elapsed time must not skip a waypoint");
      }
      // Settled arrival/action handoff is exercised separately, without teleporting a cruising robot.
    } finally {
      auto.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
    assertFalse(robot.hasRequest(RobotState.INTAKING));
    assertFalse(robot.hasRequest(RobotState.SHOOTING));
    assertEquals(0, robot.shooter.getVelocitySetpoint());
    assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
    assertEquals(0, robot.feeder.getVelocitySetpoint());
  }

  @Test void autoStopsWhenAutonomousIsDisabled() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var route = frc.robot.autos.RobotAutos.ROUTES.get(3);
    robot.drivetrain.resetPose(route.fieldPoints().get(0));
    var auto = frc.robot.autos.RobotAutos.build(route, robot);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      auto.execute();
      arriveAtFirstSectionEndpoint(auto, route.fieldPoints().get(1));
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      auto.execute();
      assertTrue(auto.isFinished(), "Disable must still stop the autonomous routine");
    } finally {
      auto.end(false);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
    assertFalse(robot.hasRequest(RobotState.INTAKING));
    assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
    assertEquals(0, robot.feeder.getVelocitySetpoint());
    assertEquals(0, robot.shooter.getVelocitySetpoint());
  }

  @Test void autoStillStopsAtEndOfAutonomousPeriod() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var route = frc.robot.autos.RobotAutos.ROUTES.get(3);
    robot.drivetrain.resetPose(route.fieldPoints().get(0));
    var auto = frc.robot.autos.RobotAutos.build(route, robot);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      auto.execute();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(Constants.Field.AUTO_LENGTH_IN_TIME + .1);
      auto.execute();
      assertTrue(auto.isFinished(), "Removing a leg timeout must not extend the autonomous period");
    } finally {
      auto.end(false);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
    assertFalse(robot.hasRequest(RobotState.INTAKING));
    assertEquals(0, robot.intakeRoller.getVelocitySetpoint());
    assertEquals(0, robot.shooter.getVelocitySetpoint());
  }
  private static frc.powerlib.vision.VisionPoseFilter prepareVisionFilter() throws Exception {
    var field = Vision.class.getDeclaredField("poseFilter");
    field.setAccessible(true);
    var filter = (frc.powerlib.vision.VisionPoseFilter) field.get(robot.vision);
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    for (int i = 20; i >= 0; i--) filter.observe(now - i * 0.02, 0, 0, true);
    return filter;
  }

  @Test void manualVisionSeedRejectsOffFieldHeightAndTiltedCaptureFrames() throws Exception {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    var field = Vision.class.getDeclaredField("lastMeasurement");
    field.setAccessible(true);
    Object previous = field.get(robot.vision);
    var filter = prepareVisionFilter();
    try {
      double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      for (var measurement : java.util.List.of(
          new frc.powerlib.vision.LimelightVision.Measurement("test", new Pose2d(99, 2, Rotation2d.kZero), now, 1, 2, .2),
          new frc.powerlib.vision.LimelightVision.Measurement("test", new Pose2d(2, 2, Rotation2d.kZero), now, 1, 2, .2, 2))) {
        field.set(robot.vision, measurement);
        assertFalse(robot.vision.seedFromFreshVision());
      }
      field.set(robot.vision, new frc.powerlib.vision.LimelightVision.Measurement("test",
          new Pose2d(2, 2, Rotation2d.kZero), now, 1, 2, .2));
      assertTrue(robot.vision.hasFreshMeasurement());
      filter.observe(edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), 12, 0, true);
      assertFalse(robot.vision.hasFreshMeasurement());
      assertFalse(robot.vision.seedFromFreshVision());
      filter.observe(edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), 0, 0, true);
      assertFalse(robot.vision.seedFromFreshVision()); // Level now cannot make the old capture safe.
    } finally { field.set(robot.vision, previous); prepareVisionFilter(); }
  }

  private static java.lang.reflect.Method positionCorrection() throws Exception {
    var apply = Vision.class.getDeclaredMethod("applyPositionCorrection", frc.powerlib.vision.LimelightVision.Measurement.class, double.class);
    apply.setAccessible(true);
    return apply;
  }

  private static frc.powerlib.vision.VisionMeasurementFusion visionFusion() throws Exception {
    var field = Vision.class.getDeclaredField("fusion");
    field.setAccessible(true);
    return (frc.powerlib.vision.VisionMeasurementFusion) field.get(robot.vision);
  }

  @Test void aSingleLargePositionOutlierCannotAdvanceTheFusionTimestamp() throws Exception {
    prepareVisionFilter();
    var fusion = visionFusion();
    double capture = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - .01;
    fusion.resetPose(capture - .01); // Isolate this test from prior manual seeds.
    var outlier = new frc.powerlib.vision.LimelightVision.Measurement("test",
        new Pose2d(TEST_X + 12, TEST_Y, Rotation2d.kZero), capture, 1, 2, .1);
    assertTrue(fusion.isNewPosition(outlier));
    assertFalse((boolean) positionCorrection().invoke(robot.vision, outlier, edu.wpi.first.wpilibj.Timer.getFPGATimestamp()));
    assertTrue(fusion.isNewPosition(outlier), "Rejected frames must not advance the fusion timestamp");
    assertEquals(TEST_X, robot.drivetrain.getState().Pose.getX(), .001);
    assertEquals(TEST_Y, robot.drivetrain.getState().Pose.getY(), .001);
  }

  @Test void visionFusionUsesEachMeasurementsUncertaintyRatherThanTheFixedBase() throws Exception {
    var fusion = visionFusion();
    var apply = positionCorrection();
    double[] displacement = new double[2];
    try {
      for (int run = 0; run < 2; run++) {
        robot.drivetrain.applyRequest(new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake());
        robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
        Thread.sleep(60); // Allow native odometry to retain history at the new starting pose.
        prepareVisionFilter();
        fusion.resetPose(edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - .02);
        double capture = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - .01;
        assertTrue(robot.drivetrain.samplePoseAt(com.ctre.phoenix6.Utils.fpgaToCurrentTime(capture)).isPresent());
        var measurement = new frc.powerlib.vision.LimelightVision.Measurement("test",
            new Pose2d(TEST_X + .4, TEST_Y, Rotation2d.fromDegrees(180)), capture, 1, 2, run == 0 ? .02 : 20);
        apply.invoke(robot.vision, measurement, edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        assertFalse(fusion.isNewPosition(measurement));
        Thread.sleep(20);
        displacement[run] = robot.drivetrain.getState().Pose.getX() - TEST_X;
      }
      assertTrue(displacement[0] > .05, "Low uncertainty must visibly correct pose");
      assertTrue(displacement[0] > 5 * displacement[1], "High uncertainty must apply much less correction");
    } finally {

      robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
    }
  }

  @Test void customShooterTrimChangesOnlyFlywheelDemandAndReadiness() {
    double previous = Constants.Shooter.SHOOTER_EXTRA_RPS;
    try {
      for (double trim : new double[] {0, 2, 5}) {
        Constants.Shooter.SHOOTER_EXTRA_RPS = trim;
        robot.requestState(RobotState.SHOOTING, requestOwner);
        double expected = 28.5 + trim; // Existing 2.5 m calibration plus the one custom trim.
        robot.shooter.inputs.velocityRotationsPerSecond = expected - Constants.Shooting.SHOOTER_TOLERANCE_RPS - .1;
        robot.shooterHood.inputs.positionRotations = .05;
        robot.execute();
        assertEquals(expected, robot.getShotSolution().shooterRps(), 1e-9);
        assertEquals(expected, robot.shooter.getVelocitySetpoint(), 1e-9);
        assertFalse(robot.getShotSolution().velocityReady());
        assertEquals(0, robot.feeder.getVelocitySetpoint());
        robot.shooter.inputs.velocityRotationsPerSecond = expected;
        robot.execute();
        assertEquals(RobotState.SHOOTING, robot.getCurrentState());
        assertTrue(robot.getShotSolution().velocityReady());
        assertEquals(-59, robot.feeder.getVelocitySetpoint(), 1e-9, "Flywheel trim must not change feeder calibration");
      }
      Constants.Shooter.SHOOTER_EXTRA_RPS = Double.NaN;
      robot.execute();
      assertFalse(robot.getShotSolution().valid());
      assertEquals(0, robot.shooter.getVelocitySetpoint());
      assertEquals(0, robot.feeder.getVelocitySetpoint());
    } finally {
      Constants.Shooter.SHOOTER_EXTRA_RPS = previous;
      robot.stopAll();
    }
  }

  @Test void finalAimTimeoutAbortsWithoutRequestingShooting() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var point = new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(90));
    var route = new frc.robot.autos.RobotAutos.Route("Final aim timeout", edu.wpi.first.wpilibj.DriverStation.Alliance.Blue,
        java.util.Collections.nCopies(7, point));
    robot.drivetrain.resetPose(point);
    var auto = frc.robot.autos.RobotAutos.build(route, robot);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      boolean aiming = false;
      for (int i = 0; i < 120 && !aiming && !auto.isFinished(); i++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        auto.execute();
        var heading = robot.drivetrain.getHeadingSetpointDegrees();
        aiming = heading != null && Math.abs(heading - 90) > 1;
        assertFalse(robot.hasRequest(RobotState.SHOOTING));
      }
      assertTrue(aiming, "Route must reach the final aiming command");
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(Constants.Auto.TURN_TIMEOUT_SECONDS + 0.1);
      auto.execute();
      assertTrue(auto.isFinished());
      // Check before cleanup: sequential command initialization must not request shooting on failure.
      assertFalse(robot.hasRequest(RobotState.SHOOTING));
      assertEquals(0, robot.shooter.getVelocitySetpoint());
    } finally {
      auto.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void settledFinalAimAllowsShooting() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    var point = new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180));
    var route = new frc.robot.autos.RobotAutos.Route("Settled final aim", edu.wpi.first.wpilibj.DriverStation.Alliance.Blue,
        java.util.Collections.nCopies(7, point));
    robot.drivetrain.resetPose(point);
    var auto = frc.robot.autos.RobotAutos.build(route, robot);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      int ticks = 0;
      while (ticks < 120 && !robot.hasRequest(RobotState.SHOOTING) && !auto.isFinished()) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        auto.execute();
        ticks++;
      }
      assertTrue(robot.hasRequest(RobotState.SHOOTING));
      assertTrue(ticks * 0.02 >= 4 * Constants.Auto.ARRIVAL_SETTLE_TIME_SECONDS,
          "The three section endpoints and final aim require their own settle window");
      assertFalse(auto.isFinished());
    } finally {
      auto.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  private static frc.powerlib.configs.PathWaypoint pathPoint(double x, double headingDegrees, boolean lockHeading,
      double timeoutSeconds) {
    return new frc.powerlib.configs.PathWaypoint(new Pose2d(x, TEST_Y, Rotation2d.fromDegrees(headingDegrees)),
        new frc.powerlib.configs.MotionConstraints(2, 1, 1, 1),
        Constants.Auto.completionCriteria(3, 10), lockHeading, timeoutSeconds);
  }

  private static frc.powerlib.auto.PathProgress pathProgress(frc.robot.commands.DrivePathCommand command)
      throws ReflectiveOperationException {
    var field = frc.robot.commands.DrivePathCommand.class.getDeclaredField("progress");
    field.setAccessible(true);
    return (frc.powerlib.auto.PathProgress) field.get(command);
  }

  @Test void continuousPathRetainsTranslationAndRotationHistoryAcrossAWaypoint() throws Exception {
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
        java.util.List.of(pathPoint(TEST_X + 2, 90, true, 3), pathPoint(TEST_X + 6, 180, false, 3)),
        reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      for (int i = 0; i < 4; i++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        command.execute();
      }
      double previousX = pointRequest(command).VelocityX;
      double previousOmega = pointRequest(command).RotationalRate;
      assertTrue(previousX > 0.05);
      assertTrue(previousOmega < -0.05);
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 2, TEST_Y, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(1, pathProgress(command).index());
      assertEquals(previousX + 0.02, pointRequest(command).VelocityX, 1e-6);
      assertEquals(previousOmega + 0.02, pointRequest(command).RotationalRate, 1e-6);
      assertEquals(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage,
          pointRequest(command).DriveRequestType);
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
      assertFalse(command.isFinished());
      command.end(true);
      assertEquals(0, pointRequest(command).VelocityX);
      assertEquals(0, pointRequest(command).RotationalRate);
      command.initialize();
      assertEquals(0, pathProgress(command).index());
      assertFalse(command.isFinished());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void continuousPathSkipsDuplicateIntermediatePointsAndSettlesOnlyAtItsEndpoint() throws Exception {
    var point = pathPoint(TEST_X, 180, true, 3);
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain, java.util.List.of(point, point, point),
        reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      for (int run = 0; run < 2; run++) {
        command.initialize();
        command.execute();
        assertEquals(2, pathProgress(command).index());
        assertFalse(command.isFinished());
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.06);
        command.execute();
        assertFalse(command.isFinished());
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.05);
        command.execute();
        assertTrue(command.isFinished());
        command.end(false);
        assertEquals(0, pointRequest(command).VelocityX);
        assertEquals(0, pointRequest(command).RotationalRate);
      }
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void waypointDeadlineRestartsOnProgressButAnExpiredArrivalCannotRescueIt() throws Exception {
    var reasons = new java.util.ArrayList<String>();
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
        java.util.List.of(pathPoint(TEST_X + 1, 180, false, 0.2), pathPoint(TEST_X + 3, 180, false, 0.2)),
        reasons::add);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.12);
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 1, TEST_Y, Rotation2d.fromDegrees(180)));
      command.execute();
      assertEquals(1, pathProgress(command).index());
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.12);
      command.execute();
      assertFalse(command.isFinished()); // More than 0.2 s total, but still within the second leg's deadline.
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.09);
      command.execute();
      assertTrue(command.isFinished());
      assertEquals(java.util.List.of("Autonomous waypoint timed out"), reasons);
      assertEquals(0, pointRequest(command).VelocityX);
      command.end(true);

      robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.21);
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 1, TEST_Y, Rotation2d.fromDegrees(180)));
      command.execute();
      assertTrue(command.isFinished());
      assertEquals(0, pathProgress(command).index()); // Timeout is checked before advancement.
      assertEquals(2, reasons.size());
      assertEquals(0, pointRequest(command).VelocityX);
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void invalidPathHeadingStopsImmediatelyWithoutWaitingForDeceleration() throws Exception {
    var reasons = new java.util.ArrayList<String>();
    var headingAvailable = new java.util.concurrent.atomic.AtomicBoolean(true);
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
        java.util.List.of(pathPoint(TEST_X + 2, 180, true, 3), pathPoint(TEST_X + 6, 180, true, 3)), reasons::add)
        .withHeadingTarget(pose -> headingAvailable.get() ? Rotation2d.fromDegrees(180) : null);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.1);
      command.execute();
      assertTrue(pointRequest(command).VelocityX > 0);
      headingAvailable.set(false);
      command.execute();
      assertTrue(command.isFinished());
      assertEquals(java.util.List.of("Autonomous heading target unavailable"), reasons);
      assertEquals(0, pointRequest(command).VelocityX);
      assertEquals(0, pointRequest(command).RotationalRate);
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void aLargerEndpointToleranceDoesNotStopBeforeAnUnreachedIntermediatePoint() throws Exception {
    var limits = new frc.powerlib.configs.MotionConstraints(1, 1, 1, 1);
    var intermediate = new frc.powerlib.configs.PathWaypoint(
        new Pose2d(TEST_X + 0.1, TEST_Y, Rotation2d.fromDegrees(180)), limits,
        new frc.powerlib.configs.CompletionCriteria(0.01, 0.1, 0.05, 0.1, 0.1), true, 3);
    var endpoint = new frc.powerlib.configs.PathWaypoint(
        new Pose2d(TEST_X + 0.11, TEST_Y, Rotation2d.fromDegrees(180)), limits,
        new frc.powerlib.configs.CompletionCriteria(0.2, 0.1, 0.05, 0.1, 0.1), true, 3);
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain, java.util.List.of(intermediate, endpoint),
        reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(0, pathProgress(command).index());
      assertTrue(pointRequest(command).VelocityX > 0,
          "Being inside the final tolerance must not prevent visiting the intermediate target");
      assertFalse(command.isFinished());
      robot.drivetrain.resetPose(intermediate.pose());
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(1, pathProgress(command).index());
      assertFalse(command.isFinished());
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.12);
      command.execute();
      assertTrue(command.isFinished());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void crossingHandoffKeepsControllerHistoryWhenTheRadiusIsSkipped() throws Exception {
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
        java.util.List.of(pathPoint(TEST_X + 1, 90, true, 3), pathPoint(TEST_X + 3, 180, false, 3)),
        Constants.Auto.waypointHandoff(), reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 0.8, TEST_Y, Rotation2d.fromDegrees(180)));
      for (int i = 0; i < 4; i++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        command.execute();
      }
      assertEquals(0, pathProgress(command).index());
      double previousX = pointRequest(command).VelocityX;
      double previousOmega = pointRequest(command).RotationalRate;
      assertTrue(previousX > 0.05);
      assertTrue(previousOmega < -0.05);
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 1.2, TEST_Y, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(1, pathProgress(command).index());
      assertEquals(previousX + 0.02, pointRequest(command).VelocityX, 1e-6);
      assertEquals(previousOmega + 0.02, pointRequest(command).RotationalRate, 1e-6);
      assertFalse(command.isFinished());
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void endpointCrossingCannotStartTheNextActionWithoutSettling() throws Exception {
    var actionStarted = new java.util.concurrent.atomic.AtomicBoolean();
    var paths = new java.util.ArrayList<frc.robot.commands.DrivePathCommand>();
    var auto = new frc.powerlib.auto.Auto("Endpoint crossing")
        .addDriveToPoint(new Pose2d(TEST_X + 1, TEST_Y, Rotation2d.fromDegrees(180)))
        .addDriveToPoint(new Pose2d(TEST_X + 2, TEST_Y, Rotation2d.fromDegrees(180)))
        .addCommand(() -> edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> actionStarted.set(true)))
        .buildContinuous(section -> {
          var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
              java.util.List.of(pathPoint(TEST_X + 1, 180, true, 3), pathPoint(TEST_X + 2, 180, true, 3)),
              Constants.Auto.waypointHandoff(), reason -> fail(reason));
          paths.add(command);
          return command;
        });
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      for (double offset : new double[] {0.8, 1.2, 1.8, 2.2}) {
        robot.drivetrain.resetPose(new Pose2d(TEST_X + offset, TEST_Y, Rotation2d.fromDegrees(180)));
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        auto.execute();
      }
      assertEquals(1, pathProgress(paths.get(0)).index());
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.12);
      auto.execute();
      assertFalse(paths.get(0).isFinished());
      assertFalse(actionStarted.get());
      assertFalse(auto.isFinished());
    } finally {
      auto.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void crossingRestartsTheDeadlineButCannotRescueAnExpiredWaypoint() throws Exception {
    var reasons = new java.util.ArrayList<String>();
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
        java.util.List.of(pathPoint(TEST_X + 1, 180, true, 0.2), pathPoint(TEST_X + 3, 180, true, 0.2)),
        Constants.Auto.waypointHandoff(), reasons::add);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 0.8, TEST_Y, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.1);
      command.execute();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 1.2, TEST_Y, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.05);
      command.execute();
      assertEquals(1, pathProgress(command).index());
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.1);
      command.execute();
      assertFalse(command.isFinished());
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.11);
      command.execute();
      assertTrue(command.isFinished());
      assertEquals(java.util.List.of("Autonomous waypoint timed out"), reasons);
      assertEquals(0, pointRequest(command).VelocityX);
      command.end(true);

      robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
      command.initialize();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 0.8, TEST_Y, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.1);
      command.execute();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 1.2, TEST_Y, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.11);
      command.execute();
      assertTrue(command.isFinished());
      assertEquals(0, pathProgress(command).index());
      assertEquals(2, reasons.size());
      assertEquals(0, pointRequest(command).VelocityX);
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void rejectedCrossingsDoNotResetTheWaypointDeadline() throws Exception {
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      for (boolean largeJump : new boolean[] {false, true}) {
        robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
        var reasons = new java.util.ArrayList<String>();
        var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
            java.util.List.of(pathPoint(TEST_X + 1, 180, true, 0.2), pathPoint(TEST_X + 3, 180, true, 0.2)),
            Constants.Auto.waypointHandoff(), reasons::add);
        try {
          command.initialize();
          double y = largeJump ? TEST_Y : TEST_Y + 0.2;
          robot.drivetrain.resetPose(new Pose2d(TEST_X + (largeJump ? 0.2 : 0.8), y, Rotation2d.fromDegrees(180)));
          edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.05);
          command.execute();
          robot.drivetrain.resetPose(new Pose2d(TEST_X + 1.2, y, Rotation2d.fromDegrees(180)));
          edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.05);
          command.execute();
          assertEquals(0, pathProgress(command).index());
          edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.11);
          command.execute();
          assertTrue(command.isFinished());
          assertEquals(java.util.List.of("Autonomous waypoint timed out"), reasons);
          assertEquals(0, pointRequest(command).VelocityX);
        } finally {
          command.end(true);
        }
      }
    } finally {
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  private static edu.wpi.first.math.kinematics.ChassisSpeeds desiredPathSpeeds(
      frc.robot.commands.DrivePathCommand command) throws ReflectiveOperationException {
    var field = frc.robot.commands.DrivePathCommand.class.getDeclaredField("desiredSpeeds");
    field.setAccessible(true);
    return (edu.wpi.first.math.kinematics.ChassisSpeeds) field.get(command);
  }

  @Test void crossTrackCorrectionUsesTheExistingSpeedBudgetAndVectorAccelerationLimits() throws Exception {
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
        java.util.List.of(pathPoint(TEST_X + 2, 180, true, 3), pathPoint(TEST_X + 4, 180, true, 3)),
        Constants.Auto.waypointHandoff(), Constants.Auto.crossTrack(), reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      var state = robot.drivetrain.getState();
      var initial = new frc.powerlib.auto.ChassisMotionLimiter(new frc.powerlib.configs.MotionConstraints(2, 1, 1, 1));
      initial.reset(edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation()));
      var previous = initial.calculate(new edu.wpi.first.math.kinematics.ChassisSpeeds(), 0);
      command.initialize();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 0.5, TEST_Y + 0.5, Rotation2d.fromDegrees(180)));
      for (double dt : new double[] {0.02, 0.01, 0.03, 0.04}) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(dt);
        command.execute();
        var desired = desiredPathSpeeds(command);
        var position = robot.drivetrain.getState().Pose.getTranslation();
        assertTrue(desired.vyMetersPerSecond < 0);
        assertTrue(Math.abs(desired.vyMetersPerSecond)
            <= Constants.Auto.CROSS_TRACK_MAX_CORRECTION_SPEED_METERS_PER_SECOND + 1e-6);
        assertTrue(Math.hypot(desired.vxMetersPerSecond, desired.vyMetersPerSecond)
            <= pathProgress(command).maxTranslationSpeed(position) + 1e-6);
        var request = pointRequest(command);
        assertTrue(Math.hypot(request.VelocityX, request.VelocityY) <= 2 + 1e-6);
        assertTrue(Math.hypot(request.VelocityX - previous.vxMetersPerSecond,
            request.VelocityY - previous.vyMetersPerSecond) <= dt + 1e-6);
        previous = new edu.wpi.first.math.kinematics.ChassisSpeeds(request.VelocityX, request.VelocityY, request.RotationalRate);
      }
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 0.5, TEST_Y - 0.5, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertTrue(desiredPathSpeeds(command).vyMetersPerSecond > 0);
      assertTrue(Math.hypot(pointRequest(command).VelocityX - previous.vxMetersPerSecond,
          pointRequest(command).VelocityY - previous.vyMetersPerSecond) <= 0.02 + 1e-6);
      assertEquals(180, robot.drivetrain.getHeadingSetpointDegrees(), 1e-9);
    } finally {
      command.end(true);
      assertEquals(0, pointRequest(command).VelocityX);
      assertEquals(0, pointRequest(command).VelocityY);
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void correctedCornerHandoffTracksTheNextPlannedSegmentWithoutResettingTheLimiter() throws Exception {
    var next = new frc.powerlib.configs.PathWaypoint(
        new Pose2d(TEST_X + 1, TEST_Y + 3, Rotation2d.fromDegrees(180)),
        new frc.powerlib.configs.MotionConstraints(2, 1, 1, 1), Constants.Auto.completionCriteria(3, 10), false, 3);
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
        java.util.List.of(pathPoint(TEST_X + 1, 180, true, 3), next),
        Constants.Auto.waypointHandoff(), Constants.Auto.crossTrack(), reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 0.8, TEST_Y, Rotation2d.fromDegrees(180)));
      for (int i = 0; i < 4; i++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        command.execute();
      }
      double previousX = pointRequest(command).VelocityX;
      double previousY = pointRequest(command).VelocityY;
      assertTrue(previousX > 0.05);
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 1.2, TEST_Y, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(1, pathProgress(command).index());
      assertEquals(new edu.wpi.first.math.geometry.Translation2d(TEST_X + 1, TEST_Y),
          pathProgress(command).currentSegment().start());
      assertTrue(desiredPathSpeeds(command).vxMetersPerSecond < 0);
      assertTrue(desiredPathSpeeds(command).vyMetersPerSecond > 0);
      assertTrue(pointRequest(command).VelocityX > 0); // Retained motion decelerates toward the new request.
      assertTrue(Math.hypot(pointRequest(command).VelocityX - previousX,
          pointRequest(command).VelocityY - previousY) <= 0.02 + 1e-6);
      assertFalse(command.isFinished());
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void correctedEndpointRecoveryStillSettlesBeforeTheNextAction() throws Exception {
    var actionStarted = new java.util.concurrent.atomic.AtomicBoolean();
    var paths = new java.util.ArrayList<frc.robot.commands.DrivePathCommand>();
    var goal = new Pose2d(TEST_X + 1, TEST_Y, Rotation2d.fromDegrees(180));
    var auto = new frc.powerlib.auto.Auto("Corrected endpoint")
        .addDriveToPoint(goal)
        .addCommand(() -> edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> actionStarted.set(true)))
        .buildContinuous(section -> {
          var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
              java.util.List.of(pathPoint(TEST_X + 1, 180, true, 3)),
              Constants.Auto.waypointHandoff(), Constants.Auto.crossTrack(), reason -> fail(reason));
          paths.add(command);
          return command;
        });
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      auto.initialize();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 1.2, TEST_Y + 0.2, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      auto.execute();
      assertTrue(desiredPathSpeeds(paths.get(0)).vxMetersPerSecond < 0);
      assertTrue(desiredPathSpeeds(paths.get(0)).vyMetersPerSecond < 0);
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.12);
      auto.execute();
      assertFalse(actionStarted.get());
      assertFalse(auto.isFinished());
      for (int tick = 0; tick < 60 && !actionStarted.get(); tick++) {
        robot.drivetrain.resetPose(goal);
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        auto.execute();
        if (tick < 4) assertFalse(actionStarted.get());
        Thread.sleep(5); // Let native CTRE feedback reflect the stopping request.
      }
      assertTrue(actionStarted.get());
      assertEquals(0, pointRequest(paths.get(0)).VelocityX);
      assertEquals(0, pointRequest(paths.get(0)).VelocityY);
    } finally {
      auto.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void crossTrackCorrectionPreservesTimeoutsAndImmediateFaultStops() throws Exception {
    var reasons = new java.util.ArrayList<String>();
    var headingAvailable = new java.util.concurrent.atomic.AtomicBoolean(true);
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain,
        java.util.List.of(pathPoint(TEST_X + 2, 180, true, 0.2), pathPoint(TEST_X + 4, 180, true, 0.2)),
        Constants.Auto.waypointHandoff(), Constants.Auto.crossTrack(), reasons::add)
        .withHeadingTarget(pose -> headingAvailable.get() ? Rotation2d.fromDegrees(180) : null);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 0.5, TEST_Y + 0.5, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.05);
      command.execute();
      assertTrue(pointRequest(command).VelocityY < 0);
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.16);
      command.execute();
      assertTrue(command.isFinished());
      assertEquals(java.util.List.of("Autonomous waypoint timed out"), reasons);
      assertEquals(0, pointRequest(command).VelocityX);
      assertEquals(0, pointRequest(command).VelocityY);
      command.end(true);

      robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
      command.initialize();
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 0.5, TEST_Y + 0.5, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertTrue(pointRequest(command).VelocityY < 0);
      headingAvailable.set(false);
      command.execute();
      assertTrue(command.isFinished());
      assertEquals("Autonomous heading target unavailable", reasons.get(1));
      assertEquals(0, pointRequest(command).VelocityX);
      assertEquals(0, pointRequest(command).VelocityY);
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void explicitRotationIsIndependentOfTheDestinationPoseAndUsesTheShortWrappedTurn() throws Exception {
    var limits = new frc.powerlib.configs.MotionConstraints(2, 1, 2, 1);
    var destination = new Pose2d(TEST_X + 10, TEST_Y, Rotation2d.fromDegrees(90));
    var command = new frc.robot.commands.DriveToPointCommand(robot.drivetrain, destination,
        Constants.Auto.completionCriteria(3, 0.5), limits,
        frc.powerlib.auto.RotationTarget.fixed(Rotation2d.fromDegrees(-179)), 3, reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(179)));
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      var desired = desiredPathSpeeds(command);
      assertTrue(desired.vxMetersPerSecond > 0);
      assertEquals(0, desired.vyMetersPerSecond, 1e-6);
      assertTrue(desired.omegaRadiansPerSecond > 0); // +2 degrees across the wrap, not -358.
      assertTrue(desired.omegaRadiansPerSecond <= limits.maxAngularSpeedToStop(Math.toRadians(1.5)) + 1e-6);
      assertEquals(-179, robot.drivetrain.getHeadingSetpointDegrees(), 1e-9);
      assertEquals(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity,
          pointRequest(command).DriveRequestType);
      assertTrue(Math.abs(pointRequest(command).RotationalRate) <= 0.02 + 1e-6);
    } finally {
      command.end(true);
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void liveHeadingOverridesAnUnlockedLegOncePerUpdateAndTargetLossStopsImmediately() throws Exception {
    var calls = new java.util.concurrent.atomic.AtomicInteger();
    var target = new java.util.concurrent.atomic.AtomicReference<Rotation2d>(Rotation2d.fromDegrees(160));
    var reasons = new java.util.ArrayList<String>();
    var command = new frc.robot.commands.DriveToPointCommand(robot.drivetrain,
        new Pose2d(TEST_X + 10, TEST_Y, Rotation2d.kZero), 3,
        new frc.powerlib.configs.MotionConstraints(2, 1, 2, 1), false, reasons::add)
        .withHeadingTarget(pose -> { calls.incrementAndGet(); return target.get(); });
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(1, calls.get());
      assertEquals(160, robot.drivetrain.getHeadingSetpointDegrees(), 1e-9);
      assertTrue(desiredPathSpeeds(command).omegaRadiansPerSecond < 0);
      double previousOmega = pointRequest(command).RotationalRate;
      target.set(Rotation2d.fromDegrees(-160));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(2, calls.get());
      assertEquals(-160, robot.drivetrain.getHeadingSetpointDegrees(), 1e-9);
      assertTrue(desiredPathSpeeds(command).omegaRadiansPerSecond > 0);
      assertTrue(Math.abs(pointRequest(command).RotationalRate - previousOmega) <= 0.02 + 1e-6);
      for (int i = 0; i < 5; i++) assertFalse(command.isFinished());
      assertEquals(2, calls.get());
      target.set(null);
      command.execute(); // Target validity is checked even without a new control timestamp.
      assertEquals(3, calls.get());
      assertTrue(command.isFinished());
      assertEquals(java.util.List.of("Autonomous heading target unavailable"), reasons);
      assertEquals(0, pointRequest(command).VelocityX);
      assertEquals(0, pointRequest(command).VelocityY);
      assertEquals(0, pointRequest(command).RotationalRate);
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void disabledOverrideSkipsWaypointCallbacksAndClearsTheHeadingSetpoint() throws Exception {
    var calls = new java.util.concurrent.atomic.AtomicInteger();
    var waypoint = new frc.powerlib.configs.PathWaypoint(
        new Pose2d(TEST_X + 10, TEST_Y, Rotation2d.fromDegrees(90)),
        new frc.powerlib.configs.MotionConstraints(2, 1, 2, 1), Constants.Auto.completionCriteria(3, 3),
        frc.powerlib.auto.RotationTarget.live(pose -> { calls.incrementAndGet(); throw new IllegalStateException(); }), 3);
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain, java.util.List.of(waypoint), reason -> fail(reason))
        .withRotationTarget(frc.powerlib.auto.RotationTarget.NONE);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      robot.drivetrain.applyRequest(new com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric(), 90.0);
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(0, calls.get());
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
      assertEquals(0, desiredPathSpeeds(command).omegaRadiansPerSecond);
      assertTrue(desiredPathSpeeds(command).vxMetersPerSecond > 0);
      assertFalse(command.isFinished());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void rotationPolicyChangesAtHandoffWithoutResettingTheMotionLimiter() throws Exception {
    var first = new frc.powerlib.configs.PathWaypoint(
        new Pose2d(TEST_X + 2, TEST_Y, Rotation2d.kZero),
        new frc.powerlib.configs.MotionConstraints(2, 1, 2, 1), Constants.Auto.completionCriteria(3, 3),
        frc.powerlib.auto.RotationTarget.fixed(Rotation2d.fromDegrees(90)), 3);
    var second = new frc.powerlib.configs.PathWaypoint(
        new Pose2d(TEST_X + 6, TEST_Y, Rotation2d.kZero), first.constraints(), first.completion(),
        frc.powerlib.auto.RotationTarget.facePoint(new edu.wpi.first.math.geometry.Translation2d(TEST_X + 2, TEST_Y - 10)), 3);
    var command = new frc.robot.commands.DrivePathCommand(robot.drivetrain, java.util.List.of(first, second), reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      for (int i = 0; i < 4; i++) {
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        command.execute();
      }
      double previousX = pointRequest(command).VelocityX;
      double previousOmega = pointRequest(command).RotationalRate;
      assertTrue(previousX > 0.05);
      assertTrue(previousOmega < -0.05);
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 2, TEST_Y, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(1, pathProgress(command).index());
      assertEquals(-90, robot.drivetrain.getHeadingSetpointDegrees(), 1e-6);
      assertTrue(desiredPathSpeeds(command).omegaRadiansPerSecond > 0);
      assertEquals(previousX + 0.02, pointRequest(command).VelocityX, 1e-6);
      assertEquals(previousOmega + 0.02, pointRequest(command).RotationalRate, 1e-6);
      assertFalse(command.isFinished());
      command.end(true);
      command.initialize();
      assertEquals(0, pathProgress(command).index());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void endpointSettlesAgainstTheLiveTargetRatherThanThePoseHeading() throws Exception {
    var target = new java.util.concurrent.atomic.AtomicReference<Rotation2d>(Rotation2d.fromDegrees(90));
    var destination = new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(90));
    var command = new frc.robot.commands.DriveToPointCommand(robot.drivetrain, destination,
        Constants.Auto.completionCriteria(3, 3), new frc.powerlib.configs.MotionConstraints(2, 1, 2, 1),
        frc.powerlib.auto.RotationTarget.live(pose -> target.get()), 3, reason -> fail(reason));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize(); // Robot faces 180; destination pose and initial target both say 90.
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.12);
      command.execute();
      assertFalse(command.isFinished());
      assertEquals(90, robot.drivetrain.getHeadingSetpointDegrees(), 1e-9);
      target.set(Rotation2d.fromDegrees(180));
      robot.drivetrain.applyRequest(new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake());
      // Native feedback continues asynchronously; let the previous turn request settle.
      for (int i = 0; i < 60 && !command.isFinished(); i++) {
        Thread.sleep(5);
        robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
        edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
        command.execute();
        if (i == 0) assertFalse(command.isFinished());
      }
      assertTrue(command.isFinished());
      assertEquals(180, robot.drivetrain.getHeadingSetpointDegrees(), 1e-9);
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void undefinedPointAndThrowingLiveTargetsAbortWithoutEscapingTheCommand() throws Exception {
    var rotationTargets = java.util.List.of(
        frc.powerlib.auto.RotationTarget.facePoint(new edu.wpi.first.math.geometry.Translation2d(TEST_X, TEST_Y)),
        frc.powerlib.auto.RotationTarget.live(pose -> { throw new IllegalStateException("Unavailable"); }),
        frc.powerlib.auto.RotationTarget.live(pose -> Rotation2d.fromRadians(Double.NaN)));
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      for (var target : rotationTargets) {
        var reasons = new java.util.ArrayList<String>();
        var command = new frc.robot.commands.DriveToPointCommand(robot.drivetrain,
            new Pose2d(TEST_X + 10, TEST_Y, Rotation2d.kZero), Constants.Auto.completionCriteria(3, 3),
            new frc.powerlib.configs.MotionConstraints(2, 1, 2, 1), target, 3, reasons::add);
        try {
          robot.drivetrain.resetPose(new Pose2d(TEST_X, TEST_Y, Rotation2d.fromDegrees(180)));
          command.initialize();
          command.execute();
          assertTrue(command.isFinished());
          assertEquals(java.util.List.of("Autonomous heading target unavailable"), reasons);
          assertEquals(0, pointRequest(command).VelocityX);
          assertEquals(0, pointRequest(command).RotationalRate);
          assertNull(robot.drivetrain.getHeadingSetpointDegrees());
        } finally { command.end(true); }
      }
    } finally { edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming(); }
  }

  private static frc.powerlib.configs.BumpCrossingConfig bumpConfig() {
    return new frc.powerlib.configs.BumpCrossingConfig(new frc.powerlib.configs.MotionConstraints(1, 1, 1, 2),
        0.6, 8, 3, 0.06, 0.15, 0.25, 6, 0.1, 0.05, Math.toRadians(5), false);
  }

  private static com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric bumpRequest(
      frc.robot.commands.CrossBumpCommand command) throws ReflectiveOperationException {
    var field = frc.powerlib.auto.BumpCrossingCommand.class.getDeclaredField("drive");
    field.setAccessible(true);
    var adapter = field.get(command);
    var request = adapter.getClass().getDeclaredField("request");
    request.setAccessible(true);
    return (com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric) request.get(adapter);
  }

  @Test void bumpCommandKeepsDrivingDespiteAnOdometryJumpAndCancellationBrakes() throws Exception {
    var reasons = new java.util.ArrayList<String>();
    var command = new frc.robot.commands.CrossBumpCommand(robot.drivetrain, null, bumpConfig(), false, reasons::add);
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      command.initialize();
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      double previous = bumpRequest(command).VelocityX;
      assertTrue(previous > 0);
      robot.drivetrain.resetPose(new Pose2d(TEST_X + 50, TEST_Y + 50, Rotation2d.fromDegrees(180)));
      edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
      command.execute();
      assertEquals(frc.powerlib.auto.BumpCrossingCommand.Phase.APPROACH, command.phase());
      assertEquals(previous + 0.02, bumpRequest(command).VelocityX, 1e-6);
      assertFalse(command.isFinished());
      assertNotNull(robot.drivetrain.getHeadingSetpointDegrees());
      assertEquals(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity,
          bumpRequest(command).DriveRequestType);
      command.end(true);
      assertEquals(frc.powerlib.auto.BumpCrossingCommand.Phase.INTERRUPTED, command.phase());
      assertEquals(java.util.List.of("Bump crossing interrupted"), reasons);
      assertNull(robot.drivetrain.getHeadingSetpointDegrees());
    } finally {
      command.end(true);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void bumpAdapterNormalizesPigeonPitchForForwardAndReverseCrossings() throws Exception {
    var pigeon = robot.drivetrain.getPigeon2();
    edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming();
    try {
      for (boolean reversed : new boolean[] {false, true}) {
        assertTrue(pigeon.getSimState().setPitch(0).isOK());
        for (int i = 0; i < 30 && Math.abs(pigeon.getPitch().refresh().getValueAsDouble()) > 0.01; i++) Thread.sleep(5);
        assertEquals(0, pigeon.getPitch().refresh().getValueAsDouble(), 0.01);
        var reasons = new java.util.ArrayList<String>();
        var command = new frc.robot.commands.CrossBumpCommand(robot.drivetrain, null, bumpConfig(), reversed, reasons::add);
        try {
          command.initialize();
          double rawPitch = reversed ? 11.25 : -11.25; // Exactly representable by the Pigeon pitch signal.
          assertTrue(pigeon.getSimState().setPitch(rawPitch).isOK());
          for (int i = 0; i < 30 && Math.abs(pigeon.getPitch().refresh().getValueAsDouble() - rawPitch) > 0.01; i++) Thread.sleep(5);
          assertEquals(rawPitch, pigeon.getPitch().refresh().getValueAsDouble(), 0.01);
          for (int i = 0; i < 8; i++) {
            edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming(0.02);
            command.execute();
          }
          assertEquals(frc.powerlib.auto.BumpCrossingCommand.Phase.CLIMBING, command.phase());
          assertTrue(reversed ? bumpRequest(command).VelocityX < 0 : bumpRequest(command).VelocityX > 0);
          assertTrue(reasons.isEmpty());
        } finally { command.end(true); }
      }
    } finally {
      pigeon.getSimState().setPitch(0);
      for (int i = 0; i < 30 && Math.abs(pigeon.getPitch().refresh().getValueAsDouble()) > 0.01; i++) Thread.sleep(5);
      edu.wpi.first.wpilibj.simulation.SimHooks.resumeTiming();
    }
  }

  @Test void visionCheckpointAdvancesOnlyWhenPositionFusionRuns() throws Exception {
    var fusedField = Vision.class.getDeclaredField("lastFusedPositionTimestamp");
    fusedField.setAccessible(true);
    var establishedField = Vision.class.getDeclaredField("headingEstablished");
    establishedField.setAccessible(true);
    double previousFused = fusedField.getDouble(robot.vision);
    boolean previousEstablished = establishedField.getBoolean(robot.vision);
    boolean previousEnabled = robot.vision.shouldUpdatePose();
    var apply = Vision.class.getDeclaredMethod("applyMeasurements", java.util.List.class, java.util.List.class);
    apply.setAccessible(true);
    var filter = prepareVisionFilter();
    var fusion = visionFusion();
    Thread.sleep(30); // Populate native capture-time pose history after the preceding clock-stepped test.
    try {
      fusedField.setDouble(robot.vision, Double.NEGATIVE_INFINITY);
      establishedField.setBoolean(robot.vision, true);
      double capture = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - 0.01;
      fusion.resetPose(capture - 0.05);
      var measurement = new frc.powerlib.vision.LimelightVision.Measurement("test-camera",
          robot.drivetrain.getState().Pose, capture, 1, 2, 0.1);
      robot.vision.setShouldUpdatePose(false);
      apply.invoke(robot.vision, java.util.List.of(measurement), java.util.List.of());
      assertEquals(Double.NEGATIVE_INFINITY, robot.vision.getLastAppliedPositionTimestampSeconds());
      robot.vision.setShouldUpdatePose(true);
      apply.invoke(robot.vision, java.util.List.of(measurement), java.util.List.of());
      assertEquals(capture, robot.vision.getLastAppliedPositionTimestampSeconds(),
          () -> filter.status() + "; " + fusion.status());
      var delayed = new frc.powerlib.vision.LimelightVision.Measurement("other-camera",
          robot.drivetrain.getState().Pose, capture - 0.01, 1, 2, 0.1);
      apply.invoke(robot.vision, java.util.List.of(delayed), java.util.List.of());
      assertFalse(fusion.isNewPosition(delayed), "A delayed camera must still be fused");
      assertEquals(capture, robot.vision.getLastAppliedPositionTimestampSeconds(), "A delayed camera cannot move the checkpoint backward");
      fusion.resetPose(capture + 0.005);
      assertEquals(capture, robot.vision.getLastAppliedPositionTimestampSeconds(), "Fusion cutoffs are not position measurements");
    } finally {
      fusedField.setDouble(robot.vision, previousFused);
      establishedField.setBoolean(robot.vision, previousEstablished);
      robot.vision.setShouldUpdatePose(previousEnabled);
    }
  }
}
