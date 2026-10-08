package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.powerlib.PowerRobotContainer;
import frc.powerlib.subsystems.AbsolutePositionSubsystem;
import frc.powerlib.subsystems.RelativePositionSubsystem;
import frc.powerlib.subsystems.VelocitySubsystem;
import frc.powerlib.subsystems.VelocityTorqueSubsystem;
import frc.robot.Constants;
import frc.robot.game.Game2026Config;
import frc.robot.game.Game2026Controller;
import frc.robot.game.Game2026Health;
import frc.robot.game.Game2026Settings;
import frc.robot.utils.FieldUtils;

public class StateMachine extends SubsystemBase {
  public enum RobotState { IDLE, READY, SHOOTING }

  private final Game2026Config gameConfig = Game2026Config.load();
  private final Game2026Controller controller = new Game2026Controller(gameConfig);
  public final Swerve drivetrain = Constants.Tuner.createDrivetrain();
  public final Vision vision = new Vision(drivetrain);

  // POWERLIB GENERATED SUBSYSTEMS START - DO NOT DELETE
  public final VelocitySubsystem feeder = new VelocitySubsystem(Constants.Feeder.FEEDER_CONFIG);
  public final VelocityTorqueSubsystem intakeRoller = new VelocityTorqueSubsystem(Constants.IntakeRoller.INTAKE_ROLLER_CONFIG);
  public final AbsolutePositionSubsystem intakeWrist = new AbsolutePositionSubsystem(Constants.IntakeWrist.INTAKE_WRIST_CONFIG);
  public final VelocitySubsystem shooter = new VelocitySubsystem(Constants.Shooter.SHOOTER_CONFIG);
  public final AbsolutePositionSubsystem shooterHood = new AbsolutePositionSubsystem(Constants.ShooterHood.SHOOTER_HOOD_CONFIG);
  public final VelocitySubsystem spindexer = new VelocitySubsystem(Constants.Spindexer.SPINDEXER_CONFIG);
  public final AbsolutePositionSubsystem turret = new AbsolutePositionSubsystem(Constants.Turret.TURRET_CONFIG);
  // POWERLIB GENERATED SUBSYSTEMS END - DO NOT DELETE

  private RobotState wantedState = Constants.StateMachine.DEFAULT_STATE;
  private boolean collecting, ejecting;
  private String autoStatus = "NONE", autoReason = "No autonomous selected";
  private Game2026Controller.Output output;

  public StateMachine() { FieldUtils.configure(gameConfig); }
  public Game2026Config getGameConfig() { return gameConfig; }
  public RobotState getWantedState() { return wantedState; }
  public void setWantedState(RobotState state) { wantedState = state; }
  public void setIntake(boolean collect, boolean eject) { collecting = collect; ejecting = eject; }
  public void resetState() {
    wantedState = RobotState.READY;
    collecting = ejecting = false;
    controller.reset();
  }
  public void stopAll() {
    resetState();
    shooter.stopVelocity(); shooter.brake();
    feeder.stopVelocity(); feeder.brake();
    spindexer.stopVelocity(); spindexer.brake();
    intakeRoller.stopVelocity(); intakeRoller.brake();
    shooterHood.stopPosition(); intakeWrist.stopPosition(); turret.stopPosition();
  }
  public void setAutoStatus(String status, String reason) { autoStatus = status; autoReason = reason; }

  @Override public void periodic() {
    if (DriverStation.isDisabled()) { resetState(); publish("DISABLED"); return; }
    // Generated characterization commands own motor output in Test mode.
    if (DriverStation.isTest()) { resetState(); publish("SYSID OWNS OUTPUTS"); return; }
    controller.setWantedState(Game2026Controller.State.valueOf(wantedState.name()));
    controller.setIntakeMode(ejecting ? Game2026Controller.IntakeMode.EJECT
        : collecting ? Game2026Controller.IntakeMode.COLLECT : Game2026Controller.IntakeMode.OFF);
    boolean healthy = Game2026Health.drive(drivetrain) && Game2026Health.mechanism(shooter)
        && Game2026Health.mechanism(shooterHood) && Game2026Health.mechanism(feeder)
        && Game2026Health.mechanism(spindexer) && Game2026Health.mechanism(intakeWrist);
    output = controller.update(new Game2026Controller.Inputs(true, false, DriverStation.getAlliance().orElse(null),
        drivetrain.getState().Pose, shooter.inputs.velocityRotationsPerSecond, shooterHood.inputs.positionRotations,
        healthy, Timer.getFPGATimestamp()), Game2026Settings.current(),
        new Game2026Config.Intake(Constants.IntakeWrist.INTAKE_MAX,
            Constants.IntakeWrist.INTAKE_IDLE, Constants.IntakeWrist.INTAKE_FEED,
            Constants.IntakeWrist.INTAKE_DEFAULT,
            gameConfig.intake().collectRps(), gameConfig.intake().ejectRps()));
    intakeWrist.setPositionRotations(MathUtil.clamp(output.wristRotations(),
        Math.min(Constants.IntakeWrist.INTAKE_MIN, Constants.IntakeWrist.INTAKE_MAX),
        Math.max(Constants.IntakeWrist.INTAKE_MIN, Constants.IntakeWrist.INTAKE_MAX)));
    if (output.rollerRps() == 0) { intakeRoller.stopVelocity(); intakeRoller.brake(); }
    else intakeRoller.setVelocity(output.rollerRps());
    if (output.runShooter()) {
      shooter.setVelocity(output.shooterRps());
      shooterHood.setPositionRotations(MathUtil.clamp(output.hoodRotations(),
          Constants.ShooterHood.SHOOTER_HOOD_MIN, Constants.ShooterHood.SHOOTER_HOOD_MAX));
    } else { shooter.stopVelocity(); shooter.brake(); }
    if (output.feedReady()) { feeder.setVelocity(output.feederRps()); spindexer.setVelocity(output.spindexerRps()); }
    else { feeder.stopVelocity(); feeder.brake(); spindexer.stopVelocity(); spindexer.brake(); }
    publish(output.reason());
  }

  private void publish(String reason) {
    boolean active = DriverStation.isEnabled() && !DriverStation.isTest();
    PowerRobotContainer.setSubsystemData("Game2026", "Heartbeat", Timer.getFPGATimestamp());
    PowerRobotContainer.setSubsystemData("Game2026", "ShotRequested", active && wantedState == RobotState.SHOOTING);
    PowerRobotContainer.setSubsystemData("Game2026", "VelocityReady", active && output != null && output.velocityReady());
    PowerRobotContainer.setSubsystemData("Game2026", "HoodReady", active && output != null && output.hoodReady());
    PowerRobotContainer.setSubsystemData("Game2026", "Aligned", active && output != null && output.aligned());
    PowerRobotContainer.setSubsystemData("Game2026", "CalibratedRange", active && output != null && output.calibratedRange());
    PowerRobotContainer.setSubsystemData("Game2026", "Target", active && output != null && output.hubAvailable() ? "HUB" : "NONE");
    PowerRobotContainer.setSubsystemData("Game2026", "VisionAccepted", vision.hasFreshAcceptedFrame());
    PowerRobotContainer.setSubsystemData("Game2026", "State", wantedState.name());
    PowerRobotContainer.setSubsystemData("Game2026", "ShotStatus", reason);
    PowerRobotContainer.setSubsystemData("Game2026", "Collecting", collecting);
    PowerRobotContainer.setSubsystemData("Game2026", "Ejecting", ejecting);
    PowerRobotContainer.setSubsystemData("Game2026", "FeedReady", output != null && DriverStation.isEnabled()
        && !DriverStation.isTest() && output.feedReady());
    if (output != null) {
      PowerRobotContainer.setSubsystemData("Game2026", "HubDistance", output.distance(), "meters");
      PowerRobotContainer.setSubsystemData("Game2026", "HeadingError", output.headingErrorDegrees(), "degrees");
    }
    PowerRobotContainer.setSubsystemData("Game2026", "AutoStatus", autoStatus);
    PowerRobotContainer.setSubsystemData("Game2026", "AutoReason", autoReason);
    SmartDashboard.putString("Auto Status", autoStatus);
    SmartDashboard.putString("Auto Reason", autoReason);
  }
}






