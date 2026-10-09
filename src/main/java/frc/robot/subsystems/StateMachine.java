// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.ShootingUtils;
import frc.robot.subsystems.states.IntakingState;
import frc.robot.subsystems.states.EjectingState;
import frc.robot.subsystems.states.SpinningUpState;
import frc.robot.subsystems.states.ShootingState;
import frc.powerlib.statemachine.State;
import frc.powerlib.subsystems.AbsolutePositionSubsystem;
import frc.powerlib.subsystems.RelativePositionSubsystem;
import frc.powerlib.subsystems.VelocitySubsystem;
import frc.powerlib.subsystems.VelocityTorqueSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.states.IdleState;
import java.util.List;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

public class StateMachine extends SubsystemBase {
  public enum RobotState {
    IDLE, READY, INTAKING, EJECTING, SPINNING_UP, SHOOTING
  }

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
  private RobotState currentState = RobotState.IDLE;
  private final Map<Object, RobotState> requests = new LinkedHashMap<>();
  private boolean requestsChanged;
  // Registration order is priority: the first matching handler wins.
  private final IdleState idleState = new IdleState();
  private final ShootingState shootingState = new ShootingState();
  private final List<State<RobotState, StateMachine>> states =
      List.of(
          new EjectingState(),
          shootingState,
          new SpinningUpState(shootingState),
          new IntakingState(),
          idleState);
  private State<RobotState, StateMachine> activeState = idleState;

  private void resetState() {
    wantedState = Constants.StateMachine.DEFAULT_STATE;
    currentState = RobotState.IDLE;
    activeState = idleState;
    requests.clear();
    requestsChanged = true;
    for (State<RobotState, StateMachine> state : states) state.reset();
  }

  public void stopAll() {
    resetState();
    ShootingUtils.stopShooter(this);
    ShootingUtils.stopFeeding(this);
    intakeRoller.stopVelocity();
    intakeRoller.brake();
    intakeWrist.stopPosition();
    shooterHood.stopPosition();
    turret.stopPosition();
  }

  public RobotState getWantedState() {
    return wantedState;
  }

  /** Replaces only this owner's request. Handler order decides which request wins. */
  public void requestState(RobotState requestedState, Object owner) {
    Objects.requireNonNull(requestedState);
    Objects.requireNonNull(owner);
    requestsChanged |= requests.put(owner, requestedState) != requestedState;
  }

  public void clearRequest(Object owner) {
    requestsChanged |= requests.remove(Objects.requireNonNull(owner)) != null;
  }

  public boolean hasRequest(RobotState requestedState) {
    return requests.containsValue(requestedState);
  }

  public RobotState getCurrentState() {
    return currentState;
  }

  /** The dashboard's actual state is the same state exposed to robot commands. */
  public RobotState getActualState() { return getCurrentState(); }

  public void setActualState(RobotState actualState) {
    this.currentState = Objects.requireNonNull(actualState);
  }

  public void execute() {
    // State handlers never compete with disabled outputs or Test/SysId commands.
    if (DriverStation.isDisabled() || DriverStation.isTest()) {
      resetState();
      return;
    }
    for (State<RobotState, StateMachine> state : states) state.prepare(this);
    selectState();
    activeState.execute(wantedState, this);
  }

  private void selectState() {
    // A new request may outrank the current state. Multiple requests can have changing guards.
    if (!requestsChanged && requests.size() <= 1 && wantedState == currentState) {
      return;
    }
    requestsChanged = false;

    for (State<RobotState, StateMachine> state : states) {
      for (RobotState request : requests.values()) {
        if (state.match(request, this)) {
          wantedState = request;
          activeState = state;
          return;
        }
      }
      if (requests.isEmpty() && state.match(Constants.StateMachine.DEFAULT_STATE, this)) {
        wantedState = Constants.StateMachine.DEFAULT_STATE;
        activeState = state;
        return;
      }
    }
    if (!requests.isEmpty()) wantedState = requests.values().iterator().next();
    // Keep running the current state until a handler can accept the request.
  }

  @Override
  public void periodic() {
    execute();
  }
}
