// POWERLIB GENERATED AUTOS - DO NOT EDIT
// Power Tool writes this file from power-tool/generated/powerlib-autos.json.
// Edit autos in the Power Tool Auto Builder; hand-written autos belong in RobotAutos.java.
package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.powerlib.auto.Auto;
import frc.powerlib.auto.AutoBuilder;
import frc.powerlib.auto.DriveConstraints;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RequestState;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;

/** Autos built in the Power Tool Auto Builder. */
public final class GeneratedAutos {
  /** Owns the state requests made by generated autos. */
  private static final Object OWNER = new Object();

  private GeneratedAutos() {}

  /** In simulation only, puts the robot at the auto's first point; a real robot keeps its vision pose. */
  private static Command startInSimulation(StateMachine robot, Pose2d start) {
    return Commands.runOnce(() -> {
      if (RobotBase.isSimulation()) robot.drivetrain.resetPose(start);
    });
  }

  public static void register(AutoBuilder autoBuilder, StateMachine robot) {
    autoBuilder.setDefaultAuto(new Auto("power_tool red left")
        .withAlliance(Alliance.Red)
        .addCommand(() -> startInSimulation(robot, new Pose2d(13.037, 2.466, Rotation2d.fromDegrees(-90.0))))
        .addCommand(() -> new RequestState(RobotState.IDLE, OWNER, robot))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(13.037, 2.466, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(3.0, 3.0, 0.05, 0.1)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(10.858, 2.466, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(3.0, 3.0, 0.2, 1.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(8.94, 0.725, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(3.0, 3.0, 0.5, 1.0))
            .atProgress(0.8, () -> robot.requestState(RobotState.INTAKING, OWNER)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(8.761, 3.503, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(1.5, 3.0, 0.5, 1.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(10.064, 2.534, Rotation2d.fromDegrees(135.0)), new DriveConstraints(1.5, 3.0, 0.2, 2.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(10.858, 2.466, Rotation2d.fromDegrees(180.0)), new DriveConstraints(3.0, 3.0, 0.2, 3.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(13.037, 2.466, Rotation2d.fromDegrees(180.0)), new DriveConstraints(3.0, 3.0, 0.2, 3.0))
            .atProgress(0.25, () -> robot.requestState(RobotState.SPINNING_UP, OWNER)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(13.845, 2.466, Rotation2d.fromDegrees(135.0)), new DriveConstraints(3.0, 3.0, 0.05, 0.1)))
        .addCommand(() -> new RequestState(RobotState.SHOOTING, OWNER, robot))
        .addCommand(() -> Commands.runOnce(() -> robot.clearRequest(OWNER), robot)));
  }
}
