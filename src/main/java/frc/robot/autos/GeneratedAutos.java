// POWERLIB GENERATED AUTOS - DO NOT EDIT
// Power Tool writes this file from power-tool/generated/powerlib-autos.json.
// Edit autos in the Power Tool Auto Builder; hand-written autos belong in RobotAutos.java.
package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  // Named points from the Auto Builder; moving one there moves it in every auto.
  private static final Pose2d LEFT_BUMP = new Pose2d(6.009, 4.565, Rotation2d.fromDegrees(0.0));

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
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(13.037, 2.466, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(3.0, 3.0, 0.05, 0.1))
            .atProgress(0.0, () -> robot.requestState(RobotState.IDLE, OWNER)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(10.858, 2.466, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(3.0, 3.0, 0.2, 1.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(9.836, 1.194, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(3.0, 3.0, 0.5, 5.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(9.083, 0.919, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(3.0, 3.0, 0.5, 1.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(8.761, 3.503, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(1.5, 3.0, 0.5, 1.0))
            .atProgress(0.0, () -> robot.requestState(RobotState.INTAKING, OWNER)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(9.638, 4.035, Rotation2d.fromDegrees(104.2)), new DriveConstraints(1.5, 3.0, 1.0, 2.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(9.584, 2.412, Rotation2d.fromDegrees(120.0)), new DriveConstraints(1.5, 3.0, 0.5, 2.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(10.858, 2.466, Rotation2d.fromDegrees(180.0)), new DriveConstraints(1.5, 3.0, 0.2, 3.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(13.037, 2.466, Rotation2d.fromDegrees(180.0)), new DriveConstraints(4.0, 4.0, 0.2, 3.0))
            .atProgress(0.0, () -> robot.requestState(RobotState.IDLE, OWNER))
            .atProgress(0.5, () -> robot.requestState(RobotState.SPINNING_UP, OWNER)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(13.845, 2.466, Rotation2d.fromDegrees(135.0)), new DriveConstraints(3.0, 3.0, 0.05, 0.1)))
        .addCommand(() -> new RequestState(RobotState.SHOOTING, OWNER, robot))
        .addCommand(() -> Commands.waitSeconds(10.0))
        .addCommand(() -> Commands.runOnce(() -> robot.clearRequest(OWNER), robot)));

    autoBuilder.addAuto(new Auto("Auto 2")
        .addCommand(() -> startInSimulation(robot, new Pose2d(2.83, 4.19, Rotation2d.fromDegrees(0.0))))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(2.83, 4.19, Rotation2d.fromDegrees(0.0)), new DriveConstraints(3.0, 3.0, 0.05, 0.1)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(15.58, 3.7, Rotation2d.fromDegrees(158.21)), new DriveConstraints(3.0, 3.0, 0.05, 0.1))
            .facing(new Translation2d(7.6, 6.89))));

    autoBuilder.addAuto(new Auto("Auto 1")
        .addCommand(() -> startInSimulation(robot, new Pose2d(6.009, 4.565, Rotation2d.fromDegrees(0.0))))
        .addCommand(() -> new DriveToPose(robot.drivetrain, LEFT_BUMP, new DriveConstraints(3.0, 3.0, 0.05, 0.1)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(10.315, 3.65, Rotation2d.fromDegrees(0.0)), new DriveConstraints(3.0, 3.0, 0.05, 0.1)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(9.979, 1.608, Rotation2d.fromDegrees(0.0)), new DriveConstraints(3.0, 3.0, 0.5, 1.0)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(8.701, 1.227, Rotation2d.fromDegrees(-90.0)), new DriveConstraints(3.0, 3.0, 0.05, 0.1)))
        .addCommand(() -> new DriveToPose(robot.drivetrain, new Pose2d(6.009, 4.565, Rotation2d.fromDegrees(-83.25)), new DriveConstraints(3.0, 3.0, 0.05, 0.1))
            .facing(new Translation2d(6.34, 1.77))));
  }
}
