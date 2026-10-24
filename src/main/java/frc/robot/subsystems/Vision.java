// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.powerlib.vision.LimelightVision;
import frc.powerlib.vision.VisionPoseFilter;
import frc.powerlib.health.HealthChecks;
import frc.robot.Constants;

public class Vision extends SubsystemBase implements AutoCloseable {
  private final Swerve drivetrain;
  private final LimelightVision limelights;
  private final VisionPoseFilter poseFilter =
      new VisionPoseFilter(Constants.Vision.POSE_FILTER_CONFIG, Constants.Vision.CONFIG.maxMeasurementAgeSeconds());
  private boolean shouldUpdatePose = true;
  private LimelightVision.Measurement lastMeasurement;
  private LimelightVision.HeadingMeasurement lastHeadingMeasurement;
  private double lastAppliedTimestamp = Double.NEGATIVE_INFINITY;
  private double lastAppliedHeadingTimestamp = Double.NEGATIVE_INFINITY;
  private String headingStatus = "Waiting for independent heading";
  private boolean positionAccepted;
  private boolean positionOutOfOrder;
  private final frc.robot.utils.VisionHeadingInitializer headingInitializer =
      new frc.robot.utils.VisionHeadingInitializer();
  private final frc.robot.utils.VisionHeadingInitializer enabledHeadingInitializer =
      new frc.robot.utils.VisionHeadingInitializer();

  public Vision(Swerve drivetrain) {
    this.drivetrain = drivetrain;
    limelights = new LimelightVision(Constants.Vision.CONFIG, Constants.Vision.HEADING_CONFIG,
        Constants.Vision.LIMELIGHT_NAMES);
  }

  public boolean hasFreshMeasurement() {
    return poseFilter.plausible(lastMeasurement, Timer.getFPGATimestamp());
  }

  /** Seed the pose from the last accepted frame, only while disabled and still fresh. */
  public boolean seedFromFreshVision() {
    if (!DriverStation.isDisabled() || !hasFreshMeasurement()) return false;
    double age = Timer.getFPGATimestamp() - lastMeasurement.timestampSeconds();
    var pose = lastMeasurement.pose();
    if (!HealthChecks.isFresh(age, Constants.Vision.CONFIG.maxMeasurementAgeSeconds())
        || !HealthChecks.finitePose(pose)) return false;
    // MT2 yaw depends on the orientation sent to the camera and cannot independently fix drift.
    var rotation = drivetrain.getState().Pose.getRotation();
    if (hasFreshHeading()) rotation = lastHeadingMeasurement.pose().getRotation();
    drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d(pose.getTranslation(), rotation));
    // Do not replay camera frames captured before this manual reset.
    lastAppliedTimestamp = Timer.getFPGATimestamp();
    lastAppliedHeadingTimestamp = lastAppliedTimestamp;
    headingInitializer.reset();
    poseFilter.resetRecovery();
    return true;
  }

  public boolean hasFreshHeading() {
    return lastHeadingMeasurement != null && HealthChecks.finitePose(lastHeadingMeasurement.pose())
        && poseFilter.captureAllowed(lastHeadingMeasurement.timestampSeconds(), Timer.getFPGATimestamp())
        && HealthChecks.isFresh(Timer.getFPGATimestamp() - lastHeadingMeasurement.timestampSeconds(),
            Constants.Vision.HEADING_CONFIG.maxMeasurementAgeSeconds());
  }

  /** Enable or disable vision corrections, for example from an autonomous path event. */
  public void setShouldUpdatePose(boolean shouldUpdate) {
    if (shouldUpdatePose != shouldUpdate) poseFilter.resetRecovery();
    shouldUpdatePose = shouldUpdate;
  }

  public boolean shouldUpdatePose() {
    return shouldUpdatePose;
  }

  @Override
  public void periodic() {
    // Fresh independent heading is needed for disabled pose seeds too.
    limelights.setThrottle(false);
    var state = drivetrain.getState();
    double now = Timer.getFPGATimestamp();
    var pitch = drivetrain.getPigeon2().getPitch();
    var roll = drivetrain.getPigeon2().getRoll();
    boolean gyroHealthy = Double.isFinite(state.Pose.getRotation().getRadians()) && (RobotBase.isSimulation()
        || HealthChecks.signalHealthy(drivetrain.getPigeon2().getYaw(), Constants.Vision.POSE_FILTER_CONFIG.maxOrientationAgeSeconds())
            && HealthChecks.signalHealthy(pitch, Constants.Vision.POSE_FILTER_CONFIG.maxOrientationAgeSeconds())
            && HealthChecks.signalHealthy(roll, Constants.Vision.POSE_FILTER_CONFIG.maxOrientationAgeSeconds()));
    poseFilter.observe(now, pitch.getValueAsDouble(), roll.getValueAsDouble(), gyroHealthy);
    var estimate = limelights.update(
        state.Pose.getRotation().getDegrees(),
        pitch.getValueAsDouble(), roll.getValueAsDouble(),
        Math.toDegrees(state.Speeds.omegaRadiansPerSecond), DriverStation.isDisabled());
    var position = estimate.orElse(null);
    var heading = limelights.getHeadingMeasurement()
        .filter(measurement -> poseFilter.captureAllowed(measurement.timestampSeconds(), now)).orElse(null);
    if (poseFilter.plausible(position, now)) lastMeasurement = position;
    if (heading != null) lastHeadingMeasurement = heading;
    // Continue reading while disabled so those cached frames are not applied after enabling.
    positionAccepted = false;
    positionOutOfOrder = false;
    applyMeasurements(position, heading);
    SignalLogger.writeString("Vision/Position/Status", poseFilter.status());
    frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "PositionStatus",
        !shouldUpdatePose ? "Vision corrections disabled"
            : !DriverStation.isEnabled() ? "Robot disabled"
            : positionOutOfOrder ? "Position predates latest fusion"
            : position == null && gyroHealthy ? "No new position frame; " + poseFilter.status()
            : poseFilter.status());
    frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "PositionAccepted", positionAccepted);
    frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "GyroHealthy", gyroHealthy);
    frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "PitchDegrees", pitch.getValueAsDouble());
    frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "RollDegrees", roll.getValueAsDouble());
    if (position != null) {
      frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "PositionStdDevMeters", position.positionStdDev());
      frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "PositionCamera", position.cameraName());
      SignalLogger.writeDouble("Vision/Position/StdDevMeters", position.positionStdDev(), "meters");
    }
  }

  private void applyMeasurements(LimelightVision.Measurement position,
      LimelightVision.HeadingMeasurement heading) {
    boolean correctedHeading = false;
    headingStatus = !shouldUpdatePose ? "Vision corrections disabled"
        : !DriverStation.isEnabled() ? "Robot disabled"
        : heading == null ? "No fresh reliable multi-tag heading" : "Heading candidate";
    if (shouldUpdatePose && DriverStation.isDisabled()) {
      correctedHeading = initializeHeading(heading, Timer.getFPGATimestamp());
      if (!correctedHeading && headingInitializer.isInitialized() && heading != null && heading.tagCount() >= 2) {
        correctedHeading = applyHeadingCorrection(heading);
      }
    } else {
      headingInitializer.reset();
    }
    if (shouldUpdatePose && DriverStation.isEnabled()) {
      correctedHeading = recoverEnabledHeading(heading, Timer.getFPGATimestamp());
      // Feed delayed measurements in capture order so an older frame cannot undo a newer one.
      boolean headingFirst = heading != null
          && (position == null || heading.timestampSeconds() <= position.timestampSeconds());
      if (headingFirst && !correctedHeading) correctedHeading = applyHeadingCorrection(heading);
      if (position != null) applyPositionCorrection(position);
      if (heading != null && !headingFirst && !correctedHeading) correctedHeading = applyHeadingCorrection(heading);
    } else {
      enabledHeadingInitializer.reset();
    }
    SignalLogger.writeBoolean("Vision/Heading/Accepted", correctedHeading);
    SignalLogger.writeString("Vision/Heading/Status", headingStatus);
    frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "HeadingStatus", headingStatus);
    frc.powerlib.PowerRobotContainer.setSubsystemData("Vision", "MT1HeadingFresh", hasFreshHeading());
    SignalLogger.writeDouble("Vision/Heading/PigeonYawDegrees",
        drivetrain.getPigeon2().getYaw().getValueAsDouble(), "degrees");
    SignalLogger.writeDouble("Vision/Heading/EstimatedDegrees",
        drivetrain.getState().Pose.getRotation().getDegrees(), "degrees");
    if (shouldUpdatePose && DriverStation.isDisabled() && position != null) {
      applyPositionCorrection(position);
    }
  }

  private boolean initializeHeading(LimelightVision.HeadingMeasurement measurement, double now) {
    if (!DriverStation.isDisabled() || !shouldUpdatePose) {
      headingInitializer.reset();
      headingStatus = "Automatic initialization requires disabled robot and enabled vision";
      return false;
    }
    if (!HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      headingInitializer.reset();
      headingStatus = "Waiting for fresh drivetrain feedback";
      return false;
    }
    var speeds = drivetrain.getState().Speeds;
    var rotation = headingInitializer.update(measurement, now,
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
        Math.toDegrees(speeds.omegaRadiansPerSecond));
    headingStatus = headingInitializer.status();
    if (rotation.isEmpty()) return false;
    // Establish absolute field heading without changing X/Y or resetting raw gyro yaw.
    drivetrain.resetRotation(rotation.get());
    lastAppliedTimestamp = now;
    lastAppliedHeadingTimestamp = now;
    SignalLogger.writeString("Robot/Control/Action", "MT1 heading initialized automatically");
    return true;
  }

  private boolean recoverEnabledHeading(LimelightVision.HeadingMeasurement measurement, double now) {
    if (!shouldUpdatePose || !DriverStation.isEnabled() || DriverStation.isTest()
        || !HealthChecks.driveHealthy(drivetrain, Constants.Shooting.MAX_SIGNAL_AGE_SECONDS)) {
      enabledHeadingInitializer.reset();
      return false;
    }
    var speeds = drivetrain.getState().Speeds;
    var rotation = enabledHeadingInitializer.update(measurement, now,
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
        Math.toDegrees(speeds.omegaRadiansPerSecond), 2);
    headingStatus = enabledHeadingInitializer.status();
    if (rotation.isEmpty()) return false;
    drivetrain.resetRotation(rotation.get());
    lastAppliedTimestamp = now;
    lastAppliedHeadingTimestamp = now;
    poseFilter.resetRecovery();
    enabledHeadingInitializer.reset();
    headingStatus = "Stationary two-tag heading recovered";
    SignalLogger.writeString("Robot/Control/Action", headingStatus);
    return true;
  }

  private void applyPositionCorrection(LimelightVision.Measurement measurement) {
    if (measurement.timestampSeconds() < lastAppliedTimestamp) {
      positionOutOfOrder = true;
      return;
    }
    var historical = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(measurement.timestampSeconds()));
    if (!poseFilter.acceptPosition(measurement, historical.orElse(null), Timer.getFPGATimestamp())) return;
    drivetrain.addVisionMeasurement(
        measurement.pose(), measurement.timestampSeconds(), measurement.standardDeviations());
    lastAppliedTimestamp = measurement.timestampSeconds();
    positionAccepted = true;
  }

  private boolean applyHeadingCorrection(LimelightVision.HeadingMeasurement measurement) {
    if (DriverStation.isTest()) { headingStatus = "Test mode"; return false; }
    if (measurement.timestampSeconds() <= lastAppliedHeadingTimestamp) {
      headingStatus = "Repeated or out-of-order heading"; return false;
    }
    lastAppliedHeadingTimestamp = measurement.timestampSeconds();
    // Swerve's addVisionMeasurement wrapper accepts FPGA time; samplePoseAt needs CTRE time.
    var historicalPose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(measurement.timestampSeconds()));
    if (historicalPose.isEmpty()) { headingStatus = "Capture pose unavailable"; return false; }
    SignalLogger.writeDouble("Vision/Heading/ErrorDegrees",
        measurement.pose().getRotation().minus(historicalPose.get().getRotation()).getDegrees(), "degrees");
    // MT1 can arrive later than MT2. Fuse at the latest applied time, carrying forward
    // the capture-time error through intervening rotation, so neither stream undoes the other.
    double fusionTimestamp = Math.max(lastAppliedTimestamp, measurement.timestampSeconds());
    var fusionPose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(fusionTimestamp));
    if (fusionPose.isEmpty()) { headingStatus = "Fusion pose unavailable"; return false; }
    var correction = measurement.correctionPose(historicalPose.get(), fusionPose.get(),
        Constants.Vision.HEADING_CONFIG.maxHeadingErrorDegrees());
    if (correction.isEmpty()) { headingStatus = "Heading error exceeds limit"; return false; }
    drivetrain.addVisionMeasurement(
        correction.get(), fusionTimestamp, measurement.standardDeviations());
    lastAppliedTimestamp = fusionTimestamp;
    headingStatus = "Independent heading applied";
    return true;
  }

  @Override
  public void close() {
    CommandScheduler.getInstance().unregisterSubsystem(this);
    limelights.close();
  }
}
