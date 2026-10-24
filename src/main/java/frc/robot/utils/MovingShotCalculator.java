package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.firecontrol.ShotCalculator;
import frc.firecontrol.ShotLUT;
import frc.firecontrol.ShotParameters;
import frc.powerlib.health.HealthChecks;
import frc.powerlib.math.LinearInterpolator;
import frc.robot.Constants;
import frc.robot.utils.ShootingUtils.ShotSolution;

/** Adapts firecontrol to our center-distance calibrations and rear-mounted shooter. */
public final class MovingShotCalculator {
  private final ShotCalculator.Config config = new ShotCalculator.Config();
  private final ShotCalculator solver = new ShotCalculator(config);
  private final double calibrationOffset;
  private static final double LOOP_SECONDS = 0.02;
  private ChassisSpeeds filteredFieldVelocity;

  public MovingShotCalculator() {
    config.launcherOffsetX = Constants.Shooting.LAUNCHER_OFFSET_X_METERS;
    config.launcherOffsetY = Constants.Shooting.LAUNCHER_OFFSET_Y_METERS;
    config.shooterAngleOffsetRad = Constants.Shooting.SHOOTER_ANGLE_OFFSET_RADIANS;
    calibrationOffset = config.launcherOffsetX * Math.cos(config.shooterAngleOffsetRad)
        + config.launcherOffsetY * Math.sin(config.shooterAngleOffsetRad);
    config.minScoringDistance = Constants.Shooting.MIN_SHOT_DISTANCE - calibrationOffset;
    config.maxScoringDistance = Constants.Shooting.MAX_SHOT_DISTANCE - calibrationOffset;
    config.minSOTMSpeed = Constants.Shooting.SOTM_MIN_SPEED_MPS;
    config.maxTiltDeg = Constants.Shooting.SOTM_MAX_TILT_DEGREES;
    // Phoenix pose is already current; avoid compensating the vision latency twice.
    config.phaseDelayMs = 0;
    var tof = new LinearInterpolator(Constants.Shooting.SHOOTER_TOF);
    var lut = new ShotLUT();
    for (double[] row : Constants.Shooting.SHOOTER_SPEEDS) {
      // The existing tables use robot-center distance. Shift keys to launcher distance
      // so an aligned stationary shot retains exactly the same speed and hood demand.
      lut.put(row[0] - calibrationOffset, new ShotParameters(row[1],
          Constants.Shooting.HOOD_ANGLE_INTERPOLATOR.getInterpolatedValue(row[0]) * 360,
          tof.getInterpolatedValue(row[0])));
    }
    solver.loadShotLUT(lut);
  }

  /** Clears the solver warm start and velocity filter, e.g. when a shot request ends. */
  public void reset() {
    solver.resetWarmStart();
    filteredFieldVelocity = null;
  }

  /** Single-pole low-pass on field velocity so stick changes and encoder noise move the lead smoothly. */
  private ChassisSpeeds filter(ChassisSpeeds measured) {
    double tau = Constants.Shooting.SOTM_VELOCITY_FILTER_SECONDS;
    double alpha = Double.isFinite(tau) && tau > 0 ? LOOP_SECONDS / (tau + LOOP_SECONDS) : 1;
    var last = filteredFieldVelocity;
    filteredFieldVelocity = last == null ? measured : new ChassisSpeeds(
        last.vxMetersPerSecond + alpha * (measured.vxMetersPerSecond - last.vxMetersPerSecond),
        last.vyMetersPerSecond + alpha * (measured.vyMetersPerSecond - last.vyMetersPerSecond),
        last.omegaRadiansPerSecond + alpha * (measured.omegaRadiansPerSecond - last.omegaRadiansPerSecond));
    return filteredFieldVelocity;
  }

  public ShotSolution calculate(Pose2d pose, ChassisSpeeds robotVelocity, Alliance alliance,
      double actualRps, double actualHood, boolean feedbackHealthy, boolean visionFresh,
      double pitchDegrees, double rollDegrees) {
    // Filter every cycle, even while the shot is invalid, so the filter never resumes from stale motion.
    boolean velocityValid = robotVelocity != null && HealthChecks.finitePose(pose)
        && Double.isFinite(robotVelocity.vxMetersPerSecond)
        && Double.isFinite(robotVelocity.vyMetersPerSecond)
        && Double.isFinite(robotVelocity.omegaRadiansPerSecond);
    if (!velocityValid) filteredFieldVelocity = null;
    var fieldVelocity = velocityValid
        ? filter(ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, pose.getRotation())) : null;
    var base = ShootingUtils.calculate(pose, alliance, actualRps, actualHood, feedbackHealthy);
    if (!base.valid() || !ShootingUtils.sotmEnabled()) { solver.resetWarmStart(); return base; }
    if (fieldVelocity == null) {
      solver.resetWarmStart(); return ShootingUtils.blocked(base, "Invalid motion or tilt feedback");
    }
    double speed = Math.hypot(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    // Not driving, including turning in place to aim: use the original standing shot unchanged.
    // Turning alone used to count as moving and could block long shots mid-turn.
    if (speed < config.minSOTMSpeed) { solver.resetWarmStart(); return base; }
    if (!Double.isFinite(pitchDegrees) || !Double.isFinite(rollDegrees)) {
      solver.resetWarmStart(); return ShootingUtils.blocked(base, "Invalid motion or tilt feedback");
    }
    double maxSpeed = Constants.Shooting.SOTM_MAX_DRIVETRAIN_SPEED_MPS;
    double releaseDelay = Constants.Shooting.SOTM_RELEASE_DELAY_SECONDS;
    double drag = Constants.Shooting.SOTM_DRAG_COEFFICIENT;
    double maxTilt = Constants.Shooting.SOTM_MAX_TILT_DEGREES;
    if (!Double.isFinite(maxSpeed) || maxSpeed <= config.minSOTMSpeed
        || !Double.isFinite(releaseDelay) || releaseDelay < 0 || releaseDelay > 0.5
        || !Double.isFinite(drag) || drag < 0 || !Double.isFinite(maxTilt) || maxTilt <= 0) {
      solver.resetWarmStart(); return ShootingUtils.blocked(base, "Invalid SOTM settings");
    }
    if (config.maxSOTMSpeed != maxSpeed || config.mechLatencyMs != releaseDelay * 1000
        || config.sotmDragCoeff != drag) solver.resetWarmStart();
    config.maxSOTMSpeed = maxSpeed;
    config.mechLatencyMs = releaseDelay * 1000;
    config.sotmDragCoeff = drag;
    config.maxTiltDeg = maxTilt;
    if (speed > maxSpeed) {
      solver.resetWarmStart(); return ShootingUtils.blocked(base, "Above SOTM speed limit");
    }
    if (Math.abs(pitchDegrees) > config.maxTiltDeg || Math.abs(rollDegrees) > config.maxTiltDeg) {
      solver.resetWarmStart(); return ShootingUtils.blocked(base, "Robot tilt exceeds shooting limit");
    }
    var filteredRobotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(fieldVelocity, pose.getRotation());
    var target = alliance == Alliance.Red ? Constants.Shooting.RED_HUB : Constants.Shooting.BLUE_HUB;
    var result = solver.calculate(new ShotCalculator.ShotInputs(pose, fieldVelocity, filteredRobotVelocity,
        target, new Translation2d(), visionFresh ? 1 : 0.5, pitchDegrees, rollDegrees));
    if (!result.isValid()) {
      solver.resetWarmStart();
      // Driving without a valid lead solution must never fall back to feeding a static shot.
      return ShootingUtils.blocked(base, "No calibrated converged moving shot");
    }
    double heading = result.driveAngle().getDegrees();
    double error = MathUtil.inputModulus(heading - pose.getRotation().getDegrees(), -180, 180);
    double calibratedRps = result.shooterRps();
    double rps = calibratedRps + Constants.Shooter.SHOOTER_EXTRA_RPS;
    double hood = MathUtil.clamp(solver.getHoodAngle(result.solvedDistanceM()) / 360
        + Constants.StateMachine.HOOD_SHOT_OFFSET_ROTATIONS,
        Constants.ShooterHood.SHOOTER_HOOD_MIN, Constants.ShooterHood.SHOOTER_HOOD_MAX);
    return new ShotSolution(true, base.distanceMeters(), heading, error, rps, hood,
        ShootingUtils.feederRps(calibratedRps),
        Double.isFinite(actualRps) && Math.abs(actualRps - rps) <= Constants.Shooting.SHOOTER_TOLERANCE_RPS,
        Double.isFinite(actualHood) && Math.abs(actualHood - hood) <= Constants.StateMachine.HOOD_TOLERANCE_ROTATIONS,
        Math.abs(error) <= Constants.Shooting.HEADING_TOLERANCE_DEGREES, feedbackHealthy,
        true, result.iterationsUsed() > 0, result.solvedDistanceM() + calibrationOffset,
        result.timeOfFlightSec(), result.driveAngularVelocityRadPerSec(), result.confidence(),
        result.iterationsUsed() > 0 ? "Compensated moving shot" : "Stationary shot");
  }
}
