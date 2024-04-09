package frc.Java_Is_UnderControl.Swerve;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomChassisSpeedsLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Util.CustomMath;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public abstract class BaseSwerveSubsystem extends SubsystemBase {

  protected SwerveDrive swerveDrive;

  protected double maxTranslationSpeed;

  protected double maxRotationSpeed;

  private Alliance defaultAlliance;

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  private double targetHeadingDegrees = Double.NaN;

  private CustomChassisSpeedsLogger targetSpeedsLogger = new CustomChassisSpeedsLogger("/SwerveSubsystem/TargetSpeeds");

  private CustomDoubleLogger absoluteTargetSpeedLogger = new CustomDoubleLogger("/SwerveSubsystem/AbsoluteTargetSpeed");

  private CustomChassisSpeedsLogger measuredSpeedsLogger = new CustomChassisSpeedsLogger(
      "/SwerveSubsystem/MeasuredSpeeds");

  private CustomDoubleLogger absoluteMeasuredSpeedLogger = new CustomDoubleLogger(
      "/SwerveSubsystem/AbsoluteMeasuredSpeed");

  private CustomPose2dLogger poseLogger = new CustomPose2dLogger("/SwerveSubsystem/Pose");

  private CustomDoubleLogger targetHeadingLogger = new CustomDoubleLogger("/SwerveSubsystem/TargetHeadingDegrees");

  private CustomDoubleLogger measuredHeadingLogger = new CustomDoubleLogger("/SwerveSubsystem/MeasuredHeadingDegrees");

  private CustomBooleanLogger isAtTargetHeading = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetHeading");

  protected BaseSwerveSubsystem(File directory, BaseSwerveConfig config) {
    this.maxTranslationSpeed = config.maxTranslationSpeed;
    this.maxRotationSpeed = config.maxRotationSpeed;
    this.defaultAlliance = config.defaultAlliance;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    this.createSwerveDriveFromYagsl(directory);
    this.setupPathPlanner(config.pathPlannerConfig);
  }

  protected abstract void updateLogs();

  private void createSwerveDriveFromYagsl(File directory) {
    try {
      this.swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(
              SwerveConstants.MAX_VEL,
              SwerveConstants.SteeringConversionFactor,
              SwerveConstants.DriveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  private void setupPathPlanner(SwervePathPlannerConfig config) {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            config.translationPid,
            config.anglePid,
            config.maxModuleSpeed,
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this);
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    this.swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public void zeroGyro() {
    this.swerveDrive.zeroGyro();
  }

  public void setHeadingCorrection(boolean active) {
    this.swerveDrive.setHeadingCorrection(active);
  }

  public void setMotorBrake(boolean brake) {
    this.swerveDrive.setMotorIdleMode(brake);
  }

  protected Pose2d getPose() {
    return this.swerveDrive.getPose();
  }

  protected Pose2d getEarlyPoseMoving(double dt) {
    Pose2d actualPose = getPose();
    Translation2d earlyTranslation = new Translation2d(actualPose.getX() + getRobotVelocity().vxMetersPerSecond * dt,
        actualPose.getY() + getRobotVelocity().vyMetersPerSecond * dt);
    Pose2d earlyPoseMoving = new Pose2d(earlyTranslation, actualPose.getRotation());
    return earlyPoseMoving;
  }

  protected Pose2d getEarlyPoseMovingWithOmega(double dt) {
    Pose2d actualPose = getPose();
    Translation2d earlyTranslation = new Translation2d(actualPose.getX() + getRobotVelocity().vxMetersPerSecond * dt,
        actualPose.getY() + getRobotVelocity().vyMetersPerSecond * dt);
    Rotation2d earlyRotation = new Rotation2d(
        actualPose.getRotation().getRadians() + getRobotVelocity().omegaRadiansPerSecond * dt);
    Pose2d earlyPoseMoving = new Pose2d(earlyTranslation, earlyRotation);
    return earlyPoseMoving;
  }

  protected void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.swerveDrive.setChassisSpeeds(chassisSpeeds);
    this.targetSpeeds = chassisSpeeds;
  }

  protected ChassisSpeeds getRobotVelocity() {
    return this.swerveDrive.getRobotVelocity();
  }

  protected double getAbsoluteRobotVelocity() {
    return CustomMath.toAbsoluteSpeed(this.getRobotVelocity());
  }

  protected Rotation2d getHeading() {
    return this.swerveDrive.getYaw();
  }

  public void lock() {
    this.swerveDrive.lockPose();
  }

  @Override
  public void periodic() {
    this.updateBaseLogs();
    this.updateLogs();
  }

  private void updateBaseLogs() {
    this.poseLogger.appendRadians(this.getPose());
    this.targetSpeedsLogger.append(this.targetSpeeds);
    this.absoluteTargetSpeedLogger.append(CustomMath.toAbsoluteSpeed(this.targetSpeeds));
    this.measuredSpeedsLogger.append(this.getRobotVelocity());
    this.absoluteMeasuredSpeedLogger.append(this.getAbsoluteRobotVelocity());
    this.targetHeadingLogger.append(this.targetHeadingDegrees);
    this.measuredHeadingLogger.append(this.getHeading().getDegrees());
  }

  protected void driveFieldOrientedLockedAngle(ChassisSpeeds speeds, Rotation2d targetHeading) {
    ChassisSpeeds speedsWithAngleAlignment = this.swerveDrive.swerveController.getRawTargetSpeeds(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        targetHeading.getRadians(),
        this.getHeading().getRadians());
    if (speedsWithAngleAlignment.omegaRadiansPerSecond > this.swerveDrive.getMaximumAngularVelocity()) {
      speedsWithAngleAlignment = new ChassisSpeeds(
          speedsWithAngleAlignment.vxMetersPerSecond,
          speedsWithAngleAlignment.vyMetersPerSecond,
          this.swerveDrive.getMaximumAngularVelocity());
    } else if (speedsWithAngleAlignment.omegaRadiansPerSecond < (-1 * this.swerveDrive.getMaximumAngularVelocity())) {
      speedsWithAngleAlignment = new ChassisSpeeds(
          speedsWithAngleAlignment.vxMetersPerSecond,
          speedsWithAngleAlignment.vyMetersPerSecond,
          -1 * this.swerveDrive.getMaximumAngularVelocity());
    }
    this.driveFieldOriented(speedsWithAngleAlignment);
    this.targetHeadingDegrees = targetHeading.getDegrees();
  }

  protected void driveFieldOriented(ChassisSpeeds speeds) {
    ChassisSpeeds correctedSpeeds = this.performAllianceSpeedDirectionCorrection(speeds);
    this.swerveDrive.driveFieldOriented(correctedSpeeds);
    this.targetSpeeds = correctedSpeeds;
    this.targetHeadingDegrees = Double.NaN;
  }

  protected void driveRobotOrientedLockedAngle(ChassisSpeeds speeds, Rotation2d targetHeading) {
    ChassisSpeeds speedsWithAngleAlignment = this.swerveDrive.swerveController.getRawTargetSpeeds(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        targetHeading.getRadians(),
        this.getHeading().getRadians());
    if (speedsWithAngleAlignment.omegaRadiansPerSecond > this.swerveDrive.getMaximumAngularVelocity()) {
      speedsWithAngleAlignment = new ChassisSpeeds(
          speedsWithAngleAlignment.vxMetersPerSecond,
          speedsWithAngleAlignment.vyMetersPerSecond,
          this.swerveDrive.getMaximumAngularVelocity());
    } else if (speedsWithAngleAlignment.omegaRadiansPerSecond < (-1 * this.swerveDrive.getMaximumAngularVelocity())) {
      speedsWithAngleAlignment = new ChassisSpeeds(
          speedsWithAngleAlignment.vxMetersPerSecond,
          speedsWithAngleAlignment.vyMetersPerSecond,
          -1 * this.swerveDrive.getMaximumAngularVelocity());
    }
    this.driveRobotOriented(speedsWithAngleAlignment);
    this.targetHeadingDegrees = targetHeading.getDegrees();
  }

  protected void driveRobotOriented(ChassisSpeeds speeds) {
    this.swerveDrive.drive(speeds);
    this.targetSpeeds = speeds;
    this.targetHeadingDegrees = Double.NaN;
  }

  private ChassisSpeeds performAllianceSpeedDirectionCorrection(ChassisSpeeds targetSpeeds) {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
        : this.defaultAlliance;
    if (alliance == Alliance.Red) {
      targetSpeeds = new ChassisSpeeds(-targetSpeeds.vxMetersPerSecond, -targetSpeeds.vyMetersPerSecond,
          targetSpeeds.omegaRadiansPerSecond);
    }
    return targetSpeeds;
  }

  protected ChassisSpeeds inputsToChassisSpeeds(double xInput, double yInput) {
    return new ChassisSpeeds(xInput * this.maxTranslationSpeed, yInput * this.maxTranslationSpeed, 0);
  }

  protected boolean isAtTargetHeading(double toleranceDegrees) {
    if (this.targetHeadingDegrees == Double.NaN) {
      this.isAtTargetHeading.append(false);
      return false;
    }
    Rotation2d targetHeading = Rotation2d.fromDegrees(this.targetHeadingDegrees);
    Rotation2d currentHeading = this.getHeading();
    double headingDifferenceDegrees = Math.abs(targetHeading.minus(currentHeading).getDegrees());
    boolean isAtHeading = headingDifferenceDegrees <= toleranceDegrees;
    this.isAtTargetHeading.append(isAtHeading);
    return isAtHeading;
  }
}
