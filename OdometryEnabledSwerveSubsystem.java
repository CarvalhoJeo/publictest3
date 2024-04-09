package frc.Java_Is_UnderControl.Swerve;

import java.io.File;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimation;
import frc.Java_Is_UnderControl.Vision.Odometry.PoseEstimator;
import frc.robot.Robot;

public abstract class OdometryEnabledSwerveSubsystem extends BaseSwerveSubsystem {

  public static double robotOrientation;

  public static double robotAngularVelocity;

  private PoseEstimator autonomousPoseEstimator;

  private PoseEstimator teleopPoseEstimator;

  private PIDController moveToPoseXAxisPid;

  private PIDController moveToPoseYAxisPid;

  private Pose2d targetPose;

  private Pose2d targetAimPose;

  private Pose2d poseVision;

  private CustomPose2dLogger targetPoseLogger = new CustomPose2dLogger("/SwerveSubsystem/TargetPose");

  private CustomPose2dLogger targetAimPositionLogger = new CustomPose2dLogger("/SwerveSubsystem/TargetAimPosition");

  private CustomBooleanLogger isAtTargetXAxisLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetXAxis");

  private CustomBooleanLogger isAtTargetYAxisLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetYAxis");

  private CustomBooleanLogger isAtTargetPoseLogger = new CustomBooleanLogger("/SwerveSubsystem/IsAtTargetPose");

  private CustomPose2dLogger poseVisionLogger = new CustomPose2dLogger("/SwerveSubsystem/PoseVision");

  public OdometryEnabledSwerveSubsystem(File directory, OdometryEnabledSwerveConfig config) {
    super(directory, config);
    this.moveToPoseXAxisPid = new PIDController(config.moveToPosePidConfig.kP, config.moveToPosePidConfig.kI,
        config.moveToPosePidConfig.kD);
    this.moveToPoseYAxisPid = new PIDController(config.moveToPosePidConfig.kP, config.moveToPosePidConfig.kI,
        config.moveToPosePidConfig.kD);
    this.autonomousPoseEstimator = config.autonomousPoseEstimator;
    this.teleopPoseEstimator = config.teleoperatedPoseEstimator;
    this.targetPose = new Pose2d();
    this.targetAimPose = new Pose2d();
    this.poseVision = new Pose2d();
  }

  private void updateOdometry() {
    if (Robot.isInAutonomous) {
      this.updateOdometryWithPoseEstimator(this.autonomousPoseEstimator);
    } else {
      this.updateOdometryWithPoseEstimator(this.teleopPoseEstimator);
    }
  }

  private void updateOdometryWithPoseEstimator(PoseEstimator poseEstimator) {
    Pose2d referencePose = this.swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition();
    Optional<PoseEstimation> possibleEstimatedPose = poseEstimator.getEstimatedPose(referencePose);
    if (possibleEstimatedPose.isPresent()) {
      PoseEstimation estimatedPose = possibleEstimatedPose.get();
      this.poseVision = estimatedPose.estimatedPose.toPose2d();
      this.swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.8, .8, 99999));
      this.swerveDrive.addVisionMeasurement(this.poseVision, estimatedPose.timestampSeconds);
    }
  }

  protected double getDistanceToPosition(Translation2d targetPosition) {
    return this.getPose().getTranslation().getDistance(targetPosition);
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(targetSpeeds, targetPosition, Rotation2d.fromDegrees(0));
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition,
      Rotation2d offsetAngle) {
    this.targetAimPose = new Pose2d(targetPosition, new Rotation2d());
    Translation2d positionDifference = this.getPose().getTranslation().minus(targetPosition);
    Rotation2d targetAngle = positionDifference.getAngle().plus(offsetAngle);
    this.driveFieldOrientedLockedAngle(targetSpeeds, targetAngle);
  }

  protected void driveAimingAtPositionMoving(double dt, ChassisSpeeds targetSpeeds, Translation2d targetPosition,
      Rotation2d offsetAngle) {
    this.targetAimPose = new Pose2d(targetPosition, new Rotation2d());
    Translation2d positionDifference = this.getEarlyPoseMoving(dt).getTranslation().minus(targetPosition);
    Rotation2d targetAngle = positionDifference.getAngle().plus(offsetAngle);
    this.driveFieldOrientedLockedAngle(targetSpeeds, targetAngle);
  }

  protected void driveToPose(Pose2d targetPose) {
    Pose2d currentPose = this.getPose();
    double targetXVelocity = this.moveToPoseXAxisPid.calculate(currentPose.getX(), targetPose.getX());
    double targetYVelocity = this.moveToPoseYAxisPid.calculate(currentPose.getY(), targetPose.getY());
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(targetXVelocity, targetYVelocity, 0);
    this.driveFieldOrientedLockedAngle(desiredSpeeds, targetPose.getRotation());
    this.targetPose = targetPose;
  }

  protected boolean isAtTargetPose(double xAxisToleranceMeters, double yAxisToleranceMeters,
      double angleToleranceDegrees) {
    double xAxisDistance = Math.abs(this.getPose().getX() - this.targetPose.getX());
    double yAxisDistance = Math.abs(this.getPose().getY() - this.targetPose.getY());
    boolean isAtXAxis = xAxisDistance <= xAxisToleranceMeters;
    boolean isAtYAxis = yAxisDistance <= yAxisToleranceMeters;
    boolean isAtAngle = this.isAtTargetHeading(angleToleranceDegrees);
    boolean isAtTargetPose = isAtXAxis && isAtYAxis && isAtAngle;
    this.isAtTargetXAxisLogger.append(isAtXAxis);
    this.isAtTargetYAxisLogger.append(isAtYAxis);
    this.isAtTargetPoseLogger.append(isAtTargetPose);
    return isAtTargetPose;
  }

  @Override
  protected void driveFieldOriented(ChassisSpeeds targetSpeeds) {
    this.targetPose = new Pose2d();
    super.driveFieldOriented(targetSpeeds);
  }

  @Override
  protected void driveRobotOriented(ChassisSpeeds targetSpeeds) {
    this.targetPose = new Pose2d();
    super.driveRobotOriented(targetSpeeds);
  }

  @Override
  public void periodic() {
    this.updateOdometry();
    super.periodic();
    this.updateOdometrySwerveLogs();
    this.updateLimelightRequiredValues();
  }

  private void updateLimelightRequiredValues() {
    robotAngularVelocity = this.swerveDrive.getRobotVelocity().omegaRadiansPerSecond;
    robotOrientation = this.swerveDrive.getOdometryHeading().getDegrees();
  }

  private void updateOdometrySwerveLogs() {
    this.targetAimPositionLogger.appendRadians(targetAimPose);
    this.poseVisionLogger.appendRadians(this.poseVision);
    this.targetPoseLogger.appendRadians(this.targetPose);
  }

}
