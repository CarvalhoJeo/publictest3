package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers.PoseEstimate;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;

public class LimelightPoseEstimator implements PoseEstimator {

  String limelightName;

  boolean only2TagsMeasurements = false;

  OdometryEnabledSwerveSubsystem swerveDrive = null;

  boolean useMegaTag1;

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  public LimelightPoseEstimator(String limelightName, boolean only2TagsMeasurements, boolean useMegaTag1) {
    this.limelightName = limelightName;
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.useMegaTag1 = useMegaTag1;
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/IsDetectingTags");
    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/NumberOfDetectedTags");
  }

  public LimelightPoseEstimator(String limelightName, boolean only2TagsMeasurements) {
    this(limelightName, only2TagsMeasurements, false);
  }

  public LimelightPoseEstimator(String limelightName) {
    this(limelightName, false);
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    if (!LimelightHelpers.getTV(this.limelightName)) {
      this.isDetectingLogger.append(false);
      this.numberOfDetectedTagsLogger.append(0);
      return Optional.empty();
    }
    PoseEstimate limelightPoseEstimate;
    if (useMegaTag1 || swerveDrive == null) {
      limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);
    } else {
      LimelightHelpers.SetRobotOrientation(this.limelightName, swerveDrive.getMegaTag2Values()[0], 0, 0, 0,
          0, 0);
      limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.limelightName);
      if (swerveDrive.getMegaTag2Values()[1] > 1 || limelightPoseEstimate.tagCount == 0) {
        return Optional.empty();
      }
    }
    PoseEstimation poseEstimation = convertPoseEstimate(limelightPoseEstimate);
    this.isDetectingLogger.append(true);
    this.detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    this.numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
    if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
      return Optional.empty();
    }
    return Optional.of(poseEstimation);
  }

  public void setSwerveDriveForGyroReference(OdometryEnabledSwerveSubsystem swerve) {
    swerveDrive = swerve;
  }

  private PoseEstimation convertPoseEstimate(PoseEstimate limelightPoseEstimate) {
    return new PoseEstimation(new Pose3d(limelightPoseEstimate.pose), limelightPoseEstimate.timestampSeconds,
        limelightPoseEstimate.tagCount);
  }

}
