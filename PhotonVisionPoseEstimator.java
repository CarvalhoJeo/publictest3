package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;

public class PhotonVisionPoseEstimator implements PoseEstimator {

  PhotonPoseEstimator photonPoseEstimator;

  boolean only2TagsMeasurements = false;

  private AprilTagFieldLayout fieldAprilTagLayout;

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition) {
    this.fieldAprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    this.fieldAprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(this.fieldAprilTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraPosition);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/NumberOfDetectedTags");
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/IsDetecting");
  }

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition, boolean only2TagsMeasurements) {
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.fieldAprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    this.fieldAprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(this.fieldAprilTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraPosition);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/NumberOfDetectedTags");
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/PhotonVisionPoseEstimator/" + camera.getName() + "/IsDetecting");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    this.photonPoseEstimator.setReferencePose(referencePose);
    Optional<EstimatedRobotPose> photonPoseEstimation = this.photonPoseEstimator.update();
    if (!photonPoseEstimation.isPresent()) {
      isDetectingLogger.append(false);
      return Optional.empty();
    }
    isDetectingLogger.append(true);
    PoseEstimation poseEstimation = convertPhotonPoseEstimation(photonPoseEstimation.get());
    if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
      return Optional.empty();
    }
    numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
    detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    return Optional.of(poseEstimation);
  }

  @Override
  public void setSwerveDriveForGyroReference(OdometryEnabledSwerveSubsystem swerve) {

  }

  private PoseEstimation convertPhotonPoseEstimation(EstimatedRobotPose photonPoseEstimation) {
    return new PoseEstimation(photonPoseEstimation.estimatedPose, photonPoseEstimation.timestampSeconds,
        photonPoseEstimation.targetsUsed.size());
  }

}
