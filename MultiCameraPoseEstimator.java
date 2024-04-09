package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;

public class MultiCameraPoseEstimator implements PoseEstimator {

  List<PoseEstimator> poseEstimatorsList;

  int numberOfTargetsUsed = 0;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  public MultiCameraPoseEstimator(List<PoseEstimator> poseEstimators) {
    this.poseEstimatorsList = poseEstimators;
    this.detectedPoseLogger = new CustomPose2dLogger("/Vision/MultiCameraEstimator/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger("/Vision/MultiCameraEstimator/NumberOfDetectedTags");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    ArrayList<Pose2d> pose2Ds = new ArrayList<>();
    ArrayList<PoseEstimation> poseEstimations = new ArrayList<>();
    for (PoseEstimator poseEstimator : this.poseEstimatorsList) {
      Optional<PoseEstimation> possiblePoseEstimation = poseEstimator.getEstimatedPose(referencePose);
      if (possiblePoseEstimation.isPresent()) {
        PoseEstimation poseEstimation = possiblePoseEstimation.get();
        for (int i = 0; i < poseEstimation.numberOfTargetsUsed; i++) {
          pose2Ds.add(poseEstimation.estimatedPose.toPose2d());
          poseEstimations.add(poseEstimation);
        }
      }
    }
    int numberOfTargetsUsed = poseEstimations.size();
    if (numberOfTargetsUsed < 2) {
      return Optional.empty();
    }
    pose2Ds.sort(new ComparatorPose2D());
    poseEstimations.sort(new ComparatorPoseEstimation());
    PoseEstimation poseEstimation = frc.Java_Is_UnderControl.Util.Util.medianPoseEstimationsList(poseEstimations);
    poseEstimation.numberOfTargetsUsed = numberOfTargetsUsed;
    detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
    return Optional.of(poseEstimation);
  }

  @Override
  public void setSwerveDriveForGyroReference(OdometryEnabledSwerveSubsystem swerve) {

  }

}
