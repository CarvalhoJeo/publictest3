package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Swerve.OdometryEnabledSwerveSubsystem;

public class NoPoseEstimator implements PoseEstimator {

  @Override
  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    return Optional.empty();
  }

  @Override
  public void setSwerveDriveForGyroReference(OdometryEnabledSwerveSubsystem swerve) {

  }

}
